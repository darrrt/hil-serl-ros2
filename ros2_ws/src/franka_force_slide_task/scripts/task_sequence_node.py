#!/usr/bin/env python3
"""
视触觉传感器数据搜集任务节点

本节点用于在 Franka 机械臂上执行 EDM 纹理板滑动数据搜集任务。
核心设计理念：使用"基于主通道前馈的恒力阻抗控制"方法，确保搜集数据时法向力恒定。

控制律设计：
τ_cmd = J^T · [K_x(x_des-x) + D_x(ẋ_des-ẋ); K_y(y_des-y) + D_y(ẏ_des-ẏ); F_target - D_z · ż; 姿态控制力矩] + τ_bias

关键参数配置：
- X, Y轴：高刚度控制（如 2000 N/m），按规划速度滑动（5~10 mm/s）
- Z轴：刚度 K_z = 0，阻尼 D_z = 10~20 Ns/m，前馈注入目标法向力 F_target（如 5N）

优势：
1. 力保持真正恒定：无论 EDM 板如何倾斜，Z轴自动跟随保持恒定压力
2. 数据解耦：视触觉图像法向变形一致，变化仅由纹理和剪切力引起
"""

import time
import yaml
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, WrenchStamped
from franka_msgs.action import Grasp, Homing, Move
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import SwitchController
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState


class TaskSequenceNode(Node):

    def __init__(self):
        super().__init__('task_sequence_node')

        # ============ 参数声明 ============
        # 目标关节位置（初始位姿）
        self.declare_parameter('target_joint_positions', [0.0, 0.168, 0.557, -2.193, -1.1723, 1.186, 0.110])
        # 关节运动等待时间（秒）
        self.declare_parameter('motion_duration', 10.0)
        # 夹爪抓取力（牛顿）
        self.declare_parameter('gripper_force', 1.0)
        # 夹爪宽度（米）
        self.declare_parameter('gripper_width', 0.01)
        # 夹爪运动速度（米/秒）
        self.declare_parameter('gripper_speed', 0.1)
        # 滑动起点X坐标（米）
        self.declare_parameter('slide_start_x', 0.4)
        # 滑动起点Y坐标（米）
        self.declare_parameter('slide_start_y', 0.0)
        # 滑动终点X坐标（米）
        self.declare_parameter('slide_end_x', 0.6)
        # 滑动终点Y坐标（米）
        self.declare_parameter('slide_end_y', 0.2)
        # 滑动速度（米/秒，推荐5~10 mm/s）
        self.declare_parameter('slide_speed', 0.02)
        # 滑动高度（Z轴，米）
        self.declare_parameter('slide_z_height', 0.15)
        # 末端姿态（四元数 [x, y, z, w]）
        self.declare_parameter('slide_orientation', [0.0, 1.0, 0.0, 0.0])
        # 是否自动启动任务序列
        self.declare_parameter('auto_start', True)
        # 夹爪命名空间
        self.declare_parameter('gripper_namespace', 'franka_gripper')

        # 可重入回调组，支持多个服务/动作并发处理
        self.cb_group = ReentrantCallbackGroup()

        # ============ 动作客户端初始化 ============
        self.gripper_ns = self.get_parameter('gripper_namespace').value

        # 抓取动作客户端
        self.grasp_client = ActionClient(
            self, Grasp, f'/{self.gripper_ns}/grasp',
            callback_group=self.cb_group)
        # 归位动作客户端
        self.homing_client = ActionClient(
            self, Homing, f'/{self.gripper_ns}/homing',
            callback_group=self.cb_group)
        # 移动动作客户端
        self.move_client = ActionClient(
            self, Move, f'/{self.gripper_ns}/move',
            callback_group=self.cb_group)

        # ============ 服务客户端初始化 ============
        # 控制器切换服务
        self.switch_controller_client = self.create_client(
            SwitchController, '/controller_manager/switch_controller',
            callback_group=self.cb_group)

        # ============ 发布器初始化 ============
        self.eq_pose_pub = self.create_publisher(
            PoseStamped, '/cartesian_impedance_force_controller/equilibrium_pose', 10)

        # ============ 控制器健康监控 ============
        self.controller_healthy = False
        self.last_force_state_time = 0.0
        self.force_state_sub = self.create_subscription(
            WrenchStamped,
            '/cartesian_impedance_force_controller/force_state',
            self._force_state_callback,
            10,
            callback_group=self.cb_group)

        # ============ 服务端初始化 ============
        # 任务触发服务（通过服务调用启动任务序列）
        self.trigger_service = self.create_service(
            Trigger, '~/start', self.trigger_callback,
            callback_group=self.cb_group)

        # ============ 夹爪状态订阅器 ============
        # 订阅夹爪关节状态，用于实时监控夹爪状态
        self.gripper_joint_state_sub = self.create_subscription(
            JointState,
            f'/{self.gripper_ns}/joint_states',
            self.gripper_state_callback,
            10,
            callback_group=self.cb_group)
        
        # 夹爪状态变量
        self.current_gripper_width = 0.0
        self.current_gripper_force = 0.0
        self.gripper_state_received = False

        self.get_logger().info('任务序列节点已初始化. 等待服务连接...')

        # 自动启动逻辑
        if self.get_parameter('auto_start').value:
            self.get_logger().info('自动启动已启用. 3秒后开始任务序列...')
            self.auto_start_timer = self.create_timer(3.0, self._auto_start_callback)

    def _wait_for_future(self, future, timeout_sec=10.0):
        """
        轮询等待 future 完成（替代 rclpy.spin_until_future_complete）
        
        问题背景：execute_task_sequence 从 timer 回调中调用，
        而 main() 中的 MultiThreadedExecutor 已经在 spin 这个 node。
        如果在回调中再调用 rclpy.spin_until_future_complete(node, future)，
        它会创建一个新的 SingleThreadedExecutor 并尝试 spin 同一个 node，
        导致两个 executor 争抢同一个 node → 死锁。
        
        解决方案：用轮询 + time.sleep 替代。
        MultiThreadedExecutor 的其他线程会处理服务/动作的响应回调，
        从而将 future 标记为 done，本方法只需等待即可。
        
        Args:
            future: rclpy Future 对象
            timeout_sec: 超时时间（秒）
        
        Returns:
            bool: future 在超时前完成返回 True，超时返回 False
        """
        start_time = time.time()
        while not future.done():
            if time.time() - start_time > timeout_sec:
                return False
            time.sleep(0.02)
        return True

    def _force_state_callback(self, msg):
        self.controller_healthy = True
        self.last_force_state_time = time.time()

    def _check_controller_alive(self, timeout_sec=2.0):
        if not self.controller_healthy:
            self.get_logger().warn('  控制器尚未发布任何力状态数据，等待中...')
            start = time.time()
            while not self.controller_healthy and time.time() - start < timeout_sec:
                time.sleep(0.1)
            if not self.controller_healthy:
                self.get_logger().error('  控制器在等待期内未发布力状态数据! 可能已崩溃。')
                return False
        if time.time() - self.last_force_state_time > timeout_sec:
            self.get_logger().error(
                f'  控制器力状态数据已中断 {time.time() - self.last_force_state_time:.1f}s! '
                f'控制器可能已崩溃。')
            return False
        return True

    def gripper_state_callback(self, msg):
        """
        夹爪关节状态回调函数
        
        Franka夹爪的joint_states包含：
        - position: [finger_joint1_position, finger_joint2_position] 每个手指的位置
        - velocity: [finger_joint1_velocity, finger_joint2_velocity] 每个手指的速度
        - effort: [finger_joint1_effort, finger_joint2_effort] 每个手指的力（单位：N）
        
        夹爪总宽度 = finger_joint1_position + finger_joint2_position
        夹爪总力 = finger_joint1_effort + finger_joint2_effort
        """
        if len(msg.position) >= 2:
            # 夹爪宽度 = 两个手指位置之和
            self.current_gripper_width = msg.position[0] + msg.position[1]
            self.gripper_state_received = True
        
        if len(msg.effort) >= 2:
            # 夹爪力 = 两个手指力之和（绝对值，因为可能为负）
            self.current_gripper_force = abs(msg.effort[0]) + abs(msg.effort[1])

    def _auto_start_callback(self):
        """自动启动回调函数"""
        self.auto_start_timer.destroy()
        self.execute_task_sequence()

    def trigger_callback(self, request, response):
        """服务触发回调函数"""
        self.get_logger().info('任务序列通过服务触发.')
        self.execute_task_sequence()
        response.success = True
        response.message = '任务序列已完成'
        return response

    def execute_task_sequence(self):
        """
        执行完整的任务序列（主流程）
        
        任务流程：
        1. 等待关节位置控制器将机械臂移动到目标位姿（接触EDM板前的初始位置）
        2. 夹爪抓取视触觉传感器（凝胶传感器）
        3. 切换到笛卡尔阻抗力控制器（启用恒力控制模式）
        4. 在EDM纹理板上执行XY滑动（Z轴保持恒力）
        """
        self.get_logger().info('=== 任务序列开始 ===')

        # 步骤1: 等待关节位置控制器将机械臂移动到目标位姿
        self.get_logger().info('步骤1: 等待关节位置控制器移动机械臂到目标位姿...')
        motion_duration = self.get_parameter('motion_duration').value
        self.get_logger().info(f'  等待 {motion_duration} 秒完成运动...')
        time.sleep(motion_duration + 2.0)
        self.get_logger().info('  关节位置运动完成.')

        # 步骤2: 夹爪抓取
        self.get_logger().info('步骤2: 夹爪抓取传感器...')
        if not self.grasp_tool():
            self.get_logger().error('  夹爪抓取失败! 终止任务.')
            return
        self.get_logger().info('  夹爪抓取成功.')
        time.sleep(1.0)

        # 步骤3: 切换到笛卡尔阻抗力控制器
        # 此步骤是实现恒力控制的关键：切换到支持力控制的控制器
        self.get_logger().info('步骤3: 切换到笛卡尔阻抗力控制器...')
        self.get_logger().info('  停用: joint_position_controller, 激活: cartesian_impedance_force_controller')
        switch_ok = self.switch_controller(
                deactivate=['joint_position_controller'],
                activate=['cartesian_impedance_force_controller'])
        if not switch_ok:
            self.get_logger().error('步骤3: 控制器切换失败! 终止任务.')
            self.get_logger().error('  请检查: 1)控制器名称是否正确 2)控制器是否已加载 3)硬件接口是否冲突')
            return
        self.get_logger().info('步骤3: 控制器切换成功. 等待1秒让控制器稳定...')
        time.sleep(1.0)

        if not self._check_controller_alive(timeout_sec=3.0):
            self.get_logger().error('步骤3: 控制器切换后健康检查失败! 可能已崩溃，终止任务。')
            return

        # 步骤3.5: 预发布滑动起始位姿作为初始平衡点
        # 控制器 on_activate 时会将 position_d_ 设为当前末端位姿，
        # 此处先发布滑动起始位姿，让控制器通过内部滤波器平滑过渡到目标位置，
        # 避免直接开始滑动时出现力矩突变
        start_x = self.get_parameter('slide_start_x').value
        start_y = self.get_parameter('slide_start_y').value
        z_height = self.get_parameter('slide_z_height').value
        orientation = self.get_parameter('slide_orientation').value
        self.get_logger().info('步骤3.5: 预发布初始平衡位姿，让控制器平滑过渡...')
        pre_pose = PoseStamped()
        pre_pose.header.frame_id = 'fr3_link0'
        pre_pose.header.stamp = self.get_clock().now().to_msg()
        pre_pose.pose.position.x = start_x
        pre_pose.pose.position.y = start_y
        pre_pose.pose.position.z = z_height
        pre_pose.pose.orientation.x = orientation[0]
        pre_pose.pose.orientation.y = orientation[1]
        pre_pose.pose.orientation.z = orientation[2]
        pre_pose.pose.orientation.w = orientation[3]
        for _ in range(200):
            pre_pose.header.stamp = self.get_clock().now().to_msg()
            self.eq_pose_pub.publish(pre_pose)
            time.sleep(0.01)
        self.get_logger().info('  初始平衡位姿已发布（2秒 @ 100Hz），控制器应已平滑过渡.')

        # 步骤4: 执行XY平面滑动（Z轴恒力控制）
        # 滑动过程中，阻抗控制器将保持Z轴恒定压力
        self.get_logger().info('步骤4: 执行XY滑动（Z轴恒力控制）...')
        self.execute_slide()
        self.get_logger().info('=== 任务序列完成 ===')

    def grasp_tool(self):
        gripper_width = self.get_parameter('gripper_width').value
        epsilon_inner = 0.001
        min_width = gripper_width - epsilon_inner
        
        if min_width <= 0.0:
            self.get_logger().error(f'  错误: 最小宽度 {min_width:.4f}m <= 0，请增大 gripper_width 或减小 epsilon_inner')
            return False
        
        self.get_logger().info(f'  当前夹爪状态: 宽度={self.current_gripper_width:.4f}m, 力={self.current_gripper_force:.2f}N')
        
        max_retries = 3
        for attempt in range(1, max_retries + 1):
            self.get_logger().info(f'  抓取尝试 ({attempt}/{max_retries})...')

            if not self.move_gripper(0.08):
                self.get_logger().error('  张开夹爪失败')
                self.get_logger().info(f'  当前夹爪状态: 宽度={self.current_gripper_width:.4f}m, 力={self.current_gripper_force:.2f}N')
                continue

            self.get_logger().info(f'  夹爪张开完成，当前宽度: {self.current_gripper_width:.4f}m')

            if not self.grasp_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('  抓取动作服务器不可用')
                continue

            goal = Grasp.Goal()
            goal.width = gripper_width
            goal.speed = self.get_parameter('gripper_speed').value
            goal.force = self.get_parameter('gripper_force').value
            goal.epsilon.inner = epsilon_inner
            goal.epsilon.outer = 0.005

            self.get_logger().info(f'  执行抓取: width={goal.width}m, min_width={min_width:.4f}m, force={goal.force}N')

            future = self.grasp_client.send_goal_async(goal)
            if not self._wait_for_future(future, timeout_sec=10.0):
                self.get_logger().error(f'  第{attempt}次抓取: 请求超时')
                self.get_logger().info(f'  当前夹爪状态: 宽度={self.current_gripper_width:.4f}m, 力={self.current_gripper_force:.2f}N')
                continue

            if future.result() is None:
                self.get_logger().error(f'  第{attempt}次抓取: 请求超时')
                self.get_logger().info(f'  当前夹爪状态: 宽度={self.current_gripper_width:.4f}m, 力={self.current_gripper_force:.2f}N')
                continue
            
            if not future.result().accepted:
                self.get_logger().warn(f'  第{attempt}次抓取: 目标被拒绝')
                self.get_logger().info(f'  当前夹爪状态: 宽度={self.current_gripper_width:.4f}m, 力={self.current_gripper_force:.2f}N')
                continue

            self.get_logger().info('  抓取目标已接受，等待执行结果...')
            
            result_future = future.result().get_result_async()
            if not self._wait_for_future(result_future, timeout_sec=15.0):
                self.get_logger().error(f'  第{attempt}次抓取: 结果超时')
                self.get_logger().info(f'  当前夹爪状态: 宽度={self.current_gripper_width:.4f}m, 力={self.current_gripper_force:.2f}N')
                continue

            if result_future.result() is None:
                self.get_logger().error(f'  第{attempt}次抓取: 结果超时')
                self.get_logger().info(f'  当前夹爪状态: 宽度={self.current_gripper_width:.4f}m, 力={self.current_gripper_force:.2f}N')
                continue

            result = result_future.result().result
            
            if result.success:
                self.get_logger().info(f'  第{attempt}次抓取成功!')
                self.get_logger().info(f'  抓取后夹爪状态: 宽度={self.current_gripper_width:.4f}m, 力={self.current_gripper_force:.2f}N')
                return True
            else:
                error_msg = result.error if result.error else "未知错误"
                self.get_logger().error(f'  第{attempt}次抓取失败! 错误信息: {error_msg}')
                self.get_logger().info(f'  当前夹爪状态: 宽度={self.current_gripper_width:.4f}m, 力={self.current_gripper_force:.2f}N')
                self.analyze_grasp_failure(attempt, gripper_width, min_width, goal.force)

        self.get_logger().error(f'抓取失败: 已重试{max_retries}次，均未成功')
        return False

    def analyze_grasp_failure(self, attempt, target_width, min_width, force):
        """
        分析抓取失败原因并给出建议
        
        Franka夹爪常见抓取失败原因：
        1. 目标宽度设置不当
        2. 抓取力不足或过大
        3. epsilon参数设置不当
        4. 物体尺寸与目标宽度不匹配
        5. 夹爪未正确张开
        """
        self.get_logger().info(f'  --- 抓取失败分析 (第{attempt}次尝试) ---')
        self.get_logger().info(f'  目标宽度: {target_width:.4f}m, 最小宽度: {min_width:.4f}m')
        self.get_logger().info(f'  当前宽度: {self.current_gripper_width:.4f}m, 当前力: {self.current_gripper_force:.2f}N')
        self.get_logger().info(f'  设定抓取力: {force}N')
        
        if self.current_gripper_width > 0.075:
            self.get_logger().warn(f'  警告: 夹爪当前宽度({self.current_gripper_width:.4f}m)接近最大宽度(0.08m)')
            self.get_logger().warn(f'        可能是物体尺寸超过夹爪最大张开范围')
        
        if self.current_gripper_width < min_width:
            self.get_logger().warn(f'  警告: 当前宽度({self.current_gripper_width:.4f}m)小于最小宽度({min_width:.4f}m)')
            self.get_logger().warn(f'        可能是物体尺寸过小或目标宽度设置过大')
        
        if force < 2.0:
            self.get_logger().warn(f'  警告: 抓取力({force}N)过小，可能无法牢固抓取')
        elif force > 20.0:
            self.get_logger().warn(f'  警告: 抓取力({force}N)过大，可能损坏物体')

    def move_gripper(self, width):
        """
        移动夹爪到指定宽度
        
        Args:
            width (float): 目标宽度（米）
            
        Returns:
            bool: 移动成功返回True，失败返回False
        """
        if not self.move_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('夹爪移动动作服务器不可用!')
            return False

        goal = Move.Goal()
        goal.width = width
        goal.speed = 0.1

        self.get_logger().info(f'  移动夹爪到宽度: {width}m')

        future = self.move_client.send_goal_async(goal)
        if not self._wait_for_future(future, timeout_sec=10.0):
            self.get_logger().error('  夹爪移动目标超时.')
            return False

        if future.result() is None:
            self.get_logger().error('  夹爪移动目标被拒绝或超时.')
            return False

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('  夹爪移动目标被拒绝.')
            return False

        result_future = goal_handle.get_result_async()
        if not self._wait_for_future(result_future, timeout_sec=15.0):
            self.get_logger().error('  夹爪移动结果超时.')
            return False

        if result_future.result() is None:
            self.get_logger().error('  夹爪移动结果超时.')
            return False

        return result_future.result().result.success

    def switch_controller(self, deactivate, activate):
        """
        切换控制器
        
        用于在关节位置控制器和笛卡尔阻抗力控制器之间切换。
        切换到阻抗力控制器是实现恒力控制的关键步骤。
        
        Args:
            deactivate (list): 需要停用的控制器名称列表
            activate (list): 需要激活的控制器名称列表
        
        Returns:
            bool: 切换成功返回True，失败返回False
        """
        self.get_logger().info('  [1/4] 等待控制器切换服务可用...')
        if not self.switch_controller_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('  [1/4] 控制器切换服务不可用! 请检查 /controller_manager/switch_controller 是否在运行')
            self.get_logger().error('  提示: 运行 `ros2 service list | grep switch_controller` 确认服务是否存在')
            return False
        self.get_logger().info('  [1/4] 控制器切换服务已连接.')

        req = SwitchController.Request()
        req.deactivate_controllers = deactivate
        req.activate_controllers = activate
        req.strictness = SwitchController.Request.BEST_EFFORT
        req.activate_asap = True
        req.timeout = Duration(sec=5, nanosec=0)

        self.get_logger().info(
            f'  [2/4] 发送切换请求: 停用={deactivate}, 激活={activate}, '
            f'strictness=BEST_EFFORT, timeout=5.0s')

        future = self.switch_controller_client.call_async(req)
        self.get_logger().info('  [2/4] 切换请求已发送，等待响应（最长10秒）...')

        if not self._wait_for_future(future, timeout_sec=10.0):
            self.get_logger().error('  [2/4] 控制器切换调用超时（10秒内未收到响应）!')
            self.get_logger().error('  可能原因: controller_manager 节点无响应，或控制器切换卡住')
            self.get_logger().error('  提示: 运行 `ros2 control list_controllers` 查看当前控制器状态')
            return False

        result = future.result()
        self.get_logger().info(f'  [3/4] 收到切换响应: ok={result.ok}')

        if not result.ok:
            self.get_logger().error('  [3/4] 控制器切换失败! controller_manager 返回 ok=False')
            self.get_logger().error('  可能原因:')
            self.get_logger().error(f'    - 停用的控制器不存在: {deactivate}')
            self.get_logger().error(f'    - 激活的控制器不存在: {activate}')
            self.get_logger().error('    - 控制器之间存在依赖冲突')
            self.get_logger().error('    - 硬件接口不支持同时激活这些控制器')
            self.get_logger().error('  提示: 运行 `ros2 control list_controllers` 确认控制器名称和状态')
            self.get_logger().error('  提示: 运行 `ros2 control list_controller_types` 确认控制器类型')
            return False

        self.get_logger().info('  [4/4] 控制器切换成功!')
        return True

    def execute_slide(self):
        """
        执行XY平面滑动运动（核心数据搜集阶段）
        
        滑动过程中：
        - X, Y轴：使用高刚度位置控制，按规划轨迹平滑滑动
        - Z轴：由笛卡尔阻抗力控制器保持恒定法向力（通过设置K_z=0并在前馈通道注入目标力）
        
        运动轨迹：使用5阶多项式插值（s = 6t^5 - 15t^4 + 10t^3）
        确保速度和加速度在起点和终点均为0，实现平滑的启动和停止。
        
        关键优势：
        1. 力保持真正恒定：无论EDM板如何倾斜，Z轴自动跟随保持恒定压力
        2. 数据解耦：视触觉图像法向变形一致，变化仅由纹理和剪切力引起
        """
        # 获取滑动参数
        start_x = self.get_parameter('slide_start_x').value    # 起点X坐标
        start_y = self.get_parameter('slide_start_y').value    # 起点Y坐标
        end_x = self.get_parameter('slide_end_x').value        # 终点X坐标
        end_y = self.get_parameter('slide_end_y').value        # 终点Y坐标
        z_height = self.get_parameter('slide_z_height').value  # Z轴高度（参考值）
        speed = self.get_parameter('slide_speed').value        # 滑动速度（推荐5~10 mm/s）
        orientation = self.get_parameter('slide_orientation').value  # 末端姿态

        # 计算滑动距离和持续时间
        distance = np.sqrt((end_x - start_x)**2 + (end_y - start_y)**2)
        duration = distance / speed if speed > 0 else 10.0

        self.get_logger().info(
            f'  滑动参数: ({start_x:.3f}, {start_y:.3f}) -> ({end_x:.3f}, {end_y:.3f}), '
            f'z={z_height:.3f}, speed={speed:.3f}m/s, duration={duration:.1f}s')

        # 创建100Hz发布频率
        rate = self.create_rate(100)
        start_time = self.get_clock().now()

        # 滑动主循环
        while rclpy.ok():
            # 计算已流逝时间和归一化时间t
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            t = min(elapsed / duration, 1.0)

            # 5阶多项式插值（s-curve）：s = 6t^5 - 15t^4 + 10t^3
            # 特性：t=0时s=0, ds/dt=0, d²s/dt²=0
            #       t=1时s=1, ds/dt=0, d²s/dt²=0
            s = 6.0 * t**5 - 15.0 * t**4 + 10.0 * t**3

            # 计算当前位置
            x = start_x + s * (end_x - start_x)
            y = start_y + s * (end_y - start_y)

            # 创建平衡姿态消息
            pose = PoseStamped()
            pose.header.frame_id = 'fr3_link0'           # 参考坐标系
            pose.header.stamp = self.get_clock().now().to_msg()  # 时间戳
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z_height              # Z轴参考高度
            pose.pose.orientation.x = orientation[0]
            pose.pose.orientation.y = orientation[1]
            pose.pose.orientation.z = orientation[2]
            pose.pose.orientation.w = orientation[3]

            # 发布平衡姿态给笛卡尔阻抗力控制器
            # 注：阻抗控制器会根据此姿态和力反馈调整实际位置
            self.eq_pose_pub.publish(pose)

            # 检查是否到达终点
            if t >= 1.0:
                break

            rate.sleep()

        self.get_logger().info('  滑动运动完成.')


def main(args=None):
    """
    主函数：初始化ROS节点并启动执行器
    
    使用MultiThreadedExecutor支持多线程回调处理，确保动作客户端和服务
    可以并发处理而不阻塞主线程。
    """
    rclpy.init(args=args)
    node = TaskSequenceNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

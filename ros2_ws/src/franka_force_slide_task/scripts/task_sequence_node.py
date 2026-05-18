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

from geometry_msgs.msg import PoseStamped
from franka_msgs.action import Grasp, Homing, Move
from controller_manager_msgs.srv import SwitchController
from std_srvs.srv import Trigger


class TaskSequenceNode(Node):

    def __init__(self):
        super().__init__('task_sequence_node')

        # ============ 参数声明 ============
        # 目标关节位置（初始位姿）
        self.declare_parameter('target_joint_positions', [0.0, 0.168, 0.557, -2.193, -1.1723, 1.186, 0.110])
        # 关节运动等待时间（秒）
        self.declare_parameter('motion_duration', 10.0)
        # 夹爪抓取力（牛顿）
        self.declare_parameter('gripper_force', 5.0)
        # 夹爪宽度（米）
        self.declare_parameter('gripper_width', 0.04)
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
        # 发布平衡姿态到笛卡尔阻抗力控制器
        # 注：此发布器用于发送期望位姿给阻抗控制器
        self.eq_pose_pub = self.create_publisher(
            PoseStamped, '/cartesian_impedance_force_controller/equilibrium_pose', 10)

        # ============ 服务端初始化 ============
        # 任务触发服务（通过服务调用启动任务序列）
        self.trigger_service = self.create_service(
            Trigger, '~/start', self.trigger_callback,
            callback_group=self.cb_group)

        self.get_logger().info('任务序列节点已初始化. 等待服务连接...')

        # 自动启动逻辑
        if self.get_parameter('auto_start').value:
            self.get_logger().info('自动启动已启用. 3秒后开始任务序列...')
            self.auto_start_timer = self.create_timer(3.0, self._auto_start_callback)

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
        if not self.switch_controller(
                deactivate=['joint_position_controller'],
                activate=['cartesian_impedance_force_controller']):
            self.get_logger().error('  控制器切换失败! 终止任务.')
            return
        self.get_logger().info('  控制器切换成功.')
        time.sleep(1.0)

        # 步骤4: 执行XY平面滑动（Z轴恒力控制）
        # 滑动过程中，阻抗控制器将保持Z轴恒定压力
        self.get_logger().info('步骤4: 执行XY滑动（Z轴恒力控制）...')
        self.execute_slide()
        self.get_logger().info('=== 任务序列完成 ===')

    def grasp_tool(self):
        """
        控制夹爪抓取视触觉传感器
        
        Returns:
            bool: 抓取成功返回True，失败返回False
        """
        # 等待抓取动作服务器启动
        if not self.grasp_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('抓取动作服务器不可用!')
            return False

        # 创建抓取目标
        goal = Grasp.Goal()
        goal.width = self.get_parameter('gripper_width').value      # 抓取宽度
        goal.speed = self.get_parameter('gripper_speed').value      # 夹爪运动速度
        goal.force = self.get_parameter('gripper_force').value      # 抓取力
        goal.epsilon.inner = 0.005   # 内边界容差（米）
        goal.epsilon.outer = 0.005   # 外边界容差（米）

        self.get_logger().info(
            f'  抓取参数: width={goal.width}m, speed={goal.speed}m/s, force={goal.force}N')

        # 发送异步抓取目标
        future = self.grasp_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() is None:
            self.get_logger().error('  抓取目标被拒绝或超时.')
            return False

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('  抓取目标被拒绝.')
            return False

        # 等待抓取结果
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=15.0)

        if result_future.result() is None:
            self.get_logger().error('  抓取结果超时.')
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
        # 等待控制器切换服务
        if not self.switch_controller_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('控制器切换服务不可用!')
            return False

        # 创建切换请求
        req = SwitchController.Request()
        req.deactivate_controllers = deactivate    # 停用的控制器
        req.activate_controllers = activate        # 激活的控制器
        req.strictness = SwitchController.Request.BEST_EFFORT  # 尽力模式
        req.start_asap = True     # 立即启动
        req.timeout = 5.0         # 超时时间

        self.get_logger().info(
            f'  切换控制器: 停用={deactivate}, 激活={activate}')

        # 发送异步服务调用
        future = self.switch_controller_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() is None:
            self.get_logger().error('  控制器切换调用超时.')
            return False

        return future.result().ok

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
        rclpy.shutdown()


if __name__ == '__main__':
    main()

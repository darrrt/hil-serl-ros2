# SERL 机器人基础设施
![](../docs/images/robot_infra_interfaces.png)

所有机器人代码的结构如下：
有一个 Flask 服务器通过 ROS 向机器人发送命令。有一个用于机器人的 gym 环境，通过 POST 请求与 Flask 服务器通信。

- `robot_server`: 托管一个 Flask 服务器，通过 ROS 向机器人发送命令
- `franka_env`: 用于机器人的 gym 环境，通过 POST 请求与 Flask 服务器通信


### 安装

1. 按照 [此处](https://frankarobotics.github.io/docs/installation_linux.html) 的说明安装 `libfranka` 和 `franka_ros`。

2. 然后从 https://github.com/rail-berkeley/serl_franka_controllers 安装 `serl_franka_controllers`

3. 然后安装此包及其依赖项。
    ```bash
    conda activate hilserl
    pip install -e .
    ```

### 使用
要开始使用机器人，首先打开机器人电源（地板上机器人控制箱背面的小开关）。在继续之前，在浏览器界面中校准末端执行器负载，以确保阻抗控制器的准确性。然后解锁机器人，启用 FCI，并进入执行模式（仅 FR3）。

以下命令用于启动阻抗控制器和 gym 环境与之通信的机器人服务器。对于双臂设置，您可以通过使用不同的 catkin_ws、ROS_MASTER_URI 和 flask_url 为每个手臂运行完全独立的服务器，即使它们具有不同的固件版本（我们有一个 Panda 和一个 FR3）。我们在 [launch_left_server.sh](robot_servers/launch_left_server.sh) 和 [launch_right_server.sh](robot_servers/launch_right_server.sh) 提供了示例。

```bash
cd robot_servers
conda activate hilserl

# 包含 serl_franka_controllers 包的 catkin_ws
source </path/to/catkin_ws>/devel/setup.bash

# 将 ROS master URI 设置为本地主机
export ROS_MASTER_URI=http://localhost:<ros_port_number>

# 启动 HTTP 服务器和 ROS 控制器的脚本
python franka_server.py \
    --gripper_type=<Robotiq|Franka|None> \
    --robot_ip=<robot_IP> \
    --gripper_ip=<[可选] Robotiq_gripper_IP> \
    --reset_joint_target=<[可选] robot_joints_when_robot_resets> \
    --flask_url=<url_to_serve> \
    --ros_port=<ros_port_number> \
```

这应该启动 ROS 节点阻抗控制器和 HTTP 服务器。您可以通过尝试移动末端执行器来测试运行情况，如果阻抗控制器正在运行，它应该是顺应的。

HTTP 服务器用于 ROS 控制器和 gym 环境之间的通信。可能的 HTTP 请求包括：

| 请求 | 描述 |
| --- | --- |
| startimp | 启动阻抗控制器 |
| stopimp | 停止阻抗控制器 |
| pose | 命令机器人前往期望的末端执行器位姿（以基坐标系给出，xyz+四元数） |
| getpos | 返回机器人基坐标系中的当前末端执行器位姿（xyz+rpy） |
| getvel | 返回机器人基坐标系中的当前末端执行器速度 |
| getforce | 返回刚度坐标系中末端执行器上的估计力 |
| gettorque | 返回刚度坐标系中末端执行器上的估计扭矩 |
| getq | 返回当前关节位置 |
| getdq | 返回当前关节速度 |
| getjacobian | 返回当前零雅可比矩阵 |
| getstate | 返回所有机器人状态 |
| jointreset | 执行关节复位 |
| activate_gripper | 激活夹具（仅 Robotiq） |
| reset_gripper | 重置夹具（仅 Robotiq） |
| get_gripper | 返回当前夹具位置 |
| close_gripper | 完全关闭夹具 |
| open_gripper | 完全打开夹具 |
| move_gripper | 将夹具移动到指定位置 |
| clearerr | 清除错误 |
| update_param | 更新阻抗控制器参数 |

这些命令也可以在终端中调用。常用的命令包括：
```bash
curl -X POST <flask_url>:5000/activate_gripper # 激活夹具
curl -X POST <flask_url>:5000/close_gripper # 关闭夹具
curl -X POST <flask_url>:5000/open_gripper # 打开夹具
curl -X POST <flask_url>:5000/getpos # 打印当前末端执行器位姿（xyz 平移和 xyzw 四元数）
curl -X POST <flask_url>:5000/getpos_euler # 获取当前末端执行器位姿（xyz 平移和 xyz 欧拉角）
curl -X POST <flask_url>:5000/jointreset # 执行关节复位
curl -X POST <flask_url>:5000/stopimp # 停止阻抗控制器
curl -X POST <flask_url>:5000/startimp # 启动阻抗控制器（**仅在 stopimp 之后运行**）
```

## 鸡蛋翻转控制器
这只是我们专门为鸡蛋翻转任务实现的一个非常简单的基于力/力矩的控制器。我们发布它仅供参考，以及那些真正有兴趣了解我们如何完成动态任务的人使用。

<span style="color: red">
免责声明：除非您知道自己在做什么并承担损坏机器人的风险，否则请勿使用此功能。但是，如果您决定运行此功能，请确保机器人前方、下方和上方有足够的空间，因为启动服务器会导致机器人移动到复位位置。</span>


### 安装
1. 完成上述所有步骤以安装 `serl_robot_controllers`。
2. 将 `egg_flip_controller` 目录复制到您的 `catkin_ws/src` 目录中。
3. 构建此包
    ```bash
    catkin_make --pkg egg_flip_controller
    ```
4. 启动鸡蛋翻转服务器
    ```bash
    cd robot_servers
    conda activate hilserl

    # 包含 serl_franka_controllers 包的 catkin_ws
    source </path/to/catkin_ws>/devel/setup.bash

    # 将 ROS master URI 设置为本地主机
    export ROS_MASTER_URI=http://localhost:<ros_port_number>

    # 启动 HTTP 服务器和 ROS 控制器的脚本
    python franka_server.py \
        --gripper_type=None \
        --robot_ip=<robot_IP> \
        --flask_url=<url_to_serve> \
        --ros_port=<ros_port_number> \
    ```

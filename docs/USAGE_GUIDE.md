# HIL-SERL 使用说明文档

> **论文**: [Precise and Dexterous Robotic Manipulation via Human-in-the-Loop Reinforcement Learning](https://arxiv.org/abs/2410.21845)  
> **作者**: Jianlan Luo, Charles Xu, Jeffrey Wu, Sergey Levine  
> **项目主页**: [https://hil-serl.github.io/](https://hil-serl.github.io/)  
> **对应代码仓库**: `rail-berkeley/hil-serl` (ROS2 版本)

---

## 目录

1. [方法概述](#1-方法概述)
2. [系统架构](#2-系统架构)
3. [代码模块详解](#3-代码模块详解)
4. [环境安装](#4-环境安装)
5. [最小测试环境](#5-最小测试环境)
6. [任务配置指南](#6-任务配置指南)
    - [RAM插入](#61-ram-插入)
    - [USB拾取与插入](#62-usb-拾取与插入)
    - [物品交接](#63-物品交接)
    - [翻蛋](#64-翻蛋)
7. [训练流程详解](#7-训练流程详解)
8. [核心参数说明](#8-核心参数说明)
9. [常见问题与调试](#9-常见问题与调试)
10. [引用](#10-引用)

---

## 1. 方法概述

### 1.1 论文要解决的核心问题

HIL-SERL (Human-in-the-Loop Sample-Efficient Reinforcement Learning) 是一个**基于视觉的、人在回路中的强化学习系统**，能够在**1~2.5 小时的真实机器人训练时间**内，让机械臂学会复杂的灵巧操作技能，达到接近 100% 的成功率。

### 1.2 四个核心任务

论文展示了以下四个代表性任务（复现了所有任务配置）：

| 任务 | 类型 | 关键特点 |
|------|------|----------|
| **RAM 插入** | 精密装配 | 单臂、固定夹爪（预先夹持 RAM） |
| **USB 拾取与插入** | 多阶段操作 | 单臂、学习控制夹爪（自主拾取+插入） |
| **物品交接** | 双臂协调 | 双臂、学习控制夹爪、双臂交替传递物体 |
| **翻蛋** | 动态操作 | 单臂、力控模式（关节扭矩控制） |

### 1.3 关键技术要素

论文的系统设计包含以下几个关键技术创新：

1. **RLPD（强化学习+演示数据）**：使用 SAC 算法，同时从在线探索数据和少量人类演示数据中采样（各 50%），大幅加速学习。

2. **人在回路的人类干预**：训练过程中，操作者使用 SpaceMouse 随时干预策略的动作。这些干预数据被收集到额外的干预缓冲区中，作为"演示数据"用于训练。

3. **基于视觉的奖励分类器**：不依赖人工设计奖励函数，而是训练一个基于 ResNet 的二分类/三分类视觉模型来判断任务是否完成。训练数据通过人工标注成功/失败图像帧来收集。

4. **混合动作空间（Hybrid Action Space）**：对于需要夹爪控制的单臂任务和双臂任务，将连续动作（机械臂位姿）和离散动作（夹爪开/合）联合学习：SAC 用于连续动作，DQN 用于离散夹爪动作。

5. **末端执行器相对坐标系（Relative Frame）**：所有观测和动作都在末端执行器坐标系下表达，而非世界坐标系，使策略对机械臂的绝对位姿不敏感，泛化性更强。

6. **顺应性模式（Compliance Mode）**：使用笛卡尔阻抗控制器，通过设置较低的刚度（stiffness）和阻尼（damping）参数，使机械臂在与环境接触时具有柔性，避免损坏。

7. **Actor-Learner 异步架构**：Actor 节点负责在真实环境中执行策略、收集数据；Learner 节点负责在 GPU 上训练策略。两者通过网络异步通信，支持将 Learner 部署在远程 GPU 服务器上。

---

## 2. 系统架构

### 2.1 整体架构图

```
┌──────────────────────────────────────────────────────────────┐
│                     Learner Node (GPU)                        │
│  ┌─────────────────────────────────────────────────────────┐ │
│  │  train_rlpd.py --learner                                 │ │
│  │  ┌───────────┐  ┌────────────┐  ┌──────────────────────┐│ │
│  │  │Replay Buf │  │ Demo Buffer│  │  SAC / Hybrid Agent  ││ │
│  │  │(online 50%)│  │(offline50%)│  │  (Actor+Critic+Enc)  ││ │
│  │  └───────────┘  └────────────┘  └──────────────────────┘│ │
│  └─────────────────────────────────────────────────────────┘ │
│         ↕ network (agentlace/TrainerServer)                   │
├──────────────────────────────────────────────────────────────┤
│                     Actor Node (Robot PC)                      │
│  ┌─────────────────────────────────────────────────────────┐ │
│  │  train_rlpd.py --actor                                   │ │
│  │  ┌─────────────────────────────────────────────────────┐│ │
│  │  │  Gym Env Wrapper Stack                              ││ │
│  │  │  ┌──────────────────┐                               ││ │
│  │  │  │ Reward Classifier│ ← 基于 ResNet 的分类器         ││ │
│  │  │  ├──────────────────┤                               ││ │
│  │  │  │ Chunking Wrapper │ ← 观测/动作序列窗口            ││ │
│  │  │  ├──────────────────┤                               ││ │
│  │  │  │ SERLObs Wrapper  │ ← 观测标准化与拼接             ││ │
│  │  │  ├──────────────────┤                               ││ │
│  │  │  │ Quat2Euler       │ ← 四元数→欧拉角                ││ │
│  │  │  ├──────────────────┤                               ││ │
│  │  │  │ RelativeFrame    │ ← 世界系→末端坐标系             ││ │
│  │  │  ├──────────────────┤                               ││ │
│  │  │  │ SpacemouseInterv │ ← 人类干预接口（SpaceMouse）   ││ │
│  │  │  ├──────────────────┤                               ││ │
│  │  │  │ FrankaEnv        │ ← 基础 Franka Gym 环境        ││ │
│  │  │  └──────────────────┘                               ││ │
│  │  └─────────────────────────────────────────────────────┘│ │
│  └─────────────────────────────────────────────────────────┘ │
│         ↕ HTTP (Flask REST API)                               │
├──────────────────────────────────────────────────────────────┤
│                  Robot Server (Flask + ROS2)                   │
│  ┌─────────────────────────────────────────────────────────┐ │
│  │  franka_server.py                                        │ │
│  │  ┌────────────────────────────────────────────────────┐ │ │
│  │  │  Flask API: /pose, /getstate, /close_gripper, ...  │ │ │
│  │  │  ROS2: 发布均衡位姿到 Cart Impedance Controller      │ │ │
│  │  │  ROS2: 订阅 Franka 状态 + Jacobian                   │ │ │
│  │  └────────────────────────────────────────────────────┘ │ │
│  └─────────────────────────────────────────────────────────┘ │
│         ↕ FCI (Fast Robot Interface)                          │
│  ┌─────────────────────────────────────────────────────────┐ │
│  │  Franka Robot + RealSense Cameras                        │ │
│  └─────────────────────────────────────────────────────────┘ │
└──────────────────────────────────────────────────────────────┘
```

### 2.2 数据流

```
1. Actor 从 Robot Server 读取状态 → 构建 obs
2. Actor 使用当前策略推理 → 得到 action
3. SpaceMouse 可能覆盖 action → 干预后的 action
4. Actor 发送 action 到 Robot Server → 机械臂执行
5. Actor 将 transition (obs, action, next_obs, reward, done) 发送到 Learner
6. Learner 从 replay buffer + demo buffer 各采样 50% → 更新策略
7. Learner 定期将更新后的策略同步回 Actor
```

---

## 3. 代码模块详解

### 3.1 仓库顶层结构

```
hil-serl-ros2/
├── README.md                   # 仓库主文档
├── LICENSE                     # Apache 2.0
├── docs/                       # 文档和图片
│   ├── franka_walkthrough.md   # Franka 实操指南
│   └── images/                 # 文档图片
├── serl_launcher/              # ★ 核心 RL 训练库
├── serl_robot_infra/           # ★ 机器人基础库（环境、服务端、相机）
├── examples/                   # ★ 各任务的训练脚本和配置
└── ros2_ws/                    # ROS2 工作空间
    └── src/
        ├── serl_franka_controllers/  # SERL 定制 Franka 控制器
        └── franka_ros2/              # Franka ROS2 驱动（上游依赖）
```

### 3.2 `serl_launcher/` — 核心 RL 训练库

这是整个系统的**算法核心**，提供了 RL 训练所需的所有基础设施。

#### 3.2.1 Agent 模块 (`serl_launcher/serl_launcher/agents/continuous/`)

| 文件 | 说明 |
|------|------|
| [sac.py](file:///home/xusj/hil-serl-ros2/serl_launcher/serl_launcher/agents/continuous/sac.py) | **标准 SAC Agent**：适用于单臂固定夹爪或双臂固定夹爪场景。包含 Actor、双 Q-Critic、Target Networks、自适应温度参数。使用 RLPD 从在线和离线缓冲各采样 50%。 |
| [sac_hybrid_single.py](file:///home/xusj/hil-serl-ros2/serl_launcher/serl_launcher/agents/continuous/sac_hybrid_single.py) | **单臂混合 SAC Agent**：适用于需要学习夹爪控制的单臂任务（如 USB 拾取与插入）。SAC 控制 6-DoF 位姿，DQN 控制离散夹爪动作（开/合）。额外包含 GraspCritic 网络评估夹爪 Q 值。 |
| [sac_hybrid_dual.py](file:///home/xusj/hil-serl-ros2/serl_launcher/serl_launcher/agents/continuous/sac_hybrid_dual.py) | **双臂混合 SAC Agent**：适用于双臂夹爪控制任务（如物品交接）。两个 SAC 分别控制左右臂位姿，两个 DQN 分别控制左右夹爪。 |
| [bc.py](file:///home/xusj/hil-serl-ros2/serl_launcher/serl_launcher/agents/continuous/bc.py) | 行为克隆 Agent，用于基线对比。 |

#### 3.2.2 网络模块 (`serl_launcher/serl_launcher/networks/`)

| 文件 | 说明 |
|------|------|
| [actor_critic_nets.py](file:///home/xusj/hil-serl-ros2/serl_launcher/serl_launcher/networks/actor_critic_nets.py) | Actor（策略网络）和 Critic（Q 值网络）的网络定义。Critic 使用 Ensemble 双 Q 网络。GraspCritic 用于混合动作空间的夹爪 DQN 部分。 |
| [reward_classifier.py](file:///home/xusj/hil-serl-ros2/serl_launcher/serl_launcher/networks/reward_classifier.py) | **奖励分类器**：ResNet 视觉编码器 + MLP 分类头。支持二分类（`n_way=1`）和多分类（如翻蛋任务的 `n_way=3`：蛋正面/反面/无蛋）。 |
| [mlp.py](file:///home/xusj/hil-serl-ros2/serl_launcher/serl_launcher/networks/mlp.py) | 通用 MLP 网络。 |
| [classifier.py](file:///home/xusj/hil-serl-ros2/serl_launcher/serl_launcher/networks/classifier.py) | 分类器训练工具。 |
| [lagrange.py](file:///home/xusj/hil-serl-ros2/serl_launcher/serl_launcher/networks/lagrange.py) | 拉格朗日乘子，用于约束优化。 |

#### 3.2.3 视觉模块 (`serl_launcher/serl_launcher/vision/`)

| 文件 | 说明 |
|------|------|
| [resnet_v1.py](file:///home/xusj/hil-serl-ros2/serl_launcher/serl_launcher/vision/resnet_v1.py) | ResNet-10 视觉编码器。支持 `pretrained=True` 时使用预训练权重（仅冻结）、`pretrained=False` 时从头训练。 |
| [data_augmentations.py](file:///home/xusj/hil-serl-ros2/serl_launcher/serl_launcher/vision/data_augmentations.py) | 数据增强（随机裁剪、色彩抖动等），用于训练时提高泛化能力。 |
| [spatial.py](file:///home/xusj/hil-serl-ros2/serl_launcher/serl_launcher/vision/spatial.py) | 空间 softmax 等空间特征提取。 |
| [film_conditioning_layer.py](file:///home/xusj/hil-serl-ros2/serl_launcher/serl_launcher/vision/film_conditioning_layer.py) | FiLM (Feature-wise Linear Modulation) 条件层。 |

#### 3.2.4 数据模块 (`serl_launcher/serl_launcher/data/`)

| 文件 | 说明 |
|------|------|
| [memory_efficient_replay_buffer.py](file:///home/xusj/hil-serl-ros2/serl_launcher/serl_launcher/data/memory_efficient_replay_buffer.py) | **内存高效回放缓冲区**：图像以 `uint8` 存储（而非 `float32`），在采样时才进行归一化。大幅降低 CPU 内存占用。 |
| [replay_buffer.py](file:///home/xusj/hil-serl-ros2/serl_launcher/serl_launcher/data/replay_buffer.py) | 标准回放缓冲区。 |
| [data_store.py](file:///home/xusj/hil-serl-ros2/serl_launcher/serl_launcher/data/data_store.py) | 基于 agentlace 的数据存储，用于 Actor-Learner 间数据传输。 |
| [dataset.py](file:///home/xusj/hil-serl-ros2/serl_launcher/serl_launcher/data/dataset.py) | 数据集工具。 |

#### 3.2.5 Wrapper 模块 (`serl_launcher/serl_launcher/wrappers/`)

| 文件 | 说明 |
|------|------|
| [serl_obs_wrappers.py](file:///home/xusj/hil-serl-ros2/serl_launcher/serl_launcher/wrappers/serl_obs_wrappers.py) | **SERLObsWrapper**：将字典观测空间展平为 `(state, images)` 元组，同时进行图像归一化和本体感知（proprioception）数据的拼接。 |
| [chunking.py](file:///home/xusj/hil-serl-ros2/serl_launcher/serl_launcher/wrappers/chunking.py) | **ChunkingWrapper**：支持观测序列窗口。当前配置使用 `obs_horizon=1`（单帧观测）。 |
| [norm.py](file:///home/xusj/hil-serl-ros2/serl_launcher/serl_launcher/wrappers/norm.py) | 观测/动作归一化。 |
| [video_wrapper.py](file:///home/xusj/hil-serl-ros2/serl_launcher/serl_launcher/wrappers/video_wrapper.py) | 视频录制功能。 |

#### 3.2.6 工具模块 (`serl_launcher/serl_launcher/utils/`)

| 文件 | 说明 |
|------|------|
| [launcher.py](file:///home/xusj/hil-serl-ros2/serl_launcher/serl_launcher/utils/launcher.py) | **核心工厂函数**：`make_sac_pixel_agent`（SAC）、`make_sac_pixel_agent_hybrid_single_arm`（单臂混合）、`make_sac_pixel_agent_hybrid_dual_arm`（双臂混合）、`make_trainer_config`、`make_wandb_logger`。 |
| [train_utils.py](file:///home/xusj/hil-serl-ros2/serl_launcher/serl_launcher/utils/train_utils.py) | 训练工具函数。 |
| [timer_utils.py](file:///home/xusj/hil-serl-ros2/serl_launcher/serl_launcher/utils/timer_utils.py) | 计时工具。 |
| [logging_utils.py](file:///home/xusj/hil-serl-ros2/serl_launcher/serl_launcher/utils/logging_utils.py) | 日志工具。 |
| [jax_utils.py](file:///home/xusj/hil-serl-ros2/serl_launcher/serl_launcher/utils/jax_utils.py) | JAX 工具函数。 |

### 3.3 `serl_robot_infra/` — 机器人基础库

#### 3.3.1 机器人服务端 (`serl_robot_infra/robot_servers/`)

| 文件 | 说明 |
|------|------|
| [franka_server.py](file:///home/xusj/hil-serl-ros2/serl_robot_infra/robot_servers/franka_server.py) | **Franka 标准服务端**：启动 Flask HTTP 服务 + ROS2 节点。核心 API 包括 `/pose`（发送笛卡尔均衡位姿）、`/getstate`（获取完整状态）、`/close_gripper`/`/open_gripper`（夹爪控制）、`/update_param`（在线修改阻抗参数）、`/jointreset`（关节重置）。 |
| [franka_eggflip_server.py](file:///home/xusj/hil-serl-ros2/serl_robot_infra/robot_servers/franka_eggflip_server.py) | **翻蛋专用服务端**：使用力控（joint torque）模式而非阻抗模式。API 发送关节扭矩命令 `/torque`。 |
| [robotiq_gripper_server.py](file:///home/xusj/hil-serl-ros2/serl_robot_infra/robot_servers/robotiq_gripper_server.py) | Robotiq 夹爪的 Modbus TCP 通信控制。 |
| [franka_gripper_server.py](file:///home/xusj/hil-serl-ros2/serl_robot_infra/robot_servers/franka_gripper_server.py) | Franka 原装夹爪的 ROS2 控制。 |
| `launch_right_server.sh` | 右手 Franka + Robotiq 夹爪的启动脚本。 |
| `launch_left_server.sh` | 左手 Franka 的启动脚本。 |
| `launch_right_eggflip_server.sh` | 翻蛋任务的启动脚本。 |

#### 3.3.2 Gym 环境 (`serl_robot_infra/franka_env/envs/`)

| 文件 | 说明 |
|------|------|
| [franka_env.py](file:///home/xusj/hil-serl-ros2/serl_robot_infra/franka_env/envs/franka_env.py) | **基础 Franka Gym 环境**：实现 `gym.Env` 接口。关键功能：`step()` 发送笛卡尔位移增量和夹爪命令；`reset()` 回到预设位姿（支持随机化）；内置安全边界框（Bounding Box）；RealSense 相机图像采集（128×128 分辨率）。 |
| [franka_wrench_env.py](file:///home/xusj/hil-serl-ros2/serl_robot_infra/franka_env/envs/franka_wrench_env.py) | **翻蛋专用环境**：使用扭矩控制（非位移控制）。发送关节扭矩命令而非位姿命令。 |
| [dual_franka_env.py](file:///home/xusj/hil-serl-ros2/serl_robot_infra/franka_env/envs/dual_franka_env.py) | **双臂环境**：管理两个 FrankaEnv 实例，合并左右的观测和动作空间。 |
| [wrappers.py](file:///home/xusj/hil-serl-ros2/serl_robot_infra/franka_env/envs/wrappers.py) | **环境 Wrapper 集合**：`SpacemouseIntervention`（单臂干预）、`DualSpacemouseIntervention`（双臂干预）、`MultiCameraBinaryRewardClassifierWrapper`（奖励分类器包装）、`Quat2EulerWrapper`（四元数→欧拉角）、`GripperCloseEnv`（固定夹爪状态）、`GripperPenaltyWrapper`（夹爪动作惩罚）。 |
| [relative_env.py](file:///home/xusj/hil-serl-ros2/serl_robot_infra/franka_env/envs/relative_env.py) | **坐标系变换**：将观测和动作从世界坐标系转换到末端执行器坐标系。`RelativeFrame` 用于单臂，`DualRelativeFrame` 用于双臂。 |

#### 3.3.3 相机模块 (`serl_robot_infra/franka_env/camera/`)

| 文件 | 说明 |
|------|------|
| [rs_capture.py](file:///home/xusj/hil-serl-ros2/serl_robot_infra/franka_env/camera/rs_capture.py) | RealSense 相机捕捉，支持设置曝光和分辨率。 |
| [video_capture.py](file:///home/xusj/hil-serl-ros2/serl_robot_infra/franka_env/camera/video_capture.py) | 多线程视频捕获。 |
| [multi_video_capture.py](file:///home/xusj/hil-serl-ros2/serl_robot_infra/franka_env/camera/multi_video_capture.py) | 多相机视频捕获。 |

#### 3.3.4 SpaceMouse 模块 (`serl_robot_infra/franka_env/spacemouse/`)

| 文件 | 说明 |
|------|------|
| [spacemouse_expert.py](file:///home/xusj/hil-serl-ros2/serl_robot_infra/franka_env/spacemouse/spacemouse_expert.py) | SpaceMouse 的 6-DoF 遥操作接口，将输入映射为机械臂末端的速度指令。 |

### 3.4 `examples/` — 训练脚本与任务配置

| 文件/目录 | 说明 |
|-----------|------|
| [train_rlpd.py](file:///home/xusj/hil-serl-ros2/examples/train_rlpd.py) | **主训练脚本**：支持 `--actor`（数据收集）和 `--learner`（策略训练）两种模式。根据配置自动选择 SAC / SAC Hybrid Single / SAC Hybrid Dual 的 Agent 类型。实现 RLPD（50% 在线 + 50% 离线/干预数据）采样。 |
| [train_reward_classifier.py](file:///home/xusj/hil-serl-ros2/examples/train_reward_classifier.py) | **奖励分类器训练脚本**：使用收集的成功/失败图像数据训练视觉分类器。 |
| [record_demos.py](file:///home/xusj/hil-serl-ros2/examples/record_demos.py) | **示范录制脚本**：使用 SpaceMouse 录制专家示范。奖励分类器自动判断成功并重置 episode。 |
| [record_success_fail.py](file:///home/xusj/hil-serl-ros2/examples/record_success_fail.py) | **奖励分类器数据收集脚本**：按空格键标记成功帧，默认所有帧标记为失败。 |
| [experiments/config.py](file:///home/xusj/hil-serl-ros2/examples/experiments/config.py) | 默认训练配置基类（学习率、批量大小、折扣因子等）。 |
| [experiments/mappings.py](file:///home/xusj/hil-serl-ros2/examples/experiments/mappings.py) | 任务名到配置类的映射字典。 |
| `experiments/ram_insertion/` | RAM 插入任务的配置（[config.py](file:///home/xusj/hil-serl-ros2/examples/experiments/ram_insertion/config.py)）、环境（[wrapper.py](file:///home/xusj/hil-serl-ros2/examples/experiments/ram_insertion/wrapper.py)）、启动脚本。 |
| `experiments/usb_pickup_insertion/` | USB 任务的配置、环境、启动脚本。 |
| `experiments/object_handover/` | 物品交接任务的配置、环境、启动脚本。 |
| `experiments/egg_flip/` | 翻蛋任务的配置、环境、启动脚本。 |

### 3.5 `ros2_ws/src/serl_franka_controllers/` — ROS2 控制器

| 文件 | 说明 |
|------|------|
| [cartesian_impedance_controller.cpp](file:///home/xusj/hil-serl-ros2/ros2_ws/src/serl_franka_controllers/src/cartesian_impedance_controller.cpp) | **笛卡尔阻抗控制器**：通过 ROS2 订阅外部发布的均衡位姿（equilibrium pose），使用阻抗控制律计算关节扭矩。支持通过 ROS2 参数在线调整刚度、阻尼、Ki 和裁剪限制。 |
| [joint_position_controller.cpp](file:///home/xusj/hil-serl-ros2/ros2_ws/src/serl_franka_controllers/src/joint_position_controller.cpp) | 关节位置控制器，用于关节重置。 |
| [config/serl_franka_controllers.yaml](file:///home/xusj/hil-serl-ros2/ros2_ws/src/serl_franka_controllers/config/serl_franka_controllers.yaml) | 控制器参数配置文件。 |
| [launch/impedance.launch.py](file:///home/xusj/hil-serl-ros2/ros2_ws/src/serl_franka_controllers/launch/impedance.launch.py) | 阻抗控制器启动文件。 |
| [launch/joint.launch.py](file:///home/xusj/hil-serl-ros2/ros2_ws/src/serl_franka_controllers/launch/joint.launch.py) | 关节控制器启动文件。 |

---

## 4. 环境安装

### 4.1 硬件要求

- **Franka Emika 机械臂**（FR3 或 FP3）+ FCI 功能激活
- **NVIDIA GPU**（推荐 RTX 3070 及以上，用于训练 Learner）
- **Intel RealSense 相机**（至少 1 台，推荐 2-3 台：2 台腕部 + 1 台侧面）
- **3DConnexion SpaceMouse**（用于人类干预和示范录制）
- **Robotiq 2F-85 夹爪**（或 Franka 原装夹爪）
- **以太网交换机**：机器人控制箱、实时 PC、GPU 工作站之间需要网络连接

### 4.2 软件安装步骤

**Step 1: 创建 Conda 环境**

```bash
conda create -n hilserl python=3.10
conda activate hilserl
```

**Step 2: 安装 JAX（GPU 版本）**

```bash
# CUDA 12 版本
pip install --upgrade "jax[cuda12_pip]==0.4.35" -f https://storage.googleapis.com/jax-releases/jax_cuda_releases.html
```

> **注意**: JAX 版本应与 CUDA 版本匹配。如果仅用 CPU（不推荐，训练会非常慢），使用 `pip install --upgrade "jax[cpu]"`。

**Step 3: 安装 serl_launcher**

```bash
cd serl_launcher
pip install -e .
pip install -r requirements.txt
cd ..
```

**Step 4: 安装 serl_robot_infra**

```bash
cd serl_robot_infra
pip install -e .
cd ..
```

**Step 5: 安装 ROS2 和 Franka 驱动**

参考 [serl_robot_infra/README.md](file:///home/xusj/hil-serl-ros2/serl_robot_infra/README.md) 和 [ROS2 Humble 官方文档](https://docs.ros.org/en/humble/Installation.html)：

```bash
# 编译 ROS2 工作空间
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 5. 最小测试环境

### 5.1 验证 Franka 控制器

编译完成后，测试阻抗控制器能否启动：

```bash
source ros2_ws/install/setup.bash
ros2 launch serl_franka_controllers impedance.launch.py robot_ip:=172.16.0.2
```

如果成功启动，应该会看到控制器加载信息。"172.16.0.2" 需替换为你的机器人控制箱 IP。

### 5.2 验证 Robot Server

在一个终端中启动 Robot Server：

```bash
cd serl_robot_infra/robot_servers
bash launch_right_server.sh
```

> 修改脚本中的 `--robot_ip`、`--gripper_ip` 和 `--flask_url` 以匹配你的设置。

在另一个终端测试 API 连接：

```bash
# 获取当前位姿（欧拉角格式）
curl -X POST http://127.0.0.1:5000/getpos_euler

# 获取完整状态
curl -X POST http://127.0.0.1:5000/getstate

# 打开夹爪
curl -X POST http://127.0.0.1:5000/open_gripper
```

### 5.3 验证 Python 环境

```bash
python -c "
from franka_env.envs.franka_env import FrankaEnv, DefaultEnvConfig
from serl_launcher.agents.continuous.sac import SACAgent
print('All imports successful!')
"
```

### 5.4 最小端到端测试（无机械臂）

使用 RAM 插入任务的配置进行**无机械臂的"fake_env"测试**（Learner 模式）：

```bash
cd examples/experiments/ram_insertion
python ../../train_rlpd.py --exp_name=ram_insertion --checkpoint_path=test_run --learner
```

这会启动 Learner 而不连接真实机器人，可以验证训练管线是否正常。

---

## 6. 任务配置指南

每配置一个新任务，需要创建 `examples/experiments/<task_name>/` 目录，包含以下文件：

- `config.py`：环境配置 + 训练配置
- `wrapper.py`：任务特定的环境包装器
- `run_actor.sh`：Actor 启动脚本
- `run_learner.sh`：Learner 启动脚本

并在 `examples/experiments/mappings.py` 中注册任务。

### 6.1 RAM 插入

**任务描述**：单臂预先夹持 RAM 条，插入主板上的插槽。固定夹爪状态（不需要学习夹爪控制）。

**硬件设置**：
- 1 台 Franka FR3（右手）
- 2 台腕部 RealSense 相机（用于策略 + 分类器）
- Robotiq 夹爪
- RAM 条放在固定的支架上

**配置要点** ([config.py](file:///home/xusj/hil-serl-ros2/examples/experiments/ram_insertion/config.py))：

```python
# EnvConfig 关键参数
TARGET_POSE = np.array([...])      # RAM 完全插入时的末端位姿
GRASP_POSE = np.array([...])       # 抓取 RAM 时的位姿
RESET_POSE = TARGET_POSE + [0, 0, 0.05, 0, 0.05, 0]  # 重置位姿
RANDOM_RESET = True                # 每次重置时随机化 XY 和 RZ
RANDOM_XY_RANGE = 0.02             # XY 随机范围（米）
RANDOM_RZ_RANGE = 0.05             # RZ 随机范围（弧度）
ABS_POSE_LIMIT_HIGH/LOW            # 安全边界框
ACTION_SCALE = (0.01, 0.06, 1)    # (位移缩放, 旋转缩放, 夹爪缩放)
MAX_EPISODE_LENGTH = 100           # 每 episode 最大步数

# TrainConfig 关键参数
image_keys = ["wrist_1", "wrist_2"]     # 策略使用的相机
classifier_keys = ["wrist_1", "wrist_2"] # 分类器使用的相机
setup_mode = "single-arm-fixed-gripper"  # 关键：固定夹爪模式
```

**环境 Wrapper 栈** ([config.py:L102-L129](file:///home/xusj/hil-serl-ros2/examples/experiments/ram_insertion/config.py#L102-L129))：

```
RAMEnv → GripperCloseEnv → SpacemouseIntervention
→ RelativeFrame → Quat2EulerWrapper → SERLObsWrapper
→ ChunkingWrapper → MultiCameraBinaryRewardClassifierWrapper
```

**任务特有逻辑** ([wrapper.py](file:///home/xusj/hil-serl-ros2/examples/experiments/ram_insertion/wrapper.py))：
- `regrasp()`：按 F1 键自动执行重新抓取 RAM 条的流程
- `go_to_reset()`：先抬升再移动到重置位姿，避免碰撞
- 使用 `COMPLIANCE_PARAM`（低刚度）和 `PRECISION_PARAM`（高刚度）两种阻抗参数切换

**操作流程**：

1. 编辑 `config.py`：修改 `SERVER_URL`、相机序列号、`IMAGE_CROP`、`TARGET_POSE` 等。
2. 收集分类器数据：`python record_success_fail.py --exp_name ram_insertion --successes_needed 200`
3. 训练分类器：`cd experiments/ram_insertion && python ../../train_reward_classifier.py --exp_name ram_insertion`
4. 录制示范：`python ../../record_demos.py --exp_name ram_insertion --successes_needed 20`
5. 启动训练：`bash run_actor.sh` + `bash run_learner.sh`（需在两个终端分别运行）
6. 评估：在 `run_actor.sh` 中添加 `--eval_checkpoint_step=<step> --eval_n_trajs=10`

**训练时间参考**：约 1.5 小时达到 100% 成功率。

---

### 6.2 USB 拾取与插入

**任务描述**：单臂从平面上拾取 USB 设备，插入 USB 端口。需要学习夹爪控制（拾取阶段）。

**硬件设置**：
- 1 台 Franka FR3（右手）
- 2 台腕部相机 + 1 台侧面相机（侧面相机用于分类器）
- Robotiq 夹爪

**配置要点** ([config.py](file:///home/xusj/hil-serl-ros2/examples/experiments/usb_pickup_insertion/config.py))：

```python
# 与 RAM 插入的主要区别
setup_mode = "single-arm-learned-gripper"   # 学习夹爪控制（混合 SAC+DQN）
image_keys = ["side_policy", "wrist_1", "wrist_2"]  # 3 个相机视图
classifier_keys = ["side_classifier"]       # 仅侧面相机用于分类器
MAX_EPISODE_LENGTH = 120                   # 更长的 episode
ACTION_SCALE = np.array([0.015, 0.1, 1])   # 更大的动作缩放

# 多相机同源的情况：side_policy 和 side_classifier 使用同一相机但不同裁剪
REALSENSE_CAMERAS = {
    "side_policy": {
        "serial_number": "130322274175", ...
    },
    "side_classifier": {
        "serial_number": "130322274175", ...  # 同一相机
    },
}
```

**环境 Wrapper 栈**：
```
USBEnv → SpacemouseIntervention → RelativeFrame
→ Quat2EulerWrapper → SERLObsWrapper → ChunkingWrapper
→ MultiCameraBinaryRewardClassifierWrapper → GripperPenaltyWrapper
```

**任务特有逻辑** ([wrapper.py](file:///home/xusj/hil-serl-ros2/examples/experiments/usb_pickup_insertion/wrapper.py))：
- 自主拾取阶段的随机化
- `init_cameras` 中对同一相机多个视图的处理（避免重复初始化）
- `GripperPenaltyWrapper(penalty=-0.02)`：每次夹爪状态切换施加 -0.02 的惩罚，鼓励减少不必要的夹爪动作

**关于混合动作空间（Hybrid Action Space）**：
当 `setup_mode = "single-arm-learned-gripper"` 时，策略的动作空间为 7 维：`(x, y, z, rx, ry, rz, gripper)`。其中前 6 维由 SAC 控制（连续动作），夹爪维度由 DQN 控制（离散动作：开/合）。Agent 类型为 `SACAgentHybridSingleArm`，包含一个额外的 `GraspCritic` 网络。

**训练时间参考**：约 2.5 小时达到 100% 成功率。

---

### 6.3 物品交接

**任务描述**：双臂协调，一只手拿着物体传递给另一只手。需要学习双臂夹爪控制。

**硬件设置**：
- 2 台 Franka FR3（左右各一台）
- 左臂腕部相机 + 侧面相机（共用，不同裁剪用于策略和分类器）
- 右臂腕部相机
- Robotiq 夹爪 × 2

**配置要点** ([config.py](file:///home/xusj/hil-serl-ros2/examples/experiments/object_handover/config.py))：

```python
# 双臂训练配置
setup_mode = "dual-arm-learned-gripper"  # 双臂混合 SAC+DQN
image_keys = ["left/wrist", "right/wrist", "left/side"]  # prefix 前缀区分左右
classifier_keys = ["left/side_classifier"]                # 左手侧面相机
proprio_keys = [
    "left/tcp_pose", "left/tcp_vel", "left/gripper_pose",
    "right/tcp_pose", "right/tcp_vel", "right/gripper_pose",
]
MAX_EPISODE_LENGTH = 200  # 双臂任务需要更长的时间

# 左右臂分别配置
LeftEnvConfig: SERVER_URL = "http://127.0.0.1:5000/"   # 左手
RightEnvConfig: SERVER_URL = "http://127.0.0.2:5000/"  # 右手
# 左右臂有各自独立的：
#   - RESET_POSE
#   - ABS_POSE_LIMIT_HIGH/LOW（安全边界）
#   - COMPLIANCE_PARAM / PRECISION_PARAM
#   - ACTION_SCALE
```

**环境 Wrapper 栈**：
```
HandOffEnv(left) + HandOffEnv(right) → DualFrankaEnv
→ DualSpacemouseIntervention → DualRelativeFrame
→ DualQuat2EulerWrapper → SERLObsWrapper
→ ChunkingWrapper → MultiCameraBinaryRewardClassifierWrapper
→ GripperPenaltyWrapper
```

**双臂启动**：
- 左手服务器：`bash launch_left_server.sh`
- 右手服务器：`bash launch_right_server.sh`

**关于双臂混合动作空间**：
动作空间为 14 维：`(left_x, ..., left_gripper, right_x, ..., right_gripper)`。Agent 类型为 `SACAgentHybridDualArm`，包含两个 GraspCritic 网络分别学习左右夹爪。

**训练时间参考**：约 1.5 小时。

---

### 6.4 翻蛋

**任务描述**：单臂使用铲子翻转锅中的蛋。使用力控模式（扭矩控制）实现动态操作。

**硬件设置**：
- 1 台 Franka FR3（右手）
- 腕部相机 + 侧面相机
- 需要特殊的铲子末端工具

**配置要点** ([config.py](file:///home/xusj/hil-serl-ros2/examples/experiments/egg_flip/config.py))：

```python
# 翻蛋使用的环境类型不同
env = FrankaWrenchEnv(...)   # 不是 FrankaEnv

# 动作空间简化为 3 维：仅控制 x 平移、z 平移、ry 旋转
# 通过 EggFlipActionWrapper 实现

# 分类器为三分类（而非二分类）
classifier_func = load_classifier_func(
    n_way=3,  # 蛋正面 / 蛋反面 / 无蛋
)

# 特殊折扣因子（动态任务）
discount = 0.985
setup_mode = "single-arm-fixed-gripper"
```

**环境 Wrapper 栈**：
```
FrankaWrenchEnv → EggFlipActionWrapper
→ EggFlipSpacemouseIntervention → Quat2EulerWrapper
→ SERLObsWrapper → ChunkingWrapper → EggClassifierWrapper
```

**翻蛋服务端**：
使用 `franka_eggflip_server.py` 启动力控服务端：
```bash
bash launch_right_eggflip_server.sh
```

**任务特有逻辑** ([wrapper.py](file:///home/xusj/hil-serl-ros2/examples/experiments/egg_flip/wrapper.py))：
- `EggClassifierWrapper`：三分类判断蛋的状态。初始化时记录蛋的初始状态（正面或反面），当分类器置信度 >= 0.9，且状态从正面变为反面（或反之），则判定成功。
- `EggFlipActionWrapper`：将 3 维动作映射到完整 7 维：只控制 `[x, z, ry]`。
- `EggFlipSpacemouseIntervention`：干预具有 0.5 秒的保持时间，适合动态任务。

**训练时间参考**：约 1 小时。

---

### 6.5 配置文件快速参考

| 配置项 | RAM 插入 | USB 拾取插入 | 物品交接 | 翻蛋 |
|--------|----------|-------------|----------|------|
| `setup_mode` | `single-arm-fixed-gripper` | `single-arm-learned-gripper` | `dual-arm-learned-gripper` | `single-arm-fixed-gripper` |
| Agent | SACAgent | SACAgentHybridSingleArm | SACAgentHybridDualArm | SACAgent |
| 动作维度 | 6 | 7 | 14 | 3 (映射到7) |
| `MAX_EPISODE_LENGTH` | 100 | 120 | 200 | 100 |
| `discount` | 0.97 | 0.98 | 0.97 | 0.985 |
| `ACTION_SCALE` | (0.01, 0.06, 1) | (0.015, 0.1, 1) | (0.04, 0.2, 1) | N/A |
| 分类器类型 | 二分类 | 二分类 | 二分类 | 三分类 |
| 环境基类 | FrankaEnv | FrankaEnv | DualFrankaEnv | FrankaWrenchEnv |
| 服务端 | franka_server.py | franka_server.py | franka_server.py ×2 | franka_eggflip_server.py |

---

## 7. 训练流程详解

### 7.1 完整训练流水线

```
Step 1: 环境配置
├── 修改 config.py 中的 SERVER_URL、相机序列号、TARGET_POSE 等
├── 调整 IMAGE_CROP 裁剪参数
└── 设置安全边界 ABS_POSE_LIMIT

Step 2: 收集奖励分类器数据
├── python record_success_fail.py --exp_name <task_name>
├── 按空格键标记成功帧（假阳性很重要！）
└── 收集 2-3 倍于成功帧的失败帧

Step 3: 训练奖励分类器
├── python train_reward_classifier.py --exp_name <task_name>
└── 检查准确率，必要时返回 Step 2

Step 4: 录制示范
├── python record_demos.py --exp_name <task_name> --successes_needed 20
└── 同时验证分类器准确性

Step 5: 训练策略
├── bash run_learner.sh（先启动，等待 Actor 连接）
├── bash run_actor.sh
└── 训练过程中适当干预

Step 6: 评估
└── run_actor.sh 中添加 --eval_checkpoint_step=<n> --eval_n_trajs=<n>
```

### 7.2 Actor-Learner 通信

Actor 和 Learner 通过 `agentlace` 库以 TCP 网络协议通信：

- **Actor → Learner**：发送 transition 数据到 `actor_env` 和 `actor_env_intvn` 两个 DataStore。
- **Learner → Actor**：定期发送更新后的策略参数。
- 默认配置下，Actor 和 Learner 运行在同一台机器上（`--ip=localhost`），但可以在 `run_actor.sh` 中修改 IP 以连接到远程 Learner。

### 7.3 干预策略建议（来自论文和文档）

论文和实操指南提供了以下干预策略建议：

1. **训练初期频繁干预**：在开始阶段（前 30 分钟），每 2-3 个 episode 干预一次，帮助策略探索正确方向。
2. **交替探索与干预**：让策略自由探索 20-30 步，然后干预引导到目标附近，让策略练习关键步骤。
3. **确保高频奖励**：在训练初期至少让 1/3 的 episodes 获得奖励，加速价值函数传播。
4. **训练后期减少干预**：当策略能独立偶尔完成任务时，大幅减少干预频率，仅在策略反复犯错时干预。
5. **练习恢复行为**：有意干预让策略练习从错误中恢复（如 USB 任务中练习从掉落的 USB 中恢复）。

### 7.4 检查点与恢复

- 每 `checkpoint_period` 步自动保存检查点到 `checkpoint_path`。
- 每 `buffer_period` 步自动保存回放缓冲区数据。
- 要恢复训练，将 `checkpoint_path` 指向之前的运行目录，代码会自动加载最新的检查点和缓冲区数据。

---

## 8. 核心参数说明

### 8.1 `DefaultTrainingConfig` 核心参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `max_steps` | 1,000,000 | 最大训练步数 |
| `batch_size` | 256 | 每次更新的批量大小 |
| `cta_ratio` | 2 | CTA 比率：每个 actor update 前的 critic update 次数。增大可加速 critic 收敛 |
| `discount` | 0.97 | 折扣因子 γ。动态任务（如翻蛋）可设更高（0.985） |
| `replay_buffer_capacity` | 200,000 | 回放缓冲区容量 |
| `random_steps` | 0 | 纯随机探索步数（HIL-SERL 不需要） |
| `training_starts` | 100 | 开始训练前需要的最小数据量 |
| `steps_per_update` | 50 | Learner 向 Actor 同步策略的步数间隔 |
| `encoder_type` | `resnet-pretrained` | 视觉编码器类型：`resnet`（从头训练）或 `resnet-pretrained`（冻结预训练权重） |
| `image_keys` | 无 | 策略使用的相机键列表 |
| `classifier_keys` | 无 | 奖励分类器使用的相机键列表 |
| `proprio_keys` | 无 | 本体感知键列表（`tcp_pose`, `tcp_vel`, `tcp_force`, `tcp_torque`, `gripper_pose` 等） |
| `setup_mode` | `single-arm-fixed-gripper` | 硬件模式，决定 Agent 类型 |

### 8.2 `DefaultEnvConfig` 核心参数

| 参数 | 说明 |
|------|------|
| `SERVER_URL` | Franka 服务端 URL |
| `REALSENSE_CAMERAS` | RealSense 相机配置字典（serial_number, dim, exposure） |
| `IMAGE_CROP` | 图像裁剪函数字典 |
| `TARGET_POSE` | 目标任务完成时的末端位姿（欧拉角格式，6 维） |
| `RESET_POSE` | Episode 重置时回到的位姿 |
| `RANDOM_RESET` | 是否启用重置位姿的随机化 |
| `RANDOM_XY_RANGE` | 重置时 XY 随机范围（米） |
| `RANDOM_RZ_RANGE` | 重置时 RZ 随机范围（弧度） |
| `ABS_POSE_LIMIT_HIGH/LOW` | 安全边界框（世界坐标系） |
| `ACTION_SCALE` | 动作缩放：`(位移缩放, 旋转缩放, 夹爪缩放)` |
| `COMPLIANCE_PARAM` | 顺应性模式阻抗参数（训练时使用，刚度低） |
| `PRECISION_PARAM` | 精确模式阻抗参数（重置时使用，刚度高） |
| `MAX_EPISODE_LENGTH` | Episode 最大步数 |
| `JOINT_RESET_PERIOD` | 关节重置周期（每 N 个 episode 执行一次关节重置） |

### 8.3 阻抗控制参数说明

`COMPLIANCE_PARAM` 和 `PRECISION_PARAM` 的关键子参数：

| 参数 | 说明 |
|------|------|
| `translational_stiffness` | 平移刚度（N/m），顺应性模式通常 2000 |
| `translational_damping` | 平移阻尼（Ns/m），通常 89 |
| `rotational_stiffness` | 旋转刚度（Nm/rad），顺应性模式通常 150 |
| `rotational_damping` | 旋转阻尼（Nms/rad），通常 7 |
| `translational_clip_*` | 各方向的力/位置裁剪限制，用于限制顺应性模式下的最大偏差 |
| `rotational_clip_*` | 各旋转轴的力矩裁剪限制 |
| `*_Ki` | 积分项增益（通常设为 0） |

---

## 9. 常见问题与调试

### 9.1 奖励分类器误判

**症状**：训练过程中 episode 在不该结束时提前终止，或任务完成但分类器不给出奖励。

**解决方案**：
1. 收集更多分类器数据，特别是针对误判场景的数据
2. 对于假阳性（False Positive），收集更多该场景下的负样本
3. 调整 `reward_func` 中的 sigmoid 阈值（默认 0.85）
4. 添加辅助条件（如 RAM 插入中的 z 轴位置检查）
5. 考虑添加更多/不同的相机视图用于分类器

### 9.2 策略不收敛

**症状**：成功率长时间不提升或始终为零。

**排查步骤**：
1. 检查分类器是否正常工作（通过 `record_demos.py` 验证）
2. 确认安全边界框设置合理，不会限制关键动作
3. 增加干预频率，特别是在训练早期
4. 降低 `ACTION_SCALE` 以获得更精细的控制
5. 检查示范数据质量（确认是成功的 trajectory）
6. 调整阻抗参数：过硬的顺应性模式可能导致策略无法探索

### 9.3 机械臂抖动或振荡

**症状**：机械臂在执行策略时出现抖动。

**解决方案**：
1. 增加阻尼参数（`translational_damping`、`rotational_damping`）
2. 减小动作缩放（`ACTION_SCALE` 的前两项）
3. 检查 `hz` 参数（默认 10 Hz），确保策略推理和通信频率一致
4. 检查 Jacobian 是否正确更新（查看 `_set_jacobian` 订阅是否正常）

### 9.4 相机问题

**症状**：相机输入为黑色或冻结。

**排查步骤**：
1. 在 RealSense Viewer 中确认相机工作正常
2. 检查序列号是否与 `REALSENSE_CAMERAS` 中的一致
3. 调整 `exposure` 参数（可尝试 10000-40000 范围）
4. 检查 USB 带宽：多个相机可能需要不同的 USB 控制器
5. 查看终端中是否有相机冻结的警告信息

### 9.5 内存不足

**症状**：训练过程中 OOM（内存不足）。

**解决方案**：
1. 减小 `replay_buffer_capacity`（默认 200,000）
2. 减小 `IMAGE_CROP` 裁剪尺寸
3. 降低相机原始分辨率（`dim` 参数）
4. 在 `run_actor.sh` 中设置 `XLA_PYTHON_CLIENT_MEM_FRACTION` 限制 JAX 内存占用

### 9.6 无法连接到 Robot Server

**症状**：启动 Actor 时出现连接错误。

**排查步骤**：
1. 确认 `franka_server.py` 正确启动且无报错
2. 确认 `SERVER_URL` 中的 IP 地址和端口正确
3. 测试：`curl -X POST http://<url>:5000/getstate`
4. 检查 Franka Desk 中 FCI 是否激活

### 9.7 关节漂移问题

**症状**：长时间运行后机械臂关节出现漂移。

**解决方案**：
1. 设置 `JOINT_RESET_PERIOD`（如 200），定期重置关节
2. 在早于配置周期时手动触发关节重置（`curl -X POST <url>/jointreset`）

---

## 10. 引用

如果您在研究中使用了 HIL-SERL，请引用以下论文：

```bibtex
@misc{luo2024hilserl,
      title={Precise and Dexterous Robotic Manipulation via Human-in-the-Loop Reinforcement Learning},
      author={Jianlan Luo and Charles Xu and Jeffrey Wu and Sergey Levine},
      year={2024},
      eprint={2410.21845},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
```

---

## 附录：关键文件索引

| 路径 | 作用 |
|------|------|
| [train_rlpd.py](file:///home/xusj/hil-serl-ros2/examples/train_rlpd.py) | 主训练脚本（Actor + Learner） |
| [record_demos.py](file:///home/xusj/hil-serl-ros2/examples/record_demos.py) | 示范数据录制 |
| [record_success_fail.py](file:///home/xusj/hil-serl-ros2/examples/record_success_fail.py) | 分类器数据收集 |
| [train_reward_classifier.py](file:///home/xusj/hil-serl-ros2/examples/train_reward_classifier.py) | 分类器训练 |
| [franka_env.py](file:///home/xusj/hil-serl-ros2/serl_robot_infra/franka_env/envs/franka_env.py) | 基础 Franka Gym 环境 |
| [wrappers.py](file:///home/xusj/hil-serl-ros2/serl_robot_infra/franka_env/envs/wrappers.py) | 环境 Wrapper（干预、分类器、坐标系转换等） |
| [franka_server.py](file:///home/xusj/hil-serl-ros2/serl_robot_infra/robot_servers/franka_server.py) | 机器人 Flask 服务端 |
| [sac_hybrid_single.py](file:///home/xusj/hil-serl-ros2/serl_launcher/serl_launcher/agents/continuous/sac_hybrid_single.py) | 单臂混合 SAC Agent |
| [launcher.py](file:///home/xusj/hil-serl-ros2/serl_launcher/serl_launcher/utils/launcher.py) | Agent 工厂函数 |
| [reward_classifier.py](file:///home/xusj/hil-serl-ros2/serl_launcher/serl_launcher/networks/reward_classifier.py) | 奖励分类器网络 |
| [cartesian_impedance_controller.cpp](file:///home/xusj/hil-serl-ros2/ros2_ws/src/serl_franka_controllers/src/cartesian_impedance_controller.cpp) | 笛卡尔阻抗控制器 |
| [mappings.py](file:///home/xusj/hil-serl-ros2/examples/experiments/mappings.py) | 任务配置映射 |
| [franka_walkthrough.md](file:///home/xusj/hil-serl-ros2/docs/franka_walkthrough.md) | 原始实操指南 |
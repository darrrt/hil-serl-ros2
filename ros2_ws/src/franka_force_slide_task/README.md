## 一、目录总览

```
serl_franka_controllers/
├── cfg/
│   └── compliance_param.cfg          # ROS1 动态参数配置(遗留)
├── config/
│   └── serl_franka_controllers.yaml  # ROS2 控制器参数配置
├── include/serl_franka_controllers/
│   ├── cartesian_impedance_controller.h  # 笛卡尔阻抗控制器头文件
│   ├── joint_position_controller.h       # 关节位置控制器头文件
│   └── pseudo_inversion.h               # 伪逆工具头文件
├── launch/
│   ├── impedance.launch.py         # ROS2 阻抗控制器启动文件
│   ├── impedance.launch            # ROS1 阻抗控制器启动文件(遗留)
│   ├── joint.launch.py             # ROS2 关节位置控制器启动文件
│   └── joint.launch                # ROS1 关节位置控制器启动文件(遗留)
├── msg/
│   └── ZeroJacobian.msg            # 自定义消息: 零空间雅可比矩阵
├── src/
│   ├── cartesian_impedance_controller.cpp  # 笛卡尔阻抗控制器实现
│   └── joint_position_controller.cpp       # 关节位置控制器实现
├── test/
│   └── test.py                     # ROS1 测试脚本(遗留)
├── CMakeLists.txt
├── package.xml
├── serl_franka_controllers_plugin.xml  # pluginlib 插件描述
├── requirements.txt
├── rosdoc.yaml
├── .gitignore
├── CHANGELOG.rst
├── LICENSE
├── README.md
├── reaemd-ros2.md
└── controller_plot.jpg
```

---

## 二、关键文件完整内容

### 1. [CMakeLists.txt](file:///home/xusj/hil-serl-ros2/ros2_ws/src/serl_franka_controllers/CMakeLists.txt)

```cmake
cmake_minimum_required(VERSION 3.8)
project(serl_franka_controllers)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_typesupport_fastrtps_cpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(franka_semantic_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Franka 0.7.0 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}_msgs
  "msg/ZeroJacobian.msg"
  DEPENDENCIES std_msgs
)

add_library(${PROJECT_NAME} SHARED
  src/joint_position_controller.cpp
  src/cartesian_impedance_controller.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  include
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
  rosidl_typesupport_fastrtps_cpp
  controller_interface
  franka_semantic_components
  geometry_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_srvs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}_msgs "rosidl_typesupport_cpp")
rosidl_get_typesupport_target(fastrtps_cpp_typesupport_target
  ${PROJECT_NAME}_msgs "rosidl_typesupport_fastrtps_cpp")

target_link_libraries(${PROJECT_NAME}
  ${cpp_typesupport_target}
  ${fastrtps_cpp_typesupport_target}
  ${Franka_LIBRARIES}
)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
)

pluginlib_export_plugin_description_file(
  controller_interface serl_franka_controllers_plugin.xml)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rosidl_typesupport_fastrtps_cpp
  controller_interface
  franka_semantic_components
  pluginlib
  rclcpp
  rclcpp_lifecycle
  hardware_interface
)

# 为自定义消息创建符号链接(解决运行时库加载问题)
install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ...)")
# ... 多个符号链接 ...

ament_package()
```

**要点**：
- C++17 标准
- 依赖 `Franka 0.7.0` (libfranka)
- 生成自定义消息 `serl_franka_controllers_msgs`（包含 `ZeroJacobian.msg`）
- 编译为共享库 `libserl_franka_controllers.so`
- 通过 pluginlib 注册为 `controller_interface::ControllerInterface` 插件

---

### 2. [package.xml](file:///home/xusj/hil-serl-ros2/ros2_ws/src/serl_franka_controllers/package.xml)

```xml
<?xml version="1.0"?>
<package format="3">
  <name>serl_franka_controllers</name>
  <version>0.1.1</version>
  <description>serl_franka_controllers provides a compliant yet accurate Cartesian impedance controller for controlling Franka Emika research robots that can be used in online reinforcement learning applications</description>
  <maintainer email="xuc@berkeley.edu">Charles Xu</maintainer>
  <license>MIT</license>
  <url type="website">https://serl-robot.github.io/</url>
  <url type="repository">https://github.com/rail-berkeley/serl_franka_controllers/</url>
  <author>Jianlan Luo, Charles Xu</author>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>

  <depend>controller_interface</depend>
  <depend>franka_hardware</depend>
  <depend>franka_semantic_components</depend>
  <depend>geometry_msgs</depend>
  <depend>hardware_interface</depend>
  <depend>libfranka</depend>
  <depend>pluginlib</depend>
  <depend>rclcpp</depend>
  <depend>rclcpp_lifecycle</depend>
  <depend>realtime_tools</depend>
  <depend>std_msgs</depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

---

### 3. [config/serl_franka_controllers.yaml](file:///home/xusj/hil-serl-ros2/ros2_ws/src/serl_franka_controllers/config/serl_franka_controllers.yaml)

```yaml
/**:
  controller_manager:
    ros__parameters:
      update_rate: 1000
      thread_priority: 98

      joint_state_broadcaster:
        type: joint_state_broadcaster/JointStateBroadcaster

      franka_robot_state_broadcaster:
        type: franka_robot_state_broadcaster/FrankaRobotStateBroadcaster

      joint_position_controller:
        type: serl_franka_controllers/JointPositionController

      cartesian_impedance_controller:
        type: serl_franka_controllers/CartesianImpedanceController

/**:
  franka_robot_state_broadcaster:
    ros__parameters:
      arm_id: "fr3"
      lock_try_count: 5
      lock_sleep_interval: 5
      lock_log_error: true
      lock_update_success: false

/**:
  joint_state_broadcaster:
    ros__parameters:
      joints:
        - fr3_joint1
        - fr3_joint2
        - fr3_joint3
        - fr3_joint4
        - fr3_joint5
        - fr3_joint6
        - fr3_joint7

/**:
  joint_position_controller:
    ros__parameters:
      arm_id: "fr3"
      target_joint_positions: [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
      k_gains: [600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0]
      d_gains: [50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0]

/**:
  cartesian_impedance_controller:
    ros__parameters:
      arm_id: "fr3"
```

**要点**：
- 控制器管理器以 1000Hz 运行，线程优先级 98
- 注册了 4 个控制器：`joint_state_broadcaster`、`franka_robot_state_broadcaster`、`joint_position_controller`、`cartesian_impedance_controller`
- 机械臂 ID 为 `fr3`（Franka Research 3）
- 关节位置控制器的目标位置为 ready 姿态 `[0, -π/4, 0, -3π/4, 0, π/2, π/4]`

---

### 4. [include/serl_franka_controllers/cartesian_impedance_controller.h](file:///home/xusj/hil-serl-ros2/ros2_ws/src/serl_franka_controllers/include/serl_franka_controllers/cartesian_impedance_controller.h)

```cpp
#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <Eigen/Dense>

#include <franka_semantic_components/franka_robot_model.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <serl_franka_controllers/msg/zero_jacobian.hpp>

namespace serl_franka_controllers {

class CartesianImpedanceController : public controller_interface::ControllerInterface {
 public:
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init() override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  void updateJointStates();
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);

  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;

  std::string arm_id_;
  static constexpr int num_joints_ = 7;

  std::array<double, 7> q_{};
  std::array<double, 7> dq_{};
  std::array<double, 7> tau_J_d_last_{};

  double filter_params_{0.005};
  double nullspace_stiffness_{20.0};
  double nullspace_stiffness_target_{20.0};
  double joint1_nullspace_stiffness_{20.0};
  double joint1_nullspace_stiffness_target_{20.0};
  const double delta_tau_max_{1.0};
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
  Eigen::Matrix<double, 6, 6> Ki_;
  Eigen::Matrix<double, 6, 6> Ki_target_;

  Eigen::Matrix<double, 3, 1> translational_clip_min_;
  Eigen::Matrix<double, 3, 1> translational_clip_max_;
  Eigen::Matrix<double, 3, 1> rotational_clip_min_;
  Eigen::Matrix<double, 3, 1> rotational_clip_max_;
  Eigen::Matrix<double, 7, 1> q_d_nullspace_;
  Eigen::Vector3d position_d_;
  Eigen::Matrix<double, 6, 1> error_;
  Eigen::Matrix<double, 6, 1> error_i;
  Eigen::Quaterniond orientation_d_;
  Eigen::Vector3d position_d_target_;
  Eigen::Quaterniond orientation_d_target_;

  std::shared_ptr<rclcpp::Publisher<serl_franka_controllers::msg::ZeroJacobian>> publisher_franka_jacobian_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_equilibrium_pose_;
  void equilibriumPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
};

}  // namespace serl_franka_controllers
```

---

### 5. [src/cartesian_impedance_controller.cpp](file:///home/xusj/hil-serl-ros2/ros2_ws/src/serl_franka_controllers/src/cartesian_impedance_controller.cpp)

```cpp
#include <serl_franka_controllers/cartesian_impedance_controller.h>

#include <cmath>
#include <memory>
#include <string>

#include <hardware_interface/loaned_command_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <franka/robot_state.h>
#include <serl_franka_controllers/pseudo_inversion.h>

namespace serl_franka_controllers {

// 命令接口: 7个关节的 effort (力矩)
controller_interface::InterfaceConfiguration
CartesianImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints_; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

// 状态接口: 7个关节的 position + velocity + FrankaRobotModel 的所有状态接口
controller_interface::InterfaceConfiguration
CartesianImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints_; ++i) {
    state_interfaces_config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
  }
  for (int i = 1; i <= num_joints_; ++i) {
    state_interfaces_config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  for (const auto& franka_robot_model_name : franka_robot_model_->get_state_interface_names()) {
    state_interfaces_config.names.push_back(franka_robot_model_name);
  }
  return state_interfaces_config;
}

// on_init: 声明参数
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianImpedanceController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "fr3");
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

// on_configure: 初始化模型、发布器、订阅器
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianImpedanceController::on_configure(const rclcpp_lifecycle::State&) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();

  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
      arm_id_ + "/robot_model", arm_id_ + "/robot_state");

  publisher_franka_jacobian_ =
      get_node()->create_publisher<serl_franka_controllers::msg::ZeroJacobian>(
          "franka_jacobian", rclcpp::SystemDefaultsQoS());

  sub_equilibrium_pose_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
      "equilibrium_pose", rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        equilibriumPoseCallback(msg);
      });

  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  position_d_target_.setZero();
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();

  translational_clip_min_.setConstant(-std::numeric_limits<double>::max());
  translational_clip_max_.setConstant(std::numeric_limits<double>::max());
  rotational_clip_min_.setConstant(-std::numeric_limits<double>::max());
  rotational_clip_max_.setConstant(std::numeric_limits<double>::max());

  return CallbackReturn::SUCCESS;
}

// on_activate: 分配状态接口，读取初始位姿，设置初始刚度和阻尼
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianImpedanceController::on_activate(const rclcpp_lifecycle::State&) {
  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);
  updateJointStates();

  auto pose_matrix = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(pose_matrix.data()));

  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
  position_d_target_ = initial_transform.translation();
  orientation_d_target_ = Eigen::Quaterniond(initial_transform.linear());

  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(q_.data());
  q_d_nullspace_ = q_initial;
  error_i.setZero();

  // 平移刚度 2000, 旋转刚度 150
  cartesian_stiffness_.setIdentity();
  cartesian_stiffness_.topLeftCorner(3, 3) << 2000.0 * Eigen::Matrix3d::Identity();
  cartesian_stiffness_.bottomRightCorner(3, 3) << 150.0 * Eigen::Matrix3d::Identity();
  // 平移阻尼 89, 旋转阻尼 7
  cartesian_damping_.setIdentity();
  cartesian_damping_.topLeftCorner(3, 3) << 89.0 * Eigen::Matrix3d::Identity();
  cartesian_damping_.bottomRightCorner(3, 3) << 7.0 * Eigen::Matrix3d::Identity();

  cartesian_stiffness_target_ = cartesian_stiffness_;
  cartesian_damping_target_ = cartesian_damping_;
  tau_J_d_last_.fill(0.0);

  return CallbackReturn::SUCCESS;
}

// on_deactivate: 释放接口
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianImpedanceController::on_deactivate(const rclcpp_lifecycle::State&) {
  franka_robot_model_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

void CartesianImpedanceController::updateJointStates() {
  for (auto i = 0; i < num_joints_; ++i) {
    q_.at(i) = state_interfaces_[i].get_value();
    dq_.at(i) = state_interfaces_[num_joints_ + i].get_value();
  }
}

// 核心 update 循环: 阻抗控制 + 零空间控制 + 科里奥利补偿
controller_interface::return_type CartesianImpedanceController::update(
    const rclcpp::Time&, const rclcpp::Duration&) {
  updateJointStates();

  // 从 FrankaRobotModel 获取雅可比、科里奥利力、末端位姿
  std::array<double, 42> jacobian_array =
      franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);
  std::array<double, 7> coriolis_array = franka_robot_model_->getCoriolisForceVector();
  std::array<double, 16> o_t_ee_array = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);

  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());

  Eigen::Affine3d transform(Eigen::Matrix4d::Map(o_t_ee_array.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());

  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(q_.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(dq_.data());

  // 计算笛卡尔误差 (位置 + 姿态)
  error_.head(3) << position - position_d_;
  for (int i = 0; i < 3; i++) {
    error_(i) = std::min(std::max(error_(i), translational_clip_min_(i)), translational_clip_max_(i));
  }

  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  error_.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  error_.tail(3) << -transform.linear() * error_.tail(3);
  for (int i = 0; i < 3; i++) {
    error_(i + 3) = std::min(std::max(error_(i + 3), rotational_clip_min_(i)), rotational_clip_max_(i));
  }

  // 积分误差 (带限幅)
  error_i.head(3) << (error_i.head(3) + error_.head(3)).cwiseMax(-0.1).cwiseMin(0.1);
  error_i.tail(3) << (error_i.tail(3) + error_.tail(3)).cwiseMax(-0.3).cwiseMin(0.3);

  Eigen::Matrix<double, 7, 1> tau_task, tau_nullspace, tau_d;

  // 雅可比转置伪逆
  Eigen::Matrix<double, 6, 7> jacobian_transpose_pinv;
  Eigen::Matrix<double, 7, 6> jacobian_T = jacobian.transpose();
  pseudoInverse(jacobian_T, jacobian_transpose_pinv);

  // 笛卡尔任务力矩: tau_task = J^T * (-K_x * e - D_x * (J * dq) - Ki * ei)
  tau_task << jacobian.transpose() *
                  (-cartesian_stiffness_ * error_ - cartesian_damping_ * (jacobian * dq) -
                   Ki_ * error_i);

  // 零空间力矩: 关节1有独立刚度
  Eigen::Matrix<double, 7, 1> dqe;
  Eigen::Matrix<double, 7, 1> qe;
  qe << q_d_nullspace_ - q;
  qe.head(1) << qe.head(1) * joint1_nullspace_stiffness_;
  dqe << dq;
  dqe.head(1) << dqe.head(1) * 2.0 * sqrt(joint1_nullspace_stiffness_);
  tau_nullspace << (Eigen::Matrix<double, 7, 7>::Identity() -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                       (nullspace_stiffness_ * qe - (2.0 * sqrt(nullspace_stiffness_)) * dqe);

  // 总力矩 = 任务力矩 + 零空间力矩 + 科里奥利补偿
  tau_d << tau_task + tau_nullspace + coriolis;

  // 力矩变化率限幅
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d_last(tau_J_d_last_.data());
  tau_d << saturateTorqueRate(tau_d, tau_J_d_last);

  // 写入命令接口
  for (size_t i = 0; i < 7; ++i) {
    command_interfaces_[i].set_value(tau_d(i));
    tau_J_d_last_[i] = tau_d(i);
  }

  // 低通滤波平滑参数更新
  cartesian_stiffness_ = filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
  cartesian_damping_ = filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
  nullspace_stiffness_ = filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
  joint1_nullspace_stiffness_ = filter_params_ * joint1_nullspace_stiffness_target_ + (1.0 - filter_params_) * joint1_nullspace_stiffness_;
  position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);
  Ki_ = filter_params_ * Ki_target_ + (1.0 - filter_params_) * Ki_;

  return controller_interface::return_type::OK;
}

// 力矩变化率限幅: 每步最大变化 delta_tau_max_ = 1.0 Nm
Eigen::Matrix<double, 7, 1> CartesianImpedanceController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

// 平衡位姿回调: 接收新的目标位姿，重置积分误差
void CartesianImpedanceController::equilibriumPoseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  error_i.setZero();
  Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
  orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w;
  if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
    orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  }
}

}  // namespace serl_franka_controllers

PLUGINLIB_EXPORT_CLASS(serl_franka_controllers::CartesianImpedanceController,
                       controller_interface::ControllerInterface)
```

---

### 6. [include/serl_franka_controllers/joint_position_controller.h](file:///home/xusj/hil-serl-ros2/ros2_ws/src/serl_franka_controllers/include/serl_franka_controllers/joint_position_controller.h)

```cpp
#pragma once

#include <array>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace serl_franka_controllers {

class JointPositionController : public controller_interface::ControllerInterface {
 public:
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  std::string arm_id_;
  const int num_joints_ = 7;
  const double motion_duration_ = 10.0;
  rclcpp::Duration elapsed_time_ = rclcpp::Duration(0, 0);
  std::array<double, 7> initial_pose_{};
  std::array<double, 7> target_pose_{};
  std::array<double, 7> k_gains_{};
  std::array<double, 7> d_gains_{};
};

}  // namespace serl_franka_controllers
```

---

### 7. [src/joint_position_controller.cpp](file:///home/xusj/hil-serl-ros2/ros2_ws/src/serl_franka_controllers/src/joint_position_controller.cpp)

```cpp
#include <serl_franka_controllers/joint_position_controller.h>

#include <cmath>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace serl_franka_controllers {

// 命令接口: 7个关节的 effort
controller_interface::InterfaceConfiguration
JointPositionController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints_; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

// 状态接口: 7个关节的 position + velocity
controller_interface::InterfaceConfiguration
JointPositionController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints_; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
  }
  for (int i = 1; i <= num_joints_; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

CallbackReturn JointPositionController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "panda");
    auto_declare<std::vector<double>>("target_joint_positions", {});
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointPositionController::on_configure(
    const rclcpp_lifecycle::State&) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  auto target_positions = get_node()->get_parameter("target_joint_positions").as_double_array();
  // ... 参数校验和赋值 ...
  auto k = get_node()->get_parameter("k_gains").as_double_array();
  auto d = get_node()->get_parameter("d_gains").as_double_array();
  // ... PD增益赋值 ...
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointPositionController::on_activate(
    const rclcpp_lifecycle::State&) {
  for (size_t i = 0; i < static_cast<size_t>(num_joints_); ++i) {
    initial_pose_[i] = state_interfaces_[i].get_value();
  }
  elapsed_time_ = rclcpp::Duration(0, 0);
  return CallbackReturn::SUCCESS;
}

// 核心 update: 五次多项式轨迹插值 + PD控制
controller_interface::return_type JointPositionController::update(
    const rclcpp::Time&, const rclcpp::Duration& period) {
  elapsed_time_ = elapsed_time_ + period;
  double t = elapsed_time_.seconds() / motion_duration_;

  for (size_t i = 0; i < static_cast<size_t>(num_joints_); ++i) {
    double q = state_interfaces_[i].get_value();
    double dq = state_interfaces_[num_joints_ + i].get_value();

    // 五次多项式插值: s(t) = 6t^5 - 15t^4 + 10t^3
    double s;
    if (t >= 1.0) { s = 1.0; }
    else { s = 6.0 * std::pow(t, 5) - 15.0 * std::pow(t, 4) + 10.0 * std::pow(t, 3); }

    double q_d = initial_pose_[i] + s * (target_pose_[i] - initial_pose_[i]);
    double dq_d = 0.0;
    if (t < 1.0) {
      dq_d = (30.0 * std::pow(t, 4) - 60.0 * std::pow(t, 3) + 30.0 * std::pow(t, 2)) *
             (target_pose_[i] - initial_pose_[i]) / motion_duration_;
    }

    // PD力矩控制
    double tau = k_gains_[i] * (q_d - q) + d_gains_[i] * (dq_d - dq);
    command_interfaces_[i].set_value(tau);
  }
  return controller_interface::return_type::OK;
}

}  // namespace serl_franka_controllers

PLUGINLIB_EXPORT_CLASS(serl_franka_controllers::JointPositionController,
                       controller_interface::ControllerInterface)
```

---

### 8. [include/serl_franka_controllers/pseudo_inversion.h](file:///home/xusj/hil-serl-ros2/ros2_ws/src/serl_franka_controllers/include/serl_franka_controllers/pseudo_inversion.h)

```cpp
#pragma once

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>

namespace serl_franka_controllers {

// 通用矩阵伪逆 (阻尼最小二乘)
inline void pseudoInverse(const Eigen::MatrixXd& M_, Eigen::MatrixXd& M_pinv_, bool damped = true) {
  double lambda_ = damped ? 0.2 : 0.0;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M_, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();
  Eigen::MatrixXd S_ = M_;
  S_.setZero();
  for (int i = 0; i < sing_vals_.size(); i++)
    S_(i, i) = (sing_vals_(i)) / (sing_vals_(i) * sing_vals_(i) + lambda_ * lambda_);
  M_pinv_ = Eigen::MatrixXd(svd.matrixV() * S_.transpose() * svd.matrixU().transpose());
}

// 7x6 矩阵伪逆特化版本 (用于雅可比转置伪逆)
inline void pseudoInverse(const Eigen::Matrix<double, 7, 6>& M_,
                          Eigen::Matrix<double, 6, 7>& M_pinv_,
                          bool damped = true) {
  double lambda_ = damped ? 0.2 : 0.0;
  Eigen::JacobiSVD<Eigen::Matrix<double, 7, 6>> svd(M_, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix<double, 6, 1> sing_vals_ = svd.singularValues();
  Eigen::Matrix<double, 7, 6> S_ = Eigen::Matrix<double, 7, 6>::Zero();
  for (int i = 0; i < 6; i++)
    S_(i, i) = (sing_vals_(i)) / (sing_vals_(i) * sing_vals_(i) + lambda_ * lambda_);
  M_pinv_ = svd.matrixV() * S_.transpose() * svd.matrixU().transpose();
}

}  // namespace serl_franka_controllers
```

---

### 9. [msg/ZeroJacobian.msg](file:///home/xusj/hil-serl-ros2/ros2_ws/src/serl_franka_controllers/msg/ZeroJacobian.msg)

```
float64[42] zero_jacobian
```

42 个 double，即 6x7 雅可比矩阵的展平表示。

---

### 10. [serl_franka_controllers_plugin.xml](file:///home/xusj/hil-serl-ros2/ros2_ws/src/serl_franka_controllers/serl_franka_controllers_plugin.xml)

```xml
<library path="serl_franka_controllers">
  <class name="serl_franka_controllers/JointPositionController"
         type="serl_franka_controllers::JointPositionController"
         base_class_type="controller_interface::ControllerInterface">
    <description>
      A controller that executes a short motion based on joint positions to demonstrate correct usage
    </description>
  </class>
  <class name="serl_franka_controllers/CartesianImpedanceController"
         type="serl_franka_controllers::CartesianImpedanceController"
         base_class_type="controller_interface::ControllerInterface">
    <description>
      A controller that renders a spring damper system in cartesian space.
    </description>
  </class>
</library>
```

---

### 11. [launch/impedance.launch.py](file:///home/xusj/hil-serl-ros2/ros2_ws/src/serl_franka_controllers/launch/impedance.launch.py)

启动参数：
- `arm_id` (默认 `fr3`)
- `namespace` (默认空)
- `urdf_file` (默认 `fr3/fr3.urdf.xacro`)
- `robot_ip` (默认 `172.16.0.3`)
- `load_gripper` (默认 `true`)
- `use_fake_hardware` (默认 `false`)

启动节点：
1. `robot_state_publisher` - 发布机器人状态
2. `ros2_control_node` - 控制器管理器，加载 yaml 配置
3. `joint_state_publisher` - 聚合关节状态
4. `spawner joint_state_broadcaster` - 启动关节状态广播器
5. `spawner franka_robot_state_broadcaster` - 启动 Franka 机器人状态广播器
6. `spawner cartesian_impedance_controller` - 启动笛卡尔阻抗控制器
7. (条件) `franka_gripper` - 启动夹爪控制器

### 12. [launch/joint.launch.py](file:///home/xusj/hil-serl-ros2/ros2_ws/src/serl_franka_controllers/launch/joint.launch.py)

与 impedance.launch.py 结构几乎相同，唯一区别是第 6 步启动的是 `joint_position_controller` 而非 `cartesian_impedance_controller`。

### 13. 遗留的 ROS1 文件

- [launch/impedance.launch](file:///home/xusj/hil-serl-ros2/ros2_ws/src/serl_franka_controllers/launch/impedance.launch) 和 [launch/joint.launch](file:///home/xusj/hil-serl-ros2/ros2_ws/src/serl_franka_controllers/launch/joint.launch)：ROS1 XML 格式启动文件，使用 `franka_control` 包
- [cfg/compliance_param.cfg](file:///home/xusj/hil-serl-ros2/ros2_ws/src/serl_franka_controllers/cfg/compliance_param.cfg)：ROS1 `dynamic_reconfigure` 配置生成器
- [test/test.py](file:///home/xusj/hil-serl-ros2/ros2_ws/src/serl_franka_controllers/test/test.py)：ROS1 测试脚本，使用 `rospy` 和 `dynamic_reconfigure`

---

## 三、使用的 franka_ros2 接口和控制器总结

### 直接依赖的 franka_ros2 组件

| 组件 | 来源包 | 用途 |
|------|--------|------|
| `franka_semantic_components::FrankaRobotModel` | `franka_semantic_components` | 获取机器人运动学模型（雅可比、科里奥利力、末端位姿） |
| `franka_hardware` | `franka_hardware` | 硬件接口层，提供 state/command interfaces |
| `franka_msgs` | `franka_msgs` | Franka 消息定义 |
| `FrankaRobotStateBroadcaster` | `franka_robot_state_broadcaster` | 广播 Franka 机器人状态 |
| `franka_description` | `franka_description` | URDF/Xacro 机器人描述 |
| `franka_gripper` | `franka_gripper` | 夹爪控制 |
| `franka::Frame::kEndEffector` | `libfranka` | 指定末端执行器坐标系 |
| `franka::robot_state` | `libfranka` | 机器人状态数据结构 |

### FrankaRobotModel 使用的具体接口

在 `CartesianImpedanceController` 中，`FrankaRobotModel` 提供了以下关键方法：

1. **`getZeroJacobian(franka::Frame::kEndEffector)`** - 获取末端执行器的零空间雅可比矩阵 (6x7)
2. **`getCoriolisForceVector()`** - 获取科里奥利力向量 (7x1)
3. **`getPoseMatrix(franka::Frame::kEndEffector)`** - 获取末端执行器的齐次变换矩阵 (4x4)
4. **`get_state_interface_names()`** - 获取模型所需的所有状态接口名称
5. **`assign_loaned_state_interfaces()`** - 分配借用的状态接口
6. **`release_interfaces()`** - 释放接口

### 硬件接口使用

| 接口类型 | 名称模式 | 方向 | 使用者 |
|----------|----------|------|--------|
| `position` | `{arm_id}_joint{i}/position` | State (读) | 两个控制器 |
| `velocity` | `{arm_id}_joint{i}/velocity` | State (读) | 两个控制器 |
| `effort` | `{arm_id}_joint{i}/effort` | Command (写) | 两个控制器 |
| FrankaRobotModel 的状态接口 | `{arm_id}/robot_model` 等 | State (读) | 仅笛卡尔阻抗控制器 |

---

## 四、代码架构总结

### 整体设计

该项目是 SERL (Scalable and Efficient Robot Learning) 框架的一部分，为 Franka Research 3 (FR3) 机器人提供两个自定义 ros2_control 控制器：

1. **CartesianImpedanceController（笛卡尔阻抗控制器）** - 核心控制器，用于强化学习在线训练
   - 实现 6 自由度笛卡尔空间阻抗控制（刚度-阻尼系统）
   - 支持 PID 控制（含积分项 Ki）
   - 零空间控制保持关节构型（关节1有独立刚度）
   - 科里奥利力前馈补偿
   - 力矩变化率限幅（安全保护，delta_tau_max = 1.0 Nm）
   - 低通滤波平滑参数更新（filter_params = 0.005）
   - 通过 `equilibrium_pose` 话题接收目标位姿
   - 通过 `franka_jacobian` 话题发布雅可比矩阵

2. **JointPositionController（关节位置控制器）** - 辅助控制器，用于初始化/复位
   - 五次多项式轨迹插值（保证位置、速度、加速度连续）
   - PD 力矩控制跟踪插值轨迹
   - 运动持续时间固定 10 秒
   - 一次性运动：从当前位置到目标位置

### 控制律公式

**笛卡尔阻抗控制器**：

```
tau_d = J^T * (-K_x * e - D_x * (J * dq) - Ki * ei) + (I - J^T * J^T+) * (K_n * (q_d_null - q) - D_n * dq) + c(q, dq)
```

其中：
- `J` = 雅可比矩阵，`J^T+` = 雅可比转置的伪逆
- `K_x` = 笛卡尔刚度 (平移 2000, 旋转 150)
- `D_x` = 笛卡尔阻尼 (平移 89, 旋转 7)
- `Ki` = 笛卡尔积分增益
- `K_n` = 零空间刚度 (20, 关节1独立)
- `c(q, dq)` = 科里奥利力

**关节位置控制器**：

```
s(t) = 6t^5 - 15t^4 + 10t^3    (五次多项式)
q_d(t) = q_init + s(t) * (q_target - q_init)
tau = K_p * (q_d - q) + K_d * (dq_d - dq)
```

### 数据流

```
franka_hardware (硬件接口层)
    |
    ├── state_interfaces: joint position, velocity, robot_model states
    ├── command_interfaces: joint effort
    |
    ▼
CartesianImpedanceController (1000Hz)
    ├── 读取: joint states + FrankaRobotModel (Jacobian, Coriolis, Pose)
    ├── 订阅: /equilibrium_pose (geometry_msgs/PoseStamped)
    ├── 发布: /franka_jacobian (ZeroJacobian)
    └── 写入: joint effort commands
    
JointPositionController (1000Hz)
    ├── 读取: joint position + velocity
    ├── 参数: target_joint_positions, k_gains, d_gains
    └── 写入: joint effort commands
```
        

## 一、Franka 相关包总览

项目中共存在 **两大来源** 的 Franka 包：

| 来源 | 包名 | 路径 |
|------|------|------|
| 官方 franka_ros2 | franka_hardware | `franka_ros2/franka_hardware/` |
| 官方 franka_ros2 | franka_msgs | `franka_ros2/franka_msgs/` |
| 官方 franka_ros2 | franka_gripper | `franka_ros2/franka_gripper/` |
| 官方 franka_ros2 | franka_description | `franka_ros2/franka_description/` |
| 官方 franka_ros2 | franka_bringup | `franka_ros2/franka_bringup/` |
| 官方 franka_ros2 | franka_example_controllers | `franka_ros2/franka_example_controllers/` |
| 官方 franka_ros2 | franka_semantic_components | `franka_ros2/franka_semantic_components/` |
| 官方 franka_ros2 | franka_robot_state_broadcaster | `franka_ros2/franka_robot_state_broadcaster/` |
| 官方 franka_ros2 | franka_fr3_moveit_config | `franka_ros2/franka_fr3_moveit_config/` |
| 官方 franka_ros2 | franka_gazebo | `franka_ros2/franka_gazebo/` |
| 官方 franka_ros2 | franka_ros2 (元包) | `franka_ros2/franka_ros2/` |
| 自定义 | serl_franka_controllers | `serl_franka_controllers/` |
| 自定义 | hil_ros2_check | `hil_ros2_check/` |

---

## 二、franka_hardware 硬件接口定义

### 2.1 硬件接口插件

- **插件名**: `franka_hardware/FrankaHardwareInterface`
- **类型**: `franka_hardware::FrankaHardwareInterface`
- **基类**: `hardware_interface::SystemInterface`
- **定义文件**: [franka_hardware.xml](file:///home/xusj/hil-serl-ros2/ros2_ws/src/franka_ros2/franka_hardware/franka_hardware.xml)

### 2.2 命令接口 (Command Interfaces)

在 [franka_hardware_interface.hpp](file:///home/xusj/hil-serl-ros2/ros2_ws/src/franka_ros2/franka_hardware/include/franka_hardware/franka_hardware_interface.hpp) 中定义了以下6类命令接口：

| 接口类型 | 接口名称 | 维度 | 说明 |
|----------|----------|------|------|
| 关节力矩 | `effort` (HW_IF_EFFORT) | 7 | 关节力矩控制 |
| 关节位置 | `position` (HW_IF_POSITION) | 7 | 关节位置控制 |
| 关节速度 | `velocity` (HW_IF_VELOCITY) | 7 | 关节速度控制 |
| 笛卡尔速度 | `cartesian_velocity` | 6 | [vx,vy,vz,wx,wy,wz] |
| 笛卡尔位姿 | `cartesian_pose_command` | 16 | 4x4齐次变换矩阵(列主序) |
| 肘部命令 | `elbow_command` | 2 | [joint3_position, joint4_sign] |

### 2.3 状态接口 (State Interfaces)

在 [franka_hardware_interface.cpp](file:///home/xusj/hil-serl-ros2/ros2_ws/src/franka_ros2/franka_hardware/src/franka_hardware_interface.cpp#L93-L128) 中导出的状态接口：

| 接口类型 | 接口名称 | 说明 |
|----------|----------|------|
| 关节位置 | `position` (每个关节) | 7个关节 |
| 关节速度 | `velocity` (每个关节) | 7个关节 |
| 关节力矩 | `effort` (每个关节) | 7个关节 |
| 机器人状态 | `robot_state` | franka::RobotState 指针 |
| 机器人模型 | `robot_model` | Model 指针 |
| 笛卡尔位姿状态 | `cartesian_pose_state` | 16元素(0-15) |
| 肘部状态 | `elbow_state` | [joint_3_position, joint_4_sign] |
| 机器人时间 | `robot_time` | 机器人运行时间 |

### 2.4 URDF ros2_control 定义

在 [franka_arm.ros2_control.xacro](file:///home/xusj/hil-serl-ros2/ros2_ws/src/franka_ros2/franka_description/robots/common/franka_arm.ros2_control.xacro) 中，每个关节注册了3个命令接口 (`position`, `velocity`, `effort`) 和3个状态接口 (`position`, `velocity`, `effort`)。GPIO 方式注册了笛卡尔速度(6个)、笛卡尔位姿(16个)和肘部命令(2个)。

### 2.5 Robot 类支持的控制模式

在 [robot.hpp](file:///home/xusj/hil-serl-ros2/ros2_ws/src/franka_ros2/franka_hardware/include/franka_hardware/robot.hpp) 中定义了5种控制模式初始化方法：

- `initializeTorqueInterface()` -- 力矩控制
- `initializeJointVelocityInterface()` -- 关节速度控制
- `initializeJointPositionInterface()` -- 关节位置控制
- `initializeCartesianVelocityInterface()` -- 笛卡尔速度控制
- `initializeCartesianPoseInterface()` -- 笛卡尔位姿控制

---

## 三、franka_msgs 消息/Action/Service 定义

### 3.1 Action 定义

定义在 [franka_msgs/action/](file:///home/xusj/hil-serl-ros2/ros2_ws/src/franka_ros2/franka_msgs/action/) 目录下：

#### (1) Grasp.action
```
# Goal
float64 width          # 目标宽度 [m]
GraspEpsilon epsilon   # 抓取容差
float64 speed          # 速度 [m/s]
float64 force          # 力 [N]
---
# Result
bool success
string error
---
# Feedback
float64 current_width  # 当前宽度 [m]
```

#### (2) Move.action
```
# Goal
float64 width          # 目标宽度 [m]
float64 speed          # 速度 [m/s]
---
# Result
bool success
string error
---
# Feedback
float64 current_width  # 当前宽度 [m]
```

#### (3) Homing.action
```
# Goal (空)
---
# Result
bool success
string error
---
# Feedback
float64 current_width  # 当前宽度 [m]
```

#### (4) ErrorRecovery.action
```
# Goal (空)
---
# Result (空)
---
# Feedback (空)
```

**Action 话题名** (在 franka_hardware 的 ActionServer 中注册):
- `franka_msgs/action/ErrorRecovery` -- 由 `franka_hardware::ActionServer` 节点提供服务

**Action 话题名** (在 franka_gripper 的 GripperActionServer 中注册):
- `franka_msgs/action/Homing` -- 夹爪归位
- `franka_msgs/action/Move` -- 夹爪移动
- `franka_msgs/action/Grasp` -- 夹爪抓取
- `control_msgs/action/GripperCommand` -- MoveIt 兼容的夹爪命令

### 3.2 消息定义 (Messages)

定义在 [franka_msgs/msg/](file:///home/xusj/hil-serl-ros2/ros2_ws/src/franka_ros2/franka_msgs/msg/) 目录下：

| 消息类型 | 字段 | 说明 |
|----------|------|------|
| `FrankaRobotState` | header, collision_indicators, measured_joint_state, desired_joint_state, measured_joint_motor_state, ddq_d[7], dtau_j[7], tau_ext_hat_filtered, elbow, k_f_ext_hat_k, o_f_ext_hat_k, inertia_ee/load/total, o_t_ee/o_t_ee_d/o_t_ee_c/f_t_ee/ee_t_k, o_dp_ee_d/o_dp_ee_c/o_ddp_ee_c, time, control_command_success_rate, robot_mode, current_errors, last_motion_errors | 完整机器人状态 |
| `Errors` | 36个bool字段 | 各种错误标志(关节限位、笛卡尔限位、自碰撞、速度违例等) |
| `Elbow` | position[2], desired_position[2], commanded_position[2], commanded_velocity[2], commanded_acceleration[2] | 肘部状态 |
| `GraspEpsilon` | inner (默认0.005m), outer (默认0.005m) | 抓取容差 |
| `CollisionIndicators` | is_cartesian_linear_collision, is_cartesian_angular_collision, is_cartesian_linear_contact, is_cartesian_angular_contact, is_joint_collision[7], is_joint_contact[7] | 碰撞/接触指示器 |

**FrankaRobotState 中的 robot_mode 枚举值**:
- 0=OTHER, 1=IDLE, 2=MOVE, 3=GUIDING, 4=REFLEX, 5=USER_STOPPED, 6=AUTOMATIC_ERROR_RECOVERY

### 3.3 服务定义 (Services)

定义在 [franka_msgs/srv/](file:///home/xusj/hil-serl-ros2/ros2_ws/src/franka_ros2/franka_msgs/srv/) 目录下：

| 服务类型 | 请求字段 | 响应字段 | 说明 |
|----------|----------|----------|------|
| `SetJointStiffness` | joint_stiffness[7] | success, error | 设置关节阻抗刚度 |
| `SetCartesianStiffness` | cartesian_stiffness[6] (x,y,z,roll,pitch,yaw) | success, error | 设置笛卡尔刚度 |
| `SetLoad` | mass, center_of_mass[3], load_inertia[9] | success, error | 设置负载参数 |
| `SetTCPFrame` | transformation[16] | success, error | 设置TCP坐标系(4x4列主序矩阵) |
| `SetStiffnessFrame` | transformation[16] | success, error | 设置刚度坐标系(4x4列主序矩阵) |
| `SetForceTorqueCollisionBehavior` | lower_torque_thresholds_nominal[7], upper_torque_thresholds_nominal[7], lower_force_thresholds_nominal[6], upper_force_thresholds_nominal[6] | success, error | 设置力/力矩碰撞行为 |
| `SetFullCollisionBehavior` | lower/upper_torque_thresholds_acceleration[7], lower/upper_torque_thresholds_nominal[7], lower/upper_force_thresholds_acceleration[6], lower/upper_force_thresholds_nominal[6] | success, error | 设置完整碰撞行为 |

这些服务由 `franka_hardware::FrankaParamServiceServer` 节点提供。

---

## 四、Gripper 相关控制器和 Action 接口

### 4.1 Gripper Action Server

- **节点类**: `franka_gripper::GripperActionServer`
- **定义文件**: [gripper_action_server.hpp](file:///home/xusj/hil-serl-ros2/ros2_ws/src/franka_ros2/franka_gripper/include/franka_gripper/gripper_action_server.hpp)
- **可执行文件**: `franka_gripper_node`
- **Launch 文件**: [gripper.launch.py](file:///home/xusj/hil-serl-ros2/ros2_ws/src/franka_ros2/franka_gripper/launch/gripper.launch.py)

提供的 Action Server：

| Action 类型 | 话题名 | 说明 |
|-------------|--------|------|
| `franka_msgs::action::Homing` | 默认由节点名空间决定 | 夹爪归位(完全打开再关闭) |
| `franka_msgs::action::Move` | 同上 | 夹爪移动到指定宽度 |
| `franka_msgs::action::Grasp` | 同上 | 夹爪以指定力抓取 |
| `control_msgs::action::GripperCommand` | 同上 | MoveIt 兼容接口 |

还提供的服务：
- `std_srvs::srv::Trigger` -- 停止夹爪运动

配置参数（[franka_gripper_node.yaml](file:///home/xusj/hil-serl-ros2/ros2_ws/src/franka_ros2/franka_gripper/config/franka_gripper_node.yaml)）：
- `state_publish_rate`: 15 Hz
- `feedback_publish_rate`: 30 Hz
- `default_speed`: 0.1 m/s
- `default_grasp_epsilon.inner`: 0.005 m
- `default_grasp_epsilon.outer`: 0.005 m

### 4.2 Gripper Example Controller

- **插件名**: `franka_example_controllers/GripperExampleController`
- **类型**: `franka_example_controllers::GripperExampleController`
- **说明**: 演示如何使用夹爪 Action 接口

---

## 五、Cartesian Impedance 控制器

### 5.1 自定义 serl_franka_controllers/CartesianImpedanceController

- **插件名**: `serl_franka_controllers/CartesianImpedanceController`
- **类型**: `serl_franka_controllers::CartesianImpedanceController`
- **基类**: `controller_interface::ControllerInterface`
- **定义文件**: [cartesian_impedance_controller.h](file:///home/xusj/hil-serl-ros2/ros2_ws/src/serl_franka_controllers/include/serl_franka_controllers/cartesian_impedance_controller.h)
- **实现文件**: [cartesian_impedance_controller.cpp](file:///home/xusj/hil-serl-ros2/ros2_ws/src/serl_franka_controllers/src/cartesian_impedance_controller.cpp)
- **插件注册**: [serl_franka_controllers_plugin.xml](file:///home/xusj/hil-serl-ros2/ros2_ws/src/serl_franka_controllers/serl_franka_controllers_plugin.xml)

**命令接口**: `effort` (7个关节力矩) -- 即 `{arm_id}_joint1/effort` 到 `{arm_id}_joint7/effort`

**状态接口**:
- `{arm_id}_joint{1-7}/position` (7个关节位置)
- `{arm_id}_joint{1-7}/velocity` (7个关节速度)
- `{arm_id}/robot_model` 和 `{arm_id}/robot_state` (通过 FrankaRobotModel 语义组件)

**订阅话题**:
- `equilibrium_pose` (`geometry_msgs/msg/PoseStamped`) -- 设置平衡位姿目标

**发布话题**:
- `franka_jacobian` (`serl_franka_controllers/msg/ZeroJacobian`) -- 发布零空间雅可比矩阵

**自定义消息** ([ZeroJacobian.msg](file:///home/xusj/hil-serl-ros2/ros2_ws/src/serl_franka_controllers/msg/ZeroJacobian.msg)):
```
float64[42] zero_jacobian
```

**控制参数** (在 [serl_franka_controllers.yaml](file:///home/xusj/hil-serl-ros2/ros2_ws/src/serl_franka_controllers/config/serl_franka_controllers.yaml) 中):
- `arm_id`: "fr3"
- 默认笛卡尔刚度: 平移 2000.0, 旋转 150.0
- 默认笛卡尔阻尼: 平移 89.0, 旋转 7.0
- 零空间刚度: 20.0
- 关节1零空间刚度: 20.0
- 滤波参数: 0.005
- 力矩变化率限制: 1.0 Nm/ms

**Launch 文件**: [impedance.launch.py](file:///home/xusj/hil-serl-ros2/ros2_ws/src/serl_franka_controllers/launch/impedance.launch.py)
- 默认加载 `cartesian_impedance_controller`
- 可选加载 gripper (`load_gripper` 参数)

### 5.2 官方示例控制器 (franka_example_controllers)

与笛卡尔控制相关的示例控制器：

| 插件名 | 类型 | 命令接口 | 说明 |
|--------|------|----------|------|
| `franka_example_controllers/CartesianPoseExampleController` | CartesianPoseExampleController | cartesian_pose_command | 笛卡尔位姿控制 |
| `franka_example_controllers/CartesianVelocityExampleController` | CartesianVelocityExampleController | cartesian_velocity | 笛卡尔速度控制 |
| `franka_example_controllers/CartesianElbowExampleController` | CartesianElbowExampleController | cartesian_pose_command + elbow_command | 笛卡尔位姿+肘部控制 |
| `franka_example_controllers/CartesianOrientationExampleController` | CartesianOrientationExampleController | cartesian_pose_command | 笛卡尔姿态控制 |
| `franka_example_controllers/ElbowExampleController` | ElbowExampleController | cartesian_velocity + elbow_command | 肘部控制(零空间) |

---

## 六、Joint Position 控制器

### 6.1 自定义 serl_franka_controllers/JointPositionController

- **插件名**: `serl_franka_controllers/JointPositionController`
- **类型**: `serl_franka_controllers::JointPositionController`
- **基类**: `controller_interface::ControllerInterface`
- **定义文件**: [joint_position_controller.h](file:///home/xusj/hil-serl-ros2/ros2_ws/src/serl_franka_controllers/include/serl_franka_controllers/joint_position_controller.h)
- **实现文件**: [joint_position_controller.cpp](file:///home/xusj/hil-serl-ros2/ros2_ws/src/serl_franka_controllers/src/joint_position_controller.cpp)

**命令接口**: `effort` (7个关节力矩) -- 即 `{arm_id}_joint1/effort` 到 `{arm_id}_joint7/effort`

**注意**: 虽然名为 "JointPositionController"，但它实际使用 **effort 接口**，通过 PD 控制律计算力矩：
```
tau = k_gains[i] * (q_d - q) + d_gains[i] * (dq_d - dq)
```

**状态接口**:
- `{arm_id}_joint{1-7}/position` (7个关节位置)
- `{arm_id}_joint{1-7}/velocity` (7个关节速度)

**控制参数** (在 [serl_franka_controllers.yaml](file:///home/xusj/hil-serl-ros2/ros2_ws/src/serl_franka_controllers/config/serl_franka_controllers.yaml) 中):
- `arm_id`: "fr3"
- `target_joint_positions`: [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785] (弧度)
- `k_gains`: [600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0]
- `d_gains`: [50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0]
- `motion_duration_`: 10.0 秒 (使用5次多项式平滑插值)

**Launch 文件**: [joint.launch.py](file:///home/xusj/hil-serl-ros2/ros2_ws/src/serl_franka_controllers/launch/joint.launch.py)
- 默认加载 `joint_position_controller`
- 可选加载 gripper

### 6.2 官方示例控制器

| 插件名 | 类型 | 命令接口 | 说明 |
|--------|------|----------|------|
| `franka_example_controllers/JointPositionExampleController` | JointPositionExampleController | position | 关节位置控制(直接位置接口) |
| `franka_example_controllers/JointImpedanceExampleController` | JointImpedanceExampleController | effort | 关节阻抗控制 |
| `franka_example_controllers/JointImpedanceWithIKExampleController` | JointImpedanceWithIKExampleController | effort | 关节阻抗+IK控制 |
| `franka_example_controllers/JointVelocityExampleController` | JointVelocityExampleController | velocity | 关节速度控制 |
| `franka_example_controllers/MoveToStartExampleController` | MoveToStartExampleController | effort | 移动到初始位置 |
| `franka_example_controllers/GravityCompensationExampleController` | GravityCompensationExampleController | effort | 重力补偿 |

---

## 七、其他关键接口

### 7.1 FrankaRobotStateBroadcaster

- **插件名**: `franka_robot_state_broadcaster/FrankaRobotStateBroadcaster`
- **类型**: `franka_robot_state_broadcaster::FrankaRobotStateBroadcaster`
- **定义文件**: [franka_robot_state_broadcaster.xml](file:///home/xusj/hil-serl-ros2/ros2_ws/src/franka_ros2/franka_robot_state_broadcaster/franka_robot_state_broadcaster.xml)
- **状态接口**: `robot_state` -- 读取 franka::RobotState 并发布 `franka_msgs/msg/FrankaRobotState`

### 7.2 FrankaRobotModel (语义组件)

- **类**: `franka_semantic_components::FrankaRobotModel`
- **定义文件**: [franka_robot_model.hpp](file:///home/xusj/hil-serl-ros2/ros2_ws/src/franka_ros2/franka_semantic_components/include/franka_semantic_components/franka_robot_model.hpp)
- **状态接口**: `{arm_id}/robot_model`, `{arm_id}/robot_state`
- **提供方法**: `getZeroJacobian()`, `getCoriolisForceVector()`, `getPoseMatrix()`, `getMassMatrix()`, `getGravityVector()` 等

### 7.3 FrankaSemanticComponents

| 组件 | 说明 |
|------|------|
| `FrankaRobotState` | 读取并解析 robot_state 接口 |
| `FrankaRobotModel` | 读取并解析 robot_model + robot_state 接口，提供动力学模型计算 |
| `FrankaCartesianPoseInterface` | 笛卡尔位姿命令/状态接口 |
| `FrankaCartesianVelocityInterface` | 笛卡尔速度命令/状态接口 |

---

## 八、控制器配置汇总

### 8.1 serl_franka_controllers 配置 ([serl_franka_controllers.yaml](file:///home/xusj/hil-serl-ros2/ros2_ws/src/serl_franka_controllers/config/serl_franka_controllers.yaml))

```yaml
controller_manager:
  update_rate: 1000
  thread_priority: 98
  joint_state_broadcaster:
    type: joint_state_broadcaster/JointStateBroadcaster
  franka_robot_state_broadcaster:
    type: franka_robot_state_broadcaster/FrankaRobotStateBroadcaster
  joint_position_controller:
    type: serl_franka_controllers/JointPositionController
  cartesian_impedance_controller:
    type: serl_franka_controllers/CartesianImpedanceController
```

### 8.2 官方 franka_bringup 配置 ([controllers.yaml](file:///home/xusj/hil-serl-ros2/ros2_ws/src/franka_ros2/franka_bringup/config/controllers.yaml))

包含全部官方示例控制器的配置，更新率 1000 Hz，所有控制器默认 `arm_id: "fr3"`。

---

## 九、关键话题/接口名称速查表

| 类别 | 名称 | 消息类型 | 方向 |
|------|------|----------|------|
| 夹爪抓取 Action | `~/grasp` | `franka_msgs/action/Grasp` | Client -> Server |
| 夹爪移动 Action | `~/move` | `franka_msgs/action/Move` | Client -> Server |
| 夹爪归位 Action | `~/homing` | `franka_msgs/action/Homing` | Client -> Server |
| MoveIt夹爪 Action | `~/gripper_command` | `control_msgs/action/GripperCommand` | Client -> Server |
| 错误恢复 Action | (ActionServer节点) | `franka_msgs/action/ErrorRecovery` | Client -> Server |
| 平衡位姿订阅 | `equilibrium_pose` | `geometry_msgs/msg/PoseStamped` | -> CartesianImpedanceController |
| 雅可比发布 | `franka_jacobian` | `serl_franka_controllers/msg/ZeroJacobian` | CartesianImpedanceController -> |
| 关节状态 | `franka/joint_states` | `sensor_msgs/msg/JointState` | Broadcaster -> |
| 夹爪关节状态 | `franka_gripper/joint_states` | `sensor_msgs/msg/JointState` | GripperNode -> |
| 机器人状态 | (FrankaRobotStateBroadcaster) | `franka_msgs/msg/FrankaRobotState` | Broadcaster -> |
| 设置关节刚度 | `~/set_joint_stiffness` | `franka_msgs/srv/SetJointStiffness` | Service |
| 设置笛卡尔刚度 | `~/set_cartesian_stiffness` | `franka_msgs/srv/SetCartesianStiffness` | Service |
| 设置负载 | `~/set_load` | `franka_msgs/srv/SetLoad` | Service |
| 设置TCP帧 | `~/set_tcp_frame` | `franka_msgs/srv/SetTCPFrame` | Service |
| 设置刚度帧 | `~/set_stiffness_frame` | `franka_msgs/srv/SetStiffnessFrame` | Service |
| 设置碰撞行为 | `~/set_force_torque_collision_behavior` | `franka_msgs/srv/SetForceTorqueCollisionBehavior` | Service |
| 设置完整碰撞行为 | `~/set_full_collision_behavior` | `franka_msgs/srv/SetFullCollisionBehavior` | Service |

> 注: `~/` 前缀表示相对于节点命名空间的话题。
        

## 新包 `franka_force_slide_task` 概览

### 📁 目录结构

```
franka_force_slide_task/
├── config/
│   └── controllers.yaml                              # 所有控制器 + 任务参数配置
├── include/franka_force_slide_task/
│   ├── cartesian_impedance_force_controller.h        # 核心：笛卡尔阻抗+力混合控制器
│   └── pseudo_inversion.h                            # 矩阵伪逆工具
├── src/
│   └── cartesian_impedance_force_controller.cpp      # 控制器实现
├── scripts/
│   └── task_sequence_node.py                         # 任务编排节点（Python）
├── launch/
│   └── task.launch.py                                # 一键启动文件
├── CMakeLists.txt
├── package.xml
└── franka_force_slide_task_plugin.xml                # pluginlib 插件描述
```

### 🔄 三步任务流程

| 步骤 | 动作 | 实现方式 |
|------|------|----------|
| **Step 1** | 关节位置控制运动到 STA 标准位置 | 复用 `serl_franka_controllers/JointPositionController`，目标位置 `[0.0, 0.168, 0.557, -2.193, -1.1723, 1.186, 0.110]` |
| **Step 2** | 夹爪以 5N 力夹取工具 | 调用 `franka_gripper/grasp` Action，参数：width=0.04m, force=5N |
| **Step 3** | XY 平面滑动 + Z 轴恒力 | 自研 `CartesianImpedanceForceController`，XY 阻抗位置控制 + Z 导纳力控制 |

### 🧠 核心控制器设计：`CartesianImpedanceForceController`

基于现有 `serl_franka_controllers/CartesianImpedanceController` 扩展，关键创新是 **Z 轴导纳力控制**：

```
控制律:
  XY 平面: F_xy = -K_xy * e_xy - D_xy * dx_xy       ← 标准阻抗控制
  Z 轴:    z_offset += (K_fp * force_error + K_fi * ∫force_error·dt) · dt  ← 导纳力控制
           F_z = -K_z * (z - z_eq - z_offset) - D_z * dz   ← 阻抗跟踪调整后的平衡点
  总力矩:  τ = J^T * F_cartesian + τ_nullspace + c(q, dq)
```

**力控制原理**：通过读取 `franka::RobotState::O_F_ext_hat_K[2]`（机器人估计的 Z 轴外力），使用 PI 控制器调整 Z 平衡位置偏移量，使机器人自动下压/上抬以维持恒定接触力。

**可调参数**（在 [controllers.yaml](file:///home/xusj/hil-serl-ros2/ros2_ws/src/franka_force_slide_task/config/controllers.yaml) 中配置）：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `desired_force_z` | 5.0 N | Z 轴目标接触力 |
| `force_kp` | 0.002 | 力控制比例增益 |
| `force_ki` | 0.0005 | 力控制积分增益 |
| `force_filter_coeff` | 0.01 | 力测量低通滤波系数 |
| `max_z_offset` | 0.05 m | Z 平衡位置最大偏移（安全限幅） |
| `max_force_integral` | 10.0 N·s | 力误差积分限幅（防积分饱和） |

**安全设计**：
- Z 平衡位置偏移有最大限制 (`max_z_offset`)
- 力误差积分有抗饱和限幅 (`max_force_integral`)
- 力矩变化率限幅 (`delta_tau_max = 1.0 Nm/ms`)
- 实时发布 `force_state` 话题用于监控（measured_force, desired_force, z_offset, force_error）

### 🎮 任务编排节点：`task_sequence_node.py`

自动执行三步序列，也可通过 `~/start` 服务手动触发：

```python
# 执行流程
1. 等待 joint_position_controller 完成运动 (10s)
2. 调用 franka_gripper/grasp action (5N力夹取)
3. 切换控制器: deactivate joint_position_controller → activate cartesian_impedance_force_controller
4. 发布 XY 轨迹到 equilibrium_pose 话题 (五次多项式插值)
```

**滑动轨迹参数**（可调）：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `slide_start_x/y` | (0.4, 0.0) | 滑动起点 (m) |
| `slide_end_x/y` | (0.6, 0.2) | 滑动终点 (m) |
| `slide_speed` | 0.02 m/s | 滑动速度 |
| `slide_z_height` | 0.15 m | 滑动时 Z 高度 |
| `gripper_force` | 5.0 N | 夹取力 |

### 🚀 启动方式

```bash
# 真实机器人
ros2 launch franka_force_slide_task task.launch.py robot_ip:=172.16.0.3

# 仿真
ros2 launch franka_force_slide_task task.launch.py use_fake_hardware:=true
```

### 📊 数据流

```
franka_hardware
    │ state_interfaces: joint pos/vel, robot_state
    │ command_interfaces: joint effort
    ▼
┌─────────────────────────────────────────────────────┐
│ Step 1: JointPositionController (serl_franka)       │
│   PD + 五次多项式插值 → 运动到 STA 位置             │
└─────────────────────────────────────────────────────┘
    │ (controller switch)
    ▼
┌─────────────────────────────────────────────────────┐
│ Step 3: CartesianImpedanceForceController (新)       │
│   XY: 阻抗位置控制 ← equilibrium_pose 话题          │
│   Z:  导纳力控制   ← O_F_ext_hat_K[2] + PI控制器   │
│   → joint effort commands                           │
└─────────────────────────────────────────────────────┘

franka_gripper (Step 2)
    │ grasp action: width=0.04m, force=5N
    ▼
    夹取工具
```

### ⚠️ 使用注意事项

1. **力方向约定**：`desired_force_z > 0` 表示期望机器人向下压（表面反力向上为正）
2. **负载配置**：如果夹取的工具较重，需要通过 `franka_msgs/srv/SetLoad` 服务设置负载参数，否则力估计会有偏差
3. **增益调参**：`force_kp` 和 `force_ki` 需要根据实际接触面刚度调整。软表面需要更大增益，硬表面需要更小增益
4. **监控话题**：`/cartesian_impedance_force_controller/force_state` 发布实时力状态，建议用 `ros2 topic echo` 监控

---

## 核心改动：从"导纳力控制"改为"前馈恒力阻抗控制"

### 之前（错误方案）❌

```
Z轴: 先用阻抗控制跟踪平衡点，再用PI控制器根据力误差调整平衡点偏移
→ 间接力控制，存在延迟，板子倾斜时力仍会波动
```

### 现在（正确方案）✅

```
Z轴: K_z = 0，直接在前馈通道注入 F_target，仅保留阻尼 D_z 防震荡
→ 直接力控制，机器人像无摩擦气缸一样随板面起伏，力始终恒定
```

### 控制律对比

**旧控制律（导纳法）：**
```
F_z = -K_z * (z - z_eq - z_offset) - D_z * ż    ← K_z≠0，间接调平衡点
z_offset += (K_fp * e_force + K_fi * ∫e_force)·dt ← PI调整偏移
```

**新控制律（前馈恒力）：**
```
F_z = F_target - D_z * ż                          ← K_z=0，直接注入目标力
```

### 关键代码变化（[cartesian_impedance_force_controller.cpp](file:///home/xusj/hil-serl-ros2/ros2_ws/src/franka_force_slide_task/src/cartesian_impedance_force_controller.cpp#L235-L238)）

```cpp
// Z轴: K_z=0, 直接前馈注入目标力
f_cartesian(2) = desired_force_z_ - z_damping_ * cartesian_velocity(2);

// XY轴: 高刚度阻抗位置控制
f_cartesian.head(2) = -xy_stiffness_ * error_.head(2) 
                      - xy_damping_ * cartesian_velocity.head(2) 
                      - Ki_.block<2,2>(0,0) * error_i.head(2);
```

### 刚度矩阵设置（[L130-L138](file:///home/xusj/hil-serl-ros2/ros2_ws/src/franka_force_slide_task/src/cartesian_impedance_force_controller.cpp#L130-L138)）

```cpp
// XY: 高刚度 2000 N/m，Z: 刚度=0
cartesian_stiffness_.topLeftCorner(3, 3) << xy_stiffness_ * Eigen::Matrix3d::Identity();
cartesian_stiffness_(2, 2) = 0.0;  // ← 关键：Z轴零刚度

// XY: 阻尼 89 Ns/m，Z: 阻尼 15 Ns/m（防震荡）
cartesian_damping_.topLeftCorner(3, 3) << xy_damping_ * Eigen::Matrix3d::Identity();
cartesian_damping_(2, 2) = z_damping_;
```

### 可调参数（[controllers.yaml](file:///home/xusj/hil-serl-ros2/ros2_ws/src/franka_force_slide_task/config/controllers.yaml#L48-L58)）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `desired_force_z` | **5.0 N** | Z轴目标恒力（直接前馈注入） |
| `z_damping` | **15.0 Ns/m** | Z轴阻尼（防止震荡，10~20范围调） |
| `xy_stiffness` | **2000.0 N/m** | XY平移刚度（高刚度精确跟踪轨迹） |
| `xy_damping` | **89.0 Ns/m** | XY平移阻尼 |
| `rot_stiffness` | **30.0 Nm/rad** | 旋转刚度（保持末端姿态） |
| `rot_damping` | **5.0 Nms/rad** | 旋转阻尼 |
| `force_filter_coeff` | **0.01** | 力测量低通滤波系数（仅用于监控，不影响控制） |

### 为什么这个方案适合触觉数据采集

1. **真正的恒力**：`F_target` 直接作为前馈注入，无论 EDM 板如何倾斜，Z 轴都会像无摩擦气缸一样随板面自动起伏，力始终精确为 `F_target`
2. **数据解耦**：法向变形完全一致，图像变化仅由 EDM 纹理和剪切力引起
3. **标签干净**：Ground Truth 法向力恒定，训练标签不会混乱
4. **无过饱和/缺失**：不会因为凝胶非线性硬化导致力从 2N 飙到 10N
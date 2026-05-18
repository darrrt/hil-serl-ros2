"""
This file starts a control server running on the real time PC connected to the franka robot.
In a screen run `python franka_eggflip_server.py`
"""
from flask import Flask, request, jsonify
import numpy as np
import time
import subprocess
import threading
from scipy.spatial.transform import Rotation as R
from absl import app, flags

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from franka_msgs.action import ErrorRecovery
from franka_msgs.msg import FrankaRobotState
from serl_franka_controllers.msg import ZeroJacobian
import geometry_msgs.msg as geom_msg
import std_msgs.msg as std_msg

FLAGS = flags.FLAGS
flags.DEFINE_string(
    "robot_ip", "172.16.0.2", "IP address of the franka robot's controller box"
)
flags.DEFINE_string(
    "gripper_ip", "192.168.1.114", "IP address of the robotiq gripper if being used"
)
flags.DEFINE_string(
    "gripper_type", "Robotiq", "Type of gripper to use: Robotiq, Franka, or None"
)
flags.DEFINE_string(
    "flask_url", "127.0.0.1", "URL for the flask server to run on."
)


class FrankaEggFlipServer:
    """Handles the starting and stopping of the wrench controller
    (as well as backup) joint recovery policy."""

    def __init__(self, node, robot_ip, gripper_type, ros_pkg_name):
        self.node = node
        self.robot_ip = robot_ip
        self.ros_pkg_name = ros_pkg_name
        self.gripper_type = gripper_type

        self.wrench_pub = node.create_publisher(
            geom_msg.WrenchStamped,
            "/cartesian_wrench_controller/wrench_target",
            10,
        )
        self.reset_pub = node.create_publisher(
            std_msg.Bool,
            "/cartesian_wrench_controller/reset",
            10,
        )
        self.recovery_client = ActionClient(
            node, ErrorRecovery, "/franka_control/error_recovery"
        )
        self.jacobian_sub = node.create_subscription(
            ZeroJacobian,
            "/cartesian_wrench_controller/franka_jacobian",
            self._set_jacobian,
            10,
        )
        time.sleep(1)
        self.state_sub = node.create_subscription(
            FrankaRobotState,
            "franka_robot_state_broadcaster/franka_robot_state",
            self._set_currpos,
            10,
        )

    def start_wrench(self):
        """Launches the wrench controller"""
        self.controller = subprocess.Popen(
            [
                "ros2",
                "launch",
                self.ros_pkg_name,
                "wrench.launch.py",
                "robot_ip:=" + self.robot_ip,
                f"load_gripper:={'true' if self.gripper_type == 'Franka' else 'false'}",
            ],
            stdout=subprocess.PIPE,
        )
        time.sleep(5)

    def stop_wrench(self):
        """Stops the wrench controller"""
        self.controller.terminate()
        self.controller.wait()
        time.sleep(1)

    def clear(self):
        """Clears any errors"""
        self.recovery_client.wait_for_server()
        goal = ErrorRecovery.Goal()
        self.recovery_client.send_goal_async(goal)

    def set_wrench(self, wrench: list):
        """Sends a wrench command to the robot. Wrench is a list of 6 floats [fx, fy, fz, tx, ty, tz]"""
        assert len(wrench) == 6
        msg = geom_msg.WrenchStamped()
        msg.header.frame_id = "0"
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.wrench.force = geom_msg.Vector3(x=wrench[0], y=wrench[1], z=wrench[2])
        msg.wrench.torque = geom_msg.Vector3(x=wrench[3], y=wrench[4], z=wrench[5])
        self.wrench_pub.publish(msg)

    def reset(self):
        msg = std_msg.Bool()
        msg.data = True
        self.reset_pub.publish(msg)
        time.sleep(2)

    def _set_currpos(self, msg):
        o_t_ee = msg.o_t_ee
        pos_vec = o_t_ee.pose.position
        ori = o_t_ee.pose.orientation
        r = R.from_quat([ori.x, ori.y, ori.z, ori.w])
        self.pos = np.array([pos_vec.x, pos_vec.y, pos_vec.z, ori.x, ori.y, ori.z, ori.w])

        measured = msg.measured_joint_state
        self.q = np.array(list(measured.position))
        self.dq = np.array(list(measured.velocity))

        k_f_ext = msg.k_f_ext_hat_k
        self.force = np.array([k_f_ext.wrench.force.x, k_f_ext.wrench.force.y, k_f_ext.wrench.force.z])
        self.torque = np.array([k_f_ext.wrench.torque.x, k_f_ext.wrench.torque.y, k_f_ext.wrench.torque.z])
        try:
            self.vel = self.jacobian @ self.dq
        except Exception:
            self.vel = np.zeros(6)
            self.node.get_logger().warn("Jacobian not set, end-effector velocity temporarily not available")

    def _set_jacobian(self, msg):
        jacobian = np.array(list(msg.zero_jacobian)).reshape((6, 7), order="F")
        self.jacobian = jacobian


def spin_thread(executor):
    executor.spin()


def main(_):
    ROS_PKG_NAME = "serl_franka_controllers"

    ROBOT_IP = FLAGS.robot_ip
    GRIPPER_IP = FLAGS.gripper_ip
    GRIPPER_TYPE = FLAGS.gripper_type

    rclpy.init()
    node = rclpy.create_node("franka_control_api")

    webapp = Flask(__name__)

    if GRIPPER_TYPE == "Robotiq":
        from robot_servers.robotiq_gripper_server import RobotiqGripperServer
        gripper_server = RobotiqGripperServer(gripper_ip=GRIPPER_IP)
    elif GRIPPER_TYPE == "Franka":
        from robot_servers.franka_gripper_server import FrankaGripperServer
        gripper_server = FrankaGripperServer()
    elif GRIPPER_TYPE == "None":
        gripper_server = None
    else:
        raise NotImplementedError("Gripper Type Not Implemented")

    robot_server = FrankaEggFlipServer(
        node=node,
        robot_ip=ROBOT_IP,
        gripper_type=GRIPPER_TYPE,
        ros_pkg_name=ROS_PKG_NAME,
    )
    robot_server.start_wrench()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    if GRIPPER_TYPE == "Franka":
        executor.add_node(gripper_server.node)
    elif GRIPPER_TYPE == "Robotiq":
        executor.add_node(gripper_server.node)

    spin_thread_obj = threading.Thread(target=spin_thread, args=(executor,), daemon=True)
    spin_thread_obj.start()

    @webapp.route("/startwrench", methods=["POST"])
    def start_wrench():
        robot_server.clear()
        robot_server.start_wrench()
        return "Started wrench"

    @webapp.route("/stopwrench", methods=["POST"])
    def stop_wrench():
        robot_server.stop_wrench()
        return "Stopped wrench"

    @webapp.route("/getpos_euler", methods=["POST"])
    def get_pose_euler():
        xyz = robot_server.pos[:3]
        r = R.from_quat(robot_server.pos[3:]).as_euler("xyz")
        return jsonify({"pose": np.concatenate([xyz, r]).tolist()})

    @webapp.route("/getpos", methods=["POST"])
    def get_pos():
        return jsonify({"pose": np.array(robot_server.pos).tolist()})

    @webapp.route("/getvel", methods=["POST"])
    def get_vel():
        return jsonify({"vel": np.array(robot_server.vel).tolist()})

    @webapp.route("/getforce", methods=["POST"])
    def get_force():
        return jsonify({"force": np.array(robot_server.force).tolist()})

    @webapp.route("/gettorque", methods=["POST"])
    def get_torque():
        return jsonify({"torque": np.array(robot_server.torque).tolist()})

    @webapp.route("/getq", methods=["POST"])
    def get_q():
        return jsonify({"q": np.array(robot_server.q).tolist()})

    @webapp.route("/getdq", methods=["POST"])
    def get_dq():
        return jsonify({"dq": np.array(robot_server.dq).tolist()})

    @webapp.route("/getjacobian", methods=["POST"])
    def get_jacobian():
        return jsonify({"jacobian": np.array(robot_server.jacobian).tolist()})

    @webapp.route("/get_gripper", methods=["POST"])
    def get_gripper():
        return jsonify({"gripper": gripper_server.gripper_pos if gripper_server else 0.0})

    @webapp.route("/reset", methods=["POST"])
    def reset():
        robot_server.reset()
        return "Reset"

    @webapp.route("/activate_gripper", methods=["POST"])
    def activate_gripper():
        print("activate gripper")
        if gripper_server:
            gripper_server.activate_gripper()
        return "Activated"

    @webapp.route("/reset_gripper", methods=["POST"])
    def reset_gripper():
        print("reset gripper")
        if gripper_server:
            gripper_server.reset_gripper()
        return "Reset"

    @webapp.route("/open_gripper", methods=["POST"])
    def open():
        print("open")
        if gripper_server:
            gripper_server.open()
        return "Opened"

    @webapp.route("/close_gripper", methods=["POST"])
    def close():
        print("close")
        if gripper_server:
            gripper_server.close()
        return "Closed"

    @webapp.route("/move_gripper", methods=["POST"])
    def move_gripper():
        gripper_pos = request.json
        pos = np.clip(int(gripper_pos["gripper_pos"]), 0, 255)
        print(f"move gripper to {pos}")
        if gripper_server:
            gripper_server.move(pos)
        return "Moved Gripper"

    @webapp.route("/clearerr", methods=["POST"])
    def clear():
        robot_server.clear()
        return "Clear"

    @webapp.route("/wrench", methods=["POST"])
    def pose():
        pos = np.array(request.json["arr"])
        robot_server.set_wrench(pos)
        return "Moved"

    @webapp.route("/getstate", methods=["POST"])
    def get_state():
        return jsonify(
            {
                "pose": np.array(robot_server.pos).tolist(),
                "vel": np.array(robot_server.vel).tolist(),
                "force": np.array(robot_server.force).tolist(),
                "torque": np.array(robot_server.torque).tolist(),
                "q": np.array(robot_server.q).tolist(),
                "dq": np.array(robot_server.dq).tolist(),
                "jacobian": np.array(robot_server.jacobian).tolist(),
                "gripper_pos": gripper_server.gripper_pos if gripper_server else 0.0,
            }
        )

    webapp.run(host=FLAGS.flask_url)


if __name__ == "__main__":
    app.run(main)

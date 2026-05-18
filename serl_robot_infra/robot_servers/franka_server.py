"""
This file starts a control server running on the real time PC connected to the franka robot.
In a screen run `python franka_server.py`
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
from franka_msgs.srv import SetLoad
from serl_franka_controllers.msg import ZeroJacobian
import geometry_msgs.msg as geom_msg

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
flags.DEFINE_list(
    "reset_joint_target",
    [0, 0, 0, -1.9, -0, 2, 0],
    "Target joint angles for the robot to reset to",
)
flags.DEFINE_string(
    "flask_url", "127.0.0.1", "URL for the flask server to run on."
)
flags.DEFINE_string(
    "launch_robot_ip", "172.16.0.2",
    "Robot IP for the launch file (may differ from robot_ip)"
)


class FrankaServer:
    """Handles the starting and stopping of the impedance controller
    (as well as backup) joint recovery policy."""

    def __init__(self, node, robot_ip, gripper_type, ros_pkg_name, reset_joint_target):
        self.node = node
        self.robot_ip = robot_ip
        self.ros_pkg_name = ros_pkg_name
        self.reset_joint_target = reset_joint_target
        self.gripper_type = gripper_type

        self.eepub = node.create_publisher(
            geom_msg.PoseStamped,
            "/cartesian_impedance_controller/equilibrium_pose",
            10,
        )
        self.recovery_client = ActionClient(
            node, ErrorRecovery, "/franka_control/error_recovery"
        )
        self.jacobian_sub = node.create_subscription(
            ZeroJacobian,
            "/cartesian_impedance_controller/franka_jacobian",
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

    def start_impedance(self):
        """Launches the impedance controller"""
        self.imp = subprocess.Popen(
            [
                "ros2",
                "launch",
                self.ros_pkg_name,
                "impedance.launch.py",
                "robot_ip:=" + self.robot_ip,
                f"load_gripper:={'true' if self.gripper_type == 'Franka' else 'false'}",
            ],
            stdout=subprocess.PIPE,
        )
        time.sleep(5)

    def stop_impedance(self):
        """Stops the impedance controller"""
        self.imp.terminate()
        self.imp.wait()
        time.sleep(1)

    def clear(self):
        """Clears any errors"""
        self.recovery_client.wait_for_server()
        goal = ErrorRecovery.Goal()
        self.recovery_client.send_goal_async(goal)

    def reset_joint(self):
        """Resets Joints (needed after running for hours)"""
        try:
            self.stop_impedance()
            self.clear()
        except Exception:
            print("impedance Not Running")
        time.sleep(3)
        self.clear()

        self.joint_controller = subprocess.Popen(
            [
                "ros2",
                "launch",
                self.ros_pkg_name,
                "joint.launch.py",
                "robot_ip:=" + self.robot_ip,
                f"load_gripper:={'true' if self.gripper_type == 'Franka' else 'false'}",
            ],
            stdout=subprocess.PIPE,
        )
        time.sleep(1)
        print("RUNNING JOINT RESET")
        self.clear()

        count = 0
        time.sleep(1)
        while not np.allclose(
            np.array(self.reset_joint_target) - np.array(self.q),
            0,
            atol=1e-2,
            rtol=1e-2,
        ):
            time.sleep(1)
            count += 1
            if count > 30:
                print("joint reset TIMEOUT")
                break

        print("RESET DONE")
        self.joint_controller.terminate()
        self.joint_controller.wait()
        time.sleep(1)
        self.clear()
        print("KILLED JOINT RESET", self.pos)

        self.start_impedance()
        print("impedance STARTED")

    def move(self, pose: list):
        """Moves to a pose: [x, y, z, qx, qy, qz, qw]"""
        assert len(pose) == 7
        msg = geom_msg.PoseStamped()
        msg.header.frame_id = "0"
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.position = geom_msg.Point(x=pose[0], y=pose[1], z=pose[2])
        msg.pose.orientation = geom_msg.Quaternion(
            x=pose[3], y=pose[4], z=pose[5], w=pose[6]
        )
        self.eepub.publish(msg)

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
    RESET_JOINT_TARGET = FLAGS.reset_joint_target

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

    robot_server = FrankaServer(
        node=node,
        robot_ip=ROBOT_IP,
        gripper_type=GRIPPER_TYPE,
        ros_pkg_name=ROS_PKG_NAME,
        reset_joint_target=RESET_JOINT_TARGET,
    )
    robot_server.start_impedance()

    set_load_client = node.create_client(SetLoad, '/franka_control/set_load')

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    if GRIPPER_TYPE == "Franka":
        executor.add_node(gripper_server.node)
    elif GRIPPER_TYPE == "Robotiq":
        executor.add_node(gripper_server.node)

    spin_thread_obj = threading.Thread(target=spin_thread, args=(executor,), daemon=True)
    spin_thread_obj.start()

    @webapp.route("/set_load", methods=["POST"])
    def set_load():
        data = request.json
        mass = float(data['mass'])
        F_x_center_load = data['F_x_center_load']
        load_inertia = data['load_inertia']
        req = SetLoad.Request()
        req.mass = mass
        req.center_of_mass = F_x_center_load
        req.load_inertia = load_inertia
        set_load_client.wait_for_service()
        set_load_client.call_async(req)
        print("Set mass to", mass)
        return "Set Load"

    @webapp.route("/startimp", methods=["POST"])
    def start_impedance():
        robot_server.clear()
        robot_server.start_impedance()
        return "Started impedance"

    @webapp.route("/stopimp", methods=["POST"])
    def stop_impedance():
        robot_server.stop_impedance()
        return "Stopped impedance"

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

    @webapp.route("/jointreset", methods=["POST"])
    def joint_reset():
        robot_server.clear()
        robot_server.reset_joint()
        return "Reset Joint"

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

    @webapp.route("/close_gripper_slow", methods=["POST"])
    def close_slow():
        print("close slow")
        if gripper_server:
            gripper_server.close_slow()
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

    @webapp.route("/pose", methods=["POST"])
    def pose():
        pos = np.array(request.json["arr"])
        robot_server.move(pos)
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

    @webapp.route("/update_param", methods=["POST"])
    def update_param():
        params = request.json
        for key, value in params.items():
            param = rclpy.parameter.Parameter(key, value=value)
            node.set_parameters([param])
        return "Updated compliance parameters"

    webapp.run(host=FLAGS.flask_url)


if __name__ == "__main__":
    app.run(main)

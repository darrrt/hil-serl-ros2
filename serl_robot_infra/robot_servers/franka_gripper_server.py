import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from franka_msgs.action import Grasp, Move
from sensor_msgs.msg import JointState
import numpy as np

from robot_servers.gripper_server import GripperServer


class FrankaGripperServer(GripperServer):
    def __init__(self):
        super().__init__()
        self.node = rclpy.create_node('franka_gripper_server')
        self.binary_gripper_pose = 0

        self.move_client = ActionClient(self.node, Move, 'franka_gripper/move')
        self.grasp_client = ActionClient(self.node, Grasp, 'franka_gripper/grasp')
        self.gripper_sub = self.node.create_subscription(
            JointState, '/franka_gripper/joint_states', self._update_gripper, 10
        )

    def open(self):
        if self.binary_gripper_pose == 0:
            return
        goal = Move.Goal()
        goal.width = 0.09
        goal.speed = 0.3
        self.move_client.wait_for_server()
        self.move_client.send_goal_async(goal)
        self.binary_gripper_pose = 0

    def close(self):
        if self.binary_gripper_pose == 1:
            return
        goal = Grasp.Goal()
        goal.width = 0.01
        goal.speed = 0.3
        goal.epsilon.inner = 1.0
        goal.epsilon.outer = 1.0
        goal.force = 130.0
        self.grasp_client.wait_for_server()
        self.grasp_client.send_goal_async(goal)
        self.binary_gripper_pose = 1

    def close_slow(self):
        if self.binary_gripper_pose == 1:
            return
        goal = Grasp.Goal()
        goal.width = 0.01
        goal.speed = 0.1
        goal.epsilon.inner = 1.0
        goal.epsilon.outer = 1.0
        goal.force = 130.0
        self.grasp_client.wait_for_server()
        self.grasp_client.send_goal_async(goal)
        self.binary_gripper_pose = 1

    def move(self, position: int):
        goal = Move.Goal()
        goal.width = float(position / (255 * 10))
        goal.speed = 0.3
        self.move_client.wait_for_server()
        self.move_client.send_goal_async(goal)

    def _update_gripper(self, msg):
        self.gripper_pos = np.sum(msg.position) / 0.08

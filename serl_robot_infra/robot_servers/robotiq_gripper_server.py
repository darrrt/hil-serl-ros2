import subprocess
import rclpy
from rclpy.node import Node
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input as inputMsg

from robot_servers.gripper_server import GripperServer


class RobotiqGripperServer(GripperServer):
    def __init__(self, gripper_ip):
        super().__init__()
        self.node = rclpy.create_node('robotiq_gripper_server')

        self.gripper = subprocess.Popen(
            [
                "ros2",
                "run",
                "robotiq_2f_gripper_control",
                "Robotiq2FGripperTcpNode.py",
                gripper_ip,
            ],
            stdout=subprocess.PIPE,
        )
        self.gripper_state_sub = self.node.create_subscription(
            inputMsg.Robotiq2FGripper_robot_input,
            "Robotiq2FGripperRobotInput",
            self._update_gripper,
            10,
        )
        self.gripperpub = self.node.create_publisher(
            outputMsg.Robotiq2FGripper_robot_output,
            "Robotiq2FGripperRobotOutput",
            10,
        )
        self.gripper_command = outputMsg.Robotiq2FGripper_robot_output()

    def activate_gripper(self):
        self.gripper_command = self._generate_gripper_command("a", self.gripper_command)
        self.gripperpub.publish(self.gripper_command)

    def reset_gripper(self):
        self.gripper_command = self._generate_gripper_command("r", self.gripper_command)
        self.gripperpub.publish(self.gripper_command)
        self.activate_gripper()

    def open(self):
        self.gripper_command = self._generate_gripper_command("o", self.gripper_command)
        self.gripperpub.publish(self.gripper_command)

    def close(self):
        self.gripper_command = self._generate_gripper_command("c", self.gripper_command)
        self.gripperpub.publish(self.gripper_command)

    def move(self, position):
        self.gripper_command = self._generate_gripper_command(position, self.gripper_command)
        self.gripperpub.publish(self.gripper_command)

    def close_slow(self):
        self.gripper_command = self._generate_gripper_command("cs", self.gripper_command)
        self.gripperpub.publish(self.gripper_command)

    def _update_gripper(self, msg):
        self.gripper_pos = 1 - msg.gPO / 255

    def _generate_gripper_command(self, char, command):
        if char == "a":
            command = outputMsg.Robotiq2FGripper_robot_output()
            command.rACT = 1
            command.rGTO = 1
            command.rSP = 255
            command.rFR = 30

        elif char == "r":
            command = outputMsg.Robotiq2FGripper_robot_output()
            command.rACT = 0
            command.rSP = 255

        elif char == "c":
            command.rPR = 255
            command.rSP = 255

        elif char == "cs":
            command.rPR = 255
            command.rSP = 50

        elif char == "o":
            command.rPR = 175
            command.rSP = 255

        try:
            command.rPR = int(char)
            if command.rPR > 255:
                command.rPR = 255
            if command.rPR < 0:
                command.rPR = 0
        except ValueError:
            pass
        return command

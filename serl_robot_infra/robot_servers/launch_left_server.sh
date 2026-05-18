source /home/xusj/hil-serl-ros2/ros2_ws/install/setup.bash

python franka_server.py \
    --robot_ip=173.16.0.2 \
    --gripper_type=Franka \
    --reset_joint_target=0,0,0,-1.9,-0,2,0 \
    --flask_url=127.0.0.1

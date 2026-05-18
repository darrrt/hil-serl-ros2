source /home/xusj/hil-serl-ros2/ros2_ws/install/setup.bash

python franka_server.py \
    --robot_ip=172.16.0.2 \
    --gripper_type=Robotiq \
    --gripper_ip=192.168.1.114 \
    --reset_joint_target=0,0,0,-1.9,-0,2,0 \
    --flask_url=127.0.0.1

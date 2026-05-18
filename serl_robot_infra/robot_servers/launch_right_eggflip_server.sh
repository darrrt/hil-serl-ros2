source /home/xusj/hil-serl-ros2/ros2_ws/install/setup.bash

python franka_eggflip_server.py \
    --robot_ip=172.16.0.2 \
    --gripper_type=None \
    --flask_url=127.0.0.1

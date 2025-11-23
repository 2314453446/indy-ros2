- 工作空间'ros2_ws'下只保留src/文件夹，然后在ros2_ws目录下重新编译：colcon build --symlink-install

indy7 机械臂+moveit2启动程序

1）终端 A（桥接节点：状态 + 控制）：
cd ~/JNU/connectstudy/ros2_ws
source install/local_setup.bash
ros2 run indy_dcp_bridge indy_dcp_fjt_server_node

2）终端 B（MoveIt）——注意也要先 source 这个 workspace，再 source ws_moveit2：
source ~/JNU/connectstudy/ros2_ws/install/local_setup.bash
source ~/ws_moveit2/install/local_setup.bash

然后运行下面你想要的
Real Robot
Start Indy Robot
ros2 launch indy_driver indy_bringup.launch.py indy_type:=indy7 indy_ip:=192.168.0.101
Start Indy with MoveIt
ros2 launch indy_moveit indy_moveit_real_robot.launch.py indy_type:=indy7 indy_ip:=192.168.0.101
Start Indy with Servoing
ros2 launch indy_moveit indy_moveit_real_robot.launch.py indy_type:=indy7 indy_ip:=192.168.0.101 servo_mode:=true

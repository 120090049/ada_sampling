roslaunch ada_sampling multi_uav_mavros_sitl_sdf.launch 

~/.local/share/ov/pkg/isaac_sim-2022.2.1/python.sh ~/catkin_ws/src/ada_sampling/scripts/isaac_sim/Isaac_Gazebo_allUAVs_ori.py

rosrun ada_sampling drone_controller.py 1
rosrun ada_sampling drone_controller.py 2
rosrun ada_sampling drone_controller.py 3

rosrun ada_sampling ship_auto_controller.py 1


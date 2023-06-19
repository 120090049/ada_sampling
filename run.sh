roslaunch ada_sampling multi_uav_mavros_sitl_sdf.launch 

~/.local/share/ov/pkg/isaac_sim-2022.2.1/python.sh ~/catkin_ws/src/ada_sampling/scripts/isaac_sim/Isaac_Gazebo_allUAVs.py

rosrun ada_sampling px4_controller_withcamera.py
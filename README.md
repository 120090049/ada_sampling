# Adaptive sampling

This repo is for Lingpeng's CMU RISS program code for the ONR project in Advanced Agent lab, which is summarized into a workshop paper: Distributed Target Estimation and Ergodic Control for Multi-robot Informative Path Planning, which can be seen in the following [link](https://drive.google.com/file/d/1IJOEOxvdY3j5BEAazWqhh8Zp-LvlOr3_/view?usp=sharing). (you can also see my poster and video in [link](https://riss.ri.cmu.edu/research-showcase/summer-scholars-2023-project-posters/), which you can find from sector of students mentored by Prof. Sycara)

In general, our research proposes a novel active learning architecture for spatial environment, addressing the problem of informative path planning in multi-robot systems. We proposed an information distribution map for path planning based on the target location and agent sensing performance, combining a Mixture of Gaussian Process for information estimation and ergodic control to extract informative samples. We developed a high-fidelity ship-tracking simulator with three VTOLs, validating our approach's effectiveness in information gathering and estimating ocean targets on a specially designed dataset. Future research could further our model into the spatial-temporal model and method for inter-robot information communication to better address real-world challenges in informative path planning.

![environment.png](https://github.com/120090049/ada_sampling/blob/main/pics/environment.png)

In general the repo consists of two part, simulator for the ocean, vessels and vtols, as well as the algorithm.

## Run simulator

The simulator is based on the CHAMP from air lab. It use gazebo for physical effect and ISAACSIM for visiual rendering.

To start with, launch gazebo (which setup all objects as well as flight control for vtols)

```jsx
roslaunch ada_sampling multi_uav_mavros_sitl_sdf.launch
```

Then, run ISAACSIM

```jsx
~/.local/share/ov/pkg/isaac_sim-2022.2.1/python.sh ~/catkin_ws/src/ada_sampling/scripts/isaac_sim/Isaac_Gazebo_allUAVs.py
```

Run control code for vtol (the num represent the vtol you are going to control, since we have three vtol, you can choose number from 1~3, you can add more by changing the launch file and corresponding ISAACSIM script accordingly):

```jsx
rosrun ada_sampling drone_controller.py 1
```

We have 8 vessels in total. We use bezier curve to model the ship movement. To make the vessel moving,  run following command (num = 1: send trajectory to the simulator and move the boat; arg = 2: record trajectory into a dataset and write it to a file; no argument: just show the trajectory using matplotlib)

```jsx
rosrun ada_sampling ship_auto_controller.py 1
```

![Untitled](https://github.com/120090049/ada_sampling/blob/main/pics/ship_tra.png)

## Run algorithm

The algorithm is realized through matlab, to run the code, please run:

```jsx
init
ship_tracking
```

The results are as follows, for the detailed implementation like sudo code, please refer to my RISS paper and video (shown in the link at the beginning)

![Untitled](https://github.com/120090049/ada_sampling/blob/main/pics/result.png)

(a) is the information estimation map with prior knowledge before the sampling starts. Each robot is represented by a differently colored circle. (b) is the information estimation after 15 iterations. The routes of robots are represented by the differently colored trajectory (c) is the comparison between the estimated information distribution and ground truth.

At the moment, I still haven’t combine the algorithm with simulator and I will keep working on the ship tracking algorithm development.

If you have any question related to the simulator or algorithm, feel free to e-mail: 120090049@link.cuhk.edu.cn or lingpenc@andrew.cmu.edu
# 3D-Informed-RRT-star

## Bibliography
If using these datasets, please cite [1]:

J Meng, VM Pawar, S Kay, A Li, “**UAV Path Planning System Based on 3D Informed RRT* for Dynamic Obstacle Avoidance**”. In *2018 IEEE International Conference on Robotics and Biomimetics* (ROBIO), 2018.

[[PDF here]](https://www.researchgate.net/profile/Jiawei-Meng-4/publication/331866286_UAV_Path_Planning_System_Based_on_3D_Informed_RRT_for_Dynamic_Obstacle_Avoidance/links/5e76c379299bf1892cff0747/UAV-Path-Planning-System-Based-on-3D-Informed-RRT-for-Dynamic-Obstacle-Avoidance.pdf)

```
@inproceedings{meng2018uav,
  title={UAV Path Planning System Based on 3D Informed RRT* for Dynamic Obstacle Avoidance},
  author={Meng, Jiawei and Pawar, Vijay M and Kay, Sebastian and Li, Angran},
  booktitle={2018 IEEE International Conference on Robotics and Biomimetics (ROBIO)},
  pages={1653--1658},
  year={2018},
  organization={IEEE}
}
```

This repository includes a simulation code for 3D Informed RRT* with redefining the shape of the search space as an oblique cylinder. 

To make this code run, you need to:
1. Run "roscore"
2. Run "rosrun rviz rviz"
3. Run "rosrun path planning env_node" (which contains the information about the boundary)
4. Run "rosrun path planning rrt_node" (which contains the 3D Informed RRT* path planner)

RVIZ parameters:  <br  />
1. Frame_id = "/path_planner"  <br  />
2. marker_topic = "path_planner_rrt"  <br  />

Once you have successfully run the path planner, the simulation should be like:

<p align="center">
  <img width="300" height="280" src="https://github.com/jiaweimeng/3D-Informed-RRT-star/blob/master/simulation.png">
</p>

<p align="center"> 
  Figure 1. Simulation of 3D Informed RRT star path planner: final path is indicated by blue, start point is indicated by red, goal point is indicated by green, rrt tree is indicated by orange and obstacle is indicated by gray.
</p>

In the 3D Informed RRT* path planner:
The new search space is redefined as an oblique cylinder and the top and bottom surfaces of it are ellipses.

How to define the new search space: 
1. Using the largest deviation between the existed path and the connection (from initial position to goal position) to define the long axis of the ellipse. 
2. Calculating the rotation angle (clockwise) and displacement of this ellipse compared with its standard form on x axis. 
3. Calculating the largest deviation between the long axis and the points on the existed path. Then, using the distance between the point (with largest deviation to long axis) and the center of the ellipse to define the short axis of the ellipse. 

Finally, with all these information, the new search space can be defined. As far as the author knows, there should be other ways to define the new search space which can make it contain smaller space. Please go for it if you want to think about that!

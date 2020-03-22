# 3D_Informed_RRT_star

This repository includes a simulation code for 3D Informed RRT* with redefining the shape of the search space as an oblique cylinder. 

To make this code run, you need to:
1. Run roscore
2. Run rviz
3. Run env_node (which contains the information about the boundary and obstacles)
4. Run rrt_node (which contains the 3D Informed RRT* path planner)

RVIZ parameters:  <br  />
1. Frame_id = "/path_planner"  <br  />
2. marker_topic = "path_planner_rrt"  <br  />

In the 3D Informed RRT* path planner:
The new search space is redefined as an oblique cylinder and the top and bottom surfaces of it are ellipses.

How to define the new search space: 
1. Using the largest deviation between the existed path and the connection (from initial position to goal position) to define the long axis of the ellipse. 
2. Calculating the rotation angle (clockwise) and displacement of this ellipse compared with its standard form on x axis. 
3. Calculating the largest deviation between the long axis and the points on the existed path. Then, using the distance between the point (with largest deviation to long axis) and the center of the ellipse to define the short axis of the ellipse. 

Finally, with all these information, the new search space can be defined. As far as the author knows, there should be other ways to define the new search space which can make it contain smaller space. Please go for it if you want to think about that!

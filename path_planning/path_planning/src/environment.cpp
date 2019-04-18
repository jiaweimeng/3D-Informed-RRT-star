#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <path_planning/rrt.h>
#include <path_planning/obstacles.h>
#include <geometry_msgs/Point.h>

#include <iostream>
#include <cmath>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>
#include <vector>

using namespace rrt;

void initializeMarkers(visualization_msgs::Marker &boundary)
{
  //init headers
	boundary.header.frame_id    = "path_planner";
	boundary.header.stamp       = ros::Time::now();
	boundary.ns                 = "path_planner";
	boundary.action             = visualization_msgs::Marker::ADD;
	boundary.pose.orientation.w = 1.0;

  //setting id for each marker
  boundary.id    = 110;

	//defining types
	boundary.type  = visualization_msgs::Marker::LINE_STRIP;

	//setting scale
	boundary.scale.x = 1;

  //assigning colors
	boundary.color.g = boundary.color.b = boundary.color.r = 0.0f;
	boundary.color.a = 1.0f;
}

vector<geometry_msgs::Point> initializeBoundary()
{
    vector<geometry_msgs::Point> bondArray;

    geometry_msgs::Point point;

    //first point
    point.x = 0;
    point.y = 0;
    point.z = 0;

    bondArray.push_back(point);

    //second point
    point.x = 0;
    point.y = 100;
    point.z = 0;

    bondArray.push_back(point);

    //third point
    point.x = 100;
    point.y = 100;
    point.z = 0;

    bondArray.push_back(point);

    //fourth point
    point.x = 100;
    point.y = 0;
    point.z = 0;
    bondArray.push_back(point);

    point.x = 100;
    point.y = 0;
    point.z = 100;
    bondArray.push_back(point);

    point.x = 0;
    point.y = 0;
    point.z = 100;
    bondArray.push_back(point);

    point.x = 0;
    point.y = 0;
    point.z = 0;
    bondArray.push_back(point);

    point.x = 100;
    point.y = 0;
    point.z = 0;
    bondArray.push_back(point);

    point.x = 100;
    point.y = 0;
    point.z = 100;
    bondArray.push_back(point);

    point.x = 100;
    point.y = 100;
    point.z = 100;
    bondArray.push_back(point);

    point.x = 100;
    point.y = 100;
    point.z = 0;
    bondArray.push_back(point);

    point.x = 100;
    point.y = 100;
    point.z = 100;
    bondArray.push_back(point);

    point.x = 0;
    point.y = 100;
    point.z = 100;
    bondArray.push_back(point);

    point.x = 0;
    point.y = 0;
    point.z = 100;
    bondArray.push_back(point);

    point.x = 0;
    point.y = 100;
    point.z = 100;
    bondArray.push_back(point);

    //last point again to complete the box
    point.x = 0;
    point.y = 100;
    point.z = 0;
    bondArray.push_back(point);

    return bondArray;
}


int main(int argc,char** argv)
{
    //initializing ROS
    ros::init(argc,argv,"env_node");
	  ros::NodeHandle n;

	  //defining Publisher
	  ros::Publisher env_publisher = n.advertise<visualization_msgs::Marker>("path_planner_rrt",1);

	  //defining markers
    visualization_msgs::Marker boundary;

    initializeMarkers(boundary);

    //initializing rrtTree
    // RRT myRRT(2.0,2.0,2.0);
    // int goalX, goalY, goalZ;
    // goalX = goalY = goalZ = 95;

    boundary.points = initializeBoundary();

    env_publisher.publish(boundary);

    while(ros::ok())
    {
        env_publisher.publish(boundary);
        ros::spinOnce();
        ros::Duration(1).sleep();
    }
	return 1;
}

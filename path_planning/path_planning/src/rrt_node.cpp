#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <path_planning/rrt.h>
#include <path_planning/obstacles.h>
#include <iostream>
#include <cmath>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>
#include <vector>
#include <time.h>
#include <std_msgs/String.h>
#include <string>

#define success false
#define running true

/*
setting scale to map the axaxa m^3 space to 100x100x100 m^3 space
*/
float scale = 4.0/100.0;

using namespace rrt;

bool status = running;


void initializeMarkers(visualization_msgs::Marker &sourcePoint,
    visualization_msgs::Marker &goalPoint,
    visualization_msgs::Marker &randomPoint,
    visualization_msgs::Marker &rrtTreeMarker,
    visualization_msgs::Marker &finalPath,
    visualization_msgs::Marker &obstacle
  )
{
  //init headers
	sourcePoint.header.frame_id    = goalPoint.header.frame_id    = randomPoint.header.frame_id    = rrtTreeMarker.header.frame_id    = finalPath.header.frame_id    = obstacle.header.frame_id   ="path_planner";
	sourcePoint.header.stamp       = goalPoint.header.stamp       = randomPoint.header.stamp       = rrtTreeMarker.header.stamp       = finalPath.header.stamp       = obstacle.header.stamp      = ros::Time::now();
	sourcePoint.ns                 = goalPoint.ns                 = randomPoint.ns                 = rrtTreeMarker.ns                 = finalPath.ns                 = obstacle.ns                = "path_planner";
	sourcePoint.action             = goalPoint.action             = randomPoint.action             = rrtTreeMarker.action             = finalPath.action             = obstacle.action            = visualization_msgs::Marker::ADD;
	sourcePoint.pose.orientation.w = goalPoint.pose.orientation.w = randomPoint.pose.orientation.w = rrtTreeMarker.pose.orientation.w = finalPath.pose.orientation.w = obstacle.pose.orientation.w = 1.0;

  //setting id for each marker
  sourcePoint.id    = 0;
	goalPoint.id      = 1;
	randomPoint.id    = 2;
	rrtTreeMarker.id  = 3;
  finalPath.id      = 4;
	obstacle.id       = 5;

	//defining types
	rrtTreeMarker.type                                    = visualization_msgs::Marker::LINE_LIST;
	finalPath.type                                        = visualization_msgs::Marker::LINE_STRIP;
	sourcePoint.type  = goalPoint.type = randomPoint.type = visualization_msgs::Marker::SPHERE;
  obstacle.type                                         = visualization_msgs::Marker::CYLINDER;

  //setting scale
	rrtTreeMarker.scale.x = 0.2;
	finalPath.scale.x     = 1;
	sourcePoint.scale.x   = goalPoint.scale.x = randomPoint.scale.x = 2;
  sourcePoint.scale.y   = goalPoint.scale.y = randomPoint.scale.y = 2;
  sourcePoint.scale.z   = goalPoint.scale.z = randomPoint.scale.z = 1;
  obstacle.scale.x      = obstacle.scale.y = 20;
  obstacle.scale.z      = 100;

  //assigning colors
	sourcePoint.color.r   = 1.0f;
	goalPoint.color.g     = 1.0f;
  randomPoint.color.b   = 1.0f;

	rrtTreeMarker.color.r = 0.8f;
	rrtTreeMarker.color.g = 0.4f;

	finalPath.color.r = 0.2f;
	finalPath.color.g = 0.2f;
	finalPath.color.b = 1.0f;

  obstacle.color.g = obstacle.color.b = obstacle.color.r = 0.8f;

	sourcePoint.color.a = goalPoint.color.a = randomPoint.color.a = rrtTreeMarker.color.a = finalPath.color.a = 1.0f;
  obstacle.color.a = 0.5f;
}

vector<geometry_msgs::Point> initializeObstacles()
{
    vector<geometry_msgs::Point> obstaclePosition;

    geometry_msgs::Point point;

    point.x = 50;
    point.y = 50;
    point.z = 50;
    obstaclePosition.push_back(point);

    return obstaclePosition;
}

/*
within a radius r, finding all the existing nodes
*/
vector<geometry_msgs::Point>  findAllNode(RRT &myRRT, RRT::rrtNode &tempNode, int radius, vector<geometry_msgs::Point> &obstArray)
{
  vector<geometry_msgs::Point> nearPoint;
  geometry_msgs::Point point;
  int i, j, twoPointDistance = 9999;
  for(i = 0; i < myRRT.getTreeSize(); i++)
  {
    twoPointDistance = sqrt(pow(tempNode.posX - myRRT.getPosX(i),2) + pow(tempNode.posY - myRRT.getPosY(i),2) + pow(tempNode.posZ - myRRT.getPosZ(i),2));
    //judging whether the node within an obstacle
    int node_status = 0;
    for(j = 0; j < obstArray.size(); j++)
    {
        if ((twoPointDistance < radius) && (sqrt(pow(obstArray[j].x - myRRT.getPosX(j), 2) + pow(obstArray[j].y - myRRT.getPosY(j), 2)) > 20.0))
        {
          node_status = node_status + 1;
        }
    }
    if (node_status == obstArray.size())
    {
        point.x = myRRT.getPosX(i);
        point.y = myRRT.getPosY(i);
        point.z = myRRT.getPosZ(i);
        nearPoint.push_back(point);
    }
  }
  return nearPoint;
}

/*
Computing the cost of all the paths including newNode
*/
int computeTheCost(RRT &myRRT, RRT::rrtNode &tempNode, vector<geometry_msgs::Point> &nearPoint)
{
  vector<float> totalLength;
  geometry_msgs::Point point, new_point, old_point;
  RRT::rrtNode node;
  vector<int> path;
  int rightPointID, nearPointID;

  rightPointID = myRRT.getID(nearPoint[0].x, nearPoint[0].y, nearPoint[0].z);

  if(nearPoint.size() >= 2)
  {
    for(int i = 0;i < nearPoint.size();i++)
    {
      nearPointID = myRRT.getID(nearPoint[i].x, nearPoint[i].y, nearPoint[i].z);
      path = myRRT.getRootToEndPath(nearPointID);
      totalLength.push_back(0);
      totalLength[i] = 0;
      if(path.size() >= 2)
      {
        for(int j = 1;j < path.size();j++)
        {
          node = myRRT.getNode(path[j - 1]);
          old_point.x = node.posX;
          old_point.y = node.posY;
          old_point.z = node.posZ;

          node = myRRT.getNode(path[j]);
          new_point.x = node.posX;
          new_point.y = node.posY;
          new_point.z = node.posZ;

          totalLength[i] = totalLength[i] + sqrt(pow(old_point.x - new_point.x, 2) + pow(old_point.y - new_point.y, 2) +pow(old_point.z - new_point.z, 2));
        }
        node = myRRT.getNode(nearPointID);
        point.x = node.posX;
        point.y = node.posY;
        point.z = node.posZ;

        totalLength[i] = totalLength[i] + sqrt(pow(point.x - tempNode.posX, 2) + pow(point.y - tempNode.posY, 2) +pow(point.z - tempNode.posZ, 2));
      }
      if(totalLength[i] > totalLength[i - 1])
      {
        rightPointID = myRRT.getID(nearPoint[i].x, nearPoint[i].y, nearPoint[i].z);
      }
      else
      {
        totalLength[i] = totalLength[i - 1];
      }
    }
  }
  return rightPointID;
}


void addBranchtoRRTTree(visualization_msgs::Marker &rrtTreeMarker, RRT::rrtNode &tempNode, RRT &myRRT)
{
  geometry_msgs::Point point;

  point.x = tempNode.posX;
  point.y = tempNode.posY;
  point.z = tempNode.posZ;
  rrtTreeMarker.points.push_back(point);

  RRT::rrtNode parentNode = myRRT.getParent(tempNode.nodeID);

  point.x = parentNode.posX;
  point.y = parentNode.posY;
  point.z = parentNode.posZ;
  rrtTreeMarker.points.push_back(point);
}

/*
computing the new aera
*/
vector<geometry_msgs::Point> findEllipseArea(RRT &myRRT, visualization_msgs::Marker &sourcePoint, visualization_msgs::Marker &goalPoint, vector<int> path)
{
  vector<geometry_msgs::Point> ellipse;
  geometry_msgs::Point farest_point, parameter;
  RRT::rrtNode node;
  float long_axis = 0, short_axis = 0, d_short = 0, l_distance = 0, s_distance = 0;
  float x, y, z, x_4, y_4, delta_x, delta_y;
  float x_cl, y_cl, z_cl;
  //Find the long axis of the ellipse area
  for(int i = 0;i < path.size();i++)
  {
    node = myRRT.getNode(path[i]);
    z = node.posZ;
    y = ((z - sourcePoint.pose.position.z)/(goalPoint.pose.position.z - sourcePoint.pose.position.z))*(goalPoint.pose.position.y - sourcePoint.pose.position.y) + sourcePoint.pose.position.y;
    x = ((z - sourcePoint.pose.position.z)/(goalPoint.pose.position.z - sourcePoint.pose.position.z))*(goalPoint.pose.position.x - sourcePoint.pose.position.x) + sourcePoint.pose.position.x;
    l_distance = sqrt(pow(node.posX - x, 2) + pow(node.posY - y, 2));
    if(l_distance > long_axis)
    {
      long_axis = l_distance;
      farest_point.x = node.posX;
      farest_point.y = node.posY;
      farest_point.z = node.posZ;
    }
  }
  parameter.x = long_axis;
  z_cl = farest_point.z;
  y_cl = ((z - sourcePoint.pose.position.z)/(goalPoint.pose.position.z - sourcePoint.pose.position.z))*(goalPoint.pose.position.y - sourcePoint.pose.position.y) + sourcePoint.pose.position.y;
  x_cl = ((z - sourcePoint.pose.position.z)/(goalPoint.pose.position.z - sourcePoint.pose.position.z))*(goalPoint.pose.position.x - sourcePoint.pose.position.x) + sourcePoint.pose.position.x;
  delta_x = farest_point.x - x_cl;
  delta_y = farest_point.y - y_cl;
  //Find the short axis of the ellipse
  if(farest_point.y == y_cl)
  {
    parameter.z = 0;                              //rotation angle equals to 0
    for(int i = 0;i < path.size();i++)
    {
      node = myRRT.getNode(path[i]);
      z = node.posZ;
      y = ((z - sourcePoint.pose.position.z)/(goalPoint.pose.position.z - sourcePoint.pose.position.z))*(goalPoint.pose.position.y - sourcePoint.pose.position.y) + sourcePoint.pose.position.y;
      x = ((z - sourcePoint.pose.position.z)/(goalPoint.pose.position.z - sourcePoint.pose.position.z))*(goalPoint.pose.position.x - sourcePoint.pose.position.x) + sourcePoint.pose.position.x;
      s_distance = sqrt(pow(node.posY - y, 2));
      if(s_distance > d_short)
      {
        d_short = s_distance;
        short_axis = sqrt(pow(node.posY - y, 2) + pow(node.posX - x, 2));
      }
    }
    parameter.y = short_axis;
  }
  else if(farest_point.x == x_cl)
  {
    parameter.z = M_PI * (90.0/180.0);                //rotation angle equals to 90
    for(int i = 0;i < path.size();i++)
    {
      node = myRRT.getNode(path[i]);
      z = node.posZ;
      y = ((z - sourcePoint.pose.position.z)/(goalPoint.pose.position.z - sourcePoint.pose.position.z))*(goalPoint.pose.position.y - sourcePoint.pose.position.y) + sourcePoint.pose.position.y;
      x = ((z - sourcePoint.pose.position.z)/(goalPoint.pose.position.z - sourcePoint.pose.position.z))*(goalPoint.pose.position.x - sourcePoint.pose.position.x) + sourcePoint.pose.position.x;
      s_distance = sqrt(pow(node.posX - x, 2));
      if(s_distance > d_short)
      {
        d_short = s_distance;
        short_axis = sqrt(pow(node.posY - y, 2) + pow(node.posX - x, 2));
      }
    }
    parameter.y = short_axis;
  }
  else if((farest_point.y - y_cl) > 0 && (farest_point.x != x_cl))
  {
    parameter.z = M_PI * (atan2(farest_point.y - y_cl, farest_point.x - x_cl)/180.0);
    for(int i = 0;i < path.size();i++)
    {
      node = myRRT.getNode(path[i]);
      z = node.posZ;
      y = ((z - sourcePoint.pose.position.z)/(goalPoint.pose.position.z - sourcePoint.pose.position.z))*(goalPoint.pose.position.y - sourcePoint.pose.position.y) + sourcePoint.pose.position.y;
      x = ((z - sourcePoint.pose.position.z)/(goalPoint.pose.position.z - sourcePoint.pose.position.z))*(goalPoint.pose.position.x - sourcePoint.pose.position.x) + sourcePoint.pose.position.x;
      x_4 = x + ( (delta_x*delta_x)*(node.posX - x) + delta_y*(node.posY - y)*delta_x )/( delta_x*delta_x + delta_y*delta_y );
      y_4 = y + ( delta_x*delta_y*(node.posX - x) + (delta_y*delta_y)*(node.posY - y) )/( delta_x+delta_x + delta_y*delta_y );
      s_distance = sqrt(pow(node.posX - x_4, 2) + pow(node.posY - y_4, 2));
      if(s_distance > d_short)
      {
        d_short = s_distance;
        short_axis = sqrt(pow(node.posY - y, 2) + pow(node.posX - x, 2));
      }
    }
    parameter.y = short_axis;
  }
  else if((farest_point.y - y_cl < 0) && (farest_point.x != x_cl))
  {
    parameter.z = M_PI * ((180.0 + atan2(farest_point.y - y_cl, farest_point.x - x_cl))/180.0);
    for(int i = 0;i < path.size();i++)
    {
      node = myRRT.getNode(path[i]);
      z = node.posZ;
      y = ((z - sourcePoint.pose.position.z)/(goalPoint.pose.position.z - sourcePoint.pose.position.z))*(goalPoint.pose.position.y - sourcePoint.pose.position.y) + sourcePoint.pose.position.y;
      x = ((z - sourcePoint.pose.position.z)/(goalPoint.pose.position.z - sourcePoint.pose.position.z))*(goalPoint.pose.position.x - sourcePoint.pose.position.x) + sourcePoint.pose.position.x;
      x_4 = x + ( (delta_x*delta_x)*(node.posX - x) + delta_y*(node.posY - y)*delta_x )/( delta_x*delta_x + delta_y*delta_y );
      y_4 = y + ( delta_x*delta_y*(node.posX - x) + (delta_y*delta_y)*(node.posY - y) )/( delta_x+delta_x + delta_y*delta_y );
      s_distance = sqrt(pow(node.posX - x_4, 2) + pow(node.posY - y_4, 2));
      if(s_distance > d_short)
      {
        d_short = s_distance;
        short_axis = sqrt(pow(node.posY - y, 2) + pow(node.posX - x, 2));
      }
    }
    parameter.y = short_axis;
  }
  ellipse.push_back(parameter);
  return ellipse;
}

// float computeNewRadius(RRT &myRRT, visualization_msgs::Marker &sourcePoint, visualization_msgs::Marker &goalPoint, vector<int> path)
// {
//   geometry_msgs::Point point;
//   RRT::rrtNode node;
//   float new_radius = 0, distance = 0;
//   float x, y, z;
//   for(int i = 0;i < path.size();i++)
//   {
//     node = myRRT.getNode(path[i]);
//
//     z = node.posZ;
//     y = ((z - sourcePoint.pose.position.z)/(goalPoint.pose.position.z - sourcePoint.pose.position.z))*(goalPoint.pose.position.y - sourcePoint.pose.position.y) + sourcePoint.pose.position.y;
//     x = ((z - sourcePoint.pose.position.z)/(goalPoint.pose.position.z - sourcePoint.pose.position.z))*(goalPoint.pose.position.x - sourcePoint.pose.position.x) + sourcePoint.pose.position.x;
//
//     distance = sqrt(pow(node.posX - x, 2) + pow(node.posY - y, 2));
//     if(distance > new_radius)
//     {
//       new_radius = distance;
//     }
//   }
//   return new_radius;
// }


bool checkIfInsideBoundary(RRT::rrtNode &tempNode)
{
    if(tempNode.posX < 0 || tempNode.posY < 0 || tempNode.posZ < 0 || tempNode.posX > 100 || tempNode.posY > 100 || tempNode.posZ > 100) return false;
    else return true;
}

bool checkIfOutsideObstacles(vector<geometry_msgs::Point> &obstArray, RRT::rrtNode &tempNode)
{
  double distance, radius;
  for(int i = 0;i < obstArray.size();i++)
  {
    radius = 10;
    distance = sqrt(pow(tempNode.posX - obstArray[i].x, 2) + pow(tempNode.posY - obstArray[i].y, 2));
    if(distance < radius)
    {
      return false;
    }
  }
  return true;
}

bool checkIfInsideNewArea(RRT::rrtNode &tempNode, vector<geometry_msgs::Point> ellipse, visualization_msgs::Marker &sourcePoint, visualization_msgs::Marker &goalPoint)
{
  float a, b, theta, equation;
  float x, y, z, x_2, y_2;
  a = ellipse[0].x;
  b = ellipse[0].y;
  theta = ellipse[0].z;
  z = tempNode.posZ;
  x = ((z - sourcePoint.pose.position.z)/(goalPoint.pose.position.z - sourcePoint.pose.position.z))*(goalPoint.pose.position.x - sourcePoint.pose.position.x) + sourcePoint.pose.position.x;
  y = ((z - sourcePoint.pose.position.z)/(goalPoint.pose.position.z - sourcePoint.pose.position.z))*(goalPoint.pose.position.y - sourcePoint.pose.position.y) + sourcePoint.pose.position.y;
  x_2 = (tempNode.posX - x)*cos(theta) + (tempNode.posY - y)*sin(theta);
  y_2 = (tempNode.posY - y)*cos(theta) - (tempNode.posX - x)*sin(theta);
  equation = pow(x_2, 2)/pow(a, 2) + pow(y_2, 2)/pow(b, 2);
  if(equation > 1.0)
  {
    return false;
  }
  else
  {
    return true;
  }
}

// bool checkIfInsideNewArea(RRT::rrtNode &tempNode, float new_radius, visualization_msgs::Marker &sourcePoint, visualization_msgs::Marker &goalPoint)
// {
//   float distance = 0;
//   float x, y, z;
//   x = ((tempNode.posZ - sourcePoint.pose.position.z)/(goalPoint.pose.position.z - sourcePoint.pose.position.z))*(goalPoint.pose.position.x - sourcePoint.pose.position.x) + sourcePoint.pose.position.x;
//   y = ((tempNode.posZ - sourcePoint.pose.position.z)/(goalPoint.pose.position.z - sourcePoint.pose.position.z))*(goalPoint.pose.position.y - sourcePoint.pose.position.y) + sourcePoint.pose.position.y;
//   z = tempNode.posZ;
//
//   distance = sqrt(pow(tempNode.posX - x, 2) + pow(tempNode.posY - y, 2));
//   if(distance >= new_radius)
//   {
//     return false;
//   }
//   else
//   {
//     return true;
//   }
// }

void generateTempPoint(RRT::rrtNode &tempNode)
{
    int x = rand() % 150 + 1;
    int y = rand() % 150 + 1;
    int z = rand() % 150 + 1;
    //std::cout<<"Random X: "<<x <<endl<<"Random Y: "<<y<<endl;
    tempNode.posX = x;
    tempNode.posY = y;
    tempNode.posZ = z;
}

bool addNewPointtoRRT(RRT &myRRT, RRT::rrtNode &tempNode, int rrtStepSize, vector<geometry_msgs::Point> &obstArray, vector<geometry_msgs::Point> &ellipse, visualization_msgs::Marker &sourcePoint, visualization_msgs::Marker &goalPoint)
{
    int nearestNodeID = myRRT.getNearestNodeID(tempNode.posX,tempNode.posY, tempNode.posZ);

    RRT::rrtNode nearestNode = myRRT.getNode(nearestNodeID);

    double theta = atan2(tempNode.posZ - nearestNode.posZ, sqrt(pow(tempNode.posX - nearestNode.posX, 2) + pow(tempNode.posY - nearestNode.posY, 2)));
    double alpha = atan2(tempNode.posY - nearestNode.posY, tempNode.posX - nearestNode.posX);

    tempNode.posZ = nearestNode.posZ + (rrtStepSize * sin(theta));
    tempNode.posX = nearestNode.posX + (rrtStepSize * cos(theta)) * cos(alpha);
    tempNode.posY = nearestNode.posY + (rrtStepSize * cos(theta)) * sin(alpha);

    if(checkIfInsideBoundary(tempNode) && checkIfOutsideObstacles(obstArray,tempNode) && checkIfInsideNewArea(tempNode, ellipse, sourcePoint, goalPoint))
    {
        vector<geometry_msgs::Point> nearPoint;
        int rightPointID;
        nearPoint = findAllNode(myRRT, tempNode, 6, obstArray);
        rightPointID = computeTheCost(myRRT, tempNode, nearPoint);
        tempNode.parentID = rightPointID;
        tempNode.nodeID = myRRT.getTreeSize();
        myRRT.addNewNode(tempNode);
        return true;
    }
    else
        return false;
}

bool checkNodetoGoal(int X, int Y, int Z, RRT::rrtNode &tempNode)
{
    double distance = sqrt(pow(X - tempNode.posX, 2) + pow(Y - tempNode.posY, 2) + pow(Z - tempNode.posZ, 2));
    if(distance <= 10)
    {
        return true;
    }
    return false;
}

void setFinalPathData(vector< vector<int> > &rrtPaths, RRT &myRRT, int i, visualization_msgs::Marker &finalpath, int goalX, int goalY, int goalZ)
{
    RRT::rrtNode tempNode;
    geometry_msgs::Point point;
    for(int j=0; j<rrtPaths[i].size();j++)
    {
        tempNode = myRRT.getNode(rrtPaths[i][j]);

        point.x = tempNode.posX;
        point.y = tempNode.posY;
        point.z = tempNode.posZ;

        finalpath.points.push_back(point);
    }

    point.x = goalX;
    point.y = goalY;
    point.z = goalZ;
    finalpath.points.push_back(point);
}


int main(int argc,char** argv)
{
    //initializing ROS
    ros::init(argc,argv,"rrt_node");
	  ros::NodeHandle n;

	  //defining Publisher
	  ros::Publisher rrt_publisher = n.advertise<visualization_msgs::Marker>("path_planner_rrt",1);

	  //defining markers
    visualization_msgs::Marker sourcePoint;
    visualization_msgs::Marker goalPoint;
    visualization_msgs::Marker randomPoint;
    visualization_msgs::Marker rrtTreeMarker;
    visualization_msgs::Marker finalPath;
    visualization_msgs::Marker obstacle;

    initializeMarkers(sourcePoint, goalPoint, randomPoint, rrtTreeMarker, finalPath, obstacle);

    //setting source and goal
    sourcePoint.pose.position.x = 2;
    sourcePoint.pose.position.y = 2;
    sourcePoint.pose.position.z = 2;

    goalPoint.pose.position.x = 95;
    goalPoint.pose.position.y = 95;
    goalPoint.pose.position.z = 95;



    vector<geometry_msgs::Point> current_position;

    current_position = initializeObstacles();

    obstacle.pose.position.x = current_position[0].x;
    obstacle.pose.position.y = current_position[0].y;
    obstacle.pose.position.z = current_position[0].z;


    ros::spinOnce();
    ros::Duration(0.01).sleep();

    srand (time(NULL));
    //initialize rrt specific variables

    //initializing rrtTree
    RRT myRRT(2.0,2.0,2.0);
    int goalX, goalY, goalZ;
    goalX = goalY = goalZ= 95;

    int rrtStepSize = 5;

    vector< vector<int> > rrtPaths;
    vector<int> path;
    int rrtPathLimit = 10;

    int shortestPathLength = 9999;
    int shortestPath = -1;
//    float new_radius = 999;

    RRT::rrtNode tempNode;

    vector<geometry_msgs::Point> obstaclePosition = initializeObstacles();

    bool addNodeResult = false, nodeToGoal = false;

    vector<geometry_msgs::Point> ellipse;

    geometry_msgs::Point point;
    point.x = 999;
    point.y = 999;
    point.z = 0;
    ellipse.push_back(point);

    vector<geometry_msgs::Point> ellipse1;

    ellipse1.push_back(point);

    while(ros::ok() && status)
    {
        if(rrtPaths.size() < rrtPathLimit)
        {
            generateTempPoint(tempNode);
            //std::cout<<"tempnode generated"<<endl;
            addNodeResult = addNewPointtoRRT(myRRT,tempNode,rrtStepSize,obstaclePosition, ellipse, sourcePoint, goalPoint);
            if(addNodeResult)
            {
               // std::cout<<"tempnode accepted"<<endl;
                addBranchtoRRTTree(rrtTreeMarker,tempNode,myRRT);
               // std::cout<<"tempnode printed"<<endl;

               rrt_publisher.publish(rrtTreeMarker);
               rrt_publisher.publish(sourcePoint);
               rrt_publisher.publish(goalPoint);
               rrt_publisher.publish(obstacle);
               rrt_publisher.publish(finalPath);
                nodeToGoal = checkNodetoGoal(goalX, goalY, goalZ, tempNode);
                if(nodeToGoal)
                {
                    path = myRRT.getRootToEndPath(tempNode.nodeID);
                    rrtPaths.push_back(path);
                    std::cout<<"New Path Found. Total paths "<<rrtPaths.size()<<endl;
                    ellipse1 = findEllipseArea(myRRT, sourcePoint, goalPoint, path);
                    if(ellipse[0].x*ellipse[0].y >= ellipse1[0].x*ellipse1[0].y)
                    {
                      ellipse = ellipse1;
                    }
                    std::cout<<"long_axis: ";
                    std::cout<< ellipse[0].x   << endl;
                    std::cout<<"short_axis: ";
                    std::cout<< ellipse[0].y   << endl;
                    std::cout<<"theta: ";
                    std::cout<< ellipse[0].z   << "\n";
                    //ros::Duration(10).sleep();
                    //std::cout<<"got Root Path"<<endl;
                }
            }
        }
        else //if(rrtPaths.size() >= rrtPathLimit)
        {
            status = success;
            std::cout<<"Finding Optimal Path"<<endl;
            for(int i=0; i<rrtPaths.size();i++)
            {
                if(rrtPaths[i].size() < shortestPath)
                {
                    shortestPath = i;
                    shortestPathLength = rrtPaths[i].size();
                }
            }
            setFinalPathData(rrtPaths, myRRT, shortestPath, finalPath, goalX, goalY, goalZ);
            rrt_publisher.publish(finalPath);
        }
        //rrt_publisher.publish(finalPath);
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
	return 1;
}

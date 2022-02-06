/*
 * rrtstar_visualization.cpp
 *
 * ---------------------------------------------------------------------
 * Copyright (C) 2022 Matthew (matthewoots at gmail.com)
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * ---------------------------------------------------------------------
 *
 * 
 * 
 */

#define KNRM  "\033[0m"
#define KRED  "\033[31m"
#define KGRN  "\033[32m"
#define KYEL  "\033[33m"
#define KBLU  "\033[34m"
#define KMAG  "\033[35m"
#define KCYN  "\033[36m"
#define KWHT  "\033[37m"

#include <string>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <math.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>

#include <geometry_msgs/Point.h>
#include "pcl_ros_lib/point_array.h"

using namespace std;
using namespace Eigen;

ros::Publisher object_marker_pub;
ros::Subscriber object_point_sub;


void object_point_callback(const pcl_ros_lib::point_array::ConstPtr &msg)
{
  pcl_ros_lib::point_array _points = *msg;

  visualization_msgs::Marker object_points, text_points;
  object_points.header.frame_id = text_points.header.frame_id = "/map";
  object_points.header.stamp = text_points.header.stamp = ros::Time::now();
  object_points.ns = text_points.ns = "object_visualization_points";
  object_points.action = text_points.action = visualization_msgs::Marker::ADD;
  object_points.pose.orientation.w = text_points.pose.orientation.w = 1.0;

  object_points.id = 0;

  object_points.type = visualization_msgs::Marker::POINTS;
  text_points.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

  // POINTS markers use x and y scale for width/height respectively
  object_points.scale.x = 0.3;
  object_points.scale.y = 0.3;

  // Points color
  object_points.color.r = 1.0f;
  object_points.color.g = 0.0f;
  object_points.color.b = 0.0f;
  object_points.color.a = 0.5f;

  int _size = _points.array.size();
  // Create the vertices for the points and lines
  for (int i = 0; i < _size; i++)
  {
    object_points.points.push_back(_points.array[i]);

    text_points.id = i + 1;
    text_points.scale.z = 0.5;
    text_points.color.r = 1.0f;
    text_points.color.g = 0.0f;
    text_points.color.b = 0.0f;
    text_points.color.a = 1.0f;

    text_points.pose.position = _points.array[i];
    text_points.pose.position.z += 1.2;

    text_points.text = "Cluster Index (" + to_string(i) + ")";
    object_marker_pub.publish(text_points);
  }

  object_marker_pub.publish(object_points);
}

int main( int argc, char** argv )
{
  double rate = 1.0;
  int _size;  
  ros::init(argc, argv, "pcl_ros_lib_visualization");
  ros::NodeHandle n("~");
  
  n.param<int>("marker_size", _size, 4);

  object_marker_pub = n.advertise<visualization_msgs::Marker>(
    "/object_point_marker", 10);    
  object_point_sub = n.subscribe<pcl_ros_lib::point_array>(
    "/object_points", 10, &object_point_callback);
  
  // ros::Rate r(rate);
  ros::spin();
  

  return 0;
}
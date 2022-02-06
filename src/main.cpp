/*
 * main.cpp
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
#include "dbscan.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <ros/ros.h>
#include <Eigen/Dense>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>

#define KNRM  "\033[0m"
#define KRED  "\033[31m"
#define KGRN  "\033[32m"
#define KYEL  "\033[33m"
#define KBLU  "\033[34m"
#define KMAG  "\033[35m"
#define KCYN  "\033[36m"
#define KWHT  "\033[37m"

#include "pcl_ros_lib/point_array.h"

using namespace std;

class ros_node_class
{
private:
    ros::NodeHandle _nh;
    ros::Publisher full_pcl_pub, altered_pcl_pub, cluster_pcl_pub;

public:
    sensor_msgs::PointCloud2 pcl_original;
    sensor_msgs::PointCloud2 pcl_altered;
    sensor_msgs::PointCloud2 pcl_clustered;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc;

    ros_node_class(ros::NodeHandle &nodeHandle)
    {
        full_pcl_pub = _nh.advertise<sensor_msgs::PointCloud2>("/full_pcl", 10);
        altered_pcl_pub = _nh.advertise<sensor_msgs::PointCloud2>("/altered_pcl", 10);
        cluster_pcl_pub = _nh.advertise<sensor_msgs::PointCloud2>("/cluster_pcl", 10);
        printf("%s[main.cpp] Constructor Setup Ready! \n", KGRN);
    }
    ~ros_node_class(){};

    void full_pcl_publisher() 
    {
        pcl_original.header.frame_id = "/map";
        full_pcl_pub.publish(pcl_original); 
        // printf("%s[main.cpp] Published full_cloud! \n", KGRN);
    }

    void altered_pcl_publisher() 
    {
        pcl_altered.header.frame_id = "/map";
        altered_pcl_pub.publish(pcl_altered); 
        // printf("%s[main.cpp] Published altered_cloud! \n", KGRN);
    }

    void cluster_pcl_publisher() 
    {
        pcl_clustered.header.frame_id = "/map";
        cluster_pcl_pub.publish(pcl_clustered); 
        // printf("%s[main.cpp] Published clustered_cloud! \n", KGRN);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_ros_lib_node");
    ros::NodeHandle _nh("~"); 

    ros_node_class ros_node_class(_nh);

    common_utility _common;
    dbscan _dbscan;
    
    // ROS Params
    std::string _file_location;
    double _resolution; // Resolution of octree
    double _translate_x, _translate_y, _translate_z;
    double _rotate_roll, _rotate_pitch, _rotate_yaw; // In degrees
    double _centroid_x, _centroid_y, _centroid_z;
    double _dimension_x, _dimension_y, _dimension_z;
    double _ros_rate;
    int _nearest_min_distance;
    float _eps; // The radius for searching neighbor points of octree
    int _min_cluster_pts;

    bool _crop, _transform, _cluster;
    bool _spin_once;

    _nh.param<std::string>("file_location", _file_location, "/cloud.pcd");
    _nh.param<double>("resolution", _resolution, 1.0);
    _nh.param<float>("eps", _eps, 0.05);
    _nh.param<int>("nearest_min_distance", _nearest_min_distance, 5);
    _nh.param<int>("min_cluster_pts", _min_cluster_pts, 50);

    _nh.param<bool>("spin_once", _spin_once, true);
    _nh.param<double>("ros_rate", _ros_rate, 1.0);
    
    _nh.param<double>("translate_x", _translate_x, 0.0);
    _nh.param<double>("translate_y", _translate_y, 0.0);
    _nh.param<double>("translate_z", _translate_z, 0.0);

    _nh.param<double>("rotate_roll", _rotate_roll, 0.0);
    _nh.param<double>("rotate_pitch", _rotate_pitch, 0.0);
    _nh.param<double>("rotate_yaw", _rotate_yaw, 0.0);

    _nh.param<double>("centroid_x", _centroid_x, 0.0);
    _nh.param<double>("centroid_y", _centroid_y, 0.0);
    _nh.param<double>("centroid_z", _centroid_z, 0.0);

    _nh.param<double>("dimension_x", _dimension_x, 0.0);
    _nh.param<double>("dimension_y", _dimension_y, 0.0);
    _nh.param<double>("dimension_z", _dimension_z, 0.0);

    _nh.param<bool>("crop", _crop, false);
    _nh.param<bool>("transform", _transform, false);
    _nh.param<bool>("cluster", _cluster, false);

    ros::Rate loop_rate(_ros_rate);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); // Initialize the point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr db_pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>); // Initialize the db point cloud
    
    pcl::io::loadPCDFile<pcl::PointXYZ>(_file_location, *cloud);// Load the pcd file
    printf("%s[main.cpp] Loaded .pcd cloud from %s\n", KGRN, _file_location.c_str());

    // Full Cloud
    sensor_msgs::PointCloud2 full_cloud = _common.pcl2ros_converter(cloud);
    printf("%s[main.cpp] Completed full_cloud to ROS message\n", KGRN);

    // Transform Cloud
    sensor_msgs::PointCloud2 transformed_cloud;
    geometry_msgs::TransformStamped transform;
    geometry_msgs::Vector3 trans;
    geometry_msgs::Quaternion q;
    tf2::Quaternion quat_tf;

    double deg2rad = 1.0 / 180.0 * 3.1415926535;

    quat_tf.setRPY(_rotate_roll * deg2rad, 
        _rotate_pitch * deg2rad, 
        _rotate_yaw * deg2rad); // Create this quaternion from roll/pitch/yaw (in radians)
    q = tf2::toMsg(quat_tf);

    trans.x = _translate_x;
    trans.y = _translate_y;
    trans.z = _translate_z;

    transform.transform.translation = trans;
    transform.transform.rotation = q;
    transform.child_frame_id = "/base";
    transform.header.frame_id = "/map";

    tf2::doTransform(full_cloud, transformed_cloud, transform);

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud_pcl = _common.ros2pcl_converter(transformed_cloud);
    printf("%s[main.cpp] Completed transformed_cloud to ROS message\n", KGRN);

    // Cropped Cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud_pcl = _common.pcl2_filter_ptr(transformed_cloud_pcl, 
        Vector3d(_centroid_x, _centroid_y, _centroid_z), Vector3d(_dimension_x, _dimension_y, _dimension_z));
    sensor_msgs::PointCloud2 cropped_cloud = _common.pcl2ros_converter(cropped_cloud_pcl);
    printf("%s[main.cpp] Completed cropped_cloud to ROS message\n", KGRN);

    db_pcl_cloud = cropped_cloud_pcl;
    _dbscan.initialization(db_pcl_cloud, _resolution, _nearest_min_distance, _eps, _min_cluster_pts);

    // Filtering
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_points_cloud = _dbscan.run_filtering();
    sensor_msgs::PointCloud2 filtered_cloud = _common.pcl2ros_converter(filtered_points_cloud);
    printf("%s[main.cpp] Completed filtered_cloud to ROS message\n", KGRN);

    // Clustering
    _dbscan.run_clustering();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clustered_points_cloud = _dbscan.get_clustered_cloud();
    sensor_msgs::PointCloud2 clustered_cloud = _common.pclcolor2ros_converter(clustered_points_cloud);
    printf("%s[main.cpp] Completed clustered_cloud to ROS message\n", KGRN);

    // Choose which altered cloud that you want to publish
    // ros_node_class.pcl_altered = transformed_cloud;
    // ros_node_class.pcl_altered = cropped_cloud;
    ros_node_class.pcl_altered = filtered_cloud;

    ros_node_class.pcl_clustered = clustered_cloud;

    ros_node_class.pcl_original = full_cloud;

    if (_spin_once)
    {
        ros_node_class.full_pcl_publisher();      
        ros_node_class.altered_pcl_publisher();
        ros_node_class.cluster_pcl_publisher();

        ros::spinOnce();
        return 0;
    }
    
    while (ros::ok())
    {       
        ros_node_class.full_pcl_publisher();
        ros_node_class.altered_pcl_publisher();
        ros_node_class.cluster_pcl_publisher();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;

    
}
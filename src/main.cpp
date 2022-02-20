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

#include <dynamic_reconfigure/server.h>
#include "pcl_ros_lib/pcl_parametersConfig.h"

#define KNRM  "\033[0m"
#define KRED  "\033[31m"
#define KGRN  "\033[32m"
#define KYEL  "\033[33m"
#define KBLU  "\033[34m"
#define KMAG  "\033[35m"
#define KCYN  "\033[36m"
#define KWHT  "\033[37m"

#include "pcl_ros_lib/point_array.h"
#include "pcl_ros_lib/pointcloud2_array.h"

using namespace std;

common_utility _common;

class ros_node_class
{
private:
    ros::NodeHandle _nh;
    ros::Publisher full_pcl_pub, altered_pcl_pub, cluster_pcl_pub, 
        object_point_pub, cluster_array_pub;

public:
    sensor_msgs::PointCloud2 pcl_original;
    sensor_msgs::PointCloud2 pcl_altered;
    sensor_msgs::PointCloud2 pcl_clustered;
    vector<geometry_msgs::Point> center_points;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc;

    vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> main_store_cluster_pc;

    double _resolution; // Resolution of octree
    Vector3d _translate, _rotate; // In degrees
    Vector3d _max, _min;
    int _nearest_min_distance;
    double _eps; // The radius for searching neighbor points of octree
    int _min_cluster_pts;

    bool ready_to_load;


    ros_node_class(ros::NodeHandle &nodeHandle, ros::NodeHandle &private_nh)
    {
        full_pcl_pub = _nh.advertise<sensor_msgs::PointCloud2>("/full_pcl", 10);
        altered_pcl_pub = _nh.advertise<sensor_msgs::PointCloud2>("/altered_pcl", 10);
        cluster_pcl_pub = _nh.advertise<sensor_msgs::PointCloud2>("/cluster_pcl", 10);
        object_point_pub = _nh.advertise<pcl_ros_lib::point_array>("/object_points", 10);
        cluster_array_pub = _nh.advertise<pcl_ros_lib::pointcloud2_array>("/clustered_array_pcl", 10);
        printf("%s[main.cpp] Constructor Setup Ready! \n", KGRN);
    }
    ~ros_node_class(){};

    void dynamic_reconfigure_server(pcl_ros_lib::pcl_parametersConfig &config, uint32_t level)
    {
        ROS_INFO("Reconfigure Request [octree]: %f %f %d %d", 
            config.resolution, config.eps, 
            config.nearest_min_distance, config.min_cluster_pts);
        // ROS_INFO("Reconfigure Request [transform]: [%f %f %f] [%f %f %f]", 
        //     config.x, config.y, config.z, 
        //     config.roll, config.pitch, config.yaw);
        
        // Only if next iteration, cannot corrupt the previous data

        _resolution = config.resolution;
        _eps = config.eps;
        _nearest_min_distance = config.nearest_min_distance;
        _min_cluster_pts = config.min_cluster_pts;

        _translate = Vector3d (config.x, config.y, config.z);
        // _rotate = Vector3d (config.roll, config.pitch, config.yaw);
        _max = Vector3d (config.max_x, config.max_y, config.max_z);
        _min = Vector3d (config.min_x, config.min_y, config.min_z);

        ready_to_load = true;
    }

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

    void object_center_publisher() 
    {
        pcl_ros_lib::point_array msg;
        for (int i = 0; i < center_points.size(); i++)
            msg.array.push_back(center_points[i]);

        object_point_pub.publish(msg); 
        // printf("%s[main.cpp] Published object_point! \n", KGRN);
    }

    void custom_cluster_publisher()
    {
        pcl_ros_lib::pointcloud2_array msg;
        for (int i=0; i < main_store_cluster_pc.size(); i++)
        {
            sensor_msgs::PointCloud2 pcl_cluster = _common.pcl2ros_converter(main_store_cluster_pc[i]);
            msg.array.push_back(pcl_cluster);
        }
        cluster_array_pub.publish(msg);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_ros_lib_node");
    ros::NodeHandle _nh("~"); 
    ros::NodeHandle priv_nh("~");
    ros_node_class ros_node_class(_nh, priv_nh);

    // Document for dynamic_reconfigure
    // https://answers.ros.org/question/340040/dynamic-reconfigure-c-boostbind-error/
    // http://wiki.ros.org/dynamic_reconfigure/Tutorials/SettingUpDynamicReconfigureForANode%28cpp%29
    dynamic_reconfigure::Server<pcl_ros_lib::pcl_parametersConfig> server;
    dynamic_reconfigure::Server<pcl_ros_lib::pcl_parametersConfig>::CallbackType f;

    f = boost::bind(&ros_node_class::dynamic_reconfigure_server, &ros_node_class, _1, _2);
    server.setCallback(f);
    
    // ROS Params
    std::string _file_location;

    double _ros_rate;

    bool _crop, _transform, _cluster;
    bool _spin_once;

    _nh.param<std::string>("file_location", _file_location, "/cloud.pcd");
    _nh.param<double>("resolution", ros_node_class._resolution, 1.0);
    _nh.param<double>("eps", ros_node_class._eps, 0.05);
    _nh.param<int>("nearest_min_distance", ros_node_class._nearest_min_distance, 5);
    _nh.param<int>("min_cluster_pts", ros_node_class._min_cluster_pts, 50);

    _nh.param<bool>("spin_once", _spin_once, true);
    _nh.param<double>("ros_rate", _ros_rate, 1.0);
    
    _nh.param<double>("translate_x", ros_node_class._translate.x(), 0.0);
    _nh.param<double>("translate_y", ros_node_class._translate.y(), 0.0);
    _nh.param<double>("translate_z", ros_node_class._translate.z(), 0.0);

    _nh.param<double>("rotate_roll", ros_node_class._rotate.x(), 0.0);
    _nh.param<double>("rotate_pitch", ros_node_class._rotate.y(), 0.0);
    _nh.param<double>("rotate_yaw", ros_node_class._rotate.z(), 0.0);

    _nh.param<double>("min_x", ros_node_class._min.x(), 0.0);
    _nh.param<double>("min_y", ros_node_class._min.y(), 0.0);
    _nh.param<double>("min_z", ros_node_class._min.z(), 0.0);

    _nh.param<double>("max_x", ros_node_class._max.x(), 0.0);
    _nh.param<double>("max_y", ros_node_class._max.y(), 0.0);
    _nh.param<double>("max_z", ros_node_class._max.z(), 0.0);

    _nh.param<bool>("crop", _crop, false);
    _nh.param<bool>("transform", _transform, false);
    _nh.param<bool>("cluster", _cluster, false);

    ros::Rate loop_rate(_ros_rate);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); // Initialize the point cloud
    
    pcl::io::loadPCDFile<pcl::PointXYZ>(_file_location, *cloud);// Load the pcd file
    printf("%s[main.cpp] Loaded .pcd cloud from %s\n", KGRN, _file_location.c_str());

    // Full Cloud
    sensor_msgs::PointCloud2 full_cloud = _common.pcl2ros_converter(cloud);
    printf("%s[main.cpp] Completed full_cloud to ROS message\n", KGRN);

    double _resolution = ros_node_class._resolution; 

    Vector3d _translate = ros_node_class._translate; 
    Vector3d _rotate = ros_node_class._rotate;

    Vector3d _min = ros_node_class._min;
    Vector3d _max = ros_node_class._max;

    int _nearest_min_distance = ros_node_class._nearest_min_distance;
    double _eps = ros_node_class._eps; // The radius for searching neighbor points of octree
    int _min_cluster_pts = ros_node_class._min_cluster_pts;

    while (ros::ok())
    {
        dbscan _dbscan;
        if (ros_node_class.ready_to_load)
        {
            // Only update from every cycle to not corrupt the data inbetween
            _resolution = ros_node_class._resolution; 

            _translate = ros_node_class._translate; 
            // _rotate = ros_node_class._rotate;
            _max = ros_node_class._max;
            _min = ros_node_class._min;
            
            _nearest_min_distance = ros_node_class._nearest_min_distance;
            _eps = ros_node_class._eps; // The radius for searching neighbor points of octree
            _min_cluster_pts = ros_node_class._min_cluster_pts;

            ros_node_class.ready_to_load = false;
            printf("%s[main.cpp] Loaded from rqt_reconfigure\n", KGRN);

        }
        sensor_msgs::PointCloud2 clear;
        ros_node_class.pcl_altered = clear;
        ros_node_class.pcl_clustered = clear;
        ros_node_class.pcl_original = clear;

        ros_node_class.main_store_cluster_pc.clear();
        ros_node_class.center_points.clear();
        pcl::PointCloud<pcl::PointXYZ>::Ptr db_pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>); // Initialize the db point cloud

        // Transform Cloud
        sensor_msgs::PointCloud2 transformed_cloud;
        geometry_msgs::TransformStamped transform;
        geometry_msgs::Vector3 trans;
        geometry_msgs::Quaternion q;
        tf2::Quaternion quat_tf;

        double deg2rad = 1.0 / 180.0 * 3.1415926535;

        quat_tf.setRPY(_rotate.x() * deg2rad, 
            _rotate.y() * deg2rad, 
            _rotate.z() * deg2rad); // Create this quaternion from roll/pitch/yaw (in radians)
        q = tf2::toMsg(quat_tf);

        trans.x = _translate.x();
        trans.y = _translate.y();
        trans.z = _translate.z();

        transform.transform.translation = trans;
        transform.transform.rotation = q;
        transform.child_frame_id = "/base";
        transform.header.frame_id = "/map";

        tf2::doTransform(full_cloud, transformed_cloud, transform);

        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud_pcl = _common.ros2pcl_converter(transformed_cloud);
        printf("%s[main.cpp] Completed transformed_cloud to ROS message\n", KGRN);
        size_t transformed_len = transformed_cloud_pcl->points.size();
        printf("%s[main.cpp] transformed_cloud_pcl size = %d\n", KGRN, transformed_len);

        // Cropped Cloud
        // pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud_pcl = _common.pcl2_filter_ptr(transformed_cloud_pcl, 
        //     _centroid, dimension);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud_pcl = _common.pcl2_filter_ptr_original(transformed_cloud_pcl, 
            _max, _min);
        
        sensor_msgs::PointCloud2 cropped_cloud = _common.pcl2ros_converter(cropped_cloud_pcl);
        printf("%s[main.cpp] Completed cropped_cloud to ROS message\n", KGRN);
        size_t cropped_len = cropped_cloud_pcl->points.size();
        printf("%s[main.cpp] cropped_cloud_pcl size = %d\n", KGRN, cropped_len);

        db_pcl_cloud = cropped_cloud_pcl;
        _dbscan.initialization(db_pcl_cloud, _resolution, _nearest_min_distance, 
            (float)_eps, _min_cluster_pts);

        // Filtering
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_points_cloud = _dbscan.run_filtering(db_pcl_cloud);
        sensor_msgs::PointCloud2 filtered_cloud = _common.pcl2ros_converter(filtered_points_cloud);
        printf("%s[main.cpp] Completed filtered_cloud to ROS message\n", KGRN);
        size_t filtered_len = filtered_points_cloud->points.size();
        printf("%s[main.cpp] filtered_points_pcl size = %d\n", KGRN, filtered_len);

        // Clustering
        _dbscan.run_clustering(filtered_points_cloud);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr clustered_points_cloud = _dbscan.get_clustered_cloud();
        printf("%s[main.cpp] Completed Clustering from dbscan\n", KGRN);
        sensor_msgs::PointCloud2 clustered_cloud = _common.pclcolor2ros_converter(clustered_points_cloud);
        printf("%s[main.cpp] Completed clustered_cloud to ROS message\n", KGRN);

        // Choose which altered cloud that you want to publish
        // ros_node_class.pcl_altered = transformed_cloud;
        // ros_node_class.pcl_altered = cropped_cloud;
        ros_node_class.pcl_altered = filtered_cloud;

        ros_node_class.pcl_clustered = clustered_cloud;

        ros_node_class.pcl_original = full_cloud;

        int total_cluster_size = _dbscan.get_cluster_pc_size();
        for (int i = 0; i < total_cluster_size; i++)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr test_cloud = _dbscan.get_cluster_pc_info(i);
            printf("%s[main.cpp] Query cloud idx %d with cluster cloud size %lu\n", KGRN, i, test_cloud->points.size());
            Vector3d centroid = _common.normalize_all_points(test_cloud);
            printf("%s[main.cpp] Centroid of Query cloud idx %d = %lf %lf %lf\n", KMAG, i, centroid.x(), centroid.y(), centroid.z());
            geometry_msgs::Point tmp_point;
            tmp_point.x = centroid.x();
            tmp_point.y = centroid.y();
            tmp_point.z = centroid.z();
            ros_node_class.center_points.push_back(tmp_point);
        }
        ros_node_class.main_store_cluster_pc = _dbscan.get_cluster_pc();

        ros_node_class.full_pcl_publisher();
        ros_node_class.altered_pcl_publisher();
        ros_node_class.cluster_pcl_publisher();
        ros_node_class.object_center_publisher();
        ros_node_class.custom_cluster_publisher();

        ros::spinOnce();

        loop_rate.sleep();

        if (_spin_once)
        {
            return 0;
        }
    }

    return 0;

    
}
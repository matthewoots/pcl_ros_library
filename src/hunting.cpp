/*
 * hunting.cpp
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

class hunting_class
{
private:
    ros::NodeHandle _nh;
    ros::Publisher altered_pcl_pub, object_point_pub;
    ros::Subscriber pcl_array_sub;

public:
    vector<geometry_msgs::Point> centroid_points;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pc;

    vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> store_cluster_pc;


    hunting_class(ros::NodeHandle &nodeHandle)
    {
        pcl_array_sub = _nh.subscribe<pcl_ros_lib::pointcloud2_array>("/clustered_array_pcl", 1, &hunting_class::pclCallBack, this);  
        object_point_pub = _nh.advertise<pcl_ros_lib::point_array>("/sub_object_points", 10);     

        printf("%s[hunter.cpp] Constructor Setup Ready! \n", KGRN);
    }
    ~hunting_class(){};

    void pclCallBack(const pcl_ros_lib::pointcloud2_array::ConstPtr &msg) 
    {
        for (int i=0; i < msg->array.size(); i++)
        {
            sensor_msgs::PointCloud2 pcl_cluster;
            pcl_cluster = msg->array[i];
            store_cluster_pc.push_back(_common.ros2pcl_converter(pcl_cluster));
        }
    }

    
    // void altered_pcl_publisher() 
    // {
    //     pcl_altered.header.frame_id = "/map";
    //     altered_pcl_pub.publish(pcl_altered); 
    //     // printf("%s[main.cpp] Published altered_cloud! \n", KGRN);
    // }

    // void cluster_pcl_publisher() 
    // {
    //     pcl_clustered.header.frame_id = "/map";
    //     cluster_pcl_pub.publish(pcl_clustered); 
    //     // printf("%s[main.cpp] Published clustered_cloud! \n", KGRN);
    // }

    void object_center_publisher() 
    {
        pcl_ros_lib::point_array msg;
        for (int i = 0; i < centroid_points.size(); i++)
            msg.array.push_back(centroid_points[i]);

        object_point_pub.publish(msg); 
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_ros_lib_hunting_node");
    ros::NodeHandle _nh("~"); 
    hunting_class hunting_class(_nh);

    
    
    // ROS Params
    double _height_resolution, _start_search_height, _last_search_height;

    double _resolution;
    int _nearest_min_distance;
    double _eps; // The radius for searching neighbor points of octree
    int _min_cluster_pts;
    double _ros_rate;
    string _file_location;

    _nh.param<std::string>("file_location", _file_location, "~/.csv");
    _nh.param<double>("height_resolution", _height_resolution, 0.1);
    _nh.param<double>("start_search_height", _start_search_height, 0);
    _nh.param<double>("final_search_height", _last_search_height, 0);

    _nh.param<double>("resolution", _resolution, 1.0);
    _nh.param<double>("eps", _eps, 0.05);
    _nh.param<int>("nearest_min_distance", _nearest_min_distance, 0.5);
    _nh.param<int>("min_cluster_pts", _min_cluster_pts, 5);   

    _nh.param<double>("ros_rate", _ros_rate, 1.0);
    ros::Rate loop_rate(_ros_rate);

    while (ros::ok())
    {
        hunting_class.centroid_points.clear();
        vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster = 
            hunting_class.store_cluster_pc;
        
        if (cluster.size() > 0)
        {
            double deg2rad = 1.0 / 180.0 * 3.1415926535;
            dbscan _dbscan;

            // Get from csv the pointclouds we want to query
            vector<int> idx;
            if (!_common.UnpackIndex(&idx, _file_location))
            {
                printf("%s[hunting.cpp] Not able to load file and get idx! \n", KRED);
                break;
            }

            int idx_size = idx.size();
            printf("%s[hunting.cpp] idx size %d\n", KBLU, idx_size);
            double height_search = (_last_search_height - _start_search_height) / _height_resolution;
            int height_idx = (int)ceil(height_search);

            // For loop for each idx chosen
            for (int i = 0; i < idx_size; i++)
            {
                vector<Vector3d> center_vectors_array;
                vector<Vector3d> vector_array;
                
                // Loop for each height
                for (int j = 0; j < height_idx; j++)
                {                    
                    double height_start_range = _start_search_height + _height_resolution * j;
                    double height_end_range = _start_search_height + _height_resolution * (j + 1);
    
                    printf("%s[hunting.cpp] height_search id: %s %d %s height: %s %.2lf %sresolution %s %.2lf\n", KGRN, KNRM, j, KGRN, KNRM, 
                    height_start_range, KGRN, KNRM,
                    _height_resolution);

                    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud_pcl = _common.pcl2_filter_ptr_original(cluster[idx[i]], 
                        Vector3d(100,100,height_end_range), Vector3d(-100,-100,height_start_range));

                    _dbscan.initialization(cropped_cloud_pcl, _resolution, _nearest_min_distance, 
                        (float)_eps, _min_cluster_pts);

                    // Filtering
                    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_points_cloud = _dbscan.run_filtering(cropped_cloud_pcl);

                    size_t filtered_len = filtered_points_cloud->points.size();
                    printf("%s[hunting.cpp] filtered_points_pcl size = %d\n", KGRN, filtered_len);

                    // Clustering
                    _dbscan.run_clustering(filtered_points_cloud);
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clustered_points_cloud = _dbscan.get_clustered_cloud();
                    // printf("%s[hunting.cpp] Completed Clustering from dbscan\n", KGRN);
                    sensor_msgs::PointCloud2 clustered_cloud = _common.pclcolor2ros_converter(clustered_points_cloud);
                    printf("%s[hunting.cpp] Completed clustered_cloud to ROS message\n", KGRN);

                    int total_cluster_size = _dbscan.get_cluster_pc_size();
                    if (total_cluster_size != 2)
                        continue;
                    
                    vector<Vector3d> center_points;
                    for (int i = 0; i < total_cluster_size; i++)
                    {
                        pcl::PointCloud<pcl::PointXYZ>::Ptr test_cloud = _dbscan.get_cluster_pc_info(i);

                        Vector3d centroid = _common.normalize_all_points(test_cloud);
                        printf("%s[hunter.cpp] Centroid of Query cloud idx %d = %lf %lf %lf\n", KMAG, i, centroid.x(), centroid.y(), centroid.z());

                        center_points.push_back(centroid);
                    }

                    Vector3d center_vectors = (center_points[0] + center_points[1]) / 2;
                    Vector3d dir = center_points[0] + center_points[1];
                    Vector3d dir_vectors =  dir / dir.norm();

                    geometry_msgs::Point tmp_point;
                    tmp_point.x = center_vectors.x();
                    tmp_point.y = center_vectors.y();
                    tmp_point.z = center_vectors.z();
                    hunting_class.centroid_points.push_back(tmp_point);

                    center_vectors_array.push_back(center_vectors);
                    vector_array.push_back(dir_vectors);

                    // ros_node_class.full_pcl_publisher();
                    // ros_node_class.altered_pcl_publisher();
                    // ros_node_class.cluster_pcl_publisher();
                    hunting_class.object_center_publisher();
                }

                int query_size =  center_vectors_array.size();

                // We handle vector and centroid inside this loop
                for (int k = 0; k < query_size; k++)
                {

                }
            }
        }
        printf("%s[hunting.cpp] height_search centroid size: %s %d\n", KGRN, KNRM, hunting_class.centroid_points.size());

        ros::spinOnce();

        loop_rate.sleep();

    }

    return 0;

    
}
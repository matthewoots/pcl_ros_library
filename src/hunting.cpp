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
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

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

    vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> altered_cluster_pc;

    sensor_msgs::PointCloud2 pcl_altered;

    hunting_class(ros::NodeHandle &nodeHandle)
    {
        pcl_array_sub = _nh.subscribe<pcl_ros_lib::pointcloud2_array>("/clustered_array_pcl", 1, &hunting_class::pclCallBack, this);  
        object_point_pub = _nh.advertise<pcl_ros_lib::point_array>("/sub_object_points", 10);
        altered_pcl_pub = _nh.advertise<sensor_msgs::PointCloud2>("/wall_pcl", 10);    

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

    
    void altered_pcl_publisher() 
    {
        pcl_altered.header.frame_id = "/map";
        altered_pcl_pub.publish(pcl_altered); 
        // printf("%s[main.cpp] Published altered_cloud! \n", KGRN);
    }

    void sub_object_center_publisher() 
    {
        pcl_ros_lib::point_array msg;
        for (int i = 0; i < centroid_points.size(); i++)
            msg.array.push_back(centroid_points[i]);

        object_point_pub.publish(msg); 
    }
};

// Used for pcl2 and adapted from 
// https://pcl.readthedocs.io/projects/tutorials/en/latest/kdtree_search.html
double kdtree_nearest_point(Vector3d point, pcl::PointCloud<pcl::PointXYZ>::Ptr _pc)
{
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

    kdtree.setInputCloud(_pc);

    pcl::PointXYZ searchPoint;
    searchPoint.x = point.x();
    searchPoint.y = point.y();
    searchPoint.z = point.z();

    // K nearest neighbor search

    int K = 1;

    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
    {
        for (std::size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
        {
            // std::cout << "    "  <<   (*_pc)[ pointIdxNKNSearch[i] ].x 
            //             << " " << (*_pc)[ pointIdxNKNSearch[i] ].y 
            //             << " " << (*_pc)[ pointIdxNKNSearch[i] ].z 
            //             << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
        }
    }

    return sqrt(pointNKNSquaredDistance[0]);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_make_wall_with_gap(int height_idx, double height_lower_bound,
    int width_idx, double width_lower_bound,
    int length_idx, double length_lower_bound, double radius)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr wall(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < height_idx; i++)
    {
        for (int j = 0; j < width_idx; j++)
        {
            for (int k = 0; k < length_idx; k++)
            {
                pcl::PointXYZ point;
                point.x = width_lower_bound + width_idx*j;
                point.y = length_lower_bound + length_idx*k;
                point.z = height_lower_bound + height_idx*i;

                if (pow(point.x,2) + pow(point.y,2) + pow(point.z,2) > pow(radius,2))
                {
                    wall->points.push_back(point);
                } 
            }
        }
    }
    return wall;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_ros_lib_hunting_node");
    ros::NodeHandle _nh("~"); 
    hunting_class hunting_class(_nh);    
    
    // ROS Params
    double _height_resolution, _start_search_height, _last_search_height;

    double _resolution, _eps; // The radius for searching neighbor points of octree
    int _nearest_min_distance, _min_cluster_pts;

    double _ros_rate;
    string _file_location;

    double _wall_min_height, _wall_max_height, _wall_thickness, 
        _wall_resolution, _wall_length;

    _nh.param<std::string>("file_location", _file_location, "~/.csv");
    _nh.param<double>("height_resolution", _height_resolution, 0.1);
    _nh.param<double>("start_search_height", _start_search_height, 0);
    _nh.param<double>("final_search_height", _last_search_height, 0);

    _nh.param<double>("resolution", _resolution, 1.0);
    _nh.param<double>("eps", _eps, 0.05);
    _nh.param<int>("nearest_min_distance", _nearest_min_distance, 0.5);
    _nh.param<int>("min_cluster_pts", _min_cluster_pts, 5); 

    _nh.param<double>("wall_min_height", _wall_min_height, 0.0);
    _nh.param<double>("wall_max_height", _wall_max_height, 0.01);
    _nh.param<double>("wall_length", _wall_length, 5.0);
    _nh.param<double>("wall_thickness", _wall_thickness, 1.0);
    _nh.param<double>("wall_resolution", _wall_resolution, 1.0);  

    _nh.param<double>("ros_rate", _ros_rate, 1.0);
    ros::Rate loop_rate(_ros_rate);

    while (ros::ok())
    {
        hunting_class.centroid_points.clear();
        hunting_class.altered_cluster_pc.clear();
        vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster = 
            hunting_class.store_cluster_pc;
        double deg2rad = 1.0 / 180.0 * 3.1415926535;
        
        if (cluster.size() > 0)
        {
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
                vector<double> yaw_array;
                printf("%s[hunting.cpp] Current Index %s %d\n", KBLU, KNRM, idx[i]);
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

                    // Move on if there is no 2 clusters
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
                    Vector3d dir = center_points[0] - center_points[1];
                    Vector3d dir_vectors =  dir / dir.norm();
                    double calculated_yaw_deg = atan2(dir_vectors.y(), dir_vectors.x()) / 3.1415926535 * 180;
                    
                    // We force yaw to be positive
                    if (calculated_yaw_deg < 0)
                        calculated_yaw_deg = 180.0 + calculated_yaw_deg;
                    printf("%s[hunter.cpp] calculated_yaw %d: %lf \n", KBLU, j, calculated_yaw_deg);

                    geometry_msgs::Point tmp_point;
                    tmp_point.x = center_vectors.x();
                    tmp_point.y = center_vectors.y();
                    tmp_point.z = center_vectors.z();
                    hunting_class.centroid_points.push_back(tmp_point);

                    center_vectors_array.push_back(center_vectors);
                    yaw_array.push_back(calculated_yaw_deg);

                    // ros_node_class.full_pcl_publisher();
                    // ros_node_class.altered_pcl_publisher();
                    // ros_node_class.cluster_pcl_publisher();
                    hunting_class.sub_object_center_publisher();
                }

                int query_size =  center_vectors_array.size();
                printf("%s[hunter.cpp] query_size %d: %s %lf \n", KBLU, i, KNRM, (double)query_size);
                
                Vector3d avg_centroid = Vector3d(0,0,0), true_centroid; 
                double avg_yaw = 0.0, true_yaw;
                // We handle vector and centroid inside this loop
                for (int k = 0; k < query_size; k++)
                {
                    avg_centroid += center_vectors_array[k];
                    avg_yaw += yaw_array[k];
                    // printf("%s[hunter.cpp] yaw_array_indi %d: %s %lf \n", KBLU, k, KNRM, yaw_array[k]);
                }

                true_centroid = avg_centroid / query_size;
                true_yaw = avg_yaw / (double)query_size;
                printf("%s[hunter.cpp] true_yaw %d: %s %lf \n", KBLU, i, KNRM, true_yaw);

                // Find radius of sphere
                double sphere_radius = kdtree_nearest_point(true_centroid, cluster[idx[i]]);
                
                printf("%s[hunter.cpp] sphere_radius %d: %s %lf \n", KBLU, i, KNRM, sphere_radius);
                // Add safety region
                sphere_radius -= 0.15;

                // Let us place the centroid at the origin to make the wall
                double suggested_height = avg_centroid.z();
                int height_idx = (int)ceil((_wall_max_height - _wall_min_height) / _wall_resolution);
                double lower_wall_boundary = - suggested_height + _wall_min_height;
                // X axis
                int width_idx = (int)ceil(_wall_thickness / _wall_resolution);
                double lower_width_boundary = -_wall_thickness / 2.0;
                // Y axis
                int length_idx = (int)ceil(_wall_length / _wall_resolution);
                double lower_length_boundary = -_wall_length / 2.0;
                
                printf("%s[hunter.cpp] height_idx %d, lower_wall_boundary %lf \n", KBLU, height_idx, lower_wall_boundary);
                printf("%s[hunter.cpp] width_idx %d, lower_width_boundary %lf \n", KBLU, width_idx, lower_width_boundary);
                printf("%s[hunter.cpp] length_idx %d, lower_length_boundary %lf \n", KBLU, length_idx, lower_length_boundary);

                // Time to make a wall
                pcl::PointCloud<pcl::PointXYZ>::Ptr wall_pcl = pcl_make_wall_with_gap(height_idx, lower_wall_boundary,
                    width_idx, lower_width_boundary,
                    length_idx, lower_length_boundary, sphere_radius);

                pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_wall = _common.base_to_transform_pcl(wall_pcl, Vector3d(0,0,true_yaw), true_centroid);

                hunting_class.altered_cluster_pc.push_back(transformed_wall);
            }

            // Let us build a point cloud message from all the query points
            
            pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_altered_msg(new pcl::PointCloud<pcl::PointXYZ>);
            for (int l = 0; l < hunting_class.altered_cluster_pc.size(); l++)
            {
                for (int m = 0; m < hunting_class.altered_cluster_pc[l]->points.size(); m++)
                {
                    tmp_altered_msg->points.push_back(hunting_class.altered_cluster_pc[l]->points[m]);
                }
            }

            hunting_class.pcl_altered = _common.pcl2ros_converter(tmp_altered_msg);
            hunting_class.altered_pcl_publisher();
        }
        
        printf("%s[hunting.cpp] height_search centroid size: %s %d\n", KGRN, KNRM, hunting_class.centroid_points.size());

        ros::spinOnce();

        loop_rate.sleep();

    }

    return 0;

    
}
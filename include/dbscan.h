/*
 * dbscan.h
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

#include "common_utils.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <ros/ros.h>
#include <Eigen/Dense>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Point.h>

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

using namespace std;

class point
{
    public:
        float x;
        float y;
        float z;
        int visited = 0;
        int cluster = 0;
        int index = 0;
        vector<int> corepts;
        vector<int> neighbor_pts;
        point() {}
        point(float a, float b, float c)
        {
            x = a;
            y = b;
            z = c;
        }
};

class dbscan
{
    private:
        common_utility _common;
        vector<point> core_cloud;
        vector<point> full_point_cloud;
        vector<vector<int>> color;

        vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> store_cluster_pc; // Initialize the store point cloud

        double _resolution; // Resolution of octree
        int _nearest_min_distance;
        float _eps; // The radius for searching neighbor points of octree
        int _min_cluster_pts;
        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud; // Initialize the point cloud
        // pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_points_cloud; // Initialize the filtered core point cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud; // Initialize the cluster point cloud

    public:
        // constructor
        dbscan()
        {
            cout<<"Constructor called"<<endl;
        }
    
        // destructor
        ~dbscan()
        {
            cout<<"Destructor called"<<endl;
        }

        void initialization(pcl::PointCloud<pcl::PointXYZ>::Ptr pc, 
            double resolution, int nearest_min_distance, float eps, int min_cluster_pts)
        {
            // filtered_points_cloud.reset();
            cluster_cloud.reset();
            // cloud.reset();
            printf("%s[dbscan.h] Clear clouds to initialize\n", KBLU);
            store_cluster_pc.clear();
            full_point_cloud.clear();
            core_cloud.clear();
            color.clear();
            
            // cloud = pc;
            _resolution = resolution;
            _nearest_min_distance = nearest_min_distance;
            _eps = eps;
            _min_cluster_pts = min_cluster_pts;
            printf("%s[dbscan.h] Initialized\n", KBLU);
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr run_filtering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_points_cloud; // Initialize the filtered core point cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_points_cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>); // Initialize the tmp core point cloud
            pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(_resolution); // Initialize the octree
            printf("%s[dbscan.h] Initialize octree \n", KBLU);

            octree.setInputCloud(cloud);
            octree.addPointsFromInputCloud();
            printf("%s[dbscan.h] Added points to octree \n", KBLU);

            size_t len = cloud->points.size();
            for (size_t i = 0; i < len; i++) // Convert the type of points to class point
            {
                    point pt = point(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
                    full_point_cloud.push_back(pt);
            }
            printf("%s[dbscan.h] Converted to points class\n", KBLU);
            printf("%s[dbscan.h] full_point_cloud size %lu\n", KBLU, full_point_cloud.size());

            // Find the core points and put them into the core_cloud(vector<point>)
            // Getting the map that is cleaner, this stage removes noise
            for (size_t i = 0; i < len; i++)
            {
                vector<int> radiussearch; // store the index of neighbor points
                vector<float> radiusdistance; // store the square of distance of neighbor points
                octree.radiusSearch(cloud->points[i], _eps, radiussearch, radiusdistance);//neighborhood research of octree
                // printf("%s[dbscan.h] Octree radiusSearch succeed %lu \n", KBLU, i);

                // This is to just remove any outlier/noise
                if (radiussearch.size() > _nearest_min_distance)
                { 
                    full_point_cloud[i].index = i; // Store the index of core points in the full_point_cloud
                    core_cloud.push_back(full_point_cloud[i]);
                    full_point_cloud[i].neighbor_pts = radiussearch; // Store the index of neighbor points in the full_point_cloud
                }
            }

            filtered_points_cloud_tmp->points.resize(core_cloud.size());
            filtered_points_cloud = filtered_points_cloud_tmp;

            printf("%s[dbscan.h] Extracting core cloud \n", KBLU);
            //copy the coordinate of points of core_cloud to that of filtered_points_cloud
            for (int i = 0; i < core_cloud.size(); i++)
            {
                    filtered_points_cloud->points[i].x = core_cloud[i].x;
                    filtered_points_cloud->points[i].y = core_cloud[i].y;
                    filtered_points_cloud->points[i].z = core_cloud[i].z;
            }
            printf("%s[dbscan.h] Complete filtering core cloud \n", KBLU);
            return filtered_points_cloud;
        }   

        void run_clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_points_cloud)
        {
            size_t len = filtered_points_cloud->points.size();
            
            pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(_resolution);
            octree.setInputCloud(filtered_points_cloud);
            octree.addPointsFromInputCloud();

            // Find the density reachable points for each core point
            for (int i = 0; i < core_cloud.size(); i++)
            {
                vector<int> pointIdxNKNSearch;
                vector<float> pointRadiusSquaredDistance;
                octree.radiusSearch(filtered_points_cloud->points[i], _eps, pointIdxNKNSearch, pointRadiusSquaredDistance);
                for (int j = 0; j < pointIdxNKNSearch.size(); j++)
                {
                    core_cloud[i].corepts.push_back(pointIdxNKNSearch[j]);
                }
            }
            printf("%s[dbscan.h] Find the neighbor core points \n", KBLU);

            // Change the value of cluster for each core point according to if the neighbor core point is density reachable
            int outcluster = 0;
            vector<int> cluster_pts_num;
            for (int i = 0; i < core_cloud.size(); i++)
            {
                int pts_num = 0;
                stack<point*> ps;
                
                // If the core point is visited already we can skip it
                if (core_cloud[i].visited == 1) continue;

                core_cloud[i].cluster = outcluster;

                ps.push(&core_cloud[i]);

                // Change the value of cluster for each point
                point *v;
                while (!ps.empty())
                {
                    v = ps.top();
                    v->visited = 1;
                    full_point_cloud[v->index].visited = 1;
                    for(int pts_i = 0; pts_i < full_point_cloud[v->index].neighbor_pts.size(); pts_i++)
                    {
                        if(full_point_cloud[full_point_cloud[v->index].neighbor_pts[pts_i]].visited == 1)
                            continue;
                        else
                        {
                            full_point_cloud[full_point_cloud[v->index].neighbor_pts[pts_i]].cluster = outcluster;
                            full_point_cloud[full_point_cloud[v->index].neighbor_pts[pts_i]].visited = 1;
                            pts_num++;
                        }

                    }
                    ps.pop();
                    for (int j = 0; j<v->corepts.size(); j++)
                    {
                        if (core_cloud[v->corepts[j]].visited == 1)
                            continue;
                        core_cloud[v->corepts[j]].cluster = core_cloud[i].cluster;
                        core_cloud[v->corepts[j]].visited = 1;
                        full_point_cloud[core_cloud[v->corepts[j]].index].cluster = outcluster;
                        full_point_cloud[core_cloud[v->corepts[j]].index].visited = 1;
                        pts_num++;
                        for(int pts_i = 0; pts_i < full_point_cloud[core_cloud[v->corepts[j]].index].neighbor_pts.size(); pts_i++)
                        {
                            if(full_point_cloud[full_point_cloud[core_cloud[v->corepts[j]].index].neighbor_pts[pts_i]].visited == 1)
                                continue;
                            else
                            {
                                full_point_cloud[full_point_cloud[core_cloud[v->corepts[j]].index].neighbor_pts[pts_i]].cluster = outcluster;
                                full_point_cloud[full_point_cloud[core_cloud[v->corepts[j]].index].neighbor_pts[pts_i]].visited = 1;
                                pts_num++;
                            }

                        }
                        ps.push(&core_cloud[v->corepts[j]]);
                    }
                }
                // printf("%s[dbscan.h] Cluster %d, Points number %d \n", KBLU, outcluster, pts_num);
                cluster_pts_num.push_back(pts_num);
                outcluster++;
            }

            int cluster_num = 0;
            for (int i = 0; i < cluster_pts_num.size(); i++)
            {
                if(cluster_pts_num[i] > _min_cluster_pts)
                {
                    cluster_num++;
                }
            }
            printf("%s[dbscan.h] Number of clusters %d \n", KBLU, cluster_num);

            color.clear();
            int acceptable_size = 0;

            // Set color for different clusters
            for (int i = 0; i < cluster_pts_num.size(); i++)
            {
                if(cluster_pts_num[i] > _min_cluster_pts)
                {
                    // When cluster satisfy the minimum cluster points
                    vector<int> color_rgb;
                    for (int i = 0; i < 3; i++)
                        color_rgb.push_back(rand() % 255 + 1);

                    color.push_back(color_rgb);
                    // printf("%s[dbscan.h] Cluster %d = Color %d %d %d \n", KBLU, i, color[i][0], color[i][1], color[i][2]);
                    acceptable_size++;
                }
            }
            printf("%s[dbscan.h] Number of clusters with acceptable size %d \n", KBLU, acceptable_size);

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
            cluster_cloud_tmp->points.resize(filtered_points_cloud->points.size());
            cluster_cloud = cluster_cloud_tmp;

            vector<pcl::PointXYZ> pc_cluster_vector[acceptable_size];

            // Color different clusters
            for (size_t i = 0; i < len; i++)
            {
                cluster_cloud->points[i].x = filtered_points_cloud->points[i].x;
                cluster_cloud->points[i].y = filtered_points_cloud->points[i].y;
                cluster_cloud->points[i].z = filtered_points_cloud->points[i].z;

                if(cluster_pts_num[full_point_cloud[i].cluster] > _min_cluster_pts)
                {
                    // When cluster satisfy the minimum cluster points
                    cluster_cloud->points[i].r = color[full_point_cloud[i].cluster][0];
                    cluster_cloud->points[i].g = color[full_point_cloud[i].cluster][1];
                    cluster_cloud->points[i].b = color[full_point_cloud[i].cluster][2];

                    // Store the point clouds into seperate point vectors
                    pc_cluster_vector[full_point_cloud[i].cluster].push_back(filtered_points_cloud->points[i]);
                }
                else 
                {
                    // When cluster is too small or points are filtered out
                    cluster_cloud->points[i].r = 255;
                    cluster_cloud->points[i].g = 255;
                    cluster_cloud->points[i].b = 255;
                }
            }

            store_cluster_pc.resize(acceptable_size);

            size_t total_points = 0; 
            for (size_t i = 0; i < acceptable_size; i++)
            {
                printf("%s[dbscan.h] Cluster %lu of size %lu with color %d %d %d\n", KCYN, i, pc_cluster_vector[i].size(), color[i][0], color[i][1], color[i][2]);
                total_points += pc_cluster_vector[i].size();

                pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_tmp(new pcl::PointCloud<pcl::PointXYZ>);
                cluster_tmp->points.resize(pc_cluster_vector[i].size());
                store_cluster_pc[i] = cluster_tmp;
                
                for (size_t j = 0; j < pc_cluster_vector[i].size(); j++)
                {
                    store_cluster_pc[i]->points[j].x = pc_cluster_vector[i][j].x;
                    store_cluster_pc[i]->points[j].y = pc_cluster_vector[i][j].y;
                    store_cluster_pc[i]->points[j].z = pc_cluster_vector[i][j].z;
                }
            }

            printf("%s[dbscan.h] Total Cluster cloud size %lu\n", KCYN, total_points);            
        }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_clustered_cloud() {return cluster_cloud;}
        
        vector<vector<int>> get_color_identifier() {return color;}

        pcl::PointCloud<pcl::PointXYZ>::Ptr get_cluster_pc_info(int idx) {return store_cluster_pc[idx];}

        size_t get_cluster_pc_size() {return store_cluster_pc.size();}

};
#ifndef _LL_LOCALIZER_MAP_MANAGER_HPP_
#define _LL_LOCALIZER_MAP_MANAGER_HPP_

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>

#include "common_lib.h"
#include "map_manager_utils.hpp"

namespace ll_localizer{

class MapManager {
public:
    MapManager() = default;
    ~MapManager() = default;

    MapManager(ros::NodeHandle &_nh) : nh(_nh){
        nh.param<double>("manager_param/block_size", block_size, 1.0);
        nh.param<double>("manager_param/block_resolution", block_resolution, 0.2);
        nh.param<int>("manager_param/temporary_map_points_threshold", temporary_map_points_threshold, 20);
        
        nh.param<double>("manager_param/global_map_filter_leaf_size", global_map_filter_leaf_size, 0.3);
        nh.param<double>("manager_param/search_radius", search_radius, 0.5);
        nh.param<int>("manager_param/min_search_points_num", min_search_points_num, 3);
        nh.param<std::string>("publish/tf_world_frame", tf_world_frame, "camera_init");

        pub_origin_map = nh.advertise<sensor_msgs::PointCloud2>("/origin_map", 1);
    }

    void AddOriginMap(CloudPtr origin_map) {
        pcl::VoxelGrid<PointType> map_filter;
        map_filter.setLeafSize(global_map_filter_leaf_size, global_map_filter_leaf_size,
                               global_map_filter_leaf_size);
        map_filter.setInputCloud(origin_map);
        map_filter.filter(*origin_map);
        ROS_INFO("\033[1;35m Origin map points num after downsample: %lu \033[0m", origin_map->points.size());
        PubOriginMap(origin_map);

        for (auto &point : origin_map->points) {
            Voxel voxel(point.x, point.y, point.z, block_size);
            VoxelHashMap::iterator search = voxel_hash_map.find(voxel);
            if (search != voxel_hash_map.end()) {
                VoxelBlock &voxel_block = search.value();
                voxel_block.AddOriginPoint(Point3f(point.x, point.y, point.z));
            } else {
                VoxelBlock voxel_block(block_size, block_resolution, temporary_map_points_threshold);
                voxel_block.AddOriginPoint(Point3f(point.x, point.y, point.z));
                voxel_hash_map[voxel] = std::move(voxel_block);
            }
        }

        for (auto it = voxel_hash_map.begin(); it != voxel_hash_map.end(); ++it) {
            auto &voxel_block = it.value();
            voxel_block.InitOriginOctree();
        }

        std::cout << "Global map loaded. Total voxels: " << voxel_hash_map.size() << std::endl;
    }

    void AddPoints(const PointVector &cloud, 
                   pcl::PointCloud<pcl::PointXYZ>::Ptr new_points) {
        for (auto pt : cloud) {
            AddPoint(pt, new_points);
        }
    }

    void AddPoint(const PointType &point, 
                  pcl::PointCloud<pcl::PointXYZ>::Ptr new_points) {
        Voxel voxel(point.x, point.y, point.z, block_size);
        VoxelHashMap::iterator search = voxel_hash_map.find(voxel);
        if (search != voxel_hash_map.end()) {
            VoxelBlock &voxel_block = search.value();
            voxel_block.AddNewPoint(Point3f(point.x, point.y, point.z));

            if (voxel_block.IsTemporaryPointsFull()) {
                voxel_block.UpdateOctree(new_points);
            }
        } else {
            VoxelBlock voxel_block(block_size, block_resolution, temporary_map_points_threshold);
            voxel_block.AddNewPoint(Point3f(point.x, point.y, point.z));
            voxel_hash_map[voxel] = std::move(voxel_block);
        }
    }

    int SearchCorrespondMapPoints(const PointType &point,
                                  PointVector &correspond_points,
                                  pcl::PointCloud<pcl::PointXYZ>::Ptr new_points) {
        Point3f pt3f(point.x, point.y, point.z);

        int search_success = 0;

        Voxel voxel(point.x, point.y, point.z, block_size);
        VoxelHashMap::iterator search = voxel_hash_map.find(voxel);
        if (search != voxel_hash_map.end()) {
            VoxelBlock &voxel_block = search.value();
            if (voxel_block.is_origin) {
                search_success = voxel_block.SearchFromOriginMapPoints(pt3f, search_radius, correspond_points);
                if (search_success) return 3;
            }
    
            search_success = voxel_block.SearchFromStaticMapPoints(pt3f, search_radius, correspond_points);
            if (search_success) return 2;
            
            search_success = voxel_block.SearchFromTemporaryMapPoints(pt3f, search_radius, correspond_points);
            AddPoint(point, new_points);
            if (search_success) return 1;
            else return 0;
            
        } else {
            AddPoint(point, new_points);
            return 0;
        }
    }

    void PubOriginMap(CloudPtr origin_map) {
        sensor_msgs::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*origin_map, laserCloudmsg);
        laserCloudmsg.header.stamp = ros::Time::now();
        laserCloudmsg.header.frame_id = tf_world_frame;
        sleep(1);
        pub_origin_map.publish(laserCloudmsg);
    }

private:
    VoxelHashMap voxel_hash_map;
    double block_size;
    double block_resolution;
    int temporary_map_points_threshold;
    double global_map_filter_leaf_size ;
    double search_radius;
    int min_search_points_num;

    ros::NodeHandle nh;
    ros::Publisher pub_origin_map;
    std::string tf_world_frame;
};

}   // namespace ll_localizer


#endif
#ifndef _LL_LOCALIZER_RELOCALIZER_HPP_
#define _LL_LOCALIZER_RELOCALIZER_HPP_

#include <chrono>
#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>
#include "relocalizer/gicp/fast_gicp.hpp"
#include "relocalizer/gicp/fast_gicp_st.hpp"

#include "common_lib.h"

namespace ll_localizer {

class Relocalizer {
public:
    Relocalizer() = default;
    Relocalizer(ros::NodeHandle &_nh) : nh(_nh){
        nh.param<std::string>("publish/tf_world_frame", tf_world_frame, "camera_init");

        nh.param<float>("relocalization_param/leaf_size", leaf_size, 0.3f);
        nh.param<double>("relocalization_param/det_range", det_range, 150.0);
        nh.param<double>("relocalization_param/max_correspondence_distance", max_correspondence_distance, 3.0);

        voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    }
    ~Relocalizer() = default;

    // float &LeafSize() { return leaf_size; }
    // double &DetRange() { return det_range; }

    void SetOriginMap(const CloudPtr& target);
    void SetInitPose(const Eigen::Affine3d& _init_pose);
    void SetInitScan(const CloudPtr source);

    Eigen::Affine3d Relocalize();

private: 
    template <typename Registration>
    void Align(Registration& reg, Eigen::Matrix4f& extra_init_pose_mat);

private:
    ros::NodeHandle nh;
    std::string tf_world_frame;

    float leaf_size = 0.3f;
    double det_range = 150.0;
    double max_correspondence_distance = 3.0;

    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_origin{new pcl::PointCloud<pcl::PointXYZ>()};
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud{new pcl::PointCloud<pcl::PointXYZ>()};
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud{new pcl::PointCloud<pcl::PointXYZ>()};
    pcl::VoxelGrid<PointType>voxel_filter;

    Eigen::Affine3d init_pose = Eigen::Affine3d::Identity();
};

}   // namespace ll_localizer

#endif
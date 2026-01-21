#include <iostream>

#include "relocalizer/relocalizer.hpp"

namespace ll_localizer {

void Relocalizer::SetOriginMap(const CloudPtr& target) {
    voxel_filter.setInputCloud(target);
    voxel_filter.filter(*target);
    
    for (auto pt : target->points) {
        pcl::PointXYZ p;
        p.x = pt.x;
        p.y = pt.y;
        p.z = pt.z;
        target_cloud_origin->points.emplace_back(p);
    }
}

void Relocalizer::SetInitPose(const Eigen::Affine3d& _init_pose) {
    init_pose = _init_pose;
    float init_x = init_pose.translation().x();
    float init_y = init_pose.translation().y();
    float init_z = init_pose.translation().z();

    for (auto pt : target_cloud_origin->points) {
        if ((pt.x - init_x) < det_range && (pt.x - init_x) > -det_range &&
            (pt.y - init_y) < det_range && (pt.y - init_y) > -det_range && 
            (pt.z - init_z) < det_range && (pt.z - init_z) > -det_range) {
            // if (pt.x * pt.x + pt.y * pt.y + pt.z * pt.z <= det_range * det_range) {
            pcl::PointXYZ p;
            p.x = pt.x;
            p.y = pt.y;
            p.z = pt.z;
            target_cloud->points.emplace_back(p);
        }
    }

    std::cout << "Origin cloud size: " << target_cloud_origin->points.size() << "\t"
              << "Target cloud size: " << target_cloud->points.size() << std::endl;
}

void Relocalizer::SetInitScan(const CloudPtr source) {
    voxel_filter.setInputCloud(source);
    voxel_filter.filter(*source);

    for (auto pt : source->points) {
        pcl::PointXYZ p;
        p.x = pt.x;
        p.y = pt.y;
        p.z = pt.z;
        source_cloud->points.emplace_back(p);
    }
    
    Eigen::Matrix4f init_pose_mat = init_pose.matrix().cast<float>();
    pcl::transformPointCloud(*source_cloud, *source_cloud, init_pose_mat);

    std::cout << "Source cloud size: " << source_cloud->points.size() << std::endl;
}

template <typename Registration>
void Relocalizer::Align(Registration& reg, Eigen::Matrix4f& extra_init_pose_mat) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned{new pcl::PointCloud<pcl::PointXYZ>()};
    auto t1 = std::chrono::high_resolution_clock::now();
    reg.setInputTarget(target_cloud);
    reg.setInputSource(source_cloud);
    reg.setMaxCorrespondenceDistance(max_correspondence_distance);
    reg.align(*aligned);
    
    extra_init_pose_mat = reg.getFinalTransformation();
    std::cout << "alignment result:\n" << extra_init_pose_mat << std::endl;
    
    double fitness_score = reg.getFitnessScore();
    std::cout << "fitness_score: " << fitness_score << std::endl;
    
    auto t2 = std::chrono::high_resolution_clock::now();
    std::cout << "alignment time cost: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << " ms" << std::endl;
}

Eigen::Affine3d Relocalizer::Relocalize() {
    Eigen::Matrix4f extra_init_pose_mat = Eigen::Matrix4f::Identity();

    fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ> fast_gicp_mt;
    // fast_gicp uses all the CPU cores by default
    // fast_gicp_mt.setNumThreads(8);
    Align(fast_gicp_mt, extra_init_pose_mat);

    Eigen::Matrix4f final_init_pose = extra_init_pose_mat * init_pose.matrix().cast<float>();
    init_pose = Eigen::Affine3d(final_init_pose.cast<double>());

    return init_pose;
}

}   // namespace ll_localizer
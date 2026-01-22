#ifndef MAP_MANAGER_UTILS_HPP
#define MAP_MANAGER_UTILS_HPP

#include <iostream>
#include <math.h>
#include <cmath>
#include <thread>
#include <mutex>
#include <fstream>
#include <vector>
#include <queue>
#include <tr1/unordered_map>
#include <unordered_map>
#include <tuple>
#include <Eigen/Core>

#include "tsl/robin_map.h"
#include "common_lib.h"
#include "octree.hpp"

namespace ll_localizer{

struct Point3f {
    Point3f(float x, float y, float z) : x(x), y(y), z(z) {}
    Point3f(float x, float y, float z, float i) : x(x), y(y), z(z), intensity(i) {}
    
    float x, y, z;
    float intensity;
};

struct Voxel {
public:
    Voxel(float vx, float vy, float vz, float revolusion)
     : x(vx / revolusion), y(vy / revolusion), z(vz / revolusion) {}
     
    Voxel(float vx, float vy, float vz, float revolusion, int x_near_block, int y_near_block, int z_near_block)
     : x(vx / revolusion + x_near_block), y(vy / revolusion + y_near_block), z(vz / revolusion + z_near_block) {}
    
    bool operator==(const Voxel &other) const {
        return this->x == other.x 
            && this->y == other.y 
            && this->z == other.z;
    }

public:
    int x, y, z;
};

class VoxelBlock {
public:
    explicit VoxelBlock(double size = 1.0, double resolution = 0.1, int threshold = 20) : 
    block_size(size), 
    block_resolution(resolution),
    temporary_map_points_threshold(threshold) {
        sample_array.resize(block_size / block_resolution * block_size / block_resolution * block_size / block_resolution, false);
        params.bucket_size = 1;
        params.min_extent = 0.1;
    }

    void AddOriginPoint(const Point3f &point) {
        origin_map_points.emplace_back(point);
        origin_map_points_num++;
    }

    void InitOriginOctree() {
        origin_map_points_octree.Initialize(origin_map_points, params);
    }

    void AddNewPoint(const Point3f &point) {
        temporary_map_points.emplace_back(point);
        incre_temporary_map_points_num++;
    }

    bool SearchFromOriginMapPoints(const Point3f &point, float radius, PointVector &results) const {
        std::vector<uint32_t> results_idx;
        origin_map_points_octree.RadiusNeighbors<common::L2Distance<Point3f>>(point, radius, results_idx);

        int result_num = results_idx.size();
        if (result_num > 0) {
            for (int i = 0; i < result_num; i++) {
                PointType pt;
                pt.x = origin_map_points[results_idx[i]].x;
                pt.y = origin_map_points[results_idx[i]].y;
                pt.z = origin_map_points[results_idx[i]].z;
                results.emplace_back(pt);
            }

            return results.size() >= options::MIN_NUM_MATCH_POINTS;
        } else {
            return false;
        }
    }

    bool SearchFromStaticMapPoints(const Point3f &point, float radius, PointVector &results) const {
        std::vector<uint32_t> results_idx;
        static_map_points_octree.RadiusNeighbors<common::L2Distance<Point3f>>(point, radius, results_idx);

        int result_num = results_idx.size();
        if (result_num > 0) {
            for (int i = 0; i < result_num; i++) {
                PointType pt;
                pt.x = static_map_points[results_idx[i]].x;
                pt.y = static_map_points[results_idx[i]].y;
                pt.z = static_map_points[results_idx[i]].z;
                results.emplace_back(pt);
            }

            return results.size() >= options::MIN_NUM_MATCH_POINTS;
        } else {
            return false;
        }
    }

    bool SearchFromTemporaryMapPoints(const Point3f &point, float radius, PointVector &results) const {
        for (size_t i = 0; i < temporary_map_points.size(); ++i) {
            const Point3f &p = temporary_map_points[i];
            float dist = sqrt((p.x - point.x) * (p.x - point.x) +
                              (p.y - point.y) * (p.y - point.y) +
                              (p.z - point.z) * (p.z - point.z));
            if (dist <= radius) {
                PointType pt;
                pt.x = p.x;
                pt.y = p.y;
                pt.z = p.z;
                results.emplace_back(pt);
            }
        }

        return results.size() >= options::MIN_NUM_MATCH_POINTS;
    }

    void AddTempToStatic(pcl::PointCloud<pcl::PointXYZ>::Ptr &new_points) {
        for (auto pt : temporary_map_points) {
            int sample_x = std::fabs(std::fmod(pt.x, block_size) / block_resolution);
            int sample_y = std::fabs(std::fmod(pt.y, block_size) / block_resolution);
            int sample_z = std::fabs(std::fmod(pt.z, block_size) / block_resolution);
            int sample_place = sample_x + sample_y * block_size / block_resolution + sample_z * block_size / block_resolution * block_size / block_resolution;
            
            if (!sample_array[sample_place]) {
                static_map_points.emplace_back(pt);

                pcl::PointXYZ p (pt.x, pt.y, pt.z);
                new_points->points.emplace_back(p);

                sample_array[sample_place] = true;
            }
        }
        
        temporary_map_points.clear();
        std::vector<Point3f>().swap(temporary_map_points);

        if (is_origin && incre_temporary_map_points_num > origin_map_points_num) {
            origin_map_points.clear();
            std::vector<Point3f>().swap(origin_map_points);
            is_origin = false;
        }
    }

    void UpdateOctree(pcl::PointCloud<pcl::PointXYZ>::Ptr &new_points) {
        AddTempToStatic(new_points);

        static_map_points_octree.Initialize(static_map_points, params);
    }

    inline bool IsTemporaryPointsFull() { return temporary_map_points.size() >= temporary_map_points_threshold; }

public:
    std::vector<Point3f> origin_map_points;
    std::vector<Point3f> static_map_points;
    std::vector<Point3f> temporary_map_points;

    int origin_map_points_num = 0;
    int incre_temporary_map_points_num = 0;
    bool is_origin = false;

private:
    double block_size;
    double block_resolution;

    int temporary_map_points_threshold;

    std::vector<bool> sample_array;

    common::Octree<Point3f> origin_map_points_octree;
    common::Octree<Point3f> static_map_points_octree;
    common::OctreeParams params;
};


struct VoxelHash {
    std::size_t operator()(const Voxel &vox) const {
        const size_t kP_x = 73856093;
        const size_t kP_y = 19349669;
        const size_t kP_z = 83492791;
        return vox.x * kP_x + vox.y * kP_y + vox.z * kP_z;
    }
};

typedef tsl::robin_map<Voxel, VoxelBlock, VoxelHash> VoxelHashMap;

}   // namespace ll_localizer

#endif
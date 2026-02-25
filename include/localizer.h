#ifndef _LL_LOCALIZER_LASER_MAPPING_H_
#define _LL_LOCALIZER_LASER_MAPPING_H_

#include <livox_ros_driver/CustomMsg.h>
#include <nav_msgs/Path.h>
#include <pcl/filters/voxel_grid.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <condition_variable>
#include <thread>

#include "imu_processing.hpp"
#include "options.h"
#include "pointcloud_preprocess.h"
#include "relocalizer/relocalizer.hpp"
#include "map_manager/map_manager.hpp"

namespace ll_localizer {

class Localizer {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    Localizer();
    ~Localizer() {
        origin_map = nullptr;
        scan_down_body = nullptr;
        scan_undistort = nullptr;
        scan_down_world = nullptr;
        LOG(INFO) << "laser mapping deconstruct";
    }

    /// init with ros
    bool InitROS(ros::NodeHandle &nh);

    void Run();

    // callbacks
    void InitialPoseCallBack(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg);
    void StandardPCLCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void LivoxPCLCallBack(const livox_ros_driver::CustomMsg::ConstPtr &msg);
    void IMUCallBack(const sensor_msgs::Imu::ConstPtr &msg_in);

    // sync lidar with imu
    bool SyncPackages();

    /// interface of mtk, customized obseravtion model
    void ObsModel(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data);

    ////////////////////////////// debug save / show ////////////////////////////////////////////////////////////////
    void PublishPath(const ros::Publisher pub_path);
    void PublishOdometry(const ros::Publisher &pub_odom_aft_mapped);
    void PublishFrameWorld();
    void PublishFrameBody(const ros::Publisher &pub_laser_cloud_body);
    void PublishFrameEffectWorld(const ros::Publisher &pub_laser_cloud_effect_world);
    void PublishUpdatedMap(const ros::Publisher &pub_updated_map);
    void Savetrajectory(const std::string &traj_file);

    void Finish();

   private:
    template <typename T>
    void SetPosestamp(T &out);

    bool LoadOriginMap();

    void Relocalize();

    void PointBodyToWorld(PointType const *pi, PointType *const po);
    void PointBodyToWorld(const common::V3F &pi, PointType *const po);
    void PointBodyLidarToIMU(PointType const *const pi, PointType *const po);

    void SubAndPubToROS(ros::NodeHandle &nh);

    bool LoadParams(ros::NodeHandle &nh);

    void PrintState(const state_ikfom &s);

   private:
    /// modules
    std::shared_ptr<Relocalizer> relocalizer = nullptr;
    std::shared_ptr<MapManager> map_manager = nullptr;
    std::mutex search_mtx;
    std::shared_ptr<PointCloudPreprocess> preprocess = nullptr;  // point cloud preprocess
    std::shared_ptr<ImuProcess> p_imu = nullptr;                 // imu process

    /// local map related
    float det_range = 300.0f;
    double cube_len = 0;
    double filter_size_map_min = 0;
    bool localmap_initialized = false;

    /// params
    std::vector<double> extrinT{3, 0.0};  // lidar-imu translation
    std::vector<double> extrinR{9, 0.0};  // lidar-imu rotation
    std::string map_file_name;
    std::string map_file_path;

    /// point clouds data
    CloudPtr origin_map{new PointCloudType()};
    pcl::PointCloud<pcl::PointXYZ>::Ptr updated_map{new pcl::PointCloud<pcl::PointXYZ>()};
    CloudPtr scan_undistort{new PointCloudType()};   // scan after undistortion
    CloudPtr scan_down_body{new PointCloudType()};   // downsampled scan in body
    CloudPtr scan_down_world{new PointCloudType()};  // downsampled scan in world
    std::vector<PointVector> nearest_points;         // nearest points of current scan
    common::VV4F corr_pts;                           // inlier pts
    common::VV4F corr_norm;                          // inlier plane norms
    pcl::VoxelGrid<PointType> voxel_scan;            // voxel filter for current scan
    std::vector<float> weights;
    std::vector<float> residuals;                    // point-to-plane residuals
    std::vector<char> point_selected_surf;           // selected points
    common::VV4F plane_coef;                         // plane coeffs

    /// ros pub and sub stuffs
    ros::Subscriber sub_init_pose;
    ros::Subscriber sub_pcl;
    ros::Subscriber sub_imu;
    ros::Publisher pub_laser_cloud_world;
    ros::Publisher pub_laser_cloud_body;
    ros::Publisher pub_laser_cloud_effect_world;
    ros::Publisher pub_updated_map;
    ros::Publisher pub_odom_aft_mapped;
    ros::Publisher pub_path;
    std::string tf_imu_frame;
    std::string tf_world_frame;

    std::mutex mtx_buffer;
    std::deque<double> time_buffer;
    std::deque<PointCloudType::Ptr> lidar_buffer;
    std::deque<sensor_msgs::Imu::ConstPtr> imu_buffer;
    nav_msgs::Odometry odom_aft_mapped;

    /// options
    bool time_sync_en = false;
    double timediff_lidar_wrt_imu = 0.0;
    double last_timestamp_lidar = 0;
    double lidar_end_time = 0;
    double last_timestamp_imu = -1.0;
    double first_lidar_time = 0.0;
    bool lidar_pushed = false;

    /// statistics and flags ///
    int scan_count = 0;
    int publish_count = 0;
    bool get_origin_map = false;
    bool get_inital_pose = false;
    bool flg_first_scan = true;
    bool flg_EKF_inited = false;
    int pcd_index = 0;
    double lidar_mean_scantime = 0.0;
    int scan_num = 0;
    bool timediff_set_flg = false;
    int effect_feat_num = 0, frame_num = 0;

    ///////////////////////// EKF inputs and output ///////////////////////////////////////////////////////
    double init_height;
    Eigen::Affine3d init_pose = Eigen::Affine3d::Identity();
    common::MeasureGroup measures;                    // sync IMU and lidar scan
    esekfom::esekf<state_ikfom, 12, input_ikfom> kf;  // esekf
    state_ikfom state_point;                          // ekf current state
    vect3 pos_lidar;                                  // lidar position after eskf update
    common::V3D euler_cur = common::V3D::Zero();      // rotation in euler angles
    bool extrinsic_est_en = true;

    /////////////////////////  debug show / save /////////////////////////////////////////////////////////
    bool run_in_offline = false;
    bool path_pub_en = true;
    bool scan_pub_en = false;
    bool dense_pub_en = false;
    bool scan_body_pub_en = false;
    bool scan_effect_pub_en = false;
    bool pcd_save_en = false;
    bool runtime_pos_log = true;
    int pcd_save_interval = -1;
    bool path_save_en = false;
    std::string dataset;

    PointCloudType::Ptr pcl_wait_save{new PointCloudType()};  // debug save
    nav_msgs::Path path;
    geometry_msgs::PoseStamped msg_body_pose;
};

}  // namespace ll_localizer

#endif  // _LL_LOCALIZER_LASER_MAPPING_H_
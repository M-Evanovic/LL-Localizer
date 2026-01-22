#include <tf/transform_broadcaster.h>
#include <execution>
#include <fstream>
#include <pcl/io/pcd_io.h>

#include "localizer.h"
#include "utils.h"

namespace ll_localizer {

Localizer::Localizer() {
    preprocess.reset(new PointCloudPreprocess());
    p_imu.reset(new ImuProcess());
}

bool Localizer::InitROS(ros::NodeHandle &nh) {
    LoadParams(nh);

    SubAndPubToROS(nh);

    // localmap init (after LoadParams)
    relocalizer = std::make_shared<Relocalizer>(nh);
    map_manager = std::make_shared<MapManager>(nh);

    LoadOriginMap();

    // esekf init
    std::vector<double> epsi(23, 0.001);
    kf.init_dyn_share(
        get_f, df_dx, df_dw,
        [this](state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) { ObsModel(s, ekfom_data); },
        options::NUM_MAX_ITERATIONS, epsi.data());

    return true;
}

bool Localizer::LoadParams(ros::NodeHandle &nh) {
    // get params from param server
    int lidar_type;
    double gyr_cov, acc_cov, b_gyr_cov, b_acc_cov;
    double filter_size_surf_min;
    common::V3D lidar_T_wrt_IMU;
    common::M3D lidar_R_wrt_IMU;

    nh.param<std::string>("localization_param/map_file_name", map_file_name, "origin_map");
    map_file_path = std::string(std::string(ROOT_DIR) + "map/") + map_file_name + std::string(".pcd");

    nh.param<double>("localization_param/init_height", init_height, 0.5);

    nh.param<bool>("path_save_en", path_save_en, true);
    nh.param<bool>("publish/path_publish_en", path_pub_en, true);
    nh.param<bool>("publish/scan_publish_en", scan_pub_en, true);
    nh.param<bool>("publish/dense_publish_en", dense_pub_en, false);
    nh.param<bool>("publish/scan_bodyframe_pub_en", scan_body_pub_en, true);
    nh.param<bool>("publish/scan_effect_pub_en", scan_effect_pub_en, false);
    nh.param<std::string>("publish/tf_imu_frame", tf_imu_frame, "body");
    nh.param<std::string>("publish/tf_world_frame", tf_world_frame, "camera_init");

    nh.param<int>("max_iteration", options::NUM_MAX_ITERATIONS, 4);
    nh.param<float>("esti_plane_threshold", options::ESTI_PLANE_THRESHOLD, 0.1);
    nh.param<bool>("common/time_sync_en", time_sync_en, false);
    nh.param<double>("filter_size_surf", filter_size_surf_min, 0.5);
    nh.param<double>("filter_size_map", filter_size_map_min, 0.5);
    nh.param<double>("cube_side_length", cube_len, 200);
    nh.param<float>("mapping/det_range", det_range, 300.f);
    nh.param<double>("mapping/gyr_cov", gyr_cov, 0.1);
    nh.param<double>("mapping/acc_cov", acc_cov, 0.1);
    nh.param<double>("mapping/b_gyr_cov", b_gyr_cov, 0.0001);
    nh.param<double>("mapping/b_acc_cov", b_acc_cov, 0.0001);
    nh.param<double>("preprocess/blind", preprocess->Blind(), 0.01);
    nh.param<float>("preprocess/time_scale", preprocess->TimeScale(), 1e-3);
    nh.param<int>("preprocess/lidar_type", lidar_type, 1);
    nh.param<int>("preprocess/scan_line", preprocess->NumScans(), 16);
    nh.param<int>("point_filter_num", preprocess->PointFilterNum(), 2);
    nh.param<bool>("feature_extract_enable", preprocess->FeatureEnabled(), false);
    nh.param<bool>("runtime_pos_log_enable", runtime_pos_log, true);
    nh.param<bool>("mapping/extrinsic_est_en", extrinsic_est_en, true);
    nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en, false);
    nh.param<int>("pcd_save/interval", pcd_save_interval, -1);
    nh.param<std::vector<double>>("mapping/extrinsic_T", extrinT, std::vector<double>());
    nh.param<std::vector<double>>("mapping/extrinsic_R", extrinR, std::vector<double>());

    LOG(INFO) << "lidar_type " << lidar_type;
    if (lidar_type == 1) {
        preprocess->SetLidarType(LidarType::AVIA);
        LOG(INFO) << "Using AVIA Lidar";
    } else if (lidar_type == 2) {
        preprocess->SetLidarType(LidarType::VELO32);
        LOG(INFO) << "Using Velodyne 32 Lidar";
    } else if (lidar_type == 3) {
        preprocess->SetLidarType(LidarType::OUST64);
        LOG(INFO) << "Using OUST 64 Lidar";
    } else if (lidar_type == 4) {
        preprocess->SetLidarType(LidarType::HESAIxt32);
        LOG(INFO) << "Using Hesai Pandar 32 Lidar";
    } else {
        LOG(WARNING) << "unknown lidar_type";
        return false;
    }

    path.header.stamp = ros::Time::now();
    path.header.frame_id = tf_world_frame;

    voxel_scan.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);

    lidar_T_wrt_IMU = common::VecFromArray<double>(extrinT);
    lidar_R_wrt_IMU = common::MatFromArray<double>(extrinR);

    p_imu->SetExtrinsic(lidar_T_wrt_IMU, lidar_R_wrt_IMU);
    p_imu->SetGyrCov(common::V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu->SetAccCov(common::V3D(acc_cov, acc_cov, acc_cov));
    p_imu->SetGyrBiasCov(common::V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu->SetAccBiasCov(common::V3D(b_acc_cov, b_acc_cov, b_acc_cov));
    return true;
}

bool Localizer::LoadOriginMap() {
    if (pcl::io::loadPCDFile<PointType>(map_file_path, *origin_map) == -1) {
        std::cerr << "Cannot read " << map_file_path << std::endl;
        return false;
    }

    ROS_INFO("\033[1;35m Receive Origin Map from %s, point num: %lu \033[0m", map_file_path.c_str(), origin_map->points.size());

    relocalizer->SetOriginMap(origin_map);
    map_manager->AddOriginMap(origin_map);

    get_origin_map = true;

    return true;
}

void Localizer::InitialPoseCallBack(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg) {
    Eigen::Vector3d init_translation; 
    Eigen::Quaterniond init_rotation;
    init_translation[0] = pose_msg->pose.pose.position.x;
    init_translation[1] = pose_msg->pose.pose.position.y;
    init_translation[2] = init_height;
    init_pose.translation() = init_translation;

    double x,y,z,w;
    x = pose_msg->pose.pose.orientation.x;
    y = pose_msg->pose.pose.orientation.y;
    z = pose_msg->pose.pose.orientation.z;
    w = pose_msg->pose.pose.orientation.w;
    init_rotation = Eigen::Quaterniond(w,x,y,z);
    init_pose.rotate(init_rotation);

    std::cout << "init pose:\n" << init_pose.matrix() << std::endl;

    relocalizer->SetInitPose(init_pose);
    
    get_inital_pose = true;

    ROS_INFO("\033[1;35m Receive Initial Pose \033[0m");
}

void Localizer::Relocalize() {
    relocalizer->SetInitScan(scan_undistort);
    init_pose = relocalizer->Relocalize();

    std::cout << "final init pose:\n" << init_pose.matrix() << std::endl;

    Eigen::Vector3d final_position = init_pose.translation();
    Eigen::Quaterniond final_rotation(init_pose.linear());
    state_ikfom init_state = kf.get_x();
    init_state.pos = final_position;
    init_state.rot = final_rotation;
    kf.change_x(init_state);

    get_inital_pose = false;
}

void Localizer::SubAndPubToROS(ros::NodeHandle &nh) {
    // ROS subscribe initialization
    std::string lidar_topic, imu_topic;
    nh.param<std::string>("common/lid_topic", lidar_topic, "/livox/lidar");
    nh.param<std::string>("common/imu_topic", imu_topic, "/livox/imu");

    sub_init_pose = nh.subscribe("/initialpose", 1, &Localizer::InitialPoseCallBack, this);

    if (preprocess->GetLidarType() == LidarType::AVIA) {
        sub_pcl = nh.subscribe<livox_ros_driver::CustomMsg>(
            lidar_topic, 200000, [this](const livox_ros_driver::CustomMsg::ConstPtr &msg) { LivoxPCLCallBack(msg); });
    } else {
        sub_pcl = nh.subscribe<sensor_msgs::PointCloud2>(
            lidar_topic, 200000, [this](const sensor_msgs::PointCloud2::ConstPtr &msg) { StandardPCLCallBack(msg); });
    }

    sub_imu = nh.subscribe<sensor_msgs::Imu>(imu_topic, 200000,
                                              [this](const sensor_msgs::Imu::ConstPtr &msg) { IMUCallBack(msg); });

    // ROS publisher init
    path.header.stamp = ros::Time::now();
    path.header.frame_id = tf_world_frame;
    
    pub_laser_cloud_world = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100000);
    pub_laser_cloud_body = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_body", 100000);
    pub_laser_cloud_effect_world = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_effect_world", 100000);
    pub_odom_aft_mapped = nh.advertise<nav_msgs::Odometry>("/Odometry", 100000);
    pub_path = nh.advertise<nav_msgs::Path>("/path", 100000);
}

void Localizer::Run() {
    if (!SyncPackages()) {
        return;
    }

    /// IMU process, kf prediction, undistortion
    p_imu->Process(measures, kf, scan_undistort);
    if (scan_undistort->empty() || (scan_undistort == nullptr)) {
        LOG(WARNING) << "No point, skip this scan!";
        return;
    }

    /// the first scan
    if (flg_first_scan) {
        if (!get_origin_map) {
            state_point = kf.get_x();
            scan_down_world->resize(scan_undistort->size());
            for (int i = 0; i < scan_undistort->size(); i++) {
                PointBodyToWorld(&scan_undistort->points[i], &scan_down_world->points[i]);
            }
            map_manager->AddPoints(scan_down_world->points);
        } else if (!get_inital_pose) {
            ROS_INFO("\033[1;33m Waiting for Init Pose \033[0m");
            return;
        } else {
            Relocalize();
            ROS_INFO("\033[1;35m Successfully Initialize Pose \033[0m");
            state_point = kf.get_x();
            scan_down_world->resize(scan_undistort->size());
            for (int i = 0; i < scan_undistort->size(); i++) {
                PointBodyToWorld(&scan_undistort->points[i], &scan_down_world->points[i]);
            }
            map_manager->AddPoints(scan_down_world->points);
        }
        
        first_lidar_time = measures.lidar_bag_time;
        flg_first_scan = false;

        ROS_INFO("\033[1;35m Start Running Localization\033[0m");
        
        return;
    }
    

    flg_EKF_inited = (measures.lidar_bag_time - first_lidar_time) >= options::INIT_TIME;

    /// downsample
    Timer::Evaluate(
        [&, this]() {
            voxel_scan.setInputCloud(scan_undistort);
            voxel_scan.filter(*scan_down_body);
        },
        "Downsample PointCloud");

    int cur_pts = scan_down_body->size();
    if (cur_pts < 5) {
        LOG(WARNING) << "Too few points, skip this scan!" << scan_undistort->size() << ", " << scan_down_body->size();
        return;
    }
    scan_down_world->resize(cur_pts);
    nearest_points.resize(cur_pts);
    weights.resize(cur_pts, 1.0f);
    residuals.resize(cur_pts, 0);
    point_selected_surf.resize(cur_pts, true);
    plane_coef.resize(cur_pts, common::V4F::Zero());

    // ICP and iterated Kalman filter update
    Timer::Evaluate(
        [&, this]() {
            // iterated state estimation
            double solve_H_time = 0;
            // update the observation model, will call nn and point-to-plane residual computation
            kf.update_iterated_dyn_share_modified(options::LASER_POINT_COV, solve_H_time);
            // save the state
            state_point = kf.get_x();
            euler_cur = SO3ToEuler(state_point.rot);
            pos_lidar = state_point.pos + state_point.rot * state_point.offset_T_L_I;
        },
        "IEKF Solve and Update");

    // publish or save map pcd
    if (run_in_offline) {
        if (pcd_save_en) {
            PublishFrameWorld();
        }
        if (path_save_en) {
            PublishPath(pub_path);
        }
    } else {
        if (pub_odom_aft_mapped) {
            PublishOdometry(pub_odom_aft_mapped);
        }
        if (path_pub_en || path_save_en) {
            PublishPath(pub_path);
        }
        if (scan_pub_en || pcd_save_en) {
            PublishFrameWorld();
        }
        if (scan_pub_en && scan_body_pub_en) {
            PublishFrameBody(pub_laser_cloud_body);
        }
        if (scan_pub_en && scan_effect_pub_en) {
            PublishFrameEffectWorld(pub_laser_cloud_effect_world);
        }
    }

    // Debug variables
    frame_num++;
}

void Localizer::StandardPCLCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    mtx_buffer.lock();
    Timer::Evaluate(
        [&, this]() {
            scan_count++;
            if (msg->header.stamp.toSec() < last_timestamp_lidar) {
                LOG(ERROR) << "lidar loop back, clear buffer";
                lidar_buffer.clear();
            }

            PointCloudType::Ptr ptr(new PointCloudType());
            preprocess->Process(msg, ptr);
            lidar_buffer.push_back(ptr);
            time_buffer.push_back(msg->header.stamp.toSec());
            last_timestamp_lidar = msg->header.stamp.toSec();
        },
        "Preprocess (Standard)");
    mtx_buffer.unlock();
}

void Localizer::LivoxPCLCallBack(const livox_ros_driver::CustomMsg::ConstPtr &msg) {
    mtx_buffer.lock();
    Timer::Evaluate(
        [&, this]() {
            scan_count++;
            if (msg->header.stamp.toSec() < last_timestamp_lidar) {
                LOG(WARNING) << "lidar loop back, clear buffer";
                lidar_buffer.clear();
            }

            last_timestamp_lidar = msg->header.stamp.toSec();

            if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() &&
                !lidar_buffer.empty()) {
                LOG(INFO) << "IMU and LiDAR not Synced, IMU time: " << last_timestamp_imu
                          << ", lidar header time: " << last_timestamp_lidar;
            }

            if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 &&
                !imu_buffer.empty()) {
                timediff_set_flg = true;
                timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
                LOG(INFO) << "Self sync IMU and LiDAR, time diff is " << timediff_lidar_wrt_imu;
            }

            PointCloudType::Ptr ptr(new PointCloudType());
            preprocess->Process(msg, ptr);
            lidar_buffer.emplace_back(ptr);
            time_buffer.emplace_back(last_timestamp_lidar);
        },
        "Preprocess (Livox)");

    mtx_buffer.unlock();
}

void Localizer::IMUCallBack(const sensor_msgs::Imu::ConstPtr &msg_in) {
    publish_count++;
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en) {
        msg->header.stamp = ros::Time().fromSec(timediff_lidar_wrt_imu + msg_in->header.stamp.toSec());
    }

    double timestamp = msg->header.stamp.toSec();

    mtx_buffer.lock();
    if (timestamp < last_timestamp_imu) {
        LOG(WARNING) << "imu loop back, clear buffer";
        imu_buffer.clear();
    }

    last_timestamp_imu = timestamp;
    imu_buffer.emplace_back(msg);
    mtx_buffer.unlock();
}

bool Localizer::SyncPackages() {
    if (lidar_buffer.empty() || imu_buffer.empty()) {
        return false;
    }

    /*** push a lidar scan ***/
    if (!lidar_pushed) {
        measures.lidar = lidar_buffer.front();
        measures.lidar_bag_time = time_buffer.front();

        if (measures.lidar->points.size() <= 1) {
            LOG(WARNING) << "Too few input point cloud!";
            lidar_end_time = measures.lidar_bag_time + lidar_mean_scantime;
        } else if (measures.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime) {
            lidar_end_time = measures.lidar_bag_time + lidar_mean_scantime;
        } else {
            scan_num++;
            lidar_end_time = measures.lidar_bag_time + measures.lidar->points.back().curvature / double(1000);
            lidar_mean_scantime +=
                (measures.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        }

        measures.lidar_end_time = lidar_end_time;
        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time) {
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = imu_buffer.front()->header.stamp.toSec();
    measures.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time)) {
        imu_time = imu_buffer.front()->header.stamp.toSec();
        if (imu_time > lidar_end_time) break;
        measures.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

void Localizer::PrintState(const state_ikfom &s) {
    LOG(INFO) << "state r: " << s.rot.coeffs().transpose() << ", t: " << s.pos.transpose()
              << ", off r: " << s.offset_R_L_I.coeffs().transpose() << ", t: " << s.offset_T_L_I.transpose();
}

/**
 * Lidar point cloud registration
 * will be called by the eskf custom observation model
 * compute point-to-plane residual here
 * @param s kf state
 * @param ekfom_data H matrix
 */
void Localizer::ObsModel(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) {
    int cnt_pts = scan_down_body->size();

    std::vector<size_t> index(cnt_pts);
    for (size_t i = 0; i < index.size(); ++i) {
        index[i] = i;
    }

    Timer::Evaluate(
        [&, this]() {
            auto R_wl = (s.rot * s.offset_R_L_I).cast<float>();
            auto t_wl = (s.rot * s.offset_T_L_I + s.pos).cast<float>();
            
            /** closest surface search and residual computation **/
            std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t &i) {
                PointType &point_body = scan_down_body->points[i];
                PointType &point_world = scan_down_world->points[i];

                /* transform to world frame */
                common::V3F p_body = point_body.getVector3fMap();
                point_world.getVector3fMap() = R_wl * p_body + t_wl;
                point_world.intensity = point_body.intensity;

                auto &points_near = nearest_points[i];
                if (ekfom_data.converge) {
                    /** Find the closest surfaces in the map **/
                    points_near.clear();
                    search_mtx.lock();
                    point_selected_surf[i] = map_manager->SearchCorrespondMapPoints(point_world, points_near);
                    search_mtx.unlock();
                    if (point_selected_surf[i]) {
                        weights[i] = 1.0f + (point_selected_surf[i] - 2) * 0.2;
                        point_selected_surf[i] = common::esti_plane(plane_coef[i], points_near, options::ESTI_PLANE_THRESHOLD);
                    }
                }

                if (point_selected_surf[i]) {
                    auto temp = point_world.getVector4fMap();
                    temp[3] = 1.0;
                    float pd2 = plane_coef[i].dot(temp);

                    bool valid_corr = p_body.norm() > 81 * pd2 * pd2;
                    if (valid_corr) {
                        point_selected_surf[i] = true;
                        residuals[i] = pd2 * weights[i];
                    } else {
                        point_selected_surf[i] = false;
                    }
                }
            });
        },
        "    ObsModel (Lidar Match)");

    effect_feat_num = 0;

    corr_pts.resize(cnt_pts);
    corr_norm.resize(cnt_pts);
    for (int i = 0; i < cnt_pts; i++) {
        if (point_selected_surf[i]) {
            corr_norm[effect_feat_num] = plane_coef[i];
            corr_pts[effect_feat_num] = scan_down_body->points[i].getVector4fMap();
            corr_pts[effect_feat_num][3] = residuals[i];

            effect_feat_num++;
        }
    }
    
    corr_pts.resize(effect_feat_num);
    corr_norm.resize(effect_feat_num);

    if (effect_feat_num < 1) {
        ekfom_data.valid = false;
        LOG(WARNING) << "No Effective Points!";
        return;
    }

    Timer::Evaluate(
        [&, this]() {
            /*** Computation of Measurement Jacobian matrix H and measurements vector ***/
            ekfom_data.h_x = Eigen::MatrixXd::Zero(effect_feat_num, 12);  // 23
            ekfom_data.h.resize(effect_feat_num);

            index.resize(effect_feat_num);
            const common::M3F off_R = s.offset_R_L_I.toRotationMatrix().cast<float>();
            const common::V3F off_t = s.offset_T_L_I.cast<float>();
            const common::M3F Rt = s.rot.toRotationMatrix().transpose().cast<float>();

            std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t &i) {
                common::V3F point_this_be = corr_pts[i].head<3>();
                common::M3F point_be_crossmat = SKEW_SYM_MATRIX(point_this_be);
                common::V3F point_this = off_R * point_this_be + off_t;
                common::M3F point_crossmat = SKEW_SYM_MATRIX(point_this);

                /*** get the normal vector of closest surface/corner ***/
                common::V3F norm_vec = corr_norm[i].head<3>();

                /*** calculate the Measurement Jacobian matrix H ***/
                common::V3F C(Rt * norm_vec);
                common::V3F A(point_crossmat * C);

                if (extrinsic_est_en) {
                    common::V3F B(point_be_crossmat * off_R.transpose() * C);
                    ekfom_data.h_x.block<1, 12>(i, 0) << norm_vec[0], norm_vec[1], norm_vec[2], A[0], A[1], A[2], B[0],
                        B[1], B[2], C[0], C[1], C[2];
                } else {
                    ekfom_data.h_x.block<1, 12>(i, 0) << norm_vec[0], norm_vec[1], norm_vec[2], A[0], A[1], A[2], 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0;
                }

                /*** Measurement: distance to the closest surface/corner ***/
                ekfom_data.h(i) = -corr_pts[i][3];
            });
        },
        "    ObsModel (IEKF Build Jacobian)");
}

/////////////////////////////////////  debug save / show /////////////////////////////////////////////////////

void Localizer::PublishPath(const ros::Publisher pub_path) {
    SetPosestamp(msg_body_pose);
    msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = tf_world_frame;

    /*** if path is too large, the rvis will crash ***/
    path.poses.push_back(msg_body_pose);
    if (run_in_offline == false) {
        pub_path.publish(path);
    }
}

void Localizer::PublishOdometry(const ros::Publisher &pub_odom_aft_mapped) {
    odom_aft_mapped.header.frame_id = tf_world_frame;
    odom_aft_mapped.child_frame_id = tf_imu_frame;
    odom_aft_mapped.header.stamp = ros::Time().fromSec(lidar_end_time);  // ros::Time().fromSec(lidar_end_time);
    SetPosestamp(odom_aft_mapped.pose);
    pub_odom_aft_mapped.publish(odom_aft_mapped);
    auto P = kf.get_P();
    for (int i = 0; i < 6; i++) {
        int k = i < 3 ? i + 3 : i - 3;
        odom_aft_mapped.pose.covariance[i * 6 + 0] = P(k, 3);
        odom_aft_mapped.pose.covariance[i * 6 + 1] = P(k, 4);
        odom_aft_mapped.pose.covariance[i * 6 + 2] = P(k, 5);
        odom_aft_mapped.pose.covariance[i * 6 + 3] = P(k, 0);
        odom_aft_mapped.pose.covariance[i * 6 + 4] = P(k, 1);
        odom_aft_mapped.pose.covariance[i * 6 + 5] = P(k, 2);
    }

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(odom_aft_mapped.pose.pose.position.x, odom_aft_mapped.pose.pose.position.y,
                                    odom_aft_mapped.pose.pose.position.z));
    q.setW(odom_aft_mapped.pose.pose.orientation.w);
    q.setX(odom_aft_mapped.pose.pose.orientation.x);
    q.setY(odom_aft_mapped.pose.pose.orientation.y);
    q.setZ(odom_aft_mapped.pose.pose.orientation.z);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, odom_aft_mapped.header.stamp, tf_world_frame, tf_imu_frame));
}

void Localizer::PublishFrameWorld() {
    if (!(run_in_offline == false && scan_pub_en) && !pcd_save_en) {
        return;
    }

    PointCloudType::Ptr laserCloudWorld;
    if (dense_pub_en) {
        PointCloudType::Ptr laserCloudFullRes(scan_undistort);
        int size = laserCloudFullRes->points.size();
        laserCloudWorld.reset(new PointCloudType(size, 1));
        for (int i = 0; i < size; i++) {
            PointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
        }
    } else {
        laserCloudWorld = scan_down_world;
    }

    if (run_in_offline == false && scan_pub_en) {
        sensor_msgs::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.frame_id = tf_world_frame;
        pub_laser_cloud_world.publish(laserCloudmsg);
        publish_count -= options::PUBFRAME_PERIOD;
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    if (pcd_save_en) {
        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval) {
            pcd_index++;
            std::string all_points_dir(std::string(std::string(ROOT_DIR) + "PCD/scans_") + std::to_string(pcd_index) +
                                       std::string(".pcd"));
            pcl::PCDWriter pcd_writer;
            LOG(INFO) << "current scan saved to /PCD/" << all_points_dir;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
}

void Localizer::PublishFrameBody(const ros::Publisher &pub_laser_cloud_body) {
    int size = scan_undistort->points.size();
    PointCloudType::Ptr laser_cloud_imu_body(new PointCloudType(size, 1));

    for (int i = 0; i < size; i++) {
        PointBodyLidarToIMU(&scan_undistort->points[i], &laser_cloud_imu_body->points[i]);
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laser_cloud_imu_body, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "body";
    pub_laser_cloud_body.publish(laserCloudmsg);
    publish_count -= options::PUBFRAME_PERIOD;
}

void Localizer::PublishFrameEffectWorld(const ros::Publisher &pub_laser_cloud_effect_world) {
    int size = corr_pts.size();
    PointCloudType::Ptr laser_cloud(new PointCloudType(size, 1));

    for (int i = 0; i < size; i++) {
        PointBodyToWorld(corr_pts[i].head<3>(), &laser_cloud->points[i]);
    }
    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laser_cloud, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = tf_world_frame;
    pub_laser_cloud_effect_world.publish(laserCloudmsg);
    publish_count -= options::PUBFRAME_PERIOD;
}

void Localizer::Savetrajectory(const std::string &traj_file) {
    std::ofstream ofs;
    ofs.open(traj_file, std::ios::out);
    if (!ofs.is_open()) {
        LOG(ERROR) << "Failed to open traj_file: " << traj_file;
        return;
    }

    ofs << "#timestamp x y z q_x q_y q_z q_w" << std::endl;
    for (const auto &p : path.poses) {
        ofs << std::fixed << std::setprecision(6) << p.header.stamp.toSec() << " " << std::setprecision(15)
            << p.pose.position.x << " " << p.pose.position.y << " " << p.pose.position.z << " " << p.pose.orientation.x
            << " " << p.pose.orientation.y << " " << p.pose.orientation.z << " " << p.pose.orientation.w << std::endl;
    }

    ofs.close();
}

///////////////////////////  private method /////////////////////////////////////////////////////////////////////
template <typename T>
void Localizer::SetPosestamp(T &out) {
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);
    out.pose.orientation.x = state_point.rot.coeffs()[0];
    out.pose.orientation.y = state_point.rot.coeffs()[1];
    out.pose.orientation.z = state_point.rot.coeffs()[2];
    out.pose.orientation.w = state_point.rot.coeffs()[3];
}

void Localizer::PointBodyToWorld(const PointType *pi, PointType *const po) {
    common::V3D p_body(pi->x, pi->y, pi->z);
    common::V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) +
                         state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void Localizer::PointBodyToWorld(const common::V3F &pi, PointType *const po) {
    common::V3D p_body(pi.x(), pi.y(), pi.z());
    common::V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) +
                         state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = std::abs(po->z);
}

void Localizer::PointBodyLidarToIMU(PointType const *const pi, PointType *const po) {
    common::V3D p_body_lidar(pi->x, pi->y, pi->z);
    common::V3D p_body_imu(state_point.offset_R_L_I * p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void Localizer::Finish() {
    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. pcd save will largely influence the real-time performences **/
    if (pcl_wait_save->size() > 0 && pcd_save_en) {
        std::string file_name = std::string("scans.pcd");
        std::string all_points_dir(std::string(std::string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        LOG(INFO) << "current scan saved to /PCD/" << file_name;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }

    LOG(INFO) << "finish done";
}
}  // namespace ll_localizer
#ifndef _LL_LOCALIZER_IMU_PROCESSING_H
#define _LL_LOCALIZER_IMU_PROCESSING_H

#include <glog/logging.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <cmath>
#include <deque>
#include <fstream>

#include "common_lib.h"
#include "so3_math.h"
#include "use-ikfom.hpp"
#include "utils.h"

namespace ll_localizer {

constexpr int MAX_INI_COUNT = 20;

bool time_list(const PointType &x, const PointType &y) { return (x.curvature < y.curvature); };

/// IMU Process and undistortion
class ImuProcess {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ImuProcess();
    ~ImuProcess();

    void Reset();
    void SetExtrinsic(const common::V3D &transl, const common::M3D &rot);
    void SetGyrCov(const common::V3D &scaler);
    void SetAccCov(const common::V3D &scaler);
    void SetGyrBiasCov(const common::V3D &b_g);
    void SetAccBiasCov(const common::V3D &b_a);
    void Process(const common::MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,
                 PointCloudType::Ptr pcl_un);

    std::ofstream fout_imu;
    Eigen::Matrix<double, 12, 12> Q;
    common::V3D cov_acc;
    common::V3D cov_gyr;
    common::V3D cov_acc_scale;
    common::V3D cov_gyr_scale;
    common::V3D cov_bias_gyr;
    common::V3D cov_bias_acc;

   private:
    void IMUInit(const common::MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N);
    void UndistortPcl(const common::MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,
                      PointCloudType &pcl_out);

    PointCloudType::Ptr cur_pcl_un;
    sensor_msgs::ImuConstPtr last_imu;
    std::deque<sensor_msgs::ImuConstPtr> v_imu;
    std::vector<common::Pose6D> IMUpose;
    std::vector<common::M3D> v_rot_pcl;
    common::M3D Lidar_R_wrt_IMU;
    common::V3D Lidar_T_wrt_IMU;
    common::V3D mean_acc;
    common::V3D mean_gyr;
    common::V3D angvel_last;
    common::V3D acc_s_last;
    double last_lidar_end_time = 0;
    int init_iter_num = 1;
    bool b_first_frame = true;
    bool imu_need_init = true;
};

ImuProcess::ImuProcess() : b_first_frame(true), imu_need_init(true) {
    init_iter_num = 1;
    Q = process_noise_cov();
    cov_acc = common::V3D(0.1, 0.1, 0.1);
    cov_gyr = common::V3D(0.1, 0.1, 0.1);
    cov_bias_gyr = common::V3D(0.0001, 0.0001, 0.0001);
    cov_bias_acc = common::V3D(0.0001, 0.0001, 0.0001);
    mean_acc = common::V3D(0, 0, -1.0);
    mean_gyr = common::V3D(0, 0, 0);
    angvel_last = common::Zero3d;
    Lidar_T_wrt_IMU = common::Zero3d;
    Lidar_R_wrt_IMU = common::Eye3d;
    last_imu.reset(new sensor_msgs::Imu());
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset() {
    mean_acc = common::V3D(0, 0, -1.0);
    mean_gyr = common::V3D(0, 0, 0);
    angvel_last = common::Zero3d;
    imu_need_init = true;
    init_iter_num = 1;
    v_imu.clear();
    IMUpose.clear();
    last_imu.reset(new sensor_msgs::Imu());
    cur_pcl_un.reset(new PointCloudType());
}

void ImuProcess::SetExtrinsic(const common::V3D &transl, const common::M3D &rot) {
    Lidar_T_wrt_IMU = transl;
    Lidar_R_wrt_IMU = rot;
}

void ImuProcess::SetGyrCov(const common::V3D &scaler) { cov_gyr_scale = scaler; }

void ImuProcess::SetAccCov(const common::V3D &scaler) { cov_acc_scale = scaler; }

void ImuProcess::SetGyrBiasCov(const common::V3D &b_g) { cov_bias_gyr = b_g; }

void ImuProcess::SetAccBiasCov(const common::V3D &b_a) { cov_bias_acc = b_a; }

void ImuProcess::IMUInit(const common::MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,
                         int &N) {
    /** 1. initializing the gravity, gyro bias, acc and gyro covariance
     ** 2. normalize the acceleration measurenments to unit gravity **/

    common::V3D cur_acc, cur_gyr;

    if (b_first_frame) {
        Reset();
        N = 1;
        b_first_frame = false;
        const auto &imu_acc = meas.imu.front()->linear_acceleration;
        const auto &gyr_acc = meas.imu.front()->angular_velocity;
        mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;
        mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
    }

    for (const auto &imu : meas.imu) {
        const auto &imu_acc = imu->linear_acceleration;
        const auto &gyr_acc = imu->angular_velocity;
        cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
        cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

        mean_acc += (cur_acc - mean_acc) / N;
        mean_gyr += (cur_gyr - mean_gyr) / N;

        cov_acc =
            cov_acc * (N - 1.0) / N + (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) * (N - 1.0) / (N * N);
        cov_gyr =
            cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) * (N - 1.0) / (N * N);

        N++;
    }
    state_ikfom init_state = kf_state.get_x();
    init_state.grav = S2(-mean_acc / mean_acc.norm() * common::G_m_s2);

    init_state.bg = mean_gyr;
    init_state.offset_T_L_I = Lidar_T_wrt_IMU;
    init_state.offset_R_L_I = Lidar_R_wrt_IMU;
    kf_state.change_x(init_state);

    esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = kf_state.get_P();
    init_P.setIdentity();
    init_P(6, 6) = init_P(7, 7) = init_P(8, 8) = 0.00001;
    init_P(9, 9) = init_P(10, 10) = init_P(11, 11) = 0.00001;
    init_P(15, 15) = init_P(16, 16) = init_P(17, 17) = 0.0001;
    init_P(18, 18) = init_P(19, 19) = init_P(20, 20) = 0.001;
    init_P(21, 21) = init_P(22, 22) = 0.00001;
    kf_state.change_P(init_P);
    last_imu = meas.imu.back();
}

void ImuProcess::UndistortPcl(const common::MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,
                              PointCloudType &pcl_out) {
    /*** add the imu of the last frame-tail to the of current frame-head ***/
    auto v_imu = meas.imu;
    v_imu.push_front(last_imu);
    const double &imu_beg_time = v_imu.front()->header.stamp.toSec();
    const double &imu_end_time = v_imu.back()->header.stamp.toSec();
    const double &pcl_beg_time = meas.lidar_bag_time;
    const double &pcl_end_time = meas.lidar_end_time;

    /*** sort point clouds by offset time ***/
    pcl_out = *(meas.lidar);
    sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);

    /*** Initialize IMU pose ***/
    state_ikfom imu_state = kf_state.get_x();
    IMUpose.clear();
    IMUpose.push_back(common::set_pose6d(0.0, acc_s_last, angvel_last, imu_state.vel, imu_state.pos,
                                          imu_state.rot.toRotationMatrix()));

    /*** forward propagation at each imu point ***/
    common::V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
    common::M3D R_imu;

    double dt = 0;

    input_ikfom in;
    for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++) {
        auto &&head = *(it_imu);
        auto &&tail = *(it_imu + 1);

        if (tail->header.stamp.toSec() < last_lidar_end_time) {
            continue;
        }

        angvel_avr << 0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
            0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
            0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
        acc_avr << 0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
            0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
            0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);

        acc_avr = acc_avr * common::G_m_s2 / mean_acc.norm();  // - state_inout.ba;

        if (head->header.stamp.toSec() < last_lidar_end_time) {
            dt = tail->header.stamp.toSec() - last_lidar_end_time;
        } else {
            dt = tail->header.stamp.toSec() - head->header.stamp.toSec();
        }

        in.acc = acc_avr;
        in.gyro = angvel_avr;
        Q.block<3, 3>(0, 0).diagonal() = cov_gyr;
        Q.block<3, 3>(3, 3).diagonal() = cov_acc;
        Q.block<3, 3>(6, 6).diagonal() = cov_bias_gyr;
        Q.block<3, 3>(9, 9).diagonal() = cov_bias_acc;
        kf_state.predict(dt, Q, in);

        /* save the poses at each IMU measurements */
        imu_state = kf_state.get_x();
        angvel_last = angvel_avr - imu_state.bg;
        acc_s_last = imu_state.rot * (acc_avr - imu_state.ba);
        for (int i = 0; i < 3; i++) {
            acc_s_last[i] += imu_state.grav[i];
        }

        double &&offs_t = tail->header.stamp.toSec() - pcl_beg_time;
        IMUpose.emplace_back(common::set_pose6d(offs_t, acc_s_last, angvel_last, imu_state.vel, imu_state.pos,
                                                 imu_state.rot.toRotationMatrix()));
    }

    /*** calculated the pos and attitude prediction at the frame-end ***/
    double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
    dt = note * (pcl_end_time - imu_end_time);
    kf_state.predict(dt, Q, in);

    imu_state = kf_state.get_x();
    last_imu = meas.imu.back();
    last_lidar_end_time = pcl_end_time;

    /*** undistort each lidar point (backward propagation) ***/
    if (pcl_out.points.empty()) {
        return;
    }
    auto it_pcl = pcl_out.points.end() - 1;
    for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--) {
        auto head = it_kp - 1;
        auto tail = it_kp;
        R_imu = common::MatFromArray(head->rot);
        vel_imu = common::VecFromArray(head->vel);
        pos_imu = common::VecFromArray(head->pos);
        acc_imu = common::VecFromArray(tail->acc);
        angvel_avr = common::VecFromArray(tail->gyr);

        for (; it_pcl->curvature / double(1000) > head->offset_time; it_pcl--) {
            dt = it_pcl->curvature / double(1000) - head->offset_time;

            /* Transform to the 'end' frame, using only the rotation
             * Note: Compensation direction is INVERSE of Frame's moving direction
             * So if we want to compensate a point at timestamp-i to the frame-e
             * p_compensate = R_imu_e ^ T * (R_i * P_i + T_ei) where T_ei is represented in global frame */
            common::M3D R_i(R_imu * Exp(angvel_avr, dt));

            common::V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);
            common::V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos);
            common::V3D p_compensate =
                imu_state.offset_R_L_I.conjugate() *
                (imu_state.rot.conjugate() * (R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) + T_ei) -
                 imu_state.offset_T_L_I);  // not accurate!

            // save Undistorted points and their rotation
            it_pcl->x = p_compensate(0);
            it_pcl->y = p_compensate(1);
            it_pcl->z = p_compensate(2);

            if (it_pcl == pcl_out.points.begin()) {
                break;
            }
        }
    }
}

void ImuProcess::Process(const common::MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,
                         PointCloudType::Ptr cur_pcl_un) {
    if (meas.imu.empty()) {
        return;
    }

    ROS_ASSERT(meas.lidar != nullptr);

    if (imu_need_init) {
        /// The very first lidar frame
        IMUInit(meas, kf_state, init_iter_num);

        imu_need_init = true;

        last_imu = meas.imu.back();

        state_ikfom imu_state = kf_state.get_x();
        if (init_iter_num > MAX_INI_COUNT) {
            cov_acc *= pow(common::G_m_s2 / mean_acc.norm(), 2);
            imu_need_init = false;

            cov_acc = cov_acc_scale;
            cov_gyr = cov_gyr_scale;
            LOG(INFO) << "IMU Initial Done";
            fout_imu.open(common::DEBUG_FILE_DIR("imu.txt"), std::ios::out);
        }

        return;
    }

    Timer::Evaluate([&, this]() { UndistortPcl(meas, kf_state, *cur_pcl_un); }, "Undistort Pcl");
}
}  // namespace ll_localizer

#endif

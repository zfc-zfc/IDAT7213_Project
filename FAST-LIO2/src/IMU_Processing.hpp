#ifndef _IMU_PROCESSING_HPP
#define _IMU_PROCESSING_HPP

#include <cmath>
#include <math.h>
#include <deque>
#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>
#include <ros/ros.h>
#include <so3_math.h>
#include <Eigen/Eigen>
#include <common_lib.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <condition_variable>
#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <fast_lio2/States.h>
#include <geometry_msgs/Vector3.h>


#define MAX_INI_COUNT (200)

const bool time_list(PointType &x, PointType &y) { return (x.curvature < y.curvature); };

// IMU Process and undistortion
class ImuProcess {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ImuProcess();

    ~ImuProcess();

    void Reset();

    void Reset(double start_timestamp, const sensor_msgs::ImuConstPtr &lastimu);

    void set_gyr_cov(const V3D &scaler);

    void set_acc_cov(const V3D &scaler);

    void set_gyr_bias_cov(const V3D &b_g);

    void set_acc_bias_cov(const V3D &b_a);

    void Process(const MeasureGroup &meas, StatesGroup &state, PointCloudXYZI::Ptr pcl_un_);

    ros::NodeHandle nh;
    ofstream fout_imu;
    V3D cov_acc;
    V3D cov_gyr;
    V3D cov_acc_scale;
    V3D cov_gyr_scale;
    V3D cov_bias_gyr;
    V3D cov_bias_acc;
    double first_lidar_time;
    int lidar_type;
    double IMU_mean_acc_norm;
    V3D offset_T_L_I;
private:
    void IMU_init(const MeasureGroup &meas, StatesGroup &state, int &N);

    void propagation_and_undist(const MeasureGroup &meas, StatesGroup &state_propagate, PointCloudXYZI &pcl_in_out);

    PointCloudXYZI::Ptr cur_pcl_un_;
    sensor_msgs::ImuConstPtr last_imu_;
    deque<sensor_msgs::ImuConstPtr> v_imu_;
    vector<Pose6D> IMUpose;
    V3D mean_acc;
    V3D mean_gyr;
    V3D angvel_last;
    V3D acc_s_last;
    double last_lidar_end_time_;
    double time_last_scan;      //上一帧开头的时间戳
    int init_iter_num = 1;
    bool b_first_frame_ = true;
    bool imu_need_init_ = true;
};

ImuProcess::ImuProcess()
        : b_first_frame_(true), imu_need_init_(true) {
    init_iter_num = 1;
    cov_acc = V3D(0.1, 0.1, 0.1);
    cov_gyr = V3D(0.1, 0.1, 0.1);
    cov_bias_gyr = V3D(0.0001, 0.0001, 0.0001);
    cov_bias_acc = V3D(0.0001, 0.0001, 0.0001);
    mean_acc = V3D(0, 0, -1.0);
    mean_gyr = V3D(0, 0, 0);
    angvel_last = Zero3d;
    last_imu_.reset(new sensor_msgs::Imu());
    fout_imu.open(DEBUG_FILE_DIR("imu.txt"), ios::out);
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset() {
    ROS_WARN("Reset ImuProcess");
    mean_acc = V3D(0, 0, -1.0);
    mean_gyr = V3D(0, 0, 0);
    angvel_last = Zero3d;
    imu_need_init_ = true;
    init_iter_num = 1;
    v_imu_.clear();
    IMUpose.clear();
    last_imu_.reset(new sensor_msgs::Imu());
    cur_pcl_un_.reset(new PointCloudXYZI());
}


void ImuProcess::set_gyr_cov(const V3D &scaler) {
    cov_gyr_scale = scaler;
}

void ImuProcess::set_acc_cov(const V3D &scaler) {
    cov_acc_scale = scaler;
}

void ImuProcess::set_gyr_bias_cov(const V3D &b_g) {
    cov_bias_gyr = b_g;
}

void ImuProcess::set_acc_bias_cov(const V3D &b_a) {
    cov_bias_acc = b_a;
}

void ImuProcess::IMU_init(const MeasureGroup &meas, StatesGroup &state_propagate, int &N) {
    /** 1. initializing the gravity, gyro bias, acc and gyro covariance
     ** 2. normalize the acceleration measurements to unit gravity **/
    ROS_INFO("IMU Initializing: %.1f %%", double(N) / MAX_INI_COUNT * 100);
    V3D cur_acc, cur_gyr;

    if (b_first_frame_) {
        Reset();
        N = 1;
        b_first_frame_ = false;
        const auto &imu_acc = meas.imu.front()->linear_acceleration;
        const auto &gyr_acc = meas.imu.front()->angular_velocity;
        mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;
        mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
        first_lidar_time = meas.lidar_beg_time;
        // cout<<"init acc norm: "<<mean_acc.norm()<<endl;
    }

    for (const auto &imu: meas.imu) {
        const auto &imu_acc = imu->linear_acceleration;
        const auto &gyr_acc = imu->angular_velocity;
        cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
        cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

        mean_acc += (cur_acc - mean_acc) / N;
        mean_gyr += (cur_gyr - mean_gyr) / N;

        cov_acc = cov_acc * (N - 1.0) / N + (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) * (N - 1.0) / (N * N);
        cov_gyr = cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) * (N - 1.0) / (N * N);

        N++;
    }

    state_propagate.gravity = -mean_acc / mean_acc.norm() * G_m_s2;
    state_propagate.rot_end = Eye3d;
    state_propagate.bias_g = mean_gyr;
//  state_propagate.bias_g.setZero();
    last_imu_ = meas.imu.back();
}

void ImuProcess::propagation_and_undist(const MeasureGroup &meas, StatesGroup &state_propagate, PointCloudXYZI &pcl_out) {
    /*** add the imu of the last frame-tail to the current frame-head ***/
    pcl_out = *(meas.lidar);//有畸变的一帧点云
    auto v_imu = meas.imu;
    v_imu.push_front(last_imu_);
    double imu_end_time = v_imu.back()->header.stamp.toSec();
    double pcl_beg_time, pcl_end_time;

    if (lidar_type == SIM) {
        pcl_beg_time = last_lidar_end_time_;
        pcl_end_time = meas.lidar_beg_time;
    } else {
        pcl_beg_time = meas.lidar_beg_time;
        /*** sort point clouds by offset time ***/
        sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);
        pcl_end_time = pcl_beg_time + pcl_out.points.back().curvature / double(1000);
    }


    /*** Initialize IMU pose ***/
    IMUpose.clear();
    IMUpose.push_back(
            set_pose6d(0.0, acc_s_last, angvel_last, state_propagate.vel_end, state_propagate.pos_end, state_propagate.rot_end));

    /*** forward propagation at each imu point ***/
    V3D acc_imu, angvel, acc, vel_imu(state_propagate.vel_end), pos_imu(state_propagate.pos_end);
    M3D R_imu(state_propagate.rot_end);
    MD(DIM_STATE, DIM_STATE) F_x;
    MD(DIM_STATE, 12) F_w;
    MD(12, 12) Sigma_w;
    Sigma_w.setZero();

    double dt = 0;
    for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++) {
        auto &&head = *(it_imu);
        auto &&tail = *(it_imu + 1);

        if (tail->header.stamp.toSec() < last_lidar_end_time_) continue;

        angvel <<  tail->angular_velocity.x, tail->angular_velocity.y, tail->angular_velocity.z;
        acc << tail->linear_acceleration.x, tail->linear_acceleration.y, tail->linear_acceleration.z;

        angvel -= state_propagate.bias_g;
        acc = acc / IMU_mean_acc_norm * G_m_s2 - state_propagate.bias_a;

        if (head->header.stamp.toSec() < last_lidar_end_time_)
            dt = tail->header.stamp.toSec() - last_lidar_end_time_;
        else
            dt = tail->header.stamp.toSec() - head->header.stamp.toSec();


        M3D acc_skew;
        acc_skew << SKEW_SYM_MATRX(acc);

        //Fill in process noise matrix Sigma_w
        Sigma_w.block<3, 3>(0, 0).diagonal() = cov_gyr;
        Sigma_w.block<3, 3>(3, 3).diagonal() = cov_acc;
        Sigma_w.block<3, 3>(6, 6).diagonal() = cov_bias_gyr;
        Sigma_w.block<3, 3>(6, 6).diagonal() = cov_bias_acc;

        //Computation of F_x, F_w Matrices
        F_x.setIdentity();
        F_w.setZero();
        F_x.block<3, 3>(0, 0) = Exp(angvel, -dt);
        F_x.block<3, 3>(0, 9) = -Eye3d * dt;
        F_x.block<3, 3>(3, 6) = Eye3d * dt;
        F_x.block<3, 3>(6, 0) = -R_imu * acc_skew * dt;
        F_x.block<3, 3>(6, 12) = -R_imu * dt;
        F_x.block<3, 3>(6, 15) = Eye3d * dt;

        F_w.block<3, 3>(0, 0) = -M3D::Identity() * dt;
        F_w.block<3, 3>(6, 3) = -R_imu * dt;
        F_w.block<3, 3>(9, 6) = -M3D::Identity() * dt;
        F_w.block<3, 3>(12, 9) = -M3D::Identity() * dt;

        // Prediction of covariance
        state_propagate.cov = F_x * state_propagate.cov * F_x.transpose()
                + F_w * Sigma_w * F_w.transpose();

        // Prediction of attitude in global frame
        R_imu = R_imu * Exp(angvel, dt);

        // Prediction of IMU position in global frame
        pos_imu = pos_imu + vel_imu * dt;

        // Prediction of velocity in global frame
        vel_imu = vel_imu + (R_imu * acc + state_propagate.gravity) * dt;

        /* save the poses at each IMU measurements (global frame)*/
        // Specific acceleration in global frame of IMU
        acc_imu = R_imu * acc + state_propagate.gravity;
        angvel_last = angvel;
        acc_s_last = acc_imu;
        double &&offs_t = tail->header.stamp.toSec() - pcl_beg_time;
        IMUpose.push_back(set_pose6d(offs_t, acc_imu, angvel, vel_imu, pos_imu, R_imu));
    }

    /*** calculated the pos and attitude prediction at the frame-end ***/
    double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
    dt = note * (pcl_end_time - imu_end_time);
    state_propagate.vel_end = vel_imu + note * acc_imu * dt;
    state_propagate.rot_end = R_imu * Exp(V3D(note * angvel), dt);
    state_propagate.pos_end = pos_imu + note * vel_imu * dt + note * 0.5 * acc_imu * dt * dt;


    last_imu_ = meas.imu.back();
    last_lidar_end_time_ = pcl_end_time;

    if (lidar_type != SIM) {
        auto pos_liD_e = state_propagate.pos_end + state_propagate.rot_end * offset_T_L_I;
        /*** un-distort each lidar point (backward propagation) ***/
        auto it_pcl = pcl_out.points.end() - 1; //a single point in k-th frame
        for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--) {
            auto head = it_kp - 1;
            R_imu << MAT_FROM_ARRAY(head->rot);//
            acc_imu << VEC_FROM_ARRAY(head->acc);

            vel_imu << VEC_FROM_ARRAY(head->vel);
            pos_imu << VEC_FROM_ARRAY(head->pos);
            angvel << VEC_FROM_ARRAY(head->gyr);

            for (; it_pcl->curvature / double(1000) > head->offset_time; it_pcl--) {
                dt = it_pcl->curvature / double(1000) - head->offset_time; //dt = t_j - t_i > 0

                /* Transform to the 'scan-end' IMU frame（I_k frame）using only rotation
                * Note: Compensation direction is INVERSE of Frame's moving direction
                * So if we want to compensate a point at timestamp-i to the frame-e
                * P_compensate = R_imu_e ^ T * (R_i * P_i + T_ei) where T_ei is represented in global frame */

                M3D R_i(R_imu * Exp(angvel, dt));
                V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt + R_i * offset_T_L_I - pos_liD_e);

                V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);
                V3D P_compensate = state_propagate.rot_end.transpose() * (R_i * P_i + T_ei);

                /// save Undistorted points and their rotation
                it_pcl->x = P_compensate(0);
                it_pcl->y = P_compensate(1);
                it_pcl->z = P_compensate(2);

                if (it_pcl == pcl_out.points.begin()) break;
            }
        }
    }

}


void ImuProcess::Process(const MeasureGroup &meas, StatesGroup &stat, PointCloudXYZI::Ptr cur_pcl_un_) {
    if (meas.imu.empty()) return;
    ROS_ASSERT(meas.lidar != nullptr);

    if (imu_need_init_) {

        /// The very first lidar frame
        IMU_init(meas, stat, init_iter_num);
        imu_need_init_ = true;
        last_imu_ = meas.imu.back();
        if (init_iter_num > MAX_INI_COUNT) {
            cov_acc *= pow(G_m_s2 / mean_acc.norm(), 2);
            imu_need_init_ = false;

            cov_acc = cov_acc_scale;
            cov_gyr = cov_gyr_scale;

            ROS_INFO("IMU Initialization Done: Gravity: %.4f %.4f %.4f, Acc norm: %.4f", stat.gravity[0],
                     stat.gravity[1], stat.gravity[2], mean_acc.norm());
            ROS_INFO("IMU Initialization Done: Gyro_bias: %.4f, %.4f, %.4f", stat.bias_g[0], stat.bias_g[1],
                     stat.bias_g[2]);
            IMU_mean_acc_norm = mean_acc.norm();
        }
        return;
    }
    propagation_and_undist(meas, stat, *cur_pcl_un_);
}

#endif

// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <omp.h>
#include "IMU_Processing.hpp"
#include <unistd.h>
#include <Python.h>
#include <Eigen/Core>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <algorithm>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <livox_ros_driver/CustomMsg.h>
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>

#ifndef DEPLOY
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
#endif

#define INIT_TIME           (0.0)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)

float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;

mutex mtx_buffer, mtx_buffer_gt;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;
string map_file_path, lid_topic, imu_topic;

int feats_down_size = 0, laserCloudValidNum = 0, \
 effect_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;

double res_mean_last = 0.05;
double gyr_cov = 0.1, acc_cov = 0.1, grav_cov = 0.0001, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double last_timestamp_lidar = 0, last_timestamp_imu = 0.0;
double filter_size_surf_min = 0, filter_size_map_min = 0;
double cube_len = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;

PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
pcl::PCDWriter pcd_writer;
string all_points_dir(string(root_dir + "/PCD/Map") + string(".pcd"));

// Time Log Variables
int kdtree_delete_counter = 0, kdtree_size_st = 0, add_point_size = 0;
double search_time_rec[100000];

int lidar_type, pcd_save_interval = -1, pcd_index = 0;
bool lidar_pushed, flg_reset, flg_exit = false;
bool scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;
bool pcd_save_en = false, path_en = true;

bool cut_frame = false;
int cut_frame_num = 1, orig_odom_freq = 10, frame_num = 0, imu_num = 0;
vector<double> Trans_LI_cov(3, 0.000);
vector<double> Rot_LI_cov(3, 0.000);
V3D mean_acc = Zero3d;

vector<BoxPointType> cub_needrm;
vector<double> extrinT(3, 0.0);
vector<double> extrinR(9, 0.0);
deque<PointCloudXYZI::Ptr> lidar_buffer;
deque<double> time_buffer;
deque<sensor_msgs::Imu::ConstPtr> imu_buffer;
vector<vector<int>> pointSearchInd_surf;
vector<PointVector> Nearest_Points;
bool point_selected_surf[100000] = {0};
float res_last[100000] = {0.0};
double total_residual;

//surf feature in map
PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr _featsArray;

pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

KD_TREE ikdtree;

M3D last_rot(M3D::Zero());
V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D position_last(Zero3d);
V3D last_odom(Zero3d);


//Accuracy Comparison
ofstream fout_pose;
string sub_gt_pose_topic;
deque<VD(7) > ground_truth;
deque<VD(4) > LIO_pose;


//estimator inputs and output;
MeasureGroup Measures;
StatesGroup state;

nav_msgs::Path path;
nav_msgs::Odometry odomAftMapped;
geometry_msgs::Quaternion geoQuat;
geometry_msgs::PoseStamped msg_body_pose;
sensor_msgs::Imu IMU_sync;

shared_ptr<Preprocess> p_pre(new Preprocess());

//IDAT7213
string student_name;
double student_ID;
bool print_student_info{false};
double id_max = 3036107751;
double id_min = 3035124427;
bool first_lidar_frame = true;
double data_start_time;
double last_lidar_time;
M3D offset_R_L_I;
V3D offset_T_L_I;

float calc_dist(PointType p1, PointType p2) {
    float d = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
    return d;
}

void calcBodyVar(Eigen::Vector3d &pb, const float range_inc,
                 const float degree_inc, Eigen::Matrix3d &var) {
    float range = sqrt(pb[0] * pb[0] + pb[1] * pb[1] + pb[2] * pb[2]);
    float range_var = range_inc * range_inc;
    Eigen::Matrix2d direction_var;
    direction_var << pow(sin(DEG2RAD(degree_inc)), 2), 0, 0,
            pow(sin(DEG2RAD(degree_inc)), 2);
    Eigen::Vector3d direction(pb);
    direction.normalize();
    Eigen::Matrix3d direction_hat;
    direction_hat << 0, -direction(2), direction(1), direction(2), 0,
            -direction(0), -direction(1), direction(0), 0;
    Eigen::Vector3d base_vector1(1, 1,
                                 -(direction(0) + direction(1)) / direction(2));
    base_vector1.normalize();
    Eigen::Vector3d base_vector2 = base_vector1.cross(direction);
    base_vector2.normalize();
    Eigen::Matrix<double, 3, 2> N;
    N << base_vector1(0), base_vector2(0), base_vector1(1), base_vector2(1),
            base_vector1(2), base_vector2(2);
    Eigen::Matrix<double, 3, 2> A = range * direction_hat * N;
    var = direction * range_var * direction.transpose() +
          A * direction_var * A.transpose();
}

void SigHandle(int sig) {
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
    sig_buffer.notify_all();
}

inline void dump_lio_state_to_log(FILE *fp) {
    V3D rot_ang(Log(state.rot_end));
    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                   // Angle
    fprintf(fp, "%lf %lf %lf ", state.pos_end(0), state.pos_end(1), state.pos_end(2)); // Pos
    fprintf(fp, "%lf %lf %lf ", state.vel_end(0), state.vel_end(1), state.vel_end(2)); // Vel
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc
    fprintf(fp, "%lf %lf %lf ", state.bias_g(0), state.bias_g(1), state.bias_g(2));    // Bias_g
    fprintf(fp, "%lf %lf %lf ", state.bias_a(0), state.bias_a(1), state.bias_a(2));    // Bias_a
    fprintf(fp, "%lf %lf %lf ", state.gravity(0), state.gravity(1), state.gravity(2)); // Bias_a  
    fprintf(fp, "\r\n");
    fflush(fp);
}

void pointBodyToWorld(PointType const *const pi, PointType *const po) {
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state.rot_end * (offset_R_L_I * p_body + offset_T_L_I) + state.pos_end);
    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->normal_x = pi->normal_x;
    po->normal_y = pi->normal_y;
    po->normal_z = pi->normal_z;
    po->intensity = pi->intensity;
}

template<typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po) {
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state.rot_end * (offset_R_L_I * p_body + offset_T_L_I) + state.pos_end);
    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const *const pi, PointTypeRGB *const po) {
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state.rot_end * (offset_R_L_I * p_body + offset_T_L_I) + state.pos_end);
    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->r = pi->normal_x;
    po->g = pi->normal_y;
    po->b = pi->normal_z;

    float intensity = pi->intensity;
    intensity = intensity - floor(intensity);

    int reflection_map = intensity * 10000;
}

int points_cache_size = 0;

void points_cache_collect() {
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    points_cache_size = points_history.size();
    for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
}


BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;

void lasermap_fov_segment() {
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    V3D pos_LiD = state.pos_end;
    if (!Localmap_Initialized) {
        //if (cube_len <= 2.0 * MOV_THRESHOLD * DET_RANGE) throw std::invalid_argument("[Error]: Local Map Size is too small! Please change parameter \"cube_side_length\" to larger than %d in the launch file.\n");
        for (int i = 0; i < 3; i++) {
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    // printf("Local Map is (%0.2f,%0.2f) (%0.2f,%0.2f) (%0.2f,%0.2f)\n", LocalMap_Points.vertex_min[0],LocalMap_Points.vertex_max[0],LocalMap_Points.vertex_min[1],LocalMap_Points.vertex_max[1],LocalMap_Points.vertex_min[2],LocalMap_Points.vertex_max[2]);
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++) {
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE ||
            dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
            need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9,
                         double(DET_RANGE * (MOV_THRESHOLD - 1)));
    for (int i = 0; i < 3; i++) {
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE) {
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
            // printf("Delete Box is (%0.2f,%0.2f) (%0.2f,%0.2f) (%0.2f,%0.2f)\n", tmp_boxpoints.vertex_min[0],tmp_boxpoints.vertex_max[0],tmp_boxpoints.vertex_min[1],tmp_boxpoints.vertex_max[1],tmp_boxpoints.vertex_min[2],tmp_boxpoints.vertex_max[2]);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) {
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
            // printf("Delete Box is (%0.2f,%0.2f) (%0.2f,%0.2f) (%0.2f,%0.2f)\n", tmp_boxpoints.vertex_min[0],tmp_boxpoints.vertex_max[0],tmp_boxpoints.vertex_min[1],tmp_boxpoints.vertex_max[1],tmp_boxpoints.vertex_min[2],tmp_boxpoints.vertex_max[2]);                     
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    if (cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    // printf("Delete Box: %d\n",int(cub_needrm.size()));
}

void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg) {
    mtx_buffer.lock();


    //IDAT7213
    if(first_lidar_frame){
        double first_pcl_time = msg->header.stamp.toSec();
        double nume = student_ID - id_min;
        double deno = id_max - id_min;
        data_start_time = first_pcl_time + 3.0 * nume/deno;
        first_lidar_frame = false;
    }
    if(msg->header.stamp.toSec() < data_start_time){
        mtx_buffer.unlock();
        sig_buffer.notify_all();
        return;
    }

    scan_count++;

    if (msg->header.stamp.toSec() < last_timestamp_lidar) {
        ROS_WARN("lidar loop back, clear buffer");
        lidar_buffer.clear();
        time_buffer.clear();
    }
    last_timestamp_lidar = msg->header.stamp.toSec();

    if (cut_frame) {
        deque<PointCloudXYZI::Ptr> ptr;
        deque<double> timestamp_lidar;
        p_pre->process_cut_frame_livox(msg, ptr, timestamp_lidar, cut_frame_num, scan_count);
        while (!ptr.empty() && !timestamp_lidar.empty()) {
            lidar_buffer.push_back(ptr.front());
            ptr.pop_front();
            time_buffer.push_back(timestamp_lidar.front() / double(1000));
            timestamp_lidar.pop_front();
        }
    } else {
        PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
        p_pre->process(msg, ptr);
        lidar_buffer.push_back(ptr);
        time_buffer.push_back(last_timestamp_lidar);
    }

    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    mtx_buffer.lock();

    //IDAT7213
    if(first_lidar_frame){
        double first_pcl_time = msg->header.stamp.toSec();
        double nume = student_ID - id_min;
        double deno = id_max - id_min;
        data_start_time = first_pcl_time + 5.0 * nume/deno;
        first_lidar_frame = false;
    }

    if(msg->header.stamp.toSec() < data_start_time){
        mtx_buffer.unlock();
        sig_buffer.notify_all();
        return;
    }

    scan_count++;
    if (msg->header.stamp.toSec() < last_timestamp_lidar) {
        ROS_ERROR("lidar loop back, clear Lidar buffer.");
        lidar_buffer.clear();
        time_buffer.clear();
    }
    last_timestamp_lidar = msg->header.stamp.toSec();

    if (lidar_type == VELO || lidar_type == OUSTER || lidar_type == PANDAR && cut_frame) {
        deque<PointCloudXYZI::Ptr> ptr;
        deque<double> timestamp_lidar;
        p_pre->process_cut_frame_pcl2(msg, ptr, timestamp_lidar, cut_frame_num, scan_count);
        while (!ptr.empty() && !timestamp_lidar.empty()) {
            lidar_buffer.push_back(ptr.front());
            ptr.pop_front();
            time_buffer.push_back(timestamp_lidar.front() / double(1000));//unit:s
            timestamp_lidar.pop_front();
        }
    } else {
        PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
        p_pre->process(msg, ptr);
        lidar_buffer.push_back(ptr);
        time_buffer.push_back(msg->header.stamp.toSec());
    }

    mtx_buffer.unlock();
    sig_buffer.notify_all();
}


void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg) {
    publish_count++;
    mtx_buffer.lock();
    double timestamp = msg->header.stamp.toSec();

    if(timestamp < data_start_time){
        mtx_buffer.unlock();
        sig_buffer.notify_all();
        return;
    }

    if (timestamp < last_timestamp_imu) {
        ROS_WARN("IMU loop back, clear IMU buffer.");
        imu_buffer.clear();
    }
    last_timestamp_imu = timestamp;
    imu_buffer.push_back(msg);

    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

bool sync_packages(MeasureGroup &meas) {
    if (lidar_buffer.empty() || imu_buffer.empty())
        return false;

    /** push a lidar scan **/
    if (!lidar_pushed) {
        meas.lidar = lidar_buffer.front();

        if (meas.lidar->points.size() <= 1) {
            ROS_WARN("Too few input point cloud!\n");
            lidar_buffer.pop_front();
            time_buffer.pop_front();
            return false;
        }
        meas.lidar_beg_time = time_buffer.front();
        if (lidar_type == SIM)
            lidar_end_time = meas.lidar_beg_time;
        else
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);

        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time)
        return false;

    /** push imu data, and pop from imu buffer **/
    double imu_time = imu_buffer.front()->header.stamp.toSec();
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time)) {
        imu_time = imu_buffer.front()->header.stamp.toSec();
        if (imu_time > lidar_end_time) break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }
    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

void gt_pos_cbk_sim(const nav_msgs::Odometry::ConstPtr &msg) {
    VD(7) gt;
    gt(6) = msg->header.stamp.toSec();
    gt.block<3, 1>(3, 0) = V3D(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    M3D gt_rot = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                    msg->pose.pose.orientation.y, msg->pose.pose.orientation.z).matrix();
    gt.block<3, 1>(0, 0) = RotMtoEuler(gt_rot);
    mtx_buffer_gt.lock();
    ground_truth.push_back(gt);
    mtx_buffer_gt.unlock();
}

void TimerCbk(const ros::TimerEvent &e) {
    if (LIO_pose.empty())
        return;
    while (ground_truth[1](6) < LIO_pose.front()(3) && ground_truth.size() > 2) {
        mtx_buffer_gt.lock();
        ground_truth.pop_front();
        mtx_buffer_gt.unlock();
    }
    if (ground_truth[0](6) < LIO_pose.front()(3) && ground_truth[1](6) >= LIO_pose.front()(3)) {
        //Linear Interpolate
        double DT = ground_truth[1](6) - ground_truth[0](6);
        double dt = LIO_pose.front()(3) - ground_truth[0](6);
        double s = dt / DT;
        V3D gt_rot_euler = s * ground_truth[1].block<3, 1>(0, 0) + (1 - s) * ground_truth[0].block<3, 1>(0, 0);
        V3D gt_position = s * ground_truth[1].block<3, 1>(3, 0) + (1 - s) * ground_truth[0].block<3, 1>(3, 0);
        fout_pose << setiosflags(ios::fixed) << setprecision(6) << LIO_pose.front().block<3, 1>(0, 0).transpose() << " "
                  << gt_position.transpose() << " " << LIO_pose.front()(3) << total_distance << endl;
        LIO_pose.pop_front();
    }
}


bool sync_packages_only_lidar(MeasureGroup &meas) {
    if (lidar_buffer.empty())
        return false;

    /** push a lidar scan **/
    if (!lidar_pushed) {
        meas.lidar = lidar_buffer.front();

        if (meas.lidar->points.size() <= 1) {
            ROS_WARN("Too few input point cloud!\n");
            lidar_buffer.pop_front();
            time_buffer.pop_front();
            return false;
        }

        meas.lidar_beg_time = time_buffer.front();

        if (lidar_type == L515)
            lidar_end_time = meas.lidar_beg_time;
        else
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);

        lidar_pushed = true;
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}


void map_incremental() {
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++) {
        /* transform to world frame */
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        /* decide if need add to map */
        if (!Nearest_Points[i].empty()) {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point;
            mid_point.x = floor(feats_down_world->points[i].x / filter_size_map_min) * filter_size_map_min +
                          0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y / filter_size_map_min) * filter_size_map_min +
                          0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z / filter_size_map_min) * filter_size_map_min +
                          0.5 * filter_size_map_min;
            float dist = calc_dist(feats_down_world->points[i], mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min &&
                fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min &&
                fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min) {
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i++) {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist) {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
        } else {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }


    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false);
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
}

void publish_frame_world(const ros::Publisher &pubLaserCloudFullRes) {
    if (scan_pub_en) {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
        int size = laserCloudFullRes->points.size();

        PointCloudXYZRGB::Ptr laserCloudWorldRGB(new PointCloudXYZRGB(size, 1));
        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++) {
            if (lidar_type == L515)
                RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
                                &laserCloudWorldRGB->points[i]);
            else
                pointBodyToWorld(&laserCloudFullRes->points[i], \
                                &laserCloudWorld->points[i]);
        }

        sensor_msgs::PointCloud2 laserCloudmsg;
        if (lidar_type == L515)
            pcl::toROSMsg(*laserCloudWorldRGB, laserCloudmsg);
        else
            pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);

        laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudFullRes.publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }


    /**************** save map ****************/
    /* 1. make sure you have enough memories
       2. noted that pcd save will influence the real-time performences **/
    if (pcd_save_en) {
        boost::filesystem::create_directories(root_dir + "/PCD");
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));
        for (int i = 0; i < size; i++) {
            pointBodyToWorld(&feats_undistort->points[i], &laserCloudWorld->points[i]);
        }

        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval) {
            pcd_index++;
            all_points_dir = string(root_dir + "/PCD/PCD") + to_string(pcd_index) + string(".pcd");
            cout << "current scan saved to " << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
}

void publish_frame_body(const ros::Publisher &pubLaserCloudFullRes_body) {
    PointCloudXYZI::Ptr laserCloudFullRes(feats_undistort);
    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*feats_undistort, laserCloudmsg);
    // laserCloudmsg.header.stamp = ros::Time::now();//.fromSec(lidar_end_time);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "camera_init";
    pubLaserCloudFullRes_body.publish(laserCloudmsg);
}

void publish_effect_world(const ros::Publisher &pubLaserCloudEffect) {
    PointCloudXYZI::Ptr laserCloudWorld(\
                    new PointCloudXYZI(effect_feat_num, 1));
    for (int i = 0; i < effect_feat_num; i++) {
        pointBodyToWorld(&laserCloudOri->points[i], &laserCloudWorld->points[i]);
    }
    sensor_msgs::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    // laserCloudFullRes3.header.stamp = ros::Time::now();
    laserCloudFullRes3.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudFullRes3.header.frame_id = "camera_init";
    pubLaserCloudEffect.publish(laserCloudFullRes3);
}

void publish_map(const ros::Publisher &pubLaserCloudMap) {
    sensor_msgs::PointCloud2 laserCloudMap;
    pcl::toROSMsg(*featsFromMap, laserCloudMap);
    // laserCloudMap.header.stamp = ros::Time::now();
    laserCloudMap.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudMap.header.frame_id = "camera_init";
    pubLaserCloudMap.publish(laserCloudMap);
}

template<typename T>
void set_posestamp(T &out) {
    out.position.x = state.pos_end(0);
    out.position.y = state.pos_end(1);
    out.position.z = state.pos_end(2);
    out.orientation.x = geoQuat.x;
    out.orientation.y = geoQuat.y;
    out.orientation.z = geoQuat.z;
    out.orientation.w = geoQuat.w;
}

void publish_odometry(const ros::Publisher &pubOdomAftMapped) {
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "aft_mapped";
    odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time);
    set_posestamp(odomAftMapped.pose.pose);

    pubOdomAftMapped.publish(odomAftMapped);

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x, \
                                    odomAftMapped.pose.pose.position.y, \
                                    odomAftMapped.pose.pose.position.z));
    q.setW(odomAftMapped.pose.pose.orientation.w);
    q.setX(odomAftMapped.pose.pose.orientation.x);
    q.setY(odomAftMapped.pose.pose.orientation.y);
    q.setZ(odomAftMapped.pose.pose.orientation.z);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "camera_init", "aft_mapped"));
}

void publish_mavros(const ros::Publisher &mavros_pose_publisher) {
    // msg_body_pose.header.stamp = ros::Time::now();
    msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);

    msg_body_pose.header.frame_id = "camera_odom_frame";
    set_posestamp(msg_body_pose.pose);
    mavros_pose_publisher.publish(msg_body_pose);
}

void publish_path(const ros::Publisher pubPath) {
    set_posestamp(msg_body_pose.pose);
    // msg_body_pose.header.stamp = ros::Time::now();
    msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = "camera_init";
    static int jjj = 0;
    jjj++;
    if (jjj % 2 == 0) // if path is too large, the rvis will crash
    {
        path.poses.push_back(msg_body_pose);
        pubPath.publish(path);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "laserMapping");
    ros::NodeHandle nh;

    nh.param<int>("mapping/point_filter_num", p_pre->point_filter_num, 2);
    nh.param<string>("map_file_path", map_file_path, "");
    nh.param<string>("common/lid_topic", lid_topic, "/livox/lidar");
    nh.param<string>("common/imu_topic", imu_topic, "/livox/imu");
    nh.param<string>("common/sub_gt_pose_topic", sub_gt_pose_topic, "");
    nh.param<double>("common/last_lidar_time", last_lidar_time, 0.0);
    nh.param<double>("student/ID", student_ID, 5124427);
    nh.param<string>("student/Name", student_name, "FAST-LIO2");
    nh.param<double>("mapping/filter_size_surf", filter_size_surf_min, 0.5);
    nh.param<double>("mapping/filter_size_map", filter_size_map_min, 0.5);
    nh.param<double>("mapping/cube_side_length", cube_len, 200);
    nh.param<float>("mapping/det_range", DET_RANGE, 300.f);
    nh.param<double>("mapping/gyr_cov", gyr_cov, 0.1);
    nh.param<double>("mapping/acc_cov", acc_cov, 0.1);
    nh.param<double>("mapping/grav_cov", grav_cov, 0.001);
    nh.param<double>("mapping/b_gyr_cov", b_gyr_cov, 0.0001);
    nh.param<double>("mapping/b_acc_cov", b_acc_cov, 0.0001);
    nh.param<double>("preprocess/blind", p_pre->blind, 1.0);
    nh.param<int>("preprocess/lidar_type", lidar_type, LIVOX);
    nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 16);
    nh.param<bool>("preprocess/feature_extract_en", p_pre->feature_enabled, 0);
    nh.param<int>("calibration/orig_odom_freq", orig_odom_freq, 10);
    nh.param<bool>("publish/path_en", path_en, true);
    nh.param<bool>("publish/scan_publish_en", scan_pub_en, 1);
    nh.param<bool>("publish/dense_publish_en", dense_pub_en, 1);
    nh.param<bool>("publish/scan_bodyframe_pub_en", scan_body_pub_en, 1);
    nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en, false);
    nh.param<int>("pcd_save/interval", pcd_save_interval, -1);
    nh.param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>());
    nh.param<vector<double>>("mapping/extrinsic_R", extrinR, vector<double>());


    cout << "lidar_type: " << lidar_type << endl;
    cout << "Run Fast-LIO 2.0." << endl;
    path.header.stamp = ros::Time().fromSec(lidar_end_time);
    path.header.frame_id = "camera_init";

    /*** variables definition ***/
    VD(DIM_STATE) solution;

    // Matrix definition and initialization
    MD(DIM_STATE, DIM_STATE) I_;
    I_.setIdentity();

    StatesGroup state_propagat;
    PointType pointOri, pointSel, coeff;

    //Set extrinsic between lidar and imu
    offset_R_L_I << MAT_FROM_ARRAY(extrinR);
    offset_T_L_I << VEC_FROM_ARRAY(extrinT);

    _featsArray.reset(new PointCloudXYZI());
    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));

    shared_ptr<ImuProcess> p_imu(new ImuProcess());
    p_pre->DET_RANGE = DET_RANGE;
    p_imu->lidar_type = p_pre->lidar_type = lidar_type;
    p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
    p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));
    p_imu->offset_T_L_I = offset_T_L_I;



    /*** debug record ***/
    FILE *fp;
    string pos_log_dir = root_dir + "/Log/pos_log.txt";
    fp = fopen(pos_log_dir.c_str(), "w");

    ofstream fout_pre, fout_out, fout_dbg;
    fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"), ios::out);
    fout_out.open(DEBUG_FILE_DIR("mat_out.txt"), ios::out);
    fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"), ios::out);
    fout_pose.open(DEBUG_FILE_DIR("pose.txt"), ios::out);
    if (fout_pre && fout_out)
        cout << "~~~~" << ROOT_DIR << " file opened" << endl;
    else
        cout << "~~~~" << ROOT_DIR << " doesn't exist" << endl;


    /*** ROS subscribe initialization ***/
    ros::Subscriber sub_pcl = p_pre->lidar_type == LIVOX ? \
        nh.subscribe(lid_topic, 200000, livox_pcl_cbk) : \
         nh.subscribe(lid_topic, 200000, standard_pcl_cbk);


    ros::Subscriber sub_imu = nh.subscribe<sensor_msgs::Imu>(imu_topic, 200000, imu_cbk);

    ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered", 100000);
    ros::Publisher pubLaserCloudFullRes_body = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered_body", 100000);
    ros::Publisher pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_effected", 100000);
    ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>
            ("/Laser_map", 100000);
    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>
            ("/aft_mapped_to_init", 100000);
    ros::Publisher pubPath = nh.advertise<nav_msgs::Path>
            ("/path", 100000);

    ground_truth.clear();
    ros::Timer timer = nh.createTimer(ros::Duration(0.01), TimerCbk);


    ros::Subscriber groundtruth_subscriber;
    if (lidar_type == SIM) {
        groundtruth_subscriber = nh.subscribe<nav_msgs::Odometry>
                (sub_gt_pose_topic, 10000, gt_pos_cbk_sim);
    }

    //ros::Publisher mavros_pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);

//------------------------------------------------------------------------------------------------------
    signal(SIGINT, SigHandle);
    ros::Rate rate(5000);
    bool status = ros::ok();

    while (status) {
        if (flg_exit) break;
        ros::spinOnce();
        if (sync_packages(Measures)) {
            if (flg_reset) {
                ROS_WARN("reset when rosbag play back.");
                p_imu->Reset();
                flg_reset = false;
                continue;
            }

            p_imu->Process(Measures, state, feats_undistort);
            //feats_undistort是L_k系中无畸变的点云坐标
            state_propagat = state;

            if (feats_undistort->empty() || (feats_undistort == NULL)) {
                first_lidar_time = Measures.lidar_beg_time;
                p_imu->first_lidar_time = first_lidar_time;
                ROS_WARN("FAST-LIO not ready, no points stored.");
                continue;
            }



            /*** Segment the map in lidar FOV ***/
            lasermap_fov_segment();

            /*** downsample the feature points in a scan ***/
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);
            feats_down_size = feats_down_body->points.size();
            /*** initialize the map kdtree ***/
            if (ikdtree.Root_Node == nullptr) {
                if (feats_down_size > 5) {
                    ikdtree.set_downsample_param(filter_size_map_min);
                    feats_down_world->resize(feats_down_size);
                    for (int i = 0; i < feats_down_size; i++) {
                        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                    }
                    ikdtree.Build(feats_down_world->points);
                }
                continue;
            }
            int featsFromMapNum = ikdtree.validnum();
            kdtree_size_st = ikdtree.size();

            if(!print_student_info){
                cout << endl << endl;
                cout << BOLDWHITE << " ============================== " << endl;
                cout << BOLDWHITE << "  Student ID:   " << BOLDGREEN << setprecision(10) << student_ID << endl
                     << BOLDWHITE << "  Student Name: " << BOLDGREEN << student_name << endl;
                cout << BOLDWHITE << " ============================== " << endl;
                cout << RESET << endl << endl;
                print_student_info = true;
            }

            /*** ICP and iterated Kalman filter update ***/
            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);
            euler_cur = RotMtoEuler(state.rot_end);

            pointSearchInd_surf.resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);
            int rematch_num = 0;
            bool nearest_search_en = true; //

            /*** state estimation ***/
            std::vector<M3D> body_var;
            std::vector<M3D> crossmat_list;
            body_var.reserve(feats_down_size);
            crossmat_list.reserve(feats_down_size);


            laserCloudOri->clear();
            corr_normvect->clear();
            total_residual = 0.0;
            /** closest surface search and residual computation **/
            //Multi-thread
#ifdef MP_EN
            omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
            //State Update
            for (int i = 0; i < feats_down_size; i++) {
                PointType &point_body = feats_down_body->points[i];
                PointType &point_world = feats_down_world->points[i];
                V3D p_body(point_body.x, point_body.y, point_body.z);
                // transform to world frame
                pointBodyToWorld(&point_body, &point_world);
                vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
                auto &points_near = Nearest_Points[i];
                uint8_t search_flag = 0;
                if (nearest_search_en) {
                    /** Find the closest planes in the map **/
                    ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis, 5);
                    if (points_near.size() < NUM_MATCH_POINTS)
                        point_selected_surf[i] = false;
                    else
                        point_selected_surf[i] = pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
                }

                res_last[i] = -1000.0f;
                if (!point_selected_surf[i] || points_near.size() < NUM_MATCH_POINTS) {
                    point_selected_surf[i] = false;
                    continue;
                }
                point_selected_surf[i] = false;
                VD(4) pabcd;
                pabcd.setZero();
                if (esti_plane(pabcd, points_near, 0.1)) //(planeValid)
                {
                    float pd2 =
                            pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
                    float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());
                    if (s > 0.9) {
                        point_selected_surf[i] = true;
                        normvec->points[i].x = pabcd(0);
                        normvec->points[i].y = pabcd(1);
                        normvec->points[i].z = pabcd(2);
                        normvec->points[i].intensity = pd2;
                        res_last[i] = abs(pd2);
                    }
                }
            }
            effect_feat_num = 0;
            for (int i = 0; i < feats_down_size; i++) {
                if (point_selected_surf[i]) {
                    laserCloudOri->points[effect_feat_num] = feats_down_body->points[i];
                    corr_normvect->points[effect_feat_num] = normvec->points[i];
                    effect_feat_num++;
                }
            }

            res_mean_last = total_residual / effect_feat_num;

            /*** Computation of Measurement Jacobian matrix H_ and residual Z_ (point-to-plane distance)***/
            MatrixXd H_(effect_feat_num, DIM_STATE);
            MatrixXd H_transpose_R_inv(DIM_STATE, effect_feat_num);
            VectorXd R_inv(effect_feat_num);
            VectorXd Z_(effect_feat_num);
            H_.setZero();
            H_transpose_R_inv.setZero();
            Z_.setZero();

            for (int i = 0; i < effect_feat_num; i++) {
                const PointType &laser_p = laserCloudOri->points[i];
                V3D point_this_L(laser_p.x, laser_p.y, laser_p.z);
                //Project points to body frame (IMU frame)
                V3D point_this = offset_R_L_I * point_this_L + offset_T_L_I;
                M3D var;
                calcBodyVar(point_this, 0.02, 0.05, var);
                var = state.rot_end * var * state.rot_end.transpose();
                M3D point_crossmat;
                point_crossmat << SKEW_SYM_MATRX(point_this);

                /*** get the normal vector of closest plane ***/
                const PointType &norm_p = corr_normvect->points[i];
                V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

                R_inv(i) = 1000;
                laserCloudOri->points[i].intensity = sqrt(R_inv(i));

                // Measurement Jacobian matrix H_ Computation
                M3D point_this_L_cross;
                point_this_L_cross << SKEW_SYM_MATRX(point_this_L);
                V3D A(point_crossmat * state.rot_end.transpose() * norm_vec);
                VD(6) A_;
                A_ << VEC_FROM_ARRAY(A), norm_p.x, norm_p.y, norm_p.z;
                H_.block<1,6>(i,0) = A_.transpose();
                H_transpose_R_inv.col(i) = H_.row(i).transpose() * 1000;


                // Residual: distance to the closest plane
                Z_(i) = - norm_p.intensity;
            }



            // Kalman Gain Computation
            MatrixXd K_(DIM_STATE, effect_feat_num);
            K_.setZero();

            // State Update
            state.rot_end = M3D::Identity();
            // Covariance Update and Reset
            MD(DIM_STATE, DIM_STATE) Jk;
            state.cov = state.cov;

            //Please note that boxplus and boxminus opraters are overloaded in include/common_lib.h,
            //So you can directly add delta_x (which is a vector) to a state, such as: state += delta_x.
            //Please note in the third formula in Page54 of the guideline, (y_k - h(x)) represents the point-to-plane distance residual, where y_k is the measurement, h(x) is the measurement model of state.
            //Here, (y_k - h(x)) is given,, which is actually the Z_ defined in line 1001.




          
            total_distance += (state.pos_end - position_last).norm();
            position_last = state.pos_end;
            euler_cur = RotMtoEuler(state.rot_end);
            geoQuat = tf::createQuaternionMsgFromRollPitchYaw
                    (euler_cur(0), euler_cur(1), euler_cur(2));

            if(abs(lidar_end_time - last_lidar_time) < 1e-3){
                cout << setprecision(6) << BOLDWHITE << "  Total Distance: " << BOLDGREEN << total_distance << RESET << endl;
                cout << BOLDWHITE << "  Position: " << BOLDGREEN << state.pos_end.transpose() << RESET << endl;
                if (pcd_save_en && pcd_save_interval < 0)
                    pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            }

            /******* Publish odometry *******/
            publish_odometry(pubOdomAftMapped);

            /*** add the feature points to map kdtree ***/
            map_incremental();

            /******* Publish points *******/
            if (scan_pub_en || pcd_save_en) publish_frame_world(pubLaserCloudFullRes);
            if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFullRes_body);
            last_odom = state.pos_end;
            last_rot = state.rot_end;
            publish_effect_world(pubLaserCloudEffect);
            if (path_en) publish_path(pubPath);
            //publish_mavros(mavros_pose_publisher);
            frame_num++;
            if (lidar_type == SIM) {
                VD(4) lio_pose;
                lio_pose.block<3,1>(0,0) = state.pos_end;
                lio_pose(3) = lidar_end_time;
                LIO_pose.push_back(lio_pose);
            }
            fout_out << euler_cur.transpose() * 57.3 << " " << state.pos_end.transpose() << " " << state.vel_end.transpose() << " "  \
 << " " << state.bias_g.transpose() << " " << state.bias_a.transpose() << " " << state.gravity.transpose() << " "
                     << total_distance << endl;
            dump_lio_state_to_log(fp);
        }
        status = ros::ok();
        rate.sleep();
    }


    cout << "Exit the process." << endl;

    
#ifndef DEPLOY
    vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;
    FILE *fp2;
    string log_dir = root_dir + "/Log/fast_lio_time_log.csv";
    fp2 = fopen(log_dir.c_str(),"w");
    fprintf(fp2,"time_stamp, total time, scan point size, incremental time, search time, delete size, delete time, tree size st, tree size end, add point size, preprocess time\n");
    for (int i = 0;i<time_log_counter; i++){
        fprintf(fp2,"%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n",T1[i],s_plot[i],int(s_plot2[i]),s_plot3[i],s_plot4[i],int(s_plot5[i]),s_plot6[i],int(s_plot7[i]),int(s_plot8[i]), int(s_plot10[i]), s_plot11[i]);
        t.push_back(T1[i]);
        s_vec.push_back(s_plot9[i]);
        s_vec2.push_back(s_plot3[i] + s_plot6[i]);
        s_vec3.push_back(s_plot4[i]);
        s_vec5.push_back(s_plot[i]);
    }
    fclose(fp2);
    if (!t.empty())
    {
        plt::figure(1);
        plt::named_plot("incremental time",t,s_vec2);
        plt::named_plot("search_time",t,s_vec3);
        plt::named_plot("total time",t,s_vec5);
        plt::named_plot("average time",t,s_vec);
        plt::title("Time Consume");
        plt::legend();
        plt::grid(true);
        plt::show();
        plt::pause(5);
        plt::close();
    }
    cout << "no points saved" << endl;
#endif
    return 0;
}

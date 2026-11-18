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
// g1d: the ROS node (subscribers, publishers, rosparam, tf, PCD saving, Python plotting) is
// replaced by the fast_lio_api interface at the end of this file. The estimator code
// (map segmentation, data sync, residual model, iterated EKF, map increment) is unchanged,
// except for the optional moving-point labeller that keeps labelled points out of the map.
#include <algorithm>
#include <deque>
#include <unordered_set>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <so3_math.h>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include "fast_lio_compat.h"
#include "IMU_Processing.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <ikd-Tree/ikd_Tree.h>
#include "fast_lio_api.h"

#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define PUBFRAME_PERIOD     (20)
#define MAX_FEATS           (100000)  // g1d: size of the per-point work arrays below

/*** Time Log Variables ***/
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
double match_time = 0, solve_time = 0, solve_const_H_time = 0;
int    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
bool   runtime_pos_log = false, pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true, path_en = true;
/**************************/

float res_last[100000] = {0.0};
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;
double time_diff_lidar_to_imu = 0.0;

mutex mtx_buffer;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;
string map_file_path, lid_topic, imu_topic;

double res_mean_last = 0.05, total_residual = 0.0;
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
int    effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
int    iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_save_interval = -1, pcd_index = 0;
bool   point_selected_surf[100000] = {0};
bool   lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
bool   scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;
int lidar_type;

vector<vector<int>>  pointSearchInd_surf; 
vector<BoxPointType> cub_needrm;
vector<PointVector>  Nearest_Points; 
vector<double>       extrinT(3, 0.0);
vector<double>       extrinR(9, 0.0);
deque<double>                     time_buffer;
deque<PointCloudXYZI::Ptr>        lidar_buffer;
deque<sensor_msgs::Imu::ConstPtr> imu_buffer;

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

KD_TREE<PointType> ikdtree;

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D position_last(Zero3d);
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);

/*** EKF inputs and output ***/
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point;
vect3 pos_lid;

struct { double x{0}, y{0}, z{0}, w{1}; } geoQuat;  // g1d: was geometry_msgs::Quaternion

shared_ptr<ImuProcess> p_imu(new ImuProcess());

inline void dump_lio_state_to_log(FILE *fp)  
{
    V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                   // Angle
    fprintf(fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1), state_point.pos(2)); // Pos  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega  
    fprintf(fp, "%lf %lf %lf ", state_point.vel(0), state_point.vel(1), state_point.vel(2)); // Vel  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc  
    fprintf(fp, "%lf %lf %lf ", state_point.bg(0), state_point.bg(1), state_point.bg(2));    // Bias_g  
    fprintf(fp, "%lf %lf %lf ", state_point.ba(0), state_point.ba(1), state_point.ba(2));    // Bias_a  
    fprintf(fp, "%lf %lf %lf ", state_point.grav[0], state_point.grav[1], state_point.grav[2]); // Bias_a  
    fprintf(fp, "\r\n");  
    fflush(fp);
}

void pointBodyToWorld_ikfom(PointType const * const pi, PointType * const po, state_ikfom &s)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}


void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

template<typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I*p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    // for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
}

BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;
void lasermap_fov_segment()
{
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;    
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    V3D pos_LiD = pos_lid;
    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    double delete_begin = omp_get_wtime();
    if(cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    kdtree_delete_time = omp_get_wtime() - delete_begin;
}

// g1d: standard_pcl_cbk / livox_pcl_cbk / imu_cbk are replaced by fast_lio_api::push_scan /
// push_imu below. LiDAR preprocessing (Livox tag/blind/filter) is done by the caller.

double lidar_mean_scantime = 0.0;
int    scan_num = 0;
bool sync_packages(MeasureGroup &meas)
{
    if (lidar_buffer.empty() || imu_buffer.empty()) {
        return false;
    }

    /*** push a lidar scan ***/
    if(!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();


        if (meas.lidar->points.size() <= 1) // time too little
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            ROS_WARN("Too few input point cloud!\n");
        }
        else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        }
        else
        {
            scan_num ++;
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
            lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        }
        if(lidar_type == MARSIM)
            lidar_end_time = meas.lidar_beg_time;

        meas.lidar_end_time = lidar_end_time;

        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time)
    {
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = imu_buffer.front()->header.stamp.toSec();
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = imu_buffer.front()->header.stamp.toSec();
        if(imu_time > lidar_end_time) break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

// g1d: world voxels (size g_dyn_voxel) holding points labelled moving in the current scan.
// With a labeller installed, map insertions are held for g_map_delay scans and dropped if any
// scan in that window labels a moving point next to them (catches parts missed in one scan).
std::unordered_set<int64_t> g_dyn_voxels;
float g_dyn_voxel = 0.2f;
int g_map_skipped = 0;
bool g_label_active = false;
int g_map_delay = 3;
struct PendingInsert
{
    PointVector add;
    PointVector no_downsample;
};
std::deque<PendingInsert> g_pending;
std::deque<std::unordered_set<int64_t>> g_dyn_history;

int64_t dyn_voxel_key(int x, int y, int z)
{
    return (static_cast<int64_t>(x) & 0x1FFFFF) << 42 | (static_cast<int64_t>(y) & 0x1FFFFF) << 21 |
           (static_cast<int64_t>(z) & 0x1FFFFF);
}

bool near_dynamic(const PointType & p, const std::unordered_set<int64_t> & voxels)
{
    if (voxels.empty()) return false;
    const int x = static_cast<int>(floor(p.x / g_dyn_voxel));
    const int y = static_cast<int>(floor(p.y / g_dyn_voxel));
    const int z = static_cast<int>(floor(p.z / g_dyn_voxel));
    for (int dx = -1; dx <= 1; ++dx)
        for (int dy = -1; dy <= 1; ++dy)
            for (int dz = -1; dz <= 1; ++dz)
                if (voxels.count(dyn_voxel_key(x + dx, y + dy, z + dz))) return true;
    return false;
}

void drop_near_dynamic(PointVector & pts, const std::unordered_set<int64_t> & voxels)
{
    if (voxels.empty()) return;
    const size_t before = pts.size();
    pts.erase(std::remove_if(pts.begin(), pts.end(),
        [&](const PointType & p) {return near_dynamic(p, voxels);}), pts.end());
    g_map_skipped += static_cast<int>(before - pts.size());
}

int process_increments = 0;
void map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    g_map_skipped = 0;
    for (int i = 0; i < feats_down_size; i++)
    {
        /* transform to world frame */
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        if (near_dynamic(feats_down_world->points[i], g_dyn_voxels))
        {
            ++g_map_skipped;
            continue;
        }
        /* decide if need add to map */
        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point; 
            mid_point.x = floor(feats_down_world->points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            float dist  = calc_dist(feats_down_world->points[i],mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min){
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    double st_time = omp_get_wtime();
    if (g_label_active && g_map_delay > 0)
    {
        g_pending.push_back({std::move(PointToAdd), std::move(PointNoNeedDownsample)});
        g_dyn_history.push_back(g_dyn_voxels);
        while (static_cast<int>(g_dyn_history.size()) > g_map_delay + 1) g_dyn_history.pop_front();
        PointToAdd.clear();
        PointNoNeedDownsample.clear();
        while (static_cast<int>(g_pending.size()) > g_map_delay)
        {
            PendingInsert & batch = g_pending.front();
            for (const auto & voxels : g_dyn_history)
            {
                drop_near_dynamic(batch.add, voxels);
                drop_near_dynamic(batch.no_downsample, voxels);
            }
            PointToAdd.insert(PointToAdd.end(), batch.add.begin(), batch.add.end());
            PointNoNeedDownsample.insert(PointNoNeedDownsample.end(),
                batch.no_downsample.begin(), batch.no_downsample.end());
            g_pending.pop_front();
        }
    }
    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false); 
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
    kdtree_incremental_time = omp_get_wtime() - st_time;
}

// g1d: publish_frame_world / publish_frame_body / publish_effect_world / publish_map /
// publish_odometry / publish_path (ROS publishers, tf, PCD saving) are removed; results are
// returned by fast_lio_api::process().

void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    double match_start = omp_get_wtime();
    laserCloudOri->clear(); 
    corr_normvect->clear(); 
    total_residual = 0.0; 

    /** closest surface search and residual computation **/
    #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
    #endif
    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body  = feats_down_body->points[i]; 
        PointType &point_world = feats_down_world->points[i]; 

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

        auto &points_near = Nearest_Points[i];

        if (ekfom_data.converge)
        {
            /** Find the closest surfaces in the map **/
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
        }

        if (!point_selected_surf[i]) continue;

        VF(4) pabcd;
        point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f))
        {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            if (s > 0.9)
            {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                res_last[i] = abs(pd2);
            }
        }
    }
    
    effct_feat_num = 0;

    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            total_residual += res_last[i];
            effct_feat_num ++;
        }
    }

    if (effct_feat_num < 1)
    {
        ekfom_data.valid = false;
        ROS_WARN("No Effective Points! \n");
        return;
    }

    res_mean_last = total_residual / effct_feat_num;
    match_time  += omp_get_wtime() - match_start;
    double solve_start_  = omp_get_wtime();
    
    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); //23
    ekfom_data.h.resize(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p  = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat<<SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() *norm_vec);
        V3D A(point_crossmat * C);
        if (extrinsic_est_en)
        {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); //s.rot.conjugate()*norm_vec);
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;
    }
    solve_time += omp_get_wtime() - solve_start_;
}

// ============================================================================================
// g1d: embeddable API (replaces int main()). Initialisation mirrors the upstream main(); the
// body of process() is the upstream main loop body with ROS publishing replaced by `Result`.
// ============================================================================================
namespace fast_lio_api
{
namespace
{
std::mutex g_api_mutex;
Params g_params;
PointLabeler g_labeler;
std::vector<uint8_t> g_dynamic;

void apply_params_locked(const Params & p)
{
    gyr_cov = p.gyr_cov;
    acc_cov = p.acc_cov;
    b_gyr_cov = p.b_gyr_cov;
    b_acc_cov = p.b_acc_cov;
    filter_size_surf_min = p.filter_size_surf;
    filter_size_map_min = p.filter_size_map;
    cube_len = p.cube_side_length;
    DET_RANGE = p.det_range;
    fov_deg = p.fov_degree;
    NUM_MAX_ITERATIONS = p.max_iteration;
    extrinsic_est_en = p.extrinsic_est_en;
    extrinT = p.extrinsic_T;
    extrinR = p.extrinsic_R;
    time_diff_lidar_to_imu = p.time_offset_lidar_to_imu;
    lidar_type = AVIA;
    if (extrinT.size() != 3 || extrinR.size() != 9) {
        throw std::invalid_argument("fast_lio: extrinsic_T needs 3 and extrinsic_R needs 9 values");
    }

    FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
    HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

    _featsArray.reset(new PointCloudXYZI());

    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);

    Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
    p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
    p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
    p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));
    p_imu->lidar_type = lidar_type;
    double epsi[23] = {0.001};
    fill(epsi, epsi+23, 0.001);
    kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);
}

void reset_locked()
{
    {
        std::lock_guard<std::mutex> lk(mtx_buffer);
        lidar_buffer.clear();
        time_buffer.clear();
        imu_buffer.clear();
        last_timestamp_lidar = 0;
        last_timestamp_imu = -1.0;
    }
    lidar_pushed = false;
    flg_first_scan = true;
    flg_EKF_inited = false;
    first_lidar_time = 0.0;
    lidar_end_time = 0.0;
    lidar_mean_scantime = 0.0;
    scan_num = 0;
    Localmap_Initialized = false;
    effct_feat_num = 0;
    res_mean_last = 0.05;

    ikdtree.~KD_TREE<PointType>();
    new (&ikdtree) KD_TREE<PointType>();
    p_imu.reset(new ImuProcess());
    kf = esekfom::esekf<state_ikfom, 12, input_ikfom>();
    state_point = state_ikfom();
    Measures = MeasureGroup();
    feats_undistort.reset(new PointCloudXYZI());
    feats_down_body.reset(new PointCloudXYZI());
    feats_down_world.reset(new PointCloudXYZI());
    normvec.reset(new PointCloudXYZI(MAX_FEATS, 1));
    laserCloudOri.reset(new PointCloudXYZI(MAX_FEATS, 1));
    corr_normvect.reset(new PointCloudXYZI(MAX_FEATS, 1));
    Nearest_Points.clear();
    pointSearchInd_surf.clear();
    g_dyn_voxels.clear();
    g_dynamic.clear();
    g_pending.clear();
    g_dyn_history.clear();

    apply_params_locked(g_params);
}
}  // namespace

void set_point_labeler(PointLabeler labeler, double exclusion_voxel, int map_delay_scans)
{
    std::lock_guard<std::mutex> api(g_api_mutex);
    g_labeler = std::move(labeler);
    g_label_active = static_cast<bool>(g_labeler);
    g_dyn_voxel = static_cast<float>(std::max(0.05, exclusion_voxel));
    g_map_delay = std::max(0, map_delay_scans);
    g_dyn_voxels.clear();
    g_dyn_history.clear();
    for (auto & batch : g_pending)
    {
        ikdtree.Add_Points(batch.add, true);
        ikdtree.Add_Points(batch.no_downsample, false);
    }
    g_pending.clear();
}

int delete_boxes(const std::vector<Box> & boxes)
{
    std::lock_guard<std::mutex> api(g_api_mutex);
    if (ikdtree.Root_Node == nullptr || boxes.empty()) return 0;
    vector<BoxPointType> b(boxes.size());
    for (size_t i = 0; i < boxes.size(); ++i)
    {
        for (int k = 0; k < 3; ++k)
        {
            b[i].vertex_min[k] = boxes[i][2 * k];
            b[i].vertex_max[k] = boxes[i][2 * k + 1];
        }
    }
    return ikdtree.Delete_Point_Boxes(b);
}

void map_points(Cloud & out)
{
    std::lock_guard<std::mutex> api(g_api_mutex);
    out.clear();
    if (ikdtree.Root_Node == nullptr) return;
    PointVector storage;
    ikdtree.flatten(ikdtree.Root_Node, storage, NOT_RECORD);
    out.reserve(storage.size());
    for (const auto & p : storage) out.push_back(p);
}

void init(const Params & params)
{
    std::lock_guard<std::mutex> api(g_api_mutex);
    g_params = params;
    reset_locked();
}

void reset()
{
    std::lock_guard<std::mutex> api(g_api_mutex);
    reset_locked();
}

bool initialized()
{
    std::lock_guard<std::mutex> api(g_api_mutex);
    return !flg_first_scan && ikdtree.Root_Node != nullptr;
}

void push_imu(const ImuSample & s)
{
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu());
    msg->header.stamp.sec = s.t - time_diff_lidar_to_imu;
    msg->angular_velocity.x = s.gyr[0];
    msg->angular_velocity.y = s.gyr[1];
    msg->angular_velocity.z = s.gyr[2];
    msg->linear_acceleration.x = s.acc[0];
    msg->linear_acceleration.y = s.acc[1];
    msg->linear_acceleration.z = s.acc[2];
    const double timestamp = msg->header.stamp.toSec();

    std::lock_guard<std::mutex> lk(mtx_buffer);
    if (timestamp < last_timestamp_imu)
    {
        ROS_WARN("imu loop back, clear buffer");
        imu_buffer.clear();
    }
    last_timestamp_imu = timestamp;
    imu_buffer.push_back(msg);
}

void push_scan(double t_begin, const Cloud::Ptr & cloud)
{
    PointCloudXYZI::Ptr ptr(new PointCloudXYZI(*cloud));
    std::lock_guard<std::mutex> lk(mtx_buffer);
    if (t_begin < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
        time_buffer.clear();
    }
    last_timestamp_lidar = t_begin;
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(t_begin);
}

bool process(Result * out)
{
    std::lock_guard<std::mutex> api(g_api_mutex);
    {
        std::lock_guard<std::mutex> lk(mtx_buffer);
        if (!sync_packages(Measures)) {
            return false;
        }
    }

    if (flg_first_scan)
    {
        first_lidar_time = Measures.lidar_beg_time;
        p_imu->first_lidar_time = first_lidar_time;
        flg_first_scan = false;
        return false;
    }

    match_time = 0;
    kdtree_search_time = 0.0;
    solve_time = 0;
    solve_const_H_time = 0;
    const double t0 = omp_get_wtime();

    p_imu->Process(Measures, kf, feats_undistort);
    state_point = kf.get_x();
    pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

    if (feats_undistort->empty() || (feats_undistort == NULL))
    {
        return false;  // IMU initialisation in progress or empty scan
    }

    flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? \
                    false : true;
    /*** Segment the map in lidar FOV ***/
    lasermap_fov_segment();

    /*** downsample the feature points in a scan ***/
    downSizeFilterSurf.setInputCloud(feats_undistort);
    downSizeFilterSurf.filter(*feats_down_body);
    feats_down_size = feats_down_body->points.size();
    if (feats_down_size > MAX_FEATS)
    {
        feats_down_body->resize(MAX_FEATS);
        feats_down_size = MAX_FEATS;
    }
    /*** initialize the map kdtree ***/
    if(ikdtree.Root_Node == nullptr)
    {
        if(feats_down_size > 5)
        {
            ikdtree.set_downsample_param(filter_size_map_min);
            feats_down_world->resize(feats_down_size);
            for(int i = 0; i < feats_down_size; i++)
            {
                pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
            }
            ikdtree.Build(feats_down_world->points);
        }
        return false;
    }

    /*** ICP and iterated Kalman filter update ***/
    if (feats_down_size < 5)
    {
        ROS_WARN("No point, skip this scan!\n");
        return false;
    }

    normvec->resize(feats_down_size);
    feats_down_world->resize(feats_down_size);
    pointSearchInd_surf.resize(feats_down_size);
    Nearest_Points.resize(feats_down_size);

    /*** iterated state estimation ***/
    double solve_H_time = 0;
    kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
    state_point = kf.get_x();
    euler_cur = SO3ToEuler(state_point.rot);
    pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
    geoQuat.x = state_point.rot.coeffs()[0];
    geoQuat.y = state_point.rot.coeffs()[1];
    geoQuat.z = state_point.rot.coeffs()[2];
    geoQuat.w = state_point.rot.coeffs()[3];

    /*** g1d: label moving points; they (and their neighbourhood) stay out of the map ***/
    g_dyn_voxels.clear();
    g_dynamic.clear();
    double label_time = 0.0;
    int dynamic_points = 0;
    if (g_labeler && flg_EKF_inited)
    {
        const double tl = omp_get_wtime();
        const M3D R_wl = state_point.rot.toRotationMatrix() * state_point.offset_R_L_I.toRotationMatrix();
        const V3D t_wl = pos_lid;
        g_dynamic.assign(feats_undistort->points.size(), 0);
        g_labeler(*feats_undistort, R_wl, t_wl, lidar_end_time, g_dynamic);
        for (size_t i = 0; i < g_dynamic.size(); ++i)
        {
            if (!g_dynamic[i]) continue;
            ++dynamic_points;
            const auto & q = feats_undistort->points[i];
            const V3D w = R_wl * V3D(q.x, q.y, q.z) + t_wl;
            g_dyn_voxels.insert(dyn_voxel_key(static_cast<int>(floor(w.x() / g_dyn_voxel)),
                static_cast<int>(floor(w.y() / g_dyn_voxel)), static_cast<int>(floor(w.z() / g_dyn_voxel))));
        }
        label_time = omp_get_wtime() - tl;
    }

    /*** add the feature points to map kdtree ***/
    map_incremental();

    if (out)
    {
        out->t_end = lidar_end_time;
        out->pos = Eigen::Vector3d(state_point.pos(0), state_point.pos(1), state_point.pos(2));
        out->rot = Eigen::Quaterniond(geoQuat.w, geoQuat.x, geoQuat.y, geoQuat.z);
        out->vel = Eigen::Vector3d(state_point.vel(0), state_point.vel(1), state_point.vel(2));
        out->bg = Eigen::Vector3d(state_point.bg(0), state_point.bg(1), state_point.bg(2));
        out->ba = Eigen::Vector3d(state_point.ba(0), state_point.ba(1), state_point.ba(2));
        out->grav = Eigen::Vector3d(state_point.grav[0], state_point.grav[1], state_point.grav[2]);
        out->pose_cov = kf.get_P().block<6, 6>(0, 0);
        out->input_points = static_cast<int>(feats_undistort->points.size());
        out->downsampled_points = feats_down_size;
        out->effective_points = effct_feat_num;
        out->residual_mean = res_mean_last;
        out->ekf_inited = flg_EKF_inited;

        Eigen::Matrix3d nn = Eigen::Matrix3d::Zero();
        for (int i = 0; i < effct_feat_num; i++)
        {
            const PointType &n = corr_normvect->points[i];
            const Eigen::Vector3d v(n.x, n.y, n.z);
            nn += v * v.transpose();
        }
        if (effct_feat_num > 0)
        {
            nn /= static_cast<double>(effct_feat_num);
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(nn);
            out->normal_eig_min = es.eigenvalues()(0);
        }
        else
        {
            out->normal_eig_min = 0.0;
        }

        const int size = feats_undistort->points.size();
        out->cloud_world.reset(new PointCloudXYZI(size, 1));
        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&feats_undistort->points[i], &out->cloud_world->points[i]);
        }
        out->dynamic.swap(g_dynamic);
        out->dynamic_points = dynamic_points;
        out->map_skipped_points = g_map_skipped;
        out->label_time_s = label_time;
        out->process_time_s = omp_get_wtime() - t0;
    }
    return true;
}

}  // namespace fast_lio_api

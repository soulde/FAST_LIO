//
// Created by soulde on 2023/7/7.
//

#ifndef FAST_LIO_FAST_LIO_H
#define FAST_LIO_FAST_LIO_H

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl/filters/voxel_grid.h>
#include <builtin_interfaces/msg/time.hpp>

#include <ikd-Tree/ikd_Tree.h>

#include "IMU_Processing.hpp"
#include "Preprocess.h"


#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)


class Fast_Lio : public rclcpp::Node {
public:
    Fast_Lio();

    ~Fast_Lio();

    static void SigHandle(int sig) {
        flag_exit = true;
        std::cout<<"catch"<<std::endl;
        sig_buffer.notify_all();
    }

private:
/*** Time Log Variables ***/
    double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
    double T1[MAXN]{}, s_plot[MAXN]{}, s_plot2[MAXN]{}, s_plot3[MAXN]{}, s_plot4[MAXN]{}, s_plot5[MAXN]{}, s_plot6[MAXN]{}, s_plot7[MAXN]{}, s_plot8[MAXN]{}, s_plot9[MAXN]{}, s_plot10[MAXN]{}, s_plot11[MAXN]{};
    double match_time = 0, solve_time = 0, solve_const_H_time = 0;
    int kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
    bool runtime_pos_log = false, pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true, path_en = true;

    int effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;


/**************************/

    float res_last[100000] = {0.0};
    float DET_RANGE = 300.0f;
    const float MOV_THRESHOLD = 1.5f;
    double time_diff_lidar_to_imu = 0.0;

    mutex mtx_buffer;
    static condition_variable sig_buffer;

    //TODO: fix this later
    string root_dir = "/home/soulde/lio_ws/src/fast_lio";
    string map_file_path, lid_topic, imu_topic;
    double res_mean_last = 0.05, total_residual = 0.0;
    double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
    double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
    double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
    double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
    int effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
    int iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_save_interval = -1, pcd_index = 0;
    bool point_selected_surf[100000] = {0};
    bool lidar_pushed{}, flg_first_scan = true, flg_EKF_inited{};
    bool scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;
    static bool flag_exit;
    vector<vector<int>> pointSearchInd_surf;
    vector<BoxPointType> cub_needrm;
    vector<PointVector> Nearest_Points;
    vector<double> extrinT;
    vector<double> extrinR;
    deque<double> time_buffer;
    deque<PointCloudXYZI::Ptr> lidar_buffer;
    deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer;

    PointCloudXYZI::Ptr featsFromMap, feats_undistort, feats_down_body, feats_down_world, normvec, laserCloudOri, corr_normvect, _featsArray;

    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterMap;

    KD_TREE<PointType> ikdtree;

    V3F XAxisPoint_body;
    V3F XAxisPoint_world;
    V3D euler_cur;

    V3D position_last;

    V3D Lidar_T_wrt_IMU;

    M3D Lidar_R_wrt_IMU;

    shared_ptr<Preprocess> p_pre;
    shared_ptr<ImuProcess> p_imu;

    double timediff_lidar_wrt_imu = 0.0;
    bool timediff_set_flg = false;

    BoxPointType LocalMap_Points{};
    bool Localmap_Initialized = false;

    /*** EKF inputs and output ***/
    MeasureGroup Measures;
    esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
    state_ikfom state_point;
    vect3 pos_lid;

    FILE *fp;
    ofstream fout_pre, fout_out, fout_dbg;
    string pos_log_dir = root_dir + "/Log/pos_log.txt";

    nav_msgs::msg::Path path;
    nav_msgs::msg::Odometry odomAftMapped;
    geometry_msgs::msg::Quaternion geoQuat;
    geometry_msgs::msg::PoseStamped msg_body_pose;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pc;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pc, pub_pc_body, pub_pc_effected, pub_map;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;

    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster;

    rclcpp::Rate rate;

    std::shared_ptr<std::thread> thread;

    void pc2Callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg);

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

    void fusionThread();

    void lasermap_fov_segment();

    template<typename T>
    void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po) {
        V3D p_body(pi[0], pi[1], pi[2]);
        V3D p_global(
                state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);

        po[0] = p_global(0);
        po[1] = p_global(1);
        po[2] = p_global(2);
    }

    void RGBpointBodyToWorld(PointType const *pi, PointType *const po);

    void pointBodyToWorld(PointType const *const pi, PointType *const po);

    void RGBpointBodyLidarToIMU(PointType const *const pi, PointType *const po);

    void points_cache_collect();

    double lidar_mean_scantime = 0.0;
    int scan_num = 0;

    bool sync_packages(MeasureGroup &meas);

    int process_increments = 0;

    void map_incremental();

    PointCloudXYZI::Ptr pcl_wait_pub;

    PointCloudXYZI::Ptr pcl_wait_save;

    void publish_frame_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudFull);

    void publish_frame_body(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudFull_body);

    void publish_effect_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudEffect);

    void publish_map(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudMap);

    template<typename T>
    void set_posestamp(T &out) {
        out.pose.position.x = state_point.pos(0);
        out.pose.position.y = state_point.pos(1);
        out.pose.position.z = state_point.pos(2);
        out.pose.orientation.x = geoQuat.x;
        out.pose.orientation.y = geoQuat.y;
        out.pose.orientation.z = geoQuat.z;
        out.pose.orientation.w = geoQuat.w;

    }

    void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr &pubOdomAftMapped);

    void publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &pubPath);

    inline void dump_lio_state_to_log(FILE *fp) ;

    void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data);
};


#endif //FAST_LIO_FAST_LIO_H

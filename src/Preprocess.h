//
// Created by soulde on 2023/7/7.
//

#ifndef FAST_LIO_PREPROCESS_H
#define FAST_LIO_PREPROCESS_H

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <omp.h>

#define IS_VALID(a)  ((abs(a)>1e8) ? true : false)

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

enum Lidar_Type {
    MID360 = 1
};
enum TIME_UNIT {
    SEC = 0, MS = 1, US = 2, NS = 3
};
enum Feature {
    Nor, Poss_Plane, Real_Plane, Edge_Jump, Edge_Plane, Wire, ZeroPoint
};
enum Surround {
    Prev, Next
};
enum E_jump {
    Nr_nor, Nr_zero, Nr_180, Nr_inf, Nr_blind
};

struct orgtype {
    double range;
    double dista;
    double angle[2];
    double intersect;
    E_jump edj[2];
    Feature ftype;

    orgtype() {
        range = 0;
        edj[Prev] = Nr_nor;
        edj[Next] = Nr_nor;
        ftype = Nor;
        intersect = 2;
    }
};

class Preprocess {
public:
    Preprocess();

    ~Preprocess();

    void process(const livox_ros_driver2::msg::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);

    void set(bool feat_en, int lid_type, double bld, int pfilt_num);

    // sensor_msgs::PointCloud2::ConstPtr pointcloud;
    PointCloudXYZI pl_full, pl_corn, pl_surf;
    PointCloudXYZI pl_buff[128]; //maximum 128 line lidar
    std::vector<orgtype> typess[128]; //maximum 128 line lidar
    float time_unit_scale;
    int lidar_type, point_filter_num, N_SCANS, SCAN_RATE, time_unit;
    double blind;
    bool feature_enabled, given_offset_time;
//    ros::Publisher pub_full, pub_surf, pub_corn;


private:
    void mid360_handler(const livox_ros_driver2::msg::CustomMsg::ConstPtr &msg);

    void give_feature(PointCloudXYZI &pl, std::vector<orgtype> &types);

    void pub_func(PointCloudXYZI &pl, const rclcpp::Time &ct);

    int plane_judge(const PointCloudXYZI &pl, std::vector<orgtype> &types, uint i, uint &i_nex,
                    Eigen::Vector3d &curr_direct);

    bool small_plane(const PointCloudXYZI &pl, std::vector<orgtype> &types, uint i_cur, uint &i_nex,
                     Eigen::Vector3d &curr_direct);

    bool edge_jump_judge(const PointCloudXYZI &pl, std::vector<orgtype> &types, uint i, Surround nor_dir);

    int group_size;
    double disA, disB, inf_bound;
    double limit_maxmid, limit_midmin, limit_maxmin;
    double p2l_ratio;
    double jump_up_limit, jump_down_limit;
    double cos160;
    double edgea, edgeb;
    double smallp_intersect, smallp_ratio;
    double vx, vy, vz;
};


#endif //FAST_LIO_PREPROCESS_H

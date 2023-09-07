//
// Created by soulde on 2023/7/7.
//

#include "Preprocess.h"


#define RETURN0     0x00
#define RETURN0AND1 0x10

Preprocess::Preprocess()
        : lidar_type(MID360), blind(0.01), point_filter_num(1) {
    inf_bound = 10;
    N_SCANS = 6;
    SCAN_RATE = 10;
    group_size = 8;
    disA = 0.01;
    disA = 0.1; // B?
    p2l_ratio = 225;
    limit_maxmid = 6.25;
    limit_midmin = 6.25;
    limit_maxmin = 3.24;
    jump_up_limit = 170.0;
    jump_down_limit = 8.0;
    cos160 = 160.0;
    edgea = 2;
    edgeb = 0.1;
    smallp_intersect = 172.5;
    smallp_ratio = 1.2;
    given_offset_time = false;

    jump_up_limit = cos(jump_up_limit / 180 * M_PI);
    jump_down_limit = cos(jump_down_limit / 180 * M_PI);
    cos160 = cos(cos160 / 180 * M_PI);
    smallp_intersect = cos(smallp_intersect / 180 * M_PI);
}

Preprocess::~Preprocess() = default;

void Preprocess::set(int lid_type, double bld, int pfilt_num) {
    lidar_type = lid_type;
    blind = bld;
    point_filter_num = pfilt_num;
}

void Preprocess::process(const livox_ros_driver2::msg::CustomMsg&msg, PointCloudXYZI::Ptr &pcl_out) {
    livoxHandler(msg);
    *pcl_out = pl_out;
}


void Preprocess::livoxHandler(const livox_ros_driver2::msg::CustomMsg &msg) {
    pl_out.clear();
    pl_full.clear();

    uint plsize = msg.point_num;

    pl_out.reserve(plsize);
    pl_full.resize(plsize);

    for (int i = 0; i < N_SCANS; i++) {
        pl_buff[i].clear();
        pl_buff[i].reserve(plsize);
    }
    uint valid_num = 0;

    for (uint i = 1; i < plsize; i++) {
        if ((msg.points[i].line < N_SCANS) &&
            ((msg.points[i].tag & 0x30) == 0x10 || (msg.points[i].tag & 0x30) == 0x00)) {
            valid_num++;
            if (valid_num % point_filter_num == 0) {
                pl_full[i].x = msg.points[i].x;
                pl_full[i].y = msg.points[i].y;
                pl_full[i].z = msg.points[i].z;
                pl_full[i].intensity = msg.points[i].reflectivity;
                pl_full[i].curvature = msg.points[i].offset_time /
                                       float(1000000); // use curvature as time of each laser points, curvature unit: ms

                if (((abs(pl_full[i].x - pl_full[i - 1].x) > 1e-7)
                     || (abs(pl_full[i].y - pl_full[i - 1].y) > 1e-7)
                     || (abs(pl_full[i].z - pl_full[i - 1].z) > 1e-7))
                    && (pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y + pl_full[i].z * pl_full[i].z >
                        (blind * blind))) {
                    pl_out.push_back(pl_full[i]);
                }
            }
        }
    }

}
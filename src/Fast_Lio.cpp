//
// Created by soulde on 2023/7/7.
//

#include "Fast_Lio.h"

condition_variable Fast_Lio::sig_buffer{};
bool Fast_Lio::flag_exit = false;

Fast_Lio::Fast_Lio() : rclcpp::Node("fast_lio"), extrinT(3, 0.0), extrinR(9, 0.0), featsFromMap(new PointCloudXYZI),
                       feats_undistort(new PointCloudXYZI), feats_down_body(new PointCloudXYZI),
                       feats_down_world(new PointCloudXYZI), normvec(new PointCloudXYZI(100000, 1)),
                       laserCloudOri(new PointCloudXYZI(100000, 1)), corr_normvect(new PointCloudXYZI(100000, 1)),
                       p_pre(new Preprocess), p_imu(new ImuProcess),
                       rate(5000), 
                       pcl_wait_pub(new PointCloudXYZI(500000, 1)),
                       pcl_wait_save(new PointCloudXYZI), _featsArray(new PointCloudXYZI()),
                       XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0), XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0),position_last(Zero3d),
                       Lidar_R_wrt_IMU(Eye3d), Lidar_T_wrt_IMU(Zero3d) {
    path_en = this->declare_parameter<bool>("path_en", true);
    scan_pub_en = this->declare_parameter<bool>("scan_publish_en", true);
    dense_pub_en = this->declare_parameter<bool>("dense_publish_en", true);
    scan_body_pub_en = this->declare_parameter<bool>("scan_bodyframe_pub_en", true);
    NUM_MAX_ITERATIONS = this->declare_parameter<int>("max_iteration", 4);
    map_file_path = this->declare_parameter<std::string>("map_file_path", "");
    lid_topic = this->declare_parameter<std::string>("lid_topic", "/livox/lidar");
    imu_topic = this->declare_parameter<string>("imu_topic", "/livox/imu");
    time_sync_en = this->declare_parameter<bool>("time_sync_en", false);
    time_diff_lidar_to_imu = this->declare_parameter<double>("time_offset_lidar_to_imu", 0.0);
    filter_size_corner_min = this->declare_parameter<double>("filter_size_corner", 0.5);
    filter_size_surf_min = this->declare_parameter<double>("filter_size_surf", 0.5);
    filter_size_map_min = this->declare_parameter<double>("filter_size_map", 0.5);
    cube_len = this->declare_parameter<double>("cube_side_length", 200.);
    DET_RANGE = this->declare_parameter<float>("det_range", 300.f);
    fov_deg = this->declare_parameter<double>("fov_degree", 180.);
    gyr_cov = this->declare_parameter<double>("gyr_cov", 0.1);
    acc_cov = this->declare_parameter<double>("acc_cov", 0.1);
    b_gyr_cov = this->declare_parameter<double>("b_gyr_cov", 0.0001);
    b_acc_cov = this->declare_parameter<double>("b_acc_cov", 0.0001);
    p_pre->lidar_type = this->declare_parameter<int>("lidar_type", 1);
    p_pre->blind = this->declare_parameter<double>("blind", 0.01);
    p_pre->N_SCANS = this->declare_parameter<int>("scan_line", 16);
    p_pre->time_unit = this->declare_parameter<int>("timestamp_unit", US);
    p_pre->SCAN_RATE = this->declare_parameter<int>("scan_rate", 10);
    p_pre->point_filter_num = this->declare_parameter<int>("point_filter_num", 2);
    runtime_pos_log = this->declare_parameter<bool>("runtime_pos_log_enable", 0);
    extrinsic_est_en = this->declare_parameter<bool>("extrinsic_est_en", true);
    pcd_save_en = this->declare_parameter<bool>("pcd_save_en", false);
    pcd_save_interval = this->declare_parameter<int>("interval", -1);
    extrinT = this->declare_parameter<vector<double>>("extrinsic_T", vector<double>(3));
    extrinR = this->declare_parameter<vector<double>>("extrinsic_R", vector<double>(9));

    this->get_parameter("path_en", path_en);
    this->get_parameter("scan_publish_en", scan_pub_en);
    this->get_parameter("dense_publish_en", dense_pub_en);
    this->get_parameter("scan_bodyframe_pub_en", scan_body_pub_en);
    this->get_parameter("max_iteration", NUM_MAX_ITERATIONS);
    this->get_parameter("map_file_path", map_file_path);
    this->get_parameter("lid_topic", lid_topic);
    this->get_parameter("imu_topic", imu_topic);
    this->get_parameter("time_sync_en", time_sync_en);
    this->get_parameter("time_offset_lidar_to_imu", time_diff_lidar_to_imu);
    this->get_parameter("filter_size_corner", filter_size_corner_min);
    this->get_parameter("filter_size_surf", filter_size_surf_min);
    this->get_parameter("filter_size_map", filter_size_map_min);
    this->get_parameter("cube_side_length", cube_len);
    this->get_parameter("det_range", DET_RANGE);
    this->get_parameter("fov_degree", fov_deg);
    this->get_parameter("gyr_cov", gyr_cov);
    this->get_parameter("acc_cov", acc_cov);
    this->get_parameter("b_gyr_cov", b_gyr_cov);
    this->get_parameter("b_acc_cov", b_acc_cov);
    this->get_parameter("lidar_type", p_pre->lidar_type);
    this->get_parameter("blind", p_pre->blind);
    this->get_parameter("scan_line", p_pre->N_SCANS);
    this->get_parameter("timestamp_unit", p_pre->time_unit);
    this->get_parameter("scan_rate", p_pre->SCAN_RATE);
    this->get_parameter("point_filter_num", p_pre->point_filter_num);

    this->get_parameter("runtime_pos_log_enable", runtime_pos_log);
    this->get_parameter("extrinsic_est_en", extrinsic_est_en);
    this->get_parameter("pcd_save_en", pcd_save_en);
    this->get_parameter("interval", pcd_save_interval);
    this->get_parameter("extrinsic_T", extrinT);
    this->get_parameter("extrinsic_R", extrinR);


    sub_pc = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(lid_topic, 200000,
                                                                          std::bind(&Fast_Lio::pc2Callback, this,
                                                                                    std::placeholders::_1));

    sub_imu = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 200000,
                                                               std::bind(&Fast_Lio::imuCallback, this,
                                                                         std::placeholders::_1));

    pub_pc = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 100000);
    pub_pc_body = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_body", 100000);
    pub_pc_effected = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected", 100000);
    pub_map = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", 100000);
    pub_odom = this->create_publisher<nav_msgs::msg::Odometry>("/Odometry", 100000);
    broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    pub_path = this->create_publisher<nav_msgs::msg::Path>("/path", 100000);
    fp = fopen(pos_log_dir.c_str(), "w");

    FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
    HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

    _featsArray.reset(new PointCloudXYZI());

    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));

    Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);

    Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);

    p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
    p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
    p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

    double epsi[23] = {0.001};
    fill(epsi, epsi + 23, 0.001);

    kf.init_dyn_share(get_f, df_dx, df_dw,
                      std::bind(&Fast_Lio::h_share_model, this, std::placeholders::_1, std::placeholders::_2),
                      NUM_MAX_ITERATIONS, epsi);
    thread.reset(new std::thread(&Fast_Lio::fusionThread, this));

    fout_pre.open(root_dir + "/LOG/" + std::string("mat_pre.txt"), ios::out);
    fout_out.open(root_dir + "/LOG/" + std::string("mat_out.txt"), ios::out);
    fout_dbg.open(root_dir + "/LOG/" + std::string("dbg.txt"), ios::out);
    if (fout_pre && fout_out)
        cout << "~~~~" << root_dir << " file opened" << endl;
    else
        cout << "~~~~" << root_dir << " doesn't exist" << endl;
}

void Fast_Lio::pc2Callback(const livox_ros_driver2::msg::CustomMsg& msg) {
    mtx_buffer.lock();
    double preprocess_start_time = omp_get_wtime();
    scan_count++;
    if (rclcpp::Time(msg.header.stamp).seconds() < last_timestamp_lidar) {
        RCLCPP_ERROR(get_logger(), "lidar loop back, clear buffer");
        lidar_buffer.clear();
    }
    last_timestamp_lidar = rclcpp::Time(msg.header.stamp).seconds();

    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() &&
        !lidar_buffer.empty()) {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n", last_timestamp_imu,
               last_timestamp_lidar);
    }

    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 &&
        !imu_buffer.empty()) {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp_lidar);

    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void Fast_Lio::imuCallback(const sensor_msgs::msg::Imu& msg_in) {
    publish_count++;

    sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(msg_in));
    msg->header.stamp = rclcpp::Time((rclcpp::Time(msg_in.header.stamp).seconds() - time_diff_lidar_to_imu) * 1e9);
    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en) {
        msg->header.stamp = rclcpp::Time((timediff_lidar_wrt_imu + rclcpp::Time(msg_in.header.stamp).seconds())*1e9);
    }

    double timestamp = rclcpp::Time(msg->header.stamp).seconds();

    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu) {
        RCLCPP_WARN(get_logger(), "imu loop back, clear buffer");
        imu_buffer.clear();
    }

    last_timestamp_imu = timestamp;

    imu_buffer.push_back(msg);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void Fast_Lio::fusionThread() {
    while (true) {
        if (flag_exit) break;
        if (sync_packages(Measures)) {
            if (flg_first_scan) {
                first_lidar_time = Measures.lidar_beg_time;
                p_imu->first_lidar_time = first_lidar_time;
                flg_first_scan = false;
                continue;
            }

            double t0, t1, t2, t3, t4, t5, match_start, solve_start, svd_time;

            match_time = 0;
            kdtree_search_time = 0.0;
            solve_time = 0;
            solve_const_H_time = 0;
            svd_time = 0;
            t0 = omp_get_wtime();

            p_imu->Process(Measures, kf, feats_undistort);

            state_point = kf.get_x();
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

            if (feats_undistort->empty() || (feats_undistort == nullptr)) {
                RCLCPP_INFO(this->get_logger(), "No point, skip this scan!\n");
                continue;
            }

            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) >= INIT_TIME;
            /*** Segment the map in lidar FOV ***/
            lasermap_fov_segment();

            /*** downsample the feature points in a scan ***/
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);

            t1 = omp_get_wtime();
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

            cout << "[ mapping ]: In num: " << feats_undistort->points.size() << " downsamp " << feats_down_size
                 << " Map num: " << featsFromMapNum << "effect num:" << effct_feat_num << endl;

            /*** ICP and iterated Kalman filter update ***/
            if (feats_down_size < 5) {
                RCLCPP_INFO(this->get_logger(), "No point, skip this scan!\n");
                continue;
            }

            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);
            fout_pre << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " "
                     << state_point.pos.transpose() << " " << ext_euler.transpose() << " "
                     << state_point.offset_T_L_I.transpose() << " " << state_point.vel.transpose() \
 << " " << state_point.bg.transpose() << " " << state_point.ba.transpose() << " " << state_point.grav << endl;

            if (0) // If you need to see map point, change to "if(1)"
            {
                PointVector().swap(ikdtree.PCL_Storage);
                ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
                featsFromMap->clear();
                featsFromMap->points = ikdtree.PCL_Storage;
            }

            pointSearchInd_surf.resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);
            int rematch_num = 0;
            bool nearest_search_en = true; //

            t2 = omp_get_wtime();

            /*** iterated state estimation ***/
            double t_update_start = omp_get_wtime();
            double solve_H_time = 0;

            kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);

            state_point = kf.get_x();
            euler_cur = SO3ToEuler(state_point.rot);
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
            geoQuat.x = state_point.rot.coeffs()[0];
            geoQuat.y = state_point.rot.coeffs()[1];
            geoQuat.z = state_point.rot.coeffs()[2];
            geoQuat.w = state_point.rot.coeffs()[3];

            double t_update_end = omp_get_wtime();

            /******* Publish odometry *******/
            publish_odometry(pub_odom);

            /*** add the feature points to map kdtree ***/
            t3 = omp_get_wtime();
            std::cout << "start update map" << std::endl;
            map_incremental();
            t5 = omp_get_wtime();

            /******* Publish points *******/
            if (path_en) publish_path(pub_path);
            if (scan_pub_en || pcd_save_en) publish_frame_world(pub_pc);
            if (scan_pub_en && scan_body_pub_en) publish_frame_body(pub_pc_body);
            // publish_effect_world(pubLaserCloudEffect);
            // publish_map(pubLaserCloudMap);
            std::cout << "start publish debug" << std::endl;
            /*** Debug variables ***/
            if (runtime_pos_log) {
                frame_num++;
                kdtree_size_end = ikdtree.size();

                aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
                aver_time_icp =
                        aver_time_icp * (frame_num - 1) / frame_num + (t_update_end - t_update_start) / frame_num;
                aver_time_match = aver_time_match * (frame_num - 1) / frame_num + (match_time) / frame_num;
                aver_time_incre = aver_time_incre * (frame_num - 1) / frame_num + (kdtree_incremental_time) / frame_num;

                aver_time_solve =
                        aver_time_solve * (frame_num - 1) / frame_num + (solve_time + solve_H_time) / frame_num;
                aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1) / frame_num + solve_time / frame_num;
                T1[time_log_counter] = Measures.lidar_beg_time;

                s_plot[time_log_counter] = t5 - t0;
                s_plot2[time_log_counter] = feats_undistort->points.size();
                s_plot3[time_log_counter] = kdtree_incremental_time;
                s_plot4[time_log_counter] = kdtree_search_time;
                s_plot5[time_log_counter] = kdtree_delete_counter;
                s_plot6[time_log_counter] = kdtree_delete_time;

                s_plot7[time_log_counter] = kdtree_size_st;
                s_plot8[time_log_counter] = kdtree_size_end;

                s_plot9[time_log_counter] = aver_time_consu;

                s_plot10[time_log_counter] = add_point_size;

                time_log_counter++;

                RCLCPP_INFO(get_logger(),
                            "[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f construct H: %0.6f \n",
                            t1 - t0, aver_time_match, aver_time_solve, t3 - t1, t5 - t3, aver_time_consu, aver_time_icp,
                            aver_time_const_H_time);
                ext_euler = SO3ToEuler(state_point.offset_R_L_I);
                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose()
                         << " " << state_point.pos.transpose() << " " << ext_euler.transpose() << " "
                         << state_point.offset_T_L_I.transpose() << " " << state_point.vel.transpose() \
 << " " << state_point.bg.transpose() << " " << state_point.ba.transpose() << " " << state_point.grav << " "
                         << feats_undistort->points.size() << endl;

//                dump_lio_state_to_log(fp);

            }
        }
        rate.sleep();
    }

    /**************** save map ****************/
    /*
     * 1. make sure you have enough memories
     * 2. pcd save will largely influence the real-time performences
     */
    if (pcl_wait_save->size() > 0 && pcd_save_en) {
        string file_name = string("scans.pcd");
        string all_points_dir(std::string(std::string(root_dir) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        cout << "current scan saved to /PCD/" << file_name << endl;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }

    fout_out.close();
    fout_pre.close();

    if (runtime_pos_log) {
        vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;
        FILE *fp2;
        string log_dir = root_dir + "/Log/fast_lio_time_log.csv";
        fp2 = fopen(log_dir.c_str(), "w");
        fprintf(fp2,
                "time_stamp, total time, scan point size, incremental time, search time, delete size, delete time, tree size st, tree size end, add point size, preprocess time\n");
        for (int i = 0; i < time_log_counter; i++) {
            fprintf(fp2, "%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n", T1[i], s_plot[i], int(s_plot2[i]),
                    s_plot3[i], s_plot4[i], int(s_plot5[i]), s_plot6[i], int(s_plot7[i]), int(s_plot8[i]),
                    int(s_plot10[i]), s_plot11[i]);
            t.push_back(T1[i]);
            s_vec.push_back(s_plot9[i]);
            s_vec2.push_back(s_plot3[i] + s_plot6[i]);
            s_vec3.push_back(s_plot4[i]);
            s_vec5.push_back(s_plot[i]);
        }
        fclose(fp2);
    }
}

void Fast_Lio::lasermap_fov_segment() {
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    V3D pos_LiD = pos_lid;
    if (!Localmap_Initialized) {
        for (int i = 0; i < 3; i++) {
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
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
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) {
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    double delete_begin = omp_get_wtime();
    if (cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    kdtree_delete_time = omp_get_wtime() - delete_begin;
}

Fast_Lio::~Fast_Lio() {
    /**************** save map ****************/
    /* 1. make sure you have enough memories
     *
     * 2. pcd save will largely influence the real-time performences
     */
    if (pcl_wait_save->size() > 0 && pcd_save_en) {
        string file_name = string("scans.pcd");
        string all_points_dir(root_dir + "PCD/" + file_name);
        pcl::PCDWriter pcd_writer;
        cout << "current scan saved to /PCD/" << file_name << endl;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }

    fout_out.close();
    fout_pre.close();

    if (runtime_pos_log) {
        vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;
        FILE *fp2;
        string log_dir = root_dir + "/Log/fast_lio_time_log.csv";
        fp2 = fopen(log_dir.c_str(), "w");
        fprintf(fp2,
                "time_stamp, total time, scan point size, incremental time, search time, delete size, delete time, tree size st, tree size end, add point size, preprocess time\n");
        for (int i = 0; i < time_log_counter; i++) {
            fprintf(fp2, "%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n", T1[i], s_plot[i], int(s_plot2[i]),
                    s_plot3[i], s_plot4[i], int(s_plot5[i]), s_plot6[i], int(s_plot7[i]), int(s_plot8[i]),
                    int(s_plot10[i]), s_plot11[i]);
            t.push_back(T1[i]);
            s_vec.push_back(s_plot9[i]);
            s_vec2.push_back(s_plot3[i] + s_plot6[i]);
            s_vec3.push_back(s_plot4[i]);
            s_vec5.push_back(s_plot[i]);
        }
        fclose(fp2);
    }
}

void Fast_Lio::RGBpointBodyToWorld(const PointType *const pi, PointType *const po) {

    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(
            state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;

}

void Fast_Lio::pointBodyToWorld(const PointType *const pi, PointType *const po) {

    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(
            state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;

}

void Fast_Lio::RGBpointBodyLidarToIMU(const PointType *const pi, PointType *const po) {

    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I * p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void Fast_Lio::points_cache_collect() {
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    // for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
}

bool Fast_Lio::sync_packages(MeasureGroup &meas) {
    if (lidar_buffer.empty() || imu_buffer.empty()) {
        return false;
    }

/*** push a lidar scan ***/
    if (!lidar_pushed) {
        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();
        if (meas.lidar->points.size() <= 1) // time too little
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            RCLCPP_WARN(this->get_logger(), "Too few input point cloud!\n");
        } else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime) {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        } else {
            scan_num++;
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
            lidar_mean_scantime +=
                    (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        }

        meas.lidar_end_time = lidar_end_time;

        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time) {
        return false;
    }

/*** push imu data, and pop from imu buffer ***/
    double imu_time = rclcpp::Time(imu_buffer.front()->header.stamp).seconds();
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time)) {
        imu_time = rclcpp::Time(imu_buffer.front()->header.stamp).seconds();
        if (imu_time > lidar_end_time) break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

void Fast_Lio::map_incremental() {
    {
        PointVector PointToAdd;
        PointVector PointNoNeedDownsample;
        PointToAdd.reserve(feats_down_size);
        PointNoNeedDownsample.reserve(feats_down_size);
        for (int i = 0; i < feats_down_size; i++) {
            /* transform to world frame */
            pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
            /* decide if need add to map */
            if (!Nearest_Points[i].empty() && flg_EKF_inited) {
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

        double st_time = omp_get_wtime();
        add_point_size = ikdtree.Add_Points(PointToAdd, true);
        ikdtree.Add_Points(PointNoNeedDownsample, false);
        add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
        kdtree_incremental_time = omp_get_wtime() - st_time;
    }
}

void
Fast_Lio::publish_effect_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudEffect) {
    PointCloudXYZI::Ptr laserCloudWorld(\
                    new PointCloudXYZI(effct_feat_num, 1));
    for (int i = 0; i < effct_feat_num; i++) {
        RGBpointBodyToWorld(&laserCloudOri->points[i], \
                            &laserCloudWorld->points[i]);
    }
    sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = rclcpp::Time(lidar_end_time * 1e9);
    laserCloudFullRes3.header.frame_id = "camera_init";
    pubLaserCloudEffect->publish(laserCloudFullRes3);
}

void Fast_Lio::h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) {

    double match_start = omp_get_wtime();
    laserCloudOri->clear();
    corr_normvect->clear();
    total_residual = 0.0;

    /** closest surface search and residual computation **/
#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    for (int i = 0; i < feats_down_size; i++) {
        PointType &point_body = feats_down_body->points[i];
        PointType &point_world = feats_down_world->points[i];

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

        auto &points_near = Nearest_Points[i];

        if (ekfom_data.converge) {
            /** Find the closest surfaces in the map **/
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected_surf[i] =
                    points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5
                                                                    ? false : true;
        }

        if (!point_selected_surf[i]) continue;

        VF(4) pabcd;
        point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f)) {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
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

    effct_feat_num = 0;

    for (int i = 0; i < feats_down_size; i++) {
        if (point_selected_surf[i]) {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            total_residual += res_last[i];
            effct_feat_num++;
        }
    }

    if (effct_feat_num < 1) {
        ekfom_data.valid = false;
        RCLCPP_WARN(get_logger(), "No Effective Points! \n");
        return;
    }

    res_mean_last = total_residual / effct_feat_num;
    match_time += omp_get_wtime() - match_start;
    double solve_start_ = omp_get_wtime();

    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); //23
    ekfom_data.h.resize(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++) {
        const PointType &laser_p = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() * norm_vec);
        V3D A(point_crossmat * C);
        if (extrinsic_est_en) {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); //s.rot.conjugate()*norm_vec);
            ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(
                    B), VEC_FROM_ARRAY(C);
        } else {
            ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(
                    A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;
    }
    solve_time += omp_get_wtime() - solve_start_;

}

void Fast_Lio::dump_lio_state_to_log(FILE *fp) {

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

void Fast_Lio::publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &pubPath) {

    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = rclcpp::Time(lidar_end_time * 1e9);
    msg_body_pose.header.frame_id = "camera_init";

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0) {
        path.poses.push_back(msg_body_pose);
        pubPath->publish(path);
    }

}

void Fast_Lio::publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr &pubOdomAftMapped) {
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = rclcpp::Time(lidar_end_time * 1e9);
    set_posestamp(odomAftMapped.pose);
    pubOdomAftMapped->publish(odomAftMapped);
    auto P = kf.get_P();
    for (int i = 0; i < 6; i++) {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i * 6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i * 6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i * 6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i * 6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i * 6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i * 6 + 5] = P(k, 2);
    }

    geometry_msgs::msg::TransformStamped trans;

    trans.transform.translation.x = odomAftMapped.pose.pose.position.x;
    trans.transform.translation.y = odomAftMapped.pose.pose.position.y;
    trans.transform.translation.z = odomAftMapped.pose.pose.position.z;

    trans.transform.rotation.w = odomAftMapped.pose.pose.orientation.w;
    trans.transform.rotation.x = odomAftMapped.pose.pose.orientation.x;
    trans.transform.rotation.y = odomAftMapped.pose.pose.orientation.y;
    trans.transform.rotation.z = odomAftMapped.pose.pose.orientation.z;

    trans.header = odomAftMapped.header;
    trans.child_frame_id = "body";

    broadcaster->sendTransform(trans);
}

void
Fast_Lio::publish_frame_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudFull) {
    if (scan_pub_en) {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld(\
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++) {
            RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
                                &laserCloudWorld->points[i]);
        }

        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);

        laserCloudmsg.header.stamp = rclcpp::Time(lidar_end_time * 1e9);
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudFull->publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }

    /**************** save map ****************/
    /*
     * 1. make sure you have enough memories
     * 2. noted that pcd save will influence the real-time performences *
     */
    if (pcd_save_en) {
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr laserCloudWorld(\
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++) {
            RGBpointBodyToWorld(&feats_undistort->points[i], \
                                &laserCloudWorld->points[i]);
        }
        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval) {
            pcd_index++;
            string all_points_dir(string(string(root_dir)
            +"PCD/scans_") +to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
}

void Fast_Lio::publish_map(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudMap) {
    sensor_msgs::msg::PointCloud2 laserCloudMap;
    pcl::toROSMsg(*featsFromMap, laserCloudMap);
    laserCloudMap.header.stamp = rclcpp::Time(lidar_end_time * 1e9);
    laserCloudMap.header.frame_id = "camera_init";
    pubLaserCloudMap->publish(laserCloudMap);
}

void Fast_Lio::publish_frame_body(
        const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudFull_body) {
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++) {
        RGBpointBodyLidarToIMU(&feats_undistort->points[i], \
                            &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = rclcpp::Time(lidar_end_time * 1e9);
    laserCloudmsg.header.frame_id = "body";
    pubLaserCloudFull_body->publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

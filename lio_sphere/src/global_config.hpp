#pragma once
#include <string>
//global configuration
    enum InitialMethod{
        KALMAN=0, IMU=1
    };
    enum MapInitMethod{
        OFF=0, COMBINE=1, SLAM=2
    };
    struct LIOConfig {
    /*--------------
        coordinate frames
            ------------*/
    std::string base_frame{"odom"};
    std::string odom_frame{"map3"};
    std::string kalman_odom_frame{"map2"};
    std::string imu_odom_frame{"map1"};
    std::string imu_frame{"imu_frame"};
    std::string lidar_frame{"pandar_frame"};
    
    /*--------------
      preprocessing
      ------------*/
    // deskewing
    bool deskewing = true; //true: on, false: off
    // range filter
    double min_range = 0.5; //lower limit for accepted point distances in m
    double max_range = 20.0; //upper limit for accepted point distances in m
    //subsampling
    double vox_l_map = 0.35; //voxel size for point cloud added to map in m
    double vox_l_gicp = 0.75; //voxel size for point cloud utilized in GICP in m
    /*--------------
      Initial Guess Determination
                    ------------*/
    InitialMethod initial_method = InitialMethod::KALMAN;
    /*--------------
    GICP Scan Matching
         ------------*/
    bool robust_kernel = true; //true: active, false: inactive
    int target_NN_k = 5; // number of points found for each point in source to construct the target set
    int gicp_max_it = 20; //maximum allowed number of iterations for the GICP loop
    double gicp_th_fitness = 1e-6; //convergence criterium for the fitness value
    double gicp_th_rmse = 1e-6; //convergence criterium for the RMSE value
    double gicp_max_corr_dist = 2; //maximum correspondence distance for GICP scan matching in m (only used when adaptive thresholding inactive)
    /*--------------
    Adaptive Thresholding
            ------------*/
    bool adaptive_th = true; //true: active, false: inactive
    double initial_th= 2.0; //sigma_0 (the initial threshold) which is assumed as long as no other corrections were computed
    double min_motion_th = 0.1; //delta_min (the minimum motion) that have to be measured so that a computed correction is considered in the adaptive threshold determination
    // map params
    double voxel_size = 0.5;
    double overlap_map_voxel_size = 3;
    int max_points_per_voxel = 1;
    int overlap_map_max_points_per_voxel = 1;
    /*--------------
    Global Map Management
            ------------*/
    MapInitMethod map_init_method = SLAM; //choose map init method (no method, take and combine raw scans, apply GraphSLAM)
    double map_init_angle = 360; //angle in degree the sphere has to be rotated so that the map initialization is performed
    int SLAM_max_it = 10; //maximum number of iterations allowed to be executed by GraphSLAM
    double SLAM_epsilon = 0.01; //convergence threshold for GraphSLAM
    double SLAM_max_dist_2 = 0.4; //maximum allowed point distance in GraphSLAM
    int SLAM_min_corr = 100; //minimum number of correspondences that need to be found so that GraphSLAM assumes two scans as overlapping
    double ikd_alpha_bal = 0.7; //balance criterium for the dynamic re-balancing of the ikd-Tree
    double ikd_alpha_del = 0.5; //delete criterium for the dynamic re-balancing of the ikd-Tree
    double ikd_downsample_l = 0.5; //l_box (cuboid box length) of downsample boxes for the automatized point reduction
    double ikd_downsample_n = 5; //n_box^max (maximum point number) of the downsample boxes

};
LIOConfig config_;
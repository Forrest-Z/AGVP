//
// Created by shydh on 12/22/17.
//

#ifndef AGVP_AMCL_H
#define AGVP_AMCL_H

#include <algorithm>
#include <vector>
#include <map>
#include <cmath>

// Signal handling
#include <signal.h>
#include <sensor_msgs.h>
#include <nav_msgs.h>

#include "map.h"
#include "pf.h"
#include "amcl_odom.h"
#include "amcl_laser.h"

#include "tf.h"

// Pose hypothesis
typedef struct
{
    // Total weight (weights sum to 1)
    double weight;

    // Mean of pose esimate
    pf_vector_t pf_pose_mean;

    // Covariance of pose estimate
    pf_matrix_t pf_pose_cov;

} amcl_hyp_t;

typedef struct
{
    base_info::StampedPose &pose_;
    float covariance_[36];
} PoseWithCov;

static const std::string scan_topic_ = "scan";

namespace amcl
{
    class AMCL
    {
    public:
        AMCL(transform2::Transformer latest_tf_,
             transform2::Transformer tf_);

        virtual ~AMCL();

        int process();

    private:
        // Pose-generating function used to uniformly distribute particles over
        // the map
        pf_vector_t uniformPoseGenerator(void *arg);

        // Callbacks
        bool globalLocalizationCallback();
        void nomotionUpdateCallback();
        void setMapCallback();

        void requestMap();

        // Helper to get odometric pose from transform system
        bool getOdomPose(base_info::StampedPose &odom_pose,
                         double &x, double &y, double &yaw,
                         const time_t &t, const int f);

        //从参数服务器获取机器人初始位置
        void updatePoseFromServer();

        void laserReceived(const base_info::LaserScan &laser_scan);

        void initialPoseReceived(const base_info::StampedPose &msg);

        void handleInitialPoseMessage(const PoseWithCov &msg);

        void mapReceived(const base_info::OccupancyGrid &msg);

        void handleMapMessage(const base_info::OccupancyGrid &msg);

        void freeMapDependentMemory();

        map_t *convertMap(const base_info::OccupancyGrid &map_msg);

        void applyInitialPose();

        transform2::Transformer tf_;

        bool sent_first_transform_;

        //parameter for what odom to use
        int odom_frame_id_;

        //parameter for what base to use
        int base_frame_id_;
        int global_frame_id_;

        //paramater to store latest odom pose
        base_info::StampedPose latest_odom_pose_;

        transform2::Transformer latest_tf_;
        bool latest_tf_valid_;

        bool use_map_topic_;
        bool first_map_only_;

        time_t save_pose_last_time;
        time_t save_pose_period;
        time_t last_laser_received_ts_;

        map_t *map_;
        char *mapdata;
        int sx, sy;
        double resolution;

        // Particle filter
        pf_t *pf_;
        double pf_err_, pf_z_;
        bool pf_init_;
        pf_vector_t pf_odom_pose_;
        double d_thresh_, a_thresh_;
        int resample_interval_;
        int resample_count_;
        double laser_min_range_;
        double laser_max_range_;
        std::vector<AMCLLaser *> lasers_;
        std::vector<bool> lasers_update_;
        std::map<int, int> frame_to_laser_;

        int max_beams_, min_particles_, max_particles_;
        double alpha1_, alpha2_, alpha3_, alpha4_, alpha5_;
        double alpha_slow_, alpha_fast_;
        double z_hit_, z_short_, z_max_, z_rand_, sigma_hit_, lambda_short_;
        //beam skip related params
        bool do_beamskip_;
        double beam_skip_distance_, beam_skip_threshold_, beam_skip_error_threshold_;
        double laser_likelihood_max_dist_;
        odom_model_t odom_model_type_;
        double init_pose_[3];
        double init_cov_[3];
        laser_model_t laser_model_type_;
        bool tf_broadcast_;

        //Nomotion update control
        //used to temporarily let amcl update samples even when no motion occurs...
        bool m_force_update;

        amcl_hyp_t *initial_pose_hyp_;
        bool first_map_received_;
        bool first_reconfigure_call_;

        AMCLOdom *odom_;
        AMCLLaser *laser_;

    };
}


#endif //AGVP_AMCL_H

//
// Created by shydh on 12/22/17.
//

#include "include/amcl.h"

namespace amcl
{
    AMCL::AMCL(transform2::Transformer latest_tf_, transform2::Transformer tf_)
            : latest_tf_(latest_tf_), tf_(tf_),
              sent_first_transform_(false),
              latest_tf_valid_(false),
              map_(NULL),
              pf_(NULL),
              resample_count_(0),
              odom_(NULL),
              laser_(NULL),
              initial_pose_hyp_(NULL),
              first_map_received_(false),
              first_reconfigure_call_(true)
    {
        laser_min_range_ = -1.0;
        laser_max_range_ = -1.0;
        max_beams_ = 30;
        min_particles_ = 100;
        max_particles_ = 5000;
        pf_err_ = 0.01;
        pf_z_ = 0.99;
        alpha1_ = 0.2;
        alpha2_ = 0.2;
        alpha3_ = 0.2;
        alpha4_ = 0.2;
        alpha5_ = 0.2;

        do_beamskip_ = false;
        beam_skip_distance_ = 0.5;
        beam_skip_threshold_ = 0.3;
        beam_skip_error_threshold_ = 0.9;

        z_hit_ = 0.95;
        z_short_ = 0.1;
        z_max_ = 0.05;
        z_rand_ = 0.05;
        sigma_hit_ = 0.2;
        lambda_short_ = 0.1;
        laser_likelihood_max_dist_ = 2.0;

        laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD;

        odom_model_type_ = ODOM_MODEL_DIFF;

        d_thresh_ = 0.2;
        a_thresh_ = M_PI / 6.0;
        odom_frame_id_ = 2;
        base_frame_id_ = 1;
        global_frame_id_ = 0;
        resample_interval_ = 2;

        alpha_slow_ = 0.001;
        alpha_fast_ = 0.1;
        tf_broadcast_ = true;
    }

    AMCL::~AMCL()
    {
        delete map_;
        //freeMapDependentMemory();
        delete pf_;
        delete odom_;
        delete laser_;
        delete initial_pose_hyp_
        delete tf_;
    }

    int AMCL::process()
    {
        return 0;
    }

    void AMCL::updatePoseFromServer()
    {
        init_pose_[0] = 0.0;
        init_pose_[1] = 0.0;
        init_pose_[2] = 0.0;
        init_cov_[0] = 0.5 * 0.5;
        init_cov_[1] = 0.5 * 0.5;
        init_cov_[2] = (M_PI/12.0) * (M_PI/12.0);

    }

}

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

#include "map.h"
#include "pf.h"
#include "amcl_odom.h"
#include "amcl_laser.h"

#include "../../transform/tf.h"

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

static const std::string scan_topic_ = "scan";

namespace amcl
{
    class AMCL
    {
    public:
        AMCL();

        virtual ~AMCL();

        int process();

    private:
        transform2::Transformer tf_;

        bool sent_first_transform_;

        transform2::Transformer latest_tf_;
        bool latest_tf_valid_;
    };
}


#endif //AGVP_AMCL_H

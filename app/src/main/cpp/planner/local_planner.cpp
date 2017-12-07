//
// Created by shydh on 7/15/17.
//

#include "include/local_planner.h"
#include "../costmap/include/costmap_2d_ros.h"

using namespace base_info;

namespace nav_core
{
    LocalPlanner::LocalPlanner()
    {

    }

    LocalPlanner::~LocalPlanner()
    {

    }

    bool LocalPlanner::computeVelocityCommands(bd_Velocity &cmd_vel)
    {
        return false;
    }

    bool LocalPlanner::isGoalReached()
    {
        return false;
    }

    bool LocalPlanner::setPlan(const std::vector<StampedPose> &plan)
    {
        return false;
    }

    void LocalPlanner::initialize(transform2::Transformer *tf,
                                  costmap_core::Costmap2DROS *costmap)
    {

    }
}

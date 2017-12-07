//
// Created by shydh on 7/15/17.
//

#ifndef AGVP_LOCAL_PLANNER_H
#define AGVP_LOCAL_PLANNER_H


#include <string>
#include "../../transform/tf.h"
#include "../../costmap/include/costmap_2d.h"
#include "../../costmap/include/costmap_2d_ros.h"

namespace nav_core
{
    /**
     * @class BaseLocalPlanner
     * @brief Provides an interface for local planners used in navigation.
     * All local planners written as plugins for the navigation stack must adhere to this interface.
     */
    class LocalPlanner
    {

    public:
        LocalPlanner();

        /**
         * @brief  Virtual destructor for the interface
         */
        virtual ~LocalPlanner();

        /**
         * @brief  Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
         * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
         * @return True if a valid velocity command was found, false otherwise
         */
        bool computeVelocityCommands(base_info::bd_Velocity &cmd_vel);

        /**
         * @brief  Check if the goal pose has been achieved by the local planner
         * @return True if achieved, false otherwise
         */
        bool isGoalReached();

        /**
         * @brief  Set the plan that the local planner is following
         * @param plan The plan to pass to the local planner
         * @return True if the plan was updated successfully, false otherwise
         */
        bool setPlan(const std::vector<base_info::StampedPose> &plan);

        /**
         * @brief  Constructs the local planner
         * @param name The name to give this instance of the local planner
         * @param tf A pointer to a transform listener
         * @param costmap_ros The cost map to use for assigning costs to local plans
         */
        void initialize(transform2::Transformer *tf, costmap_core::Costmap2DROS *costmap);


    };
} // namespace nav_core


#endif //AGVP_LOCAL_PLANNER_H

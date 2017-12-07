//
// Created by shydh on 5/9/17.
//

#ifndef AGVP_MOVE_BASE_H
#define AGVP_MOVE_BASE_H

#include <vector>
#include <string>
#include <complex>
#include <memory>
#include <condition_variable>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/core.hpp>

#include "costmap/include/costmap_2d.h"
#include "planner/include/global_planner.h"
#include "planner/include/local_planner.h"
#include "costmap/include/costmap_2d_ros.h"

namespace move_base
{
    enum MoveBaseState
    {
        PLANNING,
        CONTROLLING,
        CLEARING
    };

    typedef base_info::StampedPose POSE;

    class MoveBase
    {
    public:
        /**
         * @brief  Constructor for the actions
         * @param name The name of the action
         * @param tf A reference to a TransformListener
         */
        MoveBase(cv::Mat src_map);

        /**
         * @brief  Destructor - Cleans up
         */
        virtual ~MoveBase();

        void initCb(double *current, double *goal);

        void executeCb();

    private:
        //set up plan triple buffer
        std::vector<POSE> *planner_plan_, *latest_plan_, *controller_plan_;

        MoveBaseState state_;

        //set up the planner's thread
        bool runPlanner_;
        std::mutex planner_mutex_;
        std::condition_variable planner_cond_;
        std::thread plan_thread;

        POSE planner_goal_, current_pose_, current_position_, oscillation_pose_;
        double oscillation_distance_;

        bool shutdown_costmaps_, clearing_rotation_allowed_, recovery_behavior_enabled_;
        bool new_global_plan_, running_;
        double oscillation_timeout_;
        time_t last_valid_plan_, last_valid_control_, last_oscillation_reset_;

        double planner_frequency_, controller_frequency_,
                inscribed_radius_, circumscribed_radius_;
        time_t planner_patience_, controller_patience_;
        double conservative_reset_dist_, clearing_radius_;

        base_info::bd_Velocity cmd_vel_;

        costmap_core::Costmap2DROS *planner_costmap_, *controller_costmap_;
        std::shared_ptr<nav_core::GlobalPlanner> planner_global_;
        std::shared_ptr<nav_core::LocalPlanner> planner_local_;

        transform2::Transformer *tf_;

        std::recursive_mutex configuration_mutex_;

        cv::Mat src_map_;

        /**
         * @brief  Performs a control cycle
         * @param goal A reference to the goal to pursue
         * @param global_plan A reference to the global plan being used
         * @return True if processing of the goal is done, false otherwise
         */
        bool executeCycle(POSE &goal, std::vector<POSE> &global_plan);

        void planThread();

        POSE goalToGlobalFrame(const POSE &goal_pose_msg);

        double distance(const POSE &p1, const POSE &p2);

        /**
       * @brief  A service call that clears the costmaps of obstacles
       * @param req The service request
       * @param resp The service response
       * @return True if the service call succeeds, false otherwise
       */
        void clearCostmapsService();

        /**
       * @brief  Make a new global plan
       * @param  goal The goal to plan to
       * @param  plan Will be filled in with the plan made by the planner
       * @return  True if planning succeeds, false otherwise
       */
        bool makePlan(const POSE goal, std::vector<POSE> &plan);

        /**
       * @brief  Clears obstacles within a window around the robot
       * @param size_x The x size of the window
       * @param size_y The y size of the window
       */
        void clearCostmapWindows(double size_x, double size_y);

        /**
       * @brief This is used to wake the planner at periodic intervals.
       */
        void wakePlanner();

        void publishZeroVelocity();

        void resetState();
    };
}


#endif //AGVP_MOVE_BASE_H

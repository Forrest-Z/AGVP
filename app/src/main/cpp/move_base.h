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
        bool planCb();
        void executeCb();

        std::vector<std::pair<u_int, u_int>> get_plan();
        cv::Mat src_map_;

    private:
        //set up plan triple buffer
        std::vector<POSE> *planner_plan_, *latest_plan_, *controller_plan_;

        MoveBaseState state_;

        //set up the planner's thread
        bool runPlanner_;
        std::mutex planner_mutex_;
        std::thread plan_thread, excute_thread;

        POSE planner_goal_, current_pose_, current_position_, oscillation_pose_;
        double oscillation_distance_;

        bool shutdown_costmaps_;
        bool new_global_plan_, running_;

        double planner_frequency_, circumscribed_radius_;

        base_info::bd_Velocity cmd_vel_;

        costmap_core::Costmap2DROS *planner_costmap_, *controller_costmap_;
        std::shared_ptr<nav_core::GlobalPlanner> planner_global_;
        std::shared_ptr<nav_core::LocalPlanner> planner_local_;

        transform2::Transformer *tf_;

        /**
         * @brief  Performs a control cycle
         * @param goal A reference to the goal to pursue
         * @param global_plan A reference to the global plan being used
         * @return True if processing of the goal is done, false otherwise
         */
        bool executeCycle(POSE &goal, std::vector<POSE> &global_plan);

        double distance(const POSE &p1, const POSE &p2);

        /**
       * @brief  Make a new global plan
       * @param  goal The goal to plan to
       * @param  plan Will be filled in with the plan made by the planner
       * @return  True if planning succeeds, false otherwise
       */
        bool makePlan(const POSE goal, std::vector<POSE> &plan);

        void publishZeroVelocity();

        void resetState();
    };
}


#endif //AGVP_MOVE_BASE_H

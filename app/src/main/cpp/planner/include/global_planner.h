//
// Created by shydh on 7/2/17.
//

#ifndef AGVP_GLOBAL_PLANNER_H
#define AGVP_GLOBAL_PLANNER_H

#include <string>

#include "../../costmap/include/costmap_2d.h"
#include "PotentialCalculator.h"
#include "OrientationFilter.h"
#include "Traceback.h"
#include "Expander.h"
#include "../../costmap/include/costmap_2d_ros.h"

namespace nav_core
{
    /**
     * @class PlannerCore
     * @brief Provides a ROS wrapper for the global_planner planner
     * which runs a fast, interpolated navigation function on a costmap.
     */

    class GlobalPlanner
    {
    public:
        /**
         * @brief  Default constructor for the PlannerCore object
         */
        GlobalPlanner();

        /**
         * @brief  Constructor for the PlannerCore object
         * @param  name The name of this planner
         * @param  costmap A pointer to the costmap to use
         * @param  frame_id Frame of the costmap
         */
        GlobalPlanner(costmap_core::Costmap2D *costmap, int frame_id);

        /**
         * @brief  Default deconstructor for the PlannerCore object
         */
        ~GlobalPlanner();

        /**
         * @brief  Initialization function for the PlannerCore object
         * @param  name The name of this planner
         * @param  costmap_ros A pointer to the ROS wrapper of the costmap
         * to use for planning
         */
        void initialize(costmap_core::Costmap2DROS *costmap_ros);
        void initialize(costmap_core::Costmap2D *costmap, int frame_id);

        /**
         * @brief Given a goal pose in the world, compute a plan
         * @param start The start pose
         * @param goal The goal pose
         * @param tolerance The tolerance on the goal point for the planner
         * @param plan The plan... filled by the planner
         * @return True if a valid plan was found, false otherwise
         */
        bool makePlan(const POSE &start,
                      const POSE &goal, double tolerance,
                      std::vector<POSE> &plan);

        /**
         * @brief Compute a plan to a goal after the potential
         * for a start point has already been computed
         * (Note: You should call computePotential first)
         * @param start_x
         * @param start_y
         * @param end_x
         * @param end_y
         * @param goal The goal pose to create a plan to
         * @param plan The plan... filled by the planner
         * @return True if a valid plan was found, false otherwise
         */
        bool getPlanFromPotential(u_int start_x, u_int start_y,
                                  u_int goal_x, u_int goal_y,
                                  const POSE &goal,
                                  std::vector<POSE> &plan);

        std::vector<std::pair<u_int, u_int>> path_;

    protected:

        /**
         * @brief Store a copy of the current costmap in \a costmap.  Called by makePlan.
         */
        costmap_core::Costmap2D *costmap_;
        int frame_id_;

        bool initialized_, allow_unknown_;

    private:

        void clearRobotCell(const POSE& global_pose, u_int mx, u_int my);

        //将costmap的四个边的全部cell都设置为LETHAL_OBSTACLE
        void outlineMap(u_char* costarr, int nx, int ny, u_char value);

        //double planner_window_x_, planner_window_y_, default_tolerance_;

        std::mutex mutex_;

        PotentialCalculator *p_calc_;
        Expander *planner_;
        Traceback *path_maker_;
        OrientationFilter *orientation_filter_;

        //u_char *cost_array_;
        double *potential_array_;
        //u_int start_x_, start_y_, end_x_, end_y_;

    };
};

#endif //AGVP_GLOBAL_PLANNER_H

//
// Created by shydh on 7/2/17.
//

#include "include/global_planner.h"
#include "include/QuadraticCalculator.h"
#include "include/AStarExpansion.h"
#include "include/GridPath.h"

namespace nav_core
{
    GlobalPlanner::GlobalPlanner() :
            costmap_(NULL), initialized_(false), allow_unknown_(true)
    {

    }

    GlobalPlanner::GlobalPlanner(costmap_core::Costmap2D *costmap,
                                 int frame_id) :
            costmap_(NULL), initialized_(false), allow_unknown_(true)
    {
        //initialize the planner
        initialize(costmap, frame_id);
    }

    GlobalPlanner::~GlobalPlanner()
    {
        if (p_calc_)
            delete p_calc_;
        if (planner_)
            delete planner_;
        if (path_maker_)
            delete path_maker_;
        if (orientation_filter_)
            delete orientation_filter_;
    }

    void GlobalPlanner::initialize(costmap_core::Costmap2DROS *costmap)
    {
        initialize(costmap->getCostmap(), costmap->getGlobalFrameID());
    }

    void GlobalPlanner::initialize(costmap_core::Costmap2D *costmap,
                                   int frame_id)
    {
        if (!initialized_)
        {
            costmap_ = costmap;
            frame_id_ = frame_id;

            unsigned int cx = costmap->getSizeInCellsX(),
                    cy = costmap->getSizeInCellsY();

            if (!old_navfn_behavior_)
                convert_offset_ = 0.5;
            else
                convert_offset_ = 0.0;

            p_calc_ = new QuadraticCalculator(cx, cy);
            planner_ = new AStarExpansion(p_calc_, cx, cy);
            path_maker_ = new GridPath(p_calc_);
            orientation_filter_ = new OrientationFilter();

            planner_->setHasUnknown(allow_unknown_);

            initialized_ = true;
        } else
            return;
        LOGI("This planner has already been initialized");
    }

    bool GlobalPlanner::makePlan(const POSE &start, const POSE &goal, double tolerance,
                                 std::vector<POSE> &plan)
    {
        std::unique_lock<std::mutex> lock(mutex_);

        if (!initialized_)
            return false;

        //clear the plan, just in case清除plan
        plan.clear();

        double wx = start.x();
        double wy = start.y();

        unsigned int start_x_i, start_y_i, goal_x_i, goal_y_i;
        double start_x, start_y, goal_x, goal_y;

        if (!costmap_->worldToMap(wx, wy, start_x_i, start_y_i))
        {
            LOGI("The robot's start position is off the global costmap."
                         " Planning will always fail, "
                         "are you sure the robot has been properly localized?");
            return false;
        }

        if (old_navfn_behavior_)
        {
            start_x = start_x_i;
            start_y = start_y_i;
        } else
        {
            worldToMap(wx, wy, start_x, start_y);
        }

        wx = goal.x();
        wy = goal.y();

        if (!costmap_->worldToMap(wx, wy, goal_x_i, goal_y_i))
        {
            LOGI("The goal sent to the global planner "
                         "is off the global costmap."
                         " Planning will always fail to this goal.");
            return false;
        }
        if (old_navfn_behavior_)
        {
            goal_x = goal_x_i;
            goal_y = goal_y_i;
        } else
            worldToMap(wx, wy, goal_x, goal_y);

        //clear the starting cell within the costmap
        //because we know it can't be an obstacle
        POSE start_pose;
        clearRobotCell(start_pose, start_x_i, start_y_i);

        int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();

        //make sure to resize the underlying array that Navfn uses
        p_calc_->setSize(nx, ny);
        planner_->setSize(nx, ny);
        path_maker_->setSize(nx, ny);
        potential_array_ = new double[nx * ny];

        outlineMap(costmap_->getCharMap(), nx, ny, 0);

        bool found_legal =
                planner_->calculatePotentials(costmap_->getCharMap(),
                                              start_x, start_y, goal_x, goal_y,
                                              nx * ny * 2, potential_array_);

        if (!old_navfn_behavior_)
            planner_->clearEndpoint(costmap_->getCharMap(),
                                    potential_array_, goal_x_i, goal_y_i, 2);

        if (found_legal)
        {
            //extract the plan
            if (getPlanFromPotential(start_x, start_y, goal_x, goal_y, goal, plan))
            {
                //make sure the goal we push on has the same timestamp
                //as the rest of the plan
                POSE goal_copy = goal;
                plan.push_back(goal_copy);
            }
        }

        // add orientations if needed
        orientation_filter_->processPath(start, plan);

        delete potential_array_;
        return !plan.empty();
    }

    void GlobalPlanner::outlineMap(unsigned char *costarr,
                                   int nx, int ny, unsigned char value)
    {
        unsigned char *pc = costarr;
        for (int i = 0; i < nx; i++)
            *pc++ = value;
        pc = costarr + (ny - 1) * nx;
        for (int i = 0; i < nx; i++)
            *pc++ = value;
        pc = costarr;
        for (int i = 0; i < ny; i++, pc += nx)
            *pc = value;
        pc = costarr + nx - 1;
        for (int i = 0; i < ny; i++, pc += nx)
            *pc = value;
    }

    bool
    GlobalPlanner::getPlanFromPotential(double start_x, double start_y,
                                        double goal_x, double goal_y,
                                        const POSE &goal, std::vector<POSE> &plan)
    {
        if (!initialized_)
        {
            LOGI("This planner has not been initialized yet, "
                         "but it is being used, please call initialize() before use");
            return false;
        }

        int global_frame = frame_id_;

        //clear the plan, just in case
        plan.clear();

        std::vector<std::pair<double, double>> path;

        if (!path_maker_->getPath(potential_array_, start_x, start_y,
                                  goal_x, goal_y, path))
        {
            LOGI("NO PATH!");
            return false;
        }

        time_t plan_time = time(NULL);
        for (unsigned long i = path.size() - 1; i >= 0; i--)
        {
            std::pair<float, float> point = path[i];
            //convert the plan to world coordinates
            double world_x, world_y;
            mapToWorld(point.first, point.second, world_x, world_y);

            base_info::StampedPose pose;
            pose.stamp_ = plan_time;
            pose.frame_id_ = global_frame;
            pose.x() = world_x;
            pose.y() = world_y;
            pose.z() = 0.0;
            plan.push_back(pose);
        }
        if (old_navfn_behavior_)
        {
            plan.push_back(goal);
        }
        return !plan.empty();
    }

    void GlobalPlanner::mapToWorld(double mx, double my, double &wx, double &wy)
    {
        wx = costmap_->getOriginX() + (mx + convert_offset_)
                                      * costmap_->getResolution();
        wy = costmap_->getOriginY() + (my + convert_offset_)
                                      * costmap_->getResolution();
    }

    bool GlobalPlanner::worldToMap(double wx, double wy, double &mx, double &my)
    {
        double origin_x = costmap_->getOriginX(), origin_y = costmap_->getOriginY();
        double resolution = costmap_->getResolution();

        if (wx < origin_x || wy < origin_y)
            return false;

        mx = (wx - origin_x) / resolution - convert_offset_;
        my = (wy - origin_y) / resolution - convert_offset_;

        return mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY();

    }

    void GlobalPlanner::clearRobotCell(const POSE &global_pose, unsigned int mx, unsigned int my)
    {
        if (!initialized_)
        {
            LOGI("This planner has not been initialized yet, "
                         "but it is being used, please call initialize() before use");
            return;
        }

        //set the associated costs in the cost map to be free
        costmap_->setCost(mx, my, costmap_core::FREE_SPACE);
    }
}


#include <cmath>
#include <thread>
#include <unistd.h>

#include "move_base.h"
#include "log_utils.h"
#include "Eigen/Eigen"

using namespace Eigen;
using namespace base_info;

namespace move_base
{
    MoveBase::MoveBase(cv::Mat src_map) :
            planner_costmap_(NULL), controller_costmap_(NULL),
            planner_plan_(NULL), latest_plan_(NULL), controller_plan_(NULL),
            runPlanner_(false), new_global_plan_(false), running_(true),
            tf_(NULL)
    {
        //transform setting
        time_t now_time = time(NULL);
        tf_ = new transform2::Transformer(now_time);

        //get some parameters that will be global to the move base node
        //"planner_frequency"
        planner_frequency_ = 0.0;
        //"controller_frequency"
        //controller_frequency_ = 20.0;
        //"planner_patience",
        //planner_patience_ = (time_t) 5.0;
        //"controller_patience",
        //controller_patience_ = (time_t) 15.0;

        //"oscillation_timeout",
        //oscillation_timeout_ = 0.0;
        //"oscillation_distance",
        oscillation_distance_ = 0.5;

        //we'll assume the radius of the robot to be consistent
        //with what's specified for the costmaps
        //"local_costmap/inscribed_radius"
        //inscribed_radius_ = 0.325;
        //"local_costmap/circumscribed_radius"
        circumscribed_radius_ = 0.46;
        //"clearing_radius"
        //clearing_radius_ = circumscribed_radius_;
        //"conservative_reset_dist"
        //conservative_reset_dist_ = 3.0;

        //"shutdown_costmaps"
        shutdown_costmaps_ = false;

        src_map_ = src_map;

        //plan_thread = std::thread(&MoveBase::planThread, this);

        //set up plan triple buffer
        planner_plan_ = new std::vector<POSE>();
        //latest_plan_ = new std::vector<POSE>();
        //controller_plan_ = new std::vector<POSE>();

        //create the ros wrapper for the planner's costmap...
        //and initializer a pointer we'll use with the underlying map
        planner_costmap_ = new costmap_core::Costmap2DROS(*tf_, &src_map_);
        planner_costmap_->pause();

        //initialize the global planner
        planner_global_ = std::make_shared<nav_core::GlobalPlanner>();
        planner_global_->initialize(planner_costmap_);

        //create the ros wrapper for the controller's costmap...
        //and initializer a pointer we'll use with the underlying map
        //controller_costmap_ = new costmap_core::Costmap2DROS(*tf_, &src_map_);
        //controller_costmap_->pause();

        //create a local planner
        //planner_local_ = std::make_shared<nav_core::LocalPlanner>();
        //ROS_INFO("Created local_planner %s", local_planner.c_str());
        //planner_local_->initialize(tf_, controller_costmap_);
    }

    MoveBase::~MoveBase()
    {
        running_ = false;
        LOGI("movebase stopped!!!!!!");

        if (plan_thread.joinable())
            plan_thread.join();

        if (excute_thread.joinable())
            excute_thread.join();

        if (planner_costmap_ != NULL)
            delete planner_costmap_;
        if (controller_costmap_ != NULL)
            delete controller_costmap_;
        if (planner_plan_ != NULL)
            delete planner_plan_;
        if (latest_plan_ != NULL)
            delete latest_plan_;
        if (controller_plan_ != NULL)
            delete controller_plan_;
        if (tf_ != NULL)
            delete tf_;
    }

    void MoveBase::initCb(double *current, double *goal)
    {
        //transform setting
        time_t now_time = time(NULL);

        //set the current position
        current_pose_.x() = current[0];
        current_pose_.y() = current[1];
        current_pose_.z() = current[2];
        current_pose_.stamp_ = now_time;

        //set up plan triple buffer
        planner_goal_.x() = goal[0];
        planner_goal_.y() = goal[1];
        planner_goal_.z() = goal[2];
        planner_goal_.stamp_ = now_time;

        cmd_vel_.x() = 0.0;
        cmd_vel_.y() = 0.0;
        cmd_vel_.z() = 0.0;

        // Start actively updating costmaps based on sensor data
        LOGI("move_base,Starting costmaps initially");
        planner_costmap_->start();
        //controller_costmap_->start();

        //initially, we'll need to make a plan
        state_ = PLANNING;
        LOGI("move_base,Planning initially");
        //we're all set up now so we can start the action server
        //as_->start();

        //excute_thread = std::thread(&MoveBase::executeCb, this);

        if (!planner_costmap_)
            resetState();
    }

    bool MoveBase::planCb()
    {
        //if we are in a planning state, then we'll attempt to make a plan
        if (state_ == PLANNING)
        {
            //time to plan! get a copy of the goal and unlock the mutex
            if (makePlan(planner_goal_, *planner_plan_))
            {
                LOGI("Got Plan with %zu points!", planner_plan_->size());
                /*pointer swap the plans under mutex
                (the controller will pull from latest_plan_)
                std::vector<POSE> temp_plan = *planner_plan_;
                planner_plan_ = latest_plan_;
                *latest_plan_ = temp_plan;
                new_global_plan_ = true;*/

                //LOGI("Generated a plan from the base_global_planner");

                //make sure we only start the controller
                //if we still haven't reached the goal
                state_ = CONTROLLING;
                if (planner_frequency_ <= 0)
                    runPlanner_ = false;
                return true;
            } else if (state_ == PLANNING)
            {
                //if we didn't get a plan
                //and we are in the planning state (the robot isn't moving)
                LOGI("No Plan...");
                return false;
            }
            //Waiting for plan, in the planning state.

        } else
            return false;
    }

    void MoveBase::executeCb()
    {
        POSE goal = planner_goal_;
        std::vector<POSE> global_plan;

        LOGI("excuteCB loop started.");
        while (running_)
        {
            //the real work on pursuing a goal is done here
            //if we're done, then we'll return from execute
            if (executeCycle(goal, global_plan))
                break;

            //make sure to sleep for the remainder of our cycle time
            usleep(500);
        }
        LOGI("excuteCB loop stopped.");
        return;
    }

    bool MoveBase::executeCycle(POSE &goal,
                                std::vector<POSE> &global_plan)
    {
        //std::recursive_mutex ecl(configuration_mutex_);
        //we need to be able to publish velocity commands
        bd_Velocity cmd_vel;

        //update feedback to correspond to our curent position
        POSE global_pose;
        planner_costmap_->getRobotPose(global_pose);

        //check to see if we've moved far enough to reset our oscillation timeout
        if (distance(current_position_, oscillation_pose_) >= oscillation_distance_)
        {
            //last_oscillation_reset_ = ros::Time::now();
            oscillation_pose_ = current_position_;
        }

        //check that the observation buffers for the costmap are current,
        // we don't want to drive blind
        /*if (!controller_costmap_->isCurrent())
        {
            //ROS_WARN("[%s]:Sensor data is out of date, "
            //we're not going to allow commanding of the base for safety",
            // ros::this_node::getName().c_str());
            publishZeroVelocity();
            return false;
        }*/

        //the move_base state machine, handles the control logic for navigation
        switch (state_)
        {
            //if we're controlling, we'll attempt to find valid velocity commands
            case CONTROLLING:
                LOGI("In controlling state.");
                break;

                //we'll try to clear out space with any user-provided recovery behaviors
            case CLEARING:
                LOGI("In clearing/recovery state");
                //we'll invoke whatever recovery behavior
                // we're currently on if they're enabled
                break;
            default:
                break;
        }
        //we aren't done yet
        return false;
    }

    bool MoveBase::makePlan(const POSE goal, std::vector<POSE> &plan)
    {
        std::lock_guard<costmap_core::Costmap2D::mutex_t>
                lock(*(planner_costmap_->getCostmap()->getMutex()));
        //make sure to set the plan to be empty initially
        plan.clear();
        //get the starting pose of the robot
        POSE global_pose;
        /*if (!planner_costmap_->getRobotPose(global_pose))
        {
            LOGI("Unable to get starting pose of robot, unable to create global plan");
            return false;
        }*/
        global_pose = current_pose_;
        //if the planner fails or returns a zero length plan, planning failed
        POSE start;
        //if the user does not specify a start pose, identified by an empty frame id,
        //then use the robot's pose
        tf_->transformPose(1, global_pose, start);
        LOGI("268");
        return planner_global_->makePlan(start, goal, 0.1, plan);

    }

    double MoveBase::distance(const POSE &p1, const POSE &p2)
    {
        return hypot(p1.x() - p2.x(),
                     p1.y() - p2.y());
    }

    void MoveBase::publishZeroVelocity()
    {
        cmd_vel_.x() = 0.0;
        cmd_vel_.y() = 0.0;
        cmd_vel_.z() = 0.0;
    }

    void MoveBase::resetState()
    {
        // Disable the planner thread
        std::unique_lock<std::mutex> lock(planner_mutex_);
        runPlanner_ = false;
        lock.unlock();

        // Reset statemachine
        state_ = PLANNING;

        //if we shutdown our costmaps when we're deactivated... we'll do that now
        if (shutdown_costmaps_)
        {
            LOGI("Stopping costmaps");
            planner_costmap_->stop();
            controller_costmap_->stop();
        }
    }

    std::vector<std::pair<u_int, u_int>> MoveBase::get_plan()
    {
        return planner_global_->path_;
    }
}

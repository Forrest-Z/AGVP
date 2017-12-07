// Created by shydh on 5/9/17
//
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
        controller_frequency_ = 20.0;
        //"planner_patience",
        planner_patience_ = (time_t) 5.0;
        //"controller_patience",
        controller_patience_ = (time_t) 15.0;

        //"oscillation_timeout",
        oscillation_timeout_ = 0.0;
        //"oscillation_distance",
        oscillation_distance_ = 0.5;

        //we'll assume the radius of the robot to be consistent
        //with what's specified for the costmaps
        //"local_costmap/inscribed_radius"
        inscribed_radius_ = 0.325;
        //"local_costmap/circumscribed_radius"
        circumscribed_radius_ = 0.46;
        //"clearing_radius"
        clearing_radius_ = circumscribed_radius_;
        //"conservative_reset_dist"
        conservative_reset_dist_ = 3.0;

        //"shutdown_costmaps"
        shutdown_costmaps_ = false;
        //"clearing_rotation_allowed"
        clearing_rotation_allowed_ = true;
        //"recovery_behavior_enabled"
        recovery_behavior_enabled_ = true;

        src_map_ = src_map;

        plan_thread = std::thread(&MoveBase::planThread, this);
        plan_thread.detach();

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

        //if we shutdown our costmaps when we're deactivated... we'll do that now
        if (shutdown_costmaps_)
        {
            LOGI("move_base,Stopping costmaps");
            planner_costmap_->stop();
            //controller_costmap_->stop();
        }

        //initially, we'll need to make a plan
        state_ = PLANNING;
        LOGI("move_base,Planning initially");
        //we're all set up now so we can start the action server
        //as_->start();

        if (!planner_costmap_)
            resetState();
    }

    void MoveBase::clearCostmapsService()
    {
        //clear the costmaps
        planner_costmap_->resetLayers();
        controller_costmap_->resetLayers();
    }

    void MoveBase::executeCb()
    {
        POSE goal;
        //we have a goal so start the planner
        std::unique_lock<std::mutex> lock(planner_mutex_);
        goal = planner_goal_;
        runPlanner_ = true;
        planner_cond_.notify_one();
        lock.unlock();

        std::vector<POSE> global_plan;

        //ros::Rate r(controller_frequency_);
        if (shutdown_costmaps_)
        {
            /*ROS_DEBUG_NAMED("move_base",
                            "Starting up costmaps that were shut down previously");*/
            planner_costmap_->start();
            controller_costmap_->start();
        }

        //we want to make sure that we reset the last time
        //we had a valid plan and control
        last_valid_control_ = time(NULL);
        last_valid_plan_ = time(NULL);
        last_oscillation_reset_ = time(NULL);

        while (running_)
        {
            //we also want to check if we've changed global frames
            //because we need to transform our goal pose
            if (goal.frame_id_ != planner_costmap_->getGlobalFrameID())
            {
                goal = goalToGlobalFrame(goal);

                //we want to go back to the planning state for the next execution cycle
                state_ = PLANNING;

                //we have a new goal so make sure the planner is awake
                lock.lock();
                planner_goal_ = goal;
                runPlanner_ = true;
                planner_cond_.notify_one();
                lock.unlock();

                //make sure to reset our timeouts
                last_valid_control_ = time(NULL);
                last_valid_plan_ = time(NULL);
                last_oscillation_reset_ = time(NULL);
            }

            //the real work on pursuing a goal is done here
            bool done = executeCycle(goal, global_plan);

            //if we're done, then we'll return from execute
            if (done)
                return;

            //check if execution of the goal has completed in some way
            //ROS_DEBUG_NAMED("move_base",
            // "Full control cycle time: %.9f\n", t_diff.toSec());
            sleep(50);
            //make sure to sleep for the remainder of our cycle time
        }

        //wake up the planner thread so that it can exit cleanly
        lock.lock();
        runPlanner_ = true;
        planner_cond_.notify_one();
        lock.unlock();

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

        //push the feedback out
        //move_base_msgs::MoveBaseFeedback feedback;
        //feedback.base_position = current_position;
        //as_->publishFeedback(feedback);

        //check to see if we've moved far enough to reset our oscillation timeout
        if (distance(current_position_, oscillation_pose_) >= oscillation_distance_)
        {
            //last_oscillation_reset_ = ros::Time::now();
            oscillation_pose_ = current_position_;
        }

        //check that the observation buffers for the costmap are current,
        // we don't want to drive blind
        if (!controller_costmap_->isCurrent())
        {
            //ROS_WARN("[%s]:Sensor data is out of date, "
            //we're not going to allow commanding of the base for safety",
            // ros::this_node::getName().c_str());
            publishZeroVelocity();
            return false;
        }

        //if we have a new plan then grab it and give it to the controller
        if (new_global_plan_)
        {
            //make sure to set the new plan flag to false
            new_global_plan_ = false;

            //ROS_DEBUG_NAMED("move_base","Got a new plan...swap pointers");

            //do a pointer swap under mutex
            std::vector<POSE> temp_plan = *controller_plan_;

            std::unique_lock<std::mutex> lock(planner_mutex_);
            controller_plan_ = latest_plan_;
            *latest_plan_ = temp_plan;
            lock.unlock();
            //ROS_DEBUG_NAMED("move_base","pointers swapped!");

            if (!planner_local_->setPlan(*controller_plan_))
            {
                //ABORT and SHUTDOWN COSTMAPS
                //ROS_ERROR("Failed to pass global plan to the controller, aborting.");
                resetState();

                //disable the planner thread
                lock.lock();
                runPlanner_ = false;
                lock.unlock();

                //as_->setAborted(move_base_msgs::MoveBaseResult(),
                //                "Failed to pass global plan to the controller.");
                return true;
            }
        }

        //the move_base state machine, handles the control logic for navigation
        switch (state_)
        {
            //if we are in a planning state, then we'll attempt to make a plan
            case PLANNING:
            {
                std::unique_lock<std::mutex> lock(planner_mutex_);
                runPlanner_ = true;
                planner_cond_.notify_one();
            }
                //ROS_DEBUG_NAMED("move_base","Waiting for plan, in the planning state.");
                break;

                //if we're controlling, we'll attempt to find valid velocity commands
            case CONTROLLING:
                //ROS_DEBUG_NAMED("move_base","In controlling state.");

                //check to see if we've reached our goal
                if (planner_local_->isGoalReached())
                {
                    //ROS_DEBUG_NAMED("move_base","Goal reached!");
                    resetState();

                    //disable the planner thread
                    std::unique_lock<std::mutex> lock(planner_mutex_);
                    runPlanner_ = false;
                    lock.unlock();

                    //as_->setSucceeded(move_base_msgs::MoveBaseResult(),
                    //                  "Goal reached.");
                    return true;
                }

                //check for an oscillation condition
                if (oscillation_timeout_ > 0.0 &&
                    last_oscillation_reset_ + oscillation_timeout_ < time(NULL))
                {
                    publishZeroVelocity();
                    state_ = CLEARING;
                }

                {
                    std::unique_lock<costmap_core::Costmap2D::mutex_t>
                            lockc(*(controller_costmap_->getCostmap()->getMutex()));

                    if (planner_local_->computeVelocityCommands(cmd_vel))
                    {
                        //ROS_DEBUG_NAMED( "move_base",
                        //"Got a valid command from the local planner:
                        // %.3lf, %.3lf, %.3lf",
                        //cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z );
                        last_valid_control_ = time(NULL);
                        //make sure that we send the velocity command to the base
                        cmd_vel_ = cmd_vel;
                    } else
                    {
                        //ROS_DEBUG_NAMED("move_base",
                        // "The local planner could not find a valid plan.");
                        time_t attempt_end = last_valid_control_
                                             + static_cast<time_t >(controller_patience_);

                        //check if we've tried to find a valid control
                        // for longer than our time limit
                        if (time(NULL) > attempt_end)
                        {
                            //we'll move into our obstacle clearing mode
                            publishZeroVelocity();
                            state_ = CLEARING;
                            //recovery_trigger_ = CONTROLLING_R;
                        } else
                        {
                            //otherwise, if we can't find a valid control,
                            // we'll go back to planning
                            last_valid_plan_ = time(NULL);
                            state_ = PLANNING;
                            publishZeroVelocity();

                            //enable the planner thread in case it isn't running on a clock
                            std::unique_lock<std::mutex> lock(planner_mutex_);
                            runPlanner_ = true;
                            planner_cond_.notify_one();
                            lock.unlock();
                        }
                    }
                }

                break;

                //we'll try to clear out space with any user-provided recovery behaviors
            case CLEARING:
                //ROS_DEBUG_NAMED("move_base","In clearing/recovery state");
                //we'll invoke whatever recovery behavior
                // we're currently on if they're enabled
                if (recovery_behavior_enabled_)
                {
                    //ROS_DEBUG_NAMED("move_base_recovery",
                    // "Executing behavior %u of %zu",

                    //we at least want to give the robot some time to stop oscillating
                    // after executing the behavior
                    last_oscillation_reset_ = time(NULL);

                    //we'll check if the recovery behavior actually worked
                    //ROS_DEBUG_NAMED("move_base_recovery","Going back to planning state");
                    state_ = PLANNING;

                } else
                {
                    //ROS_DEBUG_NAMED("move_base_recovery",
                    // "All recovery behaviors have failed,
                    // locking the planner and disabling it.");
                    //disable the planner thread
                    std::unique_lock<std::mutex> lock(planner_mutex_);
                    runPlanner_ = false;
                    lock.unlock();

                    //ROS_DEBUG_NAMED("move_base_recovery",
                    // "Something should abort after this.");

                    resetState();
                    return true;
                }
                break;
            default:
                //ROS_ERROR("This case should never be reached,
                // something is wrong, aborting");
                resetState();
                //disable the planner thread
                std::unique_lock<std::mutex> lock(planner_mutex_);
                runPlanner_ = false;
                lock.unlock();
                //as_->setAborted(move_base_msgs::MoveBaseResult(),
                // "Reached a case that should not be hit in move_base.
                // This is a bug, please report it.");
                return true;
        }

        //we aren't done yet
        return false;
    }

    void MoveBase::planThread()
    {
        bool wait_for_wake = false;
        std::unique_lock<std::mutex> lock(planner_mutex_);

        //run planner
        while (running_)
        {
            //check if we should run the planner (the mutex is locked)
            while (wait_for_wake || !runPlanner_)
            {
                //if we should not be running the planner then suspend this thread
                LOGI("Planner thread is suspending");
                planner_cond_.wait(lock);
                wait_for_wake = false;
            }
            time_t start_time = time(NULL);
            (*planner_plan_).clear();

            //time to plan! get a copy of the goal and unlock the mutex
            bool gotPlan = makePlan(planner_goal_, *planner_plan_);
            if (gotPlan)
            {
                LOGI("Got Plan with %zu points!", planner_plan_->size());
                /*pointer swap the plans under mutex
                (the controller will pull from latest_plan_)*/
                std::vector<POSE> temp_plan = *planner_plan_;
                lock.lock();
                planner_plan_ = latest_plan_;
                *latest_plan_ = temp_plan;
                last_valid_plan_ = time(NULL);
                new_global_plan_ = true;

                LOGI("Generated a plan from the base_global_planner");

                //make sure we only start the controller
                //if we still haven't reached the goal
                if (runPlanner_)
                    state_ = CONTROLLING;
                if (planner_frequency_ <= 0)
                    runPlanner_ = false;
                lock.unlock();
            }
                //if we didn't get a plan
                //and we are in the planning state (the robot isn't moving)
            else if (state_ == PLANNING)
            {
                LOGI("No Plan...");
                time_t attempt_end = last_valid_plan_ + planner_patience_;

                //check if we've tried to make a plan for over our time limit
                lock.lock();
                if (time(NULL) > attempt_end && runPlanner_)
                {
                    //we'll move into our obstacle clearing mode
                    state_ = CLEARING;
                    publishZeroVelocity();
                    //recovery_trigger_ = PLANNING_R;
                }
                lock.unlock();
            }

            //take the mutex for the next iteration
            lock.lock();

            //setup sleep interface if needed
            if (planner_frequency_ > 0)
            {
                unsigned int sleep_time =
                        (static_cast<unsigned int>(start_time) +
                         static_cast<unsigned int>(1.0 / planner_frequency_) -
                         static_cast<unsigned int>(time(NULL)));
                if (sleep_time > 0.0)
                {
                    wait_for_wake = true;
                    sleep(sleep_time);
                }
            }
        }
    }

    bool MoveBase::makePlan(const POSE goal, std::vector<POSE> &plan)
    {
        std::lock_guard<costmap_core::Costmap2D::mutex_t>
                lock(*(planner_costmap_->getCostmap()->getMutex()));
        //make sure to set the plan to be empty initially
        plan.clear();

        //get the starting pose of the robot
        POSE global_pose;
        if (!planner_costmap_->getRobotPose(global_pose))
        {
            //"Unable to get starting pose of robot, unable to create global plan"
            return false;
        }
        //if the planner fails or returns a zero length plan, planning failed
        POSE start;
        //if the user does not specify a start pose, identified by an empty frame id,
        //then use the robot's pose
        tf_->transformPose(1, global_pose, start);
        return planner_global_->makePlan(start, goal, 0.1, plan);

    }

    POSE MoveBase::goalToGlobalFrame(const POSE &goal_pose_msg)
    {
        int global_frame = planner_costmap_->getGlobalFrameID();
        StampedPose goal_pose, global_pose;

        //just get the latest available transform... for accuracy they should send
        //goals in the frame of the planner
        goal_pose.stamp_ = time(NULL);

        tf_->transformPose(global_frame, goal_pose, global_pose);

        return global_pose;
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
            //LOGI("move_base", "Stopping costmaps");
            planner_costmap_->stop();
            controller_costmap_->stop();
        }
    }

    void MoveBase::clearCostmapWindows(double size_x, double size_y)
    {
        POSE global_pose;

        //clear the planner's costmap
        planner_costmap_->getRobotPose(global_pose);

        base_info::bd_Polygon clear_poly;

        double x = global_pose.x();
        double y = global_pose.y();

        base_info::bd_Point pt;

        pt.x() = x - size_x / 2;
        pt.y() = y - size_y / 2;
        clear_poly.push_back(pt);

        pt.x() = x + size_x / 2;
        pt.y() = y - size_y / 2;
        clear_poly.push_back(pt);

        pt.x() = x + size_x / 2;
        pt.y() = y + size_y / 2;
        clear_poly.push_back(pt);

        pt.x() = x - size_x / 2;
        pt.y() = y + size_y / 2;
        clear_poly.push_back(pt);

        planner_costmap_->getCostmap()->setConvexPolygonCost(clear_poly,
                                                             costmap_core::FREE_SPACE);

        //clear the controller's costmap
        controller_costmap_->getRobotPose(global_pose);

        clear_poly.clear();
        x = global_pose.x();
        y = global_pose.y();

        pt.x() = x - size_x / 2;
        pt.y() = y - size_y / 2;
        clear_poly.push_back(pt);

        pt.x() = x + size_x / 2;
        pt.y() = y - size_y / 2;
        clear_poly.push_back(pt);

        pt.x() = x + size_x / 2;
        pt.y() = y + size_y / 2;
        clear_poly.push_back(pt);

        pt.x() = x - size_x / 2;
        pt.y() = y + size_y / 2;
        clear_poly.push_back(pt);

        controller_costmap_->getCostmap()->setConvexPolygonCost(clear_poly,
                                                                costmap_core::FREE_SPACE);
    }

    void MoveBase::wakePlanner()
    {
        // we have slept long enough for rate
        planner_cond_.notify_one();
    }
}
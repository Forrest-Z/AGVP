//
// Created by shydh on 8/9/17.
//

#include <unistd.h>

#include "../base_info/global_methods.h"
#include "include/costmap_2d_ros.h"
#include "include/static_layer.h"
#include "include/obstacle_layer.h"
#include "include/inflation_layer.h"

using namespace Eigen;
using namespace base_info;

namespace costmap_core
{
    Costmap2DROS::Costmap2DROS(transform2::Transformer &tf, cv::Mat *src_mat) :
            layered_costmap_(NULL), plugin_static_(NULL),
            plugin_obstacle_(NULL), plugin_inflation_(NULL), tf_(tf),
            stop_updates_(false), initialized_(true), stopped_(false),
            robot_stopped_(false), map_update_thread_(NULL)
    {
        // make sure that we set the frames appropriately based on the tf_prefix
        // （1）Costmap初始化首先获得全局坐标系和机器人坐标系的转换
        global_frame_ = 0;
        robot_base_frame_ = 1;

        // check if we want a rolling window version of the costmap
        bool rolling_window = false, track_unknown_space = false;

        //（2）加载各个Layer，例如StaticLayer，ObstacleLayer，InflationLayer
        layered_costmap_ = new LayeredCostmap(global_frame_, src_mat,
                                              rolling_window, track_unknown_space);
        x0_ = layered_costmap_->getCostmap()->getSizeInCellsX();
        y0_ = layered_costmap_->getCostmap()->getSizeInCellsY();

        //ROS_INFO("Using plugin \"%s\"", pname.c_str());
        plugin_static_ = new StaticLayer();
        layered_costmap_->addPlugin(plugin_static_);
        plugin_static_->initialize(layered_costmap_, &tf_);

        /*plugin_obstacle_ = new ObstacleLayer();
        layered_costmap_->addPlugin(plugin_obstacle_);
        plugin_obstacle_->initialize(layered_costmap_, &tf_);

        plugin_inflation_ = new InflationLayer();
        layered_costmap_->addPlugin(plugin_inflation_);
        plugin_inflation_->initialize(layered_costmap_, &tf_);*/

        //（3）设置机器人的轮廓
        setUnpaddedRobotFootprint(makeFootprint());

        //（4）实例化了一个Costmap2DPublisher来发布可视化数据

        // create a thread to handle updating the map
        //（5）通过一个movementCB函数不断检测机器人是否在运动
        stop_updates_ = false;
        initialized_ = true;
        stopped_ = false;

        // Create a time r to check if the robot is moving
        robot_stopped_ = false;
        //moment_thread_ =
        //std::thread(base_info::setTimer(1, 0, &(Costmap2DROS::movementCB)));

        //（6）开启动态参数配置服务，服务启动了更新map的线程。
        reconfigureCB();
    }

    Costmap2DROS::~Costmap2DROS()
    {
        map_update_thread_shutdown_ = true;
        if (map_update_thread_ != NULL)
        {
            map_update_thread_->join();
            delete map_update_thread_;
        }

        moment_thread_.join();

        delete layered_costmap_;
        layered_costmap_ = NULL;
        delete plugin_static_;
        plugin_static_ = NULL;
        delete plugin_obstacle_;
        plugin_obstacle_ = NULL;
        delete plugin_inflation_;
        plugin_inflation_ = NULL;
    }

    void Costmap2DROS::setUnpaddedRobotFootprint(const std::vector<bd_Point> &points)
    {
        unpadded_footprint_ = points;
        padded_footprint_ = points;
        padFootprint(padded_footprint_, footprint_padding_);

        layered_costmap_->setFootprint(padded_footprint_);
    }

    void Costmap2DROS::movementCB()
    {
        // don't allow configuration to happen while this check occurs
        // std::recursive_mutex::scoped_lock mcl(configuration_mutex_);
        StampedPose new_pose;

        if (!getRobotPose(new_pose))
        {
            //ROS_WARN_THROTTLE(1.0,
            // "Could not get robot pose, cancelling reconfiguration");
            robot_stopped_ = false;
        }
            // make sure that the robot is not moving
        else
        {
            old_pose_ = new_pose;
            robot_stopped_ = false;
        }
    }

    void Costmap2DROS::mapUpdateLoop(double frequency)
    {
        // the user might not want to run the loop every cycle
        if (frequency == 0.0)
            return;

        while (!map_update_thread_shutdown_)
        {
            updateMap();

            if (layered_costmap_->isInitialized())
            {
                unsigned int x0, y0, xn, yn;
                layered_costmap_->getBounds(&x0, &xn, &y0, &yn);
                updateBounds(x0, xn, y0, yn);
            }
            sleep(50);
        }
    }

    void Costmap2DROS::updateBounds(unsigned int x0, unsigned int xn,
                                    unsigned int y0, unsigned int yn)
    {
        x0_ = std::min(x0, x0_);
        xn_ = std::max(xn, xn_);
        y0_ = std::min(y0, y0_);
        yn_ = std::max(yn, yn_);
    }

    void Costmap2DROS::updateMap()
    {
        if (!stop_updates_)
        {
            // get global pose
            StampedPose pose;
            if (getRobotPose(pose))
            {
                double x = pose.x(), y = pose.y(), yaw = pose.z();

                layered_costmap_->updateMap(x, y, yaw);

                bd_Polygon footprint;
                footprint.frame_id_ = global_frame_;
                footprint.stamp_ = time(NULL);
                transformFootprint(x, y, yaw, padded_footprint_, footprint);
                //footprint_pub_.publish(footprint);

                initialized_ = true;
            }
        }
    }

    void Costmap2DROS::start()
    {
        std::vector<Layer *> *plugins = layered_costmap_->getPlugins();
        // check if we're stopped or just paused
        if (stopped_)
        {
            // if we're stopped we need to re-subscribe to topics
            for (std::vector<Layer *>::iterator plugin = plugins->begin();
                 plugin != plugins->end();
                 ++plugin)
            {
                (*plugin)->activate();
            }
            stopped_ = false;
        }
        stop_updates_ = false;

        // block until the costmap is re-initialized. meaning one update cycle has run
        //while (!initialized_)
            //sleep(100);
    }

    void Costmap2DROS::stop()
    {
        stop_updates_ = true;
        std::vector<Layer *> *plugins = layered_costmap_->getPlugins();
        // unsubscribe from topics
        for (std::vector<Layer *>::iterator plugin = plugins->begin();
             plugin != plugins->end();
             ++plugin)
        {
            (*plugin)->deactivate();
        }
        initialized_ = false;
        stopped_ = true;
    }

    void Costmap2DROS::pause()
    {
        stop_updates_ = true;
        initialized_ = false;
    }

    void Costmap2DROS::resume()
    {
        stop_updates_ = false;

        // block until the costmap is re-initialized.. meaning one update cycle has run
        //ros::Rate r(100.0);
        while (!initialized_)
            sleep(100);
    }


    void Costmap2DROS::resetLayers()
    {
        Costmap2D *top = layered_costmap_->getCostmap();
        top->resetMap(0, 0, top->getSizeInCellsX(), top->getSizeInCellsY());
        std::vector<Layer *> *plugins = layered_costmap_->getPlugins();
        for (std::vector<Layer *>::iterator plugin = plugins->begin();
             plugin != plugins->end();
             ++plugin)
        {
            (*plugin)->reset();
        }
    }

    bool Costmap2DROS::getRobotPose(StampedPose &global_pose) const
    {
        global_pose.setIdentity();

        StampedPose robot_pose;
        robot_pose.setIdentity();

        robot_pose.frame_id_ = robot_base_frame_;
        robot_pose.stamp_ = time(NULL);

        // save time for checking tf delay later
        time_t current_time = time(NULL);

        // get the global pose of the robot
        tf_.transformPose(global_frame_, robot_pose, global_pose);

        // check global_pose timeout
        return current_time - global_pose.stamp_ <= transform_tolerance_;

    }

    void Costmap2DROS::getOrientedFootprint(
            std::vector<bd_Point> &oriented_footprint) const
    {
        StampedPose global_pose;
        if (!getRobotPose(global_pose))
            return;

        transformFootprint(global_pose.x(), global_pose.y(), global_pose.z(),
                           padded_footprint_, oriented_footprint);
    }

    void Costmap2DROS::reconfigureCB()
    {
        transform_tolerance_ = 0.1;
        if (map_update_thread_ != NULL)
        {
            map_update_thread_shutdown_ = true;
            map_update_thread_->join();
            delete map_update_thread_;
        }
        map_update_thread_shutdown_ = false;
        double map_update_frequency = 0.1;

        // find size parameters
        double map_width_meters = 480,
                map_height_meters = 640,
                resolution = 1024,
                origin_x = 0,
                origin_y = 0;

        if (!layered_costmap_->isSizeLocked())
        {
            layered_costmap_->resizeMap((unsigned int) (map_width_meters / resolution),
                                        (unsigned int) (map_height_meters / resolution),
                                        resolution, origin_x, origin_y);
        }

        // If the padding has changed, call setUnpaddedRobotFootprint() to
        // re-apply the padding.
        if (footprint_padding_ != 0.1)
        {
            footprint_padding_ = 0.1;
            setUnpaddedRobotFootprint(unpadded_footprint_);
        }

        map_update_thread_ =
                new std::thread(std::bind(&Costmap2DROS::mapUpdateLoop,
                                          this, map_update_frequency));
    }
}
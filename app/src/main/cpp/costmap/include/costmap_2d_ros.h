//
// Created by shydh on 8/9/17.
//

#ifndef AGVP_COSTMAP_2D_ROS_H
#define AGVP_COSTMAP_2D_ROS_H

#include "costmap_2d.h"
#include "LayeredCostmap.h"
#include "footprint.h"

namespace costmap_core
{
    class Costmap2DROS
    {
    public:
        /**
         * @brief  Constructor for the wrapper
         * @param name The name for this costmap
         * @param tf A reference to a TransformListener
         */
        Costmap2DROS(transform2::Transformer &tf, cv::Mat *src_mat);

        ~Costmap2DROS();

        /**
         * @brief  Subscribes to sensor topics if necessary and starts costmap
         * updates, can be called to restart the costmap after calls to either
         * stop() or pause()
         */
        void start();

        /**
         * @brief  Stops costmap updates and unsubscribes from sensor topics
         */
        void stop();

        /**
         * @brief  Stops the costmap from updating,
         * but sensor data still comes in over the wire
         */
        void pause();

        /**
         * @brief  Resumes costmap updates
         */
        void resume();

        void updateMap();

        /**
         * @brief Reset each individual layer
         */
        void resetLayers();

        /** @brief Same as getLayeredCostmap()->isCurrent(). */
        bool isCurrent()
        {
            return layered_costmap_->isCurrent();
        }

        /**
         * @brief Get the pose of the robot in the global frame of the costmap
         * @param global_pose Will be set to the pose of the robot in the global frame of the costmap
         * @return True if the pose was set successfully, false otherwise
         */
        bool getRobotPose(base_info::StampedPose &global_pose) const;

        /** @brief Return a pointer to the "master" costmap which receives updates from all the layers.
         *
         * Same as calling getLayeredCostmap()->getCostmap(). */
        Costmap2D *getCostmap()
        {
            return layered_costmap_->getCostmap();
        }

        /**
         * @brief  Returns the global frame of the costmap
         * @return The global frame of the costmap
         */
        inline int getGlobalFrameID()
        {
            return global_frame_;
        }

        /**
         * @brief  Returns the local frame of the costmap
         * @return The local frame of the costmap
         */
        inline int getBaseFrameID()
        {
            return robot_base_frame_;
        }

        inline LayeredCostmap *getLayeredCostmap()
        {
            return layered_costmap_;
        }

        /** @brief Returns the current padded footprint as a transform2::Polygon. */
        base_info::bd_Polygon getRobotFootprintPolygon()
        {
            return costmap_core::toPolygon(padded_footprint_);
        }

        /** @brief Return the current footprint of the robot as a vector of points.
         *
         * This version of the footprint is padded by the footprint_padding_
         * distance, set in the rosparam "footprint_padding".
         *
         * The footprint initially comes from the rosparam "footprint" but
         * can be overwritten by dynamic reconfigure or by messages received
         * on the "footprint" topic. */
        inline std::vector<base_info::bd_Point> getRobotFootprint()
        {
            return padded_footprint_;
        }

        /** @brief Return the current unpadded footprint of the robot as a vector of points.
         *
         * This is the raw version of the footprint without padding.
         *
         * The footprint initially comes from the rosparam "footprint" but
         * can be overwritten by dynamic reconfigure or by messages received
         * on the "footprint" topic. */
        inline std::vector<base_info::bd_Point> getUnpaddedRobotFootprint()
        {
            return unpadded_footprint_;
        }

        /**
         * @brief  Build the oriented footprint of the robot at the robot's current pose
         * @param  oriented_footprint Will be filled with the points
         * in the oriented footprint of the robot
         */
        void getOrientedFootprint(std::vector<base_info::bd_Point> &oriented_footprint) const;

        /** @brief Set the footprint of the robot to be the given set of
         * points, padded by footprint_padding.
         *
         * Should be a convex polygon, though this is not enforced.
         *
         * First expands the given polygon by footprint_padding_ and then
         * sets padded_footprint_ and calls
         * layered_costmap_->setFootprint().  Also saves the unpadded
         * footprint, which is available from
         * getUnpaddedRobotFootprint(). */
        void setUnpaddedRobotFootprint(const std::vector<base_info::bd_Point> &points);

    protected:
        LayeredCostmap *layered_costmap_;
        Layer *plugin_static_, *plugin_obstacle_, *plugin_inflation_;

        transform2::Transformer &tf_;  ///< @brief Used for transforming point clouds
        int global_frame_;  ///< @brief The global frame for the costmap
        int robot_base_frame_;  ///< @brief The frame_id of the robot base
        double transform_tolerance_;  ///< timeout before transform errors

    private:

        void movementCB();
        void reconfigureCB();
        void mapUpdateLoop(double frequency);

        /** @brief Include the given bounds in the changed-rectangle. */
        void updateBounds(unsigned int x0, unsigned int xn,
                          unsigned int y0, unsigned int yn);

        bool map_update_thread_shutdown_;
        bool stop_updates_, initialized_, stopped_, robot_stopped_;
        std::thread *map_update_thread_;  ///< @brief A thread for updating the map
        std::thread moment_thread_;

        base_info::StampedPose old_pose_;

        std::recursive_mutex configuration_mutex_;

        bool got_footprint_;
        std::vector<base_info::bd_Point> unpadded_footprint_;
        std::vector<base_info::bd_Point> padded_footprint_;
        float footprint_padding_;
        unsigned int x0_, xn_, y0_, yn_;
    };
}


#endif //AGVP_COSTMAP_2D_ROS_H

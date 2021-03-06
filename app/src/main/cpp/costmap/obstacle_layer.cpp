/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include "include/obstacle_layer.h"

//PLUGINLIB_EXPORT_CLASS(costmap_core::ObstacleLayer, costmap_core::Layer)

using costmap_core::NO_INFORMATION;
using costmap_core::LETHAL_OBSTACLE;
using costmap_core::FREE_SPACE;

using costmap_core::ObservationBuffer;
using costmap_core::Observation;

using namespace base_info;

namespace costmap_core
{

    void ObstacleLayer::onInitialize()
    {
        rolling_window_ = layered_costmap_->isRolling();

        bool track_unknown_space;
        track_unknown_space = layered_costmap_->isTrackingUnknown();

        if (track_unknown_space)
            default_value_ = NO_INFORMATION;
        else
            default_value_ = FREE_SPACE;

        ObstacleLayer::matchSize();
        current_ = true;

        global_frame_ = layered_costmap_->getGlobalFrameID();
        double transform_tolerance = 0.2;

        //while (ss >> source)
        {
            // get the parameters for the specific topic
            time_t observation_keep_time, expected_update_rate;
            double min_obstacle_height, max_obstacle_height;
            int sensor_frame;
            bool clearing, marking;

            sensor_frame = 2;
            observation_keep_time = 0;
            expected_update_rate = 0;
            min_obstacle_height = 0.0;
            max_obstacle_height = 2.0;
            clearing = false;
            marking = true;

            std::string raytrace_range_param_name, obstacle_range_param_name;

            // get the obstacle range for the sensor
            double obstacle_range = 2.5;

            double raytrace_range = 3.0;

            // create an observation buffer
            observation_buffers_.push_back(
                    std::shared_ptr<ObservationBuffer>
                            (new ObservationBuffer(observation_keep_time,
                                                   expected_update_rate,
                                                   min_obstacle_height,
                                                   max_obstacle_height,
                                                   obstacle_range, raytrace_range,
                                                   *tf_, global_frame_,
                                                   sensor_frame, transform_tolerance)));

            // check if we'll add this buffer to our marking observation buffers
            //if (marking)
            marking_buffers_.push_back(observation_buffers_.back());

            // check if we'll also add this buffer to our clearing observation buffers
            //if (clearing)
            clearing_buffers_.push_back(observation_buffers_.back());

            /*ROS_DEBUG(
                    "Created an observation buffer for source %s, topic %s,
                    global frame: %s, "
                            "expected update rate: %.2f, observation persistence: %.2f",
                    source.c_str(), topic.c_str(), global_frame_.c_str(),
                    expected_update_rate,
                    observation_keep_time);*/

            // create a callback for the topic
            //(data_type == "sensor_msgs")
            //laserScanCallback(,observation_buffers_);
        }
    }

    ObstacleLayer::~ObstacleLayer()
    {
    }

    void ObstacleLayer::laserScanCallback(const LaserScan &message,
                                          const std::shared_ptr<ObservationBuffer>
                                          &buffer)
    {
        // project the laser into a point cloud
        /*sensor_msgs::PointCloud2 cloud;
        cloud.header = message.frame_id_;*/

        // project the scan into a point cloud
        /*try
        {
            projector_.transformLaserScanToPointCloud(message->header.frame_id,
            *message, cloud,
                                                      *tf_);
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("High fidelity enabled, but TF returned a transform exception
            to frame %s: %s",
                     global_frame_.c_str(),
                     ex.what());
            projector_.projectLaser(*message, cloud);
        }*/

        // buffer the point cloud
        buffer->lock();
        //buffer->bufferCloud(cloud);
        buffer->unlock();
    }

    void ObstacleLayer::laserScanValidInfCallback(const LaserScan &raw_message,
                                                  const std::shared_ptr<ObservationBuffer>
                                                  &buffer)
    {
        // Filter positive infinities ("Inf"s) to max_range.
        double epsilon = 0.0001;  // a tenth of a millimeter
        LaserScan message = raw_message;
        for (size_t i = 0; i < message.range_max; i++)
        {
            double range = message.ranges[i];
            if (!std::isfinite(range) && range > 0)
            {
                message.ranges[i] = message.range_max - epsilon;
            }
        }
    }

    void ObstacleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                                     double *min_x, double *min_y,
                                     double *max_x, double *max_y)
    {
        if (rolling_window_)
            updateOrigin(robot_x - getSizeInMetersX() / 2,
                         robot_y - getSizeInMetersY() / 2);
        if (!enabled_)
            return;
        useExtraBounds(min_x, min_y, max_x, max_y);

        bool current = true;
        std::vector<Observation> observations, clearing_observations;

        // get the marking observations
        current = current && getMarkingObservations(observations);

        // get the clearing observations
        current = current && getClearingObservations(clearing_observations);

        // update the global current status
        current_ = current;

        // raytrace freespace
        for (unsigned int i = 0; i < clearing_observations.size(); ++i)
        {
            raytraceFreespace(clearing_observations[i], min_x, min_y, max_x, max_y);
        }

        // place the new obstacles into a priority queue...
        // each with a priority of zero to begin with
        for (std::vector<Observation>::const_iterator it = observations.begin();
             it != observations.end(); ++it)
        {
            const Observation &obs = *it;

            //const pcl::PointCloud <pcl::PointXYZ> &cloud = *(obs.cloud_);

            double sq_obstacle_range = obs.obstacle_range_ * obs.obstacle_range_;

            /*for (unsigned int i = 0; i < cloud.points.size(); ++i)
            {
                double px = cloud.points[i].x, py = cloud.points[i].y,
                pz = cloud.points[i].z;

                // if the obstacle is too high or too far away from the
                robot we won't add it
                if (pz > max_obstacle_height_)
                {
                    ROS_DEBUG("The point is too high");
                    continue;
                }

                // compute the squared distance from the hitpoint
                to the pointcloud's origin
                double sq_dist = (px - obs.origin_.x) * (px - obs.origin_.x) +
                                 (py - obs.origin_.y) * (py - obs.origin_.y)
                                 + (pz - obs.origin_.z) * (pz - obs.origin_.z);

                // if the point is far enough away... we won't consider it
                if (sq_dist >= sq_obstacle_range)
                {
                    ROS_DEBUG("The point is too far away");
                    continue;
                }

                // now we need to compute the map coordinates for the observation
                unsigned int mx, my;
                if (!worldToMap(px, py, mx, my))
                {
                    ROS_DEBUG("Computing map coords failed");
                    continue;
                }

                unsigned int index = getIndex(mx, my);
                costmap_[index] = LETHAL_OBSTACLE;
                touch(px, py, min_x, min_y, max_x, max_y);
            }*/
        }

        updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
    }

    void ObstacleLayer::updateFootprint(double robot_x, double robot_y,
                                        double robot_yaw, double *min_x,
                                        double *min_y,
                                        double *max_x, double *max_y)
    {
        if (!footprint_clearing_enabled_) return;
        transformFootprint(robot_x, robot_y, robot_yaw, getFootprint(),
                           transformed_footprint_);

        for (unsigned int i = 0; i < transformed_footprint_.size(); i++)
        {
            touch(transformed_footprint_[i].x(), transformed_footprint_[i].y(),
                  min_x, min_y, max_x,
                  max_y);
        }
    }

    void ObstacleLayer::updateCosts(costmap_core::Costmap2D &master_grid,
                                    int min_i, int min_j,
                                    int max_i, int max_j)
    {
        if (!enabled_)
            return;

        if (footprint_clearing_enabled_)
        {
            bd_Polygon temp;
            temp.setData(transformed_footprint_);
            setConvexPolygonCost(temp, costmap_core::FREE_SPACE);
        }

        switch (combination_method_)
        {
            case 0:  // Overwrite
                updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
                break;
            case 1:  // Maximum
                updateWithMax(master_grid, min_i, min_j, max_i, max_j);
                break;
            default:  // Nothing
                break;
        }
    }

    void ObstacleLayer::clearStaticObservations(bool marking, bool clearing)
    {
        if (marking)
            static_marking_observations_.clear();
        if (clearing)
            static_clearing_observations_.clear();
    }

    bool ObstacleLayer::getMarkingObservations(std::vector<Observation>
                                               &marking_observations) const
    {
        bool current = true;
        // get the marking observations
        for (unsigned int i = 0; i < marking_buffers_.size(); ++i)
        {
            marking_buffers_[i]->lock();
            marking_buffers_[i]->getObservations(marking_observations);
            current = marking_buffers_[i]->isCurrent() && current;
            marking_buffers_[i]->unlock();
        }
        marking_observations.insert(marking_observations.end(),
                                    static_marking_observations_.begin(),
                                    static_marking_observations_.end());
        return current;
    }

    bool ObstacleLayer::getClearingObservations(std::vector<Observation>
                                                &clearing_observations) const
    {
        bool current = true;
        // get the clearing observations
        for (unsigned int i = 0; i < clearing_buffers_.size(); ++i)
        {
            clearing_buffers_[i]->lock();
            clearing_buffers_[i]->getObservations(clearing_observations);
            current = clearing_buffers_[i]->isCurrent() && current;
            clearing_buffers_[i]->unlock();
        }
        clearing_observations.insert(clearing_observations.end(),
                                     static_clearing_observations_.begin(),
                                     static_clearing_observations_.end());
        return current;
    }

    void ObstacleLayer::activate()
    {
        // if we're stopped we need to re-subscribe to topics
        /*for (unsigned int i = 0; i < observation_subscribers_.size(); ++i)
        {
            if (observation_subscribers_[i] != NULL)
                observation_subscribers_[i]->subscribe();
        }*/

        for (unsigned int i = 0; i < observation_buffers_.size(); ++i)
        {
            if (observation_buffers_[i])
                observation_buffers_[i]->resetLastUpdated();
        }
    }

    void ObstacleLayer::deactivate()
    {
        /*for (unsigned int i = 0; i < observation_subscribers_.size(); ++i)
        {
            if (observation_subscribers_[i] != NULL)
                observation_subscribers_[i]->unsubscribe();
        }*/
    }

    void ObstacleLayer::updateRaytraceBounds(double ox, double oy, double wx, double wy,
                                             double range,
                                             double *min_x, double *min_y, double *max_x,
                                             double *max_y)
    {
        double dx = wx - ox, dy = wy - oy;
        double full_distance = hypot(dx, dy);
        double scale = std::min(1.0, range / full_distance);
        double ex = ox + dx * scale, ey = oy + dy * scale;
        touch(ex, ey, min_x, min_y, max_x, max_y);
    }

    void ObstacleLayer::reset()
    {
        deactivate();
        resetMaps();
        current_ = true;
        activate();
    }

    void ObstacleLayer::raytraceFreespace(const costmap_core::Observation
                                          &clearing_observation,
                                          double *min_x, double *min_y, double *max_x,
                                          double *max_y)
    {
        double ox = clearing_observation.origin_.x();
        double oy = clearing_observation.origin_.y();
        //pcl::PointCloud <pcl::PointXYZ> cloud = *(clearing_observation.cloud_);

        // get the map coordinates of the origin of the sensor
        unsigned int x0, y0;
        if (!worldToMap(ox, oy, x0, y0))
        {
            /*ROS_WARN_THROTTLE(
                    1.0, "The origin for the sensor at (%.2f, %.2f)
                    is out of map bounds. "
                            "So, the costmap cannot raytrace for it.",
                    ox, oy);*/
            return;
        }

        // we can pre-compute the enpoints of the map outside of the inner loop...
        // we'll need these later
        double origin_x = origin_x_, origin_y = origin_y_;
        double map_end_x = origin_x + size_x_ * resolution_;
        double map_end_y = origin_y + size_y_ * resolution_;


        touch(ox, oy, min_x, min_y, max_x, max_y);

        // for each point in the cloud, we want to trace a line
        // from the origin and clear obstacles along it
        for (unsigned int i = 0; i < 5; ++i)//cloud.points.size(); ++i)
        {
            double wx = 0.0;//cloud.points[i].x;
            double wy = 0.0;//cloud.points[i].y;

            // now we also need to make sure that the enpoint we're raytracing
            // to isn't off the costmap and scale if necessary
            double a = wx - ox;
            double b = wy - oy;

            // the minimum value to raytrace from is the origin
            if (wx < origin_x)
            {
                double t = (origin_x - ox) / a;
                wx = origin_x;
                wy = oy + b * t;
            }
            if (wy < origin_y)
            {
                double t = (origin_y - oy) / b;
                wx = ox + a * t;
                wy = origin_y;
            }

            // the maximum value to raytrace to is the end of the map
            if (wx > map_end_x)
            {
                double t = (map_end_x - ox) / a;
                wx = map_end_x - .001;
                wy = oy + b * t;
            }
            if (wy > map_end_y)
            {
                double t = (map_end_y - oy) / b;
                wx = ox + a * t;
                wy = map_end_y - .001;
            }

            // now that the vector is scaled correctly...
            // we'll get the map coordinates of its endpoint
            unsigned int x1, y1;

            // check for legality just in case
            if (!worldToMap(wx, wy, x1, y1))
                continue;

            unsigned int cell_raytrace_range =
                    cellDistance(clearing_observation.raytrace_range_);
            MarkCell marker(costmap_, FREE_SPACE);
            // and finally... we can execute our trace to clear obstacles along that line
            raytraceLine(marker, x0, y0, x1, y1, cell_raytrace_range);

            updateRaytraceBounds(ox, oy, wx, wy,
                                 clearing_observation.raytrace_range_,
                                 min_x, min_y, max_x, max_y);
        }
    }

}  // namespace costmap_2d

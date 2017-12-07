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
 *********************************************************************/
#include "include/observation_buffer.h"

using namespace std;
using namespace transform2;

namespace costmap_core
{
    ObservationBuffer::ObservationBuffer(time_t observation_keep_time,
                                         time_t expected_update_rate,
                                         double min_obstacle_height,
                                         double max_obstacle_height,
                                         double obstacle_range,
                                         double raytrace_range, Transformer &tf,
                                         int global_frame,
                                         int sensor_frame, double tf_tolerance) :
            tf_(tf), observation_keep_time_(observation_keep_time),
            expected_update_rate_(expected_update_rate),
            last_updated_(NULL), global_frame_(global_frame),
            sensor_frame_(sensor_frame),
            min_obstacle_height_(min_obstacle_height),
            max_obstacle_height_(max_obstacle_height),
            obstacle_range_(obstacle_range), raytrace_range_(raytrace_range),
            tf_tolerance_(tf_tolerance)
    {
    }

    ObservationBuffer::~ObservationBuffer()
    {
    }

    bool ObservationBuffer::setGlobalFrame(const int new_global_frame)
    {
        time_t transform_time = time(NULL);

        /*if (!tf_.waitForTransform(new_global_frame, global_frame_, transform_time,
                                  ros::Duration(tf_tolerance_),
                                  ros::Duration(0.01), &tf_error))
        {
            ROS_ERROR("Transform between %s and %s with tolerance %.2f failed: %s.",
                      new_global_frame.c_str(),
                      global_frame_.c_str(), tf_tolerance_, tf_error.c_str());
            return false;
        }*/

        list<Observation>::iterator obs_it;
        for (obs_it = observation_list_.begin();
             obs_it != observation_list_.end();
             ++obs_it)
        {
            Observation &obs = *obs_it;

            base_info::PointStamped origin;
            origin.frame_id_ = global_frame_;
            origin.stamp_ = transform_time;
            origin.setData(obs.origin_);

            // we need to transform the origin of the observation
            // to the new global frame
            //tf_.transformPoint(new_global_frame, origin, origin);
            //obs.origin_ = origin;

            // we also need to transform the cloud of the observation
            // to the new global frame
            //pcl_ros::transformPointCloud(new_global_frame,
            // *obs.cloud_, *obs.cloud_, tf_);
        }

        // now we need to update our global_frame member
        global_frame_ = new_global_frame;
        return true;
    }

    // returns a copy of the observations
    void ObservationBuffer::getObservations(vector<Observation> &observations)
    {
        // first... let's make sure that we don't have any stale observations
        purgeStaleObservations();

        // now we'll just copy the observations for the caller
        list<Observation>::iterator obs_it;
        for (obs_it = observation_list_.begin();
             obs_it != observation_list_.end();
             ++obs_it)
        {
            observations.push_back(*obs_it);
        }
    }

    void ObservationBuffer::purgeStaleObservations()
    {
        if (!observation_list_.empty())
        {
            list<Observation>::iterator obs_it = observation_list_.begin();
            // if we're keeping observations for no time...
            // then we'll only keep one observation
            if (observation_keep_time_ == 0)
            {
                observation_list_.erase(++obs_it, observation_list_.end());
                return;
            }

            // otherwise... we'll have to loop through the observations
            // to see which ones are stale
            for (obs_it = observation_list_.begin();
                 obs_it != observation_list_.end();
                 ++obs_it)
            {
                Observation &obs = *obs_it;
                // check if the observation is out of date...
                // and if it is, remove it and those that follow from the list
                /*time_t time_diff =
                        last_updated_ - pcl_conversions::fromPCL(obs.cloud_->header).stamp;
                if ((last_updated_ - pcl_conversions::fromPCL(obs.cloud_->header).stamp) >
                    observation_keep_time_)
                {
                    observation_list_.erase(obs_it, observation_list_.end());
                    return;
                }*/
            }
        }
    }

    bool ObservationBuffer::isCurrent() const
    {
        if (expected_update_rate_ == 0)
            return true;

        bool current = (time(NULL) - last_updated_) <= expected_update_rate_;
        /*if (!current)
        {
            ROS_WARN(
                    "The %s observation buffer has not been updated for %.2f seconds,
                    and it should be updated every %.2f seconds.",
                    topic_name_.c_str(), (time_t::now() - last_updated_).toSec(),
                    expected_update_rate_.toSec());
        }*/
        return current;
    }

    void ObservationBuffer::resetLastUpdated()
    {
        last_updated_ = time(NULL);
    }
}  // namespace costmap_2d


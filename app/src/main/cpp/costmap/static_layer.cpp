/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  Copyright (c) 2015, Fetch Robotics, Inc.
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
#include <unistd.h>
#include "include/static_layer.h"
#include "include/costmap_math.h"

//PLUGINLIB_EXPORT_CLASS(costmap_core::StaticLayer, costmap_core::Layer)

using costmap_core::NO_INFORMATION;
using costmap_core::LETHAL_OBSTACLE;
using costmap_core::FREE_SPACE;

using namespace base_info;

namespace costmap_core
{

    StaticLayer::StaticLayer() : global_frame_(0), map_frame_(1)
    {}

    StaticLayer::~StaticLayer()
    {}

    void StaticLayer::onInitialize()
    {
        current_ = true;

        global_frame_ = layered_costmap_->getGlobalFrameID();

        first_map_only_ = false;

        StaticLayer::reconfigureCB();
    }

    void StaticLayer::matchSize()
    {
        // If we are using rolling costmap, the static map size is
        //   unrelated to the size of the layered costmap
        // 根据master map的尺寸，更新本层的尺寸
        if (!layered_costmap_->isRolling())
        {
            Costmap2D *master = layered_costmap_->getCostmap();
            resizeMap(master->getSizeInCellsX(),
                      master->getSizeInCellsY(),
                      master->getResolution(),
                      master->getOriginX(), master->getOriginY());
        }
    }

    void StaticLayer::activate()
    {
        onInitialize();
    }

    void StaticLayer::deactivate()
    {
        /*map_sub_.shutdown();
        if (subscribe_to_updates_)
            map_update_sub_.shutdown();*/
    }

    void StaticLayer::reset()
    {
        if (first_map_only_)
        {
            has_updated_data_ = true;
        } else
        {
            onInitialize();
        }
    }

    void StaticLayer::updateBounds(double robot_x, double robot_y,
                                   double robot_yaw, double *min_x,
                                   double *min_y,
                                   double *max_x, double *max_y)
    {
        //这里设定为整张static map的大小
        if (!layered_costmap_->isRolling())
            if (!map_received_ || !(has_updated_data_ || has_extra_bounds_))
                return;

        useExtraBounds(min_x, min_y, max_x, max_y);

        double wx, wy;

        mapToWorld(x_, y_, wx, wy);
        *min_x = std::min(wx, *min_x);
        *min_y = std::min(wy, *min_y);

        mapToWorld(x_ + width_, y_ + height_, wx, wy);
        *max_x = std::max(wx, *max_x);
        *max_y = std::max(wy, *max_y);

        has_updated_data_ = false;
    }

    void StaticLayer::updateCosts(costmap_core::Costmap2D &master_grid,
                                  int min_i, int min_j, int max_i,
                                  int max_j)
    {
        if (!map_received_)
            return;

        if (!layered_costmap_->isRolling())
        {
            // if not rolling, the layered costmap (master_grid) has same coordinates
            // as this layer
            if (!use_maximum_)
                updateWithTrueOverwrite(master_grid, min_i, min_j, max_i, max_j);
            else
                updateWithMax(master_grid, min_i, min_j, max_i, max_j);
        } else
        {
            // If rolling window, the master_grid is unlikely to have same coordinates
            // as this layer
            unsigned int mx, my;
            double wx, wy;
            // Might even be in a different frame
            //首先获得map坐标系相对于global坐标系的位置，
            // 这个时候的map坐标系是随着机器人运动而运动的。
            StampedTransform transform;
            transform = tf_->lookupTransform(0);
            // Copy map data given proper transformations
            for (unsigned int i = (unsigned int) min_i; i < max_i; ++i)
            {
                for (unsigned int j = (unsigned int) min_j; j < max_j; ++j)
                {
                    // Convert master_grid coordinates (i,j) into global_frame_(wx,wy)
                    // coordinates
                    layered_costmap_->getCostmap()->mapToWorld(i, j, wx, wy);
                    // Transform from global_frame_ to map_frame_
                    bd_Point p(wx, wy, 0);
                    //p = transform(p);
                    // Set master_grid with cell from map
                    if (worldToMap(p.x(), p.y(), mx, my))
                    {
                        if (!use_maximum_)
                            master_grid.setCost(i, j, getCost(mx, my));
                        else
                            master_grid.setCost(i, j, std::max(getCost(mx, my),
                                                               master_grid.getCost(i, j)));
                    }
                }
            }
        }
    }

    void StaticLayer::reconfigureCB()
    {
        has_updated_data_ = true;
        x_ = y_ = 0;
        width_ = size_x_;
        height_ = size_y_;
    }

}  // namespace costmap_2d

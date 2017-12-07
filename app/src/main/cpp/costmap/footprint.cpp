/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "include/costmap_math.h"
#include "include/footprint.h"

using namespace base_info;
namespace costmap_core
{

    void calculateMinAndMaxDistances(const std::vector<bd_Point> &footprint,
                                     double &min_dist, double &max_dist)
    {
        min_dist = std::numeric_limits<double>::max();
        max_dist = 0.0;

        if (footprint.size() <= 2)
        {
            return;
        }

        for (unsigned int i = 0; i < footprint.size() - 1; ++i)
        {
            // check the distance from the robot center point to the first vertex
            double vertex_dist = distance(0.0, 0.0,
                                          footprint[i].x(),
                                          footprint[i].y());
            double edge_dist = costmap_core::distanceToLine(0.0, 0.0,
                                                            footprint[i].x(),
                                                            footprint[i].y(),
                                                            footprint[i + 1].x(),
                                                            footprint[i + 1].y());
            min_dist = std::min(min_dist, std::min(vertex_dist, edge_dist));
            max_dist = std::max(max_dist, std::max(vertex_dist, edge_dist));
        }

        // we also need to do the last vertex and the first vertex
        double vertex_dist = distance(0.0, 0.0,
                                      footprint.back().x(),
                                      footprint.back().y());
        double edge_dist = costmap_core::distanceToLine(0.0, 0.0,
                                                        footprint.back().x(),
                                                        footprint.back().y(),
                                                        footprint.front().x(),
                                                        footprint.front().y());
        min_dist = std::min(min_dist, std::min(vertex_dist, edge_dist));
        max_dist = std::max(max_dist, std::max(vertex_dist, edge_dist));
    }

    bd_Polygon toPolygon(std::vector<bd_Point> pts)
    {
        bd_Polygon polygon;
        polygon.setData(pts);
        return polygon;
    }

    std::vector<bd_Point> toPointVector(bd_Polygon polygon)
    {
        std::vector<bd_Point> pts;
        pts = polygon;
        return pts;
    }

    void transformFootprint(double x, double y, double theta,
                            const std::vector<bd_Point> &footprint_spec,
                            std::vector<bd_Point> &oriented_footprint)
    {
        // build the oriented footprint at a given location
        oriented_footprint.clear();
        double cos_th = cos(theta);
        double sin_th = sin(theta);
        for (unsigned int i = 0; i < footprint_spec.size(); ++i)
        {
            bd_Point new_pt;
            new_pt.x() = x + (footprint_spec[i].x() * cos_th -
                              footprint_spec[i].y() * sin_th);
            new_pt.y() = y + (footprint_spec[i].x() * sin_th +
                              footprint_spec[i].y() * cos_th);
            oriented_footprint.push_back(new_pt);
        }
    }

    void padFootprint(std::vector<bd_Point> &footprint, double padding)
    {
        // pad footprint in place
        for (unsigned int i = 0; i < footprint.size(); i++)
        {
            bd_Point &pt = footprint[i];
            pt.x() += sign0(pt.x()) * padding;
            pt.y() += sign0(pt.y()) * padding;
        }
    }


    std::vector<bd_Point> makeFootprint()
    {
        std::vector<bd_Point> points;

        // Loop over 16 angles around a circle making a point each time
        bd_Point pt;

        pt.x() = 100;
        pt.y() = 300;
        points.push_back(pt);

        pt.x() = -100;
        pt.y() = 300;
        points.push_back(pt);

        pt.x() = -100;
        pt.y() = -300;
        points.push_back(pt);

        pt.x() = 100;
        pt.y() = -300;
        points.push_back(pt);

        return points;
    }

}  // end namespace costmap_2d

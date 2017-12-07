//
// Created by shydh on 7/2/17.
//

#ifndef AGVP_ORIENTATIONFILTER_H
#define AGVP_ORIENTATIONFILTER_H

#include <vector>
#include "../../base_info/base_datatypes.h"

namespace nav_core
{
    typedef base_info::StampedPose POSE;

    enum OrientationMode { NONE, FORWARD, INTERPOLATE, FORWARDTHENINTERPOLATE };

    class OrientationFilter
    {
    public:
        OrientationFilter() : omode_(NONE) {}


        void processPath(const POSE& start,
                                 std::vector<POSE>& path);

        void pointToNext(std::vector<POSE>& path, int index);
        void interpolate(std::vector<POSE>& path,
                         int start_index, int end_index);

        void setMode(OrientationMode new_mode){ omode_ = new_mode; }
        void setMode(int new_mode){ setMode((OrientationMode) new_mode); }
    protected:
        OrientationMode omode_;
    };

} //end namespace nav_core


#endif //AGVP_ORIENTATIONFILTER_H

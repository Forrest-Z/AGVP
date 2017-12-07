//
// Created by shydh on 8/18/17.
//

#ifndef AGVP_NAV_MSGS_H
#define AGVP_NAV_MSGS_H

#include "base_datatypes.h"

namespace base_info
{
    struct MapMeta
    {
        double resolution;
        int x;
        int y;
        unsigned int width;
        unsigned int height;
        bd_Point origin;
    };

    typedef Stamped<MapMeta> MapMetaData;

    struct OccupancyGridData
    {
        MapMetaData info;
        unsigned int *data;
    };

    typedef Stamped<OccupancyGridData> OccupancyGrid;
}
#endif //AGVP_NAV_MSGS_H

//
// Created by shydh on 8/17/17.
//

#ifndef AGVP_SENSOR_MSGS_H
#define AGVP_SENSOR_MSGS_H

#include "base_datatypes.h"

namespace base_info
{
    struct LaserScanData
    {
        double angle_min;        // start angle of the scan [rad]
        double angle_max;        // end angle of the scan [rad]
        double angle_increment;  // angular distance between measurements [rad]

        double time_increment;   // time between measurements [seconds] - if your scanner
        // is moving, this will be used in interpolating position of 3d points
        double scan_time;        // time between scans [seconds]

        double range_min;        // minimum range value [m]
        double range_max;        // maximum range value [m]

        double *ranges;         // range data [m] (Note: values < range_min or >
        // range_max should be discarded)
        double *intensities;    // intensity data [device-specific units].  If your
        // device does not provide intensities, please leave the array empty.
    };

    typedef Stamped<LaserScanData> LaserScan;
}
#endif //AGVP_SENSOR_MSGS_H

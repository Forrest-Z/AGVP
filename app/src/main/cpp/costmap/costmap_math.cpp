//
// Created by shydh on 8/19/17.
//
#include "include/costmap_math.h"

namespace costmap_core
{
    double distanceToLine(double pX, double pY,
                          double x0, double y0,
                          double x1, double y1)
    {
        double A = pX - x0;
        double B = pY - y0;
        double C = x1 - x0;
        double D = y1 - y0;

        double dot = A * C + B * D;
        double len_sq = C * C + D * D;
        double param = dot / len_sq;

        double xx, yy;

        if (param < 0)
        {
            xx = x0;
            yy = y0;
        }
        else if (param > 1)
        {
            xx = x1;
            yy = y1;
        }
        else
        {
            xx = x0 + param * C;
            yy = y0 + param * D;
        }

        return distance(pX, pY, xx, yy);
    }

    bool intersects(std::vector<base_info::bd_Point> &polygon, float testx, float testy)
    {
        return false;
    }

    bool intersects(std::vector<base_info::bd_Point> &polygon1,
                    std::vector<base_info::bd_Point> &polygon2)
    {
        return false;
    }
}
//
// Created by shydh on 7/2/17.
//

#ifndef AGVP_GRIDPATH_H
#define AGVP_GRIDPATH_H

#include<vector>
#include "Traceback.h"

namespace nav_core
{
    class GridPath : public Traceback
    {
    public:
        GridPath(PotentialCalculator *p_calc) : Traceback(p_calc)
        {}

        bool getPath(double *potential,
                     double start_x, double start_y,
                     double end_x, double end_y,
                     std::vector<std::pair<double, double> > &path);
    };

} //end namespace nav_core


#endif //AGVP_GRIDPATH_H

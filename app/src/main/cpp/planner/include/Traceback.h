//
// Created by shydh on 7/2/17.
//

#ifndef AGVP_TRACEBACK_H
#define AGVP_TRACEBACK_H

#include "PotentialCalculator.h"

namespace nav_core
{
    class Traceback
    {
    public:
        Traceback(PotentialCalculator *p_calc) : p_calc_(p_calc)
        {}

        virtual ~Traceback()
        {

        }

        virtual bool getPath(double *potential,
                             double start_x, double start_y,
                             double end_x, double end_y,
                             std::vector<std::pair<double, double> > &path) = 0;

        virtual void setSize(int xs, int ys)
        {
            xs_ = xs;
            ys_ = ys;
        }

        inline int getIndex(int x, int y)
        {
            return x + y * xs_;
        }

        void setLethalCost(unsigned char lethal_cost)
        {
            lethal_cost_ = lethal_cost;
        }

    protected:
        int xs_, ys_;
        unsigned char lethal_cost_;
        PotentialCalculator *p_calc_;
    };

} //end namespace nav_core
#endif //AGVP_TRACEBACK_H

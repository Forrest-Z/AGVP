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
                             unsigned int start_x, unsigned int start_y,
                             unsigned int end_x, unsigned int end_y,
                             std::vector<std::pair<unsigned int, unsigned int> > &path) = 0;

        virtual void setSize(unsigned int xs, unsigned int ys)
        {
            xs_ = xs;
            ys_ = ys;
        }

        inline unsigned int getIndex(unsigned int x, unsigned int y)
        {
            return x + y * xs_;
        }

        /*void setLethalCost(unsigned char lethal_cost)
        {
            lethal_cost_ = lethal_cost;
        }*/

    protected:
        unsigned int xs_, ys_;
        //unsigned char lethal_cost_;
        PotentialCalculator *p_calc_;
    };

} //end namespace nav_core
#endif //AGVP_TRACEBACK_H

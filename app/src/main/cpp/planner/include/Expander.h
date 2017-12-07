//
// Created by shydh on 7/2/17.
//

#ifndef AGVP_EXPANDER_H
#define AGVP_EXPANDER_H

#define POT_HIGH 1.0e10        // unassigned cell potential

#include "PotentialCalculator.h"

namespace nav_core
{
    class Expander
    {
    public:
        Expander(PotentialCalculator *p_calc, int nx, int ny) :
                unknown_(true),
                lethal_cost_(253), neutral_cost_(50),
                factor_(3.0), p_calc_(p_calc)
        {
            setSize(nx, ny);
        }

        virtual ~Expander()
        {}

        virtual bool calculatePotentials(unsigned char *costs,
                                         double start_x, double start_y,
                                         double end_x, double end_y,
                                         int cycles, double *potential) = 0;

        /**
         * @brief  Sets or resets the size of the map
         * @param nx The x size of the map
         * @param ny The y size of the map
         */
        virtual void setSize(int nx, int ny)
        {
            nx_ = nx;
            ny_ = ny;
            ns_ = nx * ny;
        } /**< sets or resets the size of the map */

        void setLethalCost(unsigned char lethal_cost)
        {
            lethal_cost_ = lethal_cost;
        }

        virtual void setNeutralCost(unsigned char neutral_cost)
        {
            neutral_cost_ = neutral_cost;
        }

        void setFactor(double factor)
        {
            factor_ = factor;
        }

        void setHasUnknown(bool unknown)
        {
            unknown_ = unknown;
        }

        void clearEndpoint(unsigned char *costs, double *potential,
                           int gx, int gy, int s)
        {
            int startCell = toIndex(gx, gy);
            for (int i = -s; i <= s; i++)
            {
                for (int j = -s; j <= s; j++)
                {
                    int n = startCell + i + nx_ * j;
                    if (potential[n] < POT_HIGH)
                        continue;
                    unsigned char c = costs[n] + neutral_cost_;
                    double pot = p_calc_->calculatePotential(potential, c, n);
                    potential[n] = pot;
                }
            }
        }

    protected:
        inline int toIndex(int x, int y)
        {
            return x + nx_ * y;
        }

        int nx_, ny_, ns_; /**< size of grid, in pixels */
        bool unknown_;
        unsigned char lethal_cost_, neutral_cost_;
        int cells_visited_;
        double factor_;
        PotentialCalculator *p_calc_;
    };
} //end namespace nav_core


#endif //AGVP_EXPANDER_H

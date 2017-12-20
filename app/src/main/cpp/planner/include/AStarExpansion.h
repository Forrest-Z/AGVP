//
// Created by shydh on 7/2/17.
//

#ifndef AGVP_ASTAREXPANSION_H
#define AGVP_ASTAREXPANSION_H

#include <vector>
#include <algorithm>
#include "PotentialCalculator.h"
#include "Expander.h"

class Index
{
public:
    Index(unsigned int a, double b)
    {
        i = a;
        cost = b;
    }

    unsigned int i;
    double cost;
};

struct greater1
{
    bool operator()(const Index &a, const Index &b) const
    {
        return a.cost > b.cost;
    }
};

namespace nav_core
{
    class AStarExpansion : public Expander
    {
    public:
        AStarExpansion(PotentialCalculator *p_calc, int nx, int ny);

        bool calculatePotentials(unsigned char *costs,
                                 unsigned int start_x, unsigned int start_y,
                                 unsigned int end_x, unsigned int end_y,
                                 unsigned int cycles,
                                 double *potential);

    private:
        void add(unsigned char *costs, double *potential,
                 double prev_potential, unsigned int next_i,
                 unsigned int end_x, unsigned int end_y);

        std::vector<Index> queue_;
    };

} //end namespace nav_core


#endif //AGVP_ASTAREXPANSION_H

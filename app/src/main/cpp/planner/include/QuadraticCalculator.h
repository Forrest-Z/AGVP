//
// Created by shydh on 7/2/17.
//

#ifndef AGVP_QUADRATICCALCULATOR_H
#define AGVP_QUADRATICCALCULATOR_H


#include "PotentialCalculator.h"

namespace nav_core
{
    class QuadraticCalculator : public PotentialCalculator
    {
    public:
        QuadraticCalculator(int nx, int ny): PotentialCalculator(nx,ny) {}

        float calculatePotential(float* potential,
                                 unsigned char cost, int n, float prev_potential);
    };
} //end namespace nav_core


#endif //AGVP_QUADRATICCALCULATOR_H

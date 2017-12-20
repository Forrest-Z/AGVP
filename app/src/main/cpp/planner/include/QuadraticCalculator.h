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
        QuadraticCalculator(unsigned int nx, unsigned int ny) : PotentialCalculator(nx, ny)
        {}

        double calculatePotential(double *potential,
                                  unsigned char cost, int n, double prev_potential);
    };
} //end namespace nav_core


#endif //AGVP_QUADRATICCALCULATOR_H

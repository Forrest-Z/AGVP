//
// Created by shydh on 7/2/17.
//

#include "include/AStarExpansion.h"
#include "../costmap/include/costmap_2d.h"

namespace nav_core
{
    AStarExpansion::AStarExpansion(PotentialCalculator *p_calc,
                                   int xs, int ys) : Expander(p_calc, xs, ys)
    {
    }

    bool AStarExpansion::calculatePotentials(unsigned char *costs,
                                             unsigned int start_x,
                                             unsigned int start_y,
                                             unsigned int end_x,
                                             unsigned int end_y,
                                             unsigned int cycles, double *potential)
    {
        queue_.clear();
        unsigned int start_i = toIndex(start_x, start_y);

        //push the start point into queue_
        queue_.push_back(Index(start_i, 0));

        //initial all the potential as very large value
        std::fill(potential, potential + ns_, POT_HIGH);
        //initial the start position at potential as 0
        potential[start_i] = 0;

        unsigned int goal_i = toIndex(end_x, end_y);
        unsigned int cycle = 0;

        while (queue_.size() > 0 && cycle < cycles)
        {
            //get the Index that with mini cost
            Index top = queue_[0];
            //build the heap sort
            std::pop_heap(queue_.begin(), queue_.end(), greater1());
            //remove the Index with mini cost
            queue_.pop_back();

            //the Index's i from (i,cost)
            unsigned int i = top.i;
            //stop condition
            if (i == goal_i)
                return true;

            //add the neighborhood 4 points into the search scope
            add(costs, potential, potential[i], i + 1, end_x, end_y);
            add(costs, potential, potential[i], i - 1, end_x, end_y);
            add(costs, potential, potential[i], i + nx_, end_x, end_y);
            add(costs, potential, potential[i], i - nx_, end_x, end_y);

            cycle++;
        }

        return false;
    }

    void AStarExpansion::add(unsigned char *costs,
                             double *potential, double prev_potential,
                             unsigned int next_i, unsigned int end_x, unsigned int end_y)
    {
        if (next_i < 0 || next_i >= ns_)
            return;

        //it means the potential cell has been explored
        if (potential[next_i] < POT_HIGH)
            return;

        //it means this cell is obstacle
        if (costs[next_i] >= lethal_cost_
            && !(unknown_ && costs[next_i] == 1))
            return;

        // compute the next_i cell in potential
        // costs[next_i] + neutral_cost_:original cost + extra cost
        // potential[next_i] means the cost from start to current
        potential[next_i] =
                p_calc_->calculatePotential(potential,
                                            costs[next_i] + neutral_cost_,
                                            next_i, prev_potential);
        int x = next_i % nx_, y = next_i / nx_;

        //曼哈顿距离
        float distance = abs(end_x - x) + abs(end_y - y);

        //distance * neutral_cost_ means the current to the target
        //f(n)=g(n)+h(n)
        queue_.push_back(Index(next_i, potential[next_i] + distance * neutral_cost_));
        std::push_heap(queue_.begin(), queue_.end(), greater1());
    }

} //end namespace nav_core

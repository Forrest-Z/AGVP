//
// Created by shydh on 7/2/17.
//

#include "include/GridPath.h"

using namespace std;
/*首先将goal所在的点的(x,y)塞到path，
 * 然后搜索当前的点的四周的四个临近点，
 * 选取这四个临近点的potential的值最小的点min，
 * 然后把这点塞到path，然后再将当前点更新为min这点，
 * 由于start 点的potential的值是0，所以这是个梯度下降的方法。*/

bool nav_core::GridPath::getPath(double *potential,
                                 unsigned int start_x, unsigned int start_y,
                                 unsigned int end_x, unsigned int end_y,
                                 vector<pair<unsigned int, unsigned int>> &path)
{
    pair<unsigned int, unsigned int> current;
    current.first = end_x;
    current.second = end_y;

    unsigned int start_index = getIndex(start_x, start_y);

    path.push_back(current);
    unsigned int c = 0;
    unsigned int ns = xs_ * ys_;

    while (getIndex(current.first, current.second) != start_index)
    {
        double min_val = 1e10;
        unsigned int min_x = 0, min_y = 0;
        for (int xd = -1; xd <= 1; xd++)
        {
            for (int yd = -1; yd <= 1; yd++)
            {
                if (xd == 0 && yd == 0)
                    continue;
                unsigned int x = current.first + xd, y = current.second + yd;
                unsigned int index = getIndex(x, y);
                if (potential[index] < min_val)
                {
                    min_val = potential[index];
                    min_x = x;
                    min_y = y;
                }
            }
        }

        if (min_x == 0 && min_y == 0)
            return false;

        current.first = min_x;
        current.second = min_y;
        path.push_back(current);

        if (c++ > ns * 4)
        {
            return false;
        }

    }
    return true;
}

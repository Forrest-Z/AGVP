//
// Created by shydh on 7/15/17.
//

#include "include/LayeredCostmap.h"
#include "include/footprint.h"
#include <cstdio>

using std::vector;
using namespace base_info;

namespace costmap_core
{
    LayeredCostmap::LayeredCostmap(int global_frame, cv::Mat *src_mat,
                                   bool rolling_window, bool track_unknown) :
            costmap_(), global_frame_(global_frame),
            rolling_window_(rolling_window),
            initialized_(false), size_locked_(false)
    {
        track_unknown_space_ = true;
        //use_maximum_ = false;

        int temp_lethal_threshold = 100, temp_unknown_cost_value = -1;
        trinary_costmap_ = true;

        lethal_threshold_ = (unsigned char) std::max(std::min(temp_lethal_threshold,
                                                              100), 0);
        unknown_cost_value_ = (unsigned char) temp_unknown_cost_value;

        if (src_mat)
        {
            unsigned int size_x = (unsigned int)src_mat->cols,
                    size_y = (unsigned int)src_mat->rows;

            LOGI("Received a %d X %d map", size_x, size_y);

            // resize costmap if size, resolution or origin do not match
            Costmap2D *master = &costmap_;
            if (!isRolling() && (master->getSizeInCellsX() != size_x ||
                                 master->getSizeInCellsY() != size_y ))
            {
                // Update the size of the layered costmap (and all layers, including this one)
                LOGI("Resizing costmap to %d X %d.", size_x, size_y);
                resizeMap(size_x, size_y, 1024, 0, 0, true);
            }

            unsigned int index = 0;
            unsigned char *tmp = costmap_.getCharMap();
            unsigned char value;
            // initialize the costmap with static data
            for (unsigned int i = 0; i < size_y; ++i)
            {
                for (unsigned int j = 0; j < size_x; ++j)
                {
                    value = src_mat->data[index];
                    tmp[index] = interpretValue(value);
                    ++index;
                }
            }
            LOGI("Initialized the costmap with static data.");
        } else
            costmap_.setDefaultValue(255);
    }

    LayeredCostmap::~LayeredCostmap()
    {
        while (plugins_.size() > 0)
        {
            plugins_.pop_back();
        }
    }

    void LayeredCostmap::resizeMap(unsigned int size_x, unsigned int size_y,
                                   double resolution,
                                   double origin_x,
                                   double origin_y, bool size_locked)
    {
        size_locked_ = size_locked;
        costmap_.resizeMap(size_x, size_y, resolution, origin_x, origin_y);
        for (vector<Layer *>::iterator plugin = plugins_.begin();
             plugin != plugins_.end();
             ++plugin)
            (*plugin)->matchSize();
        LOGI("Initialized the costmap plugins.");
    }

    void LayeredCostmap::updateMap(double robot_x, double robot_y, double robot_yaw)
    {
        // Lock for the remainder of this function, some plugins (e.g. VoxelLayer)
        // implement thread unsafe updateBounds() functions.
        std::unique_lock<Costmap2D::mutex_t> lock(*(costmap_.getMutex()));

        // if we're using a rolling buffer costmap...
        // we need to update the origin using the robot's position
        if (rolling_window_)
        {
            double new_origin_x = robot_x - costmap_.getSizeInMetersX() / 2;
            double new_origin_y = robot_y - costmap_.getSizeInMetersY() / 2;
            costmap_.updateOrigin(new_origin_x, new_origin_y);
        }

        if (plugins_.size() == 0)
            return;

        minx_ = miny_ = 1e30;
        maxx_ = maxy_ = -1e30;

        for (vector<Layer *>::iterator plugin = plugins_.begin();
             plugin != plugins_.end();
             ++plugin)
            (*plugin)->updateBounds(robot_x, robot_y,
                                    robot_yaw, &minx_, &miny_, &maxx_, &maxy_);

        int x0, xn, y0, yn;
        costmap_.worldToMapEnforceBounds(minx_, miny_, x0, y0);
        costmap_.worldToMapEnforceBounds(maxx_, maxy_, xn, yn);

        x0 = std::max(0, x0);
        xn = std::min((int) costmap_.getSizeInCellsX(), xn + 1);
        y0 = std::max(0, y0);
        yn = std::min((int) costmap_.getSizeInCellsY(), yn + 1);

        //ROS_DEBUG("Updating area x: [%d, %d] y: [%d, %d]", x0, xn, y0, yn);

        if (xn < x0 || yn < y0)
            return;

        costmap_.resetMap((unsigned int) x0, (unsigned int) y0,
                          (unsigned int) xn, (unsigned int) yn);
        for (vector<Layer *>::iterator plugin = plugins_.begin();
             plugin != plugins_.end();
             ++plugin)
            (*plugin)->updateCosts(costmap_, x0, y0, xn, yn);

        bx0_ = (unsigned int) x0;
        bxn_ = (unsigned int) xn;
        by0_ = (unsigned int) y0;
        byn_ = (unsigned int) yn;

        initialized_ = true;
    }

    bool LayeredCostmap::isCurrent()
    {
        current_ = true;
        for (vector<Layer *>::iterator plugin = plugins_.begin();
             plugin != plugins_.end();
             ++plugin)
        {
            current_ = current_ && (*plugin)->isCurrent();
        }
        return current_;
    }

    void LayeredCostmap::setFootprint(const std::vector<bd_Point> &footprint_spec)
    {
        footprint_ = footprint_spec;
        costmap_core::calculateMinAndMaxDistances(footprint_spec, inscribed_radius_,
                                                  circumscribed_radius_);

        for (vector<Layer *>::iterator plugin = plugins_.begin();
             plugin != plugins_.end();
             ++plugin)
        {
            (*plugin)->onFootprintChanged();
        }
    }

    unsigned char LayeredCostmap::interpretValue(unsigned char value)
    {
        // check if the static value is above the unknown or lethal thresholds
        // 将参数根据阈值，设定为NO_INFORMATION
        // FREE_SPACE
        // LETHAL_OBSTACLE
        // FREE_SPACE 或者其他值。
        if (track_unknown_space_ && value == unknown_cost_value_)
            return NO_INFORMATION;
        else if (!track_unknown_space_ && value == unknown_cost_value_)
            return FREE_SPACE;
        else if (value >= lethal_threshold_)
            return LETHAL_OBSTACLE;
        else if (trinary_costmap_)
            return FREE_SPACE;

        double scale = (double) value / lethal_threshold_;
        return (unsigned char) scale * LETHAL_OBSTACLE;
    }

    /*************************************************************************/

    Layer::Layer() :
            layered_costmap_(NULL),
            current_(false),
            enabled_(false),
            tf_(NULL)
    {

    }

    void Layer::initialize(LayeredCostmap *parent, transform2::Transformer *tf)
    {
        layered_costmap_ = parent;
        tf_ = tf;
        onInitialize();
    }

    const std::vector<bd_Point> &Layer::getFootprint() const
    {
        return layered_costmap_->getFootprint();
    }
}

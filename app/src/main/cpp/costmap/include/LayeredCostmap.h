//
// Created by shydh on 7/15/17.
//

#ifndef AGVP_LAYEREDCOSTMAP_H
#define AGVP_LAYEREDCOSTMAP_H

#include <vector>
#include <string>
#include "costmap_2d.h"

namespace costmap_core
{
    class LayeredCostmap;

    class Layer
    {
    public:
        Layer();

        virtual void initialize(LayeredCostmap *parent, transform2::Transformer *tf);

        /**
         * @brief This is called by the LayeredCostmap to poll this plugin as to how
         *        much of the costmap it needs to update. Each layer can increase
         *        the size of this bounds.
         *
         * For more details, see "Layered Costmaps for Context-Sensitive Navigation",
         * by Lu et. Al, IROS 2014.
         */
        virtual void updateBounds(double robot_x, double robot_y,
                                  double robot_yaw, double *min_x, double *min_y,
                                  double *max_x, double *max_y)
        {}

        /**
         * @brief Actually update the underlying costmap, only within the bounds
         *        calculated during UpdateBounds().
         */
        virtual void updateCosts(Costmap2D &master_grid,
                                 int min_i, int min_j, int max_i, int max_j)
        {}

        /** @brief Stop publishers. */
        virtual void deactivate()
        {}

        /** @brief Restart publishers if they've been stopped. */
        virtual void activate()
        {}

        virtual void reset()
        {}

        virtual ~Layer()
        {}

        /**
         * @brief Check to make sure all the data in the layer is up to date.
         *        If the layer is not up to date, then it may be unsafe to
         *        plan using the data from this layer, and the planner may
         *        need to know.
         *
         *        A layer's current state should be managed by the protected
         *        variable current_.
         * @return Whether the data in the layer is up to date.
         */
        inline bool isCurrent() const
        {
            return current_;
        }

        /** @brief Implement this to make this layer match the size of the parent costmap. */
        virtual void matchSize()
        {}

        /** @brief Convenience function for layered_costmap_->getFootprint(). */
        const std::vector<base_info::bd_Point> &getFootprint() const;

        /** @brief LayeredCostmap calls this whenever the footprint there
         * changes (via LayeredCostmap::setFootprint()).  Override to be
         * notified of changes to the robot's footprint. */
        virtual void onFootprintChanged()
        {}

    protected:
        /** @brief This is called at the end of initialize().  Override to
         * implement subclass-specific initialization.
         *
         * tf_, name_, and layered_costmap_ will all be set already when this is called. */
        virtual void onInitialize()
        {};

        LayeredCostmap *layered_costmap_;
        bool current_;
        ///< Currently this var is managed by subclasses.
        ///< TODO: make this managed by this class and/or container class.
        bool enabled_;
        transform2::Transformer *tf_;

        //private:
        //std::vector<geometry_msgs::Point> footprint_spec_;
    };

    /**
    * @class LayeredCostmap
    * @brief Instantiates different layer plugins and aggregates them into one score
    */
    class LayeredCostmap
    {
    public:
        /**
         * @brief  Constructor for a costmap
         */
        LayeredCostmap(int global_frame, cv::Mat *src_mat,
                       bool rolling_window, bool track_unknown);

        /**
         * @brief  Destructor
         */
        ~LayeredCostmap();

        /**
         * @brief  Update the underlying costmap with new data.
         * If you want to update the map outside of the update loop that runs,
         * you can call this.
         */
        void updateMap(double robot_x, double robot_y, double robot_yaw);

        inline int getGlobalFrameID() const
        {
            return global_frame_;
        }

        /**
         * @brief 就是给class costmap_2d 的 costmap_ 成员的大小重新做分配。
         * 然后根据plugin对每一层的地图调用其父类Costmap2D成员的initial 方法，
         * 实际效果就是将plugin所指向的每一层地图的大小都设置为
         * 和LayeredCostmap::costmap_ 数据成员一样的空间大小。*/
        void resizeMap(unsigned int size_x, unsigned int size_y,
                       double resolution, double origin_x,
                       double origin_y,
                       bool size_locked = false);

        inline void getUpdatedBounds(double &minx, double &miny,
                                     double &maxx, double &maxy)
        {
            minx = minx_;
            miny = miny_;
            maxx = maxx_;
            maxy = maxy_;
        }

        bool isCurrent();

        inline Costmap2D *getCostmap()
        {
            return &costmap_;
        }

        inline bool isRolling()
        {
            return rolling_window_;
        }

        inline bool isTrackingUnknown()
        {
            return costmap_.getDefaultValue() == 1;
        }

        inline std::vector<Layer *> *getPlugins()
        {
            return &plugins_;
        }

        inline void addPlugin(Layer *plugin)
        {
            plugins_.push_back(plugin);
        }

        inline bool isSizeLocked()
        {
            return size_locked_;
        }

        inline void getBounds(unsigned int *x0, unsigned int *xn,
                              unsigned int *y0, unsigned int *yn)
        {
            *x0 = bx0_;
            *xn = bxn_;
            *y0 = by0_;
            *yn = byn_;
        }

        inline bool isInitialized()
        {
            return initialized_;
        }

        /** @brief Updates the stored footprint, updates the circumscribed
         * and inscribed radii, and calls onFootprintChanged() in all
         * layers. */
        void setFootprint(const std::vector<base_info::bd_Point> &footprint_spec);

        /** @brief Returns the latest footprint stored with setFootprint(). */
        inline const std::vector<base_info::bd_Point> &getFootprint()
        { return footprint_; }

        /** @brief The radius of a circle centered at the origin of the
         * robot which just surrounds all points on the robot's
         * footprint.
         *
         * This is updated by setFootprint(). */
        inline double getCircumscribedRadius()
        { return circumscribed_radius_; }

        /** @brief The radius of a circle centered at the origin of the
         * robot which is just within all points and edges of the robot's
         * footprint.
         *
         * This is updated by setFootprint(). */
        inline double getInscribedRadius()
        { return inscribed_radius_; }

    private:

        unsigned char interpretValue(unsigned char value);

        Costmap2D costmap_;
        int global_frame_;

        /// < @brief Whether or not the costmap should roll with the robot
        bool rolling_window_;

        bool current_;
        double minx_, miny_, maxx_, maxy_;
        unsigned int bx0_, bxn_, by0_, byn_;

        std::vector<Layer *> plugins_;

        bool initialized_;
        bool size_locked_;
        double circumscribed_radius_, inscribed_radius_;
        std::vector<base_info::bd_Point> footprint_;

        bool track_unknown_space_;
        bool trinary_costmap_;

        unsigned char lethal_threshold_, unknown_cost_value_;
    };

}  // namespace costmap_core


#endif //AGVP_LAYEREDCOSTMAP_H

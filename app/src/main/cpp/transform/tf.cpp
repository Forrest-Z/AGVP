//
// Created by shydh on 6/19/17.
//

#include "tf.h"

using namespace base_info;

namespace transform2
{
    Transformer::Transformer(time_t cache_time_) :
            tf2_buffer_(6)
    {
        StampedTransform transform = StampedTransform();
        Eigen::Isometry3d temp = Eigen::Isometry3d::Identity();;
        //0 world to base_link, 1 base_link to world
        //2 odom to base_link, 3 base_link to odom
        //4 sensor to base_link, 5 base_link to sensor
        for (int i = 0; i < 6; i++)
        {
            transform.frame_id_ = i;
            transform.child_frame_id_ = i + 1;
            transform.stamp_ = cache_time_;
            transform.setData(temp);
            tf2_buffer_.push_back(transform);
        }
    }

    Transformer::~Transformer()
    {}

    void Transformer::clear()
    {
        tf2_buffer_.clear();
    }

    void Transformer::setTransform(const StampedTransform &transform,
                                   const int transform_id)
    {
        tf2_buffer_[transform_id] = transform;
    }

    StampedTransform Transformer::lookupTransform(const int &transform_id) const
    {
        return tf2_buffer_[transform_id];
    }

    void Transformer::transformPose(const int &transform_id,
                                    const StampedPose &stamped_in,
                                    StampedPose &stamped_out) const
    {
        StampedTransform transform;
        transform = lookupTransform(transform_id);
        stamped_out.setData(transform * stamped_in);
        stamped_out.stamp_ = transform.stamp_;
        stamped_out.frame_id_ = transform_id;
    }
}

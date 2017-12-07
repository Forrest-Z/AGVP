//
// Created by shydh on 7/11/17.
//

#ifndef AGVP_BASE_DATATYPES_H
#define AGVP_BASE_DATATYPES_H

#include <string>

#include "../Eigen/Dense"
#include "../Eigen/Geometry"

namespace base_info
{
    static const double QUATERNION_TOLERANCE = 0.1f;

    /** \brief The data type which will be cross compatable with geometry_msgs
        * This is the tf datatype equivilant of a MessageStamped */
    template<typename T>
    class Stamped : public T
    {
    public:
        time_t stamp_; ///< The timestamp associated with this data
        int frame_id_; ///< The frame_id associated this data

        ///< The frame_id of the coordinate frame this transform defines
        int child_frame_id_;

        /** Default constructor */
        //Default constructor used only for preallocation
        Stamped() : frame_id_(0),
                    child_frame_id_(1)
        {};

        /** Full constructor */
        Stamped(const T &input,
                const time_t &timestamp,
                const int &frame_id,
                const int &child_frame_id) :
                T(input), stamp_(timestamp), frame_id_(frame_id),
                child_frame_id_(child_frame_id)
        {};

        /** Set the data element */
        void setData(const T &input)
        {
            *static_cast<T *>(this) = input;
        };
    };

/** \brief Comparison Operator for Stamped datatypes */
    template<typename T>
    bool operator==(const Stamped<T> &a, const Stamped<T> &b)
    {
        return a.frame_id_ == b.frame_id_ && a.stamp_ == b.stamp_ &&
               a.child_frame_id == b.child_frame_id_ &&
               static_cast<const T &>(a) == static_cast<const T &>(b);
    };

    typedef Eigen::Vector3d bd_Point;
    typedef Stamped<bd_Point> PointStamped;
    typedef Eigen::Vector3d bd_Velocity;
    typedef Stamped<std::vector<bd_Point>> bd_Polygon;
    typedef Stamped<Eigen::Vector3d> StampedPose;
    typedef Stamped<Eigen::Isometry3d> StampedTransform;

}
#endif //AGVP_BASE_DATATYPES_H

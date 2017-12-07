//
// Created by shydh on 6/19/17.
//

#ifndef AGVP_TF_H
#define AGVP_TF_H

#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <sstream>
#include <map>

#include "../base_info/base_datatypes.h"

namespace transform2
{
    enum ErrorValues
    {
        NO_ERROR = 0,
        LOOKUP_ERROR,
        CONNECTIVITY_ERROR,
        EXTRAPOLATION_ERROR
    };

    class Transformer
    {
        /** \brief A Class which provides coordinate transforms
		between any two frames in a system.
	 *
	 * This class provides a simple interface to allow recording and lookup of
	 * relationships between arbitrary frames of the system.
	 *
	 * libTF assumes that there is a tree of coordinate frame transforms
		which define the relationship between all coordinate frames.
	 * For example your typical robot would have a transform
		from global to real world.  And then from base to hand, and from base to head.
	 * But Base to Hand really is composed of base to shoulder to elbow
		to wrist to hand.
	 * libTF is designed to take care of all the intermediate steps for you.
	 *
	 * Internal Representation
	 * libTF will store frames with the parameters necessary
		for generating the transform into that frame
		from it's parent and a reference to the parent frame.
	 * Frames are designated using an std::string
	 * 0 is a frame without a parent (the top of a tree)
	 * The positions of frames over time must be pushed in.
	 *
	 * All function calls which pass frame ids can potentially
		throw the exception tf::LookupException
	 */
    public:
        /************* Constants ***********************/
        //!< The maximum number of time to recurse before assuming the tree has a loop.
        static const unsigned int MAX_GRAPH_DEPTH = 100UL;
        //!< 10.0 is the default amount of time to cache data in seconds,
        //set in cpp file.
        static const double DEFAULT_CACHE_TIME;
        //!< The default amount of time to extrapolate
        //deprecated since integration with tf2
        static const int64_t DEFAULT_MAX_EXTRAPOLATION_DISTANCE = 0ULL;


        /** Constructor
         * \param interpolating Unused, legacy always true
         * \param cache_time How long to keep a history of transforms in nanoseconds
         *
         */
        Transformer(time_t cache_time_);

        virtual ~Transformer();

        /** \brief Clear all data */
        void clear();

        /** \brief Add transform information to the tf data structure
         * \param transform The transform to store
         * \param authority The source of the information for this transform
         * returns true unless an error occured
         */
        void setTransform(const base_info::StampedTransform &transform,
                          const int transform_id);

        /*********** Accessors *************/

        /** \brief Get the transform between two frames by frame ID.
         * \param target_frame The frame to which data should be transformed
         * \param source_frame The frame where the data originated
         * \param time The time at which the value of the transform is desired.
         * (0 will get the latest)
         * \param transform The transform reference to fill.
         *
         * Possible exceptions tf::LookupException, tf::ConnectivityException,
         * tf::MaxDepthException, tf::ExtrapolationException
         */
        base_info::StampedTransform lookupTransform(const int &transform_id) const;

        /** \brief Transform a Stamped Pose into the target frame
         * This can throw anything a lookupTransform can throw
         as well as tf::InvalidArgument.*/
        void transformPose(const int &transform_id,
                           const base_info::StampedPose &stamped_in,
                           base_info::StampedPose &stamped_out) const;

    protected:
        std::vector<base_info::StampedTransform> tf2_buffer_;

    };
};


#endif //AGVP_TF_H

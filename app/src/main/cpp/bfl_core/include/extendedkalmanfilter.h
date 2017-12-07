// $Id: extendedkalmanfilter.h 29830 2009-01-14 15:10:41Z kgadeyne $
// Copyright (C) 2002 Klaas Gadeyne <first dot last at gmail dot com>
//                    Wim Meeussen  <wim dot meeussen at mech dot kuleuven dot be>
//                    Tinne De Laet <tinne dot delaet at mech dot kuleuven dot be>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation; either version 2.1 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
//

#ifndef __EXTENDED_KALMAN_FILTER__
#define __EXTENDED_KALMAN_FILTER__

#include "kalmanfilter.h"
#include "conditionalpdf.h"
#include "gaussian.h"

# include <map>

namespace bfl_core
{

/** This is a class implementing the Kalman Filter (KF) class for
    Extended Kalman Filters.

    The System- and MeasurementUpdate equasions are not linear, and
    will be approximated by local linearisations.

    @see KalmanFilter
    @note that if the system/measurement model that you pass to the
    update calls is not analytical with additive gaussian noise, you
    will get rubbish results
*/
    class ExtendedKalmanFilter : public KalmanFilter
    {
    public:
        /** Constructor
            @pre you created the prior
            @param prior pointer to the Gaussian prior density
        */
        ExtendedKalmanFilter(Gaussian *prior);

        /// Destructor
        virtual ~ExtendedKalmanFilter();

        /// Function to allocate memory needed during the measurement update,
        //  For realtime use, this function should be called before calling measUpdate
        /*  @param vector containing the dimension of the measurement models which are
            going to be used
        */
        void AllocateMeasModelExt(const std::vector<unsigned int> &meas_dimensions);

        /// Function to allocate memory needed during the measurement update
        //  For realtime use, this function should be called before calling measUpdate
        /*  @param dimension of the measurement models which is
            going to be used
        */
        void AllocateMeasModelExt(const unsigned int &meas_dimensions);

    private:
        struct MeasUpdateVariablesExt
        {
            Eigen::Matrix3d _R;
            Eigen::Matrix3d _H;
            Eigen::Vector3d _Z;

            MeasUpdateVariablesExt()
            {}

        }; //struct

    protected:
        virtual void SysUpdate(SystemModel<Eigen::Vector3d> *const sysmodel,
                               const Eigen::Vector3d &u);

        virtual void MeasUpdate(MeasurementModel<Eigen::Vector3d,
                Eigen::Vector3d> *const measmodel,
                                const Eigen::Vector3d &z,
                                const Eigen::Vector3d &s);

        // variables to avoid allocation on the heap
        Eigen::Vector3d _x;
        Eigen::Vector3d _J;
        Eigen::Matrix3d _F;
        Eigen::Matrix3d _Q;
        std::map<unsigned int, MeasUpdateVariablesExt>
                _mapMeasUpdateVariablesExt;
        std::map<unsigned int, MeasUpdateVariablesExt>::iterator
                _mapMeasUpdateVariablesExt_it;


    };  // class

} // End namespace BFL

#endif // __EXTENDED_KALMAN_FILTER__

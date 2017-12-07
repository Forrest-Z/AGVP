// $Id: gaussian.h 29890 2009-02-02 10:22:01Z tdelaet $
// Copyright (C) 2002 Klaas Gadeyne <first dot last at gmail dot com>
// Copyright (C) 2008 Tinne De Laet <first dot last at mech dot kuleuven dot be>
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
#ifndef GAUSSIAN_H
#define GAUSSIAN_H

#include "pdfunction.h"
#include "sample.h"
#include "../../Eigen/Cholesky"

namespace bfl_core
{
/// Class representing Gaussian (or normal density)
    class Gaussian : public Pdf<Eigen::Vector3d>
    {
    private:
        Eigen::Vector3d _Mu;
        Eigen::Matrix3d _Sigma;

        // variables to avoid recalculation of inverse
        bool _Sigma_changed;
        Eigen::Matrix3d _Sigma_inverse;
        double _sqrt_pow;
        Eigen::Vector3d _diff; //needed in doubleGet
        Eigen::Vector3d _tempColumn; //needed in doubleGet
        // variables to avoid allocation on the heap during resampling
        Eigen::Vector3d _samples;
        Eigen::Vector3d _sampleValue;
        Eigen::Matrix3d _Low_triangle;

    public:
        /// Constructor
        /**
         @param Mu Mean Vector of the Gaussian
         @param Sigma Covariance Matrix of the Gaussian
          */
        Gaussian(const Eigen::Vector3d &Mu,
                 const Eigen::Matrix3d &Sigma);

        /// constructor with only dimensions or nothing
        Gaussian(int dimension = 0);

        /// Default Copy Constructor will do

        /// Destructor
        virtual ~Gaussian();

        ///Clone function
        Gaussian *Clone() const;

        // Redefinition of pure virtuals
        virtual double ProbabilityGet(const Eigen::Vector3d &input);

        bool SampleFrom(std::vector<Sample<Eigen::Vector3d>> &list_samples,
                        const int num_samples,
                        int method = DEFAULT);

        virtual bool SampleFrom(Sample<Eigen::Vector3d> &one_sample,
                                int method = DEFAULT);

        virtual Eigen::Vector3d ExpectedValueGet() const;

        virtual Eigen::Matrix3d CovarianceGet();

        // For a Gaussian this should be possible
        /// Set the Expected Value
        /** Set the Expected Value
          @param mu The new Expected Value
          */
        void ExpectedValueSet (const Eigen::Vector3d& mu);

        /// Set the Covariance Matrix
        /** Set the Covariance Matrix
          @param cov The new Covariance matrix
          */
        void CovarianceSet (const Eigen::Matrix3d& cov);
    };

} // end namespace
#endif

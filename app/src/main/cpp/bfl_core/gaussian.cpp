// $Id: gaussian.cpp 29890 2009-02-02 10:22:01Z tdelaet $
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
#include "include/gaussian.h"

using namespace Eigen;
using namespace std;

namespace bfl_core
{

    Gaussian::Gaussian(const Vector3d &m, const Matrix3d &s)
    {
        // check if inputs are consistent
        _Mu = m;
        _Sigma = s;
        _Sigma_changed = true;
    }

    Gaussian::Gaussian(int dimension)
    {
        _Sigma_changed = true;
    }

    Gaussian::~Gaussian()
    {}

//Clone function
    Gaussian *Gaussian::Clone() const
    {
        return new Gaussian(*this);
    }

    double Gaussian::ProbabilityGet(const Vector3d &input)
    {
        // only calculate these variables if sigma has changed
        if (_Sigma_changed)
        {
            _Sigma_changed = false;
            _Sigma_inverse = _Sigma.inverse();
            _sqrt_pow = 1 / sqrt(pow(M_PI * 2,
                                     (double) DimensionGet()) * _Sigma.determinant());
        }

        _diff = input;
        _diff -= _Mu;
        //_Sigma_inverse.multiply(_diff, _tempColumn);
        _tempColumn = _Sigma_inverse * _diff;
        double temp = _diff.transpose().x() * _tempColumn.x() +
                      _diff.transpose().y() * _tempColumn.y() +
                      _diff.transpose().z() * _tempColumn.z();
        //double temp = _diff.transpose() * (_Sigma_inverse * _diff);
        double result = exp(-0.5 * temp) * _sqrt_pow;
        return result;
    }

// Redefined for optimal performance.  Eg. do Cholesky decomposition
// only once when drawing multiple samples at once!
// See method below for more info regarding the algorithms
    bool Gaussian::SampleFrom(vector <Sample<Vector3d>> &list_samples,
                              const int num_samples,
                              int method)
    {
        // will break real-timeness if list_samples.size()!=num_samples
        list_samples.resize((unsigned long)num_samples);
        vector < Sample < Vector3d > > ::iterator
        rit = list_samples.begin();
        switch (method)
        {
            case DEFAULT: // Cholesky Sampling
            case CHOLESKY:
            {
                LLT<MatrixXd> lltOfA(_Sigma);
                _Low_triangle = lltOfA.matrixL();
                //bool result = _Sigma.cholesky_semidefinite(_Low_triangle);

                default_random_engine generator;
                normal_distribution<double> distribution(0.0,1.0);

                while (rit != list_samples.end())
                {
                    for (unsigned int j = 1; j < DimensionGet() + 1; j++)
                        _samples(j) = distribution(generator);
                    _sampleValue = _Low_triangle * _samples;
                    _sampleValue += this->_Mu;
                    rit->ValueSet(_sampleValue);
                    rit++;
                }
                return true;
            }
            case BOXMULLER: // Implement box-muller here
                // Only for univariate distributions.
                return false;
            default:
                return false;
        }
    }


    bool Gaussian::SampleFrom(Sample<Vector3d> &one_sample, int method)
    {
        /*  Exact i.i.d. samples from a Gaussian can be drawn in several
        ways:
        - if the DimensionGet() = 1 or 2 (and the 2 variables are
        independant), we can use inversion sampling (Box-Muller
        method)
            - For larger dimensions, we use can use the Cholesky method or
        an approached based on conditional distributions.
        (see ripley87, P.98 (bibtex below)).  The Cholesky method is
        generally preferred and the only one implemented for now.
        */
        switch (method)
        {
            case DEFAULT: // Cholesky Sampling, see eg.
            case CHOLESKY: // Cholesky Sampling, see eg.
                /*
              @Book{		  ripley87,
              author	= {Ripley, Brian D.},
              title		= {Stochastic Simulation},
              publisher	= {John Wiley and Sons},
              year		= {1987},
              annote	= {ISBN 0271-6356, WBIB 1 519.245}
              }
              p.98
            */
            {
                LLT<MatrixXd> lltOfA(_Sigma);
                _Low_triangle = lltOfA.matrixL();
                //bool result = _Sigma.cholesky_semidefinite(_Low_triangle);

                /* For now we keep it simple, and use the scythe library
                 (although wrapped) with the uRNG that it uses itself only */
                /* Sample Gaussian._dimension samples from univariate
                 gaussian This could be done using several available
                 libraries, combined with different uniform RNG.  Both the
                 library to be used and the uRNG could be implemented as
                 #ifdef conditions, although I'm sure there must exist a
                 cleaner way to implement this!
              */
                default_random_engine generator;
                normal_distribution<double> distribution(0.0,1.0);

                for (unsigned int j = 0; j < DimensionGet() + 1; j++)
                    _samples[j] = distribution(generator);
                _sampleValue = (_Low_triangle * _samples) + this->_Mu;
                one_sample.ValueSet(_sampleValue);
                return true;
            }
            case BOXMULLER: // Implement box-muller here
                // Only for univariate distributions.
                return false;
            default:
                return false;
        }
    }

    Vector3d Gaussian::ExpectedValueGet() const
    {
        return _Mu;
    }

    Matrix3d Gaussian::CovarianceGet()
    {
        return _Sigma;
    }

    void Gaussian::ExpectedValueSet(const Vector3d &mu)
    {
        _Mu = mu;
    }

    void Gaussian::CovarianceSet(const Matrix3d &cov)
    {
        _Sigma = cov;
        _Sigma_changed = true;
    }

} // End namespace BFL

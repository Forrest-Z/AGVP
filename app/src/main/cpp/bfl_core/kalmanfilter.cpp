//
// Created by shydh on 7/22/17.
//

#include "include/kalmanfilter.h"

using namespace Eigen;
using namespace std;

namespace bfl_core
{
    KalmanFilter::KalmanFilter(Gaussian * prior)
            : Filter<Vector3d,Vector3d>(prior)
            , _Mu_new()
            , _Sigma_new()
            , _Sigma_temp()
            , _Sigma_temp_par()
    {
        // create posterior dencity
        _post = new Gaussian(*prior);
    }

    KalmanFilter::~KalmanFilter()
    {
        delete _post;
    }

    Gaussian *KalmanFilter::PostGet()
    {
        return (Gaussian*)Filter<Vector3d,Vector3d>::PostGet();
    }

    void KalmanFilter::AllocateMeasModel(const std::vector<unsigned int> &meas_dimensions)
    {
        unsigned int meas_dimension;
        for(int i = 0 ; i< meas_dimensions.size(); i++)
        {
            // find if variables with size meas_sizes[i] are already allocated
            meas_dimension = meas_dimensions[i];
            _mapMeasUpdateVariables_it =  _mapMeasUpdateVariables.find(meas_dimension);
            if( _mapMeasUpdateVariables_it == _mapMeasUpdateVariables.end())
            {
                //variables with size z.rows() not allocated yet
                _mapMeasUpdateVariables_it = (_mapMeasUpdateVariables.insert
                        (std::pair<unsigned int,
                                MeasUpdateVariables>( meas_dimension,
                                                      MeasUpdateVariables() ))).first;
            }
        }
    }

    void KalmanFilter::AllocateMeasModel(const unsigned int &meas_dimensions)
    {
        // find if variables with size meas_sizes[i] are already allocated
        _mapMeasUpdateVariables_it =  _mapMeasUpdateVariables.find(meas_dimensions);
        if( _mapMeasUpdateVariables_it == _mapMeasUpdateVariables.end())
        {
            //variables with size z.rows() not allocated yet
            _mapMeasUpdateVariables_it = (_mapMeasUpdateVariables.insert
                    (std::pair<unsigned int,
                            MeasUpdateVariables>( meas_dimensions,
                                                  MeasUpdateVariables() ))).first;
        }
    }

    void KalmanFilter::PostSigmaSet(const Matrix3d &s)
    {
        dynamic_cast<Gaussian *>(_post)->CovarianceSet(s);
    }

    void KalmanFilter::PostMuSet(const Vector3d &c)
    {
        dynamic_cast<Gaussian *>(_post)->ExpectedValueSet(c);
    }

    void
    KalmanFilter::CalculateSysUpdate(const Vector3d &J,
                                     const Matrix3d &F,
                                     const Matrix3d &Q)
    {
        _Sigma_temp = F * ( _post->CovarianceGet() * F.transpose());
        _Sigma_temp += Q;
        //_Sigma_temp.convertToSymmetricMatrix(_Sigma_new);

        // set new state gaussian
        PostMuSet   ( J );
        PostSigmaSet( _Sigma_temp);
    }

    void KalmanFilter::CalculateMeasUpdate(const Vector3d &z,
                                           const Vector3d &Z,
                                           const Matrix3d &H,
                                           const Matrix3d &R)
    {
        // allocate measurement for z.rows() if needed
        //AllocateMeasModel(z.rows());

        (_mapMeasUpdateVariables_it->second)._postHT =
                (_post->CovarianceGet()) * H.transpose() ;
        (_mapMeasUpdateVariables_it->second)._S_Matrix =
                H * (_mapMeasUpdateVariables_it->second)._postHT;
        (_mapMeasUpdateVariables_it->second)._S_Matrix += R;

        // _K = covariance * H' * S(-1)
        (_mapMeasUpdateVariables_it->second)._K =
                (_mapMeasUpdateVariables_it->second)._postHT *
                        ( (_mapMeasUpdateVariables_it->second)._S_Matrix.inverse());

        // calcutate new state gaussian
        // Mu = expectedValue + K*(z-Z)
        (_mapMeasUpdateVariables_it->second)._innov = z-Z;
        _Mu_new  =  (_mapMeasUpdateVariables_it->second)._K *
                (_mapMeasUpdateVariables_it->second)._innov  ;
        _Mu_new  +=  _post->ExpectedValueGet() ;
        // Sigma = post - K*H*post
        _Sigma_temp = (_post->CovarianceGet());
        _Sigma_temp_par = (_mapMeasUpdateVariables_it->second)._K * H ;
        _Sigma_temp -= _Sigma_temp_par * (_post->CovarianceGet());
        // convert to symmetric matrix
        //_Sigma_temp.convertToSymmetricMatrix(_Sigma_new);

        // set new state gaussian
        PostMuSet( _Mu_new );
        PostSigmaSet( _Sigma_temp );
    }

    bool KalmanFilter::UpdateInternal(SystemModel<Vector3d> *const sysmodel,
                                      Vector3d &u,
                                      MeasurementModel<Vector3d, Vector3d> *const measmodel,
                                      const Vector3d &z, const Vector3d &s)
    {
        if (sysmodel != NULL)
        {
            SysUpdate(sysmodel,u);
        }
        if (measmodel != NULL)
        {
            MeasUpdate(measmodel,z,s);
        }
        return true;
    }
}
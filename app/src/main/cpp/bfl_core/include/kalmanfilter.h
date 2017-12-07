//
// Created by shydh on 3/26/17.
//

#ifndef AGVP_KALMANFILTER_H
#define AGVP_KALMANFILTER_H

#include "filter_core.h"
#include "gaussian.h"
#include "analyticsystemmodel_gaussianuncertainty.h"
#include "analyticmeasurementmodel_gaussianuncertainty.h"

#include <vector>
#include <map>

#define AnalyticSys    AnalyticSystemModelGaussianUncertainty
#define AnalyticMeas   AnalyticMeasurementModelGaussianUncertainty

namespace bfl_core
{
    /// Class representing the family of all Kalman Filters (EKF, IEKF, ...)
/** This is a class representing the family of all Kalman
    Filter (KF).  Kalman filters are filters in which the Posterior
    density is represented by a Gaussian density.  Kalman filters are
    only applicable to continuous systems.

    The system of updating the Posterior density is implemented in this
    base class. However, the parameters used for this update differ for
    different KFs (Simple KF,EKF,IEKF): that's why the xUpdate members
    are still pure virtual functions.

    This class is the base class for all sorts of KFs.

    @see Gaussian
    @see LinearAnalyticSystemModelGaussianUncertainty
*/
    class KalmanFilter : public Filter<Eigen::Vector3d, Eigen::Vector3d>
    {
    public:
        /// Constructor
        /** @pre you created the prior
            @param prior pointer to the Gaussian Pdf prior density
        */
        KalmanFilter(Gaussian *prior);

        /// Destructor
        virtual ~KalmanFilter();

        // implement virtual function
        virtual Gaussian *PostGet();

        /// Function to allocate memory needed during the measurement update,
        //  For realtime use, this function should be called before calling measUpdate
        /*  @param vector containing the dimension of the measurement models which are
            going to be used
        */
        void AllocateMeasModel( const std::vector<unsigned int>& meas_dimensions);

        /// Function to allocate memory needed during the measurement update
        //  For realtime use, this function should be called before calling measUpdate
        /*  @param dimension of the measurement models which is
            going to be used
        */
        void AllocateMeasModel( const unsigned int& meas_dimensions);

    private:
        struct MeasUpdateVariables
        {
            Eigen::Matrix3d _S_Matrix;
            Eigen::Matrix3d _K;
            Eigen::Vector3d _innov;
            Eigen::Matrix3d _postHT;

            MeasUpdateVariables() {}
        }; //struct

    protected:
        // variables to avoid allocation during update calls
        Eigen::Vector3d  _Mu_new;
        Eigen::Matrix3d _Sigma_new;
        Eigen::Matrix3d _Sigma_temp;
        Eigen::Matrix3d _Sigma_temp_par;
        Eigen::Matrix3d _SMatrix;
        Eigen::Matrix3d _K;
        std::map<unsigned int, MeasUpdateVariables> _mapMeasUpdateVariables;
        std::map<unsigned int, MeasUpdateVariables>::iterator _mapMeasUpdateVariables_it;


        /** Very dirty hack to avoid ugly methods PostSigmaSet
            and PostMuSet to be public!
            NonMinimalKalmanFilter should be redesigned though!
        */
        friend class NonminimalKalmanFilter;

        /// Set covariance of posterior estimate
        void PostSigmaSet( const Eigen::Matrix3d& s);

        /// Set expected value of posterior estimate
        void PostMuSet( const Eigen::Vector3d& c);

        /** Calculate Kalman filter System Update
            \f[ x_k = J \f]
            \f[ P_k = F.P_{k-}.F' + Q \f]
        */
        void CalculateSysUpdate(const Eigen::Vector3d& J,
                                const Eigen::Matrix3d& F,
                                const Eigen::Matrix3d& Q);

        /** Calculate Kalman filter Measurement Update
            \f[ x_k = x_{k-} + K.(z - Z) \f]
            \f[ P_k = (I-K.H).P_{k-} \f]
            with
            \f[ K = P_{k-}.H'.(H.P_{k-}.H'+R)^{-1} \f]
        */
        void CalculateMeasUpdate(const Eigen::Vector3d& z,
                                 const Eigen::Vector3d& Z,
                                 const Eigen::Matrix3d& H,
                                 const Eigen::Matrix3d& R);

        /// System Update
        /** Update the filter's Posterior density using the deterministic
            inputs to the system and the system model
            @param sysmodel pointer to the system model the filter should use
            @param u input to the system
        */
        virtual void SysUpdate(SystemModel<Eigen::Vector3d>* const sysmodel,
                               const Eigen::Vector3d& u) = 0;

        /// Measurement Update (overloaded)
        /** Update the filter's Posterior density using the sensor
            measurements, an input and the measurement model.  This method is
            used when the measurements depend on the inputs too (doesn't
            happen very often, does it?)
            BEWARE: the first time the measurment update is called
            with a new size of measurement, new allocations are done
            @param measmodel pointer to the measurement model the filter
            should use
            @param z sensor measurement
            @param s input to the system (must be of the same type as u
            for now, since this was not yet implemented in ConditionalPdf
        */
        virtual void
        MeasUpdate(MeasurementModel<Eigen::Vector3d, Eigen::Vector3d>* const measmodel,
                                const Eigen::Vector3d& z,
                                const Eigen::Vector3d& s) = 0;

        virtual bool
        UpdateInternal(SystemModel<Eigen::Vector3d>* const sysmodel,
                       Eigen::Vector3d& u,
                       MeasurementModel<Eigen::Vector3d, Eigen::Vector3d>* const measmodel,
                       const Eigen::Vector3d& z,
                       const Eigen::Vector3d& s);
    }; // class
}


#endif //AGVP_KALMANFILTER_H

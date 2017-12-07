//
// Created by shydh on 7/22/17.
//

#ifndef AGVP_SYSTEMMODEL_H
#define AGVP_SYSTEMMODEL_H

#include "sample.h"
#include "conditionalpdf.h"

namespace bfl_core
{
    /** Template class representing all possible (continu and discrete)
      System Models
      @todo Check if there should be a "model" base class...
      @bug Currently supports only systemmodels of the form P(x | x, u),
      where both u and x are continu or discrete.  So it
      lacks support for mixed systems () and systems with extra
      parameters.  You are welcome to provide an API and implementation
      for this :-)
  */
    template<typename T> class SystemModel
    {
    protected:

        /// ConditionalPdf representing \f$ P(X_k | X_{k-1}, U_{k}) \f$
        /* @bug Since, for now, the library only supports only conditional
       arguments of the same type, both X and U have to be of the same
       type (ie both continu or both discrete!).  I imagine there must
       be systems for which this approach is not general enough @see
       ConditionalPdf
        */
        ConditionalPdf<T,T>* _SystemPdf;

        /// System with no inputs?
        bool _systemWithoutInputs;

    public:
        /// Constructor
        /** @param systempdf ConditionalPdf<T,T> representing \f$ P(X_k |
        X_{k-1}, U_{k}) \f$
        @see STATE_SIZE, INPUT_SIZE, _SystemPdf
        */
        SystemModel(ConditionalPdf<T,T>* systempdf=NULL);

        /// Destructor
        virtual ~SystemModel();

        /// Copy constructor
        /// SystemModel(const SystemModel<T>& model);

        /// Get State Size
        /** @return the statesize of the system
         */
        int StateSizeGet() const;

        /// Has the system inputs or not
        bool SystemWithoutInputs() const;

        // NO LONGER RELEVANT
        // void StateSizeSet(int); // necessary??
        // Get Input Size
        /* @return the statesize of the system
         */
        // int InputSizeGet() const;
        // void InputSizeSet(int); // necessary??

        /// Get the SystemPDF
        /** @return a reference to the ConditionalPdf describing the system
         */
        ConditionalPdf<T,T>* SystemPdfGet();

        /// Set the SystemPDF
        /** @param pdf a reference to the ConditionalPdf describing the system
         */
        void SystemPdfSet(ConditionalPdf<T,T>* pdf);

        /// Simulate the system
        /** @param x current state of the system
        @param u input to the system
        @return State where we arrive by simulating the system model for
        1 step
        @param sampling_method the sampling method to be used while
        sampling from the Conditional Pdf describing the system (if not
        specified = DEFAULT)
        @param sampling_args Sometimes a sampling method can have some
        extra parameters (eg mcmc sampling)
        @note Maybe the return value would better be a Sample<T> instead
        of a T
        */
        T Simulate (const T& x, const T& u,
                    int sampling_method = DEFAULT, void * sampling_args = NULL);
        /// Simulate the system (no input system)
        /** @param x current state of the system
        @return State where we arrive by simulating the system model for
        1 step
        @note Maybe the return value would better be a Sample<T> instead
        of a T
        @param sampling_method the sampling method to be used while
        sampling from the Conditional Pdf describing the system (if not
        specified = DEFAULT)
        @param sampling_args Sometimes a sampling method can have some
        extra parameters (eg mcmc sampling)
        */

        T Simulate (const T& x, int sampling_method = DEFAULT, void * sampling_args = NULL);

        /// Get the probability of arriving in a next state
        /** @param x_k the next state (at time k)
        @param x_kminusone the current state (at time k-1)
        @param u  the input
        @return the probability value
        */

        double ProbabilityGet(const T& x_k, const T& x_kminusone, const T& u );

        /// Get the probability of arriving in a next state
        /** (no-input-system)
        @param x_k the next state (at time k)
        @param x_kminusone the current state (at time k-1)
        @return the probability value
        */
        double ProbabilityGet(const T& x_k, const T& x_kminusone );
    };

    // Constructor
    template<typename T>
    SystemModel<T>::SystemModel(ConditionalPdf<T,T>* systempdf)
    {
        if (systempdf != NULL)
        {
            switch(systempdf->NumConditionalArgumentsGet())
            {
                case 1:
                {
                    _systemWithoutInputs = true;
                    _SystemPdf  = systempdf;
                    break;
                }
                case 2:
                {
                    _systemWithoutInputs = false;
                    _SystemPdf  = systempdf;
                    break;
                }
                default:
                    return;
            }
        }
    }

// Destructor
    template<typename T>
    SystemModel<T>::~SystemModel()
    {
#ifdef __DESTRUCTOR__
        cout << "SystemModel::Destructor" << endl;
#endif // __DESTRUCTOR__
        /* KG: Probably a memory leak
           Who should clean this up? Sometimes the user will have created
           this Pdf, sometimes not (eg. by copy constructor).  If we allways
           delete it here.
           There has to be a cleaner way to implement this!
        */
        // delete SystemPdf;
    }

// Copy constructor
/*
template<typename T>
SystemModel<T>::SystemModel(const SystemModel<T>& model)
{
  SystemPdf  = &(model.SystemPdfGet());
}
*/

// Get State Size
    template<typename T>
    int SystemModel<T>::StateSizeGet() const
    {
        return _SystemPdf->DimensionGet();
    }

    template<typename T>
    bool SystemModel<T>::SystemWithoutInputs() const
    {
        return _systemWithoutInputs;
    }

// Get SystemPdf
    template<typename T>
    ConditionalPdf<T,T> *SystemModel<T>::SystemPdfGet()
    {
        return _SystemPdf;
    }

// Set SystemPdf
    template<typename T>
    void SystemModel<T>::SystemPdfSet(ConditionalPdf<T,T>* pdf)
    {
        switch(pdf->NumConditionalArgumentsGet())
        {
            case 1:
            {
                _systemWithoutInputs = true;
                _SystemPdf  = pdf;
                break;
            }
            case 2:
            {
                _systemWithoutInputs = false;
                _SystemPdf  = pdf;
                break;
            }
            default:
                return;
        }
    }

// Simulate from the system model
    template<typename T>
    T SystemModel<T>::Simulate (const T& x, const T& u, int sampling_method,
                              void * sampling_args)
    {
        _SystemPdf->ConditionalArgumentSet(0,x);
        _SystemPdf->ConditionalArgumentSet(1,u);
        Sample<T> Simulated(StateSizeGet());
        _SystemPdf->SampleFrom(Simulated, sampling_method,sampling_args);
        T result = Simulated.ValueGet();
        return result;
    }

    template<typename T>
    T SystemModel<T>::Simulate (const T& x, int sampling_method,
                              void * sampling_args)
    {
        _SystemPdf->ConditionalArgumentSet(0,x);
        Sample<T> Simulated(StateSizeGet());
        _SystemPdf->SampleFrom(Simulated, sampling_method,sampling_args);
        T result = Simulated.ValueGet();
        return result;
    }

    template <typename T>
    double SystemModel<T>::ProbabilityGet (const T& x_k, const T& x_kminusone,
                                           const T& u)
    {
        _SystemPdf->ConditionalArgumentSet(0,x_kminusone);
        _SystemPdf->ConditionalArgumentSet(1,u);
        return _SystemPdf->ProbabilityGet(x_k);
    }

    template <typename T>
    double SystemModel<T>::ProbabilityGet (const T& x_k, const T& x_kminusone)
    {
        _SystemPdf->ConditionalArgumentSet(0,x_kminusone);
        return _SystemPdf->ProbabilityGet(x_k);
    }
}
#endif //AGVP_SYSTEMMODEL_H

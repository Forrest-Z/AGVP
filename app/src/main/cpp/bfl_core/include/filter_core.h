//
// Created by shydh on 3/22/17.
//

#ifndef AGVP_FILTER_CORE_H
#define AGVP_FILTER_CORE_H

#include <clocale>
#include "pdfunction.h"
#include "systemmodel.h"
#include "measurementmodel.h"

namespace bfl_core
{
    /// Abstract class representing an interface for Bayesian Filters
    /** This is the Abstract interface class that defines the interface
        of Bayesian filters.  These filters are all related to i) a
        System Model, ii) a Measurement Model and iii) a Prior density
        reflecting the subjective belief of the person applying the filter
        BEFORE getting sensor or any other form of information about the
        modeled system.

        This class is the base class for particle filters, kalman filters,
        ...

        This class is a template class with 2 templates.  In this way
        it allows filtering for "semi-discrete" models, eg. models
        with a fixed number of states (discrete states) but with
        continuous observations, as needed in Automatic Speech
        Recognition.

        @see Pdf SystemModel MeasurementModel ConditionalPdf
        @bug For now, due to a "bug" (= non-existence of a feature :-) in
        the ConditionalPdf class, STATES AND INPUTS MUST BE OF THE SAME
        TYPE (both discrete, or both continuous!  This means that you can
        use this class for the following model types:
        - States, inputs and measurements continuous (most frequently
        used?)
        - States and inputs continous, Measurements discrete
        - States and inputs discrete, Measurements continous
        - States, inputs and measurements discrete

        StateVar represents the nature of the states and inputs
        MeasVar represents the nature of the measurements

        BEWARE: The order of the template arguments is reversed with
        respect to the notation used in "measurementmodel.h"
    */
    template<typename StateVar, typename MeasVar>
    class Filter
    {
    protected:

        /// prior Pdf
        Pdf<StateVar> *_prior;

        /// Pointer to the Posterior Pdf.
        /** The Posterior Pdf represents the subjective belief of the person
        applying the filter AFTER processing inputs and measurements.
        A filter does not maintain the beliefs at all timesteps t, since
        this leads to non-constant (or ever growing if you prefer)
        memory requirements.
        However, it is possible, to copy the Posterior density at all
        timesteps in your application by means of the PostGet() member
        function
        @see PostGet()
        */
        Pdf<StateVar> *_post;

        /// Represents the current timestep of the filter
        /** @todo Check wether this really belongs here
         */
        int time_step;

        /// Actual implementation of Update, varies along filters
        /** @param sysmodel pointer to the used system model
        @param u input param for proposal density
        @param measmodel pointer to the used measurementmodel
        @param z measurement param for proposal density
        @param s sensor param for proposal density
        */
        virtual bool UpdateInternal(SystemModel<StateVar> *const sysmodel,
                                    const StateVar &u,
                                    MeasurementModel<MeasVar, StateVar> *const measmodel,
                                    const MeasVar &z,
                                    const StateVar &s)=0;

    public:
        /// Constructor
        /** @pre you created the prior
        @param prior pointer to the prior Pdf
        */
        Filter(Pdf<StateVar> *prior) : _prior(prior),
                                       time_step(0)
        {}

        /// copy constructor
        /** @bug we should make a copy of the prior
         */
        Filter(const Filter<StateVar, MeasVar> &filt)
        {}

        /// destructor
        virtual ~Filter()
        {}

        /// Reset Filter
        virtual void Reset(Pdf<StateVar> *prior)
        {
            _prior = prior;
            _post = prior;
            // cout << "Filter::Reset() Post = " << _post->ExpectedValueGet() << endl;
        }

        /// Full Update (system with inputs/sensing params)
        /** @param sysmodel pointer to the system model to use for update
        @param u input to the system
        @param measmodel pointer to the measurement model to use for update
        @param z measurement
        @param s "sensing parameter"
         */
        virtual bool Update(SystemModel<StateVar> *const sysmodel,
                            const StateVar &u,
                            MeasurementModel<MeasVar, StateVar> *const measmodel,
                            const MeasVar &z,
                            const StateVar &s)
        {
            return this->UpdateInternal(sysmodel, u, measmodel, z, s);
        }

        /// Full Update (system without inputs, with sensing params)
        /** @param sysmodel pointer to the system model to use for
        update
        @param measmodel pointer to the measurement model to use for
        update
        @param z measurement
        @param s "sensing parameter"
         */
        virtual bool Update(SystemModel<StateVar> *const sysmodel,
                            MeasurementModel<MeasVar, StateVar> *const measmodel,
                            const MeasVar &z,
                            const StateVar &s)
        {
            StateVar u;
            return this->UpdateInternal(sysmodel, u, measmodel, z, s);
        }
        /// Full Update (system without inputs/sensing params)
        /** @param sysmodel pointer to the system model to use for
        update
        @param measmodel pointer to the measurement model to use for
        update
        @param z measurement
         */
        virtual bool Update(SystemModel<StateVar> *const sysmodel,
                            MeasurementModel<MeasVar, StateVar> *const measmodel,
                            const MeasVar &z)
        {
            StateVar s;
            StateVar u;
            return this->UpdateInternal(sysmodel, u, measmodel, z, s);
        }
        /// Full Update (system with inputs, without sensing params)
        /** @param sysmodel pointer to the system model to use for update
        @param u input to the system
        @param measmodel pointer to the measurement model to use for
        update
        @param z measurement
         */
        virtual bool Update(SystemModel<StateVar> *const sysmodel,
                            const StateVar &u,
                            MeasurementModel<MeasVar, StateVar> *const measmodel,
                            const MeasVar &z)
        {
            StateVar s;
            return this->UpdateInternal(sysmodel, u, measmodel, z, s);
        }

        /// System Update (system with inputs)
        /** @param sysmodel pointer to the system model to use for update
        @param u input to the system
         */
        virtual bool Update(SystemModel<StateVar> *const sysmodel,
                            const StateVar &u)
        {
            StateVar s;
            MeasVar z;
            return this->UpdateInternal(sysmodel, u, NULL, z, s);
        }
        /// System Update (system without inputs)
        /** @param sysmodel pointer to the system model to use for update
         */
        virtual bool Update(SystemModel<StateVar> *const sysmodel)
        {
            StateVar s;
            MeasVar z;
            StateVar u;
            return this->UpdateInternal(sysmodel, u, NULL, z, s);
        }

        /// Measurement Update (system with "sensing params")
        /** @param measmodel pointer to the measurement model to use for
        update
        @param z measurement
        @param s "sensing parameter"
         */
        virtual bool Update(MeasurementModel<MeasVar, StateVar> *const measmodel,
                            const MeasVar &z,
                            const StateVar &s)
        {
            StateVar u;
            return this->UpdateInternal(NULL, u, measmodel, z, s);
        }
        /// Measurement Update (system without "sensing params")
        /** @param measmodel pointer to the measurement model to use for
        update
        @param z measurement
         */
        virtual bool Update(MeasurementModel<MeasVar, StateVar> *const measmodel,
                            const MeasVar &z)
        {
            StateVar u;
            StateVar s;
            return this->UpdateInternal(NULL, u, measmodel, z, s);
        }

        /// Get Posterior density
        /** Get the current Posterior density
        @return a pointer to the current posterior
        */
        virtual Pdf<StateVar> *PostGet()
        {
            return _post;
        }

        /// Get current time
        /** Get the current time of the filter
        @return the current timestep
        */
        int TimeStepGet() const
        {
            return time_step;
        }
    };

    // For template instantiation
}
#endif //AGVP_FILTER_CORE_H

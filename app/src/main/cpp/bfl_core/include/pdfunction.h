//
// Created by shydh on 3/22/17.
//

#ifndef AGVP_PDFUNCTION_H
#define AGVP_PDFUNCTION_H

#include <clocale>
#include <vector>

#include "sample.h"
#include "../../Eigen/Dense"

namespace bfl_core
{
    // Defines for different sampling methods
#define DEFAULT 0 // Default sampling method, must be valid for every PDF!!
#define BOXMULLER 1
#define CHOLESKY 2
#define RIPLEY 3 // For efficient sampling from discrete/mcpdfs

    /// Class PDF: Virtual Base class representing Probability Density Functions
    template<typename T>
    class Pdf
    {

    public:
        /// Constructor
        /** @param dimension int representing the number of rows of the state
        */
        Pdf(unsigned int dimension = 0);

        // Default Copy Constructor will do the job

        /// Destructor
        virtual ~Pdf();

        ///Pure virtual clone function
        virtual Pdf<T> *Clone() const = 0;

        /// Draw multiple samples from the Pdf (overloaded)
        /** @param list_samples list of samples that will contain result of sampling
            @param num_samples Number of Samples to be drawn (iid)
        @param method Sampling method to be used.  Each sampling method
        is currently represented by a #define statement, eg.
        #define BOXMULLER 1
        @param args Pointer to a struct representing extra sample
        arguments.
        "Sample Arguments" can be anything (the number of steps a
        gibbs-iterator should take, the interval width in MCMC, ... (or
        nothing), so it is hard to give a meaning to what exactly
        Sample Arguments should represent...
        @todo replace the C-call "void * args" by a more object-oriented
        structure: Perhaps something like
        virtual Sample * Sample (const int num_samples,class Sampler)
        @bug Sometimes the compiler doesn't know which method to choose!
        */
        virtual bool SampleFrom(std::vector<Sample<T>> &list_samples,
                                const unsigned int num_samples,
                                int method = DEFAULT,
                                void *args = NULL) const;

        /// Draw 1 sample from the Pdf:
        /** There's no need to create a list for only 1 sample!
        @param one_sample sample that will contain result of sampling
        @param method Sampling method to be used.  Each sampling method
        is currently represented by a #define statement, eg.
        #define BOXMULLER 1
        @param args Pointer to a struct representing extra sample
        arguments
        @see SampleFrom()
        @bug Sometimes the compiler doesn't know which method to choose!
        */
        virtual bool SampleFrom(Sample<T> &one_sample,
                                int method = DEFAULT,
                                void *args = NULL) const;

        /// Get the probability of a certain argument
        /** @param input T argument of the Pdf
        @return the probability value of the argument
        */
        virtual double ProbabilityGet(const T &input) const;

        /// Get the dimension of the argument
        /** @return the dimension of the argument
         */
        unsigned int DimensionGet() const;

        /// Set the dimension of the argument
        /** @param dim the dimension
         */
        virtual void DimensionSet(unsigned int dim);

        /// Get the expected value E[x] of the pdf
        /** Get low order statistic (Expected Value) of this AnalyticPdf
        @return The Expected Value of the Pdf (a ColumnVector with
        DIMENSION rows)
        @note No set functions here!  This can be useful for analytic
        functions, but not for sample based representations!
        @note For certain discrete Pdfs, this function has no
        meaning, what is the average between yes and no?
        */
        virtual T ExpectedValueGet() const;

        /// Get the Covariance Matrix E[(x - E[x])^2] of the Analytic pdf
        /** Get first order statistic (Covariance) of this AnalyticPdf
        @return The Covariance of the Pdf (a SymmetricMatrix of dim
        DIMENSION)
        @todo extend this more general to n-th order statistic
        @bug Discrete pdfs should not be able to use this!
        */
        virtual Eigen::Matrix3d CovarianceGet() const;

    private:
        /// Dimension of the argument x of P(x | ...).
        unsigned int _dimension;

    };

    template<typename T>
    Pdf<T>::Pdf(unsigned int dim)
    {
        //assert((int)dim >= 0);

        _dimension = dim;
    }

    template<typename T>
    Pdf<T>::~Pdf()
    {
    }

    template<typename T>
    inline unsigned int Pdf<T>::DimensionGet() const
    {
        return _dimension;
    }

    template<typename T>
    void Pdf<T>::DimensionSet(unsigned int dim)
    {
        //assert((int)dim >= 0);
        _dimension = dim;
    }

    template<typename T>
    bool Pdf<T>::SampleFrom(std::vector<Sample<T>> &list_samples,
                            const unsigned int num_samples,
                            int method,
                            void *args) const
    {
        list_samples.resize(num_samples);
        typename std::vector<Sample<T>>::iterator sample_it;
        for (sample_it = list_samples.begin();
             sample_it != list_samples.end();
             sample_it++)
            if (!this->SampleFrom(*sample_it, method, args))
                return false;

        return true;
    }

    template<typename T>
    bool Pdf<T>::SampleFrom(Sample<T> &one_sample,
                            int method,
                            void *args) const
    {
        //cerr << "Error Pdf<T>:
        // The SampleFrom function was called, but you didn't implement it!\n";
        //exit(-BFL_ERRMISUSE);
        return false;
    }

    template<typename T>
    double Pdf<T>::ProbabilityGet(const T &input) const
    {
        //cerr << "Error Pdf<T>:
        // The ProbabilityGet function was called, but you didn't implement it!\n";
        //exit(-BFL_ERRMISUSE);
        return 1;
    }

    template<typename T>
    T Pdf<T>::ExpectedValueGet() const
    {
        //cerr << "Error Pdf<T>:
        // The ExpectedValueGet function was called, but you didn't implement it!\n";
        //exit(-BFL_ERRMISUSE);
        T t;
        return t;
    }


    template<typename T>
    Eigen::Matrix3d Pdf<T>::CovarianceGet() const
    {
        //cerr << "Error Pdf<T>:
        // The CovarianceGet function was called, but you didn't implement it!\n";
        //exit(-BFL_ERRMISUSE);
        Eigen::Matrix3d m;
        return m;
    }
}
#endif //AGVP_PDFUNCTION_H

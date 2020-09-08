#pragma once


#include <vector>
#include <map>

#include <mlpack/core.hpp>
#include <mlpack/methods/gmm/gmm.hpp>
#include <armadillo>
#include <gp/gp.h>
#include <gp/gp_utils.h>
#include <gp/rprop.h>

#include "motion.h"
#include "../utils/serializable.h"
#include "../motions/dmp.h"
#include "../motions/trajectory.h"
#include "../motions/unscrew.h"

// For debugging
#include <typeinfo>

namespace ades {

    class MotionSequence2 : public Serializable
    {
    private:
        uint64_t ID;
        std::vector<std::string> inputTypes_;
        std::vector<const Motion*> motions_;
        std::map<std::string, mlpack::gmm::GMM> gmm_effectModels_;
        std::map<std::string, libgp::GaussianProcess> gp_effectModels_;


    public:
        /*! Constructor for type ades::MotionSequence2.
         *  \param inputTypes : vector of <string> specifying required parameters for this motion
         *                      sequence by name
         *  \param motions : vector of <Motion*> specifying a squence of motions (see Motion)
         *  \param gmm_effectModels : map of <string, mlpack::gmm::GMM> specifying joint probabilistic effect
         *                        models using GMMs
         *  \param gp_effectModels : map of <string, libgp::GaussianProcess> specifying conditional
                                     probabilistic effect models using GPs
         */
        MotionSequence2(std::vector<std::string> inputTypes = std::vector<std::string>(),
                       std::vector<const Motion*> motions = std::vector<const Motion*>(),
                       std::map<std::string, mlpack::gmm::GMM> gmm_effectModels = std::map<std::string, mlpack::gmm::GMM>(),
                       std::map<std::string, libgp::GaussianProcess> gp_effectModels = std::map<std::string, libgp::GaussianProcess>());

        ~MotionSequence2();

        uint64_t getID() const
        {
            return ID;
        }


        /*! Add a set of input types to this motion sequence. Checks against duplicate entries.
         *  \param inputTypes : vector of <string> specifying input types
         */
        void insertInputTypes(const std::vector<std::string> inputTypes);


        /*! Remove the specifed input types from this motion sequence.
         *  \param inputTypes : vector of <string> specifying input types to be removed
         */
        void removeInputTypes(const std::vector<std::string> inputTypes);


        /*! Return the current list of input types.
         *  \return a vector of <string> containing all input types
         */
        std::vector<std::string> getInputTypes() const
        {
            return inputTypes_;
        }

        /*! Add a motion to this motion sequence at specified step. Step hereby denotes the
         *  temporal order of execution.
         *  \param step : integer index in the motion (at which step do I do this)
         *  \param motion : the motion to insert
         */
        void insertMotion(const int step, const Motion *motion);


        /*! Remove the specifed motion (specified by its step) from this motion sequence.
         *  \param step : integer index in the motion (at which step do I do this)
         */
        void removeMotion(const int step);


        /*! Modify a motion by specifying its step. Returns a reference
         *  to a motion.
         *  \param step : integer index in the motion (at which step do I do this)
         *  \return the motion to modify
         *  \exception std::out_of_range : if the MotionSequence2 does not have an motion
         *                                 at the specified step
         */
        Motion const *modifyMotion(const int step);


        /*! Return the current list of motions.
         *  \return a vector of <const Motion> containing all input types
         */
        std::vector<const Motion*> getMotions() const
        {
            return motions_;
        }


        /*! Add an effect model by providing the parameters of the GMM.
         *  \param effectType : an effect of <string> for which to add a model
         *  \param gaussiansNb : number of gaussians to use in the GMM (effect model)
         *  \param gaussiansDim : dimensionality of gaussians to use in the GMM (effect model)
         */
        void insertGMMEffectModel(const std::string effectType, int gaussiansNb, int gaussiansDim);

        /*! Add an effect model to this motion sequence for the specified effect.
         *  \param effectType : an effect of <string> for which to add a model
         *  \param dist : the effect model
         */
        void insertGMMEffectModel(const std::string effectType, mlpack::gmm::GMM dist);


        /*! Remove the specifed effect model (specified by effectType) from this motion sequence.
         *  \param effectType : an effect of <string> for which to remove a model
         */
        void removeGMMEffectModel(const std::string effectType);


        /*! Updates the effect model for the specified effect type using a new observation.
         *  \param effectType : an effect of <string> for which to remove a model
         *  \param input: a vector of <double> specifying the input parameters
         *  \param effect: a <double> specifying the desired effect
         */
        void updateGMMEffectModel(std::string effectType, std::vector<double> input, double effect);


        /*! Return the current list of effect models.
         *  \return a map of <string, mlpack::gmm::GMM> listing all effect types with
         *          their associated model
         */
        std::map<std::string, mlpack::gmm::GMM> getGMMEffectModels() const
        {
            return gmm_effectModels_;
        }

         /*! Add an effect model from GP parameters to this  motion sequence for the specified effect.
         *  \param effectType : an effect of <string> for which to add a model
         *  \param dist : the effect
         */
        void insertGPEffectModel(const std::string effectType, int gpDim, std::string covf);


         /*! Add an effect model to this motion sequence for the specified effect.
         *  \param effectType : an effect of <string> for which to add a model
         *  \param dist : the effect model
         */
        void insertGPEffectModel(const std::string effectType, libgp::GaussianProcess dist);


        /*! Remove the specifed effect model (specified by effectType) from this motion sequence.
         *  \param effectType : an effect of <string> for which to remove a model
         */
        void removeGPEffectModel(const std::string effectType);


        /*! Updates the effect model for the specified effect type using a new observation.
         *  \param effectType : an effect of <string> for which to remove a model
         *  \param input: a vector of <double> specifying the input parameters
         *  \param effect: a <double> specifying the desired effect
         */
        void updateGPEffectModel(std::string effectType, std::vector<double> input, double effect);


        /*! Return the current list of effect models.
         *  \return a map of <string, libgp::GaussianProcess> listing all effect types with
         *          their associated model
         */
        std::map<std::string, libgp::GaussianProcess> getGPEffectModels() const
        {
            return gp_effectModels_;
        }


        /*! Estimate the confidence of parameters in terms of a specific effect, i.e.,
         *  do these parameters yield the expected effect.
         *  \param effectType : an effect of <string> for which to estimate the confidence
            \param input: a vector of <double> specifying the input parameters
            \param effect: a <double> specifying the desired effect
            \return the confidence that these parameters yield the desired effect
         */
        double estimateEffectLikelihood(const std::string effectType, std::vector<double> input, double effect);


        /*! Estimate the mean effect given input parameters.
         *  \param effectType : an effect of <string> for which to estimate the confidence
            \param input: a vector of <double> specifying the input parameters
            \return the estimated effect
         */
        double estimateEffect(const std::string effectType, std::vector<double> input);


        /*! Estimate the variance of an effect given input parameters.
         *  \param effectType : an effect of <string> for which to estimate the confidence
            \param input: a vector of <double> specifying the input parameters
            \return the estimated effect
         */
        double estimateEffectVariance(const std::string effectType, std::vector<double> input);

        template <class Archive> void save(Archive & ar, const unsigned int version) const
        {
            ar.template register_type<DMPContainer>();
            ar.template register_type<TrajectoryContainer>();
            ar.template register_type<UnscrewContainer>();

            ar & BOOST_SERIALIZATION_NVP(ID);
            ar & BOOST_SERIALIZATION_NVP(inputTypes_);

            //GMM serialization
            auto num_gmm_models = gmm_effectModels_.size();
            ar & BOOST_SERIALIZATION_NVP(num_gmm_models);

            for(auto it : gmm_effectModels_)
            {
                auto name = it.first;
                auto model = it.second;
                auto weights = model.Weights();
                auto num_gaussians = model.Gaussians();

                ar & BOOST_SERIALIZATION_NVP(name);
                ar & BOOST_SERIALIZATION_NVP(weights);
                ar & BOOST_SERIALIZATION_NVP(num_gaussians);

                for(int i = 0; i < model.Gaussians(); i++)
                {
                    auto component = model.Component(i);
                    auto pc = &component;
                    ar & mlpack::data::CreateNVP(model.Component(i), "comp"+std::to_string(i));
                }
            }

            //GP serialization
            std::vector<std::string> effectTypes;
            for(auto it : gp_effectModels_)
            {
                effectTypes.push_back(it.first);
            }

            ar & BOOST_SERIALIZATION_NVP(effectTypes);

            std::ostringstream oss;
            oss << ID;
            std::string ID_(oss.str());

            for(auto effect : effectTypes)
            {
                auto model = gp_effectModels_.at(effect);
                std::string path = DB_DIR + "/" + effect + "_" + ID_ + ".effect";
                model.write(path.c_str());
            }

            ar & BOOST_SERIALIZATION_NVP(motions_);
        }

        template <class Archive> void load(Archive & ar, const unsigned int version)
        {
            ar.template register_type<DMPContainer>();
            ar.template register_type<TrajectoryContainer>();
            ar.template register_type<UnscrewContainer>();

            ar & BOOST_SERIALIZATION_NVP(ID);
            ar & BOOST_SERIALIZATION_NVP(inputTypes_);

            //GMM deserialization
            int num_gmm_models;
            ar & BOOST_SERIALIZATION_NVP(num_gmm_models);

            for(int i = 0; i < num_gmm_models; i++)
            {
                std::string name;
                arma::vec weights;
                int num_gaussians;

                ar & BOOST_SERIALIZATION_NVP(name);
                ar & BOOST_SERIALIZATION_NVP(weights);
                ar & BOOST_SERIALIZATION_NVP(num_gaussians);

                std::vector<mlpack::distribution::GaussianDistribution> dists;
                for(int j = 0; j < num_gaussians; j++)
                {
                    mlpack::distribution::GaussianDistribution d;
                    ar & mlpack::data::CreateNVP(d, "comp"+std::to_string(j));
                    dists.push_back(d);

                }
                auto model = mlpack::gmm::GMM(dists, weights);
                gmm_effectModels_.insert(std::pair<std::string, mlpack::gmm::GMM>(name, model));
            }

            //GP deserialization
            std::vector<std::string> effectTypes;
            ar & BOOST_SERIALIZATION_NVP(effectTypes);

            std::ostringstream oss;
            oss << ID;
            std::string ID_(oss.str());

            for(auto effect : effectTypes)
            {
                std::string path = DB_DIR + "/" + effect + "_" + ID_ + ".effect";
                auto model = libgp::GaussianProcess(path.c_str());
                gp_effectModels_.insert(std::pair<std::string, libgp::GaussianProcess>(effect, model));
            }

            ar & BOOST_SERIALIZATION_NVP(motions_);
        }

        BOOST_SERIALIZATION_SPLIT_MEMBER()
    };
}

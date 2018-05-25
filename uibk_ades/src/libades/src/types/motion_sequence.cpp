#include "../../include/types/motion_sequence.h"


using namespace std;

namespace ades {

    MotionSequence::MotionSequence(vector<string> inputTypes,
                                   vector<const Motion*> motions,
                                   map<string, mlpack::gmm::GMM> gmm_effectModels,
                                   map<string, libgp::GaussianProcess> gp_effectModels
        ) : ID(reinterpret_cast<uint64_t>(&inputTypes)),
            inputTypes_(inputTypes),
            motions_(motions),
            gmm_effectModels_(gmm_effectModels),
            gp_effectModels_(gp_effectModels)
    {}

    MotionSequence::~MotionSequence(){}

    void MotionSequence::insertInputTypes(const vector<string> inputTypes)
    {
        for(auto inputType : inputTypes)
        {
            // If the input has NOT been found (thus find return end() of inputtypes)
            if(find(inputTypes_.begin(), inputTypes_.end(), inputType) == inputTypes_.end())
            {
                inputTypes_.push_back(inputType);
            }
        }
    }

    void MotionSequence::removeInputTypes(const vector<string> inputTypes)
    {
        for(auto inputType : inputTypes_) {
            auto it = find(inputTypes_.begin(), inputTypes_.end(), inputType);
            if(it != inputTypes_.end()) {
                inputTypes_.erase(it);
            }
        }
    }

    void MotionSequence::insertMotion(const int step, const Motion *motion)
    {
        motions_.insert(motions_.begin()+step, motion);
    }

    void MotionSequence::removeMotion(const int step)
    {
        motions_.erase(motions_.begin() + step);
    }

    Motion const *MotionSequence::modifyMotion(const int step)
    {
        return motions_.at(step);
    }

    void MotionSequence::insertGMMEffectModel(const string effectType, int gaussiansNb, int gaussiansDim)
    {
        // gaussiansDim + 1 because the effect are an input of the model
        mlpack::gmm::GMM newDist(gaussiansNb, gaussiansDim+1);
        gmm_effectModels_.insert(pair<string, mlpack::gmm::GMM>(effectType, newDist));
    }

    void MotionSequence::insertGMMEffectModel(const string effectType, mlpack::gmm::GMM dist)
    {
        gmm_effectModels_.insert(pair<string, mlpack::gmm::GMM>(effectType, dist));
    }

    void MotionSequence::removeGMMEffectModel(const string effectType)
    {
        gmm_effectModels_.erase(effectType);
    }

    void MotionSequence::updateGMMEffectModel(string effectType, vector<double> input, double effect)
    {
        input.push_back(effect);
        arma::mat input_(input);
        gmm_effectModels_.at(effectType).Train(input, 1, true);
    }

    void MotionSequence::insertGPEffectModel(const std::string effectType, int gpDim, std::string covf)
    {
        libgp::GaussianProcess newgp(gpDim, covf);
        gp_effectModels_.insert(pair<string, libgp::GaussianProcess>(effectType, newgp));
    }

    void MotionSequence::insertGPEffectModel(const string effectType, libgp::GaussianProcess dist)
    {
        gp_effectModels_.insert(pair<string, libgp::GaussianProcess>(effectType, dist));
    }

    void MotionSequence::removeGPEffectModel(const string effectType)
    {
        gp_effectModels_.erase(effectType);
    }

    void MotionSequence::updateGPEffectModel(string effectType, vector<double> input, double effect)
    {
        double* input_ = &input[0];
        gp_effectModels_.at(effectType).add_pattern(input_, effect);
        // optimizing with Rprop as recommended in the lib
        libgp::RProp rprop;
        rprop.init();
        rprop.maximize(&(gp_effectModels_.at(effectType)), 50, true);
    }

    double MotionSequence::estimateEffectLikelihood(const std::string effectType, std::vector<double> input, double effect)
    {
        input.push_back(effect);
        arma::vec input_(input);
        std::cout << "estimate effect likelihood" << std::endl;
        std::cout << input_ << std::endl;
        return gmm_effectModels_.at(effectType).Probability(input_);
    }

    double MotionSequence::estimateEffect(const std::string effectType, std::vector<double> input)
    {
        //double const* input_ = &input[0];
        double const * input_ = input.data();
        //input.size()
        //const double input_ = ;
        std::cout << "estimate effect" << std::endl;
        std::cout << input[0] << std::endl;
        const double input1_[2] = {0.0, 0.0};
        const double input2_[2] = {1.0, 0.0};
        const double input3_[2] = {-3.0, 3.0};
        std::cout << gp_effectModels_.at(effectType).f(input1_) << std::endl;
        std::cout << gp_effectModels_.at(effectType).f(input2_) << std::endl;
        std::cout << gp_effectModels_.at(effectType).f(input3_) << std::endl;
        return gp_effectModels_.at(effectType).f(input_);
    }

    double MotionSequence::estimateEffectVariance(const std::string effectType, std::vector<double> input)
    {
        double const* input_ = &input[0];
        return gp_effectModels_.at(effectType).var(input_);
    }
}

#pragma once


#include "../types/motion.h"

namespace ades {
    class DMPContainer : public virtual Motion
    {
        private:
            std::vector<double> K_;
            std::vector<double> D_;
            std::vector<double> weights_;
            std::vector<double> psiMatrix_;
            //std::vector<double> gaussianCenters;
            //std::vector<double> gaussianVariances;
            //std::vector<double> dmpCoeffs;
            // dmp coeffs ordered as tau, alpha_z, beta_z, alpha_g, see dmp definition:

        public:
            DMPContainer(std::vector<double> K = std::vector<double>(),
                         std::vector<double> D = std::vector<double>(),
                         std::vector<double> w = std::vector<double>(),
                         std::vector<double> psi = std::vector<double>()
                      )
            {
                K_ = K;
                D_ = D;
                weights_ = w;
                psiMatrix_ = psi;
            }

            DMPContainer(const DMPContainer & d) :
                K_(d.K_),
                D_(d.D_),
                weights_(d.weights_),
                psiMatrix_(d.psiMatrix_)
            {

            }

            ~DMPContainer()
            {

            }

            std::map<std::string, std::vector<double>> getMotionParameters() const {
              std::map<std::string, std::vector<double>> params;

              params.insert(std::pair<std::string, std::vector<double>>("K", K_));
              params.insert(std::pair<std::string, std::vector<double>>("D", D_));
              params.insert(std::pair<std::string, std::vector<double>>("weights", weights_));
              params.insert(std::pair<std::string, std::vector<double>>("psiMatrix", psiMatrix_));
              //params.insert(std::pair<std::string, std::vector<double>>("gaussianCenters", gaussianCenters));
              //params.insert(std::pair<std::string, std::vector<double>>("gaussianVariances", gaussianVariances));
              //params.insert(std::pair<std::string, std::vector<double>>("weights", weights));
              //params.insert(std::pair<std::string, std::vector<double>>("dmpCoeffs", dmpCoeffs));

              return params;
            }

            bool isTemporallyScalable() const
            {
              return true;
            }

            float getTemporalScale() const
            {
              return temporalScale;
            }

            void setTemporalScale(float scaling = 1.0)
            {
              temporalScale = scaling;
            }

            MotionType getMotionType() const
            {
              return MotionType::DMP;
            }

            template <class Archive> void serialize(Archive & ar, const unsigned int version)
            {
                ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Motion);

                ar & BOOST_SERIALIZATION_NVP(K_);
                ar & BOOST_SERIALIZATION_NVP(D_);
                ar & BOOST_SERIALIZATION_NVP(weights_);
                ar & BOOST_SERIALIZATION_NVP(psiMatrix_);
             }
    };

}

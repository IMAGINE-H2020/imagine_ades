#pragma once

#include "../types/motion.h"

namespace ades {
    class UnscrewContainer : public virtual Motion
    {
        public:
            UnscrewContainer()
            {}

            bool isTemporallyScalable() const
            {
              return false;
            }

            float getTemporalScale() const
            {
              return 1.0;
            }

            void setTemporalScale(float scaling = 1.0)
            {
		return;
            }

            UnscrewContainer(const UnscrewContainer & d)
            {}

            ~UnscrewContainer()
            {}

            std::map<std::string, std::vector<double>> getMotionParameters() const
            {
                std::map<std::string, std::vector<double>> params;
                return params;
            }

            MotionType getMotionType() const
            {
              return MotionType::Unscrewing;
            }

            template <class Archive> void serialize(Archive & ar, const unsigned int version)
            {
                ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Motion);
	    }
    };

}


#pragma once

#include "../types/motion.h"

namespace ades {
    class TrajectoryContainer : public virtual Motion
    {
        private:
            //std::vector<std::pair<double, double>> points;
            std::vector<std::vector<double>> points;
            float dimension;

        public:
            TrajectoryContainer(std::vector<std::vector<double>> p = std::vector<std::vector<double>>()) :
                points(p)
            {
                if( !p.empty() )
                {
                    if( !p.begin()->empty() )
                    {
                        dimension = p.begin()->size();
                    }
                }
            }

            // Constructor for 2d array
            //TrajectoryContainer(std::vector<double> x = std::vector<double>(), std::vector<double> y = std::vector<double>()) 
            /*TrajectoryContainer(std::vector<double> x, std::vector<double> y) 
            {
                if( x.size() != y.size() )
                {
                    std::cout << "Error; sizes need to be the same on x and y axis" << std::endl;
                }
                else
                {
                    for(auto i=0 ; i < x.size() ; i++)
                    {
                        std::vector<double> point2d;
                        point2d.push_back(x.at(i));
                        point2d.push_back(y.at(i));
                        points.push_back(point2d);
                    }
                }
            }*/

            TrajectoryContainer(const TrajectoryContainer & d) :
                points(d.points)
            {

            }

            ~TrajectoryContainer()
            {
            }

            std::map<std::string, std::vector<double>> getMotionParameters() const
            {
                std::map<std::string, std::vector<double>> params;
                
                int count=0;
                for(auto p : points)
                {
                    params.insert(std::pair<std::string, std::vector<double>>("points_dim"+std::to_string(count), p));
                    count +=1;
                }
                
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
              return MotionType::Trajectory;
            }

            template <class Archive> void serialize(Archive & ar, const unsigned int version)
            {
                ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Motion);

                ar & BOOST_SERIALIZATION_NVP(points);
             }
    };

}


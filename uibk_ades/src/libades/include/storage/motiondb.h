#pragma once


#include <vector>
#include <map>
#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>
#include <boost/serialization/map.hpp>
#include <iostream>
#include <type_traits>
#include <string>

#include "../types/motion.h"
#include "../utils/serializable.h"
#include "../motions/dmp.h"
#include "../motions/trajectory.h"
#include "../motions/unscrew.h"

// For debugging
#include <typeinfo>

namespace ades {

    class MotionDB : public Serializable
    {
    private:
        friend class boost::serialization::access;

        std::string home_;
        unsigned int version_;
    
        std::map<std::string, const Motion*> motions_;

        bool populate();

        bool serialize();

    public:
        /*! Constructor for type ades::MotionDB.
         *  \param inputTypes : vector of <string> specifying required parameters for this motion
         *                      sequence by name
         *  \param motions : vector of <Motion*> specifying a squence of motions (see Motion)
         *  \param gmm_effectModels : map of <string, mlpack::gmm::GMM> specifying joint probabilistic effect
         *                        models using GMMs
         *  \param gp_effectModels : map of <string, libgp::GaussianProcess> specifying conditional
                                     probabilistic effect models using GPs
         */
        MotionDB(std::string home, unsigned int version);
        /*MotionDB(std::vector<std::string> inputTypes = std::vector<std::string>(),
                       std::vector<const Motion*> motions = std::vector<const Motion*>());
                       */
        ~MotionDB();
        
        std::string getHome() const
        {
            return home_;
        }

        int getMotionNb()
        {
            return motions_.size();
        }

        /*! Verify by name if Motion exists in MotionDB.
         *  \param name : a Motion name
         */
        bool isInDB(std::string name);

        /*! Return the requested motion.
         *  \return a <const Motion> matching the name
         */
        Motion const* getMotion(std::string name);

        /*! Return the current list of motions.
         *  \return a vector of <const Motion> containing all input types
         */
        std::map<std::string, const Motion*> listMotion() const
        {
            return motions_;
        }

        /*! Add a motion to this motion sequence at specified step. Step hereby denotes the
         *  temporal order of execution.
         *  \param step : integer index in the motion (at which step do I do this)
         *  \param motion : the motion to insert
         */
        void insertMotion(const Motion *motion);


        /*! Remove the specifed motion (specified by its step) from this motion sequence.
         *  \param step : integer index in the motion (at which step do I do this)
         */
        void removeMotion(std::string name);


        /*! Modify a motion by specifying its step. Returns a reference
         *  to a motion.
         *  \param step : integer index in the motion (at which step do I do this)
         *  \return the motion to modify
         *  \exception std::out_of_range : if the MotionDB does not have an motion
         *                                 at the specified step
         */
        Motion const *modifyMotion(const std::string name);


        /*template <class Archive> void save(Archive & ar, const unsigned int version) const
        {
            ar.template register_type<DMPContainer>();
            ar.template register_type<TrajectoryContainer>();
            ar.template register_type<UnscrewContainer>();

            //ar & BOOST_SERIALIZATION_NVP(ID);
            //ar & BOOST_SERIALIZATION_NVP(inputTypes_);

            ar & BOOST_SERIALIZATION_NVP(motions_);
        }

        template <class Archive> void load(Archive & ar, const unsigned int version)
        {
            ar.template register_type<DMPContainer>();
            ar.template register_type<TrajectoryContainer>();
            ar.template register_type<UnscrewContainer>();

            //ar & BOOST_SERIALIZATION_NVP(ID);
            //ar & BOOST_SERIALIZATION_NVP(inputTypes_);

            ar & BOOST_SERIALIZATION_NVP(motions_);
        }

        BOOST_SERIALIZATION_SPLIT_MEMBER()
        */
    };
}

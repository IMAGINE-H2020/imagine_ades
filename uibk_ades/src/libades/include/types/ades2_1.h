#pragma once


#include <string>
#include <map>
#include <vector>
#include <iterator>


#include "motion_sequence2_1.h"
#include "../utils/serializable.h"


namespace ades {

    class Ades2_1 : public Serializable
    {
    private:
        uint64_t ID;
        std::string name_;
        std::map<std::string, std::string> preconditions_;
        std::map<std::string, std::string> effects_;

        //TODO Explicit scene targets/affordances necessary or do preconditions suffice?

        std::map<std::string, MotionSequence2_1> motion_sequences_;


    public:
        /*! Constructor for type ades::Ades2_1.
         *  \param name : a name of <string> for this Ades2_1
         *  \param preconditions : map of <string, string> specifying symbolic
         *                          precondition descriptions in a Prolog-like style; e.g., <head, body>
         *  \param effects : map of <string, string> specifying symbolic effect
         *                   descriptions in a Prolog-like style; e.g., <head, body>
         *  \param motions : map of <string, MotionSequence2_1> specifying subsymbolic
         *                   motion sequences (see Motion and MotionSequence2_1_1 for more information)
         */
        Ades2_1(std::string name = "",
             std::map<std::string, std::string> preconditions = std::map<std::string, std::string>(),
             std::map<std::string, std::string> effects = std::map<std::string, std::string>(),
             std::map<std::string, MotionSequence2_1> motions = std::map<std::string, MotionSequence2_1>()
            );

        ~Ades2_1();


        uint64_t getID() const
        {
            return ID;
        }


        std::string getName() const
        {
            return name_;
        }


        void setName(std::string name)
        {
            name_ = name;
        }


        /*! Add a set of new preconditions to this Ades2_1. Existing preconditions will be
         *  ignored.
         *  \param conditions : map of <string, string> specifying symbolic precondition
         *                      descriptions in a Prolog-like style; e.g., <head, body>
         */
        void insertPreconditions(const std::map<std::string, std::string> conditions);


        /*! Remove the preconditions whose head appears in conditions.
         *  If conditions is empty, remove all preconditions.
         *  \param conditions : vector of <string> specifying rule heads of preconditions
         *                      to be removed
         */
        void removePreconditions(const std::vector<std::string> conditions = std::vector<std::string>());


        /*! Modify a set of preconditions as specified by conditions.
         *  \param conditions : map of <string, string> specifying symbolic precondition
         *                      descriptions in a Prolog-like style; e.g., <head, body>
         */
        void modifyPreconditions(const std::map<std::string, std::string> conditions);


        /*! Return the current list of preconditions.
         *  \return a map of <string, string> containing all preconditions
         */
        std::map<std::string, std::string> getPreconditions() const
        {
            return preconditions_;
        }


        /*! Add a set of effects to this Ades2_1. Existing effects will be ignored.
        *   \param effects : map of <string, string> specifying symbolic effect
        *                    descriptions in a Prolog-like style; e.g., <head, body>
        */
        void insertEffects(const std::map<std::string, std::string> effects);


        /*! Remove the effects whose head appears in conditions.
         *  If effects is empty, remove all effects.
         *  \param effects : vector of <string> specifying rule heads of effects
         *                   to be removed
         */
        void removeEffects(const std::vector<std::string> effects = std::vector<std::string>());


        /*! Modify a set of effects as specified by effects.
         *  \param effects : map of <string, string> specifying symbolic effect
         *                      descriptions in a Prolog-like style; e.g., <head, body>
         */
        void modifyEffects(const std::map<std::string, std::string> effects);


        /*! Return the current list of effects.
         *  \return a map of <string, string> containing all effects
         */
        std::map<std::string, std::string> getEffects() const
        {
            return effects_;
        }


        /*! Add a motion sequence (see MotionSequence2_1) to this Ades2_1. If a motion
         *  sequence of the same name already exists, the new motion sequence won't be added.
         *  \param motionSequenceID : a ID of <string> to identify this sequence
         *  \param motionSequence : the motion sequence (see MotionSequence2_1) to add
         */
        void insertMotionSequence(const std::string motionSequenceID,
                                        MotionSequence2_1 motionSequence);

        /*! Removes a motion sequence (see MotionSequence2_1) by ID from this Ades2_1.
         *  \param motionSequenceID : a ID of <string> to identify the sequence
         */
        void removeMotionSequence(const std::string motionSequenceID);


        /*! Modify a motion sequence by specifying its ID. Returns a reference
         *  to a motion sequence.
         *  \param motionSequenceID the specified motion sequence
         *  \return the motion sequence to modify
         *  \exception std::out_of_range : if the Ades2_1 does not have an motion sequence
         *                                 with the specified motionSequenceID
         */
        MotionSequence2_1 *modifyMotionSequence(const std::string motionSequenceID);


        /*! Return the current list of motion sequences.
         *  \return a map of <string, MotionSequence2_1> containing all motion sequences
         */
        std::map<std::string, MotionSequence2_1> getMotionSequences() const
        {
            return motion_sequences_;
        }

        template <class Archive> void serialize(Archive & ar, const unsigned int version)
        {
            ar & BOOST_SERIALIZATION_NVP(ID);
            ar & BOOST_SERIALIZATION_NVP(name_);
            ar & BOOST_SERIALIZATION_NVP(preconditions_);
            ar & BOOST_SERIALIZATION_NVP(effects_);
            ar & BOOST_SERIALIZATION_NVP(motion_sequences_);
        }
    };
}

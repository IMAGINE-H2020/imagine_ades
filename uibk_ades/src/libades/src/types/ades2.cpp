#include "../../include/types/ades2.h"
#include "../../include/types/motion_sequence2.h"

using namespace std;

namespace ades {

    Ades2::Ades2(string name,
               map<string, string> preconditions,
               map<string, string> effects,
               map<string, MotionSequence2> motions
        ) : ID(reinterpret_cast<uint64_t>(&name)),
            name_(name),
            preconditions_(preconditions),
            effects_(effects),
            motion_sequences_(motions)
    {}

    Ades2::~Ades2()
    {}

    void Ades2::insertPreconditions(const map<string, string> conditions)
    {
        for(auto cond : conditions)
        {
            preconditions_.insert(pair<string, string>(cond.first, cond.second));
        }
    }

    void Ades2::removePreconditions(const vector<string> conditions)
    {
        if(conditions.empty())
        {
            preconditions_.clear();
        }
        else
        {
            for(auto cond : conditions)
            {
                preconditions_.erase(cond);
            }
        }
    }

    void Ades2::modifyPreconditions(const map<string, string> conditions)
    {
        for(auto cond : conditions)
        {
            preconditions_.erase(cond.first);
            preconditions_.insert(pair<string, string>(cond.first, cond.second));
        }
    }

    void Ades2::insertEffects(const map<string, string> effects)
    {
        for(auto effect : effects)
        {
            effects_.insert(pair<string, string>(effect.first, effect.second));
        }
    }

    void Ades2::removeEffects(const vector<string> effects)
    {
        if(effects.empty())
        {
            effects_.clear();
        }
        else
        {
            for(auto effect : effects)
            {
                effects_.erase(effect);
            }
        }
    }

    void Ades2::modifyEffects(const map<string, string> effects)
    {
        for(auto effect : effects)
        {
            effects_.erase(effect.first);
            effects_.insert(pair<string, string>(effect.first, effect.second));
        }
    }


    void Ades2::insertMotionSequence(const std::string motionSequenceID,
                                    const MotionSequence2 motionSequence)
    {
        motion_sequences_.insert(pair<string, MotionSequence2>(motionSequenceID, motionSequence));
    }

    void Ades2::removeMotionSequence(const std::string motionSequenceID)
    {
        motion_sequences_.erase(motionSequenceID);
    }

    MotionSequence2 *Ades2::modifyMotionSequence(const std::string motionSequenceID)
    {
        return &(motion_sequences_.at(motionSequenceID));
    }
}

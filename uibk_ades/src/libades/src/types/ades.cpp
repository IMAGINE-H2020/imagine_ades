#include "../../include/types/ades.h"
#include "../../include/types/motion_sequence.h"

using namespace std;

namespace ades {

    Ades::Ades(string name,
               map<string, string> preconditions,
               map<string, string> effects,
               map<string, MotionSequence> motions
        ) : ID(reinterpret_cast<uint64_t>(&name)),
            name_(name),
            preconditions_(preconditions),
            effects_(effects),
            motion_sequences_(motions)
    {}

    Ades::~Ades()
    {}

    void Ades::insertPreconditions(const map<string, string> conditions)
    {
        for(auto cond : conditions)
        {
            preconditions_.insert(pair<string, string>(cond.first, cond.second));
        }
    }

    void Ades::removePreconditions(const vector<string> conditions)
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

    void Ades::modifyPreconditions(const map<string, string> conditions)
    {
        for(auto cond : conditions)
        {
            preconditions_.erase(cond.first);
            preconditions_.insert(pair<string, string>(cond.first, cond.second));
        }
    }

    void Ades::insertEffects(const map<string, string> effects)
    {
        for(auto effect : effects)
        {
            effects_.insert(pair<string, string>(effect.first, effect.second));
        }
    }

    void Ades::removeEffects(const vector<string> effects)
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

    void Ades::modifyEffects(const map<string, string> effects)
    {
        for(auto effect : effects)
        {
            effects_.erase(effect.first);
            effects_.insert(pair<string, string>(effect.first, effect.second));
        }
    }


    void Ades::insertMotionSequence(const std::string motionSequenceID,
                                    const MotionSequence motionSequence)
    {
        motion_sequences_.insert(pair<string, MotionSequence>(motionSequenceID, motionSequence));
    }

    void Ades::removeMotionSequence(const std::string motionSequenceID)
    {
        motion_sequences_.erase(motionSequenceID);
    }

    MotionSequence *Ades::modifyMotionSequence(const std::string motionSequenceID)
    {
        return &(motion_sequences_.at(motionSequenceID));
    }
}

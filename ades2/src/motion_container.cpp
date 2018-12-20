//
// Created by Philipp Zech on 9/24/18.
//

#include "../include/motion_container.h"

using namespace std;

namespace ades {

    MotionContainer::MotionContainer(string name) noexcept :
        name_{name}
    {
        id_ = hash<string>{}(name_);
    };

    MotionContainer::MotionContainer(string name, MotionType type_) noexcept :
        name_{name},
        type_(type_)
    {
         id_ = hash<string>{}(name_);
    };


    string MotionContainer::getName() const noexcept {
        return name_;
    };


    ull MotionContainer::getId() const noexcept {
        return id_;
    };

    void MotionContainer::setMotionType(ades::MotionType type) noexcept {
        type_ = type;
    };


    MotionType MotionContainer::getMotionType() const noexcept {
        return type_;
    };


    EffectModel* MotionContainer::getEffectModel() noexcept {
        return &effectModel_;
    };


    //updates or inserts
    bool MotionContainer::addParameter(map<string, string> parameters) noexcept {
        for (auto param : parameters) {
            parameters_.insert_or_assign(param.first, param.second);
        }
        return true;
    };


    vector<bool> MotionContainer::deleteParameter(vector<string> parameters) noexcept {
        auto deleted = vector<bool>{};
        for(auto param : parameters) {
            auto it = parameters_.find(param);
            if (it != parameters_.end()) {
                parameters_.erase(it);
                deleted.push_back(true);
            } else {
                deleted.push_back(false);
            }
        }
        return deleted;
    };


    map<string, string> MotionContainer::getParameters() noexcept {
        return parameters_;
    };


    Trajectory MotionContainer::run(const ades::TrajectoryPoint& start, const ades::TrajectoryPoint& goal, int num_timestamps) {
        throw runtime_error("No default implementation from base class available!") ;
    };


    void MotionContainer::train(const std::vector<Trajectory> &trajectories) {
        throw runtime_error("No default implementation from base class available!") ;
    };


    Trajectory MotionContainer::simulate(const ades::TrajectoryPoint& start, const ades::TrajectoryPoint& goal, int num_timestamps) {
        throw runtime_error("No default implementation from base class available!") ;
    }


};

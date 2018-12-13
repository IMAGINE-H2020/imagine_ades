//
// Created by Philipp Zech on 9/24/18.
//

#include "../include/action_container.h"

namespace ades {

    ActionContainer::ActionContainer(string name) noexcept :
        name_{name}
    {
        id_ = hash<string>{}(name_);
    };


    string ActionContainer::getName() const noexcept {
        return name_;
    };


    ull ActionContainer::getId() const noexcept {
        return id_;
    };


    bool ActionContainer::addMotionContainer(vector<MotionContainer*> motions) noexcept {
        for (auto container : motions) {
            motions_.push_back(make_shared<MotionContainer>(*container));
        }
        return true;
    };


    vector<shared_ptr<MotionContainer>> ActionContainer::getMotions() noexcept {
        return motions_;
    };


    shared_ptr<MotionContainer> ActionContainer::getMotionContainer(variant<ull, string> identifier) {
        if (holds_alternative<ull>(identifier)) {
            auto id = get<ull>(identifier);
            auto it = find_if(motions_.begin(), motions_.end(), [id](auto motion) { return motion->getId() == id; });
            if (it != motions_.end()) {
                return *it;
            }
        } else if (holds_alternative<string>(identifier)) {
            auto name = get<string>(identifier);
            auto it = find_if(motions_.begin(), motions_.end(), [name](auto motion) { return motion->getName() == name; });
            if (it != motions_.end()) {
                return *it;
            }
        } else {
            throw runtime_error("Empty identifier in variant!");
        }
        return nullptr;
    };


    vector<bool> ActionContainer::deleteMotionContainer(vector<variant<ull, string>> identifiers) {
        auto deleted = vector<bool>{};
        for(variant<ull, string> identifier : identifiers) {
            if (holds_alternative<string>(identifier)) {
                auto name = get<string>(identifier);
                auto it = remove_if(motions_.begin(), motions_.end(),
                                    [name](auto motion_) { return name == motion_->getName(); });
                if (it != motions_.end()) {
                    motions_.erase(it, motions_.end());
                    deleted.push_back(true);
                } else {
                    deleted.push_back(false);
                }
            } else if (holds_alternative<ull>(identifier)) {
                auto id = get<ull>(identifier);
                auto it = remove_if(motions_.begin(), motions_.end(),
                                    [id](auto motion) { return id == motion->getId(); });
                if (it != motions_.end()) {
                    motions_.erase(it, motions_.end());
                    deleted.push_back(true);
                } else {
                    deleted.push_back(false);
                }
            } else {
                throw runtime_error("Empty identifier in variant!");
            }
        }
        return deleted;
    };


    bool ActionContainer::updateMotionContainer(MotionContainer* container) noexcept {
        auto it = remove_if(motions_.begin(), motions_.end(), [container](auto container_){
            return container->getId() == container_->getId() && container->getName() == container_->getName();
        });
        if (it != motions_.end()) {
            motions_.erase(it, motions_.end());
            motions_.push_back(make_shared<MotionContainer>(*container));
            return true;
        } else {
            return false;
        }
    };


    //TODO implement
    vector<shared_ptr<MotionContainer>> ActionContainer::getBestFittingMotions(vector<string> observables, map<string, vector<variant<string, double>>> parameters) {
        throw runtime_error("Not yet implemented!");
    };

};

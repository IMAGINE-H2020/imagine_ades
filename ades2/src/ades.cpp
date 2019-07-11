//
// Created by Philipp Zech on 9/24/18.
//

#include "../include/ades.h"

using namespace std;


namespace ades {

    Ades::Ades() noexcept {};


    Ades::Ades(string name) noexcept :
        name_{name}
    {
        id_ = hash<string>{}(name_);
    };


    Ades::Ades(string name, clauses preconditions, clauses postconditions, clauses invariants) noexcept :
        name_{name},
        preconditions_{preconditions},
        postconditions_{postconditions},
        invariants_{invariants}
    {
        id_ = hash<string>{}(name_);
    };

    Ades::Ades(string name, TrainingType training) noexcept :
        name_{name},
        training_{training}
    {

    };

    string Ades::getName() const noexcept {
        return name_;
    };


    ull Ades::getId() const noexcept {
        return id_;
    };

    TrainingType Ades::getTrainingType() noexcept {
        return training_;
    };


    vector<bool> Ades::addPreconditions(clauses preconditions) noexcept {
        auto added = vector<bool>{};
        for (auto condition : preconditions) {
            if (find_if(preconditions_.begin(), preconditions_.end(),
                    [condition] (auto clause_) {return condition.first == clause_.first && condition.second == clause_.second;}) == preconditions_.end()) {
                preconditions_.push_back(condition);
                added.push_back(true);
            } else {
                added.push_back(false);
            }
        }
        return added;
    };


    vector<bool> Ades::deletePreconditions(clauses preconditions) noexcept {
        auto deleted = vector<bool>{};
        for (auto condition : preconditions) {
            auto it = remove_if(preconditions_.begin(), preconditions_.end(),
                    [condition] (auto clause_) {return condition.first == clause_.first && condition.second == clause_.second;});
            if (it != preconditions.end()) {
                preconditions_.erase(it, preconditions_.end());
                deleted.push_back(true);
            } else {
                deleted.push_back(false);
            }
        }
        return deleted;
    };


    clauses Ades::getPreconditions() const noexcept {
        return preconditions_;
    };


    vector<bool> Ades::addPostconditions(clauses postconditions) noexcept {
        auto added = vector<bool>{};
        for (auto condition : postconditions) {
            if (find_if(postconditions_.begin(), postconditions_.end(),
                    [condition] (auto clause_) {return condition.first == clause_.first && condition.second == clause_.second;}) == postconditions_.end()) {
                postconditions_.push_back(condition);
                added.push_back(true);
            } else {
                added.push_back(false);
            }
        }
        return added;
    };


    vector<bool> Ades::deletePostconditions(clauses postconditions) noexcept {
        auto deleted = vector<bool>{};
        for (auto condition : postconditions) {
            auto it = remove_if(postconditions_.begin(), postconditions_.end(),
                    [condition] (auto clause_) {return condition.first == clause_.first && condition.second == clause_.second;});
            if (it != postconditions.end()) {
                postconditions_.erase(it, postconditions_.end());
                deleted.push_back(true);
            } else {
                deleted.push_back(false);
            }
        }
        return deleted;
    };


    clauses Ades::getPostconditions() const noexcept {
        return postconditions_;
    };


    vector<bool> Ades::addInvariants(clauses invariants) noexcept {
        auto added = vector<bool>{};
        for (auto invariant : invariants) {
            if (find_if(invariants_.begin(), invariants_.end(),
                    [invariant] (auto clause_) {return invariant.first == clause_.first && invariant.second == clause_.second;}) == invariants_.end()) {
                invariants_.push_back(invariant);
                added.push_back(true);
            } else {
                added.push_back(false);
            }
        }
        return added;
    };


    vector<bool> Ades::deleteInvariants(clauses invariants) noexcept {
        auto deleted = vector<bool>{};
        for (auto invariant : invariants) {
            auto it = remove_if(invariants_.begin(), invariants_.end(),
                    [invariant] (auto clause_) {return invariant.first == clause_.first && invariant.second == clause_.second;});
            if (it != invariants.end()) {
                invariants_.erase(it, invariants_.end());
                deleted.push_back(true);
            } else {
                deleted.push_back(false);
            }
        }
        return deleted;
    };


    clauses Ades::getInvariants() const noexcept {
        return invariants_;
    };


    vector<bool> Ades::addActionContainer(vector<ActionContainer*> containers) noexcept {
        auto added = vector<bool>{};
        for (auto container : containers) {
            if (find_if(actions_.begin(), actions_.end(),
                    [& container] (auto action_) {return container->getId() == action_->getId();}) == actions_.end()) {
                actions_.push_back(make_shared<ActionContainer>(*container));
                added.push_back(true);
            } else {
                added.push_back(false);
            }
        }
        return added;
    };


    vector<shared_ptr<ActionContainer>> Ades::getActionContainers() noexcept {
        return actions_;
    };


    shared_ptr<ActionContainer> Ades::getActionContainer(variant<string, ull> identifier) {
        if (holds_alternative<string>(identifier)) {
            auto name = get<string>(identifier);
            auto it = find_if(actions_.begin(), actions_.end(),
                    [name] (auto action_) {return name == action_->getName();});
            if (it != actions_.end()) {
                return *it;
            }
        } else if (holds_alternative<ull>(identifier)) {
            auto id = get<ull>(identifier);
            auto it = find_if(actions_.begin(), actions_.end(),
                    [id] (auto action_) {return id == action_->getId();});
            if (it != actions_.end()) {
                return *it;
            }
        } else {
            throw runtime_error("Empty identifier in variant!");
        }
        return nullptr;
    };


    vector<bool> Ades::deleteActionContainer(vector<variant<string, ull>> identifiers) {
        auto deleted = vector<bool>{};
        for(auto identifier : identifiers) {
            if (holds_alternative<string>(identifier)) {
                auto name = get<string>(identifier);
                auto it = remove_if(actions_.begin(), actions_.end(),
                                    [name](auto action_) { return name == action_->getName(); });
                if (it != actions_.end()) {
                    actions_.erase(it, actions_.end());
                    deleted.push_back(true);
                } else {
                    deleted.push_back(false);
                }
            } else if (holds_alternative<ull>(identifier)) {
                auto id = get<ull>(identifier);
                auto it = remove_if(actions_.begin(), actions_.end(),
                                    [id](auto action_) { return id == action_->getId(); });
                if (it != actions_.end()) {
                    actions_.erase(it, actions_.end());
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


    //TODO implement
    const vector<pair<shared_ptr<ActionContainer>, shared_ptr<MotionContainer>>> Ades::getBestFittingActions(vector<string> observables, map<string, vector<variant<string, double>>> parameters) {
        throw runtime_error("Not yet implemented!");
    };

};

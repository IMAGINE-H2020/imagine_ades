//
// Created by Philipp Zech on 9/24/18.
//

#ifndef ADES2_ADES_H
#define ADES2_ADES_H

#pragma once

#include <string>
#include <vector>
#include <variant>
#include <map>
#include <utility>
#include <functional>
#include "../include/action_container.h"
#include "../include/motion_container.h"

using namespace std;

using head =  string;
using body = string;
using clause = pair<head, body>;
using clauses = vector<clause>;
using ull = unsigned long long;

namespace ades {

    enum class TrainingType {DEMO, SIM};

    class Ades {

    private:
        string name_;

        TrainingType training_;

        ull id_;

        clauses preconditions_;

        clauses postconditions_;

        clauses invariants_;

        vector<shared_ptr<ActionContainer>> actions_;

    public:

        explicit Ades() noexcept;

        explicit Ades(string name) noexcept;

        Ades(string name, TrainingType training) noexcept;

        Ades(string name, clauses preconditions, clauses postconditions, clauses invariants) noexcept;

        ~Ades() noexcept = default;

        Ades(Ades&&) = default;

        Ades& operator=(Ades&&) = default;

        Ades(const Ades&) = default;

        Ades& operator=(const Ades&) = default;

        string getName() const noexcept;

        ull getId() const noexcept;

        TrainingType getTrainingType() noexcept;

        vector<bool> addPreconditions(clauses preconditions) noexcept;

        vector<bool> deletePreconditions(clauses preconditions) noexcept;

        clauses getPreconditions() const noexcept;

        vector<bool> addPostconditions(clauses postconditions) noexcept;

        vector<bool> deletePostconditions(clauses postconditions) noexcept;

        clauses getPostconditions() const noexcept;

        vector<bool> addInvariants(clauses invariants) noexcept;

        vector<bool> deleteInvariants(clauses invariants) noexcept;

        clauses getInvariants() const noexcept;

        vector<bool> addActionContainer(vector<ActionContainer*> containers) noexcept;

        vector<shared_ptr<ActionContainer>> getActionContainers() noexcept;

        shared_ptr<ActionContainer> getActionContainer(variant<string, ull> identifier);

        vector<bool> deleteActionContainer(vector<variant<string, ull>> identifiers);

        const vector<pair<shared_ptr<ActionContainer>, shared_ptr<MotionContainer>>> getBestFittingActions(vector<string> observables, map<string, vector<variant<string, double>>> parameters);

    };

}

#endif //ADES2_ADES_H

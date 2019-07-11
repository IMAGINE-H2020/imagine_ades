//
// Created by Philipp Zech on 9/24/18.
//

#ifndef ADES2_ACTION_CONTAINER_H
#define ADES2_ACTION_CONTAINER_H

#pragma once

#include <string>
#include <vector>
#include <variant>
#include <memory>
#include <map>
#include <algorithm>
#include "../include/motion_container.h"

using namespace std;

using ull =  unsigned long long;

namespace ades {

    class ActionContainer {
    private:
        string name_;

        ull id_;

        vector<shared_ptr<MotionContainer>> motions_;

    public:
        explicit ActionContainer(string name) noexcept;

        ~ActionContainer() noexcept = default;

        ActionContainer(ActionContainer&&) = default;

        ActionContainer& operator=(ActionContainer&&) = default;

        ActionContainer(const ActionContainer&) = default;

        ActionContainer& operator=(const ActionContainer&) = default;

        string getName() const noexcept;

        ull getId() const noexcept;

        bool addMotionContainer(vector<MotionContainer*> motions) noexcept;

        vector<shared_ptr<MotionContainer>> getMotions() noexcept;

        shared_ptr<MotionContainer> getMotionContainer(variant<ull, string> identifier);

        vector<bool> deleteMotionContainer(vector<variant<ull, string>> identifiers);

        bool updateMotionContainer(MotionContainer* container) noexcept;

        vector<shared_ptr<MotionContainer>> getBestFittingMotions(vector<string> observables, map<string, vector<variant<string, double>>> parameters);

    };

};

#endif //ADES2_ACTION_CONTAINER_H

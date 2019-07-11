//
// Created by Philipp Zech on 9/24/18.
//

#ifndef ADES2_MOTIONCONTAINER_H
#define ADES2_MOTIONCONTAINER_H

#pragma once

#include <string>
#include <vector>
#include <variant>
#include <map>
#include "effect_model.h"

using ull = unsigned long long;

using namespace std;

namespace ades {

    enum class MotionType {DMP, CPG, TRAJ};

    class MotionContainer {
    private:
        string name_;

        ull id_;

        EffectModel effectModel_;

        MotionType type_;

        map<string, string> parameters_;

    public:
        explicit MotionContainer(string name) noexcept;

        MotionContainer(string name, MotionType type) noexcept;

        ~MotionContainer() noexcept = default;

        MotionContainer(MotionContainer&&) = default;

        MotionContainer& operator=(MotionContainer&&) = default;

        MotionContainer(const MotionContainer&) = default;

        MotionContainer& operator=(const MotionContainer&) = default;

        string getName() const noexcept;

        ull getId() const noexcept;

        void setMotionType(MotionType type) noexcept;

        MotionType getMotionType() const noexcept;

        EffectModel* getEffectModel() noexcept;

        bool addParameter(map<string, string> parameters) noexcept;

        vector<bool> deleteParameter(vector<string>) noexcept;

        map<string, string> getParameters() noexcept;

        virtual void run(const char* fmt ...);

        virtual void train(const char* fmt ...);

        virtual void simulate(const char* fmt ...);

    };

};

#endif //ADES2_MOTIONCONTAINER_H

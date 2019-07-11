//
// Created by Philipp Zech on 9/24/18.
//

#ifndef ADES2_EFFECT_MODEL_H
#define ADES2_EFFECT_MODEL_H

#pragma once

#include <string>
#include <memory>
#include <map>
#include <any>

using namespace std;

namespace ades {

    class EffectModel {

    private:
        any model_;

    public:
        explicit EffectModel() noexcept;

        EffectModel(any model) noexcept;

        ~EffectModel() noexcept = default;

        EffectModel(EffectModel&&) = default;

        EffectModel& operator=(EffectModel&&) = default;

        EffectModel(const EffectModel&) = default;

        const type_info& getType() const noexcept;

        EffectModel& operator=(const EffectModel&) = default;

        bool setEffectModel(any model) noexcept;

        any getEffectModel() noexcept;

    };

};

#endif //ADES2_EFFECT_MODEL_H

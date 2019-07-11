//
// Created by Philipp Zech on 9/24/18.
//

#include "../include/effect_model.h"

namespace ades {
    EffectModel::EffectModel() noexcept {

    };


    EffectModel::EffectModel(any model) noexcept :
        model_{model}
    {
    };


    const type_info& EffectModel::getType() const noexcept {
        return model_.type();
    };


    bool EffectModel::setEffectModel(any model) noexcept {
        model_ = move(model);
    };


    any EffectModel::getEffectModel() noexcept {
        return model_;
    };

};

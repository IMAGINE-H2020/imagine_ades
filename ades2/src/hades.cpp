//
// Created by Philipp Zech on 9/24/18.
//

#include "../include/hades.h"

namespace ades {

    Hades::Hades(string name) noexcept :
        name_{name}
    {
        id_ = hash<string>{}(name_);
    };


    string Hades::getName() const noexcept {
        return name_;
    };


    ull Hades::getId() const noexcept {
        return id_;
    };


    vector<shared_ptr<Ades>> Hades::getHades() noexcept {
        return hades_;
    };


    bool Hades::addAdes(Ades* ades, unsigned int position) noexcept {
        if (position < hades_.size()) {
            hades_.insert(hades_.begin() + position, make_shared<Ades>(*ades));
            return true;
        }
        return false;
    };


    bool Hades::deleteAdes(unsigned int position) noexcept {
        hades_.erase(hades_.begin()+position);
        return true;
    };


    bool Hades::updateOrder(vector<variant<ull, string>> order) {
        auto copy{move(hades_)};
        hades_.clear();
        for (auto elem : order) {
            if (holds_alternative<ull>(elem)) {
                auto id = get<ull>(elem);
                auto it = find_if(copy.begin(), copy.end(), [id](auto ades) {return ades->getId() == id;});
                hades_.push_back((*it));
            } else if (holds_alternative<string>(elem)) {
                auto name = get<string>(elem);
                auto it = find_if(copy.begin(), copy.end(), [name](auto ades) {return ades->getName() == name;});
                hades_.push_back((*it));
            } else {
                throw runtime_error("Empty identified in variant!");
            }
        }
        return true;
    };


    vector<variant<ull, string>> Hades::getOrder() noexcept {
        auto order = vector<variant<ull, string>>{};
        for (auto ades : hades_) {
            order.push_back(variant<ull, string>{ades->getName()});
        }
        return order;
    };


    //what exactly do they need to return?
    clauses Hades::getPreconditions() const noexcept {
        return hades_.front()->getPreconditions();
    };


    clauses Hades::getPostconditions() const noexcept {
        return hades_.back()->getPostconditions();
    };


    clauses Hades::getInvariants() const noexcept {
        auto invariants = clauses{};
        for (auto ades : hades_) {
            clauses temp = ades->getInvariants();
            invariants.insert(invariants.end(), temp.begin(), temp.end());
        }
        return invariants;
    };

}

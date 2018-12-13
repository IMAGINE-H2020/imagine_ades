//
// Created by Philipp Zech on 9/24/18.
//

//TODO can we make this more elegant using CRTP??

#ifndef ADES2_HADES_H
#define ADES2_HADES_H

#pragma once

#include <vector>
#include <variant>
#include <memory>
#include "../include/ades.h"

namespace ades {

    class Hades {
    private:
        string name_;

        ull id_;

        vector<shared_ptr<Ades>> hades_;

    public:
        explicit Hades(string name) noexcept;

        ~Hades() noexcept = default;

        Hades(Hades&&) = default;

        Hades& operator=(Hades&&) = default;

        Hades(const Hades&) = default;

        Hades& operator=(const Hades&) = default;

        string getName() const noexcept;

        ull getId() const noexcept;

        vector<shared_ptr<Ades>> getHades() noexcept;

        bool addAdes(Ades* ades, unsigned int position) noexcept;

        bool deleteAdes(unsigned int position) noexcept;

        bool updateOrder(vector<variant<ull, string>> order);

        vector<variant<ull, string>> getOrder() noexcept;

        //what exactly do they need to return?
        clauses getPreconditions() const noexcept;

        clauses getPostconditions() const noexcept;

        clauses getInvariants() const noexcept;

    };

}

#endif //ADES2_HADES_H

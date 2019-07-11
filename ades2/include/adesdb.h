//
// Created by Philipp Zech on 9/24/18.
//

#ifndef ADES2_ADESDB_H
#define ADES2_ADESDB_H

#pragma once

#include <string>
#include <vector>
#include <iterator>
#include <variant>
#include <tuple>
#include <iostream>

#include <xdevapi.h>

#include "ades.h"
#include "hades.h"


using namespace std;

namespace ades {

    class AdesDB {
    private:
        string location_;

        vector <Ades> adeses_;

        vector <Hades> hadeses_;

        bool serialize() noexcept;

        bool deserialize() noexcept;

    public:
        explicit AdesDB(string location) noexcept;

        ~AdesDB() noexcept;

        AdesDB(AdesDB&&) = default;

        AdesDB& operator=(AdesDB&&) = default;

        AdesDB(const AdesDB&) = default;

        AdesDB& operator=(const AdesDB&) = default;

        const string getLocation() noexcept;

        //Ades handling
        vector<bool> addAdes(vector<Ades> ades);

        vector<Ades> listAdes() noexcept;

        const Ades* getAdes(variant<string, ull> identifier);

        vector<bool> deleteAdes(vector<variant<string, ull>> identifiers);

        bool updateAdes(Ades ades);

        bool containsAdes(variant<string, ull> identifier);

        //Hades handling
        tuple<bool, ull, string> is_referenced_by_hades(variant<string, ull> identifier);

        vector<bool> addHades(vector<Hades> hadeses);

        vector<Hades> listHades() noexcept;

        const Hades* getHades(variant<string, ull> identifier);

        vector<bool> deleteHades(vector<variant<string, ull>> identifiers);

        bool updateHades(Hades ades);

        bool containsHades(variant<string, ull> identifier);

    };

};

#endif //ADES2_ADESDB_H

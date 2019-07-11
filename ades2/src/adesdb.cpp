//
// Created by Philipp Zech on 9/24/18.
//

#include "../include/adesdb.h"


namespace ades {

    AdesDB::AdesDB(string location) noexcept : location_(location) {
        deserialize();
    }


    AdesDB::~AdesDB() noexcept {
        serialize();
    };


    bool AdesDB::deserialize() noexcept {
        return false;
    };


    bool AdesDB::serialize() noexcept {
        return false;
    };


    const string AdesDB::getLocation() noexcept {
        return location_;
    }


    vector<bool> AdesDB::addAdes(vector<Ades> adeses) {
        auto added = vector<bool>{};
        for (auto ades : adeses) {
            if (!containsAdes(variant<string,ull>{ades.getName()})) {
                adeses_.push_back(ades);
                added.push_back(true);
            } else {
                added.push_back(false);
            }
        }
        return added;
    };


    vector<Ades> AdesDB::listAdes() noexcept {
        return adeses_;
    };


    const Ades *AdesDB::getAdes(variant<string, ull> identifier) {
        if (holds_alternative<ull>(identifier)) {
            auto id = get<ull>(identifier);
            auto it = find_if(adeses_.begin(), adeses_.end(), [id](auto elem) { return elem.getId() == id; });
            if (it != adeses_.end()) {
                return &(*it);
            }
        } else if (holds_alternative<string>(identifier)) {
            auto name = get<string>(identifier);
            auto it = find_if(adeses_.begin(), adeses_.end(), [name](auto elem) { return elem.getName() == name; });
            if (it != adeses_.end()) {
                return &(*it);
            }
        } else {
            throw runtime_error("Empty identifier in variant!");
        }
        return nullptr;
    };


    vector<bool> AdesDB::deleteAdes(vector<variant<string, ull>> identifiers) {
        auto deleted = vector<bool>{};
        for (auto identifier : identifiers) {
            auto referenced = is_referenced_by_hades(identifier);
            if (holds_alternative<ull>(identifier)) {
                auto id = get<ull>(identifier);
                if (get<bool>(referenced)) {
                    cerr << "Cannot delete ADES [" << id << "] as of being referenced by HADES ["
                         << get<string>(referenced) << "]." << endl;
                } else {
                    auto it = remove_if(adeses_.begin(), adeses_.end(), [id](auto elem) { return elem.getId() == id; });
                    if (it != adeses_.end()) {
                        adeses_.erase(it, adeses_.end());
                        deleted.push_back(true);
                    } else {
                        deleted.push_back(false);
                    }
                }
            } else if (holds_alternative<string>(identifier)) {
                auto name = get<string>(identifier);
                if (get<bool>(referenced)) {
                    cerr << "Cannot delete ADES [" << name << "] as of being referenced by HADES ["
                         << get<string>(referenced) << "]." << endl;
                } else {
                    auto it = remove_if(adeses_.begin(), adeses_.end(),
                                        [name](auto elem) { return elem.getName() == name; });
                    if (it != adeses_.end()) {
                        adeses_.erase(it, adeses_.end());
                        deleted.push_back(true);
                    } else {
                        deleted.push_back(false);
                    }
                }
            } else {
                throw runtime_error("Empty identifier in variant!");
            }
        }
        return deleted;
    };


    bool AdesDB::updateAdes(Ades ades) {
        if (containsAdes(variant<string,ull>{ades.getName()})) {
            deleteAdes(vector<variant<string,ull>>{variant < string, ull > {ades.getName()}});
            adeses_.push_back(ades);
            auto referenced = is_referenced_by_hades(variant<string, ull>{ades.getName()});
            if (get<bool>(referenced)) {
                auto ref = getAdes(variant<string,ull>(ades.getName()));
                auto to_modify = *getHades(variant<string,ull>{get<string>(referenced)});
                deleteHades(vector<variant<string,ull>>{variant<string,ull>{to_modify.getName()}});
                for(auto ades_ : to_modify.getHades()) {
                    if (ades_->getName() == ref->getName() && ades_->getId() == ref->getId()) {
                        ades_ = make_shared<Ades>(*ref);
                        break;
                    }
                }
                addHades(vector<Hades>{to_modify});
            }
            return true;
        }
        return false;
    };


    bool AdesDB::containsAdes(variant<string, ull> identifier) {
        if (holds_alternative<ull>(identifier)) {
            auto id = get<ull>(identifier);
            auto it = find_if(adeses_.begin(), adeses_.end(), [id](auto elem) { return elem.getId() == id; });
            return it != adeses_.end();
        } else if (holds_alternative<string>(identifier)) {
            auto name = get<string>(identifier);
            auto it = find_if(adeses_.begin(), adeses_.end(), [name](auto elem) { return elem.getName() == name; });
            return it != adeses_.end();
        } else {
            throw runtime_error("Empty identifier in variant!");
        }
    };


    tuple<bool, ull, string> AdesDB::is_referenced_by_hades(variant<string, ull> identifier) {
        for (auto hades : hadeses_) {
            for (auto ades_ : hades.getHades()) {
                if (holds_alternative<string>(identifier)) {
                    auto name = get<string>(identifier);
                    if (ades_->getName() == name) {
                        return tuple<bool,ull,string>{true, hades.getId(), hades.getName()};
                    }
                } else if (holds_alternative<ull>(identifier)) {
                    auto id = get<ull>(identifier);
                    if (ades_->getId() == id) {
                        return tuple<bool, ull, string>{true, hades.getId(), hades.getName()};
                    }
                } else {
                    throw runtime_error("Empty identifier in variant!");
                }
            }
        }
        return tuple<bool, int, string>{false, -1, ""};
    };


    vector<bool> AdesDB::addHades(vector<Hades> hadeses) {
        auto added = vector<bool>{};
        for (auto hades : hadeses) {
            if (!containsHades(variant < string, ull > {hades.getName()})) {
                hadeses_.push_back(hades);
                added.push_back(true);
            } else {
                added.push_back(false);
            }
        }
        return added;
    };


    vector<Hades> AdesDB::listHades() noexcept {
        return hadeses_;
    };


    const Hades *AdesDB::getHades(variant<string, ull> identifier) {
        if (holds_alternative<ull>(identifier)) {
            auto id = get<ull>(identifier);
            auto it = find_if(hadeses_.begin(), hadeses_.end(), [id](auto elem) { return elem.getId() == id; });
            if (it != hadeses_.end()) {
                return &(*it);
            }
        } else if (holds_alternative<string>(identifier)) {
            auto name = get<string>(identifier);
            auto it = find_if(hadeses_.begin(), hadeses_.end(), [name](auto elem) { return elem.getName() == name; });
            if (it != hadeses_.end()) {
                return &(*it);
            }
        } else {
            throw runtime_error("Empty identifier in variant!");
        }
        return nullptr;
    };


    vector<bool> AdesDB::deleteHades(vector<variant<string, ull>> identifiers) {
        auto deleted = vector<bool>{};
        for (auto identifier : identifiers) {
            if (holds_alternative<ull>(identifier)) {
                auto id = get<ull>(identifier);
                auto it = remove_if(hadeses_.begin(), hadeses_.end(), [id](auto elem) { return elem.getId() == id; });
                if (it != hadeses_.end()) {
                    hadeses_.erase(it, hadeses_.end());
                    deleted.push_back(true);
                } else {
                    deleted.push_back(false);
                }
            } else if (holds_alternative<string>(identifier)) {
                auto name = get<string>(identifier);
                auto it = remove_if(hadeses_.begin(), hadeses_.end(),
                                    [name](auto elem) { return elem.getName() == name; });
                if (it != hadeses_.end()) {
                    hadeses_.erase(it, hadeses_.end());
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


    bool AdesDB::updateHades(Hades hades) {
        if (containsHades(variant < string, ull > {hades.getName()})) {
            deleteHades(vector<variant<string, ull>>{variant < string, ull > {hades.getName()}});
            hadeses_.push_back(hades);
            return true;
        }
        return false;
    };


    bool AdesDB::containsHades(variant<string, ull> identifier) {
        if (holds_alternative<ull>(identifier)) {
            auto id = get<ull>(identifier);
            auto it = find_if(hadeses_.begin(), hadeses_.end(), [id](auto elem) { return elem.getId() == id; });
            return it != hadeses_.end();
        } else if (holds_alternative<string>(identifier)) {
            auto name = get<string>(identifier);
            auto it = find_if(hadeses_.begin(), hadeses_.end(), [name](auto elem) { return elem.getName() == name; });
            return it != hadeses_.end();
        } else {
            throw runtime_error("Empty identifier in variant!");
        }
    };

};
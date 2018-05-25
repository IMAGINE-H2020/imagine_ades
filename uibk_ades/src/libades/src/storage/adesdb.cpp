#include "../../include/storage/adesdb.h"

using namespace std;

std::string DB_DIR;

namespace ades {

    AdesDB::AdesDB(string home, unsigned int version) :
        home_(home),
        version_(version)
    {
        DB_DIR = home_;
        boost::filesystem::path path(home_.c_str());
        if (boost::filesystem::exists(path))
        {
            std::cout << "path:" << path << std::endl;
            populate();
        } else {
            boost::filesystem::create_directories(path);
        }
    }

    AdesDB::~AdesDB()
    {
        serialize();
    }

    bool AdesDB::populate()
    {
        std::string ext = ".xml";

        boost::filesystem::path path(home_.c_str());

        for(auto& entry : boost::make_iterator_range(boost::filesystem::directory_iterator(path), {}))
        {
            if(entry.path().extension().compare(ext) == 0)
            {
                Ades temp;
                std::ifstream ifs0(entry.path().c_str());
                boost::archive::xml_iarchive ia0(ifs0);
                ia0 >> BOOST_SERIALIZATION_NVP(temp);
                ades_.push_back(temp);
            }
        }
        return true;
    }

    bool AdesDB::serialize()
    {
        boost::filesystem::path path_to_remove(home_.c_str());
        for (boost::filesystem::directory_iterator end_dir_it, it(path_to_remove); it!=end_dir_it; ++it) {
            boost::filesystem::remove_all(it->path());
        }

        for (auto ades : ades_)
        {
            std::string path = home_ + "/" + ades.getName() + ".xml";
            std::ofstream ofs(path);
            boost::archive::xml_oarchive oa(ofs);
            oa << BOOST_SERIALIZATION_NVP(ades);
        }

        return true;
    }

    bool AdesDB::isInDB(std::string name)
    {
        bool isIn = false;
        for(auto a : listAdes())
        {
            if((isIn = (a.getName() == name )))
            {
                break;
            }
        }
        return isIn;
    }

    void AdesDB::addAdes(Ades ades)
    {
        string name = ades.getName();
        auto lambda = [name] (const Ades &a) { return a.getName() == name;};
        auto elem = find_if(ades_.begin(), ades_.end(), lambda);
        if(elem == ades_.end()) {
            ades_.push_back(ades);
        } else {
            cout << "Skipping ADES: " << ades.getName() << ". Please call" <<
                " updateAdes* if you want to update. " << endl;
        }
    }

    void AdesDB::addAdes(vector<Ades> ades)
    {
        for(auto ades__ : ades)
        {
            addAdes(ades__);
        }
    }

    //void AdesDB::removeAdesByName(string name)
    bool AdesDB::removeAdesByName(string name)
    {
        auto lambda = [name] (const Ades &a) { return a.getName() == name;};
        auto rem = remove_if(ades_.begin(), ades_.end(), lambda);
        auto result = ades_.erase(rem, ades_.end());
        return isInDB(name);
        //return result == ades_.end();
    }

    void AdesDB::removeAdesByID(uint64_t id)
    {
        auto lambda = [id] (const Ades &a) { return a.getID() == id;};
        auto rem = remove_if(ades_.begin(), ades_.end(), lambda);
        ades_.erase(rem, ades_.end());
    }

    std::vector<Ades>::iterator AdesDB::updateAdesByName(string name)
    {
        auto lambda = [name] (Ades &a) { return a.getName() == name;};
        auto elem = find_if(ades_.begin(), ades_.end(), lambda);
        return elem;
    }


    std::vector<Ades>::iterator AdesDB::updateAdesByID(uint64_t id) {
        auto lambda = [id] (const Ades &a) { return a.getID() == id;};
        auto elem = find_if(ades_.begin(), ades_.end(), lambda);
        return elem;
    }


    const Ades AdesDB::getAdesByName(string name)
    {
        auto lambda = [name] (Ades &a) { return a.getName() == name;};
        auto elem = find_if(ades_.begin(), ades_.end(), lambda);
        return *elem;
    }

    const Ades AdesDB::getAdesByID(uint64_t id)
    {
        auto lambda = [id] (const Ades &a) { return a.getID() == id;};
        auto elem = find_if(ades_.begin(), ades_.end(), lambda);
        return *elem;
    }
}

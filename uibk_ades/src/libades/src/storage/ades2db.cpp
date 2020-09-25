#include "../../include/storage/ades2db.h"

using namespace std;

std::string DB2_DIR;

namespace ades {

    Ades2DB::Ades2DB(string home, unsigned int version) :
        home_(home),
        version_(version)
    {
        DB2_DIR = home_;
        boost::filesystem::path path(home_.c_str());
        if (boost::filesystem::exists(path))
        {
            std::cout << "path:" << path << std::endl;
            populate();
        } else {
            boost::filesystem::create_directories(path);
        }
    }

    Ades2DB::~Ades2DB()
    {
        serialize();
    }

    bool Ades2DB::populate()
    {
        std::string ext = ".xml";

        boost::filesystem::path path(home_.c_str());

        for(auto& entry : boost::make_iterator_range(boost::filesystem::directory_iterator(path), {}))
        {
            if(entry.path().extension().compare(ext) == 0)
            {
                Ades2 temp;
                std::ifstream ifs0(entry.path().c_str());
                boost::archive::xml_iarchive ia0(ifs0);
                ia0 >> BOOST_SERIALIZATION_NVP(temp);
                ades_.push_back(temp);
            }
        }
        return true;
    }

    bool Ades2DB::serialize()
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

    bool Ades2DB::isInDB(std::string name)
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

    void Ades2DB::addAdes(Ades2 ades)
    {
        string name = ades.getName();
        auto lambda = [name] (const Ades2 &a) { return a.getName() == name;};
        auto elem = find_if(ades_.begin(), ades_.end(), lambda);
        if(elem == ades_.end()) {
            ades_.push_back(ades);
        } else {
            cout << "Skipping ADES: " << ades.getName() << ". Please call" <<
                " updateAdes* if you want to update. " << endl;
        }
    }

    void Ades2DB::addAdes(vector<Ades2> ades)
    {
        for(auto ades__ : ades)
        {
            addAdes(ades__);
        }
    }

    //void Ades2DB::removeAdesByName(string name)
    bool Ades2DB::removeAdesByName(string name)
    {
        auto lambda = [name] (const Ades2 &a) { return a.getName() == name;};
        auto rem = remove_if(ades_.begin(), ades_.end(), lambda);
        auto result = ades_.erase(rem, ades_.end());
        return isInDB(name);
        //return result == ades_.end();
    }

    void Ades2DB::removeAdesByID(uint64_t id)
    {
        auto lambda = [id] (const Ades2 &a) { return a.getID() == id;};
        auto rem = remove_if(ades_.begin(), ades_.end(), lambda);
        ades_.erase(rem, ades_.end());
    }

    std::vector<Ades2>::iterator Ades2DB::updateAdesByName(string name)
    {
        auto lambda = [name] (Ades2 &a) { return a.getName() == name;};
        auto elem = find_if(ades_.begin(), ades_.end(), lambda);
        return elem;
    }


    std::vector<Ades2>::iterator Ades2DB::updateAdesByID(uint64_t id) {
        auto lambda = [id] (const Ades2 &a) { return a.getID() == id;};
        auto elem = find_if(ades_.begin(), ades_.end(), lambda);
        return elem;
    }


    const Ades2 Ades2DB::getAdesByName(string name)
    {
        auto lambda = [name] (Ades2 &a) { return a.getName() == name;};
        auto elem = find_if(ades_.begin(), ades_.end(), lambda);
        return *elem;
    }

    const Ades2 Ades2DB::getAdesByID(uint64_t id)
    {
        auto lambda = [id] (const Ades2 &a) { return a.getID() == id;};
        auto elem = find_if(ades_.begin(), ades_.end(), lambda);
        return *elem;
    }
}

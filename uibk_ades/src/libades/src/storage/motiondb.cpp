#include "../../include/storage/motiondb.h"

using namespace std;

namespace ades {

    MotionDB::MotionDB(std::string home, unsigned int version) : home_(home), version_(version)
    {
        boost::filesystem::path path(home_.c_str());
        if (boost::filesystem::exists(path))
        {
            std::cout << "path:" << path << std::endl;
            populate();
        } else {
            boost::filesystem::create_directories(path);
        }
    }

    MotionDB::~MotionDB()
    {
        serialize();
    }
    
    bool MotionDB::populate()
    {
        std::string ext = ".xml";

        boost::filesystem::path path(home_.c_str());

        for(auto& entry : boost::make_iterator_range(boost::filesystem::directory_iterator(path), {}))
        {
            if(entry.path().extension().compare(ext) == 0)
            {
                Motion * temp;
                std::ifstream ifs0(entry.path().c_str());
                boost::archive::xml_iarchive ia0(ifs0);
                ia0.template register_type<DMPContainer>();
                ia0.template register_type<TrajectoryContainer>();
                ia0.template register_type<UnscrewContainer>();
                ia0 >> BOOST_SERIALIZATION_NVP(temp);
                motions_.insert(std::pair<std::string, const Motion *>(temp->getName(), temp));
            }
        }
        return true;
    }

    bool MotionDB::serialize()
    {
        boost::filesystem::path path_to_remove(home_.c_str());
        for (boost::filesystem::directory_iterator end_dir_it, it(path_to_remove); it!=end_dir_it; ++it)
        {
            boost::filesystem::remove_all(it->path());
        }
        
        std::cout << "Serializing data to " << home_ << std::endl;
        for (auto pair : motions_)
        {
            const Motion * motion = pair.second;
            std::string path = home_ + "/" + motion->getName() + ".xml";
            //std::cout << "Motion: " << motion->getName() << std::endl;
            //std::cout << "Type: " << typeid(motion).name() << std::endl;
            std::ofstream ofs(path);
            //std::cout << "At: " << path << std::endl;
            boost::archive::xml_oarchive oa(ofs);
            oa.template register_type<DMPContainer>();
            oa.template register_type<TrajectoryContainer>();
            oa.template register_type<UnscrewContainer>();
            //std::cout << "Archive ..."  << std::endl;
            oa << BOOST_SERIALIZATION_NVP(motion);
            //std::cout << "serialized!" << std::endl;
        }

        return true;
    }

    bool MotionDB::isInDB(std::string name)
    {
        return motions_.find( name ) != motions_.end();
    }

    Motion const * MotionDB::getMotion(std::string name)
    {
        if( isInDB(name) )
        {
            std::cout << "Motion available ! " << std::endl;
            return motions_.at(name);
        }
        else
        {
            std::cout << "NO SUCH Motion ! " << std::endl;
        }
        return NULL;
    }

    void MotionDB::insertMotion(const Motion *motion)
    {
        motions_.insert(std::pair<std::string, const Motion*>(motion->getName(), motion));
    }

    void MotionDB::removeMotion(std::string name)
    {
        motions_.erase(name);
    }

    Motion const *MotionDB::modifyMotion(std::string name)
    {
        return motions_.at(name);
        //return *motion_it;
        //return motions_.at(step);
    }

}

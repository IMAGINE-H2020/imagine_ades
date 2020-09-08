/* File : iis_motiondb.cpp
 * 
 * @description : This node implements the methods for environmentSIM class (robot action simulator) in the block push experiment.
*/

#include "../include/iis_motiondb.h"
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

using namespace ades;

Motiondb_ros::Motiondb_ros(ros::NodeHandle &nh, std::string home, int version):
	nh_(nh),
    database(home, version)
{
    ss_get_motion = nh_.advertiseService("motiondb/get_motion", &Motiondb_ros::get_motion_srv, this);
    ss_list_motion = nh_.advertiseService("motiondb/list_motion", &Motiondb_ros::list_motion_srv, this);
    ss_store_motion = nh_.advertiseService("motiondb/store_motion", &Motiondb_ros::store_motion_srv, this);
    ss_update_motion= nh_.advertiseService("motiondb/update_motion", &Motiondb_ros::update_motion_srv, this);
    ss_delete_motion= nh_.advertiseService("motiondb/delete_motion", &Motiondb_ros::delete_motion_srv, this);

    db_changed = nh_.advertise<imagine_common::KeyValPair>("motiondb/db_updated", 1);

    shutdown = false;
}

Motiondb_ros::~Motiondb_ros()
{
}

bool Motiondb_ros::list_motion_srv(imagine_common::ListMotion::Request &rq, imagine_common::ListMotion::Response &rp)
{
    std::cout << "Request for listing motion " << std::endl;

    for(auto motion_ : database.listMotion())
    {
        //rp.motion_list.push_back(motion_->getName());
        rp.motion_list.push_back(motion_.first);
    }

    return true;
}

bool Motiondb_ros::get_motion_srv(imagine_common::GetMotion::Request &rq, imagine_common::GetMotion::Response &rp)
{
    auto motion = database.getMotion(rq.name);
    
    if( motion != NULL)
    {
        imagine_common::Motion2 mt;

        std_msgs::Float64MultiArray motion_data;
        // Motion2 is (name, type, data)
        mt.type = typeToString(motion->getMotionType());
        mt.name = motion->getName();
        // this is a map of string, vector<double>
        auto params = motion->getMotionParameters();
        motion_data.layout = std_msgs::MultiArrayLayout(); // layout;
        motion_data.layout.data_offset = 0;
        motion_data.data = std::vector<double>(); // data;
        
        motion_data.layout.dim = std::vector<std_msgs:: MultiArrayDimension>(); // dims;
        for(auto dimension : params)
        {
            std_msgs::MultiArrayDimension dim_;
            dim_.label = dimension.first;
            dim_.size = dimension.second.size();
            dim_.stride = 0;
            motion_data.layout.dim.push_back(dim_);
            // Each motion parameter type
            for(auto val : dimension.second)
            {
                //std::cout << val << ", " << std::flush;
                motion_data.data.push_back((double)val);                        
            }
        }
        mt.data = (motion_data);
        rp.motion = mt;
        return true;
    }    
    else
    {
        rp.motion = imagine_common::Motion2();
        return false;
    }

}


bool Motiondb_ros::store_motion_srv(imagine_common::StoreMotion::Request &rq, imagine_common::StoreMotion::Response &rp)
{
    std::cout << "trying to STORE " << rq.motion.name << std::endl;
    bool result = false;
    auto motion = rq.motion;
    if( !(database.isInDB(motion.name)) )
    {
        Motion * newMotion;

        std::cout << motion.type << std::endl;
        // For now we stupidly switch-case for values:
        const std::string m_types[] = {"DMP", "Trajectory", "Unscrewing"};
        std::vector<std::string> m_types_(m_types, m_types + sizeof(m_types)  / sizeof(m_types[0]) );
        int this_type = find(m_types_.begin(), m_types_.end(), motion.type) - m_types_.begin();
        
        int data_index = 0;
        std::map<std::string, std::vector<double>> params;
        for(auto i : motion.data.layout.dim)
        {
            int dim_size = i.size;
            std::string dim_label = i.label;
            std::cout << "dim labl: " << i.label << std::endl;
            std::vector<double> values(motion.data.data.begin() + data_index, motion.data.data.begin() + data_index + dim_size);
            params.insert(std::pair<std::string, std::vector<double>>(dim_label, values));
            data_index += i.size;
        }
        std::vector<std::vector<double>> points;
        switch(this_type)
        {
            case 0: // DMP
                std::cout << "> DMP" << std::endl;
                newMotion = new DMPContainer(params["K"],params["D"],params["weights"],params["psiMatrix"]);
            break;
            case 1: // Trajectory
                std::cout << "> Trajectory" << std::endl;
                for(auto p : params)
                {
                    points.push_back(p.second);
                }
                newMotion = new TrajectoryContainer(points);
            break;
            case 2: // Unscrewing
                std::cout << "> Unscrewing" << std::endl;
                newMotion = new UnscrewContainer();
            break;
            default:
                std::cout << "This motion is not implemented yet ; It has not been added to the database. " << std::endl;
            break;
        }
        newMotion->setTemporalScale(1.0);
        newMotion->setName(motion.name);
                
        database.insertMotion(newMotion);
        std::cout << "Motion nb : " << database.getMotionNb() << std::endl;
        result = (database.isInDB(motion.name));
    }
    else
    {
        std::cout << "Existing motion in the DB ; change the name or use the update service " << std::endl;
        result = false;
    }
    
    rp.success = result;
    imagine_common::KeyValPair kv;
    kv.key = rq.motion.name;
    kv.value = "stored";
    db_changed.publish(kv);

    return true;
}

bool Motiondb_ros::update_motion_srv(imagine_common::UpdateMotion::Request &rq, imagine_common::UpdateMotion::Response &rp)
{
    bool result = false;

    // string motion_name
    // Motion2 motion
    std::cout << "trying to UPDATE " << rq.motion_name << std::endl;
    // This service is mostly a copy paste of store ; some refactoring will be needed
    // "name" in the request is the target (needs to exist)
    // "motion.{motion_name, *}" is the new content (needs to be filled)
    if( (database.isInDB(rq.motion_name)) )
    {
        // get the motion to update
        auto motion_to_update = database.modifyMotion(rq.motion_name);
        // put the content of the message into a variable (no need)
        auto motion = rq.motion;

        // We create an instance of the proper motion class
        Motion * newMotion;
        std::cout << motion.type << std::endl;

        const std::string m_types[] = {"DMP", "Trajectory", "Unscrewing"};
        std::vector<std::string> m_types_(m_types, m_types + sizeof(m_types)  / sizeof(m_types[0]) );
        int this_type = find(m_types_.begin(), m_types_.end(), motion.type) - m_types_.begin();
            
        int data_index = 0;
        std::map<std::string, std::vector<double>> params;
        for(auto i : rq.motion.data.layout.dim)
        {
            int dim_size = i.size;
            std::string dim_label = i.label;
            std::vector<double> values(motion.data.data.begin() + data_index, motion.data.data.begin() + data_index + dim_size);
            params.insert(std::pair<std::string, std::vector<double>>(dim_label, values));
            data_index += i.size;
        }
        std::vector<std::vector<double>> points;
        switch(this_type)
        {
            case 0: // DMP
                std::cout << "> DMP" << std::endl;
                newMotion = new DMPContainer(params["K"],params["D"],params["weights"],params["psiMatrix"]);
              break;
                case 1: // Trajectory
		      std::cout << "> Trajectory" << std::endl;
		      for(auto p : params)
		      {
		      	points.push_back(p.second);
		      }
		      newMotion = new TrajectoryContainer(points);
                break;
                case 2: // Unscrewing
                    std::cout << "> Unscrewing" << std::endl;
                    newMotion = new UnscrewContainer();
                break;
                default:
                    std::cout << "This motion is not implemented yet ; It has not been added to the sequence. " << std::endl;
                break;
        }
        newMotion->setTemporalScale(1.0);
        newMotion->setName(motion.name);

        database.removeMotion(motion.name);
        database.insertMotion(newMotion);
        std::cout << "Motion nb : " << database.getMotionNb() << std::endl;
        result = (database.isInDB(motion.name));
    
    }
    else
    {
        std::cout << "Motion " << rq.motion_name << " not in DB, store it first" << std::endl;
        std::cout << "no update has been performed." << std::endl;
    }
    std::cout << "Motion nb : " << database.getMotionNb() << std::endl;
    rp.success = result;
    imagine_common::KeyValPair kv;
    kv.key = rq.motion_name;
    kv.value = "updated";
    db_changed.publish(kv);

    return true;
}

bool Motiondb_ros::delete_motion_srv(imagine_common::DeleteMotion::Request &rq, imagine_common::DeleteMotion::Response &rp)
{
    std::cout << "trying to DELETE " << rq.motion_name << std::endl;
    bool alreadyExists = (database.isInDB(rq.motion_name));
    bool stillThere = alreadyExists;
    std::cout << "DEL : " << rq.motion_name << " is" << (alreadyExists ? " " : " not ") << "in DB." << std::endl;
    if( alreadyExists )
    {
        database.removeMotion(rq.motion_name);
        stillThere = database.isInDB(rq.motion_name);
    }
    
    rp.success = alreadyExists & !stillThere; 
    if( rp.success )
    {
        imagine_common::KeyValPair kv;
        kv.key = rq.motion_name;
        kv.value = "deleted";
        db_changed.publish(kv);
    }
    return true;
}


bool Motiondb_ros::run()
{
    std::cout << "Starting main loop." << std::endl;
	// Wait for callback from action topic to be called.*
    while(nh_.ok() && !shutdown)
    {
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }
    std::cout << "Spinning finished, exiting ..." << std::endl;
   
	return true;
}

int main(int argc, char** argv)
{
    namespace po = boost::program_options;

    po::options_description desc{"MotionDB_Options"};
    desc.add_options()
        ("help,h","Help message")
        ("home",po::value<std::string>(), "folder containing the database files")
        ("version,v",po::value<int>()->default_value(0), "version of the databse to use");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);    

    int version = 0;
	if( !vm.count("home") ){ std::cout << "--home not set ! Quitting !" << std::endl; exit(EXIT_FAILURE); }
	if( !vm.count("version") ){ std::cout << "--version not set ! Assuming version 0 !" << std::endl; }
    //else{ version = vm["version"]; }

	//init the ROS node
	ros::init(argc, argv, "motiondb_node");
	ros::NodeHandle nh;
	std::cout << "Provided parameters:" << std::endl;
	std::cout << "--home : " << vm["home"].as<std::string>() << std::endl;
	std::cout << "--version : " << vm["version"].as<int>() << std::endl;

	Motiondb_ros ros_database(nh, vm["home"].as<std::string>(), vm["version"].as<int>());

    ros_database.run();

	std::cout << "Done." << std::endl;
	return 0;
}

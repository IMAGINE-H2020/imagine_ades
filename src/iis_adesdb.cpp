/* File : iis_adesdb.cpp
 * 
 * @description : This node implements the methods for environmentSIM class (robot action simulator) in the block push experiment.
*/

#include "../include/iis_adesdb.h"
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

using namespace ades;

Adesdb_ros::Adesdb_ros(ros::NodeHandle &nh, std::string home, int version):
	nh_(nh),
    database(home, version)
{
    ss_get_precond = nh_.advertiseService("adesdb/get_preconds", &Adesdb_ros::get_preconds_srv, this);
    
    ss_get_effect = nh_.advertiseService("adesdb/get_effects", &Adesdb_ros::get_effects_srv, this);
    ss_get_motion = nh_.advertiseService("adesdb/get_motions", &Adesdb_ros::get_motions_srv, this);

    ss_list_ades = nh_.advertiseService("adesdb/list_ades", &Adesdb_ros::list_ades_srv, this);
    ss_store_ades = nh_.advertiseService("adesdb/store_ades", &Adesdb_ros::store_ades_srv, this);
    ss_update_ades= nh_.advertiseService("adesdb/update_ades", &Adesdb_ros::update_ades_srv, this);
    ss_delete_ades= nh_.advertiseService("adesdb/delete_ades", &Adesdb_ros::delete_ades_srv, this);

    shutdown = false;
}

Adesdb_ros::~Adesdb_ros()
{
    for(auto ades_ : database.listAdes())
    {
        for(auto ms_ : ades_.getMotionSequences())
        {
            for(auto mt_ : ms_.second.getMotions())
            {
                //delete mt_;
            }
        }
    }
}


bool Adesdb_ros::list_ades_srv(iis_libades_ros::ListAdes::Request &rq, iis_libades_ros::ListAdes::Response &rp)
{
    std::cout << "Request for listing ades " << std::endl;

    for(auto ades_ : database.listAdes())
    {
        rp.ades_list.push_back(ades_.getName());
    }
}

bool Adesdb_ros::get_preconds_srv(iis_libades_ros::GetAdesPreConds::Request &rq, iis_libades_ros::GetAdesPreConds::Response &rp)
{
    std::cout << "return preconditions for " << rq.ades_name << std::endl;
    iis_libades_ros::KeyValPair fail_pc_;
    if(database.isInDB(rq.ades_name))
    {
        auto ades = database.getAdesByName(rq.ades_name);

        for(auto pc : ades.getPreconditions())
        {
            iis_libades_ros::KeyValPair pc_;
            pc_.key = pc.first;
            pc_.value = pc.second;
            rp.preconditions.push_back(pc_);
        }
    }
    else
    {
        rp.preconditions.push_back(fail_pc_);
    }
}

bool Adesdb_ros::get_effects_srv(iis_libades_ros::GetAdesEffects::Request &rq, iis_libades_ros::GetAdesEffects::Response &rp)
{
    std::cout << "return effects for " << rq.ades_name << std::endl;
    iis_libades_ros::KeyValPair fail_ef_;
    if(database.isInDB(rq.ades_name))
    {
        auto ades = database.getAdesByName(rq.ades_name);

        for(auto ef : ades.getEffects())
        {
            iis_libades_ros::KeyValPair ef_;
            ef_.key = ef.first;
            ef_.value = ef.second;
            rp.effects.push_back(ef_);
        }
    }
    else
    {
        rp.effects.push_back(fail_ef_);
    }
}

bool Adesdb_ros::get_motions_srv(iis_libades_ros::GetAdesMotions::Request &rq, iis_libades_ros::GetAdesMotions::Response &rp)
{
    // This function could just send back ms names and some info (see the Motion class from libades) and we provide another more specific service for a ades/ms_id request)
    std::cout << "return motion sequences for " << rq.ades_name << std::endl;
    
    iis_libades_ros::MotionSequence fail_mo_seq_;
    fail_mo_seq_.sequence_name = "";
    fail_mo_seq_.input_types = std::vector<std::string>();
    fail_mo_seq_.effect_prob = std::vector<iis_libades_ros::KeyValPair>();
    fail_mo_seq_.effect_pred = std::vector<iis_libades_ros::KeyValPair>();
    fail_mo_seq_.motions = std::vector<iis_libades_ros::Motion>();

    if(database.isInDB(rq.ades_name))
    {
        auto ades = database.getAdesByName(rq.ades_name);

        for(auto sequence : ades.getMotionSequences())
        {
            // prepare MotionSequence msg 
            // (sequence_name, input_types, effect_prob, effect_pred, motions)
            iis_libades_ros::MotionSequence mo_seq_;
            mo_seq_.sequence_name = sequence.first;
            mo_seq_.input_types = sequence.second.getInputTypes();

            for(auto epb : sequence.second.getGMMEffectModels())
            {
                // For each effect prob model
                iis_libades_ros::KeyValPair effect;
                effect.key = epb.first;
                effect.value = std::to_string(epb.second.Dimensionality())+" "+std::to_string(epb.second.Gaussians())+" ";
                for(int ig = 0 ; ig < epb.second.Gaussians() ; ig++)
                {
                    auto model = epb.second.Component(ig);
                    auto mean = model.Mean();
                    auto cov = model.Covariance();
                    std::cout << "Gaussian means : " << std::flush;
                    for(auto m : mean)
                    {
                       effect.value += std::to_string(m)+" ";
                       std::cout << m << " " << std::flush;
                    }
                    effect.value += ", ";
                    std::cout << std::endl << "cov size : " << size(cov) << std::endl;
                    std::cout << "Gaussian covs : " << std::flush;
                    for(auto cv : cov)
                    {
                        effect.value += std::to_string(cv)+" ";
                       std::cout << cv << " " << std::flush;
                    }
                    effect.value += "; ";
                    std::cout << "----------" << std::endl;
                }
                mo_seq_.effect_prob.push_back(effect);
            }
            for(auto epd : sequence.second.getGPEffectModels())
            {
                // For each effect model
                iis_libades_ros::KeyValPair effect;
                effect.key = epd.first;
                effect.value = std::to_string(epd.second.get_input_dim());
                mo_seq_.effect_pred.push_back(effect);
            }
           
            mo_seq_.motions = std::vector<iis_libades_ros::Motion>();

            for(auto motion : sequence.second.getMotions())
            {
                iis_libades_ros::Motion mt;
                std_msgs::Float64MultiArray motion_data;
                // Motion is (type, data)
                mt.type = typeToString(motion->getMotionType());

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
                    //std::cout << "dim size: " << dimension.first << ", " << dimension.second.size() << std::endl;
                    for(auto val : dimension.second)
                    {
                        //std::cout << val << ", " << std::flush;
                        motion_data.data.push_back((double)val);                        
                    }
                }
                mt.data = (motion_data);
                mo_seq_.motions.push_back(mt);
            }
            rp.motion_sequences.push_back(mo_seq_);
        }
    }
    else
    {
        rp.motion_sequences.push_back(fail_mo_seq_);
    }
}


bool Adesdb_ros::store_ades_srv(iis_libades_ros::StoreAdes::Request &rq, iis_libades_ros::StoreAdes::Response &rp)
{
    std::cout << "trying to STORE " << rq.ades.ades_name << std::endl;
    bool result = false;
    if( !(database.isInDB(rq.ades.ades_name)) )
    {
        std::map<std::string, std::string> newPC;
        for(auto pc : rq.ades.preconditions)
        {
            newPC.insert(std::pair<std::string, std::string>(pc.key, pc.value));
        }
        
        std::map<std::string, std::string> newEF;
        for(auto ef : rq.ades.effects)
        {
            newEF.insert(std::pair<std::string, std::string>(ef.key, ef.value));
        }
        
        std::map<std::string, MotionSequence> newMotionSequences;
        for(auto ms : rq.ades.motion_sequences)
        {
            MotionSequence ms_;
            std::vector<std::string> inputTypes;
            for(auto it : ms.input_types)
            {
                inputTypes.push_back(it);
            }
            ms_.insertInputTypes(inputTypes);

            for(auto epb : ms.effect_prob)
            {
                // Process values - this should probably go in some utils/processign file:
                int pos = 0;
                std::vector<std::string> tokens;
                std::string delimiter = " ";
                while ((pos = epb.value.find(delimiter)) != std::string::npos)
                {
                    tokens.push_back(epb.value.substr(0, pos));
                    epb.value.erase(0, pos + delimiter.length());
                }
                if(tokens.size() > 1)
                {
                    ms_.insertGMMEffectModel(epb.key, std::stoi(tokens.at(0)), std::stoi(tokens.at(1)));
                }
                else
                {
                    std::cout << "Not enough GMM parameters, effect model not inserted !" << std::endl;
                }
            }
            for(auto epd : ms.effect_pred)
            {
                // Process values - this should probably go in some utils/processign file:
                int pos = 0;
                std::vector<std::string> tokens;
                std::string delimiter = " ";
                while ((pos = epd.value.find(delimiter)) != std::string::npos)
                {
                    tokens.push_back(epd.value.substr(0, pos));
                    epd.value.erase(0, pos + delimiter.length());
                }
                if(tokens.size() > 1)
                {
                    ms_.insertGPEffectModel(epd.key, std::stoi(tokens.at(0)), tokens.at(1));
                }
                else
                {
                    std::cout << "Not enough GP parameters, effect model not inserted !" << std::endl;
                }
            }
            int motion_count=0;
            for(auto motion : ms.motions)
            {
                std::cout << motion.type << std::endl;
                // For now we stupidly switch-case for values:
                const std::string m_types[2] = {"DMP", "Trajectory"};
                std::vector<std::string> m_types_(m_types, m_types + sizeof(m_types)  / sizeof(m_types[0]) );
                int this_type = find(m_types_.begin(), m_types_.end(), motion.type) - m_types_.begin();
                Motion * newMotion;

                int data_index = 0;
                std::map<std::string, std::vector<double>> params;
                for(auto i : motion.data.layout.dim)
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
                        newMotion = new DMPContainer(params["gaussiansCenters"],params["gaussiansVariances"],params["weights"],params["dmpCoeffs"]);
                    break;
                    case 1: // Trajectory
                        std::cout << "> Trajectory" << std::endl;
			for(auto p : params)
			{
				points.push_back(p.second);
			}
                        newMotion = new TrajectoryContainer(points);
                    break;
                    default:
                        std::cout << "This motion is not implemented yet ; It has not been added to the sequence. " << std::endl;
                    break;
                }
                ms_.insertMotion(motion_count, newMotion);
                motion_count += 1;
            }
            newMotionSequences.insert(std::pair<std::string, MotionSequence>(ms.sequence_name, ms_));
        } 
     
        // Creating Ades here to use the easier insert*EffectModel methods
        Ades newAdes(rq.ades.ades_name, newPC, newEF, newMotionSequences);
        database.addAdes(newAdes);
        std::cout << "Ades nb : " << database.getAdesNb() << std::endl;
        result = (database.isInDB(rq.ades.ades_name));
    } 
    rp.success = result;
}

bool Adesdb_ros::update_ades_srv(iis_libades_ros::UpdateAdes::Request &rq, iis_libades_ros::UpdateAdes::Response &rp)
{
    std::cout << "trying to UPDATE " << rq.ades_name << std::endl;
    // This service is mostly a copy paste of store ; some refactoring will be needed
    bool result = false;
    // "name" in the request is the target (needs to exist)
    // "ades.{ades_name, *}" is the new content (needs to be filled)
    if( (database.isInDB(rq.ades_name)) )
    {
        auto ades_to_update = database.updateAdesByName(rq.ades_name);

        if( !rq.ades.preconditions.empty() )
        {
            std::cout << "Updating preconditions" << std::endl;
            std::map<std::string, std::string> newPC;
            for(auto pc : rq.ades.preconditions)
            {
                newPC.insert(std::pair<std::string, std::string>(pc.key, pc.value));
            }
            ades_to_update->modifyPreconditions(newPC);
        }
        if( !rq.ades.effects.empty() )
        {
            std::cout << "Updating effects" << std::endl;
            std::map<std::string, std::string> newEF;
            for(auto ef : rq.ades.effects)
            {
                newEF.insert(std::pair<std::string, std::string>(ef.key, ef.value));
            }
            ades_to_update->modifyEffects(newEF);
        }
        if( !rq.ades.motion_sequences.empty() )
        {
            std::cout << "Updating motion sequences" << std::endl;
            //std::map<std::string, MotionSequence> newMotionSequences;
            for(auto ms : rq.ades.motion_sequences)
            {
                MotionSequence ms_;
                std::vector<std::string> inputTypes;
                for(auto it : ms.input_types)
                {
                    inputTypes.push_back(it);
                }
                ms_.insertInputTypes(inputTypes);

                for(auto epb : ms.effect_prob)
                {
                    // Process values - this should probably go in some utils/processign file:
                    int pos = 0;
                    std::vector<std::string> tokens;
                    std::string delimiter = " ";
                    while ((pos = epb.value.find(delimiter)) != std::string::npos)
                    {
                        tokens.push_back(epb.value.substr(0, pos));
                        epb.value.erase(0, pos + delimiter.length());
                    }
                    if(tokens.size() > 1)
                    {
                        ms_.insertGMMEffectModel(epb.key, std::stoi(tokens.at(0)), std::stoi(tokens.at(1)));
                    }
                    else
                    {
                        std::cout << "Not enough GMM parameters, effect model not inserted !" << std::endl;
                    }
                }
                for(auto epd : ms.effect_pred)
                {
                    // Process values - this should probably go in some utils/processign file:
                    int pos = 0;
                    std::vector<std::string> tokens;
                    std::string delimiter = " ";
                    while ((pos = epd.value.find(delimiter)) != std::string::npos)
                    {
                        tokens.push_back(epd.value.substr(0, pos));
                        epd.value.erase(0, pos + delimiter.length());
                    }
                    if(tokens.size() > 1)
                    {
                        ms_.insertGPEffectModel(epd.key, std::stoi(tokens.at(0)), tokens.at(1));
                    }
                    else
                    {
                        std::cout << "Not enough GP parameters, effect model not inserted !" << std::endl;
                    }
                }
                int motion_count=0;
                for(auto motion : ms.motions)
                {
                    std::cout << motion.type << std::endl;
                    // For now we stupidly switch-case for values:
                    const std::string m_types[2] = {"DMP", "Trajectory"};
                    std::vector<std::string> m_types_(m_types, m_types + sizeof(m_types)  / sizeof(m_types[0]) );
                    int this_type = find(m_types_.begin(), m_types_.end(), motion.type) - m_types_.begin();
                    Motion * newMotion;

                    int data_index = 0;
                    std::map<std::string, std::vector<double>> params;
                    for(auto i : motion.data.layout.dim)
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
                            newMotion = new DMPContainer(params["gaussiansCenters"],params["gaussiansVariances"],params["weights"],params["dmpCoeffs"]);
                        break;
                        case 1: // Trajectory
				std::cout << "> Trajectory" << std::endl;
				for(auto p : params)
				{
					points.push_back(p.second);
				}
				newMotion = new TrajectoryContainer(points);
                        break;
                        default:
                            std::cout << "This motion is not implemented yet ; It has not been added to the sequence. " << std::endl;
                        break;
                    }
                    ms_.insertMotion(motion_count, newMotion);
                    motion_count += 1;
                }
                auto current_sequences = (ades_to_update->getMotionSequences());
                // This function doesn't exist but should probably be added :
                // ades_to_update->modifyMotionSequence(newMotionSequences);
                if ( current_sequences.find(ms.sequence_name) == current_sequences.end() )
                {
                    std::cout << "Motion sequence does not exist in this ADES ; inserting it." << std::endl;
                    ades_to_update->insertMotionSequence(ms.sequence_name, ms_);

                }
                else
                {
                    // remove the old one, store the new one :
                    ades_to_update->removeMotionSequence(ms.sequence_name);
                    ades_to_update->insertMotionSequence(ms.sequence_name, ms_);
                }
            }
        } 
        result = true; 
     
    }
    else
    {
        std::cout << "ADES " << rq.ades_name << " not in DB, store it first" << std::endl;
        std::cout << "no update has been performed." << std::endl;
    }
    std::cout << "Ades nb : " << database.getAdesNb() << std::endl;
    rp.success = result;

}

bool Adesdb_ros::delete_ades_srv(iis_libades_ros::DeleteAdes::Request &rq, iis_libades_ros::DeleteAdes::Response &rp)
{
    std::cout << "trying to DELETE " << rq.ades_name << std::endl;
    bool alreadyExists = (database.isInDB(rq.ades_name));
    bool stillThere = alreadyExists;
    std::cout << "DEL : " << rq.ades_name << " is" << (alreadyExists ? " " : " not ") << "in DB." << std::endl;
    if( alreadyExists )
    {
        database.removeAdesByName(rq.ades_name);
        stillThere = database.isInDB(rq.ades_name);
    }
    
    rp.success = alreadyExists & !stillThere; 
}

bool Adesdb_ros::run()
{
    std::cout << "Starting main loop." << std::endl;
	// Wait for callback from action topic to be called.
	while(nh_.ok() && !shutdown)
	{
		ros::spinOnce();
	}
    std::cout << "Spinning finished, exiting ..." << std::endl;
   
	return true;
}

int main(int argc, char** argv)
{
    namespace po = boost::program_options;

    po::options_description desc{"AdesDB_Options"};
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
	ros::init(argc, argv, "adesdb_node");
	ros::NodeHandle nh;
	std::cout << "Provided parameters:" << std::endl;
	std::cout << "--home : " << vm["home"].as<std::string>() << std::endl;
	std::cout << "--version : " << vm["version"].as<int>() << std::endl;

	Adesdb_ros ros_database(nh, vm["home"].as<std::string>(), vm["version"].as<int>());

    ros_database.run();

	std::cout << "Done." << std::endl;
	return 0;
}


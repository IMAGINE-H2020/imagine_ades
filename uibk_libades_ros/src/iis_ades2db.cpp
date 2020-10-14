/* File : iis_ades2db.cpp
 * 
 * @description : This node implements the methods for environmentSIM class (robot action simulator) in the block push experiment.
*/

#include "../include/iis_ades2db.h"
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

using namespace ades;

Ades2db_ros::Ades2db_ros(ros::NodeHandle &nh, std::string home, int version):
    nh_(nh),
    database(home, version)
{

    ss_get_precond = nh_.advertiseService("ades2db/get_preconds", &Ades2db_ros::get_preconds_srv, this);
    ss_get_effect = nh_.advertiseService("ades2db/get_effects", &Ades2db_ros::get_effects_srv, this);
    ss_get_motion = nh_.advertiseService("ades2db/get_motions", &Ades2db_ros::get_motions_srv, this);
    ss_get_motion_name = nh_.advertiseService("ades2db/get_motion_names", &Ades2db_ros::get_motion_names_srv, this);


    ss_list_ades = nh_.advertiseService("ades2db/list_ades", &Ades2db_ros::list_ades_srv, this);
    ss_store_ades = nh_.advertiseService("ades2db/store_ades", &Ades2db_ros::store_ades_srv, this);
    ss_update_ades = nh_.advertiseService("ades2db/update_ades", &Ades2db_ros::update_ades_srv, this);
    ss_update_ades_motion = nh_.advertiseService("ades2db/update_ades_motion", &Ades2db_ros::update_ades_motion_srv, this);
    ss_delete_ades = nh_.advertiseService("ades2db/delete_ades", &Ades2db_ros::delete_ades_srv, this);
    ss_add_motion_sequence = nh_.advertiseService("ades2db/add_motion_sequence", &Ades2db_ros::add_motion_sequence_srv, this);


    ss_update_effect_models = nh_.advertiseService("ades2db/update_effect_models", &Ades2db_ros::update_effect_models_srv, this);
    ss_estimate_effect = nh_.advertiseService("ades2db/estimate_effect", &Ades2db_ros::estimate_effect_srv, this);

    client_listmotion = nh.serviceClient<imagine_common::ListMotion>("motiondb/list_motion");
    client_storemotion = nh.serviceClient<imagine_common::StoreMotion>("motiondb/store_motion");
    client_getmotion = nh.serviceClient<imagine_common::GetMotion>("motiondb/get_motion");
    client_updatemotion = nh.serviceClient<imagine_common::UpdateMotion>("motiondb/update_motion");
    client_deletemotion = nh.serviceClient<imagine_common::DeleteMotion>("motiondb/delete_motion");

    db_changed = nh_.advertise<imagine_common::KeyValPair>("ades2db/db_updated", 1);

    shutdown = false;
}

Ades2db_ros::~Ades2db_ros()
{
}

bool Ades2db_ros::list_ades_srv(imagine_common::ListAdes::Request &rq, imagine_common::ListAdes::Response &rp)
{
    std::cout << "Request for listing ades " << std::endl;

    for(auto ades_ : database.listAdes())
    {
        rp.ades_list.push_back(ades_.getName());
    }

    return true;
}

// Keep original ADES services for preconds and effects
bool Ades2db_ros::get_preconds_srv(imagine_common::GetAdesPreConds::Request &rq, imagine_common::GetAdesPreConds::Response &rp)
{
    std::cout << "return preconditions for " << rq.ades_name << std::endl;
    imagine_common::KeyValPair fail_pc_;
    if(database.isInDB(rq.ades_name))
    {
        auto ades = database.getAdesByName(rq.ades_name);

        for(auto pc : ades.getPreconditions())
        {
            imagine_common::KeyValPair pc_;
            pc_.key = pc.first;
            pc_.value = pc.second;
            rp.preconditions.push_back(pc_);
        }
    }
    else
    {
        rp.preconditions.push_back(fail_pc_);
    }

    return true;
}

// Keep original ADES services for preconds and effects
bool Ades2db_ros::get_effects_srv(imagine_common::GetAdesEffects::Request &rq, imagine_common::GetAdesEffects::Response &rp)
{
    std::cout << "return effects for " << rq.ades_name << std::endl;
    imagine_common::KeyValPair fail_ef_;
    if(database.isInDB(rq.ades_name))
    {
        auto ades = database.getAdesByName(rq.ades_name);

        for(auto ef : ades.getEffects())
        {
            imagine_common::KeyValPair ef_;
            ef_.key = ef.first;
            ef_.value = ef.second;
            rp.effects.push_back(ef_);
        }
    }
    else
    {
        rp.effects.push_back(fail_ef_);
    }

    return true;
}

// TODO
bool Ades2db_ros::get_motions_srv(imagine_common::GetAdesMotions::Request &rq, imagine_common::GetAdesMotions::Response &rp)
{
    // This function could just send back ms names and some info (see the Motion class from libades) and we provide another more specific service for a ades/ms_id request)
    std::cout << "return motion sequences for " << rq.ades_name << std::endl;
    
    imagine_common::MotionSequence fail_mo_seq_;
    fail_mo_seq_.sequence_name = "";
    fail_mo_seq_.score = -1.0;
    fail_mo_seq_.input_types = std::vector<std::string>();
    fail_mo_seq_.effect_prob = std::vector<imagine_common::KeyValPair>();
    fail_mo_seq_.effect_pred = std::vector<imagine_common::KeyValPair>();

    if(database.isInDB(rq.ades_name))
    {
        std::cout << "Ades in DB" << std::endl;
        auto ades = database.getAdesByName(rq.ades_name);

        for(auto sequence : ades.getMotionSequences())
        {
            std::cout << "sequence: " << sequence.first << std::endl;
            // prepare MotionSequence msg 
            // (sequence_name, input_types, effect_prob, effect_pred, motions)
            imagine_common::MotionSequence mo_seq_;
            mo_seq_.sequence_name = sequence.first;
            mo_seq_.input_types = sequence.second.getInputTypes();
            mo_seq_.score = sequence.second.getScore();

            for(auto epb : sequence.second.getGMMEffectModels())
            {
                // For each effect prob model
                imagine_common::KeyValPair effect;
                effect.key = epb.first;
                effect.value = std::to_string(epb.second.Dimensionality())+" "+std::to_string(epb.second.Gaussians())+"\n";
                for(int ig = 0 ; ig < epb.second.Gaussians() ; ig++)
                {
                    auto model = epb.second.Component(ig);
                    auto mean = model.Mean();
                    auto cov = model.Covariance();
                    //std::cout << "Gaussian means : " << std::flush;
                    effect.value += " means: ";
                    for(auto m : mean)
                    {
                       effect.value += std::to_string(m)+" ";
                       //std::cout << m << " " << std::flush;
                    }
                    //std::cout << std::endl << "cov size : " << size(cov) << std::endl;
                    //std::cout << "Gaussian covs : " << std::flush;
                    effect.value += " covs: ";
                    for(auto cv : cov)
                    {
                        effect.value += std::to_string(cv)+" ";
                        //std::cout << cv << " " << std::flush;
                    }
                    effect.value += ";\n";
                    //std::cout << "----------" << std::endl;
                }
                mo_seq_.effect_prob.push_back(effect);
            }
            for(auto epd : sequence.second.getGPEffectModels())
            {
                // For each effect model
                imagine_common::KeyValPair effect;
                effect.key = epd.first;
                effect.value = std::to_string(epd.second.get_input_dim());
                mo_seq_.effect_pred.push_back(effect);
            }
           
            // For motions, we need to fetch them from the motionDB
            mo_seq_.motions = std::vector<imagine_common::Motion>();
            
            for(auto motion_name : sequence.second.getMotions())
            {
                std::cout << "Motion: " << motion_name << std::endl;
                
                imagine_common::GetMotion get_motion_call;
                get_motion_call.request.name = motion_name;
                client_getmotion.call(get_motion_call);
                auto found_motion = get_motion_call.response.motion;
                //std::cout << "Is the motion existing? " << (!found_motion.name.empty() ? "TRU" : "FALZ")  << std::endl;
                if( !found_motion.name.empty() )
                {
                    mo_seq_.motions.push_back(found_motion);
                }
                else
                {
                    std::cout << "Error, motion not found in DB ; returning empty motion in sequence." << std::endl;
                    imagine_common::Motion missing_mt;                    
                    missing_mt.name = "missing_motion";
                    mo_seq_.motions.push_back(missing_mt);
                }

                // Archeological code in these brackets
                {
                    /*imagine_common::Motion mt;
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
                    mt.data = (motion_data);*/
                    //mo_seq_.motions.push_back(mt);
                }
            }
            
            rp.motion_sequences.push_back(mo_seq_);
        }
    }
    else
    {
        rp.motion_sequences.push_back(fail_mo_seq_);
        //return false;
    }

    return true;
}

// TODO
bool Ades2db_ros::get_motion_names_srv(imagine_common::GetAdesMotionNames::Request &rq, imagine_common::GetAdesMotionNames::Response &rp)
{
    // This function just send back motion names +
    std::cout << "return motions for motion sequence " << rq.motion_sequence << " of ADES for " << rq.ades_name << std::endl;
    
    auto ms_motion_names = std::vector<std::string>();
    
    if(database.isInDB(rq.ades_name))
    {
        std::cout << "Ades in DB" << std::endl;
        auto ades = database.getAdesByName(rq.ades_name);

        for(auto sequence : ades.getMotionSequences())
        {
            std::cout << "sequence: " << sequence.first << std::endl;
            if(sequence.first == rq.motion_sequence )
            {
                for(auto motion_name : sequence.second.getMotions())
                {
                    std::cout << "Motion: " << motion_name << std::endl;
                    ms_motion_names.push_back(motion_name);
                }
            }
        }
        rp.motion_names = ms_motion_names;
    }
    else
    {
        std::cout << "ADES not found in DB" << std::endl;
    }

    return true;
}

bool Ades2db_ros::store_ades_srv(imagine_common::StoreAdes::Request &rq, imagine_common::StoreAdes::Response &rp)
{
    std::cout << "trying to STORE " << rq.ades.ades_name << std::endl;
    bool result = false;
    
    if( !(database.isInDB(rq.ades.ades_name)) )
    {
        std::map<std::string, std::string> newPC;
        std::map<std::string, std::string> newEF;
        
        for(auto pc : rq.ades.preconditions){ newPC.insert(std::pair<std::string, std::string>(pc.key, pc.value)); }
        for(auto ef : rq.ades.effects){ newEF.insert(std::pair<std::string, std::string>(ef.key, ef.value)); }
        
        std::map<std::string, MotionSequence2> newMotionSequences;
        for(auto ms : rq.ades.motion_sequences)
        {
            //MotionSequence ms_;
            MotionSequence2 ms2_;
            ms2_.setScore(ms.score);
            std::vector<std::string> inputTypes;
            for(auto it : ms.input_types)
            {
                inputTypes.push_back(it);
            }
            //ms_.insertInputTypes(inputTypes);
            ms2_.insertInputTypes(inputTypes);

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
                std::cout << "Token size: " << tokens.size() << std::endl;
                if(tokens.size() > 1)
                {
                    //ms_.insertGMMEffectModel(epb.key, std::stoi(tokens.at(0)), std::stoi(tokens.at(1)));
                    ms2_.insertGMMEffectModel(epb.key, std::stoi(tokens.at(0)), std::stoi(tokens.at(1)));
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
                    //ms_.insertGPEffectModel(epd.key, std::stoi(tokens.at(0)), tokens.at(1));
                    ms2_.insertGPEffectModel(epd.key, std::stoi(tokens.at(0)), tokens.at(1));
                }
                else
                {
                    std::cout << "Not enough GP parameters, effect model not inserted !" << std::endl;
                }
            }
            int motion_count=0;
            for(auto motion : ms.motions)
            {
                std::cout << motion.name << ", " << motion.type << std::endl;

                imagine_common::GetMotion get_motion_call;
                get_motion_call.request.name = motion.name;
                client_getmotion.call(get_motion_call);
                auto found_motion = get_motion_call.response.motion;
                std::cout << "Is the motion existing? " << (!found_motion.name.empty() ? "TRU" : "FALZ")  << std::endl;

                // If motion does not exist, we call the store_motion service of the motion DB to add the motion
                // In any case, we add the name to the ades2 motion sequence
                if( found_motion.name.empty() )
                {
                    // motion not found
                    imagine_common::StoreMotion store_motion_call;
                    store_motion_call.request.motion = motion;
                    client_storemotion.call(store_motion_call);
                }
                else{ std::cout << "Motion found, continuing ..." << std::endl; }

                
                // For now we stupidly switch-case for values:
                /*const std::string m_types[] = {"DMP", "Trajectory", "Unscrewing"};
                std::vector<std::string> m_types_(m_types, m_types + sizeof(m_types)  / sizeof(m_types[0]) );
                int this_type = find(m_types_.begin(), m_types_.end(), motion.type) - m_types_.begin();
                Motion * newMotion;
                int data_index = 0;
                std::map<std::string, std::vector<double>> params;
                for(auto i : motion.data.layout.dim)
                {
                    int dim_size = i.size;
                    std::string dim_label = i.label;
                    //std::cout << "dim labl: " << i.label << std::endl;
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
                        //newMotion = new DMPContainer(params["gaussiansCenters"],params["gaussiansVariances"],params["weights"],params["dmpCoeffs"]);
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
                }*/

                // insert the motion name here :
                //ms_.insertMotion(motion_count, newMotion);
                ms2_.insertMotion(motion_count, motion.name);
                motion_count += 1;
            }
            //newMotionSequences.insert(std::pair<std::string, MotionSequence>(ms.sequence_name, ms_));
            newMotionSequences.insert(std::pair<std::string, MotionSequence2>(ms.sequence_name, ms2_));
        }
        /*for( auto nms : newMotionSequences)
        {
            std::cout << "nms: " << nms.first << std::endl;
        }*/
     
        // Creating Ades here to use the easier insert*EffectModel methods
        Ades2 newAdes(rq.ades.ades_name, newPC, newEF, newMotionSequences);
        database.addAdes(newAdes);
        std::cout << "Ades nb : " << database.getAdesNb() << std::endl;
        result = (database.isInDB(rq.ades.ades_name));
        
        rp.success = result;
        imagine_common::KeyValPair kv;
        kv.key = rq.ades.ades_name;
        kv.value = "stored";
        db_changed.publish(kv);
    }
    else
    {
        std::cout << "ADES " << rq.ades.ades_name << " ALREADY in DB, update it instead" << std::endl;
        std::cout << "no insertion has been performed." << std::endl;
    }
    
    return true;
}

bool Ades2db_ros::update_ades_srv(imagine_common::UpdateAdes::Request &rq, imagine_common::UpdateAdes::Response &rp)
{
    std::cout << "trying to UPDATE " << rq.ades_name << std::endl;
    // This service is mostly a copy paste of store ; some refactoring will be needed
    bool result = false;
    // "name" in the request is the target (needs to exist)
    // "ades.{ades_name, *}" is the new content (needs to be filled)
    
    if( (database.isInDB(rq.ades_name)) )
    {
        std::cout << "KIKOO UPDATE ADES FOUND " << rq.ades_name << std::endl;

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
        
        std::cout << "UPDATE ADES - motion sequences len: " << rq.ades.motion_sequences.size() << std::endl;   
        if( !rq.ades.motion_sequences.empty() )
        {
            std::cout << "Updating motion sequences" << std::endl;
            //std::map<std::string, MotionSequence> newMotionSequences;
            for(auto ms : rq.ades.motion_sequences)
            {
                MotionSequence2 ms2_;
                ms2_.setScore(ms.score);
                std::vector<std::string> inputTypes;
                for(auto it : ms.input_types)
                {
                    inputTypes.push_back(it);
                }
                ms2_.insertInputTypes(inputTypes);

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
                        ms2_.insertGMMEffectModel(epb.key, std::stoi(tokens.at(0)), std::stoi(tokens.at(1)));
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
                        ms2_.insertGPEffectModel(epd.key, std::stoi(tokens.at(0)), tokens.at(1));
                    }
                    else
                    {
                        std::cout << "Not enough GP parameters, effect model not inserted !" << std::endl;
                    }
                }

                int motion_count=0;
                for(auto motion : ms.motions)
                {
                    // Check if motion already exists ;
                    // if yes, modify it from motionDB
                    // else insert it ?
                    // 
                    imagine_common::GetMotion get_motion_call;
                    get_motion_call.request.name = motion.name;
                    client_getmotion.call(get_motion_call);
                    auto found_motion = get_motion_call.response.motion;
                    std::cout << "UPDATE: Is the motion existing? " << (!found_motion.name.empty() ? "TRU" : "FALZ")  << std::endl;

                    // If motion does not exist, we call the store_motion service of the motion DB to add the motion
                    // Else, we update it in the motionDB (update_motion)
                    // In any case, we add/update the name to the ades2 motion sequence
                    if( found_motion.name.empty() )
                    {
                        // motion not found
                        imagine_common::StoreMotion store_motion_call;
                        store_motion_call.request.motion = motion;
                        client_storemotion.call(store_motion_call);
                    }
                    else
                    {
                        std::cout << "Motion found, updating in motionDB ..." << std::endl;
                        imagine_common::UpdateMotion update_motion_call;
                        update_motion_call.request.motion_name = motion.name;
                        update_motion_call.request.motion = motion;
                        client_updatemotion.call(update_motion_call);   
                    }

                    std::cout << motion.type << std::endl;

                    ms2_.insertMotion(motion_count, motion.name);
                    //ms2_.insertMotion(motion_count, newMotion);
                    motion_count += 1;
                }
                auto current_sequences = (ades_to_update->getMotionSequences());
                // This function doesn't exist but should probably be added :
                // ades_to_update->modifyMotionSequence(newMotionSequences);
                if ( current_sequences.find(ms.sequence_name) == current_sequences.end() )
                {
                    std::cout << "Motion sequence does not exist in this ADES ; inserting it." << std::endl;
                    ades_to_update->insertMotionSequence(ms.sequence_name, ms2_);

                }
                else
                {
                    // remove the old one, store the new one :
                    ades_to_update->removeMotionSequence(ms.sequence_name);
                    ades_to_update->insertMotionSequence(ms.sequence_name, ms2_);
                }
            }
        }
        else
        {
            std::cout << "EMPTY motion sequence" << std::endl;
        }

        result = true;
        
        rp.success = result;
        imagine_common::KeyValPair kv;
        kv.key = rq.ades_name;
        kv.value = "updated";
        db_changed.publish(kv); 
    }
    else
    {
        std::cout << "ADES " << rq.ades_name << " not in DB, store it first" << std::endl;
        std::cout << "no update has been performed." << std::endl;
    }
    std::cout << "Ades nb : " << database.getAdesNb() << std::endl;
    
    return result;
}

bool Ades2db_ros::add_motion_sequence_srv(imagine_common::AddMotionSequence::Request &rq, imagine_common::AddMotionSequence::Response &rp)
{
    std::cout << "ADDING motion sequence to ADES ... NOT IMPLEMENTED ..." << std::endl;

    if(!rq.ades_name.empty())
    {
        bool adesExists = (database.isInDB(rq.ades_name));

        std::cout << "Trying to add " << rq.motion_sequence.sequence_name << " to " << rq.ades_name << std::endl;
        auto ades_to_update = database.updateAdesByName(rq.ades_name);

        std::cout << "Updating motion sequences" << std::endl;
        
        auto ms = rq.motion_sequence;

        //MotionSequence2 ms_;
        MotionSequence2 ms2_;
        ms2_.setScore(ms.score);

        std::vector<std::string> inputTypes;
        for(auto it : ms.input_types)
        {
            inputTypes.push_back(it);
        }
        ms2_.insertInputTypes(inputTypes);

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
                ms2_.insertGMMEffectModel(epb.key, std::stoi(tokens.at(0)), std::stoi(tokens.at(1)));
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
                ms2_.insertGPEffectModel(epd.key, std::stoi(tokens.at(0)), tokens.at(1));
            }
            else
            {
                std::cout << "Not enough GP parameters, effect model not inserted !" << std::endl;
            }
        }
        int motion_count=0;
        for(auto motion : ms.motions)
        {
            // Check if motion already exists ;
            // if yes, modify it from motionDB
            // else insert it
            // 
            imagine_common::GetMotion get_motion_call;
            get_motion_call.request.name = motion.name;
            client_getmotion.call(get_motion_call);
            auto found_motion = get_motion_call.response.motion;
            std::cout << "UPDATE: Is the motion existing? " << (!found_motion.name.empty() ? "TRU" : "FALZ")  << std::endl;

            // If motion does not exist, we call the store_motion service of the motion DB to add the motion
            // Else, we update it in the motionDB (update_motion)
            // In any case, we add/update the name to the ades2 motion sequence
            if( found_motion.name.empty() )
            {
                // motion not found
                imagine_common::StoreMotion store_motion_call;
                store_motion_call.request.motion = motion;
                client_storemotion.call(store_motion_call);
            }
            else
            {
                std::cout << "Motion found, updating in motionDB ..." << std::endl;
                imagine_common::UpdateMotion update_motion_call;
                update_motion_call.request.motion_name = motion.name;
                update_motion_call.request.motion = motion;
                client_updatemotion.call(update_motion_call);   
            }
            std::cout << motion.type << std::endl;

            ms2_.insertMotion(motion_count, motion.name);
            motion_count += 1;
        }
        auto current_sequences = (ades_to_update->getMotionSequences());
        if ( current_sequences.find(ms.sequence_name) == current_sequences.end() )
        {
            std::cout << "Motion sequence does not exist in this ADES ; inserting it." << std::endl;
            ades_to_update->insertMotionSequence(ms.sequence_name, ms2_);
        }
        else
        {
            // remove the old one, store the new one :
            ades_to_update->removeMotionSequence(ms.sequence_name);
            ades_to_update->insertMotionSequence(ms.sequence_name, ms2_);
        }
    } 

    rp.success = true;
    imagine_common::KeyValPair kv;
    kv.key = rq.ades_name;
    kv.value = "updated";
    db_changed.publish(kv); 

    return true;
}

bool Ades2db_ros::update_ades_motion_srv(imagine_common::UpdateAdesMotionNames::Request &rq, imagine_common::UpdateAdesMotionNames::Response &rp)
{
    std::cout << "Updating ADES motion sequence motion names ..." << std::endl;

    if(!rq.ades_name.empty())
    {
        bool adesExists = (database.isInDB(rq.ades_name));

        if(!rq.sequence_name.empty() & adesExists)
        {
            auto target_ades = database.updateAdesByName(rq.ades_name);
            MotionSequence2 * target_motionsequence = target_ades->modifyMotionSequence(rq.sequence_name);
            int count = 0;
            int current_motion_nb = target_motionsequence->getMotionNumber();
            int future_motion_nb = rq.sequence_motion_names.size();
            for(int motion_ = 0 ; motion_ < current_motion_nb ; motion_++)
            {
                target_motionsequence->removeMotion(motion_);
            }
            for(int motion_ = 0 ; motion_ < future_motion_nb ; motion_++)
            {
                target_motionsequence->insertMotion(motion_, rq.sequence_motion_names[motion_]);
            }
        }
        else
        {
            std::cout << "Incorrect Motion Sequence name." << std::endl;       
        }
    }
    else
    {
        std::cout << "Incorrect ADES name." << std::endl;

    }

    return true;
}


bool Ades2db_ros::delete_ades_srv(imagine_common::DeleteAdes::Request &rq, imagine_common::DeleteAdes::Response &rp)
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
    if( rp.success )
    {
        imagine_common::KeyValPair kv;
        kv.key = rq.ades_name;
        kv.value = "deleted";
        db_changed.publish(kv);
    }
    return true;
}

//ss_update_effect_models = nh_.advertiseService("adesdb/update_effect_models", &Ades2db_ros::update_effect_models_srv, this);
bool Ades2db_ros::update_effect_models_srv(imagine_common::UpdateEffects::Request &rq, imagine_common::UpdateEffects::Response &rp)
{
    std::cout << "Received sample: " << rq.ades_name << ", " << rq.samples.size() << std::endl;
    /*
    if(database.isInDB(rq.ades_name))
    {
        std::cout << "Ades known" << std::endl;
        //auto ades = database.getAdesByName(rq.ades_name);
        auto ades = database.updateAdesByName(rq.ades_name);
        auto all_ms_ = ades->getMotionSequences();
        if(!all_ms_.empty())
        {
            // We assume there can be only one motion sequence
            auto ms_ = ades->modifyMotionSequence(all_ms_.begin()->first);

            for(auto s : rq.samples)
            {
                auto gmms_ = ms_->getGMMEffectModels();
                if( gmms_.find(s.effect_type) != gmms_.end() )
                {
                    std::cout << "Ades GMM known" << std::endl;
                    ms_->updateGMMEffectModel(s.effect_type, s.input, s.effect);
                    std::cout << "GMM updated" << std::endl;
                }
                auto gps_ = ms_->getGPEffectModels();
                if( gps_.find(s.effect_type) != gps_.end() )
                {
                    std::cout << "Ades GP known" << std::endl;
                    ms_->updateGPEffectModel(s.effect_type, s.input, s.effect);
                    std::cout << "GP updated" << std::endl;
                }
                else
                {
                    std::cout << "Unknown effect type" << std::endl;
                }
                rp.success = true; // to refine to take into account all model updates
                imagine_common::KeyValPair kv;
                kv.key = rq.ades_name;
                kv.value = "updated";
                db_changed.publish(kv);
            }
        }
        else
        {
            std::cout << "No motion sequence to modify, no update done." << std::endl;
            rp.success = false;
        }
    }
    else
    {
        std::cout << "Unknown ades, no update done." << std::endl;
        rp.success = false;
    }
    */
    return true;
}

bool Ades2db_ros::estimate_effect_srv(imagine_common::EstimateEffect::Request &rq, imagine_common::EstimateEffect::Response &rp)
{
    rp.value = 0.0;
    rp.probability = -1;
    std::cout << rq.ades_name << ", " << rq.sequence_name << ", " << rq.effect_type << ", " /*<< rq.input*/ << std::endl;
    if(database.isInDB(rq.ades_name))
    {
        auto ades = database.getAdesByName(rq.ades_name);
        auto all_ms_ = ades.getMotionSequences();
        if( all_ms_.find(rq.sequence_name) != all_ms_.end() )
        {
            double effect_value=0.0;
            auto sequence = all_ms_.find(rq.sequence_name);
            if( (sequence->second).getGPEffectModels().find(rq.effect_type) != (sequence->second).getGPEffectModels().end() )
            {
                //auto effect_model = (sequence->second).getGPEffectModels().at(rq.effect_type);
                rp.value = effect_value = (sequence->second).estimateEffect(rq.effect_type, rq.input);
            }
            if( (sequence->second).getGMMEffectModels().find(rq.effect_type) != (sequence->second).getGMMEffectModels().end() )
            {
                //auto effect_prob_model = (sequence->second).getGMMEffectModels().at(rq.effect_type);
                rp.probability = (sequence->second).estimateEffectLikelihood(rq.effect_type, rq.input, effect_value);
            }
        }
        else
        {
            std::cout << "Sequence name unknown, can't estimate effect." << std::endl;
        }

    }
    else
    {
        std::cout << "ADES name unknown, can't estimate effect." << std::endl;
    }
}

bool Ades2db_ros::run()
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
    ros::init(argc, argv, "ades2db_node");
    ros::NodeHandle nh;
    std::cout << "Provided parameters:" << std::endl;
    std::cout << "--home : " << vm["home"].as<std::string>() << std::endl;
    std::cout << "--version : " << vm["version"].as<int>() << std::endl;

    Ades2db_ros ros_database(nh, vm["home"].as<std::string>(), vm["version"].as<int>());

    ros_database.run();

    std::cout << "Done." << std::endl;
    return 0;
}

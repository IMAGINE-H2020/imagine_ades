#pragma once

//#include <map>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>

#include <ades/libades.h>
#include "imagine_common/Dummy.h"

#include "imagine_common/ListAdes.h"
#include "imagine_common/StoreAdes.h"
#include "imagine_common/UpdateAdes.h"
#include "imagine_common/UpdateAdesMotionNames.h"
#include "imagine_common/AddMotionSequence.h"
#include "imagine_common/DeleteAdes.h"

#include "imagine_common/ListMotion.h"
#include "imagine_common/StoreMotion.h"
#include "imagine_common/GetMotion.h"
#include "imagine_common/UpdateMotion.h"
#include "imagine_common/DeleteMotion.h"

#include "imagine_common/UpdateEffects.h"
#include "imagine_common/EstimateEffect.h"

#include "imagine_common/GetAdesPreConds.h"
#include "imagine_common/GetAdesEffects.h"
#include "imagine_common/GetAdesMotions.h"
#include "imagine_common/GetAdesMotionNames.h"
#include "imagine_common/GetAdesMotionSequenceNames.h"

#include "imagine_common/KeyValPair.h"
#include "imagine_common/Motion.h"
#include "imagine_common/MotionSequence.h"

class Ades2db_ros
{
    private:
        ades::Ades2DB database;

        ros::NodeHandle nh_;

        ros::ServiceServer ss_get_precond;
        ros::ServiceServer ss_get_effect;
        ros::ServiceServer ss_get_motion;
        ros::ServiceServer ss_get_motion_name;
        ros::ServiceServer ss_get_motion_sequence_name;
        ros::ServiceServer ss_list_ades;
        ros::ServiceServer ss_store_ades;
        ros::ServiceServer ss_update_ades;
        ros::ServiceServer ss_update_ades_motion;
        ros::ServiceServer ss_add_motion_sequence;
        ros::ServiceServer ss_delete_ades;
        ros::ServiceServer ss_update_effect_models;
        ros::ServiceServer ss_estimate_effect;

        ros::ServiceClient client_listmotion;
        ros::ServiceClient client_storemotion;
        ros::ServiceClient client_getmotion;
        ros::ServiceClient client_updatemotion;
        ros::ServiceClient client_deletemotion;

        ros::Publisher db_changed;

        // some flags
        bool shutdown;

    public:
        Ades2db_ros(ros::NodeHandle & n, std::string home, int version);
        ~Ades2db_ros();

        bool run();

        // callbacks / services
        bool get_preconds_srv(imagine_common::GetAdesPreConds::Request &rq, imagine_common::GetAdesPreConds::Response &rp);

        bool get_effects_srv(imagine_common::GetAdesEffects::Request &rq, imagine_common::GetAdesEffects::Response &rp);
        bool get_motions_srv(imagine_common::GetAdesMotions::Request &rq, imagine_common::GetAdesMotions::Response &rp);
        bool get_motion_names_srv(imagine_common::GetAdesMotionNames::Request &rq, imagine_common::GetAdesMotionNames::Response &rp);
        bool get_motion_sequence_names_srv(imagine_common::GetAdesMotionSequenceNames::Request &rq, imagine_common::GetAdesMotionSequenceNames::Response &rp);

        bool list_ades_srv(imagine_common::ListAdes::Request &rq, imagine_common::ListAdes::Response &rp);
        bool store_ades_srv(imagine_common::StoreAdes::Request &rq, imagine_common::StoreAdes::Response &rp);
        bool update_ades_srv(imagine_common::UpdateAdes::Request &rq, imagine_common::UpdateAdes::Response &rp);
        bool update_ades_motion_srv(imagine_common::UpdateAdesMotionNames::Request &rq, imagine_common::UpdateAdesMotionNames::Response &rp);
        bool delete_ades_srv(imagine_common::DeleteAdes::Request &rq, imagine_common::DeleteAdes::Response &rp);
        bool add_motion_sequence_srv(imagine_common::AddMotionSequence::Request &rq, imagine_common::AddMotionSequence::Response &rp);


        bool update_effect_models_srv(imagine_common::UpdateEffects::Request &rq, imagine_common::UpdateEffects::Response &rp);
        bool estimate_effect_srv(imagine_common::EstimateEffect::Request &rq, imagine_common::EstimateEffect::Response &rp);

};

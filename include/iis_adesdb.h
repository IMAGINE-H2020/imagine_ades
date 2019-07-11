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
#include "imagine_common/DeleteAdes.h"

#include "imagine_common/UpdateEffects.h"
#include "imagine_common/EstimateEffect.h"

#include "imagine_common/GetAdesPreConds.h"
#include "imagine_common/GetAdesEffects.h"
#include "imagine_common/GetAdesMotions.h"

#include "imagine_common/KeyValPair.h"
#include "imagine_common/Motion.h"
#include "imagine_common/MotionSequence.h"
/*
#include "uibk_libades_ros/ListAdes.h"
#include "uibk_libades_ros/StoreAdes.h"
#include "uibk_libades_ros/UpdateAdes.h"
#include "uibk_libades_ros/DeleteAdes.h"

#include "uibk_libades_ros/UpdateEffects.h"
#include "uibk_libades_ros/EstimateEffect.h"

#include "uibk_libades_ros/GetAdesPreConds.h"
#include "uibk_libades_ros/GetAdesEffects.h"
#include "uibk_libades_ros/GetAdesMotions.h"

#include "uibk_libades_ros/KeyValPair.h"
#include "uibk_libades_ros/Motion.h"
#include "uibk_libades_ros/MotionSequence.h"
*/

class Adesdb_ros
{
    private:
        ades::AdesDB database;

        ros::NodeHandle nh_;

        ros::ServiceServer ss_get_precond;
        ros::ServiceServer ss_get_effect;
        ros::ServiceServer ss_get_motion;
        ros::ServiceServer ss_list_ades;
        ros::ServiceServer ss_store_ades;
        ros::ServiceServer ss_update_ades;
        ros::ServiceServer ss_delete_ades;
        ros::ServiceServer ss_update_effect_models;
        ros::ServiceServer ss_estimate_effect;

        ros::Publisher db_changed;

        // some flags
        bool shutdown;

    public:
        Adesdb_ros(ros::NodeHandle & n, std::string home, int version);
        ~Adesdb_ros();

        bool run();

        // callbacks / services
        bool get_preconds_srv(imagine_common::GetAdesPreConds::Request &rq, imagine_common::GetAdesPreConds::Response &rp);

        bool get_effects_srv(imagine_common::GetAdesEffects::Request &rq, imagine_common::GetAdesEffects::Response &rp);
        bool get_motions_srv(imagine_common::GetAdesMotions::Request &rq, imagine_common::GetAdesMotions::Response &rp);

        bool list_ades_srv(imagine_common::ListAdes::Request &rq, imagine_common::ListAdes::Response &rp);
        bool store_ades_srv(imagine_common::StoreAdes::Request &rq, imagine_common::StoreAdes::Response &rp);
        bool update_ades_srv(imagine_common::UpdateAdes::Request &rq, imagine_common::UpdateAdes::Response &rp);
        bool delete_ades_srv(imagine_common::DeleteAdes::Request &rq, imagine_common::DeleteAdes::Response &rp);

        bool update_effect_models_srv(imagine_common::UpdateEffects::Request &rq, imagine_common::UpdateEffects::Response &rp);
        bool estimate_effect_srv(imagine_common::EstimateEffect::Request &rq, imagine_common::EstimateEffect::Response &rp);

};

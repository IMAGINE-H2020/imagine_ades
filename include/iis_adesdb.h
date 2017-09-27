#pragma once

//#include <map>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
//#include "iis_adesdb/MotionContainer.h"

#include <libades.h>
#include "iis_libades_ros/Dummy.h"
#include "iis_libades_ros/ListAdes.h"
#include "iis_libades_ros/StoreAdes.h"
#include "iis_libades_ros/UpdateAdes.h"
#include "iis_libades_ros/DeleteAdes.h"
#include "iis_libades_ros/UpdateEffects.h"
#include "iis_libades_ros/GetAdesPreConds.h"
#include "iis_libades_ros/GetAdesEffects.h"
#include "iis_libades_ros/GetAdesMotions.h"
#include "iis_libades_ros/KeyValPair.h"
#include "iis_libades_ros/Motion.h"
#include "iis_libades_ros/MotionSequence.h"

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


        // some flags
        bool shutdown;

    public:
        Adesdb_ros(ros::NodeHandle & n, std::string home, int version);
        ~Adesdb_ros();

        bool run();

        // callbacks / services
        bool get_preconds_srv(iis_libades_ros::GetAdesPreConds::Request &rq, iis_libades_ros::GetAdesPreConds::Response &rp);

        bool get_effects_srv(iis_libades_ros::GetAdesEffects::Request &rq, iis_libades_ros::GetAdesEffects::Response &rp);
        bool get_motions_srv(iis_libades_ros::GetAdesMotions::Request &rq, iis_libades_ros::GetAdesMotions::Response &rp);
        
        bool list_ades_srv(iis_libades_ros::ListAdes::Request &rq, iis_libades_ros::ListAdes::Response &rp);
        bool store_ades_srv(iis_libades_ros::StoreAdes::Request &rq, iis_libades_ros::StoreAdes::Response &rp);
        bool update_ades_srv(iis_libades_ros::UpdateAdes::Request &rq, iis_libades_ros::UpdateAdes::Response &rp);
        bool delete_ades_srv(iis_libades_ros::DeleteAdes::Request &rq, iis_libades_ros::DeleteAdes::Response &rp);

        bool update_effect_models_srv(iis_libades_ros::UpdateEffects::Request &rq, iis_libades_ros::UpdateEffects::Response &rp);

};

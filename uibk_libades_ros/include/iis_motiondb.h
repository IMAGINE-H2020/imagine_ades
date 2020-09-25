#pragma once

//#include <map>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>

#include <ades/libades.h>
#include "imagine_common/Dummy.h"

#include "imagine_common/ListMotion.h"
#include "imagine_common/StoreMotion.h"
#include "imagine_common/UpdateMotion.h"
#include "imagine_common/DeleteMotion.h"
#include "imagine_common/GetMotion.h"

#include "imagine_common/KeyValPair.h"
#include "imagine_common/Motion.h"
#include "imagine_common/MotionSequence.h"

class Motiondb_ros
{
    private:
        ades::MotionDB database;

        ros::NodeHandle nh_;

        ros::ServiceServer ss_get_motion;
        ros::ServiceServer ss_list_motion;
        ros::ServiceServer ss_store_motion;
        ros::ServiceServer ss_update_motion;
        ros::ServiceServer ss_delete_motion;
        ros::Publisher db_changed;

        // some flags
        bool shutdown;

    public:
        Motiondb_ros(ros::NodeHandle & n, std::string home, int version);
        ~Motiondb_ros();

        bool run();

        // callbacks / services
        bool get_motion_srv(imagine_common::GetMotion::Request &rq, imagine_common::GetMotion::Response &rp);

        bool list_motion_srv(imagine_common::ListMotion::Request &rq, imagine_common::ListMotion::Response &rp);
        bool store_motion_srv(imagine_common::StoreMotion::Request &rq, imagine_common::StoreMotion::Response &rp);
        bool update_motion_srv(imagine_common::UpdateMotion::Request &rq, imagine_common::UpdateMotion::Response &rp);
        bool delete_motion_srv(imagine_common::DeleteMotion::Request &rq, imagine_common::DeleteMotion::Response &rp);

};

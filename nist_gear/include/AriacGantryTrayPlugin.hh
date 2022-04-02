/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
/*
 * Desc: Tray located on the gantry
 * Author: Zeid Kootbally
 */
#ifndef _GAZEBO_GANTRY_TRAY_PLUGIN_HH_
#define _GAZEBO_GANTRY_TRAY_PLUGIN_HH_

#include <string>

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_broadcaster.h>
#include <string>

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_broadcaster.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/util/system.hh>
#include <nist_gear/ARIAC.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/util/system.hh>
#include <string>
#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/util/system.hh>
#include <gazebo/sensors/sensors.hh>
#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/util/system.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>

namespace gazebo {

    class AriacGantryTrayPlugin : public SensorPlugin
    {

    public:
        AriacGantryTrayPlugin();
        virtual ~AriacGantryTrayPlugin();
        virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    private:
        virtual void OnUpdate();
        sensors::ContactSensorPtr parentSensor;
        event::ConnectionPtr updateConnection;
        GazeboRosPtr gazeboRos;
        ros::Publisher contactPub;
        transport::NodePtr gazebo_node;
        int update_rate;
    };

}

#endif

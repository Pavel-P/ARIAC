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

// correct tray - wrong pose
// correct parts

// not all point bonus

// wrong tray - 0 for correct pose and not all bonus
// correct -

// wrong tray and wron pose similar to number 2
#include <cstdlib>
#include <string>

#include "AriacGantryTrayPlugin.hh"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(AriacGantryTrayPlugin)

/////////////////////////////////////////////////
AriacGantryTrayPlugin::AriacGantryTrayPlugin() : SensorPlugin()
{
}

/////////////////////////////////////////////////
AriacGantryTrayPlugin::~AriacGantryTrayPlugin()
{

}

/////////////////////////////////////////////////
void AriacGantryTrayPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  gzdbg << "AriacGantryTrayPlugin: Starting contact sensor on gantry tray.";
  
// Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libGantryTrayPlugin.so' in the gazebo_ros package)");
    return;
  }

  this->update_rate = 0;
  if (_sdf->HasElement("updateRate"))
    this->update_rate = _sdf->Get<int>("updateRate");
  
  if (this->update_rate > 0)
    gzdbg << "GantryTrayPlugin running at " << this->update_rate << " Hz\n";
  else
    gzdbg << "GantryTrayPlugin running at the default update rate\n";

  // std::string topic;
  // double rate;
  // gazeboRos = GazeboRosPtr(new GazeboRos(_sensor, _sdf, "ContactSensor"));
  // gazeboRos->isInitialized();
  // gazeboRos->getParameter<std::string>(topic, "topicName", "/gazebo/base_collision");
  // gazeboRos->getParameter<double>(rate, "updateRate", 10.0);
  // contactPub = gazeboRos->node()->advertise<gazebo_model_collision_plugin::Contact>(topic, 5);

  // Get the parent sensor.
  this->parentSensor = std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor) {
    gzerr << "CollisionPlugin requires a ContactSensor.\n";
    return;
  }

  this->parentSensor->SetUpdateRate(this->update_rate);

  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated(std::bind(&AriacGantryTrayPlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);

  // Initialize Gazebo transport
  this->gazebo_node = transport::NodePtr(new transport::Node());
  this->gazebo_node->Init();
}

/////////////////////////////////////////////////
void AriacGantryTrayPlugin::OnUpdate()
{
  // gazebo_model_collision_plugin::Contact msg;
  // msg.header.stamp = ros::Time::now();
  // msg.header.frame_id = this->parentSensor->ParentName();
  // msg.header.frame_id = msg.header.frame_id.substr(msg.header.frame_id.find("::") + 2);
  // std::vector<std::string> objs_hit;

  // msgs::Contacts contacts;
  // contacts = this->parentSensor->Contacts();
  // for (unsigned int i = 0; i < contacts.contact_size(); ++i) {
  //   std::string obj_name = contacts.contact(i).collision2();
  //   while (true) {
  //     if (obj_name.find(std::string("::")) != std::string::npos) {
  //       obj_name = obj_name.substr(obj_name.find("::") + 2);
  //     }
  //     else {
  //       break;
  //     }
  //   }

  //   if (obj_name == "ground" || obj_name == "sun") {
  //     continue;
  //   }

  //   if (std::find(objs_hit.begin(), objs_hit.end(), obj_name) == objs_hit.end()) {
  //     objs_hit.push_back(obj_name);
  //   }
  // }

  // if (objs_hit.size() > 0 || contactPub.getNumSubscribers() < 1) {
  //   msg.objects_hit = objs_hit;
  //   contactPub.publish(msg);
  // }

  return;
}

// /////////////////////////////////////////////////
// void GantryTrayPlugin::ProcessContactingModels()
// {
  
// }

// /////////////////////////////////////////////////
// void GantryTrayPlugin::OnSubscriberConnect(const ros::SingleSubscriberPublisher& pub)
// {
//   //disable subscription in competition mode
//   auto subscriberName = pub.getSubscriberName();
//   gzwarn << "New subscription from node: " << subscriberName << std::endl;
// }


// /////////////////////////////////////////////////
// void GantryTrayPlugin::UnlockContactingModels()
// {
//   boost::mutex::scoped_lock lock(this->mutex);
//   physics::JointPtr fixedJoint;

//   for (auto fixedJoint : this->fixedJoints)
//   {
//     fixedJoint->Detach();
//   }
//   this->fixedJoints.clear();

//   for (auto model : this->contactingModels)
//   {
//     model->SetGravityMode(false);
//     auto modelName = model->GetName();
//     auto linkName = modelName + "::link";
//     auto link = model->GetLink(linkName);
//     if (link == NULL)
//     {
//       // If the model was inserted into the world using the "population" SDF tag,
//       // the link will have an additional namespace of the model type.
//       linkName = modelName + "::" + ariac::DetermineModelType(modelName) + "::link";
//       link = model->GetLink(linkName);
//       if (link == NULL)
//       {
//         gzwarn << "Couldn't find link to remove joint with: " << linkName;
//         continue;
//       }
//     }
//     link->SetGravityMode(true);
//     model->SetAutoDisable(true);
//   }
// }


// /////////////////////////////////////////////////
// void GantryTrayPlugin::LockContactingModels()
// {
//   boost::mutex::scoped_lock lock(this->mutex);
//   physics::JointPtr fixedJoint;
//   gzdbg << "Number of models in contact with the tray: " << this->contactingModels.size() << std::endl;
//   for (auto model : this->contactingModels)
//   {
//   // Create the joint that will attach the models
//   fixedJoint = this->world->Physics()->CreateJoint(
//         "fixed", this->model);
//   auto jointName = this->model->GetName() + "_" + model->GetName() + "__joint__";
//   gzdbg << "Creating fixed joint: " << jointName << std::endl;
//   fixedJoint->SetName(jointName);

//   model->SetGravityMode(false);

//   // Lift the part slightly because it will fall through the tray if the tray is animated
//   model->SetWorldPose(model->WorldPose() + ignition::math::Pose3d(0,0,0.01,0,0,0));

//   auto modelName = model->GetName();
//   auto linkName = modelName + "::link";
//   auto link = model->GetLink(linkName);
//   if (link == NULL)
//   {
//     // If the model was inserted into the world using the "population" SDF tag,
//     // the link will have an additional namespace of the model type.
//     linkName = modelName + "::" + ariac::DetermineModelType(modelName) + "::link";
//     link = model->GetLink(linkName);
//     if (link == NULL)
//     {
//       gzwarn << "Couldn't find link to make joint with: " << linkName;
//       continue;
//     }
//   }
//   link->SetGravityMode(false);
//   fixedJoint->Load(link, this->parentLink, ignition::math::Pose3d());
//   fixedJoint->Attach(this->parentLink, link);
//   fixedJoint->Init();
//   this->fixedJoints.push_back(fixedJoint);
//   model->SetAutoDisable(true);
//   }
// }

// /////////////////////////////////////////////////
// // void GantryTrayPlugin::HandleLockModelsRequest(ConstGzStringPtr &_msg)
// // {
// //   gzdbg << "Handle Lock Models service called.\n";
// //   (void)_msg;
// //   this->LockContactingModels();
// // }





// bool GantryTrayPlugin::HandleLockModelsRequest(ros::ServiceEvent<std_srvs::Trigger::Request, 
// std_srvs::Trigger::Response>& event)
// {
//   std_srvs::Trigger::Response& res = event.getResponse();

//   const std::string& callerName = event.getCallerName();
//   gzdbg << "Handle lock models service called by: " << callerName << std::endl;

//   // (void)_msg;
//   this->LockContactingModels();
//   res.success = true;
//   return true;
// }

// // void GantryTrayPlugin::HandleUnlockModelsRequest(ConstGzStringPtr &_msg)
// // {
// //   gzdbg << "Handle Unlock Models service called.\n";
// //   (void)_msg;
// //   this->UnlockContactingModels();
// // }

// bool GantryTrayPlugin::HandleUnlockModelsRequest(ros::ServiceEvent<std_srvs::Trigger::Request, 
// std_srvs::Trigger::Response>& event)
// {
//   std_srvs::Trigger::Response& res = event.getResponse();

//   const std::string& callerName = event.getCallerName();
//   gzdbg << "Handle unlock tray service called by: " << callerName << std::endl;

//   this->UnlockContactingModels();
//   res.success = true;
//   return true;
// }


// /////////////////////////////////////////////////
// bool GantryTrayPlugin::HandleClearService(
//   ros::ServiceEvent<std_srvs::Trigger::Request, std_srvs::Trigger::Response>& event)
// {
//   std_srvs::Trigger::Response& res = event.getResponse();

//   const std::string& callerName = event.getCallerName();
//   gzdbg << "Handle gantry clear tray service called by: " << callerName << std::endl;

//   // During the competition, this environment variable will be set.
//   auto compRunning = std::getenv("ARIAC_COMPETITION");
//   if (compRunning && callerName.compare("/gazebo") != 0)
//   {
//     std::string errStr = "Competition is running so this service is not enabled.";
//     gzerr << errStr << std::endl;
//     ROS_ERROR_STREAM(errStr);
//     res.success = false;
//     return true;
//   }

//   this->UnlockContactingModels();
//   this->ClearContactingModels();
//   res.success = true;
//   return true;
// }


// void GantryTrayPlugin::PublishTFTransform(const common::Time sim_time)
// {
//   ignition::math::Pose3d objectPose = this->tray_pose;
//   geometry_msgs::TransformStamped tfStamped;
//   tfStamped.header.stamp = ros::Time(sim_time.sec, sim_time.nsec);
//   tfStamped.header.frame_id = "world";
//   tfStamped.child_frame_id = this->tf_frame_name;
//   tfStamped.transform.translation.x = objectPose.Pos().X();
//   tfStamped.transform.translation.y = objectPose.Pos().Y();
//   tfStamped.transform.translation.z = objectPose.Pos().Z();
//   tfStamped.transform.rotation.x = objectPose.Rot().X();
//   tfStamped.transform.rotation.y = objectPose.Rot().Y();
//   tfStamped.transform.rotation.z = objectPose.Rot().Z();
//   tfStamped.transform.rotation.w = objectPose.Rot().W();

//   tf_broadcaster.sendTransform(tfStamped);
// }

#include <ros/ros.h>
#include <string.h>
#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/SwitchController.h>

std::vector<std::string> GetStaticControllers(ros::NodeHandle& nh, std::string robot_name) {
    std::vector<std::string> static_controllers;

    std::string srv_name = "/ariac/" + robot_name + "/controller_manager/list_controllers";
    ros::ServiceClient client = nh.serviceClient<controller_manager_msgs::ListControllers>(srv_name);
    controller_manager_msgs::ListControllers srv;

    if (client.call(srv)){
      for (auto state : srv.response.controller) {
            if (state.name.rfind("static", 0) == 0){
                static_controllers.push_back(state.name);
            }
        }
    }
    else {
        ROS_ERROR_STREAM("Unable to call " << srv_name);
    }

    return static_controllers;
}

void StopRobot(ros::NodeHandle& nh, std::vector<std::string> static_controllers, std::string robot_name) {
    controller_manager_msgs::SwitchController srv;
    srv.request.start_controllers = static_controllers;
    srv.request.stop_controllers.push_back(robot_name + "_arm_controller");

    std::string srv_name = "/ariac/" + robot_name + "/controller_manager/switch_controller";
    ros::ServiceClient client = nh.serviceClient<controller_manager_msgs::SwitchController>(srv_name);

    srv.request.strictness = controller_manager_msgs::SwitchControllerRequest::BEST_EFFORT;

    if (client.call(srv)) {
        if (srv.response.ok) {
            ROS_INFO_STREAM("Stopped " << robot_name << " controller");
        }
        else {
            ROS_ERROR_STREAM("Unable to stop controller");
        }   
    }
}

void StartRobot(ros::NodeHandle& nh, std::vector<std::string> static_controllers, std::string robot_name) {
    controller_manager_msgs::SwitchController srv;
    srv.request.stop_controllers = static_controllers;
    srv.request.start_controllers.push_back(robot_name + "_arm_controller");

    std::string srv_name = "/ariac/" + robot_name + "/controller_manager/switch_controller";
    ros::ServiceClient client = nh.serviceClient<controller_manager_msgs::SwitchController>(srv_name);

    srv.request.strictness = 1;

    if (client.call(srv)) {
        if (srv.response.ok) {
            ROS_INFO_STREAM("Started " << robot_name << " controller");
        }
        else {
            ROS_ERROR_STREAM("Unable to start controller");
        }   
    }
}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "controller_pauser");
    ros::NodeHandle node;

    bool stop_kitting;
    bool start_kitting;
    bool stop_gantry;
    bool start_gantry;

    ros::Rate rate(10);

    while (ros::ok()) {
        node.param<bool>("/stop_kitting_robot", stop_kitting, false);
        node.param<bool>("/start_kitting_robot", start_kitting, false);
        node.param<bool>("/stop_gantry_robot", stop_gantry, false);
        node.param<bool>("/start_gantry_robot", start_gantry, false);

        if (stop_kitting) {
            std::string robot_name = "kitting";
            std::vector<std::string> static_controllers = GetStaticControllers(node, robot_name);
            StopRobot(node, static_controllers, robot_name);
            node.setParam("/stop_kitting_robot", false);
        } else if (start_kitting) {
            std::string robot_name = "kitting";
            std::vector<std::string> static_controllers = GetStaticControllers(node, robot_name);
            StartRobot(node, static_controllers, robot_name);
            node.setParam("/start_kitting_robot", false);
        } else if (stop_gantry) {
            std::string robot_name = "gantry";
            std::vector<std::string> static_controllers = GetStaticControllers(node, robot_name);
            StopRobot(node, static_controllers, robot_name);
            node.setParam("/stop_gantry_robot", false);
        } else if (start_gantry) {
            std::string robot_name = "gantry";
            std::vector<std::string> static_controllers = GetStaticControllers(node, robot_name);
            StartRobot(node, static_controllers, robot_name);
            node.setParam("/start_gantry_robot", false);
        }

        rate.sleep();
    }

    return 0;
}
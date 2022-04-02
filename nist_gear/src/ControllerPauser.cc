#include <ros/ros.h>
#include <string.h>
#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/SwitchController.h>

std::vector<std::string> GetStaticControllers(ros::NodeHandle& nh, std::string robot_name) {
    std::vector<std::string> static_controllers;

    std::string srv_name = "/ariac/" + robot_name + "/controller_manager/list_controllers";
    ros::ServiceClient client = nh.serviceClient<controller_manager_msgs::ListControllers>(srv_name);
    controller_manager_msgs::ListControllers srv;

    if (client.call(srv)) {
        for (auto state : srv.response.controller) {
            if (state.name.rfind("static", 0) == 0) {
                static_controllers.push_back(state.name);
                // ROS_INFO_STREAM(state.name);
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

    if (robot_name == "gantry") {
        // Also stop the gantry
        srv.request.stop_controllers.push_back(robot_name + "_controller");
    }

    std::string srv_name = "/ariac/" + robot_name + "/controller_manager/switch_controller";
    ros::ServiceClient client = nh.serviceClient<controller_manager_msgs::SwitchController>(srv_name);

    srv.request.strictness = 1;

    if (client.call(srv)) {
        if (srv.response.ok) {
            // gzdbg << "Stopped " << robot_name << " controller\n";
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

    if (robot_name == "gantry") {
        // Also start the gantry
        srv.request.start_controllers.push_back(robot_name + "_controller");
    }

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

int main(int argc, char** argv) {
    ros::init(argc, argv, "controller_pauser");
    ros::NodeHandle node;

    std::string robot_name = "kitting";

    std::vector<std::string> static_controllers = GetStaticControllers(node, robot_name);

    StopRobot(node, static_controllers, robot_name);

    ros::Duration(30).sleep();

    StartRobot(node, static_controllers, robot_name);

    return 0;
}
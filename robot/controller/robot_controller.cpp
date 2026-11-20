// Copyright 2023 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stddef.h>

#include <string>
#include <vector>

#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "robot_controler.hpp"
#include "wire_config.hpp"

using config_type = controller_interface::interface_configuration_type;

namespace Controller
{
RobotController::RobotController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn RobotController::on_init()
{
    // should have error handling
    joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
    command_interface_types_ =
        auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
    state_interface_types_ =
        auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

    return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration RobotController::command_interface_configuration()
    const
{
    controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

    conf.names.reserve(joint_names_.size() * command_interface_types_.size());
    for (const auto &joint_name : joint_names_)
    {
        for (const auto &interface_type : command_interface_types_)
        {
            conf.names.push_back(joint_name + "/" + interface_type);
        }
    }

    return conf;
}

controller_interface::InterfaceConfiguration RobotController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

    conf.names.reserve(joint_names_.size() * state_interface_types_.size());
    for (const auto &joint_name : joint_names_)
    {
        for (const auto &interface_type : state_interface_types_)
        {
            conf.names.push_back(joint_name + "/" + interface_type);
        }
    }

    return conf;
}

controller_interface::CallbackReturn RobotController::on_configure(const rclcpp_lifecycle::State &)
{
    std::string error;
    if (!ik_.init(get_robot_description(), "world", "end_effector_pose", error))
    {
        RCLCPP_ERROR(get_node()->get_logger(), "Error building ik, %s", error.c_str());
        return CallbackReturn::ERROR;
    }

    if (ik_.jointNames() != joint_names_)
    {
        RCLCPP_ERROR(get_node()->get_logger(), "Joint names differ in IK setup");
        return CallbackReturn::ERROR;
    }

    q_.resize(ik_.numJoints());

    try
    {
        wire_.emplace(wire::loadWire(*get_node()));  // same function + params as wire_generator
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to build wire: %s", e.what());
        return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(get_node()->get_logger(), "Wire loaded, length %.3f m", wire_->length());
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotController::on_activate(const rclcpp_lifecycle::State &)
{
    // clear out vectors in case of restart
    joint_position_command_interface_.clear();
    joint_velocity_command_interface_.clear();
    joint_position_state_interface_.clear();
    joint_velocity_state_interface_.clear();

    // assign command interfaces
    for (auto &interface : command_interfaces_)
    {
        command_interface_map_[interface.get_interface_name()]->push_back(interface);
    }

    // assign state interfaces
    for (auto &interface : state_interfaces_)
    {
        state_interface_map_[interface.get_interface_name()]->push_back(interface);
    }

    return CallbackReturn::SUCCESS;
}

controller_interface::return_type RobotController::update(
    const rclcpp::Time &time,
    const rclcpp::Duration & /*period*/)
{
    // get ee_pose from ik_solver fk
    for (size_t i = 0; i < joint_position_state_interface_.size(); i++)
    {
        std::optional<double> joint_position =
            joint_position_state_interface_[i].get().get_optional();
        if (!joint_position.has_value())
        {
            return controller_interface::return_type::OK;
        }

        q_(static_cast<unsigned int>(i)) = joint_position.value();
    }

    if (!ik_.fk(q_, ee_pose_))
    {
        return controller_interface::return_type::ERROR;
    }

    double s = wire_->project(Eigen::Vector3d(ee_pose_.p.x(), ee_pose_.p.y(), ee_pose_.p.z()));

    Eigen::Vector3d goal_pose = wire_->position(s);

    RCLCPP_INFO_THROTTLE(
        get_node()->get_logger(),
        *get_node()->get_clock(),
        500,
        "Wire Pose: %.4f, Goal pose: x %.2f, y %.2f, z %.2f",
        s,
        goal_pose[0],
        goal_pose[1],
        goal_pose[2]);

    return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn RobotController::on_deactivate(const rclcpp_lifecycle::State &)
{
    return CallbackReturn::SUCCESS;
}

}  // namespace Controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(Controller::RobotController, controller_interface::ControllerInterface)
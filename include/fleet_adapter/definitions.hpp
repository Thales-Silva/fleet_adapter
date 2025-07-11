#ifndef DEFINITIONS_HPP
#define DEFINITIONS_HPP

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <rmf_fleet_adapter/agv/Adapter.hpp>
#include <rmf_fleet_adapter/agv/EasyFullControl.hpp>

namespace fleet_adapter
{
    typedef rmf_fleet_adapter::agv::Adapter Adapter;
    typedef rmf_fleet_adapter::agv::EasyFullControl EasyFullControl;
    typedef rmf_fleet_adapter::agv::EasyFullControl::FleetConfiguration FleetConfiguration;
    typedef rmf_fleet_adapter::agv::EasyFullControl::RobotConfiguration RobotConfiguration;
    typedef rmf_fleet_adapter::agv::EasyFullControl::RobotCallbacks RobotCallbacks;
    typedef rmf_fleet_adapter::agv::EasyFullControl::RobotState RobotState;
    typedef rmf_fleet_adapter::agv::RobotUpdateHandle RobotUpdateHandle;
    typedef rmf_fleet_adapter::agv::EasyFullControl::EasyRobotUpdateHandle EasyRobotUpdateHandle;
    typedef std::shared_ptr<EasyRobotUpdateHandle> EasyRobotUpdateHandlePtr;
    typedef std::shared_ptr<RobotUpdateHandle> RobotUpdateHandlePtr;
    typedef rmf_fleet_adapter::agv::EasyFullControl::Destination Destination;
    typedef rmf_fleet_adapter::agv::EasyFullControl::CommandExecution CommandExecution;
    typedef rmf_fleet_adapter::agv::EasyFullControl::NavigationRequest NavigationRequest;
    typedef rmf_fleet_adapter::agv::RobotUpdateHandle::ActivityIdentifier ActivityIdentifier;
    typedef rmf_fleet_adapter::agv::RobotUpdateHandle::ActivityIdentifierPtr ActivityIdentifierPtr;
    typedef rmf_fleet_adapter::agv::RobotUpdateHandle::ConstActivityIdentifierPtr ConstActivityIdentifierPtr;
    typedef nav2_msgs::action::NavigateToPose NavigateToPose;
    typedef rclcpp_action::ClientGoalHandle<NavigateToPose> NavGoalHandle;

    inline void exit_with_exception(std::string msg)
    {
        rclcpp::shutdown();
        throw std::runtime_error(msg);
    }
}

#endif
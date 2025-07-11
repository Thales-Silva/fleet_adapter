#ifndef DATA_HPP
#define DATA_HPP

#include <fleet_adapter/robot.hpp>

namespace fleet_adapter
{
    struct Data
    {
        std::string fleet_name;
        std::vector<std::thread> threads;
        rclcpp::Node::SharedPtr node_ = nullptr;
        std::shared_ptr<FleetConfiguration> config_ = nullptr;
        std::shared_ptr<Adapter> adapter_ = nullptr;
        std::shared_ptr<EasyFullControl> fleet_handle_ = nullptr;
        std::unordered_map<std::string, std::shared_ptr<fleet_adapter::Robot>> robots;
    };
}

#endif
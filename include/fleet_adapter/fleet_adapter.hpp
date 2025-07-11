#ifndef FLEET_ADAPTER_ROSI_HPP
#define FLEET_ADAPTER_ROSI_HPP

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <rmf_fleet_adapter/agv/EasyFullControl.hpp>

#include <fleet_adapter/definitions.hpp>
#include <fleet_adapter/robot.hpp>
#include <fleet_adapter/data.hpp>

#include <yaml-cpp/yaml.h>

namespace fleet_adapter
{
    class FleetAdapter : public std::enable_shared_from_this<FleetAdapter>
    {
        private:
            std::vector<std::string> ns;
            std::vector<std::string> name;
            std::vector<std::string> initial_map_name;
            std::vector<std::string> charger_name;
            
            std::vector<std::vector<double>> rmf_pts;
            std::vector<std::vector<double>> odom_pts;

            double robot_update_period;
            
            FleetAdapter();
            std::shared_ptr<fleet_adapter::Data> data_;
            void initialize();

        public:
            static std::shared_ptr<FleetAdapter> create();    
            ~FleetAdapter();
            void addRobots();
            void run();
    };
}

#endif
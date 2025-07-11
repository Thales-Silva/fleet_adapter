#include <fleet_adapter/fleet_adapter.hpp>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto fleet_adapter = fleet_adapter::FleetAdapter::create();

    fleet_adapter->run();

    rclcpp::shutdown();

    return 0;
}
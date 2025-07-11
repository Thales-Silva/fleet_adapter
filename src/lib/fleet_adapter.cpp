#include <fleet_adapter/fleet_adapter.hpp>

using namespace fleet_adapter;

std::shared_ptr<FleetAdapter> FleetAdapter::create()
{
    auto instance = std::shared_ptr<FleetAdapter>(new FleetAdapter());
    instance->initialize();
    return instance;
}

void FleetAdapter::initialize()
{
    data_ = std::make_shared<fleet_adapter::Data>();
    
    data_->adapter_ = Adapter::make("fleet_adapter");
    data_->node_ = std::make_shared<rclcpp::Node>("robot_manager");
    data_->adapter_->start();

    RCLCPP_INFO(data_->node_->get_logger(), "Fleet adapter bringup.");

    data_->threads.push_back(std::thread( 
        [n = data_->node_]() -> void
        {
            while (rclcpp::ok())
                rclcpp::spin_some(n);
        }
    ));

    std::string fleet_config_path = data_->node_->declare_parameter<std::string>("fleet_config_path", std::string(""));
    if (fleet_config_path.empty())
        exit_with_exception("[ ERROR] No parameter fleet_config_path wasn't found in launch file.");

    std::string nav_graph_path = data_->node_->declare_parameter<std::string>("nav_graph_path", std::string(""));
    if (nav_graph_path.empty())
        exit_with_exception("[ ERROR] No parameter nav_graph_path wasn't found in launch file.");

    auto config = EasyFullControl::FleetConfiguration::from_config_files(fleet_config_path, nav_graph_path);
    if (!config.has_value())
        exit_with_exception("[ ERROR] Failed to load fleet_config_path and/or nav_graph_path.");

    RCLCPP_INFO(data_->node_->get_logger(), "Configuring fleet [%s].", data_->fleet_name.c_str());
    
    data_->fleet_name = config->fleet_name();
    if (data_->fleet_name.empty())
        exit_with_exception("[ ERROR] No parameter fleet_name wasn't found in configuration file.");

    data_->fleet_handle_ = data_->adapter_->add_easy_fleet(config.value());

    YAML::Node node = YAML::LoadFile(fleet_config_path);

    robot_update_period = 0.1;

    if (node["rmf_fleet"]["robot_state_update_frequency"].IsDefined())
        robot_update_period = 1 / node["rmf_fleet"]["robot_state_update_frequency"].as<double>();
    else
        RCLCPP_WARN(data_->node_->get_logger(), "robot_state_update_frequency not found on configuration file. Using 10 Hz.");
    
    auto robots = node["rmf_fleet"]["robots"];
    if (robots.IsDefined())
    {
        for (auto i = robots.begin(); i != robots.end(); i++)
        {
            auto robot_i = i->second;
            
            name.push_back(i->first.as<std::string>());
            
            if (!robot_i["namespace"].IsDefined())
                exit_with_exception("[ ERROR] Parameter namespace of a robot wasn't found in configuration file.");

            ns.push_back(robot_i["namespace"].as<std::string>());

            initial_map_name.push_back(robot_i["initial_map"].as<std::string>());
            charger_name.push_back(robot_i["charger"].as<std::string>());
        }
    }
    else
        exit_with_exception("[ ERROR] Parameter rmf_fleet.robots wasn't found in configuration file.");

    auto rmf_yaml = node["rmf_fleet"]["reference_coordinates"]["L1"]["rmf"];
    auto odom_yaml = node["rmf_fleet"]["reference_coordinates"]["L1"]["robot"];
    
    if (rmf_yaml.IsDefined())
        rmf_pts = rmf_yaml.as<std::vector<std::vector<double>>>();

    if (odom_yaml.IsDefined())
        odom_pts = odom_yaml.as<std::vector<std::vector<double>>>();    

    RCLCPP_INFO(data_->node_->get_logger(), "Adding robots to the fleet.");

    addRobots();
}

void FleetAdapter::addRobots()
{
    auto add_robot = [data = data_](const std::string &ns,
        const std::string &name,
        const std::string charger_name,
        const std::string &map_name,
        rclcpp::Node::SharedPtr n_,
        std::vector<std::vector<double>> rmf_points,
        std::vector<std::vector<double>> odom_points) -> void
    {
        auto insertion = data->robots.insert({name, nullptr});
        if (!insertion.second)
        {
            RCLCPP_WARN(data->node_->get_logger(), "Attempted to add robot with name [%s] more than once. Ignoring...", name.c_str());
            return;
        }

        RCLCPP_INFO(data->node_->get_logger(), "Adding robot [%s] to the fleet.", name.c_str());

        auto robot = std::make_shared<Robot>();

        if (robot->initialize(ns, name, charger_name, map_name, n_))
        {
            insertion.first->second = std::move(robot);

            RobotCallbacks robot_cbs(
                [r = insertion.first->second, n = n_](Destination dest, CommandExecution cmd_execution) -> void
                {
                    r->navigate(dest, cmd_execution);
                },
                [r = insertion.first->second, n = n_](RobotUpdateHandle::ConstActivityIdentifierPtr id_) -> void
                {
                    r->stop();
                },
                [r = insertion.first->second, n = n_](const std::string& category, const nlohmann::json& description,
                    RobotUpdateHandle::ActionExecution execution) -> void
                {
                    RCLCPP_INFO(n->get_logger(), "Received action request [%s] for robot [%s]", 
                            category.c_str(), r->getName().c_str());
                    
                    // exemplo simples
                    if (category == "charge")
                    {
                        execution.underway("Starting charging");
                        // lógica de carregamento
                        execution.finished();
                    }
                    else
                        execution.error("Unknown action category");
                }
            );
            
            robot_cbs.with_localization(
                {}
            );

            insertion.first->second->setOdom2RmfTransform(rmf_points, odom_points);
            
            if (!insertion.first->second->startOdometrySubscriber())
            {
                RCLCPP_WARN(data->node_->get_logger(), "Robot [%s] odom topic is mute. Could not add robot to the fleet.", name.c_str());
                return;
            }

            RCLCPP_INFO(data->node_->get_logger(), "Adding robot [%s] to the map [%s] with intial pose x: %f, y: %f, phi: %f.",
                name.c_str(),
                insertion.first->second->getState().map().c_str(),
                insertion.first->second->getState().position().x(),
                insertion.first->second->getState().position().y(),
                insertion.first->second->getState().position().z());

            RobotConfiguration robot_config(std::vector<std::string>(1, charger_name),
                false,
                0.3,
                0.3);
            
            insertion.first->second->getRobotHandlePtr() = data->fleet_handle_->add_robot(insertion.first->first,
                insertion.first->second->getState(),
                robot_config,
                robot_cbs);
            
            if (!insertion.first->second->getRobotHandlePtr())
            {
                RCLCPP_INFO(data->node_->get_logger(), "Method add_robot() returned a nullptr."
                " Robot [%s] could not be added to the fleet.", name.c_str());
                return;
            }

            insertion.first->second->startRobotUpdate();
        }
        else
            RCLCPP_INFO(data->node_->get_logger(), "Failed to initialize."
            " Robot [%s] could not be added to the fleet.", name.c_str());
    }; 

    for (size_t i = 0; i != name.size(); i++)
        data_->threads.push_back(std::thread(add_robot,
            ns[i],
            name[i],
            charger_name[i],
            initial_map_name[i],
            data_->node_,
            rmf_pts,
            odom_pts));
}

void FleetAdapter::run()
{
    data_->adapter_->wait();
}

FleetAdapter::FleetAdapter()
{

}

FleetAdapter::~FleetAdapter()
{
    
}
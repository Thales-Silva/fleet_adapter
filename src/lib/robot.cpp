#include <fleet_adapter/robot.hpp>

using namespace fleet_adapter;

Robot::Robot() :
    node_(nullptr),
    odom_sub_(nullptr),
    nav2_client_(nullptr),
    odom_rcv(false),
    update_running(false),
    activity_id_(nullptr),
    name(""),
    charger(""),
    map_name(""),
    battery_soc(0.99),
    robot_handle_(nullptr),
    goal_handle_(nullptr)
{

}

bool Robot::initialize(const std::string& ns,
    const std::string& robot_name,
    const std::string& charger_name,
    const std::string& initial_map_name,
    rclcpp::Node::SharedPtr n_)
{
    name = robot_name;
    charger = charger_name;
    map_name = initial_map_name;
    node_ = n_;

    nav2_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "/navigate_to_pose");

    RCLCPP_INFO(node_->get_logger(), "Blocking this thread until Nav2 action server of robot %s is up.", name.c_str());
    
    if (!nav2_client_->wait_for_action_server(std::chrono::milliseconds(5000)))
    {
        RCLCPP_INFO(node_->get_logger(), "Robot %s initialization failed. Nav2 action server wasn't found after 5 seconds.", name.c_str());
        return false;
    }
    
    robot_state_ = std::make_shared<RobotState>(map_name, Eigen::Vector3d::Zero(), battery_soc);

    return true;
}

void Robot::setOdom2RmfTransform(std::vector<std::vector<double>> rmf_points,
    std::vector<std::vector<double>> odom_points)
{
    Eigen::MatrixXd p_rmf(rmf_points[0].size() + 1, rmf_points.size());
    Eigen::MatrixXd p_odom(odom_points[0].size() + 1, odom_points.size());

    for (size_t i = 0; i != rmf_points.size(); i++)
    {
        p_rmf.col(i) = Eigen::Vector3d(rmf_points[i][0], rmf_points[i][1], 1.0);
        p_odom.col(i) = Eigen::Vector3d(odom_points[i][0], odom_points[i][1], 1.0);
    }

    Eigen::Matrix3d eig_rmf_odom = p_rmf * p_odom.transpose() * (p_odom * p_odom.transpose()).inverse();

    std::cerr << "Homogeneous matrix transform from odom -> rmf is: " << std::endl;
    std::cerr << eig_rmf_odom << std::endl;

    tf2::Vector3 p_rmf_odom(eig_rmf_odom.col(2)[0], eig_rmf_odom.col(2)[1], 0);
    tf2::Quaternion q_rmf_odom;

    double yaw = std::atan2(eig_rmf_odom(1,0), eig_rmf_odom(0,0));
    q_rmf_odom.setRPY(0, 0, yaw);

    t_rmf_odom.setOrigin(p_rmf_odom);
    t_rmf_odom.setRotation(q_rmf_odom);

    t_odom_rmf = t_rmf_odom.inverse();
}

void Robot::navigate(Destination desired_pose, CommandExecution cmd_exec)
{
    {
        std::lock_guard<std::mutex> lock(mute);
        if (goal_handle_)
        {
            RCLCPP_WARN(node_->get_logger(), 
                "Previous goal still active. Cancelling before sending new one for robot [%s].",
                name.c_str());

            nav2_client_->async_cancel_goal(goal_handle_);
            goal_handle_.reset();
        }
    }

    tf2::Quaternion q_rmf;
    q_rmf.setRPY(0.0, 0.0, desired_pose.yaw());

    tf2::Vector3 p_rmf(desired_pose.position().x(),
                       desired_pose.position().y(),
                       0);
    
    tf2::Transform t_robot_rmf = t_odom_rmf * tf2::Transform(q_rmf, p_rmf);

    NavigateToPose::Goal goal = NavigateToPose::Goal();
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = node_->get_clock()->now();

    goal.pose.pose.position.set__x(t_robot_rmf.getOrigin().getX());
    goal.pose.pose.position.set__y(t_robot_rmf.getOrigin().getY());
    goal.pose.pose.set__orientation(tf2::toMsg(t_robot_rmf.getRotation()));

    auto goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    goal_options.goal_response_callback =
    [this, cmd = cmd_exec](const NavGoalHandle::SharedPtr &handle_) -> void
    {
        if (!handle_)
            RCLCPP_ERROR(node_->get_logger(),
            "Navigation goal was rejected by server on robot [%s]", name.c_str());
        else
        {
            RCLCPP_INFO(node_->get_logger(),
            "Navigation goal was accepted by server on robot [%s]", name.c_str());

            goal_handle_ = handle_;
            activity_id_ = cmd.identifier();
        }
    };

    goal_options.result_callback =
    [this, cmd = &cmd_exec](const NavGoalHandle::WrappedResult &result) -> void
    {
        switch (result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(node_->get_logger(),
                    "Goal succeeded for robot %s.", name.c_str());
                break;

            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(node_->get_logger(),
                    "Goal aborted for robot %s.", name.c_str());
                break;

            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(node_->get_logger(),
                    "Goal canceled for robot %s.", name.c_str());
                break;
            
            default:
                RCLCPP_ERROR(node_->get_logger(), "Unknown result code.");
                break;
        }

        activity_id_ = nullptr;
        
        cmd->finished();
        goal_handle_.reset();
    };

    nav2_client_->async_send_goal(goal, goal_options);
}

void Robot::stop()
{
    if (goal_handle_)
    {
        std::lock_guard<std::mutex> lock(mute);
        nav2_client_->async_cancel_goal(goal_handle_);
        goal_handle_.reset();
    }
}

RobotState Robot::getState()
{
    std::lock_guard<std::mutex> lock(mute);
    return *robot_state_;
}

std::string Robot::getName()
{
    return name;
}

EasyRobotUpdateHandlePtr& Robot::getRobotHandlePtr()
{
    return robot_handle_;
}

bool Robot::startOdometrySubscriber()
{
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>("/odom",
        rclcpp::QoS(5), std::bind(&Robot::odometryCallback, this, std::placeholders::_1));
    
    std::unique_lock<std::mutex> lock(odom_cv_mute);
    bool success = odom_cv.wait_for(lock, std::chrono::seconds(5), [this]() -> bool
    {
        return odom_rcv;
    });

    if (!success)
    {
        RCLCPP_WARN(node_->get_logger(), "Timeout waiting for first odometry message.");
        return false;
    }

    return true;
}

void Robot::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    {
        std::lock_guard<std::mutex> lock(mute);

        tf2::Quaternion q_odom(msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);

        tf2::Vector3 p_odom(msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z);
        
        tf2::Transform t_rmf_robot = t_rmf_odom * tf2::Transform(q_odom, p_odom);

        tf2::Matrix3x3 m(t_rmf_robot.getRotation());

        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        robot_state_->set_position(Eigen::Vector3d({t_rmf_robot.getOrigin()[0],
                                                        t_rmf_robot.getOrigin()[1],
                                                        yaw}));
    }

    {
        std::lock_guard<std::mutex> lock(odom_cv_mute);
        if (!odom_rcv)
        {
            odom_rcv = true;
            odom_cv.notify_all();
        }
    }
}

void Robot::robotUpdateThread()
{
    while (update_running && rclcpp::ok())
    {
        std::lock_guard<std::mutex> lock(mute);
        
        if (robot_handle_)
            robot_handle_->update(*robot_state_, activity_id_);
        else
            RCLCPP_ERROR(node_->get_logger(), "Trying to use robot_handle_ while its null.");
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return;
}

void Robot::startRobotUpdate()
{
    if (!update_thread.joinable())
    {
        update_running = true;
        update_thread = std::thread(std::bind(&Robot::robotUpdateThread, this));
    }
}

Robot::~Robot()
{
    if (update_thread.joinable())
    {
        update_running = false;
        update_thread.join();
    }
}
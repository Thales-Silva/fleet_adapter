#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <memory>
#include <optional>
#include <string>
#include <mutex>
#include <condition_variable>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/exceptions.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <fleet_adapter/definitions.hpp>

namespace fleet_adapter
{
    class Robot : public std::enable_shared_from_this<Robot>
    {   
        private:
            rclcpp::Node::SharedPtr node_;
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
            rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav2_client_;

            bool odom_rcv;
            std::condition_variable odom_cv;
            std::mutex odom_cv_mute;
            
            tf2::Transform t_rmf_odom;
            tf2::Transform t_odom_rmf;

            std::thread update_thread;
            std::atomic<bool> update_running;
            std::mutex mute;

            std::string name;
            std::string charger;
            std::string map_name;

            ConstActivityIdentifierPtr activity_id_;
            EasyRobotUpdateHandlePtr robot_handle_;
            NavGoalHandle::SharedPtr goal_handle_;
            std::shared_ptr<RobotState> robot_state_;

            std::optional<Eigen::Vector3d> location;
            double battery_soc;
            std::optional<rmf_traffic::Duration> remaining_time;

        public:
            Robot();
            ~Robot();

            bool initialize(const std::string& ns,
                const std::string& robot_name,
                const std::string& charger_name,
                const std::string& initial_map_name,
                rclcpp::Node::SharedPtr n_);

            /** @brief This method recieves a series of points mapped into both, RMF and
            * odom coordinate systems. Supposing that there is an homogeneous matrix transform
            * between the two, calculates the right inverse to find the matrix T from odom to
            * rmf. Currently these points come from a yaml file... Tho, we might use aruco to
            * help us solving from rmf <-> map <-> aruco <-> odom <-> robot
            */
            void setOdom2RmfTransform(std::vector<std::vector<double>> rmf_points,
                std::vector<std::vector<double>> odom_points);
            
            /** @brief This method recieves a navigation call from the RMF, transforms it
            * from the RMF to the odom coordinate system (which is the same as map by now),
            * then sends it to the according navigation action server (nav2 for now), holding
            * the goal handle and the command identifier provided by the RMF.
            */
            void navigate(Destination desired_pose,
                CommandExecution cmd_exec);
            void stop();
            
            RobotState getState();
            std::string getName();
            EasyRobotUpdateHandlePtr& getRobotHandlePtr();
            
            /** @brief This method starts the odometry subscriber and waits for 5 seconds
            * for the first message arrives. Case it fails to receoeve messages, it returns
            * false.
            */
            bool startOdometrySubscriber();

            /** @brief This method listens to the /odom topic, gets the robot position,
            * transforms it to the RMF coordinate system then saves it into the robot_state_
            * object. The t_rmf_odom and t_odom_rmf are static transforms, by now calculated
            * from the set of points provided in the fleet_config.yaml.
            */
            void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

            /** @brief This method constantly updates RMF about the robot position
            * using this class's robot_state_ object. The odometry topic updates 
            */
            void robotUpdateThread();

            /** @brief This method starts the robotUpdateThread() 
            */
            void startRobotUpdate();
    };
}

#endif
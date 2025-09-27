#pragma once

#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <ae_hyu_msgs/msg/wpnt_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>

#include <Eigen/Dense>
#include <optional>
#include <atomic>
#include <thread>
#include <chrono>
#include <string>
#include <vector>

#include "crazy_controller/utils.hpp"

namespace crazy_controller {

class MAP_Controller; // Forward declaration

class Controller : public rclcpp::Node {
public:
    Controller();
    ~Controller() = default;

    // Public interface
    void wait_for_messages();
    void shutdown_handler();
    void publish_stop_command();

private:
    // Core functionality
    void control_loop();
    std::pair<double,double> map_cycle();

    // Initialization
    bool initialize();
    void init_map_controller();
    void declare_l1_dynamic_parameters_from_yaml(const std::string& yaml_path);

    // Callback functions
    void local_waypoint_cb(const ae_hyu_msgs::msg::WpntArray::SharedPtr msg);
    void car_state_cb(const nav_msgs::msg::Odometry::SharedPtr msg);
    void car_state_frenet_cb(const nav_msgs::msg::Odometry::SharedPtr msg);
    void on_parameter_event(const rcl_interfaces::msg::ParameterEvent & event);

    // Configuration parameters
    int rate_ = 40;
    std::string LUT_path_;
    std::string mode_;

    // ROS publishers and subscribers
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Subscription<ae_hyu_msgs::msg::WpntArray>::SharedPtr sub_local_waypoints_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_car_state_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_car_state_frenet_;

    // Timers
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr param_init_timer_;

    // Parameter handling
    std::unique_ptr<ParameterEventHandler> param_handler_;
    std::shared_ptr<rclcpp::ParameterEventCallbackHandle> callback_handle_;

    // State variables
    std::optional<double> speed_now_;
    std::optional<Eigen::RowVector3d> position_in_map_;
    std::optional<Eigen::Vector4d> position_in_map_frenet_;
    Eigen::MatrixXd waypoint_array_in_map_;
    Eigen::VectorXd acc_now_;

    // Safety and control
    int waypoint_safety_counter_ = 0;
    std::atomic<bool> shutdown_requested_{false};

    // MAP controller
    std::unique_ptr<crazy_controller::MAP_Controller> map_controller_;
};

} // namespace crazy_controller
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <ae_hyu_msgs/msg/wpnt_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <Eigen/Dense>
#include <optional>
#include <atomic>
#include <thread>
#include <chrono>
#include <string>
#include <vector>

#include "crazy_controller/utils.hpp"

namespace crazy_controller {

class PP_Controller; // Forward declaration

class PP_Controller_Node : public rclcpp::Node {
public:
    PP_Controller_Node();
    ~PP_Controller_Node() = default;

    // Public interface
    void wait_for_messages();
    void shutdown_handler();
    void publish_stop_command();

private:
    // Core functionality
    void control_loop();
    std::pair<double,double> pp_cycle();

    // Initialization
    bool initialize();
    void init_pp_controller();
    void declare_pp_dynamic_parameters_from_yaml(const std::string& yaml_path);

    // Callback functions
    void track_length_cb(const ae_hyu_msgs::msg::WpntArray::SharedPtr msg);
    void local_waypoint_cb(const ae_hyu_msgs::msg::WpntArray::SharedPtr msg);
    void car_state_cb(const nav_msgs::msg::Odometry::SharedPtr msg);
    void car_state_frenet_cb(const nav_msgs::msg::Odometry::SharedPtr msg);
    void on_parameter_event(const rcl_interfaces::msg::ParameterEvent & event);

    // Configuration parameters
    int rate_ = 40;
    std::string LUT_path_;
    std::string mode_;
    std::string map_frame_ = "map";
    std::string base_link_frame_ = "base_link";

    // ROS publishers and subscribers
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Subscription<ae_hyu_msgs::msg::WpntArray>::SharedPtr sub_track_length_;
    rclcpp::Subscription<ae_hyu_msgs::msg::WpntArray>::SharedPtr sub_local_waypoints_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_car_state_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_car_state_frenet_;

    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Timers
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr param_init_timer_;

    // Parameter handling
    std::unique_ptr<ParameterEventHandler> param_handler_;
    std::shared_ptr<rclcpp::ParameterEventCallbackHandle> callback_handle_;

    // State variables
    std::optional<double> track_length_;
    std::optional<double> speed_now_;
    std::optional<Eigen::RowVector3d> position_in_map_;
    std::optional<Eigen::Vector4d> position_in_map_frenet_;
    Eigen::MatrixXd waypoint_array_in_map_;
    Eigen::VectorXd acc_now_;

    // Safety and control
    int waypoint_safety_counter_ = 0;
    std::atomic<bool> shutdown_requested_{false};

    // PP controller implementation
    std::unique_ptr<crazy_controller::PP_Controller> pp_controller_;
};

} // namespace crazy_controller

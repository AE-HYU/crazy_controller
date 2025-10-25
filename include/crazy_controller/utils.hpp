#pragma once

#include <Eigen/Dense>
#include <string>
#include <functional>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>

// Forward declarations
namespace crazy_controller {

// Parameter Event Handler
class ParameterEventHandler {
public:
    ParameterEventHandler(std::shared_ptr<rclcpp::Node> node, const rclcpp::QoS& qos);
    ~ParameterEventHandler();

    std::shared_ptr<rclcpp::ParameterEventCallbackHandle> add_parameter_event_callback(
        std::function<void(const rcl_interfaces::msg::ParameterEvent&)> callback
    );

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
};

// Utility functions
namespace utils {

// Steering lookup utilities
class SteeringLookup {
public:
    SteeringLookup(const std::string& csv_path);
    ~SteeringLookup();
    double lookup_steer_angle(double accel, double vel) const;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
};

// Math utilities
template<typename T>
constexpr T clamp(const T& v, const T& lo, const T& hi) {
    return (v < lo) ? lo : (hi < v) ? hi : v;
}

// Geometry utilities
int nearest_waypoint(const Eigen::Vector2d& position, const Eigen::MatrixXd& waypoints);

} // namespace utils

// MAP Controller types and class
struct MAPResult {
    double speed;
    double acceleration;
    double jerk;
    double steering_angle;
    Eigen::Vector2d L1_point;
    double L1_distance;
    int idx_nearest_waypoint;
};

using Logger = std::function<void(const std::string&)>;

class MAP_Controller {
public:
    MAP_Controller(
        double t_clip_min,
        double t_clip_max,
        double m_l1,
        double q_l1,
        double speed_lookahead,
        double lat_err_coeff,
        double acc_scaler_for_steer,
        double dec_scaler_for_steer,
        double start_scale_speed,
        double end_scale_speed,
        double downscale_factor,
        double speed_lookahead_for_steer,
        double loop_rate,
        const std::string& LUT_path,
        Logger logger_info,
        Logger logger_warn);

    MAPResult main_loop(
        const Eigen::RowVector3d& position_in_map,
        const Eigen::MatrixXd& waypoint_array_in_map,
        double speed_now,
        const Eigen::Vector2d& position_in_map_frenet,
        const Eigen::VectorXd& acc_now,
        double track_length);

    // Parameter setters (inline implementations)
    void set_t_clip_min(double v) { t_clip_min_ = v; }
    void set_t_clip_max(double v) { t_clip_max_ = v; }
    void set_m_l1(double v) { m_l1_ = v; }
    void set_q_l1(double v) { q_l1_ = v; }
    void set_speed_lookahead(double v) { speed_lookahead_ = v; }
    void set_lat_err_coeff(double v) { lat_err_coeff_ = v; }
    void set_acc_scaler_for_steer(double v) { acc_scaler_for_steer_ = v; }
    void set_dec_scaler_for_steer(double v) { dec_scaler_for_steer_ = v; }
    void set_start_scale_speed(double v) { start_scale_speed_ = v; }
    void set_end_scale_speed(double v) { end_scale_speed_ = v; }
    void set_downscale_factor(double v) { downscale_factor_ = v; }
    void set_speed_lookahead_for_steer(double v) { speed_lookahead_for_steer_ = v; }

private:
    // Implementation methods
    double calc_steering_angle(const Eigen::Vector2d& L1_point,
                               double L1_distance,
                               double yaw,
                               double lat_e_norm,
                               const Eigen::Vector2d& v);
    std::pair<Eigen::Vector2d, double> calc_L1_point(double lateral_error);
    std::optional<double> calc_speed_command(const Eigen::Vector2d& v, double lat_e_norm);
    std::pair<double, double> calc_lateral_error_norm() const;
    double speed_adjust_lat_err(double global_speed, double lat_e_norm) const;
    double distance(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2);
    double acc_scaling(double steer) const;
    double speed_steer_scaling(double steer, double speed) const;
    int nearest_waypoint(const Eigen::Vector2d& position, const Eigen::MatrixXd& waypoints_xy) const;
    Eigen::Vector2d waypoint_at_distance_before_car(double distance,
                                                    const Eigen::MatrixXd& waypoints_xy,
                                                    int idx_waypoint_behind_car) const;

    // Parameters
    double t_clip_min_;
    double t_clip_max_;
    double m_l1_;
    double q_l1_;
    double speed_lookahead_;
    double lat_err_coeff_;
    double acc_scaler_for_steer_;
    double dec_scaler_for_steer_;
    double start_scale_speed_;
    double end_scale_speed_;
    double downscale_factor_;
    double speed_lookahead_for_steer_;
    double loop_rate_;
    std::string LUT_path_;

    // State variables
    Eigen::RowVector3d position_in_map_;
    Eigen::MatrixXd waypoint_array_in_map_;
    double speed_now_;
    Eigen::Vector2d position_in_map_frenet_;
    Eigen::VectorXd acc_now_;
    double track_length_;
    std::optional<double> speed_command_;
    std::optional<int> idx_nearest_waypoint_;
    double curvature_waypoints_;

    // Components (order matters for initialization)
    Logger logger_info_;
    Logger logger_warn_;
    utils::SteeringLookup steer_lookup_;
    double curr_steering_angle_;
};

// Pure Pursuit Controller types and class
struct PPResult {
    double speed;
    double acceleration;
    double jerk;
    double steering_angle;
    Eigen::Vector2d L1_point;
    double L1_distance;
    int idx_nearest_waypoint;
};

class PP_Controller {
public:
    PP_Controller(
        double t_clip_min,
        double t_clip_max,
        double m_l1,
        double q_l1,
        double speed_lookahead,
        double lat_err_coeff,
        double acc_scaler_for_steer,
        double dec_scaler_for_steer,
        double start_scale_speed,
        double end_scale_speed,
        double downscale_factor,
        double speed_lookahead_for_steer,
        double loop_rate,
        const std::string& LUT_path,
        Logger logger_info,
        Logger logger_warn);

    PPResult main_loop(
        const Eigen::RowVector3d& position_in_map,
        const Eigen::MatrixXd& waypoint_array_in_map,
        double speed_now,
        const Eigen::Vector2d& position_in_map_frenet,
        const Eigen::VectorXd& acc_now,
        double track_length);

    // Parameter setters (inline implementations)
    void set_t_clip_min(double v) { t_clip_min_ = v; }
    void set_t_clip_max(double v) { t_clip_max_ = v; }
    void set_m_l1(double v) { m_l1_ = v; }
    void set_q_l1(double v) { q_l1_ = v; }
    void set_speed_lookahead(double v) { speed_lookahead_ = v; }
    void set_lat_err_coeff(double v) { lat_err_coeff_ = v; }
    void set_acc_scaler_for_steer(double v) { acc_scaler_for_steer_ = v; }
    void set_dec_scaler_for_steer(double v) { dec_scaler_for_steer_ = v; }
    void set_start_scale_speed(double v) { start_scale_speed_ = v; }
    void set_end_scale_speed(double v) { end_scale_speed_ = v; }
    void set_downscale_factor(double v) { downscale_factor_ = v; }
    void set_speed_lookahead_for_steer(double v) { speed_lookahead_for_steer_ = v; }

private:
    // Implementation methods
    double calc_steering_angle(const Eigen::Vector2d& L1_point,
                               double L1_distance,
                               double yaw,
                               double lat_e_norm,
                               const Eigen::Vector2d& v);
    std::pair<Eigen::Vector2d, double> calc_L1_point(double lateral_error);
    std::optional<double> calc_speed_command(const Eigen::Vector2d& v, double lat_e_norm);
    std::pair<double, double> calc_lateral_error_norm() const;
    double speed_adjust_lat_err(double global_speed, double lat_e_norm) const;
    double distance(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2);
    double acc_scaling(double steer) const;
    double speed_steer_scaling(double steer, double speed) const;
    int nearest_waypoint(const Eigen::Vector2d& position, const Eigen::MatrixXd& waypoints_xy) const;
    Eigen::Vector2d waypoint_at_distance_before_car(double distance,
                                                    const Eigen::MatrixXd& waypoints_xy,
                                                    int idx_waypoint_behind_car) const;

    // Parameters
    double t_clip_min_;
    double t_clip_max_;
    double m_l1_;
    double q_l1_;
    double speed_lookahead_;
    double lat_err_coeff_;
    double acc_scaler_for_steer_;
    double dec_scaler_for_steer_;
    double start_scale_speed_;
    double end_scale_speed_;
    double downscale_factor_;
    double speed_lookahead_for_steer_;
    double loop_rate_;
    std::string LUT_path_;

    // State variables
    Eigen::RowVector3d position_in_map_;
    Eigen::MatrixXd waypoint_array_in_map_;
    double speed_now_;
    Eigen::Vector2d position_in_map_frenet_;
    Eigen::VectorXd acc_now_;
    double track_length_;
    std::optional<double> speed_command_;
    std::optional<int> idx_nearest_waypoint_;
    double curvature_waypoints_;

    // Components (order matters for initialization)
    Logger logger_info_;
    Logger logger_warn_;
    double curr_steering_angle_;
    // Note: LUT_path_ is kept for compatibility but steering lookup is not used in Pure Pursuit
};

} // namespace crazy_controller
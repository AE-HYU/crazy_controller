#include "crazy_controller/utils.hpp"

#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <limits>
#include <cmath>
#include <algorithm>
#include <cctype>
#include <utility>

namespace crazy_controller {

// AUG Controller implementation
using Logger = std::function<void(const std::string&)>;

AUG_Controller::AUG_Controller(
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
    double diff_threshold,
    double deacc_gain,
    const std::string& LUT_path,
    double Cf,
    double Cr,
    double L,
    double lf,
    double lr,
    double m,
    double kf,
    double kr,
    Logger logger_info,
    Logger logger_warn)
  : t_clip_min_(t_clip_min),
    t_clip_max_(t_clip_max),
    m_l1_(m_l1),
    q_l1_(q_l1),
    speed_lookahead_(speed_lookahead),
    lat_err_coeff_(lat_err_coeff),
    acc_scaler_for_steer_(acc_scaler_for_steer),
    dec_scaler_for_steer_(dec_scaler_for_steer),
    start_scale_speed_(start_scale_speed),
    end_scale_speed_(end_scale_speed),
    downscale_factor_(downscale_factor),
    speed_lookahead_for_steer_(speed_lookahead_for_steer),
    diff_threshold_(diff_threshold),
    deacc_gain_(deacc_gain),
    LUT_path_(LUT_path),
    Cf_(Cf),
    Cr_(Cr),
    L_(L),
    lf_(lf),
    lr_(lr),
    m_(m),
    kf_(kf),
    kr_(kr),
    acc_now_(10),
    curvature_waypoints_(0.0),
    logger_info_(std::move(logger_info)),
    logger_warn_(std::move(logger_warn)),
    curr_steering_angle_(0.0),
    first_steering_calculation_(true)
{
  acc_now_.setZero();
  // Note: AUG does not use steering lookup table
}

AUGResult AUG_Controller::main_loop(
    const Eigen::RowVector3d& position_in_map,
    const Eigen::MatrixXd& waypoint_array_in_map,
    double speed_now,
    const Eigen::Vector2d& position_in_map_frenet,
    const Eigen::VectorXd& acc_now)
{
  position_in_map_ = position_in_map;
  waypoint_array_in_map_ = waypoint_array_in_map;
  speed_now_ = speed_now;
  position_in_map_frenet_ = position_in_map_frenet;
  acc_now_ = acc_now;

  const double yaw = position_in_map_(2);
  Eigen::Vector2d v(std::cos(yaw) * speed_now_, std::sin(yaw) * speed_now_);

  auto [lat_e_norm, lateral_error] = calc_lateral_error_norm();

  speed_command_ = calc_speed_command(v, lat_e_norm);

  double speed = 0.0, acceleration = 0.0, jerk = 0.0;
  if (speed_command_.has_value()) {
    speed = std::max(0.0, speed_command_.value());
  } else {
    speed = 0.0;
    acceleration = 0.0;
    jerk = 0.0;
    if (logger_warn_) logger_warn_("[AUG Controller] speed was none");
  }

  auto [L1_point, L1_distance] = calc_L1_point(lateral_error);

  if (!std::isfinite(L1_point.x()) || !std::isfinite(L1_point.y())) {
    throw std::runtime_error("L1_point is invalid");
  }

  // Startup blending: commented out to match controller/aug.py
  if (idx_nearest_waypoint_.has_value()) {
    const int nearest_idx = static_cast<int>(idx_nearest_waypoint_.value());
    if (nearest_idx >= 0 && nearest_idx < waypoint_array_in_map_.rows()) {
      const double profile_speed = waypoint_array_in_map_(nearest_idx, 2);
      const double diff = std::abs(profile_speed - speed_now_);
      if (diff >= diff_threshold_) { // configurable threshold
        const double prev_speed = speed;
        speed = deacc_gain_ * (speed + speed_now_); // configurable blending gain
        if (logger_info_) logger_info_(
          std::string("[AUG Controller] Startup blend active: |profile - v| = ") +
          std::to_string(diff) + " m/s (threshold=" + std::to_string(diff_threshold_) + ")"
          ", gain=" + std::to_string(deacc_gain_) + ", speed " + std::to_string(prev_speed) +
          " -> " + std::to_string(speed));
      }
    }
  }

  double steering_angle = calc_steering_angle(L1_point, L1_distance, yaw, lat_e_norm, v);

  AUGResult out;
  out.speed = speed;
  out.acceleration = acceleration;
  out.jerk = jerk;
  out.steering_angle = steering_angle;
  out.L1_point = L1_point;
  out.L1_distance = L1_distance;
  out.idx_nearest_waypoint = idx_nearest_waypoint_.value_or(-1);
  return out;
}

double AUG_Controller::calc_steering_angle(const Eigen::Vector2d& L1_point,
                                           double L1_distance,
                                           double yaw,
                                           double lat_e_norm,
                                           const Eigen::Vector2d& v)
{
  // Calculate lookahead position for speed adjustment
  const double adv_ts_st = speed_lookahead_for_steer_;
  Eigen::Vector2d la_position(
      position_in_map_(0) + v(0) * adv_ts_st,
      position_in_map_(1) + v(1) * adv_ts_st);

  const int idx_la_steer =
      nearest_waypoint(la_position, waypoint_array_in_map_.leftCols<2>());

  const double speed_la_for_lu = waypoint_array_in_map_(idx_la_steer, 2);
  // const double speed_for_lu = speed_adjust_lat_err(speed_la_for_lu, lat_e_norm);

  // Calculate L1 vector
  Eigen::Vector2d L1_vector(L1_point.x() - position_in_map_(0),
                            L1_point.y() - position_in_map_(1));

  // Calculate eta (angle between vehicle heading and L1 vector)
  double eta = 0.0;
  if (L1_vector.norm() == 0.0) {
    if (logger_warn_) logger_warn_("[AUG Controller] norm of L1 vector was 0, eta is set to 0");
    eta = 0.0;
  } else {
    Eigen::Vector2d nvec(-std::sin(yaw), std::cos(yaw));
    eta = std::asin(nvec.dot(L1_vector) / L1_vector.norm());
  }

  // AUG steering angle formula
  double steering_angle = 0.0;
  if (L1_distance == 0.0) {
    if (logger_warn_) logger_warn_("[AUG Controller] L1_distance is 0, steering_angle is set to 0");
    steering_angle = 0.0;
  } else if (speed_now_ < 0.1) {
    if (logger_warn_) logger_warn_("[AUG Controller] speed_now < 0.1 , steering_angle is set to 0");
    steering_angle = 0.0;
  } else {
    const double lat_acc = 2.0 * (speed_now_ * speed_now_) * std::sin(eta) / L1_distance;

    // Calculate tire cornering stiffness with adjustment
    const double Cf = Cf_ / (1.0 + kf_ * std::pow(std::abs(lat_acc), 3));
    const double Cr = Cr_ / (1.0 + kr_ * std::pow(std::abs(lat_acc), 3));

    const double K_us = m_ * ((Cr * lr_ - Cf * lf_) / (Cf * Cr * L_));
    steering_angle = lat_acc * (L_ / (speed_now_ * speed_now_) + K_us);
  }

  // Apply speed-based downscaling (commented out to match controller/aug.py)
  // steering_angle = speed_steer_scaling(steering_angle, speed_now_);

  // Apply acceleration-based scaling (commented out to match controller/aug.py)
  // steering_angle = acc_scaling(steering_angle);

  // Apply speed multiplier (commented out to match controller/aug.py)
  // steering_angle *= utils::clamp(1.0 + (speed_now_ / 10.0), 1.0, 1.25);

  // Apply rate limiting (0.4 rad/step) - skip on first calculation
  const double threshold = 0.4;
  if (first_steering_calculation_) {
    first_steering_calculation_ = false;
    if (logger_info_) logger_info_("[AUG Controller] First steering calculation, skipping rate limiting");
  } else if (std::abs(steering_angle - curr_steering_angle_) > threshold) {
    if (logger_info_) {
      double clamped_angle = utils::clamp(steering_angle, curr_steering_angle_ - threshold, curr_steering_angle_ + threshold);
      logger_info_("[AUG Controller] steering angle clipped: " + std::to_string(steering_angle) +
                   " -> " + std::to_string(clamped_angle));
    }
    steering_angle = utils::clamp(steering_angle,
                           curr_steering_angle_ - threshold,
                           curr_steering_angle_ + threshold);
  }

  // Apply hard limits
  const double max_steering_angle = 0.45;
  steering_angle = utils::clamp(steering_angle, -max_steering_angle, max_steering_angle);

  curr_steering_angle_ = steering_angle;
  return steering_angle;
}

std::pair<Eigen::Vector2d, double> AUG_Controller::calc_L1_point(double lateral_error) {
  // Find nearest waypoint
  idx_nearest_waypoint_ =
      nearest_waypoint(position_in_map_.head<2>(), waypoint_array_in_map_.leftCols<2>());

  if (!idx_nearest_waypoint_.has_value()) {
    idx_nearest_waypoint_ = 0;
  }

  // Calculate mean curvature from nearest waypoint forward (동일: MAP과 완전히 일치)
  if ((waypoint_array_in_map_.rows() - static_cast<Eigen::Index>(idx_nearest_waypoint_.value())) > 2) {
    const int lookahead_idx = static_cast<int>(std::floor(speed_now_ * speed_lookahead_ * 1.0 * 10.0));
    const Eigen::Index end_idx = std::min(
      static_cast<Eigen::Index>(idx_nearest_waypoint_.value()) + static_cast<Eigen::Index>(lookahead_idx),
      waypoint_array_in_map_.rows()
    );

    const Eigen::Index num_rows = end_idx - static_cast<Eigen::Index>(idx_nearest_waypoint_.value());
    if (num_rows > 0) {
      curvature_waypoints_ = waypoint_array_in_map_
        .block(static_cast<Eigen::Index>(idx_nearest_waypoint_.value()), 5, num_rows, 1)
        .cwiseAbs()
        .mean();
    }
  }

  // Calculate adaptive L1 distance
  double L1_distance = q_l1_ + speed_now_ * m_l1_;

  // Apply lateral error-based lower bound (commented out to match controller/aug.py)
  // const double lateral_multiplier = (lateral_error > 1.0) ? 2.0 : std::sqrt(2.0);
  // const double lower_bound = std::max(t_clip_min_, lateral_multiplier * lateral_error);
  // L1_distance = utils::clamp(L1_distance, lower_bound, t_clip_max_);

  // if (logger_info_ && lateral_error > 1.0) {
  //   logger_info_("[AUG Controller] Large lateral error: " + std::to_string(lateral_error) +
  //                "m, L1_distance: " + std::to_string(L1_distance) + "m");
  // }

  // Get waypoint at L1 distance ahead
  Eigen::Vector2d L1_point =
      waypoint_at_distance_before_car(L1_distance,
                                      waypoint_array_in_map_.leftCols<2>(),
                                      idx_nearest_waypoint_.value());
  return {L1_point, L1_distance};
}

std::optional<double> AUG_Controller::calc_speed_command(const Eigen::Vector2d& v,
                                                         double lat_e_norm)
{
  // Calculate lookahead position
  const double adv_ts_sp = speed_lookahead_;
  Eigen::Vector2d la_position(
      position_in_map_(0) + v(0) * adv_ts_sp,
      position_in_map_(1) + v(1) * adv_ts_sp);

  // Find nearest waypoint to lookahead position
  const int idx_la_position =
      nearest_waypoint(la_position, waypoint_array_in_map_.leftCols<2>());

  // Get global speed from waypoint
  double global_speed = waypoint_array_in_map_(idx_la_position, 2);

  // Adjust speed based on lateral error (commented out to match controller/aug.py)
  // global_speed = speed_adjust_lat_err(global_speed, lat_e_norm);

  return global_speed;
}

double AUG_Controller::distance(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) {
  return (p2 - p1).norm();
}

// Acceleration scaling (commented out to match controller/aug.py)
// double AUG_Controller::acc_scaling(double steer) const {
//   const double mean_acc = (acc_now_.size() > 0) ? acc_now_.mean() : 0.0;
//   if (mean_acc >= 0.8) {
//     return steer * acc_scaler_for_steer_;
//   } else if (mean_acc <= -0.8) {
//     return steer * dec_scaler_for_steer_;
//   }
//   return steer;
// }

// Speed-based steering scaling (commented out to match controller/aug.py)
// double AUG_Controller::speed_steer_scaling(double steer, double speed) const {
//   const double speed_diff = std::max(0.1, end_scale_speed_ - start_scale_speed_);
//   const double factor = 1.0 - utils::clamp((speed - start_scale_speed_) / speed_diff, 0.0, 1.0) * downscale_factor_;
//   return steer * factor;
// }

std::pair<double,double> AUG_Controller::calc_lateral_error_norm() const {
  const double lateral_error = std::abs(position_in_map_frenet_(1));

  // Clip to [0, 2] meters and normalize to [0, 0.5]
  const double max_lat_e = 2.0;
  const double min_lat_e = 0.0;
  const double lat_e_clip = utils::clamp(lateral_error, min_lat_e, max_lat_e);
  const double lat_e_norm = 0.5 * ((lat_e_clip - min_lat_e) / (max_lat_e - min_lat_e));
  return {lat_e_norm, lateral_error};
}

// Speed adjustment based on lateral error (commented out to match controller/aug.py)
// double AUG_Controller::speed_adjust_lat_err(double global_speed, double lat_e_norm) const {
//   double lat_e_coeff = lat_err_coeff_;
//   lat_e_norm *= 2.0;
//
//   const double curv = utils::clamp(2.0 * ( (curvature_waypoints_) / 0.8 ) - 2.0, 0.0, 1.0);
//   global_speed *= (1.0 - lat_e_coeff + lat_e_coeff * std::exp(-lat_e_norm * curv));
//   return global_speed;
// }

int AUG_Controller::nearest_waypoint(const Eigen::Vector2d& position,
                                     const Eigen::MatrixXd& waypoints_xy) const
{
  const Eigen::Index N = waypoints_xy.rows();
  if (N <= 0) return 0;
  Eigen::Vector2d pos = position;
  double best = std::numeric_limits<double>::max();
  int best_idx = 0;
  for (Eigen::Index i = 0; i < N; ++i) {
    double d = (pos - waypoints_xy.row(i).transpose()).norm();
    if (d < best) {
      best = d;
      best_idx = static_cast<int>(i);
    }
  }
  return best_idx;
}

Eigen::Vector2d AUG_Controller::waypoint_at_distance_before_car(double distance,
                                                                const Eigen::MatrixXd& waypoints_xy,
                                                                int idx_waypoint_behind_car) const
{
  if (!std::isfinite(distance)) distance = t_clip_min_;
  double d_distance = distance;
  const double waypoints_distance = 0.1;
  int d_index = static_cast<int>(d_distance / waypoints_distance + 0.5);

  const int idx = std::min<int>(static_cast<int>(waypoints_xy.rows()) - 1,
                                idx_waypoint_behind_car + d_index);
  return waypoints_xy.row(idx).transpose();
}

// Dummy implementations for commented-out methods to satisfy header
double AUG_Controller::acc_scaling(double steer) const {
  return steer;  // No-op when commented out
}

double AUG_Controller::speed_steer_scaling(double steer, double speed) const {
  return steer;  // No-op when commented out
}

double AUG_Controller::speed_adjust_lat_err(double global_speed, double lat_e_norm) const {
  return global_speed;  // No-op when commented out
}

} // namespace crazy_controller

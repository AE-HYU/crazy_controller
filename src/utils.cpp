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

// Parameter Event Handler implementation
struct ParameterEventHandler::Impl {
    std::shared_ptr<rclcpp::ParameterEventHandler> handler_;

    Impl(std::shared_ptr<rclcpp::Node> node, const rclcpp::QoS& qos) {
        handler_ = std::make_shared<rclcpp::ParameterEventHandler>(node, qos);
    }
};

ParameterEventHandler::ParameterEventHandler(std::shared_ptr<rclcpp::Node> node, const rclcpp::QoS& qos)
    : pimpl_(std::make_unique<Impl>(node, qos)) {
}

ParameterEventHandler::~ParameterEventHandler() = default;

std::shared_ptr<rclcpp::ParameterEventCallbackHandle> ParameterEventHandler::add_parameter_event_callback(
    std::function<void(const rcl_interfaces::msg::ParameterEvent&)> callback) {
    return pimpl_->handler_->add_parameter_event_callback(callback);
}

// Steering lookup implementation - completely self-contained
namespace utils {

namespace steering_lookup {

static inline bool is_nan(double x) {
    return std::isnan(x);
}

struct TwoNeighbors {
    double closest;
    std::size_t closest_idx;
    double second_closest;
    std::size_t second_idx;
};

std::pair<double, std::size_t> find_nearest(const std::vector<double>& array, double value) {
    if (array.empty()) {
        throw std::runtime_error("find_nearest: array is empty");
    }
    double best_val = array[0];
    std::size_t best_idx = 0;
    double best_dist = std::abs(array[0] - value);

    for (std::size_t i = 1; i < array.size(); ++i) {
        double d = std::abs(array[i] - value);
        if (d < best_dist) {
            best_dist = d;
            best_idx = i;
            best_val = array[i];
        }
    }
    return {best_val, best_idx};
}

TwoNeighbors find_closest_neighbors(const std::vector<double>& array_in, double value) {
    if (array_in.empty()) {
        throw std::runtime_error("find_closest_neighbors: array is empty");
    }

    std::vector<double> array;
    array.reserve(array_in.size());
    for (double v : array_in) {
        if (is_nan(v)) break;
        array.push_back(v);
    }
    if (array.empty()) {
        throw std::runtime_error("find_closest_neighbors: array has only NaNs");
    }

    auto [closest, closest_idx] = find_nearest(array, value);

    if (closest_idx == 0) {
        // Lower boundary: clamp to minimum value
        return {array[0], 0, array[0], 0};
    } else if (closest_idx == array.size() - 1) {
        // Upper boundary: use last two points for extrapolation
        std::size_t i = array.size() - 1;
        if (i >= 1) {
            return {array[i], i, array[i-1], i-1};
        } else {
            return {array[i], i, array[i], i};
        }
    } else {
        std::size_t left  = closest_idx - 1;
        std::size_t right = closest_idx + 1;

        double dl = std::abs(array[left]  - value);
        double dr = std::abs(array[right] - value);

        std::size_t second_idx = (dr < dl) ? right : left;
        double second_closest = array[second_idx];

        return {closest, closest_idx, second_closest, second_idx};
    }
}

class LookupSteerAngleImpl {
public:
    using Logger = std::function<void(const std::string&)>;

    LookupSteerAngleImpl(const std::string& lookup_table_path, Logger logger = nullptr)
        : lu_(load_csv_matrix(lookup_table_path)), logger_(std::move(logger)) {
    }

    double lookup_steer_angle(double accel, double vel) const {
        const double sign_accel = (accel > 0.0) ? 1.0 : -1.0;
        accel = std::abs(accel);

        if (lu_.rows() < 2 || lu_.cols() < 2) {
            throw std::runtime_error("lookup table size invalid");
        }

        std::vector<double> lu_vs;
        lu_vs.reserve(static_cast<std::size_t>(lu_.cols() - 1));
        for (Eigen::Index j = 1; j < lu_.cols(); ++j) {
            lu_vs.push_back(lu_(0, j));
        }

        std::vector<double> lu_steers;
        lu_steers.reserve(static_cast<std::size_t>(lu_.rows() - 1));
        for (Eigen::Index i = 1; i < lu_.rows(); ++i) {
            lu_steers.push_back(lu_(i, 0));
        }

        auto nearest = find_nearest(lu_vs, vel);
        std::size_t c_v_idx = nearest.second;

        std::vector<double> accel_col;
        accel_col.reserve(static_cast<std::size_t>(lu_.rows() - 1));
        for (Eigen::Index i = 1; i < lu_.rows(); ++i) {
            accel_col.push_back(lu_(i, static_cast<Eigen::Index>(c_v_idx + 1)));
        }

        auto neigh = find_closest_neighbors(accel_col, accel);

        double steer_angle;
        if (neigh.closest_idx == neigh.second_idx) {
            steer_angle = lu_steers[neigh.closest_idx];
        } else {
            const double x0 = neigh.closest;
            const double x1 = neigh.second_closest;
            const double y0 = lu_steers[neigh.closest_idx];
            const double y1 = lu_steers[neigh.second_idx];

            if (std::abs(x1 - x0) < 1e-12) {
                steer_angle = y0;
            } else {
                const double t = (accel - x0) / (x1 - x0);
                steer_angle = y0 + (y1 - y0) * t;
            }
        }

        return steer_angle * sign_accel;
    }

private:
    Eigen::MatrixXd load_csv_matrix(const std::string& path) {
        std::ifstream fin(path);
        if (!fin.is_open()) {
            throw std::runtime_error("Failed to open CSV: " + path);
        }

        std::vector<std::vector<double>> rows;
        std::string line;
        std::size_t max_cols = 0;

        while (std::getline(fin, line)) {
            std::vector<double> cols;
            std::stringstream ss(line);
            std::string cell;
            while (std::getline(ss, cell, ',')) {
                if (!cell.empty() && (cell.front() == ' ' || cell.front() == '\t')) {
                    cell.erase(cell.begin(), std::find_if(cell.begin(), cell.end(),
                              [](unsigned char ch){ return !std::isspace(ch); }));
                }
                if (!cell.empty() && (cell.back() == ' ' || cell.back() == '\t')) {
                    cell.erase(std::find_if(cell.rbegin(), cell.rend(),
                              [](unsigned char ch){ return !std::isspace(ch); }).base(),
                              cell.end());
                }
                if (cell.empty()) {
                    cols.push_back(std::numeric_limits<double>::quiet_NaN());
                } else {
                    try {
                        cols.push_back(std::stod(cell));
                    } catch (...) {
                        cols.push_back(std::numeric_limits<double>::quiet_NaN());
                    }
                }
            }
            max_cols = std::max(max_cols, cols.size());
            rows.emplace_back(std::move(cols));
        }

        const std::size_t r = rows.size();
        const std::size_t c = max_cols;
        Eigen::MatrixXd M(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c));
        for (std::size_t i = 0; i < r; ++i) {
            rows[i].resize(c, std::numeric_limits<double>::quiet_NaN());
            for (std::size_t j = 0; j < c; ++j) {
                M(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j)) = rows[i][j];
            }
        }
        return M;
    }

    Eigen::MatrixXd lu_;
    Logger logger_;
};

} // namespace steering_lookup

// SteeringLookup wrapper implementation
struct SteeringLookup::Impl {
    std::unique_ptr<steering_lookup::LookupSteerAngleImpl> lookup_;

    Impl(const std::string& csv_path) {
        lookup_ = std::make_unique<steering_lookup::LookupSteerAngleImpl>(csv_path, nullptr);
    }
};

SteeringLookup::SteeringLookup(const std::string& csv_path)
    : pimpl_(std::make_unique<Impl>(csv_path)) {
}

SteeringLookup::~SteeringLookup() = default;

double SteeringLookup::lookup_steer_angle(double accel, double vel) const {
    return pimpl_->lookup_->lookup_steer_angle(accel, vel);
}

// Utility functions
int nearest_waypoint(const Eigen::Vector2d& position, const Eigen::MatrixXd& waypoints) {
    if (waypoints.rows() == 0) return -1;

    double min_dist = std::numeric_limits<double>::max();
    int nearest_idx = 0;

    for (int i = 0; i < waypoints.rows(); ++i) {
        double dx = waypoints(i, 0) - position(0);
        double dy = waypoints(i, 1) - position(1);
        double dist = dx * dx + dy * dy;

        if (dist < min_dist) {
            min_dist = dist;
            nearest_idx = i;
        }
    }

    return nearest_idx;
}

} // namespace utils

// MAP Controller implementation
using Logger = std::function<void(const std::string&)>;

MAP_Controller::MAP_Controller(
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
    const std::string& LUT_path,
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
    LUT_path_(LUT_path),
    acc_now_(10),
    curvature_waypoints_(0.0),
    logger_info_(std::move(logger_info)),
    logger_warn_(std::move(logger_warn)),
    steer_lookup_(LUT_path_),
    curr_steering_angle_(0.0)
{
  acc_now_.setZero();
}

MAPResult MAP_Controller::main_loop(
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
    if (logger_warn_) logger_warn_("[Controller] speed was none");
  }

  auto [L1_point, L1_distance] = calc_L1_point(lateral_error);

  if (!std::isfinite(L1_point.x()) || !std::isfinite(L1_point.y())) {
    throw std::runtime_error("L1_point is invalid");
  }

  double steering_angle = calc_steering_angle(L1_point, L1_distance, yaw, lat_e_norm, v);

  MAPResult out;
  out.speed = speed;
  out.acceleration = acceleration;
  out.jerk = jerk;
  out.steering_angle = steering_angle;
  out.L1_point = L1_point;
  out.L1_distance = L1_distance;
  out.idx_nearest_waypoint = idx_nearest_waypoint_.value_or(-1);
  return out;
}

double MAP_Controller::calc_steering_angle(const Eigen::Vector2d& L1_point,
                                           double L1_distance,
                                           double yaw,
                                           double lat_e_norm,
                                           const Eigen::Vector2d& v)
{
  const double adv_ts_st = speed_lookahead_for_steer_;
  Eigen::Vector2d la_position(
      position_in_map_(0) + v(0) * adv_ts_st,
      position_in_map_(1) + v(1) * adv_ts_st);

  const int idx_la_steer =
      nearest_waypoint(la_position, waypoint_array_in_map_.leftCols<2>());

  const double speed_la_for_lu = waypoint_array_in_map_(idx_la_steer, 2);
  const double speed_for_lu = speed_adjust_lat_err(speed_la_for_lu, lat_e_norm);

  Eigen::Vector2d L1_vector(L1_point.x() - position_in_map_(0),
                            L1_point.y() - position_in_map_(1));
  double eta = 0.0;
  if (L1_vector.norm() == 0.0) {
    if (logger_warn_) logger_warn_("[Controller] norm of L1 vector was 0, eta is set to 0");
    eta = 0.0;
  } else {
    Eigen::Vector2d nvec(-std::sin(yaw), std::cos(yaw));
    eta = std::asin(nvec.dot(L1_vector) / L1_vector.norm());
  }

  double lat_acc = 0.0;
  if (L1_distance == 0.0 || std::sin(eta) == 0.0) {
    if (logger_warn_) logger_warn_("[Controller] L1 * np.sin(eta), lat_acc is set to 0");
    lat_acc = 0.0;
  } else {
    lat_acc = 2.0 * speed_for_lu * speed_for_lu / L1_distance * std::sin(eta);
  }

  double steering_angle = steer_lookup_.lookup_steer_angle(lat_acc, speed_for_lu);

  steering_angle = speed_steer_scaling(steering_angle, speed_for_lu);

  steering_angle = acc_scaling(steering_angle);

  steering_angle *= utils::clamp(1.0 + (speed_now_ / 10.0), 1.0, 1.25);

  const double threshold = 0.4;

  // Allow larger initial steering angle change to avoid startup clipping
  static bool first_steering_calculation = true;
  if (first_steering_calculation) {
    first_steering_calculation = false;
    if (logger_info_) logger_info_("[MAP Controller] First steering calculation, skipping rate limiting");
  } else if (std::abs(steering_angle - curr_steering_angle_) > threshold) {
    if (logger_info_) {
      double clamped_angle = utils::clamp(steering_angle, curr_steering_angle_ - threshold, curr_steering_angle_ + threshold);
      logger_info_("[MAP Controller] steering angle clipped: " + std::to_string(steering_angle) +
                   " -> " + std::to_string(clamped_angle));
    }
    steering_angle = utils::clamp(steering_angle,
                           curr_steering_angle_ - threshold,
                           curr_steering_angle_ + threshold);
  }

  const double max_steering_angle = 0.45;
  steering_angle = utils::clamp(steering_angle, -max_steering_angle, max_steering_angle);

  curr_steering_angle_ = steering_angle;
  return steering_angle;
}

std::pair<Eigen::Vector2d, double> MAP_Controller::calc_L1_point(double lateral_error) {
  idx_nearest_waypoint_ =
      nearest_waypoint(position_in_map_.head<2>(), waypoint_array_in_map_.leftCols<2>());

  if (!idx_nearest_waypoint_.has_value()) {
    idx_nearest_waypoint_ = 0;
  }

  if ((waypoint_array_in_map_.rows() - idx_nearest_waypoint_.value()) > 2) {
    curvature_waypoints_ =
        (waypoint_array_in_map_.block(idx_nearest_waypoint_.value(), 5,
                                      waypoint_array_in_map_.rows() - idx_nearest_waypoint_.value(), 1)
         .cwiseAbs().mean());
  }

  double L1_distance = q_l1_ + speed_now_ * m_l1_;

  double gain = 1.0 - 0.25 * curvature_waypoints_;
  L1_distance *= gain;

  // For large lateral errors, increase L1 distance more aggressively to improve stability
  const double lateral_multiplier = (lateral_error > 1.0) ? 2.0 : std::sqrt(2.0);
  const double lower_bound = std::max(t_clip_min_, lateral_multiplier * lateral_error);
  L1_distance = utils::clamp(L1_distance, lower_bound, t_clip_max_);

  if (logger_info_ && lateral_error > 1.0) {
    logger_info_("[MAP Controller] Large lateral error: " + std::to_string(lateral_error) +
                 "m, L1_distance: " + std::to_string(L1_distance) + "m");
  }

  Eigen::Vector2d L1_point =
      waypoint_at_distance_before_car(L1_distance,
                                      waypoint_array_in_map_.leftCols<2>(),
                                      idx_nearest_waypoint_.value());
  return {L1_point, L1_distance};
}

std::optional<double> MAP_Controller::calc_speed_command(const Eigen::Vector2d& v,
                                                         double lat_e_norm)
{
  const double adv_ts_sp = speed_lookahead_;
  Eigen::Vector2d la_position(
      position_in_map_(0) + v(0) * adv_ts_sp,
      position_in_map_(1) + v(1) * adv_ts_sp);

  const int idx_la_position =
      nearest_waypoint(la_position, waypoint_array_in_map_.leftCols<2>());

  double global_speed = waypoint_array_in_map_(idx_la_position, 2);
  global_speed = speed_adjust_lat_err(global_speed, lat_e_norm);
  return global_speed;
}

double MAP_Controller::distance(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) {
  return (p2 - p1).norm();
}

double MAP_Controller::acc_scaling(double steer) const {
  const double mean_acc = (acc_now_.size() > 0) ? acc_now_.mean() : 0.0;
  if (mean_acc >= 0.8) {
    return steer * acc_scaler_for_steer_;
  } else if (mean_acc <= -0.8) {
    return steer * dec_scaler_for_steer_;
  }
  return steer;
}

double MAP_Controller::speed_steer_scaling(double steer, double speed) const {
  const double speed_diff = std::max(0.1, end_scale_speed_ - start_scale_speed_);
  const double factor = 1.0 - utils::clamp((speed - start_scale_speed_) / speed_diff, 0.0, 1.0) * downscale_factor_;
  return steer * factor;
}

std::pair<double,double> MAP_Controller::calc_lateral_error_norm() const {
  const double lateral_error = std::abs(position_in_map_frenet_(1));

  // Increase max lateral error to handle off-raceline starts
  const double max_lat_e = 2.0;  // Increased from 0.5 to 2.0 meters
  const double min_lat_e = 0.0;
  const double lat_e_clip = utils::clamp(lateral_error, min_lat_e, max_lat_e);
  const double lat_e_norm = 0.5 * ((lat_e_clip - min_lat_e) / (max_lat_e - min_lat_e));
  return {lat_e_norm, lateral_error};
}

double MAP_Controller::speed_adjust_lat_err(double global_speed, double lat_e_norm) const {
  double lat_e_coeff = lat_err_coeff_;
  lat_e_norm *= 2.0;

  const double curv = utils::clamp(2.0 * ( (curvature_waypoints_) / 0.8 ) - 2.0, 0.0, 1.0);
  global_speed *= (1.0 - lat_e_coeff + lat_e_coeff * std::exp(-lat_e_norm * curv));
  return global_speed;
}

int MAP_Controller::nearest_waypoint(const Eigen::Vector2d& position,
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

Eigen::Vector2d MAP_Controller::waypoint_at_distance_before_car(double distance,
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

} // namespace crazy_controller
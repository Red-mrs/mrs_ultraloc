#include <mrs_ultraloc/uwb_uvdar_fusion_node.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <Eigen/Dense>

namespace mrs_ultraloc
{

namespace {

/* Nanoseconds since the epoch as one double. Message stamps are only ever
 * compared against this node's clock, never against a calendar, so a scalar
 * keeps the staleness checks free of rclcpp::Time clock-type mismatches between
 * this node and the publishers it listens to. */
double toSec(const builtin_interfaces::msg::Time& t)
{
  return static_cast<double>(t.sec) + 1.0e-9 * static_cast<double>(t.nanosec);
}

builtin_interfaces::msg::Time toBuiltinTime(double sec)
{
  builtin_interfaces::msg::Time out;

  const double clamped = std::isfinite(sec) ? sec : 0.0;
  const double whole   = std::floor(clamped);

  out.sec     = static_cast<int32_t>(whole);
  out.nanosec = static_cast<uint32_t>(std::lround((clamped - whole) * 1.0e9));

  /* Rounding the fractional part up can push a whole second into nanoseconds. */
  if (out.nanosec >= 1000000000U) {
    out.sec += 1;
    out.nanosec = 0U;
  }

  return out;
}

/* How long ago `stamp` was, clamped at zero so a publisher running slightly
 * ahead of this clock counts as fresh rather than as expired. A stamp that is
 * missing or zero means "never measured", which no age can satisfy. */
double ageSec(double now, double stamp)
{
  if (!std::isfinite(stamp) || stamp <= 0.0) {
    return std::numeric_limits<double>::infinity();
  }

  return std::max(0.0, now - stamp);
}

/* Elapsed time since a reference instant of this node's own making. Unlike
 * ageSec(), a zero reference is not treated as "never": start_time_ is set from
 * the clock, and a clock that genuinely reads zero should still get its grace
 * period rather than an immediate warning. */
double elapsedSec(double now, double since)
{
  return std::max(0.0, now - since);
}

Eigen::Matrix3d toMatrix(const double row_major[9])
{
  Eigen::Matrix3d matrix;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      matrix(row, column) = row_major[3 * row + column];
    }
  }
  return matrix;
}

void fromMatrix(const Eigen::Matrix3d& matrix, double row_major[9])
{
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      row_major[3 * row + column] = matrix(row, column);
    }
  }
}

/* A covariance only has to be positive semi-definite, so the zero eigenvalue a
 * tangent-plane bearing covariance carries is legitimate. Only negative
 * eigenvalues are round-off; clamping those keeps a downstream filter from being
 * handed an indefinite matrix. */
Eigen::Matrix3d clampNegativeEigenvalues(const Eigen::Matrix3d& covariance)
{
  const Eigen::Matrix3d symmetric = 0.5 * (covariance + covariance.transpose());

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(symmetric);
  if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite()) {
    return Eigen::Matrix3d::Zero();
  }

  const Eigen::Vector3d eigenvalues = solver.eigenvalues().cwiseMax(0.0);
  return solver.eigenvectors() * eigenvalues.asDiagonal() * solver.eigenvectors().transpose();
}

} // namespace

/* UwbUvdarFusionNode() //{ */

UwbUvdarFusionNode::UwbUvdarFusionNode(const rclcpp::NodeOptions& options) : rclcpp::Node("uwb_uvdar_fusion", options)
{
  start_time_ = get_clock()->now().nanoseconds() * 1.0e-9;

  loadParams();
  parseIdPairs();
  createInterfaces();

  RCLCPP_INFO(get_logger(),
              "Fusing %zu UWB:UVDAR pair(s), '%s' + '%s' -> '%s' at %.1f Hz", uwb_to_uvdar_.size(),
              bearing_sub_->get_topic_name(), range_sub_->get_topic_name(), pub_fused_->get_topic_name(),
              publish_rate_hz_);
}

//}

/* loadParams() //{ */

void UwbUvdarFusionNode::loadParams()
{
  /* Both inputs are given as absolute defaults, because the producers live under
   * namespaces of their own (/uav/uvdar/..., /uav/uwb/distance) and a relative
   * name would silently resolve against whichever namespace this node happens to
   * be placed in. */
  bearing_topic_ = declare_parameter<std::string>("bearing_topic", "/uav/uvdar/bearing/camera_0/observations");
  uwb_topic_     = declare_parameter<std::string>("uwb_topic", "/uav/uwb/distance");
  output_topic_  = declare_parameter<std::string>("output_topic", "uwb_uvdar_fusion/targets");

  publish_rate_hz_     = declare_parameter<double>("publish_rate_hz", publish_rate_hz_);
  bearing_timeout_sec_ = declare_parameter<double>("bearing_timeout_sec", bearing_timeout_sec_);
  range_timeout_sec_   = declare_parameter<double>("range_timeout_sec", range_timeout_sec_);
  min_range_m_         = declare_parameter<double>("min_range_m", min_range_m_);
  max_range_m_         = declare_parameter<double>("max_range_m", max_range_m_);

  bearing_sigma_rad_     = declare_parameter<double>("bearing_sigma_rad", bearing_sigma_rad_);
  range_sigma_m_         = declare_parameter<double>("range_sigma_m", range_sigma_m_);
  min_range_variance_m2_ = declare_parameter<double>("min_range_variance_m2", min_range_variance_m2_);
  diagnostics_rate_hz_   = declare_parameter<double>("diagnostics_rate_hz", diagnostics_rate_hz_);
  queue_depth_           = declare_parameter<int>("queue_depth", queue_depth_);
  debug_                 = declare_parameter<bool>("debug", debug_);

  if (publish_rate_hz_ <= 0.0) {
    throw std::runtime_error("publish_rate_hz must be positive.");
  }

  if (queue_depth_ <= 0) {
    throw std::runtime_error("queue_depth must be positive.");
  }

  if (bearing_timeout_sec_ <= 0.0 || range_timeout_sec_ <= 0.0) {
    throw std::runtime_error("bearing_timeout_sec and range_timeout_sec must be positive.");
  }

  if (min_range_m_ < 0.0 || max_range_m_ <= min_range_m_) {
    throw std::runtime_error("max_range_m must be greater than a non-negative min_range_m.");
  }

  if (!std::isfinite(bearing_sigma_rad_) || bearing_sigma_rad_ < 0.0 || !std::isfinite(range_sigma_m_) ||
      range_sigma_m_ < 0.0 || !std::isfinite(min_range_variance_m2_) || min_range_variance_m2_ < 0.0) {
    throw std::runtime_error(
        "bearing_sigma_rad, range_sigma_m and min_range_variance_m2 must be finite and non-negative.");
  }
}

//}

/* parseIdPairs() //{ */

void UwbUvdarFusionNode::parseIdPairs()
{
  /* Written as "0xAA:28" pairs so the addressing scheme stays in the config: the
   * UWB side hexadecimal, because the modules are addressed that way, the UVDAR
   * side decimal, because that is what the tracker decodes. The four default
   * entries are the ones the ROS 1 fusion was built around. */
  const auto raw_pairs = declare_parameter<std::vector<std::string>>(
      "uwb_uvdar_id_pairs", std::vector<std::string> {"0xAA:28", "0xBB:29", "0xCC:30", "0xDD:31"});

  if (raw_pairs.empty()) {
    throw std::runtime_error("uwb_uvdar_id_pairs must contain at least one \"uwb_address:uvdar_id\" pair.");
  }

  std::map<int, uint32_t> uvdar_to_uwb;

  for (const auto& entry : raw_pairs) {
    const auto separator = entry.find(':');
    if (separator == std::string::npos) {
      throw std::runtime_error("Invalid uwb_uvdar_id_pairs entry '" + entry + "', expected \"uwb_address:uvdar_id\".");
    }

    /* Base 0 on both sides, so a decimal pair needs no prefix and a hexadecimal
     * UWB address carries its 0x. */
    const std::string uwb_field   = entry.substr(0, separator);
    const std::string uvdar_field = entry.substr(separator + 1);

    uint32_t uwb_address = 0;
    int      uvdar_id    = 0;

    try {
      uwb_address = static_cast<uint32_t>(std::stoul(uwb_field, nullptr, 0));
      uvdar_id    = std::stoi(uvdar_field, nullptr, 0);
    }
    catch (const std::exception&) {
      throw std::runtime_error("Invalid uwb_uvdar_id_pairs entry '" + entry + "', expected \"uwb_address:uvdar_id\".");
    }

    if (uvdar_id < 0) {
      throw std::runtime_error("UVDAR id in uwb_uvdar_id_pairs entry '" + entry +
                               "' must not be negative; negative ids mark unidentified tracks.");
    }

    /* One UWB address describes one vehicle and so does one signal id. A
     * duplicate here is a mistyped config that would otherwise fuse two vehicles
     * into one target. */
    const auto [uwb_it, uwb_inserted] = uwb_to_uvdar_.emplace(uwb_address, uvdar_id);
    if (!uwb_inserted) {
      std::ostringstream message;
      message << "UWB address 0x" << std::hex << uwb_address << std::dec << " is paired with two UVDAR ids ("
              << uwb_it->second << " and " << uvdar_id << ") in uwb_uvdar_id_pairs.";
      throw std::runtime_error(message.str());
    }

    const auto [uvdar_it, uvdar_inserted] = uvdar_to_uwb.emplace(uvdar_id, uwb_address);
    if (!uvdar_inserted) {
      std::ostringstream message;
      message << "UVDAR id " << uvdar_id << " is paired with two UWB addresses (0x" << std::hex << uvdar_it->second
              << " and 0x" << uwb_address << std::dec << ") in uwb_uvdar_id_pairs.";
      throw std::runtime_error(message.str());
    }
  }
}

//}

/* createInterfaces() //{ */

void UwbUvdarFusionNode::createInterfaces()
{
  /* The bearing endpoint publishes reliably so that both reliable and
   * best-effort consumers can match it; subscribing reliably here keeps the
   * retransmission that pairing offers. The range topic is published keep-last-
   * one, which a deeper reliable history still matches. */
  const auto input_qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<std::size_t>(queue_depth_))).reliable();

  bearing_sub_ = create_subscription<uvdar_core::msg::BearingObservationArrayStamped>(
      bearing_topic_, input_qos,
      [this](const uvdar_core::msg::BearingObservationArrayStamped::ConstSharedPtr& msg) { bearingCallback(msg); });

  range_sub_ = create_subscription<uwb_driver::msg::UwbRangeStamped>(
      uwb_topic_, input_qos, [this](const uwb_driver::msg::UwbRangeStamped::ConstSharedPtr& msg) { rangeCallback(msg); });

  pub_fused_ = create_publisher<mrs_ultraloc::msg::FusionTargetArrayStamped>(
      output_topic_, rclcpp::QoS(rclcpp::KeepLast(static_cast<std::size_t>(queue_depth_))));

  timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / publish_rate_hz_), [this]() { publishFusion(); });
}

//}

/* bearingCallback() //{ */

void UwbUvdarFusionNode::bearingCallback(const uvdar_core::msg::BearingObservationArrayStamped::ConstSharedPtr& msg)
{
  if (!msg) {
    return;
  }

  const double stamp = toSec(msg->header.stamp);

  /* Two observations of one id in one batch should not happen - the tracker
   * resolves one track per decoded signal - but averaging guards against a
   * producer that puts, say, two cameras on one topic. Averaging unit vectors
   * rather than angles keeps the result a direction.
   *
   * The accumulator carries its own zeroing, because operator[] on a map of
   * Eigen matrices default-constructs the value, which for Eigen leaves the
   * coefficients uninitialised. */
  struct Accumulator {
    Eigen::Vector3d direction = Eigen::Vector3d::Zero();
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    int             count      = 0;
    bool            measured   = false;
  };

  std::map<int, Accumulator> accumulated;

  for (const auto& observation : msg->observations) {
    if (observation.id < 0) {
      /* Unidentified track: no UWB address can be attached to it. */
      continue;
    }

    Eigen::Vector3d bearing(observation.bearing.x, observation.bearing.y, observation.bearing.z);
    if (!bearing.allFinite() || bearing.norm() <= std::numeric_limits<double>::epsilon()) {
      continue;
    }

    bearing.normalize();

    Accumulator& entry = accumulated[observation.id];
    entry.direction += bearing;
    entry.covariance += toMatrix(observation.covariance.data());
    entry.count += 1;
    entry.measured = entry.measured || !observation.predicted;
  }

  std::scoped_lock lock(data_mutex_);

  for (const auto& [id, entry] : accumulated) {
    if (entry.direction.norm() <= std::numeric_limits<double>::epsilon()) {
      continue;
    }

    const Eigen::Vector3d averaged = entry.direction.normalized();

    BearingSample sample;
    sample.x         = averaged.x();
    sample.y         = averaged.y();
    sample.z         = averaged.z();
    sample.predicted = !entry.measured;
    sample.stamp     = stamp;
    sample.frame_id  = msg->header.frame_id;

    const Eigen::Matrix3d mean_covariance = entry.covariance / static_cast<double>(entry.count);
    sample.has_covariance                 = mean_covariance.allFinite() && mean_covariance.norm() > 0.0;
    if (sample.has_covariance) {
      fromMatrix(mean_covariance, sample.covariance);
    }

    bearings_[id] = std::move(sample);
  }
}

//}

/* peerAddress() //{ */

std::optional<uint32_t> UwbUvdarFusionNode::peerAddress(const uwb_driver::msg::UwbRangeStamped& msg)
{
  /* A module reports its own address next to the two endpoints of the exchange, so
   * the peer is whichever endpoint is not us. When it is neither, the report came
   * from somebody else's module and says nothing about our distance to anybody. */
  if (msg.range.own_address == msg.range.responder_address) {
    return msg.range.initiator_address;
  }

  if (msg.range.own_address == msg.range.initiator_address) {
    return msg.range.responder_address;
  }

  return std::nullopt;
}

//}

/* rangeCallback() //{ */

void UwbUvdarFusionNode::rangeCallback(const uwb_driver::msg::UwbRangeStamped::ConstSharedPtr& msg)
{
  if (!msg) {
    return;
  }

  const auto peer = peerAddress(*msg);
  if (!peer) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Ignoring range with own_address %u, which is neither initiator %u nor responder %u; the "
                         "module is not this vehicle's.",
                         msg->range.own_address, msg->range.initiator_address, msg->range.responder_address);
    return;
  }

  const double distance = msg->range.distance;
  if (!std::isfinite(distance) || distance < min_range_m_ || distance > max_range_m_) {
    /* Outside the plausible window: either an invalid fix, which the modules report
     * rather than suppress, or a multipath outlier further than the camera could
     * plausibly be resolving a blinker. */
    RCLCPP_DEBUG(get_logger(), "Ignoring range %.2f m to 0x%X, outside [%.2f, %.2f] m.", distance, *peer,
                 min_range_m_, max_range_m_);
    return;
  }

  {
    std::scoped_lock lock(data_mutex_);

    RangeSample sample;
    sample.distance    = distance;
    sample.own_address = msg->range.own_address;
    sample.stamp       = toSec(msg->header.stamp);

    ranges_[*peer] = std::move(sample);
  }

  if (debug_) {
    RCLCPP_INFO_STREAM(get_logger(), "range " << distance << " m to 0x" << std::hex << *peer << std::dec);
  }
}

//}

/* publishFusion() //{ */

void UwbUvdarFusionNode::publishFusion()
{
  const double now = get_clock()->now().nanoseconds() * 1.0e-9;

  mrs_ultraloc::msg::FusionTargetArrayStamped output;
  output.header.stamp = toBuiltinTime(now);

  {
    std::scoped_lock lock(data_mutex_);

    for (const auto& [uwb_address, uvdar_id] : uwb_to_uvdar_) {
      const auto bearing_it = bearings_.find(uvdar_id);
      const auto range_it   = ranges_.find(uwb_address);

      if (bearing_it == bearings_.end() || range_it == ranges_.end()) {
        continue;
      }

      const BearingSample& bearing = bearing_it->second;
      const RangeSample&   range   = range_it->second;

      /* Both sides have to describe the same encounter. Ranges arrive in bursts of
       * a few hertz while bearings come continuously, so the two sensors need their
       * own timeouts. */
      if (ageSec(now, bearing.stamp) > bearing_timeout_sec_ || ageSec(now, range.stamp) > range_timeout_sec_) {
        continue;
      }

      Eigen::Vector3d direction(bearing.x, bearing.y, bearing.z);
      if (!direction.allFinite() || direction.norm() <= std::numeric_limits<double>::epsilon()) {
        continue;
      }
      direction.normalize();

      /* The whole geometry of the fusion: a direction, scaled by a distance. The
       * bearing already lives in a robot-fixed frame, so no camera model and no
       * mounting rotation enter here - see the class comment for what that assumes. */
      const Eigen::Vector3d position = direction * range.distance;

      /* Angular uncertainty grows into lateral uncertainty with distance, and range
       * uncertainty is purely radial, so the two terms occupy orthogonal subspaces
       * and just add. */
      Eigen::Matrix3d bearing_covariance = Eigen::Matrix3d::Zero();
      if (bearing.has_covariance) {
        bearing_covariance = toMatrix(bearing.covariance);
      } else {
        /* Nothing usable from the tracker: fall back to bearing_sigma_rad spread
         * evenly over the tangent plane, which is I - b b^T. */
        bearing_covariance = (bearing_sigma_rad_ * bearing_sigma_rad_) *
                             (Eigen::Matrix3d::Identity() - direction * direction.transpose());
      }
      bearing_covariance = clampNegativeEigenvalues(bearing_covariance);

      const double      radial_variance    = std::max(range_sigma_m_ * range_sigma_m_, min_range_variance_m2_);
      const Eigen::Matrix3d position_covariance =
          (range.distance * range.distance) * bearing_covariance + radial_variance * (direction * direction.transpose());

      mrs_ultraloc::msg::FusionTarget target;
      target.id                = uvdar_id;
      target.own_id            = range.own_address;
      target.position.x        = position.x();
      target.position.y        = position.y();
      target.position.z        = position.z();
      target.distance          = range.distance;
      target.bearing.x         = direction.x();
      target.bearing.y         = direction.y();
      target.bearing.z         = direction.z();
      target.bearing_predicted = bearing.predicted;
      target.bearing_stamp     = toBuiltinTime(bearing.stamp);
      target.range_stamp       = toBuiltinTime(range.stamp);
      fromMatrix(position_covariance, target.covariance.data());

      /* One frame per message. One bearing topic means one camera means one frame,
       * so a second distinct frame_id means the input was remapped mid-flight. */
      if (output.header.frame_id.empty()) {
        output.header.frame_id = bearing.frame_id;
      } else if (output.header.frame_id != bearing.frame_id) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Bearing frame changed from '%s' to '%s'; all positions are published in the newest one.",
                             output.header.frame_id.c_str(), bearing.frame_id.c_str());
        output.header.frame_id = bearing.frame_id;
      }

      output.targets.push_back(std::move(target));

      if (debug_) {
        RCLCPP_INFO_STREAM(get_logger(), "fused id " << uvdar_id << " (0x" << std::hex << uwb_address << std::dec
                                                     << ") at [" << position.transpose() << "] from " << range.distance
                                                     << " m, bearing " << std::fixed << ageSec(now, bearing.stamp)
                                                     << " s old, range " << ageSec(now, range.stamp) << " s old");
      }
    }
  }

  const bool published = !output.targets.empty();
  if (published) {
    pub_fused_->publish(std::move(output));
  }

  reportStaleness(now, published);
}

//}

/* reportStaleness() //{ */

void UwbUvdarFusionNode::reportStaleness(const double now, const bool published)
{
  if (diagnostics_rate_hz_ <= 0.0) {
    return;
  }

  const double interval = 1.0 / diagnostics_rate_hz_;
  if (last_diagnostics_time_ != 0.0 && ageSec(now, last_diagnostics_time_) < interval) {
    return;
  }
  last_diagnostics_time_ = now;

  std::scoped_lock lock(data_mutex_);

  /* A pair member that has never appeared is a config problem - a mistyped id, or
   * a topic nobody is publishing - and it will never fix itself, so it is worth
   * one warning each. A member that reported once and stopped is just a teammate
   * out of range, which is normal and only interesting while nothing at all is
   * being published. */
  const double never_seen_grace = std::max(5.0 * bearing_timeout_sec_, 5.0 * range_timeout_sec_);

  std::ostringstream stale_bearings;
  std::ostringstream stale_ranges;
  size_t             stale_bearings_count = 0;
  size_t             stale_ranges_count   = 0;

  for (const auto& [uwb_address, uvdar_id] : uwb_to_uvdar_) {
    const auto bearing_it = bearings_.find(uvdar_id);
    if (bearing_it == bearings_.end()) {
      if (elapsedSec(now, start_time_) > never_seen_grace) {
        warnOnce("bearing:" + std::to_string(uvdar_id),
                 "Never received a UVDAR bearing for id " + std::to_string(uvdar_id) + " on '" +
                     std::string(bearing_sub_->get_topic_name()) +
                     "'; check the id pairing and that the bearing endpoint is running.");
      }
    } else if (ageSec(now, bearing_it->second.stamp) > bearing_timeout_sec_) {
      if (stale_bearings_count++ > 0) {
        stale_bearings << ",";
      }
      stale_bearings << " " << uvdar_id;
    }

    const auto range_it = ranges_.find(uwb_address);
    if (range_it == ranges_.end()) {
      if (elapsedSec(now, start_time_) > never_seen_grace) {
        std::ostringstream address;
        address << std::hex << uwb_address;
        warnOnce("range:" + address.str(),
                 "Never received a UWB range from 0x" + address.str() + " on '" +
                     std::string(range_sub_->get_topic_name()) +
                     "'; check the id pairing and that this vehicle is in range.");
      }
    } else if (ageSec(now, range_it->second.stamp) > range_timeout_sec_) {
      if (stale_ranges_count++ > 0) {
        stale_ranges << ",";
      }
      stale_ranges << " 0x" << std::hex << uwb_address << std::dec;
    }
  }

  /* Nothing fused on this tick: name what is missing, repeating at the diagnostics
   * rate so a sensor that dies mid-flight is reported while it stays dead. */
  if (!published && (stale_bearings_count > 0 || stale_ranges_count > 0)) {
    RCLCPP_WARN(get_logger(), "No target fused: no bearing fresher than %.1f s for id(s)%s, no range fresher than "
                              "%.1f s for address(es)%s.",
                bearing_timeout_sec_, stale_bearings.str().c_str(), range_timeout_sec_, stale_ranges.str().c_str());
  }
}

//}

/* warnOnce() //{ */

void UwbUvdarFusionNode::warnOnce(const std::string& key, const std::string& message)
{
  if (warned_.count(key) != 0) {
    return;
  }

  warned_.insert(key);
  RCLCPP_WARN(get_logger(), "%s", message.c_str());
}

//}

//}

} // namespace mrs_ultraloc

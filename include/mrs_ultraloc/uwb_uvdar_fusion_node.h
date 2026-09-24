#pragma once

#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <uvdar_core/msg/bearing_observation_array_stamped.hpp>
#include <uwb_driver/msg/uwb_range_stamped.hpp>

#include <mrs_ultraloc/msg/fusion_target.hpp>
#include <mrs_ultraloc/msg/fusion_target_array_stamped.hpp>

namespace mrs_ultraloc
{

/*
 * Combines a UVDAR bearing with a UWB range for one camera and publishes the
 * resulting 3D position of each other vehicle.
 *
 * The bearing arrives already expressed in a robot-fixed frame
 * (BearingObservationArrayStamped.header.frame_id), so no camera calibration is
 * involved here: the position of the target is simply that bearing scaled by the
 * UWB range.
 *
 * One target is one UWB address, tied to the UVDAR signal id the same vehicle
 * blinks. Both sides of that pairing come from the `uwb_uvdar_id_pairs`
 * parameter, so nothing about the addressing scheme is compiled in.
 *
 * The fused position is published in the frame the bearing was stamped with. For
 * a camera that is its own optical frame; composing several cameras needs a
 * mounting transform, which this node deliberately does not do.
 */
class UwbUvdarFusionNode : public rclcpp::Node
{
public:
  explicit UwbUvdarFusionNode(const rclcpp::NodeOptions& options);

  UwbUvdarFusionNode(const UwbUvdarFusionNode&)            = delete;
  UwbUvdarFusionNode& operator=(const UwbUvdarFusionNode&) = delete;

private:
  /* Newest bearing seen for one UVDAR signal id. */
  struct BearingSample
  {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    /* Row-major 3x3 covariance of the unit bearing, typically rank two. */
    double covariance[9] = {0.0};

    bool   predicted = false;
    bool   has_covariance = false;
    double stamp = 0.0;

    /* Frame the bearing is expressed in, taken from the batch header. */
    std::string frame_id;
  };

  /* Newest range seen for one UWB address. */
  struct RangeSample
  {
    double   distance = 0.0;
    uint32_t own_address = 0;
    double   stamp = 0.0;
  };

  void loadParams();
  void parseIdPairs();
  void createInterfaces();

  void bearingCallback(const uvdar_core::msg::BearingObservationArrayStamped::ConstSharedPtr& msg);
  void rangeCallback(const uwb_driver::msg::UwbRangeStamped::ConstSharedPtr& msg);
  void publishFusion();

  /* The peer address of a range report: whichever endpoint is not us. */
  static std::optional<uint32_t> peerAddress(const uwb_driver::msg::UwbRangeStamped& msg);

  /* Warns about pair members that never appeared or have gone quiet, at
   * diagnostics_rate_hz_. Called from the publish tick, which is also the only
   * place that knows whether anything was fused. */
  void reportStaleness(double now, bool published);

  /* Logs `message` the first time `key` is seen, and never again. */
  void warnOnce(const std::string& key, const std::string& message);

  rclcpp::Subscription<uvdar_core::msg::BearingObservationArrayStamped>::SharedPtr bearing_sub_;
  rclcpp::Subscription<uwb_driver::msg::UwbRangeStamped>::SharedPtr                range_sub_;
  rclcpp::Publisher<mrs_ultraloc::msg::FusionTargetArrayStamped>::SharedPtr        pub_fused_;
  rclcpp::TimerBase::SharedPtr                                                     timer_;

  /* Guards bearings_ and ranges_, which the subscriptions and the timer touch. */
  mutable std::mutex data_mutex_;

  /* UVDAR signal id -> newest bearing. */
  std::map<int, BearingSample> bearings_;

  /* UWB address -> newest range. */
  std::map<uint32_t, RangeSample> ranges_;

  /* UWB address -> UVDAR signal id of the same vehicle. */
  std::map<uint32_t, int> uwb_to_uvdar_;

  std::string bearing_topic_;
  std::string uwb_topic_;
  std::string output_topic_;

  double publish_rate_hz_      = 20.0;
  double bearing_timeout_sec_  = 1.0;
  double range_timeout_sec_    = 2.0;
  double min_range_m_          = 0.3;
  double max_range_m_          = 100.0;

  /* Linearised bearing uncertainty, as an angular standard deviation in rad. */
  double bearing_sigma_rad_ = 0.02;

  /* Standard deviation of the UWB range, in metres. */
  double range_sigma_m_ = 0.30;

  /* Smallest variance reported on the axis along the bearing, in m^2. */
  double min_range_variance_m2_ = 0.01;

  /* Diagnostics interval for topics and pair members that stopped reporting. */
  double diagnostics_rate_hz_ = 0.5;

  int    queue_depth_ = 10;
  bool   debug_       = false;

  /* Reference instants for diagnostics, in seconds from this node's clock. */
  double start_time_            = 0.0;
  double last_diagnostics_time_ = 0.0;

  /* Pair members already reported as never seen, so the warning is not repeated. */
  std::set<std::string> warned_;
};

} // namespace mrs_ultraloc

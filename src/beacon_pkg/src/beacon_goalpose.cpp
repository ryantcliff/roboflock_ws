#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <robot_localization/srv/from_ll.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

// Converts a beacon measurement using the robot's GPS datum. This node only
// reports positions; it does not send navigation goals or velocity commands.
class BeaconGoalPose : public rclcpp::Node
{
public:
  BeaconGoalPose() : Node("beacon_goalpose")
  {
    max_age_ = declare_parameter("max_fix_age", 2.0);
    request_timeout_ = declare_parameter("request_timeout", 1.0);
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    if (!std::isfinite(max_age_) || max_age_ <= 0.0 ||
      !std::isfinite(request_timeout_) || request_timeout_ <= 0.0 || map_frame_.empty())
    {
      throw std::invalid_argument("Timeouts must be positive and finite; map_frame must be set");
    }
    converter_ = create_client<FromLL>("/fromLL");
    publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>("/beacon/map_pose", 10);
    // A service can exist before navsat_transform has established its datum.
    // Fresh GPS odometry in the expected frame is the readiness signal.
    gps_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/gps", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {gps_odometry_ = msg;});
    subscription_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gps/beacon/fix", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {
        ++generation_;  // Even an invalid new fix invalidates an older pending result.
        latest_fix_ = valid_fix(*msg) ? msg : nullptr;
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {tick();});
  }

private:
  using FromLL = robot_localization::srv::FromLL;
  using SteadyClock = std::chrono::steady_clock;

  bool fresh(const builtin_interfaces::msg::Time & stamp) const
  {
    if (stamp.sec == 0 && stamp.nanosec == 0) {return false;}
    const double age = (now() - rclcpp::Time(stamp, get_clock()->get_clock_type())).seconds();
    return age >= -0.2 && age <= max_age_;
  }

  bool valid_fix(const sensor_msgs::msg::NavSatFix & fix) const
  {
    return fix.status.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX &&
           fresh(fix.header.stamp) && std::isfinite(fix.latitude) &&
           std::isfinite(fix.longitude) && std::isfinite(fix.altitude) &&
           fix.latitude >= -90.0 && fix.latitude <= 90.0 &&
           fix.longitude >= -180.0 && fix.longitude <= 180.0;
  }

  bool ready() const
  {
    return gps_odometry_ && gps_odometry_->header.frame_id == map_frame_ &&
           fresh(gps_odometry_->header.stamp) && converter_->service_is_ready();
  }

  void tick()
  {
    if (pending_) {
      if (std::chrono::duration<double>(SteadyClock::now() - request_started_).count() >
        request_timeout_)
      {
        converter_->remove_pending_request(request_id_);
        pending_ = false;
        RCLCPP_WARN(get_logger(), "GPS conversion timed out; waiting for a new beacon fix");
      }
      return;
    }
    if (!latest_fix_ || sent_generation_ == generation_ || !ready() ||
      !valid_fix(*latest_fix_)) {return;}
    auto request = std::make_shared<FromLL::Request>();
    request->ll_point.latitude = latest_fix_->latitude;
    request->ll_point.longitude = latest_fix_->longitude;
    request->ll_point.altitude = latest_fix_->altitude;
    const auto generation = generation_;
    const auto fix = latest_fix_;
    sent_generation_ = generation;
    pending_ = true;
    request_started_ = SteadyClock::now();
    request_id_ = converter_->async_send_request(
      request, [this, generation, fix](rclcpp::Client<FromLL>::SharedFuture future) {
        pending_ = false;
        if (generation != generation_ || !valid_fix(*fix) || !ready() ||
          std::chrono::duration<double>(SteadyClock::now() - request_started_).count() >
          request_timeout_) {return;}
        try {
          const auto point = future.get()->map_point;
          if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.z)) {return;}
          geometry_msgs::msg::PoseStamped pose;
          // Preserve measurement age so a downstream follower can detect loss.
          pose.header.stamp = fix->header.stamp;
          pose.header.frame_id = map_frame_;
          pose.pose.position = point;
          pose.pose.position.z = 0.0;  // Planar navigation; this is a position, not a heading.
          pose.pose.orientation.w = 1.0;
          publisher_->publish(pose);
        } catch (const std::exception & error) {
          RCLCPP_WARN(get_logger(), "GPS conversion failed: %s", error.what());
        }
      }).request_id;
  }

  double max_age_;
  double request_timeout_;
  std::string map_frame_;
  uint64_t generation_{0};
  uint64_t sent_generation_{0};
  bool pending_{false};
  int64_t request_id_{0};
  SteadyClock::time_point request_started_;
  sensor_msgs::msg::NavSatFix::ConstSharedPtr latest_fix_;
  nav_msgs::msg::Odometry::ConstSharedPtr gps_odometry_;
  rclcpp::Client<FromLL>::SharedPtr converter_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gps_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BeaconGoalPose>());
  rclcpp::shutdown();
  return 0;
}

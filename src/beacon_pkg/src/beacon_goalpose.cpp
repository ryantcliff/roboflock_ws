#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include <geodesy/utm.h>
#include <geodesy/wgs84.h>
#include <geographic_msgs/msg/geo_point.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/string.hpp>

class BeaconGoalPose : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  BeaconGoalPose()
  : Node("beacon_goalpose")
  {
    stand_off_distance_ = declare_parameter<double>("stand_off_distance", 2.0);
    goal_update_distance_ = declare_parameter<double>("goal_update_distance", 1.0);
    goal_update_period_ = declare_parameter<double>("goal_update_period", 2.0);
    fix_timeout_ = declare_parameter<double>("fix_timeout", 3.0);

    robot_subscription_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gps/robot/fix", rclcpp::SensorDataQoS(),
      std::bind(&BeaconGoalPose::robot_callback, this, std::placeholders::_1));
    beacon_subscription_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gps/beacon/fix", rclcpp::SensorDataQoS(),
      std::bind(&BeaconGoalPose::beacon_callback, this, std::placeholders::_1));

    state_publisher_ = create_publisher<std_msgs::msg::String>("/tracking/state", 10);
    action_client_ = rclcpp_action::create_client<NavigateToPose>(
      this, "navigate_to_pose");
    timer_ = create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&BeaconGoalPose::update_goal, this));
  }

private:
  static bool valid_fix(const sensor_msgs::msg::NavSatFix & fix)
  {
    return fix.status.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX &&
           std::isfinite(fix.latitude) && std::isfinite(fix.longitude);
  }

  static geodesy::UTMPoint to_utm(const sensor_msgs::msg::NavSatFix & fix)
  {
    geographic_msgs::msg::GeoPoint point;
    point.latitude = fix.latitude;
    point.longitude = fix.longitude;
    point.altitude = fix.altitude;
    geodesy::UTMPoint utm;
    geodesy::fromMsg(point, utm);
    return utm;
  }

  void robot_callback(const sensor_msgs::msg::NavSatFix::SharedPtr message)
  {
    if (!valid_fix(*message)) {
      robot_valid_ = false;
      publish_state("robot_fix_invalid");
      return;
    }

    robot_ = to_utm(*message);
    robot_valid_ = true;
    robot_received_ = now();
    if (!origin_set_) {
      origin_ = robot_;
      origin_set_ = true;
      RCLCPP_INFO(get_logger(), "Set map datum from first valid robot GPS fix");
    }
  }

  void beacon_callback(const sensor_msgs::msg::NavSatFix::SharedPtr message)
  {
    if (!valid_fix(*message)) {
      beacon_valid_ = false;
      publish_state("beacon_fix_invalid");
      return;
    }

    beacon_ = to_utm(*message);
    beacon_valid_ = true;
    beacon_received_ = now();
  }

  bool fixes_ready()
  {
    if (!origin_set_ || !robot_valid_ || !beacon_valid_) {
      publish_state("waiting_for_fixes");
      return false;
    }
    const auto current = now();
    if ((current - robot_received_).seconds() > fix_timeout_) {
      publish_state("robot_fix_stale");
      return false;
    }
    if ((current - beacon_received_).seconds() > fix_timeout_) {
      publish_state("beacon_fix_stale");
      return false;
    }
    if (robot_.zone != beacon_.zone || robot_.band != beacon_.band) {
      publish_state("utm_zone_mismatch");
      return false;
    }
    return true;
  }

  void update_goal()
  {
    if (!fixes_ready()) {
      cancel_active_goal();
      return;
    }

    const double east = beacon_.easting - robot_.easting;
    const double north = beacon_.northing - robot_.northing;
    const double distance = std::hypot(east, north);
    if (distance <= stand_off_distance_) {
      cancel_active_goal();
      publish_state("within_follow_distance");
      return;
    }

    const double scale = (distance - stand_off_distance_) / distance;
    const double goal_easting = robot_.easting + east * scale;
    const double goal_northing = robot_.northing + north * scale;
    const double goal_x = goal_easting - origin_.easting;
    const double goal_y = goal_northing - origin_.northing;
    const double yaw = std::atan2(north, east);

    const auto current = now();
    if (goal_sent_) {
      const double moved = std::hypot(goal_x - last_goal_x_, goal_y - last_goal_y_);
      const double age = (current - last_goal_time_).seconds();
      if (moved < goal_update_distance_ && age < goal_update_period_) {
        return;
      }
    }

    if (!action_client_->wait_for_action_server(std::chrono::seconds(0))) {
      publish_state("nav2_unavailable");
      return;
    }

    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = current;
    goal.pose.pose.position.x = goal_x;
    goal.pose.pose.position.y = goal_y;
    goal.pose.pose.orientation.z = std::sin(yaw / 2.0);
    goal.pose.pose.orientation.w = std::cos(yaw / 2.0);

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback =
      [this](const GoalHandle::SharedPtr & handle) {
        active_goal_ = handle;
        publish_state(handle ? "goal_accepted" : "goal_rejected");
      };
    options.result_callback =
      [this](const GoalHandle::WrappedResult & result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          publish_state("goal_reached");
        } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
          publish_state("goal_canceled");
        } else {
          publish_state("goal_failed");
        }
      };
    cancel_active_goal();
    action_client_->async_send_goal(goal, options);

    last_goal_x_ = goal_x;
    last_goal_y_ = goal_y;
    last_goal_time_ = current;
    goal_sent_ = true;
    publish_state("goal_sent");
  }

  void cancel_active_goal()
  {
    if (active_goal_) {
      action_client_->async_cancel_goal(active_goal_);
      active_goal_.reset();
    }
  }

  void publish_state(const std::string & state)
  {
    if (state == last_state_) {
      return;
    }
    last_state_ = state;
    std_msgs::msg::String message;
    message.data = state;
    state_publisher_->publish(message);
    RCLCPP_INFO(get_logger(), "Tracking state: %s", state.c_str());
  }

  double stand_off_distance_;
  double goal_update_distance_;
  double goal_update_period_;
  double fix_timeout_;
  bool origin_set_{false};
  bool robot_valid_{false};
  bool beacon_valid_{false};
  bool goal_sent_{false};
  double last_goal_x_{0.0};
  double last_goal_y_{0.0};
  rclcpp::Time robot_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time beacon_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_goal_time_{0, 0, RCL_ROS_TIME};
  geodesy::UTMPoint origin_;
  geodesy::UTMPoint robot_;
  geodesy::UTMPoint beacon_;
  std::string last_state_;

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr robot_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr beacon_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  GoalHandle::SharedPtr active_goal_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BeaconGoalPose>());
  rclcpp::shutdown();
  return 0;
}

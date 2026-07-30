#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>

class OdomVisualization : public rclcpp::Node
{
public:
  OdomVisualization() : Node("odom_visualization")
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    child_frame_id_ = declare_parameter<std::string>("child_frame_id", "base");
    mesh_resource_ = declare_parameter<std::string>(
        "mesh_resource", "package://odom_visualization/meshes/hummingbird.mesh");
    scale_ = declare_parameter<double>("robot_scale", 1.0);
    publish_tf_ = declare_parameter<bool>("publish_tf", false);
    color_r_ = declare_parameter<double>("color.r", 0.0);
    color_g_ = declare_parameter<double>("color.g", 0.0);
    color_b_ = declare_parameter<double>("color.b", 0.0);
    color_a_ = declare_parameter<double>("color.a", 1.0);

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("path", 10);
    velocity_pub_ = create_publisher<visualization_msgs::msg::Marker>("velocity", 10);
    trajectory_pub_ = create_publisher<visualization_msgs::msg::Marker>("trajectory", 10);
    robot_pub_ = create_publisher<visualization_msgs::msg::Marker>("robot", 10);
    height_pub_ = create_publisher<sensor_msgs::msg::Range>("height", 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&OdomVisualization::odomCallback, this, std::placeholders::_1));
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    path_.header.frame_id = frame_id_;
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr message)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = message->header;
    if (pose.header.frame_id.empty()) pose.header.frame_id = frame_id_;
    pose.pose = message->pose.pose;
    pose_pub_->publish(pose);

    path_.header.stamp = pose.header.stamp;
    const auto &position = pose.pose.position;
    const double dx = position.x - last_path_position_.x;
    const double dy = position.y - last_path_position_.y;
    const double dz = position.z - last_path_position_.z;
    if (!has_last_path_pose_ || dx * dx + dy * dy + dz * dz > 0.25)
    {
      path_.poses.push_back(pose);
      last_path_position_ = position;
      has_last_path_pose_ = true;
      while (path_.poses.size() > 1000)
        path_.poses.erase(path_.poses.begin());
      path_pub_->publish(path_);
    }

    visualization_msgs::msg::Marker velocity;
    velocity.header = pose.header;
    velocity.ns = "velocity";
    velocity.id = 0;
    velocity.type = visualization_msgs::msg::Marker::ARROW;
    velocity.action = visualization_msgs::msg::Marker::ADD;
    velocity.scale.x = 0.04;
    velocity.scale.y = 0.08;
    velocity.scale.z = 0.08;
    velocity.color.r = 1.0;
    velocity.color.a = 1.0;
    geometry_msgs::msg::Point start = pose.pose.position;
    geometry_msgs::msg::Point end = start;
    end.x += message->twist.twist.linear.x;
    end.y += message->twist.twist.linear.y;
    end.z += message->twist.twist.linear.z;
    velocity.points = {start, end};
    velocity_pub_->publish(velocity);

    visualization_msgs::msg::Marker trajectory = velocity;
    trajectory.ns = "trajectory";
    trajectory.type = visualization_msgs::msg::Marker::LINE_STRIP;
    trajectory.scale.x = 0.025;
    trajectory.color.r = 0.1;
    trajectory.color.g = 0.8;
    trajectory.color.b = 1.0;
    trajectory.points.reserve(path_.poses.size());
    for (const auto &item : path_.poses) trajectory.points.push_back(item.pose.position);
    trajectory_pub_->publish(trajectory);

    visualization_msgs::msg::Marker robot;
    robot.header = pose.header;
    robot.ns = "robot";
    robot.id = 0;
    robot.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    robot.action = visualization_msgs::msg::Marker::ADD;
    robot.pose = pose.pose;
    robot.scale.x = robot.scale.y = robot.scale.z = scale_;
    robot.color.r = color_r_;
    robot.color.g = color_g_;
    robot.color.b = color_b_;
    robot.color.a = color_a_;
    robot.mesh_resource = mesh_resource_;
    robot.mesh_use_embedded_materials = true;
    robot_pub_->publish(robot);

    sensor_msgs::msg::Range height;
    height.header = pose.header;
    height.radiation_type = sensor_msgs::msg::Range::INFRARED;
    height.field_of_view = 0.01;
    height.min_range = 0.0;
    height.max_range = 100.0;
    height.range = static_cast<float>(pose.pose.position.z);
    height_pub_->publish(height);

    if (publish_tf_)
    {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = pose.header;
      transform.child_frame_id = child_frame_id_;
      transform.transform.translation.x = pose.pose.position.x;
      transform.transform.translation.y = pose.pose.position.y;
      transform.transform.translation.z = pose.pose.position.z;
      transform.transform.rotation = pose.pose.orientation;
      broadcaster_->sendTransform(transform);
    }
  }

  std::string frame_id_, child_frame_id_, mesh_resource_;
  double scale_{1.0}, color_r_{0.0}, color_g_{0.0}, color_b_{0.0}, color_a_{1.0};
  bool publish_tf_{false};
  bool has_last_path_pose_{false};
  geometry_msgs::msg::Point last_path_position_;
  nav_msgs::msg::Path path_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr velocity_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr robot_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr height_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomVisualization>());
  rclcpp::shutdown();
  return 0;
}

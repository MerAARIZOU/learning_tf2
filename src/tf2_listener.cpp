#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("tf2_listener");

  tf2_ros::Buffer tf_buffer;
  tf_buffer =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer);

  rclcpp::Rate loop_rate(10);
  while (rclcpp::ok()) {
    try {
      geometry_msgs::msg::TransformStamped transform_stamped;
      transform_stamped =
          tf_buffer.lookupTransform("base_link", "odom", rclcpp::Time(0));
      RCLCPP_INFO(node->get_logger(), "Translation: (%f, %f, %f)",
                  transform_stamped.transform.translation.x,
                  transform_stamped.transform.translation.y,
                  transform_stamped.transform.translation.z);
      // Vous pouvez également afficher la rotation ici
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(node->get_logger(), "Could not transform %s to %s: %s",
                  "target_frame", "source_frame", ex.what());
    }
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
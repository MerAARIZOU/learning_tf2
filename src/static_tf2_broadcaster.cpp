// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

class StaticFramePublisher : public rclcpp::Node {
public:
  explicit StaticFramePublisher() : Node("static_tf2_broadcaster") {
    tf_static_broadcaster_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // Publish static transforms once at startup
    this->make_transforms();
  }

private:
  void make_transforms() {
    geometry_msgs::msg::TransformStamped t;
    tf2::Quaternion q;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "base_footprint";
    t.child_frame_id = "base_link";

    t.transform.translation.x = 0;
    t.transform.translation.y = 0;
    t.transform.translation.z = 0;

    q.setRPY(0, 0, 0);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_static_broadcaster_->sendTransform(t);

    /*t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = "base_link";

    t.transform.translation.x = 0;
    t.transform.translation.y = 0;
    t.transform.translation.z = 0;

    q.setRPY(0, 0, 0);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_static_broadcaster_->sendTransform(t);*/
  }

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};

int main(int argc, char *argv[]) {
  auto logger = rclcpp::get_logger("logger");

  // Pass parameters and initialize node
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticFramePublisher>());
  rclcpp::shutdown();
  return 0;
}

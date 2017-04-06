#ifndef ROS_UTILS_SAFE_TRANSFORM_WRAPPER_HPP_
#define ROS_UTILS_SAFE_TRANSFORM_WRAPPER_HPP_

#include <ros/ros.h>
#include <string>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/TransformStamped.h>

namespace ros_utils {

class SafeTransformWrapper {
  public:
    SafeTransformWrapper();
    ~SafeTransformWrapper() = default;

    /// Get a valid transform, blocks until the transform is received or
    /// the request takes too long and times out
    bool __attribute__((warn_unused_result)) getTransformAtTime(
            geometry_msgs::TransformStamped& transform,
            const std::string& target_frame,
            const std::string& source_frame,
            const ros::Time& time,
            const ros::Duration& timeout) const;

  private:
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

} // namespace ros_utils

#endif // include guard

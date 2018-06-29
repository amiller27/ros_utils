#ifndef ROS_UTILS_PARAM_UTILS_HPP_
#define ROS_UTILS_PARAM_UTILS_HPP_

#include <ros/ros.h>
#include <string>

namespace ros_utils {

class ParamUtils {
  public:
    template<typename T>
    static T getParam(const ros::NodeHandle& nh, const std::string& name)
    {
        T val;
        ROS_ASSERT_MSG(nh.getParam(name, val),
                       "Failed to retrieve parameter: %s",
                       name.c_str());
        return val;
    }
};

} // namespace ros_utils

#endif // include guard

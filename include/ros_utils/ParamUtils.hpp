#ifndef ROS_UTILS_PARAM_UTILS_HPP_
#define ROS_UTILS_PARAM_UTILS_HPP_

#include <ros/ros.h>
#include <string>
#include <sstream>

namespace ros_utils {

class ParamUtils {
  public:
    template <typename T>
    static T getParam(const ros::NodeHandle& nh, const std::string& name) {
        T val;
        const bool success = nh.getParam(name, val);

        if (!success) {
            std::ostringstream ss;
            ss << "Failed to retrieve parameter: " << name;
            throw std::runtime_error(ss.str().c_str());
        }

        return val;
    }
};

}  // namespace ros_utils

#endif  // include guard

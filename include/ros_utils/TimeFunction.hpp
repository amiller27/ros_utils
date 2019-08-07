#pragma once

#include <ros/ros.h>
#include <chrono>

namespace ros_utils {

namespace detail {

class TimeTracker {
  public:
    TimeTracker() : start_(std::chrono::high_resolution_clock::now()) {}
    ~TimeTracker() {
        const auto end = std::chrono::high_resolution_clock::now();
        const auto ticks =
                std::chrono::duration_cast<std::chrono::microseconds>(end
                                                                      - start_)
                        .count();
        ROS_WARN_STREAM("Took " << ticks << "us");
    }

  private:
    const std::chrono::high_resolution_clock::time_point start_;
};

}  // end namespace detail

template <typename F, typename... Args>
std::result_of_t<F(Args...)> call_and_time(F&& f, Args&&... args) {
    detail::TimeTracker tracker{};
    return f(std::forward<Args>(args)...);
}

}  // end namespace ros_utils

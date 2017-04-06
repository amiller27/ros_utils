#ifndef ROS_UTILS_LINEAR_MSG_INTERPOLATOR_HPP_
#define ROS_UTILS_LINEAR_MSG_INTERPOLATOR_HPP_

#include <functional>
#include <ros/ros.h>
#include <string>

namespace ros_utils {

/// Subscribes to a topic and does linear interpolation between stamped
/// messages on that topic
///
/// MsgType must be a stamped message type (must have a header), and OutType
/// must implement standard linear operations (addition and scalar
/// multiplication and division)
template<class MsgType, typename OutType>
class LinearMsgInterpolator {
  public:
    //
    // CONSTRUCTORS
    //

    /// Constructor
    /// @param[in] nh                   NodeHandle for topic listener
    /// @param[in] topic_name           Name of topic to subscribe to
    /// @param[in] timeout              Max amount of time to wait for
    ///                                 blocking calls
    /// @param[in] allowed_time_offset  Allowed distance between requested
    ///                                 message time and returned message time
    /// @param[in] extractor            Function to extract an OutType from a
    ///                                 MsgType
    /// @param[in] queue_size           Queue size for subscriber
    LinearMsgInterpolator(ros::NodeHandle& nh,
                          const std::string& topic_name,
                          const ros::Duration& timeout,
                          const ros::Duration& allowed_time_offset,
                          const std::function<OutType(const MsgType&)>& extractor,
                          size_t queue_size = 100);

    ~LinearMsgInterpolator() = default;

    //
    // PUBLIC METHODS
    //

    /// Blocks waiting for a message to come in at the requested time,
    /// returns true if the result is valid.
    bool __attribute__((warn_unused_result)) getInterpolatedMsgAtTime(
            OutType& out,
            const ros::Time& time);

    /// Returns the last update time of the interpolator
    ///
    /// getInterpolatedMsgAtTime can only be called with times greater than
    /// or equal to this time
    const ros::Time& getLastUpdateTime() const;

    /// Blocks waiting for an initial message until the interpolator is ready
    ///
    /// The interpolator cannot be used until this method is called and
    /// return true.
    bool __attribute__((warn_unused_result)) waitUntilReady(
            const ros::Duration& startup_timeout);

  private:
    //
    // PRIVATE METHODS
    //

    /// Callback to handle messages on topic_name_
    void msgCallback(const typename MsgType::ConstPtr& msg);

    /// Comparator that compares a message by its timestamp
    static bool timeVsMsgStampedComparator(const MsgType& msg,
                                           const ros::Time& time)
    {
        return msg.header.stamp < time;
    }

    /// Blocks while waiting until we have an message in the queue at time
    /// or both before and after time
    ///
    /// Returns false if the operation times out or the precondition is not
    /// satisfied
    ///
    /// Precondition: the queue must contain a message with
    /// msg.header.stamp < last_update_time_
    ///
    /// Postcondition: the queue will contain a message with
    /// msg.header.stamp >= time and a message with
    /// msg.header.stamp < last_update_time_
    bool __attribute__((warn_unused_result)) waitForMsgAtTime(
            const ros::Time& time) const;

    //
    // INSTANCE VARIABLES
    //

    /// getInterpolatedMsgAtTime will allow requests for data at time t to
    /// return data at time (t - allowed_time_offset_) if none shows up
    /// at time t within the timeout
    const ros::Duration allowed_time_offset_;

    /// Function that extracts linearly interpolable data from a message
    const std::function<OutType(const MsgType&)> extractor_;

    /// Last time that getInterpolatedMsgAtTime was called
    ros::Time last_update_time_;

    /// Queue of received messages
    ///
    /// This will always (after waitUntilReady is called) contain at least one
    /// message older than the last update time
    std::vector<MsgType> msg_queue_;

    /// Subscriber for topic_name_
    const ros::Subscriber subscriber_;

    /// Max allowed wait time for waitForMsgAtTime
    const ros::Duration timeout_;

    /// Name of topic this interpolator is listening to
    const std::string topic_name_;
};

} // namespace ros_utils

#include "LinearMsgInterpolatorImpl.hpp"

#endif // include guard

/*
 * Implementation for the template class LinearMsgInterpolator
 * NOT TO BE INCLUDED ANYWHERE ELSE
 */

namespace ros_utils {

//
// CONSTRUCTOR
//

template<class MsgType, typename OutType>
LinearMsgInterpolator<MsgType, OutType>::LinearMsgInterpolator(
        ros::NodeHandle& nh,
        const std::string& topic_name,
        const ros::Duration& timeout,
        const ros::Duration& allowed_time_offset,
        const std::function<OutType(const MsgType&)>& extractor,
        size_t queue_size)
    : allowed_time_offset_(allowed_time_offset),
      extractor_(extractor),
      last_update_time_(),
      msg_queue_(),
      subscriber_(nh.subscribe(topic_name,
                               queue_size,
                               &LinearMsgInterpolator::msgCallback,
                               this)),
      timeout_(timeout),
      topic_name_(topic_name)
{
}

//
// PUBLIC METHODS
//

template<class MsgType, typename OutType>
bool LinearMsgInterpolator<MsgType, OutType>::getInterpolatedMsgAtTime(
        OutType& out,
        const ros::Time& time)
{
    if (time < last_update_time_) {
        ROS_ERROR_STREAM("LinearMsgInterpolator for ("
                      << topic_name_
                      << "): "
                      << "Attempted to get message with timestamp ("
                      << time
                      << ") before last fetched message at time ("
                      << last_update_time_
                      << ")");
        return false;
    }

    // Wait until there's a message with stamp >= time - allowed_time_offset_
    if (!waitForMsgAtTime(time - allowed_time_offset_))
    {
        ROS_ERROR_STREAM("LinearMsgInterpolator for ("
                      << topic_name_
                      << "): "
                      << "Can't get interpolated message, timed out waiting");
        return false;
    }

    ROS_ASSERT(msg_queue_.back().header.stamp >= time - allowed_time_offset_);

    if (msg_queue_.back().header.stamp <= time) {
        out = extractor_(msg_queue_.back());
        last_update_time_ = time;
        return true;
    }

    // Get first msg that is >= the requested time
    typename std::vector<MsgType>::iterator geq_msg
        = std::lower_bound(
                msg_queue_.begin(),
                msg_queue_.end(),
                time,
                &LinearMsgInterpolator::timeVsMsgStampedComparator);

    ROS_ASSERT_MSG(geq_msg > msg_queue_.begin(), "geq_msg - 1 out of bounds");
    ROS_ASSERT_MSG(geq_msg < msg_queue_.end(), "geq_msg out of bounds");

    // Linear interpolation between next_msg and last_msg
    const MsgType& next_msg = *geq_msg;
    const MsgType& last_msg = *(geq_msg - 1);

    const OutType& next_data = extractor_(next_msg);
    const OutType& last_data = extractor_(last_msg);

    double dt = (next_msg.header.stamp - last_msg.header.stamp).toSec();

    out = ((time - last_msg.header.stamp).toSec() / dt) * next_data
        + ((next_msg.header.stamp - time).toSec() / dt) * last_data;
    last_update_time_ = time;
    return true;
}

template<class MsgType, typename OutType>
const ros::Time& LinearMsgInterpolator<MsgType, OutType>::getLastUpdateTime()
        const
{
    return last_update_time_;
}

template<class MsgType, typename OutType>
bool LinearMsgInterpolator<MsgType, OutType>::waitUntilReady(
        const ros::Duration& startup_timeout)
{
    const ros::Time start_time = ros::Time::now();
    while (ros::ok()
           && msg_queue_.empty()
           && ros::Time::now() < start_time + startup_timeout) {
        ros::spinOnce();
        ros::Duration(0.005).sleep();
    }

    if (msg_queue_.empty()) {
        ROS_ERROR_STREAM("LinearMsgInterpolator for ("
                      << topic_name_
                      << ") failed to fetch initial message");
        return false;
    } else {
        last_update_time_ = msg_queue_.front().header.stamp
                          + ros::Duration(0, 1);
        return true;
    }
}

//
// PRIVATE METHODS
//

template<class MsgType, typename OutType>
void LinearMsgInterpolator<MsgType, OutType>::msgCallback(
        const typename MsgType::ConstPtr& msg)
{
    if (!msg_queue_.empty()
            && msg->header.stamp < msg_queue_.back().header.stamp) {
        ROS_ERROR_STREAM("LinearMsgInterpolator for ("
                      << topic_name_
                      << "): "
                      << "Ignoring message with timestamp of "
                      << msg->header.stamp
                      << " because the queue already has a message with stamp "
                      << msg_queue_.back().header.stamp);
        return;
    }

    msg_queue_.push_back(*msg);

    // Keep only one message older than the last update time
    typename std::vector<MsgType>::const_iterator first_geq_time
        = std::lower_bound(
                msg_queue_.begin(),
                msg_queue_.end(),
                last_update_time_,
                &LinearMsgInterpolator::timeVsMsgStampedComparator);
    if (first_geq_time - 1 > msg_queue_.begin()) {
        msg_queue_.erase(msg_queue_.begin(), first_geq_time - 1);
    }
}

template<class MsgType, typename OutType>
bool LinearMsgInterpolator<MsgType, OutType>::waitForMsgAtTime(
        const ros::Time& time) const
{
    if (msg_queue_.front().header.stamp >= last_update_time_) {
        ROS_ERROR_STREAM("Class invariant false in LinearMsgInterpolator for ("
                      << topic_name_
                      << "): "
                      << "contains no messages older than last_update_time_");
        return false;
    }

    const ros::Time start_time = ros::Time::now();
    while (ros::ok()
           && msg_queue_.back().header.stamp < time
           && ros::Time::now() < start_time + timeout_) {
        ros::spinOnce();
        ros::Duration(0.005).sleep();
    }

    return (msg_queue_.back().header.stamp >= time);
}

} // namespace ros_utils

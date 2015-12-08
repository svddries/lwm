#include "lwm/ros/lrf_buffer.h"

#include <geolib/ros/tf_conversions.h>
#include <ros/node_handle.h>
#include <tf/transform_listener.h>

namespace lwm
{

// ----------------------------------------------------------------------------------------------------

LRFBuffer::LRFBuffer() : tf_listener_(nullptr)
{
}

// ----------------------------------------------------------------------------------------------------

LRFBuffer::~LRFBuffer()
{
    delete tf_listener_;
}

// ----------------------------------------------------------------------------------------------------

void LRFBuffer::initialize(const std::string& topic)
{
    ros::NodeHandle nh;
    nh.setCallbackQueue(&cb_queue_);

   sub_scan_ = nh.subscribe(topic, 1, &LRFBuffer::scanCallback, this);

    if (!tf_listener_)
        tf_listener_ = new tf::TransformListener;
}

// ----------------------------------------------------------------------------------------------------

// Polls to see if there is a new scan with transform. If not, returns false
bool LRFBuffer::nextScan(const std::string& root_frame, std::vector<float>& ranges, geo::Pose3D& sensor_pose)
{
    // - - - - - - - - - - - - - - - - - -
    // Fetch scan image and place in image buffer

    cb_queue_.callAvailable();

    if (scan_buffer_.empty())
        return false;

    sensor_msgs::LaserScan::ConstPtr scan_msg = scan_buffer_.front();

    // - - - - - - - - - - - - - - - - - -
    // Determine pose based on TF

    try
    {
        tf::StampedTransform t_sensor_pose;
        tf_listener_->lookupTransform(root_frame, scan_msg->header.frame_id, scan_msg->header.stamp, t_sensor_pose);
        geo::convert(t_sensor_pose, sensor_pose);

        updateLaserModel(*scan_msg);

        ranges = scan_msg->ranges;

        scan_buffer_.pop();

        return true;
    }
    catch(tf::ExtrapolationException& ex)
    {
        try
        {
            // Now we have to check if the error was an interpolation or extrapolation error (i.e., the image is too old or
            // to new, respectively). If it is too old, discard it.

            tf::StampedTransform latest_sensor_pose;
            tf_listener_->lookupTransform(root_frame, scan_msg->header.frame_id, ros::Time(0), latest_sensor_pose);
            // If image time stamp is older than latest transform, throw it out
            if (latest_sensor_pose.stamp_ > scan_msg->header.stamp)
            {
                scan_buffer_.pop();
                ROS_WARN_STREAM("[IMAGE_BUFFER] Image too old to look-up tf: image timestamp = " << std::fixed
                                << scan_msg->header.stamp);
            }
        }
        catch(tf::TransformException& exc)
        {
            ROS_WARN("[IMAGE_BUFFER] Could not get latest sensor pose (probably because tf is still initializing): %s", ex.what());
        }
    }
    catch(tf::TransformException& ex)
    {
        ROS_WARN("[IMAGE_BUFFER] Could not get sensor pose: %s", ex.what());
    }

    return false;
}

// ----------------------------------------------------------------------------------------------------

// Blocks until a new scan with transform is found. Returns false if no scan or tf could be found within 'timeout_sec' seconds
bool LRFBuffer::waitForRecentScan(const std::string& root_frame, std::vector<float>& ranges, geo::Pose3D& sensor_pose, double timeout_sec)
{
    // - - - - - - - - - - - - - - - - - -
    // Wait until we get a new image

    ros::Time t_start = ros::Time::now();

    while(ros::ok())
    {
        if (ros::Time::now() - t_start > ros::Duration(timeout_sec))
            return false;

        // Remember old size
        std::size_t old_size = scan_buffer_.size();

        // Check if new messages in callback queue
        cb_queue_.callAvailable();

        // If there is a new message, we're done
        if (scan_buffer_.size() > old_size)
            break;
        else
            ros::Duration(0.1).sleep();
    }

    sensor_msgs::LaserScan::ConstPtr last_msg = scan_buffer_.back();

    // - - - - - - - - - - - - - - - - - -
    // Wait until we have a tf

    if (!tf_listener_->waitForTransform(root_frame, last_msg->header.frame_id, last_msg->header.stamp, ros::Duration(timeout_sec)))
        return false;

    // - - - - - - - - - - - - - - - - - -
    // Calculate tf

    try
    {
        tf::StampedTransform t_sensor_pose;
        tf_listener_->lookupTransform(root_frame, last_msg->header.frame_id, last_msg->header.stamp, t_sensor_pose);
        geo::convert(t_sensor_pose, sensor_pose);
    }
    catch(tf::TransformException& ex)
    {
        ROS_WARN("[IMAGE_BUFFER] Could not get sensor pose: %s", ex.what());
        return false;
    }

    ranges = last_msg->ranges;
    updateLaserModel(*last_msg);

    return true;
}

// ----------------------------------------------------------------------------------------------------

void LRFBuffer::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    scan_buffer_.push(msg);
}

// ----------------------------------------------------------------------------------------------------

void LRFBuffer::updateLaserModel(const sensor_msgs::LaserScan& msg)
{
    if (lrf_model_.getNumBeams() == msg.ranges.size())
        return;

    lrf_model_.setNumBeams(msg.ranges.size());
    lrf_model_.setAngleLimits(msg.angle_min, msg.angle_max);
    lrf_model_.setRangeLimits(msg.range_min, msg.range_max);
}

// ----------------------------------------------------------------------------------------------------


} // end namespace lwm


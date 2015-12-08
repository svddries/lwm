#ifndef LWM_LRF_BUFFER_H_
#define LWM_LRF_BUFFER_H_

#include <geolib/datatypes.h>
#include <geolib/sensors/LaserRangeFinder.h>
#include <queue>
#include <ros/callback_queue.h>
#include <ros/subscriber.h>
#include <sensor_msgs/LaserScan.h>

namespace tf
{
class TransformListener;
}

namespace lwm
{

// ----------------------------------------------------------------------------------------------------

class LRFBuffer
{

public:

    LRFBuffer();

    ~LRFBuffer();

    void initialize(const std::string& topic);

    // Polls to see if there is a new scan with transform. If not, returns false
    bool nextScan(const std::string& root_frame, std::vector<float>& ranges, geo::Pose3D& sensor_pose);

    // Blocks until a new scan with transform is found. Returns false if no scan or tf could be found within 'timeout_sec' seconds
    bool waitForRecentScan(const std::string& root_frame, std::vector<float>& ranges, geo::Pose3D& sensor_pose, double timeout_sec);

    const geo::LaserRangeFinder& model() const { return lrf_model_; }

private:

    ros::CallbackQueue cb_queue_;

    ros::Subscriber sub_scan_;

    std::queue<sensor_msgs::LaserScan::ConstPtr> scan_buffer_;

    tf::TransformListener* tf_listener_;

    geo::LaserRangeFinder lrf_model_;

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    void updateLaserModel(const sensor_msgs::LaserScan& msg);
};

} // end namespace lwm

#endif

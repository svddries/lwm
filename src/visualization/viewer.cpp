#include "lwm/visualization/viewer.h"

#include "lwm/world_model.h"

#include <opencv2/highgui/highgui.hpp>

namespace lwm
{

cv::Point worldToCanvas(const geo::Vec2& p)
{
    return cv::Point(300, 300) + cv::Point(-50 * p.y, -50 * p.x);
}

// ----------------------------------------------------------------------------------------------------

Viewer::Viewer()
{
}

// ----------------------------------------------------------------------------------------------------

Viewer::~Viewer()
{
}

// ----------------------------------------------------------------------------------------------------

void Viewer::show(const WorldModel& wm)
{
    cv::Mat canvas(600, 600, CV_8UC3, cv::Scalar(50, 50, 50));

    for(std::vector<Line>::const_iterator it = wm.lines().begin(); it != wm.lines().end(); ++it)
    {
        const Line& l = *it;
        const geo::Vec2& p1 = wm.vertices()[l.i1];
        const geo::Vec2& p2 = wm.vertices()[l.i2];

        cv::line(canvas, worldToCanvas(p1), worldToCanvas(p2), cv::Scalar(255, 255, 255), 1);
    }

    cv::imshow("world model", canvas);
    cv::waitKey(3);
}

// ----------------------------------------------------------------------------------------------------

} // end namespace lwm


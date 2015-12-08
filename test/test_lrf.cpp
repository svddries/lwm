#include <lwm/ros/lrf_buffer.h>
#include <lwm/visualization/viewer.h>
#include <lwm/world_model.h>

#include "timer.h"

#include <ros/init.h>
#include <ros/rate.h>

// ----------------------------------------------------------------------------------------------------

namespace lwm
{

class Updater
{

public:

    Updater() : sensor_pose_(geo::Transform2::identity()) {}

    void update(lwm::WorldModel& wm, const geo::LaserRangeFinder& model,
                const std::vector<float>& ranges, const geo::Transform2& delta_movement)
    {
        sensor_pose_ = sensor_pose_ * delta_movement;
        geo::Transform2 sensor_pose_inv = sensor_pose_.inverse();

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // Transform the vertices

        std::vector<geo::Vec2> t_vertices(wm.vertices().size());
        for(unsigned int i = 0; i < wm.vertices().size(); ++i)
            t_vertices[i] = sensor_pose_inv * wm.vertices()[i];

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // Render all world model lines

        std::vector<double> wm_ranges(model.getNumBeams(), 0);
        for(std::vector<Line>::const_iterator it = wm.lines().begin(); it != wm.lines().end(); ++it)
        {
            const Line& l = *it;
            const geo::Vec2& p1 = t_vertices[l.i1];
            const geo::Vec2& p2 = t_vertices[l.i2];

            model.renderLine(p1, p2, wm_ranges);
        }

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // Fit lines through sensor points and add to world model

        bool has_last_point = false;
        int i_vertex_last = -1;
        float rs_last;
        geo::Vec2 p_last;

        for(unsigned int i = 0; i < ranges.size(); ++i)
        {
            float rs = ranges[i];
            if (rs <= 0 || (has_last_point && std::abs(rs_last - rs) > 0.1))
            {
                i_vertex_last = -1;
                has_last_point = false;
                continue;
            }

            float rm = wm_ranges[i];

            if (rm > 0 && rs > rm - 0.1)
            {
                i_vertex_last = -1;
                has_last_point = false;
                continue;
            }

            geo::Vec3 p_3d = model.rayDirections()[i] * rs;
            geo::Vec2 p(p_3d.x, p_3d.y);
            geo::Vec2 p_world = sensor_pose_ * p;

            if (has_last_point)
            {
                int i1 = i_vertex_last >= 0 ? i_vertex_last : wm.addVertex(p_last);
                int i2 = wm.addVertex(p_world);
                wm.addLine(i1, i2);
                i_vertex_last = i2;
            }
            else
            {
                i_vertex_last = -1;
            }

            rs_last = rs;
            has_last_point = true;
            p_last = p_world;
        }
    }

private:

    geo::Transform2 sensor_pose_;

};

}

// ----------------------------------------------------------------------------------------------------

void convert3DTo2D(const geo::Pose3D& pose_3d, geo::Transform2& pose_2d)
{
    pose_2d.R = geo::Mat2(pose_3d.R.xx, pose_3d.R.xy, pose_3d.R.yx, pose_3d.R.yy);
    pose_2d.t = geo::Vec2(pose_3d.t.x, pose_3d.t.y);
}

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mwm_test_amigo");

    lwm::LRFBuffer scan_buffer;
    scan_buffer.initialize("/amigo/base_laser/scan");

    lwm::Updater updater;
    lwm::WorldModel world_model;
    lwm::Viewer viewer;

    bool has_previous_pose = false;
    geo::Transform2 previous_pose;

    ros::Rate r(100);
    while(ros::ok())
    {
        std::vector<float> ranges;
        geo::Pose3D sensor_pose;
        if (scan_buffer.nextScan("/amigo/odom", ranges, sensor_pose))
        {
            geo::Transform2 sensor_pose_2d;
            convert3DTo2D(sensor_pose, sensor_pose_2d);

            if (has_previous_pose)
            {
                geo::Transform2 delta = previous_pose.inverse() * sensor_pose_2d;

                Timer timer;
                updater.update(world_model, scan_buffer.model(), ranges, delta);
                timer.stop();

                std::cout << world_model.vertices().size() << " vertices" << std::endl;
                std::cout << world_model.lines().size() << " lines" << std::endl;
                std::cout << "Update took " << timer.getElapsedTimeInMilliSec() << " ms" << std::endl;
                std::cout << "-----------------------------" << std::endl;
            }

            previous_pose = sensor_pose_2d;
            has_previous_pose = true;
        }

        viewer.show(world_model);

        r.sleep();
    }

    return 0;
}

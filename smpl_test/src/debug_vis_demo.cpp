#include <cmath>
#include <thread>

#include <ros/ros.h>

#define SV_PACKAGE_NAME "smpl_test"
#include <smpl/debug/visualize.h>
#include <smpl_ros/debug/visualizer_ros.h>

auto MakeCubeMarker(double t) -> smpl::visual::Marker
{
    ROS_DEBUG("make cube marker");
    auto marker = smpl::visual::Marker{ };

    marker.pose.position = Eigen::Vector3d(std::sin(t), 0.0, 0.0);
    marker.pose.orientation = Eigen::Quaterniond::Identity();
    marker.shape = smpl::visual::Cube{ 1.0, 1.0, 1.0 };
    marker.color = smpl::visual::Color{ 1.0f, 0.0f, 0.0f, 1.0f };
    marker.frame_id = "map";
    marker.ns = "cube";

    return marker;
}

auto MakeSphereMarker(double t) -> smpl::visual::Marker
{
    ROS_DEBUG("make sphere marker");
    auto marker = smpl::visual::Marker{ };

    marker.pose.position = Eigen::Vector3d(0.0, std::sin(t), 0.0);
    marker.pose.orientation = Eigen::Quaterniond::Identity();
    marker.shape = smpl::visual::Sphere{ 0.5 };
    marker.color = smpl::visual::Color{ 0.0f, 1.0f, 0.0f, 1.0f };
    marker.frame_id = "map";
    marker.ns = "sphere";

    return marker;
}

auto MakeCylinderMarker(double t) -> smpl::visual::Marker
{
    ROS_DEBUG("make cylinder marker");
    auto marker = smpl::visual::Marker{ };
    marker.pose.position = Eigen::Vector3d(0.0, 0.0, std::sin(t));
    marker.pose.orientation = Eigen::Quaterniond::Identity();
    marker.shape = smpl::visual::Cylinder{ 0.5, 1.0 };
    marker.color = smpl::visual::Color{ 0.0f, 0.0f, 1.0f, 1.0f };
    marker.frame_id = "map";
    marker.ns = "cylinder";
    return marker;
}

void VisualizeCube()
{
    auto beginning = ros::Time::now();

    ros::Rate loop_rate(30.0);
    while (ros::ok()) {
        auto now = ros::Time::now();

        SV_SHOW_INFO_NAMED("cube", MakeCubeMarker((now - beginning).toSec()));

        loop_rate.sleep();
    }
}

void VisualizeSphere()
{
    auto beginning = ros::Time::now();

    ros::Rate loop_rate(30.0);
    while (ros::ok()) {
        auto now = ros::Time::now();

        SV_SHOW_INFO_NAMED("sphere", MakeSphereMarker((now - beginning).toSec()));

        loop_rate.sleep();
    }
}

void VisualizeCylinder()
{
    auto beginning = ros::Time::now();

    ros::Rate loop_rate(30.0);
    while (ros::ok()) {
        auto now = ros::Time::now();

        SV_SHOW_WARN_NAMED("cylinder", MakeCylinderMarker((now - beginning).toSec()));

        loop_rate.sleep();
    }
}

/// This program demonstrates functionality provided by smpl's debug
/// visualization module and associates. There's a few important things being
/// demonstrated here:
///
/// (1) smpl's debug visualization module works through a global debug
///     visualizer, registered by the user. Here, we register a global debug
///     visualizer that visualizes markers by publishing messages of type
///     visualization_msgs/MarkerArray, converted from smpl's visual::Marker.
///
/// (2) Visualization levels are set dynamically so that the demonstration
///     alternates between visualizing a cube and visualizing a sphere. The
///     level of the cylinder visualization is also modified, but not set high
///     enough to disable the visualization. Also, to set visualization levels
///     dynamically, the name of the visualization must be resolved via the
///     SV_NAME_PREFIX macro. which is a concatenation of the root visualization
///     name and the name given to all visualizations in the package, configured
///     in cmake via SV_PACKAGE_NAME.
///
/// (3) Thread-safety of the module is stressed by visualizing the objects and
///     modifying the visualization levels in parallel.
///
/// (4) Some other things that would be worth demonstrating: (i) visualization
///     names shared between different visualization macros, (ii) alternative
///     macros i.e. cond and timed variants, (iii) direct Marker/MarkerArray
///     interoperability, (iv) other marker types...
///
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "debug_vis_demo");
    ros::NodeHandle nh;

    smpl::VisualizerROS visualizer;
    smpl::visual::set_visualizer(&visualizer);

    std::thread cube_vis_thread(VisualizeCube);
    std::thread sphere_vis_thread(VisualizeSphere);
    std::thread cylinder_vis_thread(VisualizeCylinder);

    ROS_INFO("SV_PACKAGE_NAME: %s", SV_NAME_PREFIX);

    ros::Rate loop_rate(1.0);
    bool enabled = true;
    while (ros::ok()) {
        if (enabled) {
            smpl::visual::set_visualization_level(
                    std::string(SV_NAME_PREFIX) + ".cube",
                    smpl::visual::Level::Warn);
            smpl::visual::set_visualization_level(
                    std::string(SV_NAME_PREFIX) + ".sphere",
                    smpl::visual::Level::Info);
            smpl::visual::set_visualization_level(
                    std::string(SV_NAME_PREFIX) + ".cylinder",
                    smpl::visual::Level::Warn);
        } else {
            smpl::visual::set_visualization_level(
                    std::string(SV_NAME_PREFIX) + ".cube",
                    smpl::visual::Level::Info);
            smpl::visual::set_visualization_level(
                    std::string(SV_NAME_PREFIX) + ".sphere",
                    smpl::visual::Level::Warn);
            smpl::visual::set_visualization_level(
                    std::string(SV_NAME_PREFIX) + ".cylinder",
                    smpl::visual::Level::Info);
        }
        enabled = !enabled;

        loop_rate.sleep();
    }

    cube_vis_thread.join();
    sphere_vis_thread.join();
    cylinder_vis_thread.join();

    smpl::visual::unset_visualizer();
    return 0;
}

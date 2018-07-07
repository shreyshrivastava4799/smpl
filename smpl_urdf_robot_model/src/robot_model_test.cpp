// standard includes
#include <stdio.h>

// system includes
#include <ros/ros.h>
#include <smpl/debug/visualize.h>
#include <smpl/debug/visualizer_ros.h>
#include <urdf_parser/urdf_parser.h>

// project includes
#include <smpl_urdf_robot_model/robot_model.h>
#include <smpl_urdf_robot_model/robot_state.h>

namespace smpl {

auto MakeShapeVisualization(const Shape* shape) -> sbpl::visual::Shape
{
    switch (shape->type) {
    case ShapeType::Sphere:
        ROS_WARN("Unimplemented sphere type");
        return sbpl::visual::Shape{ };
    case ShapeType::Box:
        ROS_WARN("Unimplemented box type");
        return sbpl::visual::Shape{ };
    case ShapeType::Cylinder:
        ROS_WARN("Unimplemented cylinder type");
        return sbpl::visual::Shape{ };
        break;
    case ShapeType::Mesh:
    {
        auto* tmp = static_cast<const Mesh*>(shape);
        return sbpl::visual::MeshResource{ tmp->filename, tmp->scale };
    }
    }
}

auto MakeRobotVisualization(const RobotState* state)
    -> std::vector<sbpl::visual::Marker>
{
    ROS_INFO("Make visualization");

    std::vector<sbpl::visual::Marker> markers;
    auto id = 0;
    for (auto& link : Links(state->model)) {
        auto* pose = GetLinkTransform(state, &link);
        for (auto& visual : link.visual) {
            sbpl::visual::Marker m;

            m.pose = *GetVisualBodyTransform(state, &visual);
            m.shape = MakeShapeVisualization(visual.shape);
            m.color = sbpl::visual::Color{ 0.5f, 0.5f, 0.5f, 1.0f };
            m.frame_id = "map";
            m.ns = "test_visual";
            m.lifetime = 0;
            m.id = id++;
            markers.push_back(m);
        }
    }

    ROS_INFO("Visualize %zu shapes", markers.size());
    return markers;
}

auto MakeCollisionVisualization(const RobotState* state)
    -> std::vector<sbpl::visual::Marker>
{
    ROS_INFO("Make visualization");

    std::vector<sbpl::visual::Marker> markers;
    auto id = 0;
    for (auto& link : Links(state->model)) {
        for (auto& collision : link.collision) {
            sbpl::visual::Marker m;
            m.pose = *GetCollisionBodyTransform(state, &collision);
            m.shape = MakeShapeVisualization(collision.shape);
            m.color = sbpl::visual::Color{ 0.5f, 0.5f, 0.5f, 1.0f };
            m.frame_id = "map";
            m.ns = "test_collision";
            m.lifetime = 0;
            m.id = id++;
            markers.push_back(m);
        }
    }

    ROS_INFO("Visualize %zu shapes", markers.size());
    return markers;
}

} // namespace smpl

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "robot_model_test");
    ros::NodeHandle nh;

    sbpl::VisualizerROS visualizer;
    sbpl::visual::set_visualizer(&visualizer);
    ros::Duration(1.0).sleep();

    auto model = urdf::parseURDFFile(argv[1]);

    smpl::RobotModel robot_model;
    if (!InitRobotModel(&robot_model, model.get())) {
        return 1;
    }

    printf("Name: %s\n", GetName(&robot_model)->c_str());
    printf("Joint Count: %zu\n", GetJointCount(&robot_model));
    printf("Link Count: %zu\n", GetLinkCount(&robot_model));

#if 0
    std::vector<const smpl::Link*> links;
    links.push_back(GetRootLink(&robot_model));

    while (!links.empty()) {
        auto* link = links.back();
        links.pop_back();
        printf("Link %s\n", link->name.c_str());
        printf("  parent joint: %s\n", link->parent ? link->parent->name.c_str() : "(none)");
        printf("  child joints:\n");
        for (auto* child = link->children; child != NULL; child = child->sibling) {
            printf("    %s\n", child->name.c_str());
            links.push_back(child->child);
        }
    }
#endif

    printf("Links:\n");
    for (auto& link : Links(&robot_model)) {
        printf("  %s\n", link.name.c_str());
        printf("    parent joint: %s\n", link.parent ? link.parent->name.c_str() : "(none)");
        printf("    child joints:\n");
        for (auto* child = link.children; child != NULL; child = child->sibling) {
            printf("      %s\n", child->name.c_str());
        }
    }

    printf("Joints:\n");
    for (auto& joint : Joints(&robot_model)) {
        printf("  %s\n", joint.name.c_str());
    }

    printf("Variables:\n");
    for (auto& variable : Variables(&robot_model)) {
        printf("  %s\n", variable.name.c_str());
        printf("    has position limits: %s\n", variable.limits.has_position_limits ? "true" : "false");
        if (variable.limits.has_position_limits) {
            printf("    min_position: %f\n", variable.limits.min_position);
            printf("    max_position: %f\n", variable.limits.max_position);
            printf("    max_velocity: %f\n", variable.limits.max_velocity);
            printf("    max_effort: %f\n", variable.limits.max_effort);
        }
        printf("    joint: %s\n", variable.joint->name.c_str());
    }

    smpl::RobotState state;
    Init(&state, &robot_model);

    SetToDefaultValues(&state);
    printf("default state:\n");
    for (auto& variable : Variables(&robot_model)) {
        printf("%f\n", GetVariablePosition(&state, &variable));
    }

    printf("transforms:\n");
    UpdateLinkTransforms(&state);
    for (int i = 0; i < GetLinkCount(&robot_model); ++i) {
        auto* trans = GetLinkTransform(&state, i);
        printf("  %s: [ [ %f, %f, %f, %f ], [ %f, %f, %f, %f ], [ %f, %f, %f, %f ], [ %f, %f, %f, %f ] ]\n",
                GetLinkName(&robot_model, i)->c_str(),
                (*trans)(0, 0), (*trans)(1, 0), (*trans)(2, 0), (*trans)(3, 0),
                (*trans)(0, 1), (*trans)(1, 1), (*trans)(2, 1), (*trans)(3, 1),
                (*trans)(0, 2), (*trans)(1, 2), (*trans)(2, 2), (*trans)(3, 2),
                (*trans)(0, 3), (*trans)(1, 3), (*trans)(2, 3), (*trans)(3, 3));

    }

    SV_SHOW_INFO(MakeRobotVisualization(&state));
    SV_SHOW_INFO(MakeCollisionVisualization(&state));

    ros::Duration(1.0).sleep();

    return 0;
}


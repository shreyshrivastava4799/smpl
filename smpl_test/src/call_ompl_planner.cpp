/// \author Benjamin Cohen
/// \author Andrew Dornbush

// standard includes
#include <stdlib.h>
#include <string>
#include <thread>
#include <vector>

// system includes
#include <eigen_conversions/eigen_msg.h>
#include <leatherman/print.h>
#include <leatherman/utils.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/PlanningScene.h>
#include <ros/ros.h>
#include <kdl_conversions/kdl_msg.h>
#include <smpl/ros/planner_interface.h>
#include <smpl/distance_map/euclid_distance_map.h>
#include <sbpl_collision_checking/collision_space.h>
#include <sbpl_kdl_robot_model/kdl_robot_model.h>
#include <visualization_msgs/MarkerArray.h>
#include <smpl/angles.h>
#include <smpl/debug/visualizer_ros.h>

#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>

#include <smpl_ompl_interface/ompl_interface.h>

#include <urdf_parser/urdf_parser.h>

#include "collision_space_scene.h"
#include "pr2_allowed_collision_pairs.h"

namespace smpl = sbpl::motion;

void FillGoalConstraint(
    const std::vector<double>& pose,
    std::string frame_id,
    moveit_msgs::Constraints& goals)
{
    if (pose.size() < 6) {
        return;
    }

    goals.position_constraints.resize(1);
    goals.orientation_constraints.resize(1);
    goals.position_constraints[0].header.frame_id = frame_id;

    goals.position_constraints[0].constraint_region.primitives.resize(1);
    goals.position_constraints[0].constraint_region.primitive_poses.resize(1);
    goals.position_constraints[0].constraint_region.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    goals.position_constraints[0].constraint_region.primitive_poses[0].position.x = pose[0];
    goals.position_constraints[0].constraint_region.primitive_poses[0].position.y = pose[1];
    goals.position_constraints[0].constraint_region.primitive_poses[0].position.z = pose[2];

//    goals.position_constraints[0].position.x = pose[0];
//    goals.position_constraints[0].position.y = pose[1];
//    goals.position_constraints[0].position.z = pose[2];

    Eigen::Quaterniond q;
    sbpl::angles::from_euler_zyx(pose[5], pose[4], pose[3], q);
    tf::quaternionEigenToMsg(q, goals.orientation_constraints[0].orientation);

    geometry_msgs::Pose p;
    p.position = goals.position_constraints[0].constraint_region.primitive_poses[0].position;
    p.orientation = goals.orientation_constraints[0].orientation;
    leatherman::printPoseMsg(p, "Goal");

    /// set tolerances
    goals.position_constraints[0].constraint_region.primitives[0].dimensions.resize(3, 0.015);
    goals.orientation_constraints[0].absolute_x_axis_tolerance = 0.05;
    goals.orientation_constraints[0].absolute_y_axis_tolerance = 0.05;
    goals.orientation_constraints[0].absolute_z_axis_tolerance = 0.05;

    ROS_INFO("Done packing the goal constraints message.");
}

auto GetCollisionCube(
    const geometry_msgs::Pose& pose,
    std::vector<double>& dims,
    const std::string& frame_id,
    const std::string& id)
    -> moveit_msgs::CollisionObject
{
    moveit_msgs::CollisionObject object;
    object.id = id;
    object.operation = moveit_msgs::CollisionObject::ADD;
    object.header.frame_id = frame_id;
    object.header.stamp = ros::Time::now();

    shape_msgs::SolidPrimitive box_object;
    box_object.type = shape_msgs::SolidPrimitive::BOX;
    box_object.dimensions.resize(3);
    box_object.dimensions[0] = dims[0];
    box_object.dimensions[1] = dims[1];
    box_object.dimensions[2] = dims[2];

    object.primitives.push_back(box_object);
    object.primitive_poses.push_back(pose);
    return object;
}

auto GetCollisionCubes(
    std::vector<std::vector<double>>& objects,
    std::vector<std::string>& object_ids,
    const std::string& frame_id)
    -> std::vector<moveit_msgs::CollisionObject>
{
    std::vector<moveit_msgs::CollisionObject> objs;
    std::vector<double> dims(3,0);
    geometry_msgs::Pose pose;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;

    if (object_ids.size() != objects.size()) {
        ROS_INFO("object id list is not same length as object list. exiting.");
        return objs;
    }

    for (size_t i = 0; i < objects.size(); i++) {
        pose.position.x = objects[i][0];
        pose.position.y = objects[i][1];
        pose.position.z = objects[i][2];
        dims[0] = objects[i][3];
        dims[1] = objects[i][4];
        dims[2] = objects[i][5];

        objs.push_back(GetCollisionCube(pose, dims, frame_id, object_ids.at(i)));
    }
    return objs;
}

auto GetCollisionObjects(
    const std::string& filename,
    const std::string& frame_id)
    -> std::vector<moveit_msgs::CollisionObject>
{
    char sTemp[1024];
    int num_obs = 0;
    std::vector<std::string> object_ids;
    std::vector<std::vector<double> > objects;
    std::vector<moveit_msgs::CollisionObject> objs;

    FILE* fCfg = fopen(filename.c_str(), "r");

    if (fCfg == NULL) {
        ROS_INFO("ERROR: unable to open objects file. Exiting.\n");
        return objs;
    }

    // get number of objects
    if (fscanf(fCfg,"%s",sTemp) < 1) {
        printf("Parsed string has length < 1.\n");
    }

    num_obs = atoi(sTemp);

    ROS_INFO("%i objects in file",num_obs);

    //get {x y z dimx dimy dimz} for each object
    objects.resize(num_obs);
    object_ids.clear();
    for (int i=0; i < num_obs; ++i) {
        if (fscanf(fCfg,"%s",sTemp) < 1) {
            printf("Parsed string has length < 1.\n");
        }
        object_ids.push_back(sTemp);

        objects[i].resize(6);
        for (int j=0; j < 6; ++j)
        {
            if (fscanf(fCfg,"%s",sTemp) < 1) {
                printf("Parsed string has length < 1.\n");
            }
            if (!feof(fCfg) && strlen(sTemp) != 0) {
                objects[i][j] = atof(sTemp);
            }
        }
    }

    return GetCollisionCubes(objects, object_ids, frame_id);
}

bool ReadInitialConfiguration(
    ros::NodeHandle& nh,
    moveit_msgs::RobotState& state)
{
    XmlRpc::XmlRpcValue xlist;

    // joint_state
    if (nh.hasParam("initial_configuration/joint_state")) {
        nh.getParam("initial_configuration/joint_state", xlist);

        if (xlist.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            ROS_WARN("initial_configuration/joint_state is not an array.");
        }

        if (xlist.size() > 0) {
            std::cout << xlist << std::endl;
            for (int i = 0; i < xlist.size(); ++i) {
                state.joint_state.name.push_back(std::string(xlist[i]["name"]));

                if (xlist[i]["position"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                    state.joint_state.position.push_back(double(xlist[i]["position"]));
                }
                else {
                    ROS_DEBUG("Doubles in the yaml file have to contain decimal points. (Convert '0' to '0.0')");
                    if (xlist[i]["position"].getType() == XmlRpc::XmlRpcValue::TypeInt) {
                        int pos = xlist[i]["position"];
                        state.joint_state.position.push_back(double(pos));
                    }
                }
            }
        }
    }
    else {
        ROS_WARN("initial_configuration/joint_state is not on the param server.");
    }

    // multi_dof_joint_state
    if (nh.hasParam("initial_configuration/multi_dof_joint_state")) {
        nh.getParam("initial_configuration/multi_dof_joint_state", xlist);

        if (xlist.getType() == XmlRpc::XmlRpcValue::TypeArray) {
            if (xlist.size() != 0) {
                auto &multi_dof_joint_state = state.multi_dof_joint_state;
                multi_dof_joint_state.joint_names.resize(xlist.size());
                multi_dof_joint_state.transforms.resize(xlist.size());
                for (int i = 0; i < xlist.size(); ++i) {
                    multi_dof_joint_state.joint_names[i] = std::string(xlist[i]["joint_name"]);

                    Eigen::Quaterniond q;
                    sbpl::angles::from_euler_zyx(
                            (double)xlist[i]["yaw"], (double)xlist[i]["pitch"], (double)xlist[i]["roll"], q);

                    geometry_msgs::Quaternion orientation;
                    tf::quaternionEigenToMsg(q, orientation);

                    multi_dof_joint_state.transforms[i].translation.x = xlist[i]["x"];
                    multi_dof_joint_state.transforms[i].translation.y = xlist[i]["y"];
                    multi_dof_joint_state.transforms[i].translation.z = xlist[i]["z"];
                    multi_dof_joint_state.transforms[i].rotation.w = orientation.w;
                    multi_dof_joint_state.transforms[i].rotation.x = orientation.x;
                    multi_dof_joint_state.transforms[i].rotation.y = orientation.y;
                    multi_dof_joint_state.transforms[i].rotation.z = orientation.z;
                }
            } else {
                ROS_WARN("initial_configuration/multi_dof_joint_state array is empty");
            }
        } else {
            ROS_WARN("initial_configuration/multi_dof_joint_state is not an array.");
        }
    }

    ROS_INFO("Read initial state containing %zu joints and %zu multi-dof joints", state.joint_state.name.size(), state.multi_dof_joint_state.joint_names.size());
    return true;
}

struct RobotModelConfig
{
    std::string group_name;
    std::vector<std::string> planning_joints;
    std::string planning_link;
    std::string kinematics_frame;
    std::string chain_tip_link;
};

bool ReadRobotModelConfig(const ros::NodeHandle &nh, RobotModelConfig &config)
{
    if (!nh.getParam("group_name", config.group_name)) {
        ROS_ERROR("Failed to read 'group_name' from the param server");
        return false;
    }

    std::string planning_joint_list;
    if (!nh.getParam("planning_joints", planning_joint_list)) {
        ROS_ERROR("Failed to read 'planning_joints' from the param server");
        return false;
    }

    if (!nh.getParam("planning_link", config.planning_link)) {
        ROS_ERROR("Failed to read 'planning_link' from the param server");
        return false;
    }

    std::stringstream joint_name_stream(planning_joint_list);
    while (joint_name_stream.good() && !joint_name_stream.eof()) {
        std::string jname;
        joint_name_stream >> jname;
        if (jname.empty()) {
            continue;
        }
        config.planning_joints.push_back(jname);
    }

    // only required for generic kdl robot model?
    nh.getParam("kinematics_frame", config.kinematics_frame);
    nh.getParam("chain_tip_link", config.chain_tip_link);
    return true;
}

struct PlannerConfig
{
    std::string discretization;
    std::string mprim_filename;
    bool use_xyz_snap_mprim;
    bool use_rpy_snap_mprim;
    bool use_xyzrpy_snap_mprim;
    bool use_short_dist_mprims;
    double xyz_snap_dist_thresh;
    double rpy_snap_dist_thresh;
    double xyzrpy_snap_dist_thresh;
    double short_dist_mprims_thresh;
};

bool ReadPlannerConfig(const ros::NodeHandle &nh, PlannerConfig &config)
{
    if (!nh.getParam("discretization", config.discretization)) {
        ROS_ERROR("Failed to read 'discretization' from the param server");
        return false;
    }

    if (!nh.getParam("mprim_filename", config.mprim_filename)) {
        ROS_ERROR("Failed to read param 'mprim_filename' from the param server");
        return false;
    }

    if (!nh.getParam("use_xyz_snap_mprim", config.use_xyz_snap_mprim)) {
        ROS_ERROR("Failed to read param 'use_xyz_snap_mprim' from the param server");
        return false;
    }

    if (!nh.getParam("use_rpy_snap_mprim", config.use_rpy_snap_mprim)) {
        ROS_ERROR("Failed to read param 'use_rpy_snap_mprim' from the param server");
        return false;
    }

    if (!nh.getParam("use_xyzrpy_snap_mprim", config.use_xyzrpy_snap_mprim)) {
        ROS_ERROR("Failed to read param 'use_xyzrpy_snap_mprim' from the param server");
        return false;
    }

    if (!nh.getParam("use_short_dist_mprims", config.use_short_dist_mprims)) {
        ROS_ERROR("Failed to read param 'use_short_dist_mprims' from the param server");
        return false;
    }

    if (!nh.getParam("xyz_snap_dist_thresh", config.xyz_snap_dist_thresh)) {
        ROS_ERROR("Failed to read param 'xyz_snap_dist_thresh' from the param server");
        return false;
    }

    if (!nh.getParam("rpy_snap_dist_thresh", config.rpy_snap_dist_thresh)) {
        ROS_ERROR("Failed to read param 'rpy_snap_dist_thresh' from the param server");
        return false;
    }

    if (!nh.getParam("xyzrpy_snap_dist_thresh", config.xyzrpy_snap_dist_thresh)) {
        ROS_ERROR("Failed to read param 'xyzrpy_snap_dist_thresh' from the param server");
        return false;
    }

    if (!nh.getParam("short_dist_mprims_thresh", config.short_dist_mprims_thresh)) {
        ROS_ERROR("Failed to read param 'use_xyz_snap_mprim' from the param server");
        return false;
    }

    return true;
}

auto SetupRobotModel(const std::string& urdf, const RobotModelConfig &config)
    -> std::unique_ptr<smpl::KDLRobotModel>
{
    std::unique_ptr<smpl::KDLRobotModel> rm;

    if (config.kinematics_frame.empty() || config.chain_tip_link.empty()) {
        ROS_ERROR("Failed to retrieve param 'kinematics_frame' or 'chain_tip_link' from the param server");
        return rm;
    }

    ROS_INFO("Construct Generic KDL Robot Model");
    rm.reset(new sbpl::motion::KDLRobotModel);

    if (!rm->init(
            urdf,
            config.planning_joints,
            config.kinematics_frame,
            config.chain_tip_link))
    {
        ROS_ERROR("Failed to initialize robot model.");
        rm.reset();
        return std::move(rm);
    }

    if (!rm->setPlanningLink(config.planning_link)) {
        ROS_ERROR("Failed to set planning link to '%s'", config.planning_link.c_str());
        rm.reset();
        return std::move(rm);
    }

    return std::move(rm);
}

void initAllowedCollisionsPR2(sbpl::collision::CollisionSpace &cspace)
{
    sbpl::collision::AllowedCollisionMatrix acm;
    for (auto& pair : PR2AllowedCollisionPairs) {
        acm.setEntry(pair.first, pair.second, true);
    }
    cspace.setAllowedCollisionMatrix(acm);
}

auto ConstructStateSpace(
    const urdf::ModelInterface& urdf,
    const std::vector<std::string>& planning_joints)
    -> std::shared_ptr<ompl::base::StateSpace>
{
    auto space = std::make_shared<ompl::base::CompoundStateSpace>();

    for (auto& joint_name : planning_joints) {
        auto joint = urdf.getJoint(joint_name);
        switch (joint->type) {
        case urdf::Joint::UNKNOWN:
            return NULL;
        case urdf::Joint::FIXED:
            break;
        case urdf::Joint::PRISMATIC:
        case urdf::Joint::REVOLUTE:
        {
            auto subspace = std::make_shared<ompl::base::RealVectorStateSpace>(1);
            if (joint->safety) {
                subspace->setBounds(joint->safety->soft_lower_limit, joint->safety->soft_upper_limit);
            } else if (joint->limits) {
                subspace->setBounds(joint->limits->lower, joint->limits->upper);
            } else {
                subspace->setBounds(-1.0, 1.0);
            }
            space->addSubspace(subspace, 1.0);
            break;
        }
        case urdf::Joint::CONTINUOUS:
        {
            auto subspace = std::make_shared<ompl::base::SO2StateSpace>();
            space->addSubspace(subspace, 1.0);
            break;
        }
        case urdf::Joint::PLANAR:
        {
            auto subspace = std::make_shared<ompl::base::SE2StateSpace>();
            ompl::base::RealVectorBounds bounds(2);
            bounds.setLow(-1.0);
            bounds.setHigh(1.0);
            subspace->setBounds(bounds);
            space->addSubspace(subspace, 1.0);
            break;
        }
        case urdf::Joint::FLOATING:
        {
            auto subspace = std::make_shared<ompl::base::SE3StateSpace>();
            ompl::base::RealVectorBounds bounds(3);
            bounds.setLow(-1.0);
            bounds.setHigh(1.0);
            subspace->setBounds(bounds);
            space->addSubspace(subspace, 1.0);
            break;
        }
        default:
            ROS_WARN("Skip unrecognized joint type");
            break;
        }
    }
    return space;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "smpl_ompl_test");
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    sbpl::VisualizerROS visualizer(nh, 100);
    sbpl::viz::set_visualizer(&visualizer);

    // let publishers set up
    ros::Duration(1.0).sleep();

    /////////////////
    // Robot Model //
    /////////////////

    const char *robot_description_key = "robot_description";
    std::string robot_description_param;
    if (!nh.searchParam(robot_description_key, robot_description_param)) {
        ROS_ERROR("Failed to find 'robot_description' key on the param server");
        return 1;
    }

    std::string robot_description;
    if (!nh.getParam(robot_description_param, robot_description)) {
        ROS_ERROR("Failed to retrieve param 'robot_description' from the param server");
        return 1;
    }

    RobotModelConfig robot_config;
    if (!ReadRobotModelConfig(ros::NodeHandle("~robot_model"), robot_config)) {
        ROS_ERROR("Failed to read robot model config from param server");
        return 1;
    }

    auto rm = SetupRobotModel(robot_description, robot_config);
    if (!rm) {
        ROS_ERROR("Failed to set up Robot Model");
        return 1;
    }

    auto urdf = urdf::parseURDF(robot_description);
    if (!urdf) {
        ROS_ERROR("Failed to parse URDF");
        return 1;
    }
    auto state_space = ConstructStateSpace(*urdf, robot_config.planning_joints);

    ////////////////////
    // Occupancy Grid //
    ////////////////////

    const double df_size_x = 3.0;
    const double df_size_y = 3.0;
    const double df_size_z = 3.0;
    const double df_res = 0.02;
    const double df_origin_x = -0.75;
    const double df_origin_y = -1.5;
    const double df_origin_z = 0.0;
    const double max_distance = 1.8;

    typedef sbpl::EuclidDistanceMap DistanceMapType;

    ROS_INFO("Create distance map");
    auto df = std::make_shared<DistanceMapType>(
            df_origin_x, df_origin_y, df_origin_z,
            df_size_x, df_size_y, df_size_z,
            df_res,
            max_distance);

    ROS_INFO("Create grid");
    const bool ref_counted = false;
    sbpl::OccupancyGrid grid(df, ref_counted);

    // everyone needs to know the name of the planning frame for reasons...
    std::string planning_frame;
    if (!ph.getParam("planning_frame", planning_frame)) {
        ROS_ERROR("Failed to retrieve param 'planning_frame' from the param server");
        return 1;
    }

    ROS_INFO("Planning Frame: %s", planning_frame.c_str());

    grid.setReferenceFrame(planning_frame);
    SV_SHOW_INFO(grid.getBoundingBoxVisualization());

    ///////////////////////
    // Collision Checker //
    ///////////////////////

    // This whole manage storage for all the scene objects and must outlive
    // its associated CollisionSpace instance.
    CollisionSpaceScene scene;

    sbpl::collision::CollisionModelConfig cc_conf;
    if (!sbpl::collision::CollisionModelConfig::Load(ph, cc_conf)) {
        ROS_ERROR("Failed to load Collision Model Config");
        return 1;
    }

    sbpl::collision::CollisionSpace cc;
    if (!cc.init(
            &grid,
            robot_description,
            cc_conf,
            robot_config.group_name,
            robot_config.planning_joints))
    {
        ROS_ERROR("Failed to initialize Collision Space");
        return 1;
    }

    if (cc.robotCollisionModel()->name() == "pr2") {
        initAllowedCollisionsPR2(cc);
    }

    /////////////////
    // Scene Setup //
    /////////////////

    scene.SetCollisionSpace(&cc);

    std::string object_filename;
    ph.param<std::string>("object_filename", object_filename, "");

    // read in collision objects from file and add to the scene
    if (!object_filename.empty()) {
        auto objects = GetCollisionObjects(object_filename, planning_frame);
        for (auto& object : objects) {
            scene.ProcessCollisionObjectMsg(object);
        }
    }

    // read in start state from file and update the scene
    moveit_msgs::RobotState start_state;
    if (!ReadInitialConfiguration(ph, start_state)) {
        ROS_ERROR("Failed to get initial configuration.");
        return 1;
    }
    if (!scene.SetRobotState(start_state)) {
        ROS_ERROR("Failed to set start state on Collision Space Scene");
        return 1;
    }

    cc.setWorldToModelTransform(Eigen::Affine3d::Identity());

    SV_SHOW_INFO(grid.getDistanceFieldVisualization(0.2));

    // The KDL Robot Model must be given the transform from the planning frame
    // to the kinematics frame, which is assumed to not be a function of the
    // planning joint variables.
    geometry_msgs::Transform transform;
    transform.translation.x = -0.05;
    transform.translation.y = 0.0;
    transform.translation.z = 0.959;
    transform.rotation.w = 1.0;
    KDL::Frame f;
    tf::transformMsgToKDL(transform, f);
    rm->setKinematicsToPlanningTransform(f, "what?");

    SV_SHOW_INFO(cc.getCollisionRobotVisualization());
    SV_SHOW_INFO(cc.getCollisionWorldVisualization());
    SV_SHOW_INFO(cc.getOccupiedVoxelsVisualization());

    ///////////////////
    // Planner Setup //
    ///////////////////

    PlannerConfig planning_config;
    if (!ReadPlannerConfig(ros::NodeHandle("~planning"), planning_config)) {
        ROS_ERROR("Failed to read planner config");
        return 1;
    }

    ompl::geometric::SimpleSetup ss(state_space);
    ss.setStateValidityChecker([&](const ompl::base::State* state)
    {
        std::vector<double> values;
        state_space->copyToReals(values, state);
        return cc.isStateValid(values);
    });

    auto planner = std::make_shared<smpl::OMPLPlanner>(ss.getSpaceInformation(), &grid);

    smpl::PlanningParams params;
    params.planning_frame = planning_frame;
    planner->params().setParam("mprim_filename", planning_config.mprim_filename);
    planner->params().setParam("use_xyz_snap_mprim", planning_config.use_xyz_snap_mprim ? "true" : "false");
    planner->params().setParam("use_rpy_snap_mprim", planning_config.use_rpy_snap_mprim ? "true" : "false");
    planner->params().setParam("use_xyzrpy_snap_mprim", planning_config.use_xyzrpy_snap_mprim ? "true" : "false");
    planner->params().setParam("use_short_dist_mprims", planning_config.use_short_dist_mprims ? "true" : "false");
    planner->params().setParam("xyz_snap_dist_thresh", std::to_string(planning_config.xyz_snap_dist_thresh));
    planner->params().setParam("rpy_snap_dist_thresh", std::to_string(planning_config.rpy_snap_dist_thresh));
    planner->params().setParam("xyzrpy_snap_dist_thresh", std::to_string(planning_config.xyzrpy_snap_dist_thresh));
    planner->params().setParam("short_dist_mprims_thresh", std::to_string(planning_config.short_dist_mprims_thresh));
    planner->params().setParam("epsilon", "100.0");

    params.planning_link_sphere_radius = 0.02;

    params.addParam("discretization", planning_config.discretization);
    params.addParam("repair_time", 5.0);

    //////////////
    // Planning //
    //////////////

    std::vector<double> goal(6, 0);
    ph.param("goal/x", goal[0], 0.0);
    ph.param("goal/y", goal[1], 0.0);
    ph.param("goal/z", goal[2], 0.0);
    ph.param("goal/roll", goal[3], 0.0);
    ph.param("goal/pitch", goal[4], 0.0);
    ph.param("goal/yaw", goal[5], 0.0);

    moveit_msgs::MotionPlanRequest req;
    moveit_msgs::MotionPlanResponse res;

    req.allowed_planning_time = 60.0;
    req.goal_constraints.resize(1);
    FillGoalConstraint(goal, planning_frame, req.goal_constraints[0]);
    req.group_name = robot_config.group_name;
    req.max_acceleration_scaling_factor = 1.0;
    req.max_velocity_scaling_factor = 1.0;
    req.num_planning_attempts = 1;
//    req.path_constraints;
    req.planner_id = "arastar.bfs.manip";
    req.start_state = start_state;
//    req.trajectory_constraints;
//    req.workspace_parameters;

    // plan
    ROS_INFO("Calling solve...");
    auto solved = ss.solve(60.0);
    if (!solved) {
        ROS_ERROR("Failed to plan.");
        return 1;
    }

    ///////////////////////////////////
    // Visualizations and Statistics //
    ///////////////////////////////////

    // TODO: print statistics

    ROS_INFO("Animate path");

    size_t pidx = 0;
    while (ros::ok()) {
        auto& point = res.trajectory.joint_trajectory.points[pidx];
        auto markers = cc.getCollisionRobotVisualization(point.positions);
        for (auto& m : markers.markers) {
            m.ns = "path_animation";
        }
        SV_SHOW_INFO(markers);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        pidx++;
        pidx %= res.trajectory.joint_trajectory.points.size();
    }

    return 0;
}

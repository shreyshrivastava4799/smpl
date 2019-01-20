#include "test_scenario.h"

// standard includes
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

// system includes
#include <eigen_conversions/eigen_msg.h>
#include <smpl/distance_map/euclid_distance_map.h>
#include <smpl/stl/memory.h>
#include <smpl_urdf_robot_model/robot_state_visualization.h>
#include <smpl/defer.h>
#include <smpl/console/console.h>

// project includes
#include "pr2_allowed_collision_pairs.h"

// Search up the parameter hierarchy, find a key, and retrieve the parameter's
// value.
template <class T>
bool FindParam(const ros::NodeHandle& nh, const std::string& key, T& out)
{
    auto full_param = std::string();
    if (!nh.searchParam(key, full_param)) {
        SMPL_WARN("Failed to find '%s' key on the param server", key.c_str());
        return false;
    }

    if (!nh.getParam(full_param, out)) {
        SMPL_WARN("Failed to retrieve param '%s' from the param server", full_param.c_str());
        return false;
    }

    return true;
}

// Read the initial configuration of the robot from the param server.
static
bool ReadInitialConfiguration(
    const ros::NodeHandle& nh,
    moveit_msgs::RobotState& state)
{
    // joint_state
    XmlRpc::XmlRpcValue xlist;
    if (FindParam(nh, "initial_configuration/joint_state", xlist)) {
        if (xlist.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            SMPL_WARN("initial_configuration/joint_state is not an array.");
        }

        if (xlist.size() > 0) {
            for (int i = 0; i < xlist.size(); ++i) {
                state.joint_state.name.push_back(std::string(xlist[i]["name"]));

                if (xlist[i]["position"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                    state.joint_state.position.push_back(double(xlist[i]["position"]));
                } else {
                    SMPL_DEBUG("Doubles in the yaml file have to contain decimal points. (Convert '0' to '0.0')");
                    if (xlist[i]["position"].getType() == XmlRpc::XmlRpcValue::TypeInt) {
                        int pos = xlist[i]["position"];
                        state.joint_state.position.push_back(double(pos));
                    }
                }
            }
        }
    } else {
        SMPL_WARN("initial_configuration/joint_state is not on the param server.");
    }

    // multi_dof_joint_state
    if (FindParam(nh, "initial_configuration/multi_dof_joint_state", xlist)) {
        if (xlist.getType() == XmlRpc::XmlRpcValue::TypeArray) {
            if (xlist.size() != 0) {
                auto &multi_dof_joint_state = state.multi_dof_joint_state;
                multi_dof_joint_state.joint_names.resize(xlist.size());
                multi_dof_joint_state.transforms.resize(xlist.size());
                for (int i = 0; i < xlist.size(); ++i) {
                    multi_dof_joint_state.joint_names[i] = std::string(xlist[i]["joint_name"]);

                    Eigen::Quaterniond q;
                    smpl::angles::from_euler_zyx(
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
                SMPL_WARN("initial_configuration/multi_dof_joint_state array is empty");
            }
        } else {
            SMPL_WARN("initial_configuration/multi_dof_joint_state is not an array.");
        }
    }

    SMPL_INFO("Read initial state containing %zu joints and %zu multi-dof joints", state.joint_state.name.size(), state.multi_dof_joint_state.joint_names.size());
    return true;
}

static
auto MakeCollisionCube(
    const geometry_msgs::Pose& pose,
    std::vector<double>& dims,
    const std::string& frame_id,
    const std::string& id)
    -> moveit_msgs::CollisionObject
{
    auto object = moveit_msgs::CollisionObject();
    object.id = id;
    object.operation = moveit_msgs::CollisionObject::ADD;
    object.header.frame_id = frame_id;
    object.header.stamp = ros::Time::now();

    auto box_object = shape_msgs::SolidPrimitive();
    box_object.type = shape_msgs::SolidPrimitive::BOX;
    box_object.dimensions.resize(3);
    box_object.dimensions[0] = dims[0];
    box_object.dimensions[1] = dims[1];
    box_object.dimensions[2] = dims[2];

    object.primitives.push_back(box_object);
    object.primitive_poses.push_back(pose);
    return object;
}

// Read a list of collision objects from file.
static
auto GetCollisionObjects(
    const std::string& filename,
    const std::string& frame_id)
    -> std::vector<moveit_msgs::CollisionObject>
{
    char sTemp[1024];
    auto num_obs = 0;
    auto object_ids = std::vector<std::string>();
    auto objects = std::vector<std::vector<double>>();
    auto objs = std::vector<moveit_msgs::CollisionObject>();

    auto* fCfg = fopen(filename.c_str(), "r");

    if (fCfg == NULL) {
        SMPL_INFO("ERROR: unable to open objects file. Exiting.\n");
        return objs;
    }

    // get number of objects
    if (fscanf(fCfg,"%s",sTemp) < 1) {
        printf("Parsed string has length < 1.\n");
    }

    num_obs = atoi(sTemp);

    SMPL_INFO("%i objects in file",num_obs);

    //get {x y z dimx dimy dimz} for each object
    objects.resize(num_obs);
    object_ids.clear();
    for (int i = 0; i < num_obs; ++i) {
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

    auto dims = std::vector<double>(3, 0);
    auto pose = geometry_msgs::Pose();
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;

    if (object_ids.size() != objects.size()) {
        SMPL_INFO("object id list is not same length as object list. exiting.");
        return objs;
    }

    for (size_t i = 0; i < objects.size(); i++) {
        pose.position.x = objects[i][0];
        pose.position.y = objects[i][1];
        pose.position.z = objects[i][2];
        dims[0] = objects[i][3];
        dims[1] = objects[i][4];
        dims[2] = objects[i][5];

        objs.push_back(MakeCollisionCube(pose, dims, frame_id, object_ids.at(i)));
    }
    return objs;
}

// Search for the URDF on the param server using a common key.
static
auto GetRobotDescription(const ros::NodeHandle& nh) -> std::pair<std::string, bool>
{
    auto robot_description_key = "robot_description";
    auto robot_description = std::string();
    if (!FindParam(nh, robot_description_key, robot_description)) {
        return std::make_pair("", false);
    }
    return std::make_pair(std::move(robot_description), true);
}

static
auto MakeRobotModel(
    const std::string& type,
    const ros::NodeHandle& nh,
    const moveit_msgs::RobotState* start_state)
    -> std::unique_ptr<smpl::RobotModel>
{
    if (type == "kdl") {
        auto kdl_model = smpl::make_unique<smpl::KDLRobotModel>();

        auto robot_description = std::string();
        auto found = false;
        std::tie(robot_description, found) = GetRobotDescription(nh);
        if (!found) {
            return false;
        }

        auto kinematics_frame = std::string();
        auto chain_tip_link = std::string();

        if (!FindParam(nh, "robot_model/kinematics_frame", kinematics_frame)) return false;
        if (!FindParam(nh, "robot_model/chain_tip_link", chain_tip_link)) return false;

        SMPL_INFO("Construct KDL Robot Model");
        if (!InitKDLRobotModel(
                kdl_model.get(),
                robot_description,
                kinematics_frame,
                chain_tip_link))
        {
            SMPL_ERROR("Failed to initialize robot model");
            return false;
        }

        auto reference_state = smpl::urdf::RobotState();
        InitRobotState(&reference_state, &kdl_model->robot_model, start_state);
        SetReferenceState(kdl_model.get(), GetVariablePositions(&reference_state));
        return std::move(kdl_model);
    } else if (type == "pr2") {
        auto pr2_model = smpl::make_unique<smpl::PR2RobotModel>();

        auto robot_description = std::string();
        auto found = false;
        std::tie(robot_description, found) = GetRobotDescription(nh);
        if (!found) {
            return false;
        }

        auto kinematics_frame = std::string();
        auto chain_tip_link = std::string();

        if (!FindParam(nh, "robot_model/kinematics_frame", kinematics_frame)) return false;
        if (!FindParam(nh, "robot_model/chain_tip_link", chain_tip_link)) return false;

        SMPL_INFO("Construct KDL Robot Model");
        if (!InitPR2RobotModel(
                pr2_model.get(),
                robot_description,
                kinematics_frame,
                chain_tip_link))
        {
            SMPL_ERROR("Failed to initialize robot model");
            return false;
        }

        auto reference_state = smpl::urdf::RobotState();
        InitRobotState(&reference_state, &pr2_model->kdl_model.robot_model, start_state);
        SetReferenceState(pr2_model.get(), GetVariablePositions(&reference_state));
        return std::move(pr2_model);
    } else {
        return NULL;
    }
}

/////////////////////////
// Interface Functions //
/////////////////////////

// Initialize a scenario from the parameters on the param server.
bool InitTestScenario(
    TestScenario* scenario,
    const ros::NodeHandle& nh,
    const ros::NodeHandle& ph)
{
    smpl::viz::set_visualizer(&scenario->visualizer);
    ros::Duration(0.1).sleep(); // let the publisher setup

    ////////////////////
    // Load the scene //
    ////////////////////

    if (!ReadInitialConfiguration(ph, scenario->start_state)) {
        SMPL_ERROR("Failed to get initial configuration.");
        return false;
    }

    //////////////////////
    // Init Robot Model //
    //////////////////////

    auto robot_model_type = std::string();
    if (!FindParam(ph, "robot_model_type", robot_model_type)) {
        return false;
    }

    scenario->planning_model = MakeRobotModel(
            robot_model_type, ph, &scenario->start_state);
    if (scenario->planning_model == NULL) {
        SMPL_ERROR("Failed to create Robot Model");
        return false;
    }

    auto planning_frame = std::string();
    if (!FindParam(ph, "planning_frame", planning_frame)) {
        SMPL_ERROR("Failed to retrieve param 'planning_frame' from the param server");
        return false;
    }
    SMPL_INFO("planning frame = %s", planning_frame.c_str());

    ////////////////////
    // Load the Robot //
    ////////////////////

    // Robot description required to initialize collision checker and robot
    // model... TODO: we end up often reading this in twice, once to create the
    // collision model and again to create the planning model. Since we don't
    // know the planning model type, we don't know whether it requires the URDF
    // (the string, a ModelInterface, or the RobotCollisionModel). If we  defer
    // setting the planning group in the CollisionSpace, we can load the
    // ModelInterface, use it to initialize the CollisionSpace, the planning
    // model can use it, if possible, and we can then update the planning group
    // after the planning model is determined.
    auto robot_description = std::string();
    auto found = false;
    std::tie(robot_description, found) = GetRobotDescription(ph);
    if (!found) {
        return false;
    }

    ////////////////////////////////////////////////////////
    // Initialize the Collision Checker used for planning //
    ////////////////////////////////////////////////////////

    {
        SMPL_INFO("Initialize Occupancy Grid");

        auto df_size_x = 3.0;
        auto df_size_y = 3.0;
        auto df_size_z = 3.0;
        auto df_res = 0.02;
        auto df_origin_x = -0.75;
        auto df_origin_y = -1.5;
        auto df_origin_z = 0.0;
        auto max_distance = 1.8;

        using DistanceMapType = smpl::EuclidDistanceMap;

        auto df = std::make_shared<DistanceMapType>(
                df_origin_x, df_origin_y, df_origin_z,
                df_size_x, df_size_y, df_size_z,
                df_res,
                max_distance);

        auto ref_counted = false;
        scenario->grid = smpl::OccupancyGrid(std::move(df), ref_counted);

        scenario->grid.setReferenceFrame(planning_frame);
        SV_SHOW_INFO(scenario->grid.getBoundingBoxVisualization());
    }

    auto cc_conf = smpl::collision::CollisionModelConfig();
    if (!smpl::collision::CollisionModelConfig::Load(ph, cc_conf)) {
        SMPL_ERROR("Failed to load Collision Model Config");
        return false;
    }

    auto group_name = std::string();
    if (!FindParam(ph, "group_name", group_name)) {
        SMPL_ERROR("Failed to read 'group_name' from the param server");
        return false;
    }

    if (!scenario->collision_model.init(
            &scenario->grid,
            robot_description,
            cc_conf,
            group_name,
            scenario->planning_model->getPlanningJoints()))
    {
        SMPL_ERROR("Failed to initialize Collision Space");
        return false;
    }

    if (scenario->collision_model.robotCollisionModel()->name() == "pr2") {
        auto acm = smpl::collision::AllowedCollisionMatrix();
        for (auto& pair : PR2AllowedCollisionPairs) {
            acm.setEntry(pair.first, pair.second, true);
        }
        scenario->collision_model.setAllowedCollisionMatrix(acm);
    }

    // TODO: This retention is kinda stupid...
    scenario->scene.SetCollisionSpace(&scenario->collision_model);

    // TODO: ...and is why objects must be later added instead of after
    // CollisionSpaceScene initialization
    // Read in collision objects from file and add to the scene...
    auto object_filename = std::string();
    FindParam(ph, "object_filename", object_filename);

    if (!object_filename.empty()) {
        auto objects = GetCollisionObjects(object_filename, planning_frame);
        for (auto& object : objects) {
            scenario->scene.ProcessCollisionObjectMsg(object);
        }
    }

    // Set reference state in the collision model...
    // TODO: this retention is also stupid?
    if (!scenario->scene.SetRobotState(scenario->start_state)) {
        SMPL_ERROR("Failed to set start state on Collision Space Scene");
        return false;
    }

    scenario->collision_model.setWorldToModelTransform(Eigen::Affine3d::Identity());

    SV_SHOW_DEBUG_NAMED("distance_field", scenario->grid.getDistanceFieldVisualization(0.2));
    SV_SHOW_DEBUG(scenario->collision_model.getCollisionRobotVisualization());
    SV_SHOW_INFO(scenario->collision_model.getCollisionWorldVisualization());
    SV_SHOW_INFO(scenario->collision_model.getOccupiedVoxelsVisualization());
    return true;
}

// Initialize an smpl::urdf::RobotState and set its initial state from a
// moveit_msgs::RobotState.
static
bool InitRobotState(
    smpl::urdf::RobotState* robot_state,
    const smpl::urdf::RobotModel* robot_model,
    const moveit_msgs::RobotState* state_msg)
{
    if (!InitRobotState(robot_state, robot_model)) {
        SMPL_ERROR("Failed to initialize Robot State");
        return false;
    }
    for (auto i = 0; i < state_msg->joint_state.name.size(); ++i) {
        auto* var = GetVariable(robot_model, &state_msg->joint_state.name[i]);
        if (var == NULL) {
            SMPL_WARN("Variable '%s' not found in the Robot Model", state_msg->joint_state.name[i].c_str());
            return false;
        }
        SMPL_INFO("Set joint %s to %f", state_msg->joint_state.name[i].c_str(), state_msg->joint_state.position[i]);
        SetVariablePosition(robot_state, var, state_msg->joint_state.position[i]);
    }
    return true;
}

auto MakeRobotState(
    const moveit_msgs::RobotState* robot_state,
    const smpl::RobotModel* model)
    -> std::pair<smpl::RobotState, bool>
{
    auto state = smpl::RobotState();
    for (auto& var_name : model->getPlanningJoints()) {
        auto found = false;
        for (auto i = 0; i < robot_state->joint_state.name.size(); ++i) {
            if (robot_state->joint_state.name[i] == var_name) {
                state.push_back(robot_state->joint_state.position[i]);
                found = true;
                break;
            }
        }
        if (!found) {
            SMPL_ERROR("Joint variable '%s' was not found in robot state", var_name.c_str());
            return std::make_pair(smpl::RobotState(), false);
        }
    }

    return std::make_pair(state, true);
}

int WritePathCSV(
    const smpl::RobotModel* model,
    const std::vector<smpl::RobotState>* path,
    const char* filepath)
{
    auto* f = fopen(filepath, "w");
    if (f == NULL) return -1;
    DEFER(fclose(f));

    for (auto i = 0; i < model->getPlanningJoints().size(); ++i) {
        if (i != 0) {
            if (fputs(",", f) < 0) return -1;
        }
        auto& var = model->getPlanningJoints()[i];
        if (fputs(var.c_str(), f) < 0) return -1;
    }
    if (fputs("\n", f) < 0) return -1;
    for (auto& point : *path) {
        for (auto i = 0; i < point.size(); ++i) {
            if (i != 0) {
                if (fputs(",", f) < 0) return -1;
            }
            if (fprintf(f, "%f", point[i]) < 0) return -1;
        }
        if (fputs("\n", f) < 0) return -1;
    }

    return 0;
}

int AnimateSolution(
    TestScenario* scenario,
    smpl::RobotModel* planning_model,
    const std::vector<smpl::RobotState>* path)
{
    SMPL_INFO("Animate path");

    auto pidx = 0;
    while (ros::ok()) {
        auto& point = (*path)[pidx];
        auto id = (int32_t)0;
        auto robot_markers = planning_model->GetVisualization(point);
        for (auto& marker : robot_markers) {
            marker.ns = "path_animation";
            marker.frame_id = "map";
            marker.id = id++;
            marker.color = smpl::visual::Color{ 0.0f, 1.0f, 0.0f, 1.0f };
        }

#if 1
        auto collision_markers = scenario->collision_model.getCollisionRobotVisualization(point);
        for (auto& m : collision_markers.markers) {
            m.ns = "path_animation";
        }
#else
        auto collision_markers = visualization_msgs::MarkerArray();
#endif
        for (auto& marker : collision_markers.markers) {
            marker.ns = "path_animation";
            marker.header.frame_id = "map";
            marker.id = id++;
        }

        SV_SHOW_INFO(robot_markers);
        SV_SHOW_INFO(collision_markers);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        pidx++;
        pidx %= path->size();
    }

    return 0;
}


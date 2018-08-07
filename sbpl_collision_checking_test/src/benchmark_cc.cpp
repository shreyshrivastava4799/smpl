////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2016, Andrew Dornbush
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     1. Redistributions of source code must retain the above copyright notice
//        this list of conditions and the following disclaimer.
//     2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names of its
//        contributors may be used to endorse or promote products derived from
//        this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

/// \author Andrew Dornbush

// standard includes
#include <cmath>
#include <cstdlib>
#include <chrono>
#include <fstream>
#include <iostream>
#include <random>

// system includes
#include <ros/ros.h>
#include <smpl/distance_map/distance_map.h>
#include <smpl/distance_map/sparse_distance_map.h>
#include <smpl/distance_map/euclid_distance_map.h>
#include <smpl/occupancy_grid.h>
#include <sbpl_collision_checking/collision_space.h>
#include <urdf/model.h>

#include "pr2_allowed_collision_pairs.h"

auto CreateGrid(const ros::NodeHandle& nh, double max_dist)
    -> std::unique_ptr<smpl::OccupancyGrid>
{
    const char* world_collision_model_param = "world_collision_model";
    std::string wcm_key;
    if (!nh.searchParam(world_collision_model_param, wcm_key)) {
        ROS_ERROR("Failed to find 'world_collision_model' key on the param server");
        return nullptr;
    }

    XmlRpc::XmlRpcValue wcm_config;
    if (!nh.getParam(wcm_key, wcm_config)) {
        ROS_ERROR("Failed to retrieve '%s' from the param server", wcm_key.c_str());
        return nullptr;
    }

    if (wcm_config.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
        !wcm_config.hasMember("frame_id") ||
        !wcm_config.hasMember("size_x") ||
        !wcm_config.hasMember("size_y") ||
        !wcm_config.hasMember("size_z") ||
        !wcm_config.hasMember("origin_x") ||
        !wcm_config.hasMember("origin_y") ||
        !wcm_config.hasMember("origin_z") ||
        !wcm_config.hasMember("res_m") ||
        !wcm_config.hasMember("max_distance_m"))
    {
        ROS_ERROR("'%s' param is malformed", world_collision_model_param);
        ROS_ERROR_STREAM("has frame_id member " << wcm_config.hasMember("frame_id"));
        ROS_ERROR_STREAM("has size_x member " << wcm_config.hasMember("size_x"));
        ROS_ERROR_STREAM("has size_y member " << wcm_config.hasMember("size_y"));
        ROS_ERROR_STREAM("has size_z member " << wcm_config.hasMember("size_z"));
        ROS_ERROR_STREAM("has origin_x member " << wcm_config.hasMember("origin_x"));
        ROS_ERROR_STREAM("has origin_y member " << wcm_config.hasMember("origin_y"));
        ROS_ERROR_STREAM("has origin_z member " << wcm_config.hasMember("origin_z"));
        ROS_ERROR_STREAM("has res_m member " << wcm_config.hasMember("res_m"));
        ROS_ERROR_STREAM("has max_distance_m member " << wcm_config.hasMember("max_distance_m"));
        return nullptr;
    }

    const std::string world_frame = wcm_config["frame_id"];
    const double size_x = wcm_config["size_x"];
    const double size_y = wcm_config["size_y"];
    const double size_z = wcm_config["size_z"];
    const double res_m = wcm_config["res_m"];
    const double origin_x = wcm_config["origin_x"];
    const double origin_y = wcm_config["origin_y"];
    const double origin_z = wcm_config["origin_z"];
    const double cfg_max_dist = wcm_config["max_distance_m"];
    const double max_distance_m = std::max(max_dist + sqrt(3.0) * res_m, cfg_max_dist);
    const bool ref_counted = true;

    ROS_INFO("Occupancy Grid:");
    ROS_INFO("  origin: (%0.3f, %0.3f, %0.3f)", origin_x, origin_y, origin_z);
    ROS_INFO("  size: (%0.3f, %0.3f, %0.3f)", size_x, size_y, size_z);
    ROS_INFO("  res: %0.3f", res_m);
    ROS_INFO("  max_distance: %0.3f", max_distance_m);

    const int dflib = 2; // 0 -> my dense, 1 -> my sparse, 2 -> df

    std::unique_ptr<smpl::OccupancyGrid> grid;
    if (dflib == 0) {
        auto df = std::make_shared<smpl::EuclidDistanceMap>(
                origin_x, origin_y, origin_z,
                size_x, size_y, size_z,
                res_m,
                max_distance_m);

        grid = std::unique_ptr<smpl::OccupancyGrid>(
                new smpl::OccupancyGrid(df, ref_counted));

    } else if (dflib == 1){
        auto df = std::make_shared<smpl::SparseDistanceMap>(
                origin_x, origin_y, origin_z,
                size_x, size_y, size_z,
                res_m,
                max_distance_m);
        grid = std::unique_ptr<smpl::OccupancyGrid>(
                new smpl::OccupancyGrid(df, ref_counted));
    } else {
        grid = std::unique_ptr<smpl::OccupancyGrid>(
                new smpl::OccupancyGrid(
                        size_x, size_y, size_z,
                        res_m,
                        origin_x, origin_y, origin_z,
                        max_distance_m,
                        ref_counted));
    }

    grid->setReferenceFrame(world_frame);
    return grid;
}

class CollisionSpaceProfiler
{
public:

    bool init();

    struct ProfileResults
    {
        int check_count;
    };

    ProfileResults profileCollisionChecks(double time_limit);
    ProfileResults profileDistanceChecks(double time_limit);
    int exportCheckedStates(const char* filename, int count);
    int verifyCheckedStates(const char* filename);

private:

    ros::NodeHandle m_nh;
    std::unique_ptr<smpl::OccupancyGrid> m_grid;
    std::vector<std::string> m_planning_joints;
    smpl::collision::RobotCollisionModelPtr m_rcm;
    smpl::collision::CollisionSpace m_cspace;
    std::default_random_engine m_rng;
    ros::Publisher m_pub;

    std::vector<double> createRandomState();

    void initACM(smpl::collision::AllowedCollisionMatrix& acm);
};

bool CollisionSpaceProfiler::init()
{
    std::string robot_description_key;
    if (!m_nh.searchParam("robot_description", robot_description_key)) {
        ROS_ERROR("Failed to find 'robot_description' key on the param server");
        return false;
    }

    urdf::Model urdf;
    if (!urdf.initParam(robot_description_key)) {
        ROS_ERROR("Failed to initialize URDF from parameter '%s'", robot_description_key.c_str());
        return false;
    }

    if (urdf.getName() != "pr2") {
        ROS_ERROR("This benchmark is intended for the PR2 robot");
        return false;
    }

    smpl::collision::CollisionModelConfig config;
    if (!smpl::collision::CollisionModelConfig::Load(m_nh, config)) {
        ROS_ERROR("Failed to load collision model config");
        return false;
    }

    m_rcm = smpl::collision::RobotCollisionModel::Load(urdf, config);

    ROS_INFO("max leaf sphere radius: %0.3f", m_rcm->maxLeafSphereRadius());

    m_grid = CreateGrid(m_nh, m_rcm->maxSphereRadius());

    const std::string group_name = "right_arm";

    // hardcoded joint names corresponding to 'right_arm' joint group from SRDF
    m_planning_joints =
    {
        "r_shoulder_pan_joint",
        "r_shoulder_lift_joint",
        "r_upper_arm_roll_joint",
        "r_elbow_flex_joint",
        "r_forearm_roll_joint",
        "r_wrist_flex_joint",
        "r_wrist_roll_joint",
    };

    if (!m_cspace.init(m_grid.get(), m_rcm, group_name, m_planning_joints)) {
        ROS_ERROR("Failed to initialize Collision Space");
        return false;
    }

    smpl::collision::AllowedCollisionMatrix acm;
    initACM(acm);
    m_cspace.setAllowedCollisionMatrix(acm);

    m_pub = m_nh.advertise<visualization_msgs::MarkerArray>(
            "visualization_markers", 100);

    return true;
}

CollisionSpaceProfiler::ProfileResults
CollisionSpaceProfiler::profileCollisionChecks(double time_limit)
{
    ROS_INFO("Evaluating %0.3f seconds of collision checks", time_limit);

    ROS_INFO("Begin collision check benchmarking");

    int check_count = 0;
    double elapsed = 0.0;
    while (ros::ok() && elapsed < time_limit) {
        auto variables = createRandomState();
        auto start = std::chrono::high_resolution_clock::now();
        double dist;
        bool res = m_cspace.checkCollision(variables, dist);
        auto finish = std::chrono::high_resolution_clock::now();
        elapsed += std::chrono::duration<double>(finish - start).count();
        ++check_count;
    }

    ProfileResults res;
    res.check_count = check_count;
    return res;
}

CollisionSpaceProfiler::ProfileResults
CollisionSpaceProfiler::profileDistanceChecks(double time_limit)
{
    ROS_INFO("Evaluating %0.3f seconds of distance checks", time_limit);

    ROS_INFO("Begin distance check benchmarking");

    int check_count = 0;
    double elapsed = 0.0;
    while (ros::ok() && elapsed < time_limit) {
        auto variables = createRandomState();
        auto start = std::chrono::high_resolution_clock::now();
        double dist = m_cspace.collisionDistance(variables);
        auto finish = std::chrono::high_resolution_clock::now();
        elapsed += std::chrono::duration<double>(finish - start).count();
        ++check_count;
    }

    ProfileResults res;
    res.check_count = check_count;
    return res;
}

std::vector<double> CollisionSpaceProfiler::createRandomState()
{
    std::vector<double> out;
    out.reserve(m_planning_joints.size());
    for (const std::string& var_name : m_planning_joints) {
        if (m_rcm->jointVarIsContinuous(var_name)) {
            std::uniform_real_distribution<double> dist(-M_PI, M_PI);
            out.push_back(dist(m_rng));

        }
        else if (!m_rcm->jointVarHasPositionBounds(var_name)) {
            std::uniform_real_distribution<double> dist;
            out.push_back(dist(m_rng));
        }
        else {
            std::uniform_real_distribution<double> dist(
                    m_rcm->jointVarMinPosition(var_name),
                    m_rcm->jointVarMaxPosition(var_name));
            out.push_back(dist(m_rng));
        }
    }
    return out;
}

void CollisionSpaceProfiler::initACM(smpl::collision::AllowedCollisionMatrix& acm)
{
    for (auto& pair : PR2AllowedCollisionPairs) {
        acm.setEntry(pair.first, pair.second, true);
    }
}

int CollisionSpaceProfiler::exportCheckedStates(const char* filename, int count)
{
    std::ofstream ofs(filename);
    if (!ofs.is_open()) {
        return 1;
    }

    for (int i = 0; i < count && ros::ok(); ++i) {
        auto variables = createRandomState();
        double dist;
        bool res = m_cspace.checkCollision(variables, dist);
        m_pub.publish(m_cspace.getCollisionRobotVisualization(variables));
        for (auto v : variables) {
            ofs << std::setprecision(12) << v << ' ';
        }
        ofs << (int)res << '\n';
    }

    return 0;
}

int CollisionSpaceProfiler::verifyCheckedStates(const char* filename)
{
    std::ifstream ifs(filename);
    if (!ifs.is_open()) {
        ROS_ERROR("Failed to open %s", filename);
        return 1;
    }

    std::vector<double> vals(7);
    int ires;
    int line_count = 0;
    int diff = 0;
    int now_positive = 0;
    int now_negative = 0;
    while (ifs >> vals[0] >> vals[1] >> vals[2] >>vals[3] >> vals[4] >> vals[5] >> vals[6] >> ires) {
        ++line_count;
        double dist;
        bool res = m_cspace.checkCollision(vals, dist);
        if ((int)res != ires) {
            ROS_ERROR("Different result for line %d", line_count);
            ++diff;
            if (res) {
                ++now_positive;
            } else {
                ++now_negative;
            }
        }
    }

    if (diff) {
        ROS_ERROR("checks for %d configurations differ (%d positive, %d negative)", diff, now_positive, now_negative);
    }

    return diff;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "benchmark");
    ros::NodeHandle nh;

    if (argc < 2) {
        ROS_INFO("Usage: benchmark <command> <args...>");
        return 1;
    }

    const char* cmd = argv[1];

    CollisionSpaceProfiler prof;
    if (!prof.init()) {
        ROS_ERROR("Failed to initialize profiler");
        return 1;
    }

    if (0 == strcmp(cmd, "export")) {
        const char* filename = argc > 2 ? argv[2] : "checks.csv";
        int count = argc > 3 ? atoi(argv[3]) : 100000;
        return prof.exportCheckedStates(filename, count);
    }
    else if (0 == strcmp(cmd, "verify")) {
        if (argc < 3) {
            ROS_ERROR("Usage: benchmark verify <filename>");
            return 1;
        }
        const char* filename = argv[2];
        return prof.verifyCheckedStates(filename);
    }
    else if (0 == strcmp(cmd, "profile")) {
        const double time_limit = argc > 2 ? std::atof(argv[2]) : 10.0;
        if (time_limit == 0.0) {
            ROS_WARN("Did you make a mistake?");
        }

        {
            auto res = prof.profileCollisionChecks(time_limit);
            ROS_INFO("check count: %d", res.check_count);
            ROS_INFO("checks / second: %g", res.check_count / time_limit);
            ROS_INFO("seconds / check: %g", time_limit / res.check_count);
        }
        {
            auto res = prof.profileDistanceChecks(time_limit);
            ROS_INFO("check count: %d", res.check_count);
            ROS_INFO("checks / second: %g", res.check_count / time_limit);
            ROS_INFO("seconds / check: %g", time_limit / res.check_count);
        }
    } else if (0 == strcmp(cmd, "load")) {
        return 0;
    }

    return 0;
}

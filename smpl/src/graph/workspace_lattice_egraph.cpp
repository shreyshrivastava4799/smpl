#include <smpl/graph/workspace_lattice_egraph.h>

// standard includes
#include <fstream>

// system includes
#include <boost/filesystem.hpp>

// project includes
#include <smpl/csv_parser.h>
#include <smpl/console/console.h>

namespace sbpl {
namespace motion {

bool WorkspaceLatticeEGraph::extractPath(
    const std::vector<int>& ids,
    std::vector<RobotState>& path)
{
    return false;
}

bool ParseExperienceGraphFile(
    const std::string& filepath,
    RobotModel* robot_model,
    std::vector<RobotState>& egraph_states)
{
    std::ifstream fin(filepath);
    if (!fin.is_open()) {
        return false;
    }

    CSVParser parser;
    auto with_header = true;
    if (!parser.parseStream(fin, with_header)) {
        SMPL_ERROR("Failed to parse experience graph file '%s'", filepath.c_str());
        return false;
    }

    SMPL_INFO("Parsed experience graph file");
    SMPL_INFO("  Has Header: %s", parser.hasHeader() ? "true" : "false");
    SMPL_INFO("  %zu records", parser.recordCount());
    SMPL_INFO("  %zu fields", parser.fieldCount());

    auto jvar_count = robot_model->getPlanningJoints().size();
    if (parser.fieldCount() != jvar_count) {
        SMPL_ERROR("Parsed experience graph contains insufficient number of joint variables");
        return false;
    }

    egraph_states.reserve(parser.totalFieldCount());
    for (size_t i = 0; i < parser.recordCount(); ++i) {
        RobotState state(jvar_count);
        for (size_t j = 0; j < parser.fieldCount(); ++j) {
            try {
                state[j] = std::stod(parser.fieldAt(i, j));
            } catch (const std::invalid_argument& ex) {
                SMPL_ERROR("Failed to parse egraph state variable (%s)", ex.what());
                return false;
            } catch (const std::out_of_range& ex) {
                SMPL_ERROR("Failed to parse egraph state variable (%s)", ex.what());
                return false;
            }
        }
        egraph_states.push_back(std::move(state));
    }

    SMPL_INFO("Read %zu states from experience graph file", egraph_states.size());
    return true;
}

// Load the experience graph from a database of paths:
// 1. Convert the raw path to an ExperienceGraph, which captures the
// connectivity of the demonstration (some states are retained as unique nodes
// and others make up the local paths between nodes)
// 2. Reserve special states in the graph for each unique node in the
// ExperienceGraph.
// 3. Construct a mapping between unique experience graph nodes and states in
// the graph.
//
// ExperienceGraph to uniquely label nodes
bool WorkspaceLatticeEGraph::loadExperienceGraph(const std::string& path)
{
    boost::filesystem::path p(path);
    if (!boost::filesystem::is_directory(p)) {
        SMPL_ERROR("'%s' is not a directory", path.c_str());
        return false;
    }

    for (auto dit = boost::filesystem::directory_iterator(p);
        dit != boost::filesystem::directory_iterator(); ++dit)
    {
        auto& filepath = dit->path().generic_string();
        std::vector<RobotState> egraph_states;
        if (!ParseExperienceGraphFile(filepath, robot(), egraph_states)) {
            continue;
        }

        if (egraph_states.empty()) continue;

        SMPL_INFO("Create hash entries for experience graph states");

        auto& pp = egraph_states.front();
        WorkspaceCoord pdp(this->dofCount());
        this->stateRobotToCoord(pp, pdp);

        auto pid = this->egraph.insert_node(pp);
        this->coord_to_node[pdp].push_back(pid);

//        auto entry_id = this->reserveHashEntry();
    }

    return false;
}

void WorkspaceLatticeEGraph::getExperienceGraphNodes(
    int state_id,
    std::vector<ExperienceGraph::node_id>& nodes)
{
}

bool WorkspaceLatticeEGraph::shortcut(int src_id, int dst_id, int& cost)
{
    return false;
}

bool WorkspaceLatticeEGraph::snap(
    int first_id,
    int second_id,
    int& cost)
{
    return false;
}

auto WorkspaceLatticeEGraph::getExperienceGraph() const -> const ExperienceGraph*
{
    return NULL;
}

auto WorkspaceLatticeEGraph::getExperienceGraph() -> ExperienceGraph*
{
    return NULL;
}

int WorkspaceLatticeEGraph::getStateID(ExperienceGraph::node_id n) const
{
    return -1;
}

Extension* WorkspaceLatticeEGraph::getExtension(size_t class_code)
{
    return NULL;
}

} // namespace motion
} // namespace sbpl


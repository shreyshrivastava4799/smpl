#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <ompl/base/Planner.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/DiscreteStateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/spaces/TimeStateSpace.h>

#include <smpl/collision_checker.h>
#include <smpl/robot_model.h>
#include <smpl/console/console.h>
#include <smpl/console/nonstd.h>
#include <smpl/search/arastar.h>
#include <smpl/graph/manip_lattice.h>
#include <smpl/graph/manip_lattice_action_space.h>
#include <smpl/heuristic/joint_dist_heuristic.h>

namespace smpl {

template <class T, class... Args>
auto make_unique(Args&&... args) -> std::unique_ptr<T> {
    return std::unique_ptr<T>(new T(args...));
}

struct CollisionChecker : public sbpl::motion::CollisionChecker
{
    bool isStateValid(
        const sbpl::motion::RobotState& state,
        bool verbose = false) override
    {
        return true;
    }

    bool isStateToStateValid(
        const sbpl::motion::RobotState& start,
        const sbpl::motion::RobotState& finish,
        bool verbose = false) override
    {
        return true;
    }

    bool interpolatePath(
        const sbpl::motion::RobotState& start,
        const sbpl::motion::RobotState& finish,
        std::vector<sbpl::motion::RobotState>& path) override
    {
        path.push_back(start);
        path.push_back(finish);
        return true;
    }

    auto getExtension(size_t class_code) -> sbpl::motion::Extension* override {
        if (class_code == sbpl::motion::GetClassCode<sbpl::motion::CollisionChecker>()) {
            return this;
        }
        return NULL;
    }
};

struct RobotModel : public sbpl::motion::RobotModel
{
    ompl::base::SpaceInformation* si = NULL;

    struct VariableProperties
    {
        double min_position = 0.0;
        double max_position = 0.0;

        enum VariableFlags {
            CONTINUOUS = 1,
            BOUNDED = 2
        };
        uint8_t flags = 0;

        double max_velocity = 0.0;
        double max_acceleration = 0.0;
    };

    std::vector<VariableProperties> variables;

    double minPosLimit(int vidx) const override {
        return variables[vidx].min_position;
    }

    double maxPosLimit(int vidx) const override {
        return variables[vidx].max_position;
    }

    bool hasPosLimit(int vidx) const override {
        return (variables[vidx].flags & VariableProperties::BOUNDED) != 0;
    }

    bool isContinuous(int vidx) const override {
        return (variables[vidx].flags & VariableProperties::CONTINUOUS) != 0;
    }

    double velLimit(int vidx) const override {
        return variables[vidx].max_velocity;
    }

    double accLimit(int vidx) const override {
        return variables[vidx].max_acceleration;
    }

    bool checkJointLimits(const sbpl::motion::RobotState& state, bool verbose = false) override {
        return true;
    }

    auto getExtension(size_t class_code) -> sbpl::motion::Extension* override {
        if (class_code == sbpl::motion::GetClassCode<sbpl::motion::RobotModel>()) {
            return this;
        }
        return NULL;
    }
};

bool MakeVariableProperties(
    const ompl::base::StateSpace* space,
    std::vector<std::string>& names,
    std::vector<RobotModel::VariableProperties>& props)
{
    const ompl::base::CompoundStateSpace* compound_space;
    const ompl::base::DiscreteStateSpace* discrete_space;
    const ompl::base::RealVectorStateSpace* real_space;
    const ompl::base::SO2StateSpace* so2_space;
    const ompl::base::SO3StateSpace* so3_space;
    const ompl::base::TimeStateSpace* time_space;
    if ((compound_space = dynamic_cast<const ompl::base::CompoundStateSpace*>(space)) != NULL) {
        for (auto& ss : compound_space->getSubspaces()) {
            if (!MakeVariableProperties(ss.get(), names, props)) {
                return false;
            }
        }
    } else if ((discrete_space = dynamic_cast<const ompl::base::DiscreteStateSpace*>(space)) != NULL) {
        RobotModel::VariableProperties prop;
        prop.min_position = discrete_space->getLowerBound();
        prop.max_position = discrete_space->getUpperBound();
        prop.flags |= RobotModel::VariableProperties::BOUNDED;

        prop.max_velocity = std::numeric_limits<double>::quiet_NaN();
        prop.max_acceleration = std::numeric_limits<double>::quiet_NaN();

        names.push_back("discrete");
        props.push_back(prop);
    } else if ((real_space = dynamic_cast<const ompl::base::RealVectorStateSpace*>(space)) != NULL) {
        for (auto i = 0; i < real_space->getDimension(); ++i) {
            RobotModel::VariableProperties prop;
            prop.min_position = real_space->getBounds().low[i];
            prop.max_position = real_space->getBounds().high[i];
            prop.flags |= RobotModel::VariableProperties::BOUNDED;
            prop.max_velocity = std::numeric_limits<double>::quiet_NaN();
            prop.max_acceleration = std::numeric_limits<double>::quiet_NaN();

            if (real_space->getDimensionName(i).empty()) {
                names.push_back("real" + std::to_string(i));
            } else {
                names.push_back(real_space->getDimensionName(i));
            }
            props.push_back(prop);
        }
    } else if ((so2_space = dynamic_cast<const ompl::base::SO2StateSpace*>(space)) != NULL) {
        RobotModel::VariableProperties prop;
        prop.min_position = -std::numeric_limits<double>::infinity();
        prop.max_position = std::numeric_limits<double>::infinity();
        prop.flags |= RobotModel::VariableProperties::CONTINUOUS;
        prop.max_velocity = std::numeric_limits<double>::quiet_NaN();
        prop.max_acceleration = std::numeric_limits<double>::quiet_NaN();
        names.push_back("so2");
        props.push_back(prop);
    } else if ((so3_space = dynamic_cast<const ompl::base::SO3StateSpace*>(space)) != NULL) {
        RobotModel::VariableProperties prop;
        prop.min_position = -std::numeric_limits<double>::infinity();
        prop.max_position = std::numeric_limits<double>::infinity();
        prop.flags |= RobotModel::VariableProperties::CONTINUOUS;
        prop.max_velocity = std::numeric_limits<double>::quiet_NaN();
        prop.max_acceleration = std::numeric_limits<double>::quiet_NaN();
        names.push_back("so3_r");
        props.push_back(prop);

        names.push_back("so3_p");
        props.push_back(prop);

        names.push_back("so3_y");
        props.push_back(prop);
    } else {
        return false; // unsupported state space type
    }

    return true;
}

struct Planner : public ompl::base::Planner
{
    // world model interface
    RobotModel model;
    CollisionChecker checker;

    // params
    sbpl::motion::PlanningParams params;

    // graph
    sbpl::motion::ManipLattice space;
    sbpl::motion::ManipLatticeActionSpace actions;

    // heuristic
    sbpl::motion::JointDistHeuristic heuristic;

    // search
    sbpl::ARAStar search;

    Planner(const ompl::base::SpaceInformationPtr& si) :
        ompl::base::Planner(si, "smpl"),
        search(&space, &heuristic)
    {
        specs_.approximateSolutions = false;
        specs_.canReportIntermediateSolutions = true; //false;
        specs_.directed = true;
        specs_.multithreaded = false;
        specs_.optimizingPaths = true;
        specs_.provingSolutionNonExistence = true;

        // hmm...
        specs_.recognizedGoal = ompl::base::GoalType::GOAL_ANY;

        this->model.si = si.get();

        std::cout << si->getStateSpace()->getDimension() << '\n';

        std::map<std::string, std::string> params;
        si->getStateSpace()->params().getParams(params);

        si->getStateSpace()->computeLocations();
        auto& locations = si->getStateSpace()->getValueLocations();
        std::cout << locations.size() << " locations\n";
        for (auto& location : locations) {
            std::cout << "  index: " << location.index << '\n';
            std::cout << "  chain size: " << location.stateLocation.chain.size() << '\n';
            std::cout << "  chain: " << location.stateLocation.chain << '\n';
            std::cout << "  space dimension: " << location.stateLocation.space->getDimension() << '\n';
        }

        std::vector<int> signature;
        si->getStateSpace()->computeSignature(signature);
        std::cout << "signature: " << signature << '\n';

        std::cout << "name: " << si->getStateSpace()->getName() << '\n';
        std::cout << "measure: " << si->getStateSpace()->getMeasure() << '\n';
        std::cout << "maximum extent: " << si->getStateSpace()->getMaximumExtent() << '\n';

        {
            std::vector<std::string> names;
            std::vector<RobotModel::VariableProperties> props;
            if (!MakeVariableProperties(si->getStateSpace().get(), names, props)) {
                SMPL_ERROR("Failed to construct Planner!");
            }

            std::cout << "variable names: " << names << '\n';

            model.setPlanningJoints(names);
            model.variables = std::move(props);
        }

        std::cout << "Params:\n";
        for (auto& p : params) {
            std::cout << "    " << p.first << ": " << p.second << '\n';
        }

        std::cout << "Projections:\n";
        for (auto& proj : si->getStateSpace()->getRegisteredProjections()) {
            std::cout << "  " << proj.first << '\n';
        }

#if 0
        if (!this->actions.init(&this->space)) {
            SMPL_ERROR("Failed to initialize Manip Lattice Action Space");
        }

        std::vector<double> resolutions;
        if (!this->space.init(&this->model, &this->checker, &this->params, resolutions, &this->actions)) {
            SMPL_ERROR("Failed to initialize manip lattice");
        }
#endif

        {
            auto setter = [&](double val) {
                SMPL_INFO("Set epsilon to %f", val);
                this->search.set_initialsolution_eps(val);
            };

            auto getter = [&]() {
                return this->search.get_initial_eps();
            };

            this->ompl::base::Planner::params().declareParam<double>("epsilon", setter, getter);

//            this->ompl::base::Planner::params().getParam();

//            this->ompl::base::Planner::params().remove("epsilon");
        }
    }

    auto solve(const ompl::base::PlannerTerminationCondition& ptc)
        -> ompl::base::PlannerStatus override
    {
        // TODO: add user-defined termination condition to ARA*/other planners

        // TODO: when a solution path is found, save it to the instance of
        // ProblemDefinition using its addSolutionPath member.
//        this->getProblemDefinition()->addSolutionPath();

        // relevant planner status messages:
        //  ompl::base::PlannerStatus::INVALID_START
        //  ompl::base::PlannerStatus::INVALID_GOAL
        //  ompl::base::PlannerStatus::UNRECOGNIZED_GOAL_TIME
        //  ompl::base::PlannerStatus::TIMEOUT
        //  ompl::base::PlannerStatus::EXACT_SOLUTION
        //  ompl::base::PlannerStatus::ABORT
        return ompl::base::PlannerStatus(ompl::base::PlannerStatus::TIMEOUT);
    }

    void clear() override
    {
        ompl::base::Planner::clear();

        SMPL_INFO("TODO: smpl::Planner::clear");
        // NOTE: can set this->setup_ to false to communicate that setup should
        // be called again

//        ompl::base::SelfConfig config;
    }

    void setup() override
    {
        ompl::base::Planner::setup();

        SMPL_INFO("TODO: smpl::Planner::setup");
    }

    void getPlannerData(ompl::base::PlannerData& data) const override
    {
        ompl::base::Planner::getPlannerData(data);

        SMPL_INFO("TODO: smpl::Planner::getPlannerData");
    }
};

} // namespace smpl

bool isStateValid(const ompl::base::State* state)
{
    auto* s = state->as<ompl::base::SE3StateSpace::StateType>();
    auto dx = s->getX();
    auto dy = s->getY();
    auto dz = s->getZ();
    return (dx * dx + dy * dy + dz * dz > 0.5 * 0.5);
}

namespace spns = boost;

int main(int argc, char* argv[])
{
    auto space = spns::make_shared<ompl::base::SE3StateSpace>();

    ompl::base::RealVectorBounds bounds(3);
    bounds.setLow(-1);
    bounds.setHigh(1);

    space->setBounds(bounds);

//    auto si = spns::make_shared<ompl::base::SpaceInformation>(space);

    ompl::geometric::SimpleSetup ss(space);
    ss.setStateValidityChecker([](const ompl::base::State* state) {
            return isStateValid(state);
    });

    ompl::base::ScopedState<> start(space);
    start.random();

    ompl::base::ScopedState<> goal(space);
    goal.random();

    ss.setStartAndGoalStates(start, goal);

    auto planner = spns::make_shared<smpl::Planner>(ss.getSpaceInformation());
    planner->ompl::base::Planner::params().setParam("epsilon", "100.0");

    ss.setPlanner(planner);

    auto solved = ss.solve(1.0);

    if (solved) {
        std::cout << "Found solution:" << std::endl;
        ss.simplifySolution();
        ss.getSolutionPath().print(std::cout);
    }

    return 0;
}

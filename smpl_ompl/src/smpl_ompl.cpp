#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <ompl/base/Planner.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/DiscreteStateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/spaces/TimeStateSpace.h>

#include <ompl/base/goals/GoalLazySamples.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/goals/GoalRegion.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/goals/GoalSampleableRegion.h>

#include <smpl/angles.h>
#include <smpl/collision_checker.h>
#include <smpl/robot_model.h>
#include <smpl/console/console.h>
#include <smpl/console/nonstd.h>
#include <smpl/search/arastar.h>
#include <smpl/graph/manip_lattice.h>
#include <smpl/graph/manip_lattice_action_space.h>
#include <smpl/heuristic/joint_dist_heuristic.h>
#include <smpl/console/console.h>

// :/ - would like to eventually rename sbpl::motion to smpl and scope this appropriately
namespace smpl {

// stolen defer from scdl
template <class Callable>
struct CallOnDestruct
{
    Callable c;
    CallOnDestruct(Callable c) : c(c) { }
    ~CallOnDestruct() { c(); }
};

template <class Callable>
CallOnDestruct<Callable> MakeCallOnDestruct(Callable c) {
    return CallOnDestruct<Callable>(c);
}

// preprocessor magic to get a prefix to concatenate with the __LINE__ macro
#define MAKE_LINE_IDENT_JOIN_(a, b) a##b
#define MAKE_LINE_IDENT_JOIN(a, b) MAKE_LINE_IDENT_JOIN_(a, b)
#define MAKE_LINE_IDENT(prefix) MAKE_LINE_IDENT_JOIN(prefix, __LINE__)

// create an obscurely named CallOnDestruct with an anonymous lambda that
// executes the given statement sequence
#define DEFER(fun) auto MAKE_LINE_IDENT(tmp_call_on_destruct_) = ::smpl::MakeCallOnDestruct([&](){ fun; })

template <class T, class... Args>
auto make_unique(Args&&... args) -> std::unique_ptr<T> {
    return std::unique_ptr<T>(new T(args...));
}

struct RobotModel : public sbpl::motion::RobotModel
{
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
    ompl::base::SpaceInformation* si = NULL;

    double minPosLimit(int vidx) const override;
    double maxPosLimit(int vidx) const override;
    bool hasPosLimit(int vidx) const override;
    bool isContinuous(int vidx) const override;
    double velLimit(int vidx) const override;
    double accLimit(int vidx) const override;
    bool checkJointLimits(const sbpl::motion::RobotState& state, bool verbose = false) override;
    auto getExtension(size_t class_code) -> sbpl::motion::Extension* override;
};

struct CollisionChecker : public sbpl::motion::CollisionChecker
{
    ompl::base::StateSpace* space;
    ompl::base::StateValidityChecker* checker;
    ompl::base::MotionValidator* validator;

    bool isStateValid(
        const sbpl::motion::RobotState& state,
        bool verbose = false) override;

    bool isStateToStateValid(
        const sbpl::motion::RobotState& start,
        const sbpl::motion::RobotState& finish,
        bool verbose = false) override;

    bool interpolatePath(
        const sbpl::motion::RobotState& start,
        const sbpl::motion::RobotState& finish,
        std::vector<sbpl::motion::RobotState>& path) override;

    auto getExtension(size_t class_code) -> sbpl::motion::Extension* override;
};

enum struct ConcreteSpaceType
{
    C_FOREST_STATE_SPACE_WRAPPER,
    MORSE_STATE_SPACE_TYPE,
};

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

    Planner(const ompl::base::SpaceInformationPtr& si);

    void setProblemDefinition(const ompl::base::ProblemDefinitionPtr& pdef) override;

    auto solve(const ompl::base::PlannerTerminationCondition& ptc)
        -> ompl::base::PlannerStatus override;

    void clear() override;

    void checkValidity() override;

    void setup() override;

    void getPlannerData(ompl::base::PlannerData& data) const override;
};

static
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

static
auto MakeStateSMPL(
    const ompl::base::StateSpace* space,
    const ompl::base::State* state)
    -> sbpl::motion::RobotState
{
    sbpl::motion::RobotState s;
    space->copyToReals(s, state);
    return s;
};

static
auto MakeStateOMPL(
    const ompl::base::StateSpace* space,
    const sbpl::motion::RobotState& state)
    -> ompl::base::State*
{
    auto* s = space->allocState();
    space->copyFromReals(s, state);
    return s;
}

///////////////////////////////
// RobotModel Implementation //
///////////////////////////////

double RobotModel::minPosLimit(int vidx) const
{
    return variables[vidx].min_position;
}

double RobotModel::maxPosLimit(int vidx) const
{
    return variables[vidx].max_position;
}

bool RobotModel::hasPosLimit(int vidx) const
{
    return (variables[vidx].flags & VariableProperties::BOUNDED) != 0;
}

bool RobotModel::isContinuous(int vidx) const
{
    return (variables[vidx].flags & VariableProperties::CONTINUOUS) != 0;
}

double RobotModel::velLimit(int vidx) const
{
    return variables[vidx].max_velocity;
}

double RobotModel::accLimit(int vidx) const
{
    return variables[vidx].max_acceleration;
}

bool RobotModel::checkJointLimits(const sbpl::motion::RobotState& state, bool verbose)
{
    // TODO: extract these from SpaceInformation
    return true;
}

auto RobotModel::getExtension(size_t class_code) -> sbpl::motion::Extension*
{
    if (class_code == sbpl::motion::GetClassCode<sbpl::motion::RobotModel>()) {
        return this;
    }
    return NULL;
}

/////////////////////////////////////
// CollisionChecker Implementation //
/////////////////////////////////////

bool CollisionChecker::isStateValid(
    const sbpl::motion::RobotState& state,
    bool verbose)
{
    auto* s = MakeStateOMPL(this->space, state);
    DEFER(this->space->freeState(s));
    return this->checker->isValid(s);
}

bool CollisionChecker::isStateToStateValid(
    const sbpl::motion::RobotState& start,
    const sbpl::motion::RobotState& finish,
    bool verbose)
{
    auto* s = MakeStateOMPL(this->space, start);
    auto* f = MakeStateOMPL(this->space, finish);
    DEFER(this->space->freeState(s));
    DEFER(this->space->freeState(f));
    return this->validator->checkMotion(s, f);
}

bool CollisionChecker::interpolatePath(
    const sbpl::motion::RobotState& start,
    const sbpl::motion::RobotState& finish,
    std::vector<sbpl::motion::RobotState>& path)
{
    // TODO: only used practically for shortcutting...we'll let ompl handle
    // path simplification and not think too hard about that here
    path.push_back(start);
    path.push_back(finish);
    return true;
}

auto CollisionChecker::getExtension(size_t class_code)
    -> sbpl::motion::Extension*
{
    if (class_code == sbpl::motion::GetClassCode<sbpl::motion::CollisionChecker>()) {
        return this;
    }
    return NULL;
}

////////////////////////////
// Planner Implementation //
////////////////////////////

Planner::Planner(const ompl::base::SpaceInformationPtr& si) :
    ompl::base::Planner(si, "smpl"),
    search(&space, &heuristic)
{
    SMPL_INFO("Construct Planner");

    specs_.approximateSolutions = false;
    specs_.canReportIntermediateSolutions = true; //false;
    specs_.directed = true;
    specs_.multithreaded = false;
    specs_.optimizingPaths = true;
    specs_.provingSolutionNonExistence = true;

    // hmm...
    specs_.recognizedGoal = ompl::base::GoalType::GOAL_ANY;

    ////////////////////////////////
    // Log state space properties //
    ////////////////////////////////

    SMPL_INFO("dimensions = %u", si->getStateSpace()->getDimension());

    std::map<std::string, std::string> params;
    si->getStateSpace()->params().getParams(params);

    si->getStateSpace()->computeLocations();
    auto& locations = si->getStateSpace()->getValueLocations();
    SMPL_INFO("locations = %zu", locations.size());
    for (auto& location : locations) {
        SMPL_INFO("  index = %zu", location.index);
        SMPL_INFO("  chain size = %zu", location.stateLocation.chain.size());
        SMPL_INFO_STREAM("  chain: " << location.stateLocation.chain);
        SMPL_INFO("  dimension = %u", location.stateLocation.space->getDimension());
    }

    std::vector<int> signature;
    si->getStateSpace()->computeSignature(signature);
    SMPL_INFO_STREAM("signature = " << signature);

    SMPL_INFO("name = %s", si->getStateSpace()->getName().c_str());
    SMPL_INFO("measure = %f", si->getStateSpace()->getMeasure());
    SMPL_INFO("maximum extent = %f", si->getStateSpace()->getMaximumExtent());

    SMPL_INFO("Params:");
    for (auto& p : params) {
        SMPL_INFO("  %s: %s", p.first.c_str(), p.second.c_str());
    }

    SMPL_INFO("Projections:");
    for (auto& proj : si->getStateSpace()->getRegisteredProjections()) {
        SMPL_INFO("  %s", proj.first.c_str());
    }

    ///////////////////////////
    // Initialize RobotModel //
    ///////////////////////////

    this->model.si = si.get();

    {
        std::vector<std::string> names;
        std::vector<RobotModel::VariableProperties> props;
        if (!MakeVariableProperties(si->getStateSpace().get(), names, props)) {
            SMPL_WARN("Failed to construct Planner!");
            return;
        }

        SMPL_INFO_STREAM("variable names = " << names);

        model.setPlanningJoints(names);
        model.variables = std::move(props);
    }

    ////////////////////////////////////////////
    // Initialize Collision Checker Interface //
    ////////////////////////////////////////////

    this->checker.space = this->getSpaceInformation()->getStateSpace().get();
    this->checker.checker = this->getSpaceInformation()->getStateValidityChecker().get();
    this->checker.validator = this->getSpaceInformation()->getMotionValidator().get();

    //////////////////////////////
    // Initialize Manip Lattice //
    //////////////////////////////

    auto res = 0.05;

    std::vector<double> resolutions;
    resolutions.resize(this->model.getPlanningJoints().size(), res);
    if (!this->space.init(
            &this->model,
            &this->checker,
            &this->params,
            resolutions,
            &this->actions))
    {
        SMPL_WARN("Failed to initialize manip lattice");
        return;
    }

    if (!this->actions.init(&this->space)) {
        SMPL_WARN("Failed to initialize Manip Lattice Action Space");
        return;
    }

    for (int i = 0; i < (int)this->model.getPlanningJoints().size(); ++i) {
        std::vector<double> mprim(this->model.getPlanningJoints().size(), 0.0);
        mprim[i] = res;
        this->actions.addMotionPrim(mprim, false);
    }

#if 0
    SMPL_INFO("Action Set:");
    for (auto ait = this->actions.begin(); ait != this->actions.end(); ++ait) {
        SMPL_INFO("  type: %s", to_cstring(ait->type));
        if (ait->type == sbpl::motion::MotionPrimitive::SNAP_TO_RPY) {
            SMPL_INFO("    enabled: %s", this->actions.useAmp(sbpl::motion::MotionPrimitive::SNAP_TO_RPY) ? "true" : "false");
            SMPL_INFO("    thresh: %0.3f", this->actions.ampThresh(sbpl::motion::MotionPrimitive::SNAP_TO_RPY));
        } else if (ait->type == sbpl::motion::MotionPrimitive::SNAP_TO_XYZ) {
            SMPL_INFO("    enabled: %s", this->actions.useAmp(sbpl::motion::MotionPrimitive::SNAP_TO_XYZ) ? "true" : "false");
            SMPL_INFO("    thresh: %0.3f", this->actions.ampThresh(sbpl::motion::MotionPrimitive::SNAP_TO_XYZ));
        } else if (ait->type == sbpl::motion::MotionPrimitive::SNAP_TO_XYZ_RPY) {
            SMPL_INFO("    enabled: %s", this->actions.useAmp(sbpl::motion::MotionPrimitive::SNAP_TO_XYZ_RPY) ? "true" : "false");
            SMPL_INFO("    thresh: %0.3f", this->actions.ampThresh(sbpl::motion::MotionPrimitive::SNAP_TO_XYZ_RPY));
        } else if (ait->type == sbpl::motion::MotionPrimitive::LONG_DISTANCE ||
            ait->type == sbpl::motion::MotionPrimitive::SHORT_DISTANCE)
        {
            SMPL_INFO_STREAM("    action: " << ait->action);
        }
    }
#endif

    //////////////////////////
    // Initialize Heuristic //
    //////////////////////////

    if (!heuristic.init(&this->space)) {
        SMPL_WARN("Failed to initialize heuristic");
        return;
    }

    ///////////////////////////
    // Initialize the Search //
    ///////////////////////////

    ////////////////////////
    // Declare Parameters //
    ////////////////////////

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

        // TODO: other parameters here
        // ...search timing parameters
        // ...search optimization parameters
        // ...distance-dependent action space
        // ...state space resolutions here?
    }
}

bool IsAnyGoal(void* user, const sbpl::motion::RobotState& state)
{
    auto* planner = static_cast<Planner*>(user);
    auto* space = planner->getSpaceInformation()->getStateSpace().get();
    return planner->getProblemDefinition()->getGoal()->isSatisfied(
            MakeStateOMPL(space, state));
}

auto Planner::solve(const ompl::base::PlannerTerminationCondition& ptc)
    -> ompl::base::PlannerStatus
{
    SMPL_INFO("Planner::solve");

    auto* si = this->getSpaceInformation().get();
    auto* ss = si->getStateSpace().get();
    auto* pdef = this->getProblemDefinition().get();

    if (pdef->getSpaceInformation().get() != si) {
        SMPL_ERROR("wtf");
    }

    /////////////////////////
    // Set the start state //
    /////////////////////////

    {
        if (pdef->getStartStateCount() > 1) {
            SMPL_WARN("Received multiple start states. Why?");
            return ompl::base::PlannerStatus(ompl::base::PlannerStatus::INVALID_START);
        }

        auto* start = pdef->getStartState(0);
        auto start_state = MakeStateSMPL(ss, start);
        SMPL_INFO_STREAM("start state = " << start_state);
        if (!this->space.setStart(start_state)) {
            SMPL_WARN("Failed to set start state");
            return ompl::base::PlannerStatus(ompl::base::PlannerStatus::INVALID_START);
        }
    }

    ////////////////////////
    // Set the goal state //
    ////////////////////////

    {
        sbpl::motion::GoalConstraint goal_condition;

        // Inheritance hierarchy for goal types:
        // GoalAny
        //      GoalRegion
        //          GoalSampleableRegion
        //              GoalState
        //              GoalStates
        //                  GoalLazySamples

        auto& abstract_goal = pdef->getGoal();
        switch (abstract_goal->getType()) {
        case ompl::base::GoalType::GOAL_ANY:
        case ompl::base::GoalType::GOAL_REGION:
        case ompl::base::GoalType::GOAL_SAMPLEABLE_REGION:
        case ompl::base::GoalType::GOAL_STATES:
        case ompl::base::GoalType::GOAL_LAZY_SAMPLES:
        {
            auto* goal = static_cast<ompl::base::Goal*>(abstract_goal.get());
            goal_condition.type = sbpl::motion::GoalType::USER_GOAL_CONSTRAINT_FN;
            goal_condition.check_goal = IsAnyGoal;
            goal_condition.check_goal_user = this;
            break;
        }
        case ompl::base::GoalType::GOAL_STATE:
        {
            auto* goal = static_cast<ompl::base::GoalState*>(abstract_goal.get());
            auto goal_state = MakeStateSMPL(ss, goal->getState());
            SMPL_INFO_STREAM("goal state = " << goal_state);
            goal_condition.type = sbpl::motion::GoalType::JOINT_STATE_GOAL;
            goal_condition.angles = goal_state;
            // UGH
            goal_condition.angle_tolerances.resize(
                    goal_condition.angles.size(),
                    0.5 * this->space.resolutions().front());
            break;
        }
        default:
            SMPL_WARN("Unrecognized OMPL goal type");
            break;
        }

        if (!space.setGoal(goal_condition)) {
            SMPL_WARN("Failed to set goal");
            return ompl::base::PlannerStatus(ompl::base::PlannerStatus::INVALID_GOAL);
        }
    }

    //////////////////
    // Do the thing //
    //////////////////

    // TODO: hmmm, is this needed? this should probably be part of clear()
    // and allow the state of the search to persist between calls
    this->search.force_planning_from_scratch();

    sbpl::ARAStar::TimeParameters time_params;
    time_params.bounded = true;
    time_params.improve = true;
    time_params.type = sbpl::ARAStar::TimeParameters::USER;
    time_params.timed_out_fun = [&]() { return ptc.eval(); };

    auto start_id = space.getStartStateID();
    auto goal_id = space.getGoalStateID();
    this->search.set_start(start_id);
    this->search.set_goal(goal_id);

    std::vector<int> solution;
    int cost;
    auto res = this->search.replan(time_params, &solution, &cost);

    if (!res) {
        SMPL_WARN("Failed to find solution");
        return ompl::base::PlannerStatus(ompl::base::PlannerStatus::TIMEOUT);
    }

    SMPL_INFO("Expands: %d", this->search.get_n_expands());
    SMPL_INFO("Expands (Init): %d", this->search.get_n_expands_init_solution());
    SMPL_INFO("Epsilon: %f", this->search.get_final_epsilon());
    SMPL_INFO("Epsilon (Init): %f", this->search.get_initial_eps());

#if 0
    // TODO: hidden ARA*-specific return codes
    switch (res) {
    case 0: // SUCCESS
        SMPL_INFO("Planner found optimal solution");
        return ompl::base::PlannerStatus(ompl::base::PlannerStatus::EXACT_SOLUTION);
        break;
    case 4: // TIMED_OUT
        SMPL_INFO("Planner timed out");
        return ompl::base::PlannerStatus(ompl::base::PlannerStatus::TIMEOUT);
        break;
    case 5: // EXHAUSTED_OPEN_LIST
        SMPL_INFO("Planner returned with an exact solution");
        return ompl::base::PlannerStatus(ompl::base::PlannerStatus::EXACT_SOLUTION);
        break;
    }
#endif

    //////////////////////////////////////////////////////////
    // Convert discrete state path to continuous state path //
    //////////////////////////////////////////////////////////

    std::vector<sbpl::motion::RobotState> path;
    if (!space.extractPath(solution, path)) {
        return ompl::base::PlannerStatus::CRASH;
    }

    auto ompl_path = boost::make_shared<ompl::geometric::PathGeometric>(
            this->getSpaceInformation());

    // TODO: convert path
//    ompl_path->append();
    this->getProblemDefinition()->addSolutionPath(ompl_path);

    return ompl::base::PlannerStatus(ompl::base::PlannerStatus::EXACT_SOLUTION);
}

void Planner::setProblemDefinition(const ompl::base::ProblemDefinitionPtr& pdef)
{
    SMPL_INFO("Planner::setProblemDefinition");
    ompl::base::Planner::setProblemDefinition(pdef);
}

void Planner::clear()
{
    SMPL_INFO("TODO: Planner::clear");
    ompl::base::Planner::clear();

    // NOTE: can set this->setup_ to false to communicate that setup should
    // be called again
}

void Planner::setup()
{
    SMPL_INFO("Planner::setup");
    ompl::base::Planner::setup();
}

void Planner::checkValidity()
{
    SMPL_INFO("Planner::checkValidity");
    ompl::base::Planner::checkValidity(); // lol, throws exceptions
}

void Planner::getPlannerData(ompl::base::PlannerData& data) const
{
    SMPL_INFO("TODO: Planner::getPlannerData");
    ompl::base::Planner::getPlannerData(data);
}

} // namespace smpl

using StateSpaceType = ompl::base::SE2StateSpace;
bool isStateValid(const ompl::base::State* state)
{
    auto* s = state->as<StateSpaceType::StateType>();
    auto dx = s->getX();
    auto dy = s->getY();
    return (dx * dx + dy * dy > 0.5 * 0.5);
}

namespace spns = boost;

int main(int argc, char* argv[])
{
    auto space = spns::make_shared<StateSpaceType>();

    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(-1);
    bounds.setHigh(1);

    space->setBounds(bounds);

    ompl::geometric::SimpleSetup ss(space);
    ss.setStateValidityChecker(
            [](const ompl::base::State* state) {
                return isStateValid(state);
            });

    ompl::base::ScopedState<StateSpaceType> start(space);
    ompl::base::ScopedState<StateSpaceType> goal(space);

    int count = 0;
    do {
        start.random();
        goal.random();

        SMPL_INFO_STREAM("start = " << start.reals());
        SMPL_INFO("  r = %f", sqrt(start->getX() * start->getX() + start->getY() * start->getY()));

        SMPL_INFO_STREAM("goal = " << goal.reals());
        SMPL_INFO("  r = %f", sqrt(goal->getX() * goal->getX() + goal->getY() * goal->getY()));
        ++count;
    } while (!isStateValid(start.get()) || !isStateValid(goal.get()));
    SMPL_INFO("Sampled %d start/goal pairs", count);

    ss.setStartAndGoalStates(start, goal);

    auto planner = spns::make_shared<smpl::Planner>(ss.getSpaceInformation());
    planner->ompl::base::Planner::params().setParam("epsilon", "100.0");

    ss.setPlanner(planner);

    auto solved = ss.solve(5.0);

    if (solved) {
        std::cout << "Found solution:" << std::endl;
        ss.simplifySolution();
        ss.getSolutionPath().print(std::cout);
    }

    return 0;
}

// standard includes
#include <math.h>
#include <iostream>
#include <memory>

// project includes
#include <boost/make_shared.hpp>
#include <smpl_ompl_interface/ompl_interface.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <smpl/console/console.h>
#include <smpl/console/nonstd.h>

using StateSpaceType = ompl::base::SE2StateSpace;

bool isStateValid(const ompl::base::State* state)
{
    auto* s = state->as<StateSpaceType::StateType>();
    auto dx = s->getX();
    auto dy = s->getY();
    return (dx * dx + dy * dy > 0.5 * 0.5);
}

#ifdef ROS_INDIGO
namespace spns = boost;
#else
namespace spns = std;
#endif

namespace smpl = sbpl::motion;

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

    auto planner = spns::make_shared<smpl::OMPLPlanner>(ss.getSpaceInformation());
    planner->params().setParam("epsilon", "100.0");

    ss.setPlanner(planner);

    auto solved = ss.solve(5.0);

    if (solved) {
        std::cout << "Found solution:" << std::endl;
        ss.simplifySolution();
        ss.getSolutionPath().print(std::cout);
    }

    return 0;
}

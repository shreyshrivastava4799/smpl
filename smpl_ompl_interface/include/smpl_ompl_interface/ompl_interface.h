#ifndef SMPL_OMPL_INTERFACE_OMPL_INTERFACE_H
#define SMPL_OMPL_INTERFACE_OMPL_INTERFACE_H

// standard includes
#include <memory>

// system includes
#include <ompl/base/Planner.h>

namespace sbpl {
namespace motion {

namespace detail {
struct PlannerImpl;
} // namespace detail

struct OMPLPlanner : public ompl::base::Planner
{
    std::unique_ptr<detail::PlannerImpl> m_impl;

    OMPLPlanner(const ompl::base::SpaceInformationPtr& si);
    ~OMPLPlanner();

    void setProblemDefinition(const ompl::base::ProblemDefinitionPtr& pdef) override;

    auto solve(const ompl::base::PlannerTerminationCondition& ptc)
        -> ompl::base::PlannerStatus override;

    void clear() override;

    void checkValidity() override;

    void setup() override;

    void getPlannerData(ompl::base::PlannerData& data) const override;
};

} // namespace motion
} // namespace smpl

#endif


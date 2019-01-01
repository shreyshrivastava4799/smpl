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

#ifndef SMPL_COST_FUNCTION_H
#define SMPL_COST_FUNCTION_H

#include <smpl/types.h>

namespace smpl {

class GoalConstraint;
class ManipLattice;
struct ManipLatticeAction;
struct ManipLatticeState;

class CostFunction
{
public:

    virtual ~CostFunction();

    bool Init(ManipLattice* space);

    auto GetPlanningSpace() -> ManipLattice*;
    auto GetPlanningSpace() const -> const ManipLattice*;

    virtual bool UpdateStart(int state_id);
    virtual bool UpdateGoal(GoalConstraint* goal);

    virtual int GetActionCost(
        int state_id,
        const ManipLatticeAction* action,
        int succ_id) = 0;

private:

    ManipLattice* m_space = NULL;
};

class UniformCostFunction : public CostFunction
{
public:

    int cost_per_action = 1;

    int GetActionCost(
        int state_id,
        const ManipLatticeAction* action,
        int succ_id) final;
};

class L1NormCostFunction : public CostFunction
{
public:

    int GetActionCost(
        int state_id,
        const ManipLatticeAction* action,
        int succ_id) final;
};

class L2NormCostFunction : public CostFunction
{
public:

    int GetActionCost(
        int state_id,
        const ManipLatticeAction* action,
        int succ_id) final;
};

class LInfNormCostFunction : public CostFunction
{
public:

    int GetActionCost(
        int state_id,
        const ManipLatticeAction* action,
        int succ_id) final;
};

class LazyCostFunction
{
public:

    virtual ~LazyCostFunction();

    virtual auto GetLazyActionCost(
        int state_id,
        const ManipLatticeAction* action,
        int succ_id)
        -> std::pair<int, bool> = 0;

    virtual auto GetTrueActionCost(
        int state_id,
        const ManipLatticeAction* action,
        int succ_id)
        -> int = 0;
};

// A lazily-evaluated cost function that returns a lower-bound uniform cost for
// every action. When the true cost is evaluated, return the result of an
// associate cost function.
class DefaultLazyCostFunction : public LazyCostFunction
{
public:

    auto GetLazyActionCost(
        int state_id,
        const ManipLatticeAction* action,
        int succ_id)
        -> std::pair<int, bool> final;

    auto GetTrueActionCost(
        int state_id,
        const ManipLatticeAction* action,
        int succ_id)
        -> int final;

public:

    CostFunction* cost_fun = NULL;
};

} // namespace smpl

#endif

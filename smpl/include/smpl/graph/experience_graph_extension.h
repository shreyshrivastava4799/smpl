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

#ifndef SMPL_EXPERIENCE_GRAPH_EXTENSION_H
#define SMPL_EXPERIENCE_GRAPH_EXTENSION_H

#include <string>
#include <vector>

#include <smpl/extension.h>
#include <smpl/graph/experience_graph.h>

namespace smpl {

class ExperienceGraphExtension : public virtual Extension
{
public:

    virtual bool loadExperienceGraph(const std::string& path) = 0;

    virtual void getExperienceGraphNodes(
        int state_id,
        std::vector<ExperienceGraph::node_id>& nodes) = 0;

    virtual bool shortcut(int first_id, int second_id, int& cost) = 0;

    virtual bool snap(int first_id, int second_id, int& cost) = 0;

    virtual const ExperienceGraph* getExperienceGraph() const = 0;
    virtual ExperienceGraph* getExperienceGraph() = 0;

    virtual int getStateID(ExperienceGraph::node_id n) const = 0;
};

} // namespace smpl

#endif

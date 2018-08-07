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

#ifndef sbpl_collision_base_collision_states_h
#define sbpl_collision_base_collision_states_h

// standard includes
#include <ostream>
#include <vector>

// system includes
#include <Eigen/Dense>

// project includes
#include <sbpl_collision_checking/base_collision_models.h>

namespace smpl {
namespace collision {

struct CollisionSpheresState;

/// \brief Collision Sphere State Specification
struct CollisionSphereState
{
    const CollisionSphereModel* model;
    const CollisionSpheresState* parent_state;
    Eigen::Vector3d pos;
    const CollisionSphereState *left, *right;
    int version;

    CollisionSphereState() :
        model(nullptr), parent_state(nullptr), pos(), left(nullptr), right(nullptr), version(-1)
    { }

    // see note in CollisionSphereModel
    bool isLeaf() const { return !left; }
    int index() const;
};

std::ostream& operator<<(std::ostream& o, const CollisionSphereState& css);

class CollisionSpheresState;

class CollisionSphereStateTree
{
public:

    typedef std::vector<CollisionSphereState>       container_type;

    typedef container_type::value_type              value_type;
    typedef container_type::allocator_type          allocator_type;

    typedef container_type::size_type               size_type;
    typedef container_type::difference_type         difference_type;

    typedef container_type::reference               reference;
    typedef container_type::const_reference         const_reference;

    typedef container_type::iterator                iterator;
    typedef container_type::const_iterator          const_iterator;
    typedef container_type::reverse_iterator        reverse_iterator;
    typedef container_type::const_reverse_iterator  const_reverse_iterator;

    void buildFrom(CollisionSpheresState* parent_state);

    CollisionSphereStateTree() : m_tree() { }
    CollisionSphereStateTree(const CollisionSphereStateTree&);
    CollisionSphereStateTree(CollisionSphereStateTree&&);
    CollisionSphereStateTree& operator=(const CollisionSphereStateTree&);
    CollisionSphereStateTree& operator=(CollisionSphereStateTree&&);

    CollisionSphereState* root() { return &m_tree.back(); }
    const CollisionSphereState* root() const { return &m_tree.back(); }

    /// \name Vector-like Element Access
    ///@{
    reference at(size_type pos) { return m_tree.at(pos); }
    const_reference at(size_type pos) const { return m_tree.at(pos); }
    reference operator[](size_type pos) { return m_tree[pos]; }
    const_reference operator[](size_type pos) const { return m_tree[pos]; }
    reference front() { return m_tree.front(); }
    const_reference front() const { return m_tree.front(); }
    reference back() { return m_tree.back(); }
    const_reference back() const { return m_tree.back(); }
    CollisionSphereState* data() { return m_tree.data(); }
    const CollisionSphereState* data() const { return m_tree.data(); }
    ///@}

    /// \name Vector-like Iterators
    ///@{
    iterator begin() { return m_tree.begin(); }
    const_iterator begin() const { return m_tree.begin(); }
    const_iterator cbegin() const { return m_tree.cbegin(); }
    iterator end() { return m_tree.end(); }
    const_iterator end() const { return m_tree.end(); }
    const_iterator cend() const { return m_tree.cend(); }
    reverse_iterator rbegin() { return m_tree.rbegin(); }
    const_reverse_iterator rbegin() const { return m_tree.rbegin(); }
    const_reverse_iterator crbegin() const { return m_tree.crbegin(); }
    reverse_iterator rend() { return m_tree.rend(); }
    const_reverse_iterator rend() const { return m_tree.rend(); }
    const_reverse_iterator crend() const { return m_tree.crend(); }
    ///@}

    bool empty() const { return m_tree.empty(); }
    size_t size() const { return m_tree.size(); }

    // TODO: swap?

private:

    friend std::ostream& operator<<(std::ostream& o, const CollisionSphereStateTree& tree);

    container_type m_tree;
};

std::ostream& operator<<(std::ostream& o, const CollisionSphereStateTree& tree);

/// \brief Collision Spheres State Specification
struct CollisionSpheresState
{
    const CollisionSpheresModel* model;
    CollisionSphereStateTree spheres;
    int index;
};

std::ostream& operator<<(std::ostream& o, const CollisionSpheresState& css);

/// \brief Collision Voxels State Specification
struct CollisionVoxelsState
{
    const CollisionVoxelsModel* model;
    std::vector<Eigen::Vector3d> voxels; // in the model frame
};

std::ostream& operator<<(std::ostream& o, const CollisionVoxelsState& cvs);

/// \brief Collision Group State
struct CollisionGroupState
{
    const CollisionGroupModel* model;
    std::vector<int> spheres_indices; ///< sphere states inside the group
    std::vector<int> voxels_indices; ///< voxels states outside the group
};

std::ostream& operator<<(std::ostream& o, const CollisionGroupState& cgs);

inline
int CollisionSphereState::index() const
{
    return std::distance(&parent_state->spheres[0], this);
}

} // namespace collision
} // namespace smpl

#endif

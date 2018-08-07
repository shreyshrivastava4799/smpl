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

#ifndef sbpl_collision_base_collision_models_h
#define sbpl_collision_base_collision_models_h

// standard includes
#include <ostream>
#include <string>
#include <vector>

// system includes
#include <Eigen/Dense>

// project includes
#include <sbpl_collision_checking/collision_model_config.h>

namespace smpl {
namespace collision {

class CollisionSpheresModel;

struct CollisionGeometry;

/// \brief Collision Sphere Model Specification
struct CollisionSphereModel
{
    std::string name;
    Eigen::Vector3d center; ///< offset from link center
    double radius;
    int priority = 0;
    const CollisionSpheresModel* parent = nullptr;
    const CollisionSphereModel* left = nullptr;
    const CollisionSphereModel* right = nullptr;

    /// collision geometry this sphere is attached to, relevant to leaf spheres
    /// TODO: wasted space for internal spheres?
    const CollisionGeometry* geom = nullptr; 
    /// For specific collision geometry, identifies which piece of the shape
    /// this collision sphere is attached to, e.g. the index of a triangle
    /// within a triangle mesh.
    int shape_index = 0;

    // nodes can either have 0 or 2 children
    bool isLeaf() const { return left == right; }
    int index() const;
};

std::ostream& operator<<(std::ostream& o, const CollisionSphereModel& csm);

class CollisionSphereModelTree
{
public:

    typedef std::vector<CollisionSphereModel>       container_type;

    typedef container_type::value_type              value_type;
    typedef container_type::allocator_type          allocator_type;

    typedef container_type::size_type               size_type;
    typedef container_type::difference_type         difference_type;

    typedef container_type::const_reference         const_reference;

    typedef container_type::const_iterator          const_iterator;
    typedef container_type::const_reverse_iterator  const_reverse_iterator;

    // disallow copy/assign for now since the underyling tree structure is kept in a
    // compact array with internal references
    CollisionSphereModelTree() : m_tree() { }
    CollisionSphereModelTree(const CollisionSphereModelTree& o) = delete;
    CollisionSphereModelTree(CollisionSphereModelTree&& o);

    CollisionSphereModelTree& operator=(const CollisionSphereModelTree&) = delete;
    CollisionSphereModelTree& operator=(CollisionSphereModelTree&&) = delete;

    void buildFrom(const std::vector<CollisionSphereConfig>& spheres);
    void buildFrom(const std::vector<CollisionSphereModel>& spheres);

    void buildFrom(const std::vector<const CollisionSphereModel*>& spheres);

    const CollisionSphereModel* root() const { return &m_tree.back(); }

    /// \name Vector-like Element Access
    ///@{
    const_reference at(size_type pos) const { return m_tree.at(pos); }
    const_reference operator[](size_type pos) const { return m_tree[pos]; }
    const_reference front() const { return m_tree.front(); }
    const_reference back() const { return m_tree.back(); }
    const CollisionSphereModel* data() const { return m_tree.data(); }
    ///@}

    /// \name Vector-like Iterators
    ///@{
    const_iterator begin() const { return m_tree.begin(); }
    const_iterator cbegin() const { return m_tree.cbegin(); }
    const_iterator end() const { return m_tree.end(); }
    const_iterator cend() const { return m_tree.cend(); }
    const_reverse_iterator rbegin() const { return m_tree.rbegin(); }
    const_reverse_iterator crbegin() const { return m_tree.crbegin(); }
    const_reverse_iterator rend() const { return m_tree.rend(); }
    const_reverse_iterator crend() const { return m_tree.crend(); }
    ///@}

    bool empty() const { return m_tree.empty(); }
    size_t size() const { return m_tree.size(); }

    double maxRadius() const;
    double maxLeafRadius() const;

private:

    friend std::ostream& operator<<(std::ostream& o, const CollisionSphereModelTree& tree);

    template <typename Sphere>
    size_t buildRecursive(
        typename std::vector<const Sphere*>::iterator msfirst,
        typename std::vector<const Sphere*>::iterator mslast);

    size_t buildMetaRecursive(
        std::vector<const CollisionSphereModel*>::iterator msfirst,
        std::vector<const CollisionSphereModel*>::iterator mslast);

    void computeOptimalBoundingSphere(
        const CollisionSphereModel& s1,
        const CollisionSphereModel& s2,
        Eigen::Vector3d& c, double& r);

    template <typename Sphere>
    int computeLargestBoundingBoxAxis(
        typename std::vector<const Sphere*>::iterator msfirst,
        typename std::vector<const Sphere*>::iterator mslast);

public: // TODO: mimic iterators from CollisionSphereTree
    container_type m_tree;
};

std::ostream& operator<<(std::ostream& o, const CollisionSphereModelTree& tree);

/// \brief Collision Spheres Model Specification
struct CollisionSpheresModel
{
    int link_index;
    CollisionSphereModelTree spheres;
};

std::ostream& operator<<(std::ostream& o, const CollisionSpheresModel& csm);

/// \brief Collision Voxels Model Specification
struct CollisionVoxelsModel
{
    int link_index; // -1 if not attached to a link
    double voxel_res;
    std::vector<Eigen::Vector3d> voxels; // in the link frame
};

std::ostream& operator<<(std::ostream& o, const CollisionVoxelsModel& cvm);

/// \brief Collision Group Model Specification
struct CollisionGroupModel
{
    std::string name;
    std::vector<int> link_indices; // TODO: rename body_indices
};

std::ostream& operator<<(std::ostream& o, const CollisionGroupModel& cgm);

struct SphereIndex
{
    int ss;
    int s;

    SphereIndex() : ss(), s() { }
    SphereIndex(int ss, int s) : ss(ss), s(s) { }
};

std::ostream& operator<<(std::ostream& o, const SphereIndex& i);
std::string to_string(const SphereIndex& i);

inline
int CollisionSphereModel::index() const
{
    return std::distance(&parent->spheres[0], this);
}

} // namespace collision
} // namespace smpl

#endif

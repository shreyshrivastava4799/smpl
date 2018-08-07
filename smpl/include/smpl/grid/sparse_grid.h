////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2017, Andrew Dornbush
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

#ifndef SMPL_SPARSE_GRID_H
#define SMPL_SPARSE_GRID_H

// standard includes
#include <memory>

// project includes
#include <smpl/octree/octree.h>

namespace smpl {

/// This class represents a resizeable three-dimensional array of sparse data.
/// For space efficiency, the underlying data is stored in compressed octree
/// format. The octree is grown to dynamically to represent the local
/// differences between neighboring cells.
///
/// Cells never exist with completely uninitialized values. When given no value
/// upon construction, the value for all cells is value initialized (which may
/// be uninitialized for primitive types). When a default value is given upon
/// construction, the value for all cells is copy initialized from the input
/// argument.
///
/// The preferred method of access to cells is through the get() and set()
/// functions. These functions will automatically prune the underlying octree
/// during traversal, optimally reducing memory consumption. In the event nodes
/// are expanded, pointers and references will not be invalidated, but may not
/// reflect a cell's true value. In the event nodes are collapsed, pointers and
/// references to collapsed node's children will be invalidated.
///
/// If the above behavior of pointer and reference invalidation is undesirable,
/// unique leaf nodes may be implicitly created through use of the non-const
/// operator() method. This will force maximum-depth expansion of nodes down to
/// the underlying cell.
///
/// If a sequence of gets() and sets() would result in many unnecessary node
/// expansions and collapses, the set_lazy() function may be used in place of
/// set() to skip automatic pruning of nodes. The underlying octree may then be
/// explicitly pruned by calling the prune() function, which will prune all
/// nodes where applicable for maximum compression.
template <class T, class Allocator = std::allocator<T>>
class SparseGrid
{
public:

    using TreeType = OcTree<T, Allocator>;
    using node_type = typename TreeType::node_type;

    using value_type        = typename TreeType::value_type;
    using size_type         = typename TreeType::size_type;
    using reference         = value_type&;
    using const_reference   = const value_type&;
    using index_type        = int;

    SparseGrid();
    SparseGrid(const T& value);
    SparseGrid(size_type size_x, size_type size_y, size_type size_z);
    SparseGrid(
        size_type size_x, size_type size_y, size_type size_z,
        const T& value);

    explicit SparseGrid(const Allocator& alloc);
    explicit SparseGrid(const T& value, const Allocator& alloc);
    explicit SparseGrid(
        size_type size_x, size_type size_y, size_type size_z,
        const Allocator& alloc);
    explicit SparseGrid(
        size_type size_x, size_type size_y, size_type size_z,
        const T& value, const Allocator& alloc);

    /// \name Size Properties
    ///@{
    size_type size() const;
    size_type max_size() const;
    size_type size_x() const;
    size_type size_y() const;
    size_type size_z() const;

    int max_depth() const;

    size_type mem_usage() const;
    ///@}

    /// \name Element Access
    ///@{
    const_reference operator()(index_type x, index_type y, index_type z) const;

    reference operator()(index_type x, index_type y, index_type z);

    const_reference get(index_type x, index_type y, index_type z) const;
    ///@}

    /// \name Modifiers
    ///@{
    void reset(const T& value);
    void assign(const T& value);

    void set(index_type x, index_type y, index_type z, const T& data);
    void set_lazy(index_type x, index_type y, index_type z, const T& data);

    void prune();

    template <class UnaryPredicate>
    void prune(UnaryPredicate p);

    void resize(size_type size_x, size_type size_y, size_type size_z);
    void resize(size_type size_x, size_type size_y, size_type size_z, const T& value);
    ///@}

    template <class Pred>
    size_type mem_usage_full(const Pred& pred);

    template <typename Callable>
    void accept(Callable c);

    template <typename Callable>
    void accept_coords(Callable c);

    const TreeType &tree() const { return m_tree; }

private:

    OcTree<T, Allocator> m_tree;

    int m_max_depth;
    size_type m_size[3];

    int compute_max_depth(
        size_type size_x,
        size_type size_y,
        size_type size_z) const;

    const node_type* get_node(
        int rdepth,
        const node_type* n,
        index_type x,
        index_type y,
        index_type z) const;

    node_type* get_node_unique(
        int rdepth,
        node_type* n,
        index_type x,
        index_type y,
        index_type z);

    bool set_node(
        int rdepth,
        node_type* n,
        index_type x,
        index_type y,
        index_type z,
        const T& value);

    void set_node_lazy(
        int rdepth,
        node_type* n,
        index_type x,
        index_type y,
        index_type z,
        const T& value);

    node_type* create_node(
        int rdepth,
        node_type* n,
        index_type x,
        index_type y,
        index_type z);

    bool collapsible(node_type* n) const;

    bool prune(node_type* n);

    template <class UnaryPredicate>
    bool prune(node_type* n, UnaryPredicate p);

    template <typename Callable>
    void accept_coords(
        Callable c, node_type* n,
        size_type first_x, size_type first_y, size_type first_z,
        size_type last_x, size_type last_y, size_type last_z);
};

} // namespace smpl

#include "detail/sparse_grid.hpp"

#endif

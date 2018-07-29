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

#ifndef SMPL_OCTREE_H
#define SMPL_OCTREE_H

// standard includes
#include <cstdlib>
#include <memory>
#include <utility>
#include <stack>
#include <vector>

#include "detail/octree_base.h"

namespace smpl {

/// A tree data structure storing elements that have either 8 children, if the
/// node is internal, or no children, if the node is a leaf node. This class
/// contains functionality for maintenance of the tree (expanding or collapsing
/// nodes), traversal of the tree, and basic properties of the tree such as its
/// current depth.
template <
    class T,
    class Allocator = std::allocator<T>>
class OcTree : protected detail::OcTreeBase<T, Allocator>
{
protected:

    using Base = detail::OcTreeBase<T, Allocator>;

    template <class U, class V> struct dfs_iterator;

public:

    using value_type        = T;
    using allocator_type    = Allocator;
    using size_type         = std::size_t;

    using typename Base::node_type;
    using Base::clear;

    using iterator          = dfs_iterator<node_type*, value_type>;
    using const_iterator    = dfs_iterator<const node_type*, const value_type>;

    OcTree();
    OcTree(const T& value);
    OcTree(T&& value);

    explicit OcTree(const Allocator& alloc);
    explicit OcTree(const T& value, const Allocator& alloc);
    explicit OcTree(T&& value, const Allocator& alloc);

    OcTree(const OcTree& o);
    OcTree(const OcTree& o, const Allocator&  alloc);

    OcTree(OcTree&& o);
//    OcTree(OcTree&& o, const Allocator& alloc);

    ~OcTree();

    OcTree& operator=(const OcTree& rhs);
    OcTree& operator=(OcTree&& rhs);

    allocator_type get_allocator() const { return Base::get_allocator(); }

    /// \name Size Properties
    ///@{
    int depth() const;

    size_type num_nodes() const;
    size_type num_leaves() const;

    size_type mem_usage() const;
    ///@}

    node_type* root();
    const node_type* root() const;

    void collapse_node(node_type* n, const T& value);

    void expand_node(node_type* n);

    /// \name Iteration
    ///@{
    std::pair<iterator, iterator> nodes()
    {
        std::pair<iterator, iterator> p;
        p.first.s.push(root());
        return p;
    }

    std::pair<const_iterator, const_iterator> nodes() const
    {
        std::pair<const_iterator, const_iterator> p;
        p.first.s.push(root());
        return p;
    }

    struct leaf_iterator;
    std::pair<leaf_iterator, leaf_iterator> leaves();

    template <typename Callable>
    void accept(Callable c);
    ///@}

protected:

    using atraits               = std::allocator_traits<allocator_type>;

    using typename Base::natraits;
    using typename Base::node_allocator_type;
    using Base::m_impl;
    using Base::get_node_allocator;
    using Base::alloc_children;
    using Base::dealloc_children;
    using Base::construct_node;
//    using Base::destroy_node;
    using Base::construct_children;
    using Base::destroy_children;
//    using Base::clear;

    template <class U, class V>
    struct dfs_iterator
    {
        std::stack<U, std::vector<U>> s;

        dfs_iterator& operator++();
        dfs_iterator operator++(int);
        V& operator*() const;
        V* operator->() const;
        bool operator==(const dfs_iterator& i) const;
        bool operator!=(const dfs_iterator& i) const;
    };

    int depth(const node_type* n) const;
    size_type count_nodes(const node_type* n) const;
    size_type count_leaves(const node_type* n) const;

    size_type mem_usage(const node_type* n) const;

    void clone_children(node_type *nout, const node_type *nin);
    void move_children(node_type *nout, node_type *nin);

    template <typename Callable>
    void accept(Callable c, node_type* n);
};

} // namespace smpl

#include "detail/octree.hpp"

#endif

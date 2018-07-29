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

#ifndef SMPL_OCTREE_BASE_H
#define SMPL_OCTREE_BASE_H

// standard includes
#include <assert.h>
#include <memory>
#include <utility>

namespace smpl {
namespace detail {

template <typename T>
struct OcTreeNode
{
    T value;
    OcTreeNode* children;

    template <typename... Args>
    OcTreeNode(Args&&... args) :
        value(std::forward<Args>(args)...),
        children(nullptr)
    { }

    OcTreeNode* child(int i) const
    {
        assert(children);
        assert(i >= 0 && i < 8);
        return &children[i];
    }
};

/// Base class for OcTree which manages (de)allocation and (de)construction of
/// child nodes. Nodes are structured so that they contain (1) their stored
/// value and (2) an array of 0 or 8 children. All children are (de)allocated
/// and (de)constructed at once, so containment of subsets of children is not
/// allowed. Storage for children is allocated on demand as nodes are expanded.
template <class T, class Allocator>
class OcTreeBase
{
protected:

    using node_allocator_type =
            typename std::allocator_traits<Allocator>::template
            rebind_alloc<OcTreeNode<T>>;

    using type_allocator_type =
            typename std::allocator_traits<Allocator>::template
            rebind_alloc<T>;

public:

    using allocator_type    = Allocator;
    using node_type         = OcTreeNode<T>;

    /// Construct with a default value for the root node
    OcTreeBase() : m_impl() { }

    OcTreeBase(const node_allocator_type& a) : m_impl(a) { }

    OcTreeBase(node_allocator_type&& a) : m_impl(std::move(a)) { }

    // Construct given constructor arguments for the root node
    template <typename... Args>
    OcTreeBase(Args&&... args) : m_impl(std::forward<Args>(args)...) { }

    template <typename... Args>
    OcTreeBase(const node_allocator_type& a, Args&&... args) :
        m_impl(a, std::forward<Args>(args)...)
    { }

    template <typename... Args>
    OcTreeBase(node_allocator_type&& a, Args&&... args) :
        m_impl(std::move(a), std::forward<Args>(args)...)
    { }

    OcTreeBase(OcTreeBase&& o) : m_impl(std::move(o.m_impl)) { }

    ~OcTreeBase() { clear(); }

    allocator_type get_allocator() const noexcept
    { return allocator_type(get_node_allocator()); }

    void clear() { clear_node(&m_impl.m_node); m_impl.m_node.children = nullptr; }

protected:

    using natraits = std::allocator_traits<node_allocator_type>;

    struct OcTreeImpl : public node_allocator_type
    {
        node_type m_node;

        OcTreeImpl() : node_allocator_type(), m_node() { }

        OcTreeImpl(const node_allocator_type& a) :
            node_allocator_type(a), m_node()
        { }

        OcTreeImpl(node_allocator_type&& a) :
            node_allocator_type(a), m_node()
        { }

        template <typename... Args>
        OcTreeImpl(Args&&... args) :
            node_allocator_type(), m_node(std::forward<Args>(args)...)
        { }

        template <typename... Args>
        OcTreeImpl(const node_allocator_type& a, Args&&... args) :
            node_allocator_type(a), m_node(std::forward<Args>(args)...)
        { }

        template <typename... Args>
        OcTreeImpl(node_allocator_type&& a, Args&&... args) :
            node_allocator_type(a), m_node(std::forward<Args>(args)...)
        { }

        OcTreeImpl(OcTreeImpl&& o) :
            node_allocator_type(std::move(o)),
            m_node(std::move(o.m_node))
        {
            o.m_node.children = nullptr;
        }
    };

    OcTreeImpl m_impl;

    const node_allocator_type& get_node_allocator() const noexcept;

    node_allocator_type& get_node_allocator() noexcept;

    type_allocator_type get_type_allocator() const noexcept;

    void alloc_children(node_type *n);

    void dealloc_children(node_type *n);

    template <typename... Args>
    void construct_node(node_type *n, Args&&... args);

    void destroy_node(node_type *n);

    template <typename... Args>
    void construct_children(node_type *n, Args&&... args);

    void destroy_children(node_type *n);

    void clear_node(node_type *node);
};

} // namespace detail
} // namespace smpl

#include "octree_base.hpp"

#endif

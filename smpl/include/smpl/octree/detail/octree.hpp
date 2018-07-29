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

#ifndef SMPL_OCTREE_HPP
#define SMPL_OCTREE_HPP

#include "../octree.h"

// standard includes
#include <cmath>
#include <algorithm>

namespace smpl {

/// Construct an OcTree whose root node's value is initialized through value
/// initialization
template <class T, class Allocator>
OcTree<T, Allocator>::OcTree() : Base(node_allocator_type(Allocator()))
{
}

/// Construct an OcTree whose root node's value is initialized through copy
/// construction
template <class T, class Allocator>
OcTree<T, Allocator>::OcTree(const T& value) :
    Base(node_allocator_type(Allocator()), value)
{
}

/// Construct an OcTree whose root node's value is initialized through move
/// construction
template <class T, class Allocator>
OcTree<T, Allocator>::OcTree(T&& value) :
    Base(node_allocator_type(Allocator()), std::move(value))
{
}

/// Construct an OcTree whose root node's value is initialized through value
/// initialization
template <class T, class Allocator>
OcTree<T, Allocator>::OcTree(const Allocator& alloc) :
    Base(node_allocator_type(alloc))
{
}

/// Construct an OcTree whose root node's value is initialized through copy
/// construction
template <class T, class Allocator>
OcTree<T, Allocator>::OcTree(const T& value, const Allocator& alloc) :
    Base(node_allocator_type(alloc), value)
{
}

/// Construct an OcTree whose root node's value is initialized through move
/// construction
template <class T, class Allocator>
OcTree<T, Allocator>::OcTree(T&& value, const Allocator& alloc) :
    Base(node_allocator_type(alloc), std::move(value))
{
}

/// Copy constructor. Constructs the OcTree with a copy of the contents of $o.
/// The allocator is obtained as if by calling
/// std::allocator_traits<allocator_type>::select_on_container_copy_construction()
/// with $other's node allocator.
template <class T, class Allocator>
OcTree<T, Allocator>::OcTree(const OcTree& o) :
    Base(natraits::select_on_container_copy_construction(o.get_node_allocator()), o.m_impl.m_node.value)
{
    clone_children(root(), o.root());
}

// Copy constructor. Constructs the OcTree with a copy of the contents of $o.
template <class T, class Allocator>
OcTree<T, Allocator>::OcTree(const OcTree& o, const Allocator& alloc) :
    Base(alloc, o.m_impl.m_node.value)
{
    clone_children(root(), o.root());
}

/// Move constructor. Constructs the OcTree with the contents of \p o using move
/// semantics. Allocator is obtained by move-construction from the allocator
/// belonging to \p o
template <class T, class Allocator>
OcTree<T, Allocator>::OcTree(OcTree&& o) :
    Base((Base&&)std::move(o))
{
}

/// Destructs the OcTree.
template <class T, class Allocator>
OcTree<T, Allocator>::~OcTree()
{
    clear();
}

/// Copy assignment operator. Replaces the contents with a copy of the contents
/// of \p rhs.
template <class T, class Allocator>
OcTree<T, Allocator>&
OcTree<T, Allocator>::operator=(const OcTree& rhs)
{
    if (this != &rhs) {
        clear();

        typedef std::allocator_traits<node_allocator_type> natraits;
        if (typename natraits::propagate_on_container_copy_assignment()) {
            // note: always free memory, no need to check this
//            if (get_node_allocator() != rhs.get_node_allocator()) { }

            // copy-assign allocator
            get_node_allocator() = rhs.get_node_allocator();
        }

        m_impl.m_node.value = rhs.m_impl.m_node.value;
        clone_children(root(), rhs.root());
    }
    return *this;
}

/// Move assignment operator. Replaces the contents with those of \p rhs using
/// move semantics. If std::allocator_traits<node_allocator_type> is true, the
/// target allocator is replaced by a copy of the source allocator. If it is
/// false and the source and target allocators do not compare equal, the target
/// cannot take ownership of the source memory and must move-assign each element
/// individually, allocating additional memory using its own allocator as
/// needed. In any case, all elements originally present in *this are either
/// destroyed or replaced by elementwise move-assignment.
template <class T, class Allocator>
OcTree<T, Allocator>&
OcTree<T, Allocator>::operator=(OcTree&& rhs)
{
    if (this != &rhs) {
        // TODO: don't need to clear the entire tree if we're doing element-wise
        // move assignment.
        clear();

        // TODO: compile time if this
        if (typename natraits::propagate_on_container_move_assignment()) {
            // move assign root node
            // TODO: it's a little weird here that the root node is move
            // assigned but the remaining elements are untouched...
            m_impl.m_node = std::move(rhs.m_impl.m_node);

            // remove children from rhs root
            rhs.m_impl.m_node.children = nullptr;

            // move-assign allocator
            get_node_allocator() = std::move(rhs.get_node_allocator());
        } else {
            if (get_node_allocator() != rhs.get_node_allocator()) {
                m_impl.m_node.value = std::move(rhs.m_impl.m_node.value);

                // clone tree, move assign each child
                move_children(root(), rhs.root());
            } else {
                m_impl.m_node = std::move(rhs.m_impl.m_node);
                rhs.m_impl.m_node.children = nullptr;
            }
        }
    }
    return *this;
}

/// Returns the largest depth of any element in the OcTree. Returns 0 for an
/// OcTree consisting of only the root node.
template <class T, class Allocator>
int
OcTree<T, Allocator>::depth() const
{
    return depth(root());
}

/// Return the number of nodes, including both leaf and internal nodes. Note
/// that theoretically the number of total nodes may not fit within this value,
/// if the platform is capable of addressing more memory than size_t.
template <class T, class Allocator>
typename OcTree<T, Allocator>::size_type
OcTree<T, Allocator>::num_nodes() const
{
    return count_nodes(root());
}

/// Return the number of leaf nodes. See num_nodes() for note on platform and
/// implementation limits.
template <class T, class Allocator>
typename OcTree<T, Allocator>::size_type
OcTree<T, Allocator>::num_leaves() const
{
    return count_leaves(root());
}

/// Return the approximate number of bytes used by the octree. Note the size is
/// obtained from sizeof which will not account for dynamic memory allocated by
/// an element
template <class T, class Allocator>
typename OcTree<T, Allocator>::size_type
OcTree<T, Allocator>::mem_usage() const
{
    return mem_usage(root());
}

/// Return a pointer to the root node. This is never null.
template <class T, class Allocator>
typename OcTree<T, Allocator>::node_type*
OcTree<T, Allocator>::root()
{
    return &m_impl.m_node;
}

/// Return a const pointer to the root node. This is never null.
template <class T, class Allocator>
const typename OcTree<T, Allocator>::node_type*
OcTree<T, Allocator>::root() const
{
    return &m_impl.m_node;
}

/// Collapse a node by destroying and deallocating its children. This function
/// is not called recursively on the children, and it is assumed the given node
/// has no grandchildren, otherwise memory will be leaked.
template <class T, class Allocator>
void OcTree<T, Allocator>::collapse_node(node_type* n, const T& value)
{
    // set value for node, before obliterating children in case value comes from
    // them
    n->value = value;
    destroy_children(n);
    dealloc_children(n);
    n->children = nullptr;
}

/// Expand a node by allocating and constructing its children with the value of
/// the element stored in the node. This function does not check for the
/// existence of previous children and assumes no prior children exist have been
/// created, otherwise memory will be leaked.
template <class T, class Allocator>
void OcTree<T, Allocator>::expand_node(node_type* n)
{
    assert(n && !n->children);
    alloc_children(n);
    construct_children(n, n->value);
}

/// Accept a function to be called on every node in the OcTree, in depth-first
/// order.
template <class T, class Allocator>
template <typename Callable>
void OcTree<T, Allocator>::accept(Callable fun)
{
    accept(fun, root());
}

template <class T, class Allocator>
int
OcTree<T, Allocator>::depth(const node_type* n) const
{
    if (!n->children) {
        return 0;
    }
    else {
        int d = 0;
        for (const node_type* c = n->children; c != n->children + 8; ++c) {
            d = std::max(d, depth(c));
        }

        return 1 + d;
    }
}

template <class T, class Allocator>
typename OcTree<T, Allocator>::size_type
OcTree<T, Allocator>::count_nodes(const node_type* n) const
{
    if (!n->children) {
        return 1;
    }
    else {
        int sum = 1;
        for (const node_type* c = n->children; c != n->children + 8; ++c) {
            sum += count_nodes(c);
        }

        return sum;
    }
}

template <class T, class Allocator>
typename OcTree<T, Allocator>::size_type
OcTree<T, Allocator>::count_leaves(const node_type* n) const
{
    if (!n->children) {
        return 1;
    }
    else {
        int sum = 0;
        for (const node_type* c = n->children; c != n->children + 8; ++c) {
            sum += count_leaves(c);
        }

        return sum;
    }
}

template <class T, class Allocator>
typename OcTree<T, Allocator>::size_type
OcTree<T, Allocator>::mem_usage(const node_type* n) const
{
    if (!n->children) {
        return sizeof(T);
    }
    else {
        int sum = 0;
        for (const node_type* c = n->children; c != n->children + 8; ++c) {
            sum += mem_usage(c);
        }

        return sum;
    }
}

template <class T, class Allocator>
void
OcTree<T, Allocator>::clone_children(node_type *nout, const node_type *nin)
{
    assert(!nout->children);
    if (nin->children) {
        alloc_children(nout);
        for (node_type *co = nout->children, *ci = nin->children;
             co < nout->children + 8; ++co, ++ci)
        {
            construct_node(co, ci->value);
            clone_children(co, ci);
        }
    }
}

template <class T, class Allocator>
void
OcTree<T, Allocator>::move_children(node_type *nout, node_type *nin)
{
    assert (!nout->children);
    if (nin->children) {
        alloc_children(nout);
        for (node_type *co = nout->children, *ci = nin->children;
            co < nout->children + 8; ++co, ++ci)
        {
            construct_node(co, std::move(ci->value));
            move_children(co, ci);
        }
    }
}

template <class T, class Allocator>
template <typename Callable>
void OcTree<T, Allocator>::accept(Callable fn, node_type* n)
{
    fn(n->value);

    if (n->children) {
        for (node_type* c = n->children; c != n->children + 8; ++c) {
            accept(fn, c);
        }
    }
}

template <class T, class Allocator>
template <class U, class V>
auto OcTree<T, Allocator>::dfs_iterator<U, V>::operator++() -> dfs_iterator&
{
    U t = s.top();
    s.pop();
    if (t->children) {
        for (int i = 0; i < 8; ++i) {
            s.push(&t->children[i]);
        }
    }
    return *this;
}

template <class T, class Allocator>
template <class U, class V>
auto OcTree<T, Allocator>::dfs_iterator<U, V>::operator++(int) -> dfs_iterator
{
    dfs_iterator i = *this;
    ++*this;
    return i;
}

template <class T, class Allocator>
template <class U, class V>
auto OcTree<T, Allocator>::dfs_iterator<U, V>::operator*() const -> V&
{
    return s.top()->value;
}

template <class T, class Allocator>
template <class U, class V>
auto OcTree<T, Allocator>::dfs_iterator<U, V>::operator->() const -> V*
{
    return &s.top()->value;
}

template <class T, class Allocator>
template <class U, class V>
bool OcTree<T, Allocator>::dfs_iterator<U, V>::operator==(
    const dfs_iterator& i) const
{
    return s == i.s;
}

template <class T, class Allocator>
template <class U, class V>
bool OcTree<T, Allocator>::dfs_iterator<U, V>::operator!=(
    const dfs_iterator& i) const
{
    return !(s == i.s);
}

} // namespace smpl

#endif

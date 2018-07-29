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

#ifndef SMPL_EXPERIENCE_GRAPH_H
#define SMPL_EXPERIENCE_GRAPH_H

// standard includes
#include <cstdlib>
#include <vector>

// system includes
#include <smpl/types.h>

namespace smpl {

// undirected
class ExperienceGraph
{
    struct Node;
    struct Edge;

    typedef std::vector<Node> node_container;
    typedef std::vector<Edge> edge_container;

public:

    typedef node_container::size_type node_id;
    typedef edge_container::size_type edge_id;

private:

    struct Node
    {
        typedef std::pair<edge_id, node_id> adjacency;
        typedef std::vector<adjacency> adjacent_edge_container;

        RobotState state;
        adjacent_edge_container edges;

        Node(const RobotState& state) : state(state) { }
    };

    struct Edge
    {
        std::vector<RobotState> waypoints;
        node_id snode;
        node_id tnode;

        Edge(node_id uid, node_id vid) : waypoints(), snode(uid), tnode(vid) { }

        Edge(const std::vector<RobotState>& waypoints, node_id uid, node_id vid)
            : waypoints(waypoints), snode(uid), tnode(vid) { }
    };

public:

    typedef node_container::size_type nodes_size_type;
    typedef edge_container::size_type edges_size_type;
    typedef Node::adjacent_edge_container::size_type degree_size_type;

    struct node_iterator : std::iterator<std::random_access_iterator_tag, node_id>
    {
        node_iterator(node_id id);

        const value_type operator*() const;
        node_iterator operator++(int);
        node_iterator& operator++();

        node_iterator& operator+=(difference_type n);
        node_iterator& operator-=(difference_type n);
        difference_type operator-(node_iterator it);
        bool operator==(node_iterator it) const;
        bool operator!=(node_iterator it) const;

    private:

        node_id m_id;
    };

    struct edge_iterator : std::iterator<std::random_access_iterator_tag, edge_id>
    {
        edge_iterator(edge_id id);

        const value_type operator*() const;
        edge_iterator operator++(int);
        edge_iterator& operator++();

        edge_iterator& operator+=(difference_type n);
        edge_iterator& operator-=(difference_type n);
        difference_type operator-(edge_iterator it);
        bool operator==(edge_iterator it) const;
        bool operator!=(edge_iterator it) const;

    private:

        edge_id m_id;
    };

    typedef Node::adjacent_edge_container::const_iterator adjacent_edge_iterator;

    struct incident_edge_iterator : std::iterator<std::random_access_iterator_tag, edge_id>
    {
        incident_edge_iterator(adjacent_edge_iterator it);

        const value_type operator*() const;
        incident_edge_iterator operator++(int);
        incident_edge_iterator& operator++();

        incident_edge_iterator& operator+=(difference_type n);
        incident_edge_iterator& operator-=(difference_type n);
        difference_type operator-(incident_edge_iterator it);
        bool operator==(incident_edge_iterator it) const;
        bool operator!=(incident_edge_iterator it) const;

    private:

        adjacent_edge_iterator m_it;
    };

    struct adjacency_iterator : std::iterator<std::random_access_iterator_tag, node_id>
    {
        adjacency_iterator(adjacent_edge_iterator it);

        const value_type operator*() const;
        adjacency_iterator operator++(int);
        adjacency_iterator& operator++();

        adjacency_iterator& operator+=(difference_type n);
        adjacency_iterator& operator-=(difference_type n);
        difference_type operator-(adjacency_iterator it);
        bool operator==(adjacency_iterator it) const;
        bool operator!=(adjacency_iterator it) const;

    private:

        adjacent_edge_iterator m_it;
    };

    ExperienceGraph();

    std::pair<node_iterator, node_iterator> nodes() const;
    std::pair<edge_iterator, edge_iterator> edges() const;
    std::pair<incident_edge_iterator, incident_edge_iterator> edges(node_id id) const;
    std::pair<adjacency_iterator, adjacency_iterator> adjacent_nodes(node_id id) const;

    degree_size_type degree(node_id id) const;
    node_id source(edge_id id) const;
    node_id target(edge_id id) const;

    nodes_size_type num_nodes() const { return m_nodes.size(); }
    edges_size_type num_edges() const { return m_edges.size(); }

    bool edge(node_id uid, node_id vid) const;

    node_id insert_node(const RobotState& state);
    void erase_node(node_id id);

    edge_id insert_edge(node_id uid, node_id vid);
    edge_id insert_edge(
        node_id uid,
        node_id vid,
        const std::vector<RobotState>& path);

    void erase_edge(node_id uid, node_id vid);
    void erase_edge(edge_id id);

    const RobotState& state(node_id id) const { return m_nodes[id].state; }
    RobotState& state(node_id id) { return m_nodes[id].state; }

    const std::vector<RobotState>& waypoints(edge_id id) const {
        return m_edges[id].waypoints;
    }
    std::vector<RobotState>& waypoints(edge_id id) {
        return m_edges[id].waypoints;
    }

private:

    std::vector<Node> m_nodes;
    std::vector<Edge> m_edges;

    // cached storage for mapping edge ids to their id adjustments during node
    // removal
    std::vector<std::ptrdiff_t> m_shift;

    void insert_incident_edge(edge_id eid, node_id uid, node_id vid);
};

inline
ExperienceGraph::incident_edge_iterator::incident_edge_iterator(
    adjacent_edge_iterator it)
:
    m_it(it)
{
}

inline
const ExperienceGraph::incident_edge_iterator::value_type
ExperienceGraph::incident_edge_iterator::operator*() const
{
    return m_it->first;
}

inline
ExperienceGraph::incident_edge_iterator
ExperienceGraph::incident_edge_iterator::operator++(int)
{
    incident_edge_iterator it(m_it);
    ++m_it;
    return it;
}

inline
ExperienceGraph::incident_edge_iterator&
ExperienceGraph::incident_edge_iterator::operator++()
{
    ++m_it;
    return *this;
}

inline
ExperienceGraph::incident_edge_iterator&
ExperienceGraph::incident_edge_iterator::operator+=(difference_type n)
{
    m_it += n;
    return *this;
}

inline
ExperienceGraph::incident_edge_iterator&
ExperienceGraph::incident_edge_iterator::operator-=(difference_type n)
{
    return operator+=(-n);
}

inline
ExperienceGraph::incident_edge_iterator::difference_type
ExperienceGraph::incident_edge_iterator::operator-(incident_edge_iterator it)
{
    return m_it - it.m_it;
}

inline
bool
ExperienceGraph::incident_edge_iterator::operator==(incident_edge_iterator it) const
{
    return it.m_it == m_it;
}

inline
bool
ExperienceGraph::incident_edge_iterator::operator!=(incident_edge_iterator it) const
{
    return it.m_it != m_it;
}

inline
ExperienceGraph::adjacency_iterator::adjacency_iterator(
    adjacent_edge_iterator it)
:
    m_it(it)
{
}

inline
const ExperienceGraph::adjacency_iterator::value_type
ExperienceGraph::adjacency_iterator::operator*() const
{
    return m_it->second;
}

inline
ExperienceGraph::adjacency_iterator
ExperienceGraph::adjacency_iterator::operator++(int)
{
    adjacency_iterator it(m_it);
    ++m_it;
    return it;
}

inline
ExperienceGraph::adjacency_iterator&
ExperienceGraph::adjacency_iterator::operator++()
{
    ++m_it;
    return *this;
}

inline
ExperienceGraph::adjacency_iterator&
ExperienceGraph::adjacency_iterator::operator+=(difference_type n)
{
    m_it += n;
    return *this;
}

inline
ExperienceGraph::adjacency_iterator&
ExperienceGraph::adjacency_iterator::operator-=(difference_type n)
{
    return operator+=(-n);
}

inline
ExperienceGraph::adjacency_iterator::difference_type
ExperienceGraph::adjacency_iterator::operator-(adjacency_iterator it)
{
    return m_it - it.m_it;
}

inline
bool
ExperienceGraph::adjacency_iterator::operator==(adjacency_iterator it) const
{
    return it.m_it == m_it;
}

inline
bool
ExperienceGraph::adjacency_iterator::operator!=(adjacency_iterator it) const
{
    return it.m_it != m_it;
}

inline
ExperienceGraph::node_iterator::node_iterator(node_id id) :
    m_id(id)
{
}

inline
const ExperienceGraph::node_iterator::value_type
ExperienceGraph::node_iterator::operator*() const
{
    return m_id;
}

inline
ExperienceGraph::node_iterator
ExperienceGraph::node_iterator::operator++(int)
{
    return node_iterator(m_id++);
}

inline
ExperienceGraph::node_iterator&
ExperienceGraph::node_iterator::operator++()
{
    ++m_id;
    return *this;
}

inline
ExperienceGraph::node_iterator&
ExperienceGraph::node_iterator::operator+=(difference_type n)
{
    m_id += n;
    return *this;
}

inline
ExperienceGraph::node_iterator&
ExperienceGraph::node_iterator::operator-=(difference_type n)
{
    m_id -= n;
    return *this;
}

inline
ExperienceGraph::node_iterator::difference_type
ExperienceGraph::node_iterator::operator-(node_iterator it)
{
    return (difference_type)m_id - (difference_type)it.m_id;
}

inline
bool ExperienceGraph::node_iterator::operator==(node_iterator it) const
{
    return m_id == it.m_id;
}

inline
bool ExperienceGraph::node_iterator::operator!=(node_iterator it) const
{
    return m_id != it.m_id;
}

inline
ExperienceGraph::edge_iterator::edge_iterator(edge_id id) :
    m_id(id)
{
}

inline
const ExperienceGraph::edge_iterator::value_type
ExperienceGraph::edge_iterator::operator*() const
{
    return m_id;
}

inline
ExperienceGraph::edge_iterator
ExperienceGraph::edge_iterator::operator++(int)
{
    return edge_iterator(m_id++);
}

inline
ExperienceGraph::edge_iterator&
ExperienceGraph::edge_iterator::operator++()
{
    ++m_id;
    return *this;
}

inline
ExperienceGraph::edge_iterator&
ExperienceGraph::edge_iterator::operator+=(difference_type n)
{
    m_id += n;
    return *this;
}

inline
ExperienceGraph::edge_iterator&
ExperienceGraph::edge_iterator::operator-=(difference_type n)
{
    m_id -= n;
    return *this;
}

inline
ExperienceGraph::edge_iterator::difference_type
ExperienceGraph::edge_iterator::operator-(edge_iterator it)
{
    return (difference_type)m_id - (difference_type)it.m_id;
}

inline
bool ExperienceGraph::edge_iterator::operator==(edge_iterator it) const
{
    return m_id == it.m_id;
}

inline
bool ExperienceGraph::edge_iterator::operator!=(edge_iterator it) const
{
    return m_id != it.m_id;
}

} // namespace smpl

#endif

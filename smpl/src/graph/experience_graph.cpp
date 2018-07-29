#include <smpl/graph/experience_graph.h>

#include <assert.h>
#include <algorithm>
#include <stdexcept>

namespace smpl {

ExperienceGraph::ExperienceGraph()
{
}

/// Return a pair of iterators to the range of nodes in the graph.
std::pair<
    ExperienceGraph::node_iterator,
    ExperienceGraph::node_iterator>
ExperienceGraph::nodes() const
{
    return std::make_pair(
            node_iterator(0),
            node_iterator(m_nodes.size()));
}

/// Return a pair of iterators to the range of edges in the graph.
std::pair<
    ExperienceGraph::edge_iterator,
    ExperienceGraph::edge_iterator>
ExperienceGraph::edges() const
{
    return std::make_pair(
            edge_iterator(0),
            edge_iterator(m_edges.size()));
}

/// Return a pair of iterators to the range of incident edges for a node.
std::pair<
    ExperienceGraph::incident_edge_iterator,
    ExperienceGraph::incident_edge_iterator>
ExperienceGraph::edges(node_id id) const
{
    return std::make_pair(
            incident_edge_iterator(m_nodes[id].edges.begin()),
            incident_edge_iterator(m_nodes[id].edges.end()));
}

/// Return a pair of iterators to the range of adjacent nodes for a node.
std::pair<
    ExperienceGraph::adjacency_iterator,
    ExperienceGraph::adjacency_iterator>
ExperienceGraph::adjacent_nodes(node_id id) const
{
    return std::make_pair(
            adjacency_iterator(m_nodes[id].edges.begin()),
            adjacency_iterator(m_nodes[id].edges.end()));
}

/// Return the degree (number of incident edges) of a node.
ExperienceGraph::degree_size_type ExperienceGraph::degree(node_id id) const
{
    return m_nodes[id].edges.size();
}

/// Return the id of an edge's source node.
ExperienceGraph::node_id ExperienceGraph::source(edge_id id) const
{
    return m_edges[id].snode;
}

/// Return the id of an edge's target node.
ExperienceGraph::node_id ExperienceGraph::target(edge_id id) const
{
    return m_edges[id].tnode;
}

/// Test if an edge exists between two nodes.
bool ExperienceGraph::edge(node_id uid, node_id vid) const
{
    if (uid >= m_nodes.size() || vid >= m_nodes.size()) {
        throw std::out_of_range("ExperienceGraph::edge called with invalid node ids");
    }

    if (m_nodes[uid].edges.size() < m_nodes[vid].edges.size()) {
        for (const auto& adj : m_nodes[uid].edges) {
            if (adj.second == vid) {
                return true;
            }
        }
    } else {
        for (const auto& adj : m_nodes[vid].edges) {
            if (adj.second == uid) {
                return true;
            }
        }
    }
    return false;
}

/// Insert a node.
ExperienceGraph::node_id ExperienceGraph::insert_node(const RobotState& state)
{
    m_nodes.emplace_back(state);
    return m_nodes.size() - 1;
}

/// Erase a node. Node iterators and ids after the erased node are invalidated.
/// Edge iterators and ids after the lowest removed incident edge are
/// invalidated. All incident edge and adjacenct node iterators are invalidated.
void ExperienceGraph::erase_node(node_id id)
{
    if (id >= m_nodes.size()) {
        throw std::out_of_range("ExperienceGraph::erase_node called with invalid node id");
    }

    const Node& rem_node = m_nodes[id];

    // the number of edges to be removed and the smallest id, for updating
    // adjacency edge ids
    if (rem_node.edges.size() > 0) {
        auto will_remove = [id](const Edge& e) {
            return e.snode == id | e.tnode == id;
        };

        // map from edge id to how many positions it will shift down
        m_shift.assign(m_edges.size(), 0);
        std::ptrdiff_t shift = 0;

        // simultaneously remove deleted edges and map from original edge ids
        // to the amount they must be shifted later
        auto first = std::find_if(m_edges.begin(), m_edges.end(), will_remove);
        if (first != m_edges.end()) {
            // for every element later in the sequence
            auto i = first;
            auto j = std::next(m_shift.begin(), std::distance(m_edges.begin(), first));
            ++shift;
            // i = remainder range iterator
            // j = corresponding shift array tracking following i
            for ( ; ++j, ++i != m_edges.end(); ) {
                *j = shift;
                if (!(will_remove(*i))) {
                    // shift this element and update the retained range
                    *first++ = std::move(*i);
                } else {
                    ++shift;
                }
            }
        }
        m_edges.erase(first, m_edges.end());

        for (auto& node : m_nodes) {
            // skip the node to be removed for a small optimization
            if (&node == &rem_node) {
                continue;
            }

            // remove adjacency entries
            auto it = std::remove_if(
                    node.edges.begin(), node.edges.end(),
                    [&](const Node::adjacency& a) {
                        return a.second == id;
                    });
            node.edges.resize(std::distance(node.edges.begin(), it));

            // update node and edge ids
            for (Node::adjacency& a : node.edges) {
                a.first -= m_shift[a.first];
                if (a.second > id) {
                    --a.second;
                }
            }
        }
    }

    // update the node ids stored in edges
    for (auto& edge : m_edges) {
        if (edge.snode > id) {
            --edge.snode;
        }
        if (edge.tnode > id) {
            --edge.tnode;
        }
    }

    // remove the node
    m_nodes.erase(m_nodes.begin() + id);
}

/// Insert an edge, allowing parallel edges and self-loops.
/// \return The ID of the inserted edge
ExperienceGraph::edge_id ExperienceGraph::insert_edge(node_id uid, node_id vid)
{
    if (uid >= m_nodes.size() || vid >= m_nodes.size()) {
        throw std::out_of_range("ExperienceGraph::insert_edge called with invalid node ids");
    }

    m_edges.emplace_back(uid, vid);
    ExperienceGraph::edge_id eid = m_edges.size() - 1;
    insert_incident_edge(eid, uid, vid);
    return eid;
}

/// Insert an edge, allowing parallel edges and self-loops.
/// \return The ID of the inserted edge
ExperienceGraph::edge_id ExperienceGraph::insert_edge(
    node_id uid,
    node_id vid,
    const std::vector<RobotState>& path)
{
    if (uid >= m_nodes.size() || vid >= m_nodes.size()) {
        throw std::out_of_range("ExperienceGraph::insert_edge called with invalid node ids");
    }

    m_edges.emplace_back(path, uid, vid);
    ExperienceGraph::edge_id eid = m_edges.size() - 1;
    insert_incident_edge(eid, uid, vid);
    return eid;
}

/// Erase the edge between two nodes, if one exists. All incident edge and
/// adjacent node iterators for the source and target nodes, edge iterators
/// pointing to edges after the erased edge, and edge ids greater than the
/// erased edge id are invalidated. TODO: erase all parallel edges?
void ExperienceGraph::erase_edge(node_id uid, node_id vid)
{
    if (uid >= m_nodes.size() || vid >= m_nodes.size()) {
        throw std::out_of_range("ExperienceGraph::erase_edge called with invalid node ids");
    }

    // search through the smaller of the two adjacency lists for the edge id
    // and call erase_edge on it when found
    if (m_nodes[uid].edges.size() < m_nodes[vid].edges.size()) {
        auto& out_edges = m_nodes[uid].edges;
        auto it = std::find_if(
                out_edges.begin(),
                out_edges.end(),
                [&](const Node::adjacency& a) {
                    return a.second == vid;
                });
        if (it != out_edges.end()) {
            erase_edge(it->first);
        }
    } else {
        auto& out_edges = m_nodes[vid].edges;
        auto it = std::find_if(
                out_edges.begin(),
                out_edges.end(),
                [&](const Node::adjacency& a) {
                    return a.second == uid;
                });
        if (it != out_edges.end()) {
            erase_edge(it->first);
        }
    }
}

/// Erase an edge. All incident edge and adjacent node iterators for the source
/// and target nodes, edge iterators pointing to edges after the erased edge,
/// and edge ids greater than the erased edge id are invalidated.
void ExperienceGraph::erase_edge(edge_id id)
{
    if (id >= m_edges.size()) {
        throw std::out_of_range("ExperienceGraph::erase_edge called with invalid edge id");
    }

    const Edge& e = m_edges[id];

    // remove incident edge from source node and update edge ids
    assert(e.snode < m_nodes.size());
    auto ait = std::remove_if(
            m_nodes[e.snode].edges.begin(),
            m_nodes[e.snode].edges.end(),
            [&](Node::adjacency& a) {
                bool rem = a.first == id;
                if (a.first > id) {
                    --a.first;
                }
                return rem;
            });
    m_nodes[e.snode].edges.erase(ait, m_nodes[e.snode].edges.end());

    // for non-self-loops, remove incident edge from target node and update edge
    // ids
    if (e.tnode != e.snode) {
        assert(e.tnode < m_nodes.size());
        ait = std::remove_if(
                m_nodes[e.tnode].edges.begin(),
                m_nodes[e.tnode].edges.end(),
                [&](Node::adjacency& a) {
                    bool rem = a.first == id;
                    if (a.first > id) {
                        --a.first;
                    }
                    return rem;
                });
        m_nodes[e.tnode].edges.erase(ait, m_nodes[e.tnode].edges.end());
    }

    // remove the edge
    m_edges.erase(std::next(m_edges.begin(), id));
}

inline
void ExperienceGraph::insert_incident_edge(edge_id eid, node_id uid, node_id vid)
{
    if (vid != uid) {
        m_nodes[uid].edges.emplace_back(eid, vid);
        m_nodes[vid].edges.emplace_back(eid, uid);
    } else {
        m_nodes[uid].edges.emplace_back(eid, vid);
    }
}

} // namespace smpl

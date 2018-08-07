#include <iostream>
#include <unordered_map>

#include <Eigen/Dense>
#define BOOST_TEST_MODULE OcTreeTest
#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>

#include <smpl/grid/sparse_grid.h>

BOOST_AUTO_TEST_CASE(DefaultConstructorTest)
{
    smpl::SparseGrid<int> g;
    BOOST_CHECK_EQUAL(g.max_depth(), 16);

    std::uint64_t max_coord = 1u << 16;
    BOOST_CHECK_EQUAL(g.size(), max_coord * max_coord * max_coord);

    BOOST_CHECK_EQUAL(g.tree().num_leaves(), 1);
    BOOST_CHECK_EQUAL(g.tree().num_nodes(), 1);
}

BOOST_AUTO_TEST_CASE(ValueConstructorTest)
{
    smpl::SparseGrid<int> g(8);
    BOOST_CHECK_EQUAL(g.get(0, 0, 0), 8);
}

BOOST_AUTO_TEST_CASE(SetSingleNodeTest)
{
    smpl::SparseGrid<int> g(0);

    BOOST_CHECK_EQUAL(g.tree().num_nodes(), 1);
    BOOST_CHECK_EQUAL(g.tree().num_leaves(), 1);

    g.set(0, 0, 0, 8);

    BOOST_CHECK_EQUAL(g.get(0, 0, 0), 8);
    BOOST_CHECK_EQUAL(g.get(0, 0, 1), 0);
    BOOST_CHECK_EQUAL(g.get(0, 1, 0), 0);
    BOOST_CHECK_EQUAL(g.get(0, 1, 1), 0);
    BOOST_CHECK_EQUAL(g.get(1, 0, 0), 0);
    BOOST_CHECK_EQUAL(g.get(1, 0, 1), 0);
    BOOST_CHECK_EQUAL(g.get(1, 1, 0), 0);
    BOOST_CHECK_EQUAL(g.get(1, 1, 1), 0);

    BOOST_CHECK_EQUAL(g.tree().num_leaves(), 15 * 7 + 8);
}

BOOST_AUTO_TEST_CASE(CopyConstructorTest)
{
    smpl::SparseGrid<int> g(0);
    g.set(0, 0, 0, 8);

    smpl::SparseGrid<int> cg(g);
    BOOST_CHECK_EQUAL(cg.get(0, 0, 0), 8);

    BOOST_CHECK_EQUAL(cg.size_x(), g.size_x());
    BOOST_CHECK_EQUAL(cg.size_y(), g.size_y());
    BOOST_CHECK_EQUAL(cg.size_z(), g.size_z());
    BOOST_CHECK_EQUAL(cg.size(), g.size());
    BOOST_CHECK_EQUAL(cg.max_depth(), g.max_depth());
}

BOOST_AUTO_TEST_CASE(MoveConstructorTest)
{
    smpl::SparseGrid<int> g(10);
    g.set(0, 0, 0, 8);

    smpl::SparseGrid<int> g2(std::move(g));

    BOOST_CHECK_EQUAL(g2.get(0, 0, 0), 8);
}

BOOST_AUTO_TEST_CASE(CopyAssignmentTest)
{
    smpl::SparseGrid<int> g1(10);

    smpl::SparseGrid<int> g2(8);
    g2.set(0, 0, 0, 8);

    g1 = g2;

    BOOST_CHECK_EQUAL(g1.get(0, 0, 0), g2.get(0, 0, 0));

    BOOST_CHECK_EQUAL(g1.tree().num_leaves(), g2.tree().num_leaves());
    BOOST_CHECK_EQUAL(g1.tree().num_nodes(), g2.tree().num_nodes());

    BOOST_CHECK_EQUAL(g1.size_x(), g2.size_x());
    BOOST_CHECK_EQUAL(g1.size_y(), g2.size_y());
    BOOST_CHECK_EQUAL(g1.size_z(), g2.size_z());
    BOOST_CHECK_EQUAL(g1.size(), g2.size());
    BOOST_CHECK_EQUAL(g1.max_depth(), g2.max_depth());
}

BOOST_AUTO_TEST_CASE(MoveAssignmentTest)
{
    smpl::SparseGrid<int> t1(10);

    smpl::SparseGrid<int> t2(8);
    t2.set(0, 0, 0, 6); // set a node to create an interesting memory scenario

    t1 = std::move(t2);

    BOOST_CHECK_EQUAL(t1.get(0, 0, 0), 6);
}

BOOST_AUTO_TEST_CASE(SetAndUnsetNodeTest)
{
    smpl::SparseGrid<int> g(0);

    g.set(0, 0, 0, 8);
    g.set(0, 0, 0, 0);

    BOOST_CHECK_EQUAL(g.tree().num_nodes(), 1);
    BOOST_CHECK_EQUAL(g.tree().num_leaves(), 1);
}

BOOST_AUTO_TEST_CASE(CheckerboardTest)
{
    smpl::SparseGrid<int> g(0);
}

BOOST_AUTO_TEST_CASE(ExtentsTest)
{
    auto max_coord = std::numeric_limits<std::uint16_t>::max();
    smpl::SparseGrid<int> g(0);

    g.set(0,         0,         0,         8);
    g.set(0,         0,         max_coord, 8);
    g.set(0,         max_coord, 0,         8);
    g.set(0,         max_coord, max_coord, 8);
    g.set(max_coord, 0,         0,         8);
    g.set(max_coord, 0,         max_coord, 8);
    g.set(max_coord, max_coord, 0,         8);
    g.set(max_coord, max_coord, max_coord, 8);

    BOOST_CHECK_EQUAL(g.tree().num_leaves(), 848);
}

BOOST_AUTO_TEST_CASE(ResetTest)
{
    smpl::SparseGrid<int> g(8);
    g.set(0, 0, 0, 6);

    g.reset(4);
    BOOST_CHECK_EQUAL(g.tree().num_leaves(), 1);
    BOOST_CHECK_EQUAL(g.tree().num_nodes(), 1);
    BOOST_CHECK_EQUAL(g.get(0, 0, 0), 4);
}

/// Test automatic compression on a small sparse grid
BOOST_AUTO_TEST_CASE(AutoCompressionTest)
{
    smpl::SparseGrid<int> g(8, 8, 8, 0);

    BOOST_CHECK_EQUAL(g.max_depth(), 3);
    BOOST_CHECK_EQUAL(g.size(), 8 * 8 * 8);

    for (int x = 0; x < 8; ++x) {
    for (int y = 0; y < 8; ++y) {
    for (int z = 0; z < 8; ++z) {
        g.set(x, y, z, 10);
    }
    }
    }

    BOOST_CHECK_EQUAL(g.tree().num_leaves(), 1);
    BOOST_CHECK_EQUAL(g.tree().num_nodes(), 1);
}

/// Test bounded depth for a smallish sparse grid
BOOST_AUTO_TEST_CASE(BoundedGridDepthTest)
{
    smpl::SparseGrid<int> g(1024, 1024, 1024);
    BOOST_CHECK_EQUAL(g.max_depth(), 10);
}

BOOST_AUTO_TEST_CASE(StringTreeTest)
{
    using StringGrid = smpl::SparseGrid<std::string>;
    StringGrid t;
    t.reset("test");
    BOOST_CHECK_EQUAL(t.get(0, 0, 0), "test");
}

BOOST_AUTO_TEST_CASE(LazySetNodeTest)
{
    smpl::SparseGrid<int> g(10);

    // enforce node creation
    int i = 0;
    for (int x = 0; x < 16; ++x) {
    for (int y = 0; y < 16; ++y) {
    for (int z = 0; z < 16; ++z) {
        g.set_lazy(x, y, z, i++);
    }
    }
    }

    for (int x = 0; x < 16; ++x) {
    for (int y = 0; y < 16; ++y) {
    for (int z = 0; z < 16; ++z) {
        g.set_lazy(x, y, z, 10);
    }
    }
    }

    BOOST_CHECK_NE(g.tree().num_nodes(), 1);
    BOOST_CHECK_NE(g.tree().num_leaves(), 1);

    g.prune();

    BOOST_CHECK_EQUAL(g.tree().num_nodes(), 1);
    BOOST_CHECK_EQUAL(g.tree().num_leaves(), 1);

    g.set(0, 0, 0, 8); // add a node back in
    g.prune();

    BOOST_CHECK_NE(g.tree().num_nodes(), 1);
    BOOST_CHECK_NE(g.tree().num_leaves(), 1);
}

BOOST_AUTO_TEST_CASE(IteratorTest)
{
    smpl::SparseGrid<int> g(13);

    auto max_coord = std::numeric_limits<std::uint16_t>::max();

    g.set(0,         0,         0,         8);
    g.set(0,         0,         max_coord, 8);
    g.set(0,         max_coord, 0,         8);
    g.set(0,         max_coord, max_coord, 8);
    g.set(max_coord, 0,         0,         8);
    g.set(max_coord, 0,         max_coord, 8);
    g.set(max_coord, max_coord, 0,         8);
    g.set(max_coord, max_coord, max_coord, 8);

    auto nodes = g.tree().nodes();

    // count the number of nodes with each value
    std::unordered_map<int, int> value_count;
    for (; nodes.first != nodes.second; ++nodes.first) {
        auto it = value_count.find(*nodes.first);
        if (it == value_count.end()) {
            value_count[*nodes.first] = 1;
        } else {
            ++it->second;
        }
    }

    for (const auto& entry : value_count) {
//        std::cout << entry.first << ": " << entry.second << std::endl;
    }
}

BOOST_AUTO_TEST_CASE(VisitorTest)
{
    auto dispatcher = [](int& v) { v = 8; };

    auto print = [](int v) { std::cout << v << std::endl; };

    smpl::SparseGrid<int> g(8);
    g.set(0, 0, 0, 4);

    g.accept(dispatcher);
    g.prune();

    BOOST_CHECK_EQUAL(g.tree().num_leaves(), 1);
    BOOST_CHECK_EQUAL(g.tree().num_nodes(), 1);

    g.accept(print);
}

BOOST_AUTO_TEST_CASE(AlignmentTest)
{
    smpl::SparseGrid<Eigen::Vector4d> g(Eigen::Vector4d::Zero());

    g.set(0, 0, 0, Eigen::Vector4d::Ones());

    // ensure alignment of octree nodes is not weaker than element type
    BOOST_CHECK_GE(
        alignof(smpl::detail::OcTreeNode<Eigen::Vector4d>),
        alignof(Eigen::Vector4d));
    BOOST_CHECK_GE(
        alignof(smpl::detail::OcTreeBase<
                Eigen::Vector4d,
                std::allocator<Eigen::Vector4d>>),
        alignof(Eigen::Vector4d));
}

BOOST_AUTO_TEST_CASE(LeafCoordsVisitorTest)
{
    auto max_coord = std::numeric_limits<std::uint16_t>::max();
    smpl::SparseGrid<int> g(0);
    g.set(0, 0, 0, 1);

//    g.set(0,         0,         0,         8);
//    g.set(0,         0,         max_coord, 8);
//    g.set(0,         max_coord, 0,         8);
//    g.set(0,         max_coord, max_coord, 8);
//    g.set(max_coord, 0,         0,         8);
//    g.set(max_coord, 0,         max_coord, 8);
//    g.set(max_coord, max_coord, 0,         8);
//    g.set(max_coord, max_coord, max_coord, 8);

    auto print_coords = [](int value, size_t first_x, size_t first_y, size_t first_z, size_t last_x, size_t last_y, size_t last_z)
    {
        std::cout << value << " @ [" << first_x << ", " << first_y << ", " << first_z << "] x [" << last_x << ", " << last_y << ", " << last_z << "]" << std::endl;
    };

//    g.accept_coords(print_coords);
}

BOOST_AUTO_TEST_CASE(CompareMemUsageTest)
{
    int scale = 2;
    const int xmax = scale * 512;
    const int ymax = scale * 512;
    const int zmax = scale * 80;
    smpl::SparseGrid<int> g(0);
    for (int x = 0; x < xmax; ++x) {
    for (int y = 0; y < ymax; ++y) {
    for (int z = 0; z < zmax; ++z) {
        if ((x == 0       ) | (y == 0       ) | (z == 0       )|
            (x == xmax - 1) | (y == ymax - 1) | (z == zmax - 1))
        {
            g.set(x, y, z, 1);
        }
    }
    }
    }

    auto comp_size = g.mem_usage();
    std::cout << "memory usage: " << std::endl;
    std::cout << comp_size << " B" << std::endl;
    std::cout << (double)comp_size / (1024.0) << " KB" << std::endl;
    std::cout << (double)comp_size / (1024.0 * 1024.0) << " MB" << std::endl;
    std::cout << (double)comp_size / (1024.0 * 1024.0 * 1024.0) << " GB" << std::endl;

    auto pred = [](int val) { return val == 1; };
    auto full_size = g.mem_usage_full(pred);

    std::cout << "memory usage full: " << std::endl;
    std::cout << full_size << " B" << std::endl;
    std::cout << (double)full_size / (1024.0) << " KB" << std::endl;
    std::cout << (double)full_size / (1024.0 * 1024.0) << " MB" << std::endl;
    std::cout << (double)full_size / (1024.0 * 1024.0 * 1024.0) << " GB" << std::endl;
}

BOOST_AUTO_TEST_CASE(ResizeTest)
{
    smpl::SparseGrid<int> g;
    g.resize(8, 8, 8);
    BOOST_CHECK_EQUAL(g.max_depth(), 3);
}

// TODO: Test throwing constructor/destructor

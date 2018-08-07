#include <iostream>

#define BOOST_TEST_MODULE BinaryGridTest
#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>

#include <smpl/grid/sparse_binary_grid.h>

BOOST_AUTO_TEST_CASE(DefaultConstructorTest)
{
    smpl::SparseBinaryGrid<> g;
    BOOST_CHECK_EQUAL(g.max_depth(), 15);

    std::uint64_t max_coord = 1u << 16;
    BOOST_CHECK_EQUAL(g.size(), max_coord * max_coord * max_coord);

    BOOST_CHECK_EQUAL(g.tree().num_leaves(), 1);
    BOOST_CHECK_EQUAL(g.tree().num_nodes(), 1);
}

BOOST_AUTO_TEST_CASE(ValueConstructorTest)
{
    smpl::SparseBinaryGrid<> g(true);
    for (int x = 0; x < 8; ++x) {
    for (int y = 0; y < 8; ++y) {
    for (int z = 0; z < 8; ++z) {
        BOOST_CHECK_EQUAL(g.get(x, y, z), true);
    }
    }
    }
}

BOOST_AUTO_TEST_CASE(SetSingleNodeTest)
{
    smpl::SparseBinaryGrid<> g(false);

    BOOST_CHECK_EQUAL(g.tree().num_nodes(), 1);
    BOOST_CHECK_EQUAL(g.tree().num_leaves(), 1);

    g.set(0, 0, 0, true);

    BOOST_CHECK_EQUAL(g.get(0, 0, 0), true);
    BOOST_CHECK_EQUAL(g.get(0, 0, 1), false);
    BOOST_CHECK_EQUAL(g.get(0, 1, 0), false);
    BOOST_CHECK_EQUAL(g.get(0, 1, 1), false);
    BOOST_CHECK_EQUAL(g.get(1, 0, 0), false);
    BOOST_CHECK_EQUAL(g.get(1, 0, 1), false);
    BOOST_CHECK_EQUAL(g.get(1, 1, 0), false);
    BOOST_CHECK_EQUAL(g.get(1, 1, 1), false);

    BOOST_CHECK_EQUAL(g.tree().num_leaves(), 106);
}

BOOST_AUTO_TEST_CASE(CopyConstructorTest)
{
    smpl::SparseBinaryGrid<> g(false);
    g.set(0, 0, 0, true);

    smpl::SparseBinaryGrid<> cg(g);
    BOOST_CHECK_EQUAL(cg.get(0, 0, 0), true);

    BOOST_CHECK_EQUAL(cg.size_x(), g.size_x());
    BOOST_CHECK_EQUAL(cg.size_y(), g.size_y());
    BOOST_CHECK_EQUAL(cg.size_z(), g.size_z());
    BOOST_CHECK_EQUAL(cg.size(), g.size());
    BOOST_CHECK_EQUAL(cg.max_depth(), g.max_depth());
}

BOOST_AUTO_TEST_CASE(MoveConstructorTest)
{
    smpl::SparseBinaryGrid<> g(false);
    g.set(0, 0, 0, true);

    smpl::SparseBinaryGrid<> g2(std::move(g));

    BOOST_CHECK_EQUAL(g2.get(0, 0, 0), true);
}

BOOST_AUTO_TEST_CASE(CopyAssignmentTest)
{
    smpl::SparseBinaryGrid<> g1(false);

    smpl::SparseBinaryGrid<> g2(false);
    g2.set(0, 0, 0, true);

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
    smpl::SparseBinaryGrid<> t1(true);

    smpl::SparseBinaryGrid<> t2(true);
    t2.set(0, 0, 0, false);

    t1 = std::move(t2);

    BOOST_CHECK_EQUAL(t1.get(0, 0, 0), false);
}

BOOST_AUTO_TEST_CASE(SetAndUnsetNodeTest)
{
    smpl::SparseBinaryGrid<> g(false);

    g.set(0, 0, 0, true);
    g.set(0, 0, 0, false);

    BOOST_CHECK_EQUAL(g.tree().num_nodes(), 1);
    BOOST_CHECK_EQUAL(g.tree().num_leaves(), 1);
}

BOOST_AUTO_TEST_CASE(CheckerboardTest)
{
    smpl::SparseBinaryGrid<> g(false);
}

BOOST_AUTO_TEST_CASE(ExtentsTest)
{
    auto max_coord = std::numeric_limits<std::uint16_t>::max();
    smpl::SparseBinaryGrid<> g(false);

    g.set(0,         0,         0,         true);
    g.set(0,         0,         max_coord, true);
    g.set(0,         max_coord, 0,         true);
    g.set(0,         max_coord, max_coord, true);
    g.set(max_coord, 0,         0,         true);
    g.set(max_coord, 0,         max_coord, true);
    g.set(max_coord, max_coord, 0,         true);
    g.set(max_coord, max_coord, max_coord, true);

    BOOST_CHECK_EQUAL(g.tree().num_leaves(), 792);
}

BOOST_AUTO_TEST_CASE(ResetTest)
{
    smpl::SparseBinaryGrid<> g(false);
    g.set(0, 0, 0, true);

    g.reset(false);
    BOOST_CHECK_EQUAL(g.tree().num_leaves(), 1);
    BOOST_CHECK_EQUAL(g.tree().num_nodes(), 1);
    BOOST_CHECK_EQUAL(g.get(0, 0, 0), false);
}

/// Test automatic compression on a small sparse grid
BOOST_AUTO_TEST_CASE(AutoCompressionTest)
{
    smpl::SparseBinaryGrid<> g(8, 8, 8, false);

    BOOST_CHECK_EQUAL(g.max_depth(), 2);
    BOOST_CHECK_EQUAL(g.size(), 8 * 8 * 8);

    for (int x = 0; x < 8; ++x) {
    for (int y = 0; y < 8; ++y) {
    for (int z = 0; z < 8; ++z) {
        g.set(x, y, z, true);
    }
    }
    }

    BOOST_CHECK_EQUAL(g.tree().num_leaves(), 1);
    BOOST_CHECK_EQUAL(g.tree().num_nodes(), 1);
}

/// Test bounded depth for a smallish sparse grid
BOOST_AUTO_TEST_CASE(BoundedGridDepthTest)
{
    smpl::SparseBinaryGrid<> g(1024, 1024, 1024);
    BOOST_CHECK_EQUAL(g.max_depth(), 9);
}

BOOST_AUTO_TEST_CASE(LazySetNodeTest)
{
    smpl::SparseBinaryGrid<> g(false);

    // enforce node creation
    for (int x = 0; x < 16; ++x) {
    for (int y = 0; y < 16; ++y) {
    for (int z = 0; z < 16; ++z) {
        g.set_lazy(x, y, z, true);
    }
    }
    }

    for (int x = 0; x < 16; ++x) {
    for (int y = 0; y < 16; ++y) {
    for (int z = 0; z < 16; ++z) {
        g.set_lazy(x, y, z, false);
    }
    }
    }

    BOOST_CHECK_NE(g.tree().num_nodes(), 1);
    BOOST_CHECK_NE(g.tree().num_leaves(), 1);

    g.prune();

    BOOST_CHECK_EQUAL(g.tree().num_nodes(), 1);
    BOOST_CHECK_EQUAL(g.tree().num_leaves(), 1);

    g.set(0, 0, 0, true); // add a node back in
    g.prune();

    BOOST_CHECK_NE(g.tree().num_nodes(), 1);
    BOOST_CHECK_NE(g.tree().num_leaves(), 1);
}

/*
BOOST_AUTO_TEST_CASE(VisitorTest)
{
    auto dispatcher = [](bool& v) { v = 8; };

    auto print = [](int v) { std::cout << v << std::endl; };

    smpl::SparseBinaryGrid<> g(false);
    g.set(0, 0, 0, true);

    g.accept(dispatcher);
    g.prune();

    BOOST_CHECK_EQUAL(g.tree().num_leaves(), 1);
    BOOST_CHECK_EQUAL(g.tree().num_nodes(), 1);

    g.accept(print);
}

BOOST_AUTO_TEST_CASE(LeafCoordsVisitorTest)
{
    auto max_coord = std::numeric_limits<std::uint16_t>::max();
    smpl::SparseBinaryGrid<int> g(0);
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
*/

BOOST_AUTO_TEST_CASE(CompareMemUsageTest)
{
    int scale = 2;
    const int xmax = scale * 512;
    const int ymax = scale * 512;
    const int zmax = scale * 80;
    smpl::SparseBinaryGrid<> g(false);
    for (int x = 0; x < xmax; ++x) {
    for (int y = 0; y < ymax; ++y) {
    for (int z = 0; z < zmax; ++z) {
        if ((x == 0       ) | (y == 0       ) | (z == 0       )|
            (x == xmax - 1) | (y == ymax - 1) | (z == zmax - 1))
        {
            g.set(x, y, z, true);
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

    // TODO: accept should accept SparseBinaryGrid reference and const_reference
    // instead of underlying type
    auto pred = [](std::uint8_t val) { return val != 0; };
    auto full_size = g.mem_usage_full(pred);

    std::cout << "memory usage full: " << std::endl;
    std::cout << full_size << " B" << std::endl;
    std::cout << (double)full_size / (1024.0) << " KB" << std::endl;
    std::cout << (double)full_size / (1024.0 * 1024.0) << " MB" << std::endl;
    std::cout << (double)full_size / (1024.0 * 1024.0 * 1024.0) << " GB" << std::endl;
}

BOOST_AUTO_TEST_CASE(ResizeTest)
{
    smpl::SparseBinaryGrid<> g;
    g.resize(8, 8, 8);
    BOOST_CHECK_EQUAL(g.max_depth(), 2);
}

// TODO: Test throwing constructor/destructor

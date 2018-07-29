#include <cstdint>
#include <string>
#include <utility>

#define BOOST_TEST_MODULE OcTreeTest
#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>

#include <smpl/grid/sparse_grid.h>

struct TrackedInt
{
    enum ConstructionFlags : std::uint8_t {
        CONSTRUCTED =      1 << 0,
        DESTROYED =        1 << 1,
        COPY_CONSTRUCTED = 1 << 2,
        MOVE_CONSTRUCTED = 1 << 3,
        MOVED =            1 << 4,
        ASSIGNED =         1 << 5,
        MOVE_ASSIGNED =    1 << 6,
    };

    int val;
    int count;  // track the number of copies since original construction

    std::uint8_t state;

    TrackedInt() : state(CONSTRUCTED) { }

    TrackedInt(int v) :
        val(v),
        count(0),
        state(CONSTRUCTED)
    { }

    ~TrackedInt()
    {
        state |= DESTROYED;
    }

    TrackedInt(const TrackedInt& o) :
        val(o.val),
        count(o.count + 1),
        state(COPY_CONSTRUCTED)
    {
    }

    TrackedInt(TrackedInt&& o) :
        val(o.val),
        count(o.count),
        state(MOVE_CONSTRUCTED)
    {
        o.state |= MOVED;
    }

    TrackedInt& operator=(const TrackedInt& rhs)
    {
        val = rhs.val;
        count = rhs.count;
        state |= ASSIGNED;
        return *this;
    }

    TrackedInt& operator=(TrackedInt&& rhs)
    {
        val = rhs.val;
        count = rhs.count;
        state |= MOVE_ASSIGNED;
        if (this != &rhs) {
            rhs.state |= MOVED;
        }
        return *this;
    }

    bool set(ConstructionFlags flags) const { return (bool)(state & flags); }

    operator int() const { return val; }
};

BOOST_AUTO_TEST_CASE(DefaultConstructorTest)
{
    smpl::OcTree<TrackedInt> tree;

    BOOST_CHECK(tree.root() != nullptr);
    BOOST_CHECK_EQUAL(tree.depth(), 0);
    BOOST_CHECK_EQUAL(tree.num_nodes(), 1);
    BOOST_CHECK_EQUAL(tree.num_leaves(), 1);

    BOOST_CHECK(tree.root()->children == nullptr);
    BOOST_CHECK(tree.root()->value.set(TrackedInt::CONSTRUCTED));
}

BOOST_AUTO_TEST_CASE(ValueConstructorTestCase)
{
    // 8 will be implicitly converted to a TrackedInt and the constructor will
    // be fed an rvalue reference to the temporary TrackedInt.
    smpl::OcTree<TrackedInt> tree(8);

    BOOST_CHECK(tree.root() != nullptr);
    BOOST_CHECK_EQUAL(tree.depth(), 0);
    BOOST_CHECK_EQUAL(tree.num_nodes(), 1);
    BOOST_CHECK_EQUAL(tree.num_leaves(), 1);

    BOOST_CHECK_EQUAL(tree.root()->value, 8);
    BOOST_CHECK(tree.root()->children == nullptr);

    BOOST_CHECK(tree.root()->value.set(TrackedInt::MOVE_CONSTRUCTED));
}

BOOST_AUTO_TEST_CASE(AllocatorConstructorTestCase)
{
    BOOST_CHECK(false);
}

BOOST_AUTO_TEST_CASE(ValueAllocatorConstructorTestCase)
{
    BOOST_CHECK(false);
}

BOOST_AUTO_TEST_CASE(ExpandNodeTest)
{
    // TODO: might want to add functionality both for creating children for
    // a node (possibly with constructor arguments), and creating children while
    // also copying the parent's value into them

    smpl::OcTree<TrackedInt> tree(8);
    tree.expand_node(tree.root());
    BOOST_CHECK(tree.root()->children != nullptr);

    auto* children = tree.root()->children;
    for (auto* child = children; child != children + 8; ++child) {
        BOOST_CHECK_EQUAL(child->value, 8);
        BOOST_CHECK(child->value.set(TrackedInt::COPY_CONSTRUCTED));
    }

    BOOST_CHECK_EQUAL(tree.depth(), 1);
    BOOST_CHECK_EQUAL(tree.num_nodes(), 1 + 8);
    BOOST_CHECK_EQUAL(tree.num_leaves(), 8);
}

BOOST_AUTO_TEST_CASE(CopyConstructorTest)
{
    smpl::OcTree<TrackedInt> tree(8);
    tree.expand_node(tree.root());

    smpl::OcTree<TrackedInt> copy(tree);

    BOOST_CHECK(copy.root() != nullptr);
    BOOST_CHECK_EQUAL(copy.root()->value, 8);
    BOOST_CHECK(copy.root()->children != nullptr);

    // ensure values are copied
    auto* orig_children = tree.root()->children;
    auto* copy_children = copy.root()->children;
    for (int i = 0; i < 8; ++i) {
        BOOST_CHECK_EQUAL(copy_children[i].value, orig_children[i].value);
        BOOST_CHECK(copy_children[i].value.set(TrackedInt::COPY_CONSTRUCTED));
    }

    BOOST_CHECK_EQUAL(copy.depth(), tree.depth());
    BOOST_CHECK_EQUAL(copy.num_nodes(), tree.num_nodes());
    BOOST_CHECK_EQUAL(copy.num_leaves(), tree.num_leaves());

    // TODO: test copy of allocators
}

BOOST_AUTO_TEST_CASE(MoveConstructorTest)
{
    smpl::OcTree<int> tree(8);
    tree.expand_node(tree.root());

    auto* orig_children = tree.root()->children;
    int orig_depth = tree.depth();
    auto orig_node_count = tree.num_nodes();
    auto orig_leaf_count = tree.num_leaves();

    // NOTE: root node's value and children are moved, but the root node itself
    // is not moved (can't test for identity)

    smpl::OcTree<int> m(std::move(tree));
    BOOST_CHECK(m.root() != nullptr);
    BOOST_CHECK_EQUAL(m.root()->value, 8);
    BOOST_CHECK(m.root()->children != nullptr);
    BOOST_CHECK_EQUAL(m.root()->children, orig_children);
    BOOST_CHECK_EQUAL(m.depth(), orig_depth);
    BOOST_CHECK_EQUAL(m.num_nodes(), orig_node_count);
    BOOST_CHECK_EQUAL(m.num_leaves(), orig_leaf_count);
}

BOOST_AUTO_TEST_CASE(CopyAssignmentTest)
{
    smpl::OcTree<TrackedInt> tree1(10);
    tree1.expand_node(tree1.root());

    smpl::OcTree<TrackedInt> tree2(8);
    tree2.expand_node(tree2.root());
    tree2.expand_node(tree2.root()->child(0));

    tree1 = tree2;

    BOOST_CHECK_EQUAL(tree1.depth(), tree2.depth());
    BOOST_CHECK_EQUAL(tree1.num_nodes(), tree2.num_nodes());
    BOOST_CHECK_EQUAL(tree1.num_leaves(), tree2.num_leaves());

    BOOST_CHECK_EQUAL(tree1.root()->value, tree2.root()->value);
    BOOST_CHECK_EQUAL(tree1.root()->child(0)->value, tree2.root()->child(0)->value);

    BOOST_CHECK(tree2.root()->value.set(TrackedInt::ASSIGNED));

    // TODO: test assignment of allocator (propagating and non-propagating)
}

BOOST_AUTO_TEST_CASE(MoveAssignmentTest)
{
    smpl::OcTree<int> tree1(10);
    tree1.expand_node(tree1.root());

    auto* orig_children = tree1.root()->children;
    int orig_depth = tree1.depth();
    auto orig_node_count = tree1.num_nodes();
    auto orig_leaf_count = tree1.num_leaves();

    smpl::OcTree<int> tree2(8);

    tree2 = std::move(tree1);
    BOOST_CHECK(tree2.root() != nullptr);
    BOOST_CHECK_EQUAL(tree2.root()->value, 10);
    BOOST_CHECK(tree2.root()->children != nullptr);
    BOOST_CHECK_EQUAL(tree2.root()->children, orig_children);

    // TODO: test move assignment of allocator (propagating and non-propagating)

    // TODO: test that tree2's root element was assigned via move assignment
    // from tree1's root element

    // TODO: test that destruction of tree1 in no way interferes with tree2
}

BOOST_AUTO_TEST_CASE(ClearTest)
{
    smpl::OcTree<int> tree(10);

    tree.expand_node(tree.root());
    tree.expand_node(tree.root()->child(0));

    tree.clear();

    BOOST_CHECK_EQUAL(tree.depth(), 0);
    BOOST_CHECK_EQUAL(tree.num_nodes(), 1);
    BOOST_CHECK_EQUAL(tree.num_leaves(), 1);
}

BOOST_AUTO_TEST_CASE(StringTreeTest)
{
    smpl::OcTree<std::string> tree("cafef00d");
}

BOOST_AUTO_TEST_CASE(SizeTest)
{
    // ensure that empty base class optimization is being employed to remove
    // empty allocator
    BOOST_CHECK_EQUAL(
            sizeof(smpl::detail::OcTreeBase<int, std::allocator<int>>),
            sizeof(smpl::detail::OcTreeNode<int>));

    BOOST_CHECK_EQUAL(
            sizeof(smpl::OcTree<int>), sizeof(smpl::detail::OcTreeNode<int>));
}

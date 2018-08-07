#ifndef SMPL_SPARSE_BINARY_GRID_DETAIL_H
#define SMPL_SPARSE_BINARY_GRID_DETAIL_H

#include "../sparse_binary_grid.h"

#include <stdio.h>

namespace smpl {

template <class Allocator>
SparseBinaryGrid<Allocator>::SparseBinaryGrid() :
    m_grid(reduced(1 << 16), reduced(1 << 16), reduced(1 << 16))
{
    m_osize[0] = m_osize[1] = m_osize[2] = 1u << 16;
}

template <class Allocator>
SparseBinaryGrid<Allocator>::SparseBinaryGrid(bool value) :
    m_grid(reduced(1 << 16), reduced(1 << 16), reduced(1 << 16), initval(value))
{
    m_osize[0] = m_osize[1] = m_osize[2] = 1u << 16;
}

template <class Allocator>
SparseBinaryGrid<Allocator>::SparseBinaryGrid(
    size_type size_x,
    size_type size_y,
    size_type size_z)
:
    m_grid(reduced(size_x), reduced(size_y), reduced(size_z))
{
    m_osize[0] = size_x;
    m_osize[1] = size_y;
    m_osize[2] = size_z;
}

template <class Allocator>
SparseBinaryGrid<Allocator>::SparseBinaryGrid(
    size_type size_x,
    size_type size_y,
    size_type size_z,
    bool value)
:
    m_grid(reduced(size_x), reduced(size_y), reduced(size_z), initval(value))
{
    m_osize[0] = size_x;
    m_osize[1] = size_y;
    m_osize[2] = size_z;
}

template <class Allocator>
SparseBinaryGrid<Allocator>::SparseBinaryGrid(const Allocator& alloc) :
    m_grid(reduced(1 << 16), reduced(1 << 16), reduced(1 << 16), alloc)
{
    m_osize[0] = m_osize[1] = m_osize[2] = 1u << 16;
}

template <class Allocator>
SparseBinaryGrid<Allocator>::SparseBinaryGrid(bool value, const Allocator& alloc) :
    m_grid(initval(value), alloc)
{
    m_osize[0] = m_osize[1] = m_osize[2] = 1u << 16;
}

template <class Allocator>
SparseBinaryGrid<Allocator>::SparseBinaryGrid(
    size_type size_x,
    size_type size_y,
    size_type size_z,
    const Allocator& alloc)
:
    m_grid(size_x, size_y, size_z, alloc)
{
    m_osize[0] = size_x;
    m_osize[1] = size_y;
    m_osize[2] = size_z;
}

template <class Allocator>
SparseBinaryGrid<Allocator>::SparseBinaryGrid(
    size_type size_x,
    size_type size_y,
    size_type size_z,
    bool value,
    const Allocator& alloc)
:
    m_grid(size_x, size_y, size_z, initval(value), alloc)
{
    m_osize[0] = size_x;
    m_osize[1] = size_y;
    m_osize[2] = size_z;
}

template <class Allocator>
auto SparseBinaryGrid<Allocator>::size() const -> size_type
{
    return size_x() * size_y() * size_z();
}

template <class Allocator>
auto SparseBinaryGrid<Allocator>::max_size() const -> size_type
{
    return m_grid.max_size();
}

template <class Allocator>
auto SparseBinaryGrid<Allocator>::size_x() const -> size_type
{
    return m_osize[0];
}

template <class Allocator>
auto SparseBinaryGrid<Allocator>::size_y() const -> size_type
{
    return m_osize[1];
}

template <class Allocator>
auto SparseBinaryGrid<Allocator>::size_z() const -> size_type
{
    return m_osize[2];
}

template <class Allocator>
int SparseBinaryGrid<Allocator>::max_depth() const
{
    return m_grid.max_depth();
}

template <class Allocator>
auto SparseBinaryGrid<Allocator>::mem_usage() const -> size_type
{
    return m_grid.mem_usage();
}

template <class Allocator>
auto SparseBinaryGrid<Allocator>::operator()(
    index_type x,
    index_type y,
    index_type z) const -> const_reference
{
    std::uint8_t mask = get_mask(x, y, z);

    x >>= 1;
    y >>= 1;
    z >>= 1;

    std::uint8_t curr = m_grid(x, y, z);

    return (bool)(curr & mask);
}

template <class Allocator>
auto SparseBinaryGrid<Allocator>::operator()(
    index_type x,
    index_type y,
    index_type z) -> reference
{
    std::uint8_t mask = get_mask(x, y, z);

    x >>= 1;
    y >>= 1;
    z >>= 1;

    std::uint8_t &curr = m_grid(x, y, z);
    return { &curr, mask };
}

template <class Allocator>
auto SparseBinaryGrid<Allocator>::get(
    index_type x,
    index_type y,
    index_type z) const -> const_reference
{
    return this->operator()(x, y, z);
}

template <class Allocator>
void SparseBinaryGrid<Allocator>::reset(bool value)
{
    m_grid.reset(initval(value));
}

template <class Allocator>
void SparseBinaryGrid<Allocator>::assign(bool value)
{
    m_grid.assign(initval(value));
}

template <class Allocator>
void SparseBinaryGrid<Allocator>::set(
    index_type x,
    index_type y,
    index_type z,
    bool data)
{
    std::uint8_t mask = get_mask(x, y, z);
    x >>= 1;
    y >>= 1;
    z >>= 1;

    std::uint8_t curr = m_grid.get(x, y, z);
    std::uint8_t next = data ? (curr | mask) : (curr & ~mask);
    // TODO(Andrew): should be able to check this within the call to set_lazy
    if (curr != next) {
        m_grid.set(x, y, z, next);
    }
}

template <class Allocator>
void SparseBinaryGrid<Allocator>::set_lazy(
    index_type x,
    index_type y,
    index_type z,
    bool data)
{
    std::uint8_t mask = get_mask(x, y, z);
    x >>= 1;
    y >>= 1;
    z >>= 1;
    std::uint8_t curr = m_grid.get(x, y ,z);
    std::uint8_t next = data ? (curr | mask) : (curr & ~mask);
    if (curr != next) {
        m_grid.set_lazy(x, y, z, next);
    }
}

template <class Allocator>
void SparseBinaryGrid<Allocator>::prune()
{
    m_grid.prune();
}

template <class Allocator>
template <class UnaryPredicate>
void SparseBinaryGrid<Allocator>::prune(UnaryPredicate p)
{
    m_grid.prune(p);
}

template <class Allocator>
void SparseBinaryGrid<Allocator>::resize(
    size_type size_x,
    size_type size_y,
    size_type size_z)
{
    m_grid.resize(reduced(size_x), reduced(size_y), reduced(size_z));
}

template <class Allocator>
void SparseBinaryGrid<Allocator>::resize(
    size_type size_x,
    size_type size_y,
    size_type size_z,
    bool value)
{
    m_grid.resize(reduced(size_x), reduced(size_y), reduced(size_z), initval(value));
}

template <class Allocator>
template <class Pred>
auto SparseBinaryGrid<Allocator>::mem_usage_full(const Pred& pred) -> size_type
{
    return m_grid.mem_usage_full(pred);
}

template <class Allocator>
template <typename Callable>
void SparseBinaryGrid<Allocator>::accept_coords(Callable c)
{
    return m_grid.accept_coords(c);
}

template <class Allocator>
std::uint8_t SparseBinaryGrid<Allocator>::initval(bool value) const
{
    return value ? 0xFF : 0x00;
}

template <class Allocator>
auto SparseBinaryGrid<Allocator>::reduced(size_type s) const -> size_type
{
    return (s >> 1) + (s & 0x00000001);
}

template <class Allocator>
std::uint8_t SparseBinaryGrid<Allocator>::get_mask(int x, int y, int z) const
{
    std::uint8_t cx = 0x01 & x;
    std::uint8_t cy = 0x01 & y;
    std::uint8_t cz = 0x01 & z;

    return 1 << ((cx << 2) | (cy << 1) | (cz));
}

} // namespace smpl

#endif

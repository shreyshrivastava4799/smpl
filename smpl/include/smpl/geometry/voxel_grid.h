//////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2015, Andrew Dornbush
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the copyright holder nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
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
//////////////////////////////////////////////////////////////////////////////

#ifndef SMPL_VOXEL_GRID_H
#define SMPL_VOXEL_GRID_H

// standard includes
#include <vector>

// project includes
#include <smpl/geometry/discretize.h>
#include <smpl/spatial.h>

namespace smpl {
namespace geometry {

struct MemoryIndex
{
    int idx;

    MemoryIndex() : idx() { }
    MemoryIndex(int idx) : idx(idx) { }
};

struct MemoryCoord
{
    int x;
    int y;
    int z;

    MemoryCoord() : x(), y(), z() { }
    MemoryCoord(int x, int y, int z) : x(x), y(y), z(z) { }
};

struct GridCoord
{
    int x;
    int y;
    int z;

    GridCoord() : x(), y(), z() { }
    GridCoord(int x, int y, int z) : x(x), y(y), z(z) { }
};

struct WorldCoord
{
    double x;
    double y;
    double z;

    WorldCoord() : x(), y(), z() { }
    WorldCoord(double x, double y, double z) : x(x), y(y), z(z) { }
};

class VoxelGridBase
{
public:

    typedef unsigned char value_type;

    VoxelGridBase(
        const Vector3& origin,
        const Vector3& size,
        const Vector3& res)
    :
        m_origin(origin),
        m_size(size),
        m_res(res)
    {
        m_min = origin;
        m_max = origin + size;
    }

    VoxelGridBase(
        const Vector3& min,
        const Vector3& max,
        const Vector3& res,
        int)
    {
        m_min = min;
        m_max = max;
        m_origin = min;
        m_size = max - min;
        m_res = res;
    }

    const Vector3& origin() const { return m_origin; }
    const Vector3& size() const { return m_size; }
    const Vector3& res() const { return m_res; }

protected:

    Vector3 m_min;
    Vector3 m_max;
    Vector3 m_origin;
    Vector3 m_size;
    Vector3 m_res;

    std::vector<value_type> m_grid;
};

template <class Discretizer>
class VoxelGrid : public VoxelGridBase
{
public:

    typedef VoxelGridBase Base;
    typedef Base::value_type value_type;

    VoxelGrid(
        const Vector3& origin,
        const Vector3& size,
        const Vector3& res,
        const Discretizer& x_disc,
        const Discretizer& y_disc,
        const Discretizer& z_disc);

    VoxelGrid(
        const Vector3& min,
        const Vector3& max,
        const Vector3& res,
        const Discretizer& x_disc,
        const Discretizer& y_disc,
        const Discretizer& z_disc,
        int d)
    :
        Base(min, max, res, d),
        m_x_disc(x_disc),
        m_y_disc(y_disc),
        m_z_disc(z_disc)
    {
        m_min_gc.x = m_x_disc.discretize(min.x());
        m_min_gc.y = m_y_disc.discretize(min.y());
        m_min_gc.z = m_z_disc.discretize(min.z());

        m_max_gc.x = m_x_disc.discretize(max.x());
        m_max_gc.y = m_y_disc.discretize(max.y());
        m_max_gc.z = m_z_disc.discretize(max.z());

        m_grid.resize(sizeX() * sizeY() * sizeZ(), false);
    }

    int sizeX() const { return m_max_gc.x - m_min_gc.x + 1; }
    int sizeY() const { return m_max_gc.y - m_min_gc.y + 1; }
    int sizeZ() const { return m_max_gc.z - m_min_gc.z + 1; }

    value_type& operator()(const MemoryIndex& index);
    value_type& operator()(const MemoryCoord& coord);
    value_type& operator()(const GridCoord& coord);
    value_type& operator()(const WorldCoord& coord);
    value_type& operator[](const MemoryIndex& index);
    value_type& operator[](const MemoryCoord& coord);
    value_type& operator[](const GridCoord& coord);
    value_type& operator[](const WorldCoord& coord);

    value_type operator()(const MemoryIndex& index) const;
    value_type operator()(const MemoryCoord& coord) const;
    value_type operator()(const GridCoord& coord) const;
    value_type operator()(const WorldCoord& coord) const;
    value_type operator[](const MemoryIndex& index) const;
    value_type operator[](const MemoryCoord& coord) const;
    value_type operator[](const GridCoord& coord) const;
    value_type operator[](const WorldCoord& coord) const;

    MemoryIndex memoryToIndex(const MemoryCoord& coord) const;
    MemoryIndex gridToIndex(const GridCoord& coord) const;
    MemoryIndex worldToIndex(const WorldCoord& coord) const;
    MemoryCoord indexToMemory(const MemoryIndex& index) const;
    MemoryCoord gridToMemory(const GridCoord& coord) const;
    MemoryCoord worldToMemory(const WorldCoord& coord) const;
    GridCoord   indexToGrid(const MemoryIndex& index) const;
    GridCoord   memoryToGrid(const MemoryCoord& coord) const;
    GridCoord   worldToGrid(const WorldCoord& coord) const;
    WorldCoord  indexToWorld(const MemoryIndex& index) const;
    WorldCoord  memoryToWorld(const MemoryCoord& coord) const;
    WorldCoord  gridToWorld(const GridCoord& coord) const;

protected:

    GridCoord m_min_gc;
    GridCoord m_max_gc;

    Discretizer m_x_disc;
    Discretizer m_y_disc;
    Discretizer m_z_disc;
};

template <typename Discretizer>
VoxelGrid<Discretizer>::VoxelGrid(
    const Vector3& origin,
    const Vector3& size,
    const Vector3& res,
    const Discretizer& x_disc,
    const Discretizer& y_disc,
    const Discretizer& z_disc)
:
    VoxelGridBase(origin, size, res),
    m_x_disc(x_disc),
    m_y_disc(y_disc),
    m_z_disc(z_disc)
{
    m_min_gc.x = m_x_disc.discretize(origin.x());
    m_min_gc.y = m_y_disc.discretize(origin.y());
    m_min_gc.z = m_z_disc.discretize(origin.z());

    m_max_gc.x = m_x_disc.discretize(origin.x() + size.x());
    m_max_gc.y = m_y_disc.discretize(origin.y() + size.y());
    m_max_gc.z = m_z_disc.discretize(origin.z() + size.z());

    m_grid.resize(sizeX() * sizeY() * sizeZ(), false);
}

template <typename Discretizer>
typename VoxelGrid<Discretizer>::value_type&
VoxelGrid<Discretizer>::operator()(const MemoryIndex& index)
{
    assert(index.idx >= 0 && index.idx < m_grid.size());
    return m_grid[index.idx];
}

template <typename Discretizer>
typename VoxelGrid<Discretizer>::value_type&
VoxelGrid<Discretizer>::operator()(const MemoryCoord& coord)
{
    return this->operator()(memoryToIndex(coord));
}

template <typename Discretizer>
typename VoxelGrid<Discretizer>::value_type&
VoxelGrid<Discretizer>::operator()(const GridCoord& coord)
{
    return this->operator()(gridToIndex(coord));
}

template <typename Discretizer>
typename VoxelGrid<Discretizer>::value_type&
VoxelGrid<Discretizer>::operator()(const WorldCoord& coord)
{
    return this->operator()(worldToIndex(coord));
}

template <typename Discretizer>
typename VoxelGrid<Discretizer>::value_type&
VoxelGrid<Discretizer>::operator[](const MemoryIndex& index)
{
    return this->operator()(index);
}

template <typename Discretizer>
typename VoxelGrid<Discretizer>::value_type&
VoxelGrid<Discretizer>::operator[](const MemoryCoord& coord)
{
    return this->operator()(coord);
}

template <typename Discretizer>
typename VoxelGrid<Discretizer>::value_type&
VoxelGrid<Discretizer>::operator[](const GridCoord& coord)
{
    return this->operator()(coord);
}

template <typename Discretizer>
typename VoxelGrid<Discretizer>::value_type&
VoxelGrid<Discretizer>::operator[](const WorldCoord& coord)
{
    return this->operator()(coord);
}

template <typename Discretizer>
typename VoxelGrid<Discretizer>::value_type
VoxelGrid<Discretizer>::operator()(const MemoryIndex& index) const
{
    assert(index.idx >= 0 && index.idx < m_grid.size());
    return m_grid[index.idx];
}

template <typename Discretizer>
typename VoxelGrid<Discretizer>::value_type
VoxelGrid<Discretizer>::operator()(const MemoryCoord& coord) const
{
    return this->operator()(memoryToIndex(coord));
}

template <typename Discretizer>
typename VoxelGrid<Discretizer>::value_type
VoxelGrid<Discretizer>::operator()(const GridCoord& coord) const
{
    return this->operator()(gridToIndex(coord));
}

template <typename Discretizer>
typename VoxelGrid<Discretizer>::value_type
VoxelGrid<Discretizer>::operator()(const WorldCoord& coord) const
{
    return this->operator()(worldToIndex(coord));
}

template <typename Discretizer>
typename VoxelGrid<Discretizer>::value_type
VoxelGrid<Discretizer>::operator[](const MemoryIndex& index) const
{
    return this->operator()(index);
}

template <typename Discretizer>
typename VoxelGrid<Discretizer>::value_type
VoxelGrid<Discretizer>::operator[](const MemoryCoord& coord) const
{
    return this->operator()(coord);
}

template <typename Discretizer>
typename VoxelGrid<Discretizer>::value_type
VoxelGrid<Discretizer>::operator[](const GridCoord& coord) const
{
    return this->operator()(coord);
}

template <typename Discretizer>
typename VoxelGrid<Discretizer>::value_type
VoxelGrid<Discretizer>::operator[](const WorldCoord& coord) const
{
    return this->operator()(coord);
}

template <typename Discretizer>
MemoryIndex
VoxelGrid<Discretizer>::memoryToIndex(const MemoryCoord& coord) const
{
    return MemoryIndex(coord.x * sizeY() * sizeZ() + coord.y * sizeZ() + coord.z);
}

template <typename Discretizer>
MemoryIndex
VoxelGrid<Discretizer>::gridToIndex(const GridCoord& coord) const
{
    return memoryToIndex(gridToMemory(coord));
}

template <typename Discretizer>
MemoryIndex
VoxelGrid<Discretizer>::worldToIndex(const WorldCoord& coord) const
{
    return memoryToIndex(gridToMemory(worldToGrid()));
}

template <typename Discretizer>
MemoryCoord
VoxelGrid<Discretizer>::indexToMemory(const MemoryIndex& index) const
{
    int x = index.idx / (sizeZ() * sizeY());
    int y = (index.idx - x * (sizeZ() * sizeY())) / sizeZ();
    int z = index.idx - (x * sizeZ() * sizeY()) - (y * sizeZ());
    return MemoryCoord(x, y, z);
}

template <typename Discretizer>
MemoryCoord
VoxelGrid<Discretizer>::gridToMemory(const GridCoord& coord) const
{
    const int x = coord.x - m_min_gc.x;
    const int y = coord.y - m_min_gc.y;
    const int z = coord.z - m_min_gc.z;
    return MemoryCoord(x, y, z);
}

template <typename Discretizer>
MemoryCoord
VoxelGrid<Discretizer>::worldToMemory(const WorldCoord& coord) const
{
    return gridToMemory(worldToGrid(coord));
}

template <typename Discretizer>
GridCoord
VoxelGrid<Discretizer>::indexToGrid(const MemoryIndex& index) const
{
    return memoryToGrid(indexToMemory(index));
}

template <typename Discretizer>
GridCoord
VoxelGrid<Discretizer>::memoryToGrid(const MemoryCoord& coord) const
{
    int x = m_min_gc.x + coord.x;
    int y = m_min_gc.y + coord.y;
    int z = m_min_gc.z + coord.z;
    return GridCoord(x, y, z);
}

template <typename Discretizer>
GridCoord
VoxelGrid<Discretizer>::worldToGrid(const WorldCoord& coord) const
{
    return GridCoord(
            m_x_disc.discretize(coord.x),
            m_y_disc.discretize(coord.y),
            m_z_disc.discretize(coord.z));
}

template <typename Discretizer>
WorldCoord
VoxelGrid<Discretizer>::indexToWorld(const MemoryIndex& index) const
{
    return gridToWorld(indexToGrid(index));
}

template <typename Discretizer>
WorldCoord
VoxelGrid<Discretizer>::memoryToWorld(const MemoryCoord& coord) const
{
    return gridToWorld(memoryToGrid(coord));
}

template <typename Discretizer>
WorldCoord
VoxelGrid<Discretizer>::gridToWorld(const GridCoord& coord) const
{
    const double x = m_x_disc.continuize(coord.x);
    const double y = m_y_disc.continuize(coord.y);
    const double z = m_z_disc.continuize(coord.z);
    return WorldCoord(x, y, z);
}

//////////////////////
// MinDiscVoxelGrid //
//////////////////////

class HalfResVoxelGrid : public VoxelGrid<HalfResDiscretizer>
{
public:

    HalfResVoxelGrid(
        const Vector3& origin,
        const Vector3& size,
        const Vector3& res)
    :
        VoxelGrid(
            origin, size, res,
            HalfResDiscretizer(res.x()),
            HalfResDiscretizer(res.y()),
            HalfResDiscretizer(res.z()))
    { }
};

class PivotVoxelGrid : public VoxelGrid<PivotDiscretizer>
{
public:

    PivotVoxelGrid(
        const Vector3& origin,
        const Vector3& size,
        const Vector3& res,
        const Vector3& pivot)
    :
        VoxelGrid(
            origin, size, res,
            PivotDiscretizer(res.x(), pivot.x()),
            PivotDiscretizer(res.y(), pivot.y()),
            PivotDiscretizer(res.z(), pivot.z()))
    { }

    PivotVoxelGrid(
        const Vector3& min,
        const Vector3& max,
        const Vector3& res,
        const Vector3& pivot,
        int d)
    :
        VoxelGrid(
            min, max, res,
            PivotDiscretizer(res.x(), pivot.x()),
            PivotDiscretizer(res.y(), pivot.y()),
            PivotDiscretizer(res.z(), pivot.z()),
            d)
    { }
};

} // namespace geometry
} // namespace smpl

#endif

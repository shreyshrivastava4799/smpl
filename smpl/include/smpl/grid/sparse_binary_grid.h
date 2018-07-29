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

#ifndef SMPL_SPARSE_BINARY_GRID_H
#define SMPL_SPARSE_BINARY_GRID_H

// standard includes
#include <cstdint>
#include <memory>

// project includes
#include <smpl/grid/sparse_grid.h>

namespace smpl {

template <class Allocator = std::allocator<std::uint8_t>>
class SparseBinaryGrid
{
    struct packed_bool_ref;

public:

    using GridType = SparseGrid<std::uint8_t, Allocator>;
    using TreeType = typename GridType::TreeType;

    using value_type        = bool;
    using size_type         = typename GridType::size_type;
    using reference         = packed_bool_ref;
    using const_reference   = bool;
    using index_type        = int;

    SparseBinaryGrid();
    SparseBinaryGrid(bool value);
    SparseBinaryGrid(size_type size_x, size_type size_y, size_type size_z);
    SparseBinaryGrid(
        size_type size_x, size_type size_y, size_type size_z,
        bool value);

    explicit SparseBinaryGrid(const Allocator& alloc);
    explicit SparseBinaryGrid(bool, const Allocator& alloc);
    explicit SparseBinaryGrid(
        size_type size_x, size_type size_y, size_type size_z,
        const Allocator& alloc);
    explicit SparseBinaryGrid(
        size_type size_x, size_type size_y, size_type size_z,
        bool value, const Allocator& alloc);

    /// \name Size Properties
    ///@{
    size_type size() const;
    size_type max_size() const;
    size_type size_x() const;
    size_type size_y() const;
    size_type size_z() const;

    int max_depth() const;

    size_type mem_usage() const;
    ///@}

    /// \name Element Access
    ///@{
    const_reference operator()(index_type x, index_type y, index_type z) const;

    reference operator()(index_type x, index_type y, index_type z);

    const_reference get(index_type x, index_type y, index_type z) const;
    ///@}

    /// \name Modifiers
    ///@{
    void reset(bool value);
    void assign(bool value);

    void set(index_type x, index_type y, index_type z, bool data);
    void set_lazy(index_type x, index_type y, index_type z, bool data);

    void prune();

    template <class UnaryPredicate>
    void prune(UnaryPredicate p);

    void resize(size_type size_x, size_type size_y, size_type size_z);
    void resize(size_type size_x, size_type size_y, size_type size_z, bool value);
    ///@}

    template <class Pred>
    size_type mem_usage_full(const Pred& pred);

    template <typename Callable>
    void accept_coords(Callable c);

    const TreeType& tree() const { return m_grid.tree(); }

private:

    struct packed_bool_ref
    {
        std::uint8_t*   fptr; // pointer to field
        std::uint8_t    mask; // offset into field

        operator bool() const { return (bool)(*fptr & mask); }

        packed_bool_ref& operator=(bool rhs) {
            if (rhs) {
                *fptr |= mask;
            } else {
                *fptr &= ~mask;
            }
            return *this;
        }
    };

    SparseGrid<std::uint8_t, Allocator> m_grid;
    size_type m_osize[3];

    std::uint8_t initval(bool value) const;
    size_type reduced(size_type s) const;
    std::uint8_t get_mask(int x, int y, int z) const;
};

} // namespace smpl

#include "detail/sparse_binary_grid.hpp"

#endif

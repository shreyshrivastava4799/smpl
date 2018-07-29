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

#ifndef SBPL_COLLISION_CHECKING_WORLD_COLLISION_MODEL_H
#define SBPL_COLLISION_CHECKING_WORLD_COLLISION_MODEL_H

// standard includes
#include <memory>
#include <string>
#include <vector>

// system includes
#include <Eigen/Dense>
#include <smpl/occupancy_grid.h>
#include <visualization_msgs/MarkerArray.h>

namespace smpl {
namespace collision {

struct CollisionObject;

class WorldCollisionModel
{
public:

    WorldCollisionModel(OccupancyGrid* grid);
    WorldCollisionModel(const WorldCollisionModel& o, OccupancyGrid* grid);

    auto grid() -> OccupancyGrid* { return m_grid; }
    auto grid() const -> const OccupancyGrid* { return m_grid; }

    bool insertObject(const CollisionObject* object);
    bool removeObject(const CollisionObject* object);

    bool moveShapes(const CollisionObject* object);
    bool insertShapes(const CollisionObject* object);
    bool removeShapes(const CollisionObject* object);

    bool hasObject(const CollisionObject* object) const;
    bool hasObjectWithName(const std::string& id) const;

    void removeAllObjects();

    void reset();

    auto getWorldVisualization() const -> visualization_msgs::MarkerArray;
    auto getCollisionWorldVisualization() const -> visualization_msgs::MarkerArray;

    void setPadding(double padding) { m_padding = padding; }
    double padding() const { return m_padding; }

private:

    OccupancyGrid* m_grid;

    using VoxelList = std::vector<Eigen::Vector3d>;
    struct ObjectCollisionModel {
        const CollisionObject* object;

        // occupied voxels for this object in the grid reference frame, one
        // list for each shape in the object
        std::vector<VoxelList> cached_voxels;
    };
    std::vector<ObjectCollisionModel> m_object_models;

    double m_padding;

    ////////////////////
    // Generic Shapes //
    ////////////////////

    bool haveObject(const CollisionObject* object) const;

    auto getObjectCollisionModel(const CollisionObject* object) const
        -> const ObjectCollisionModel*;

    bool checkObjectInsert(const CollisionObject* object) const;
    bool checkObjectRemove(const CollisionObject* object) const;
    bool checkObjectMoveShape(const CollisionObject* object) const;
    bool checkObjectInsertShape(const CollisionObject* object) const;
    bool checkObjectRemoveShape(const CollisionObject* object) const;
};

typedef std::shared_ptr<WorldCollisionModel> WorldCollisionModelPtr;
typedef std::shared_ptr<const WorldCollisionModel> WorldCollisionModelConstPtr;

} // namespace collision
} // namespace smpl

#endif

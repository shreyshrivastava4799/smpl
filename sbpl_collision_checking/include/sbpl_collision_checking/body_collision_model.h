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

namespace smpl {
namespace collision {

class BodyCollisionModel
{
public:

    BodyCollisionModel();

    /// \brief Insert a set of collision spheres models
    ///
    /// Objects of type InputIt must be able to be dereferenced and then
    /// implicitly convertible to type const CollisionSpheresModelConfig&.
    /// Objects of type Indexer must meet the requirements of a Callable type
    /// with ArgumentType const std::string& and ReturnType int
    template <typename InputIt, typename Indexer>
    void insertCollisionSpheresModels(
        InputIt first, InputIt last,
        const Indexer& indexer);

    /// \brief Remove a set of collision spheres models
    ///
    /// Objects of type InputIt must be able to be dereferenced and then
    /// implicitly convertible to type const std::string&. Objects of type
    /// Indexer must meet the requirements of a Callable type with ArgumentType
    /// const std::string& and ReturnType int
    template <typename InputIt, typename Indexer>
    void removeCollisionSpheresModel(
        InputIt first, InputIt last,
        const Indexer& indexer)
    { }

    /// \brief Insert a set of collision voxels models
    ///
    /// Objects of type InputIt must be able to be dereferenced and then
    /// implicitly convertible to type const CollisionVoxelsModelConfig&.
    /// Objects of type Indexer must meet the requirements of a Callable type
    /// with ArgumentType const std::string& and ReturnType int
    template <typename InputIt, typename Indexer>
    void insertCollisionVoxelsModels(
        InputIt first, InputIt last,
        const Indexer& indexer);

    /// \brief Remove a set of collision voxels models
    ///
    /// Objects of type InputIt must be able to be dereferenced and then
    /// implicitly convertible to type const std::string&. Objects of type
    /// Indexer must meet the requirements of a Callable type with ArgumentType
    /// const std::string& and ReturnType int
    template <typename InputIt, typename Indexer>
    void removeCollisionVoxelsModel(
        InputIt first, InputIt last,
        const Indexer& indexer)
    { }

    /// \brief Insert a set of group models
    ///
    /// Objects of type InputIt must be able to be dereferenced and then
    /// implicitly convertible to type const CollisionGroupConfig&. Objects
    /// of type Indexer must meet the requirements of a Callable type with
    /// ArgumentType const std::string& and ReturnType int
    template <typename InputIt, typename Indexer>
    void insertGroupModels(
        InputIt first, InputIt last,
        const Indexer& indexer);

    /// \brief Remove a set of group models
    ///
    /// Objects of type InputIt must be able to be dereferenced and then
    /// implicitly convertible to type const std::string&. Objects of type
    /// Indexer must meet the requirements of a Callable type with ArgumentType
    /// const std::string& and ReturnType int
    template <typename InputIt, typename Indexer>
    void removeGroupModels(
        InputIt first, InputIt last,
        const Indexer& indexer)
    { }

private:

    std::vector<CollisionSpheresModel>  m_spheres_models;
    std::vector<CollisionVoxelsModel>   m_voxels_models;
    std::vector<CollisionGroupModel>    m_group_models;
    hash_map<std::string, int>          m_group_name_to_index;
};

template <typename InputIt, typename Indexer>
void BodyCollisionModel::insertCollisionSpheresModels(
    InputIt first, Inputit last,
    const Indexer& indexer)
{
    auto prev_size = m_spheres_models.size();
    auto new_size = prev_size + std::distance(first, last);
    auto prev_cap = m_spheres_models.capacity();
    m_spheres_models.resize(new_size);
    auto i = prev_size;
    for (auto it = first; it != last; ++it) {
        const CollisionSpheresModelConfig& spheres_config = *it;
        CollisionSpheresModel& spheres_model = m_spheres_models[i++];

        // attach to the body
        spheres_model.link_index = indexer(spheres_config.link_name);

        // initialize sphere models
        spheres_model.spheres.resize(spheres_config.spheres.size());
        for (size_t j = 0; j < spheres_model.spheres.size(); ++j) {
            const CollisionSphereConfig& sphere_config = spheres_config.spheres[j];
            CollisionSphereModel& sphere_model = spheres_model.spheres[j];
            sphere_model.name = sphere_config.name;
            sphere_model.center = Eigen::Vector3d(sphere_config.x, sphere_config.y, sphere_config.z);
            sphere_model.radius = sphere_config.radius;
            sphere_model.priority = sphere_config.priority;
        }
    }
}

template <typename InputIt, typename Indexer>
void BodyCollisionModel::insertCollisionVoxelsModels(
    InputIt first, InputIt last,
    const Indexer& indexer)
{
    auto prev_size = m_voxels_models.size();
    auto new_size = prev_size + std::distance(first, last);
    auto prev_cap = m_voxels_models.capacity();
    m_voxels_models.resize(new_size);
    auto i = prev_size;
    for (auto it = first; it != last; ++it) {
        const CollisionVoxelsModelConfig& voxels_config = *it;
        CollisionVoxelsModel& voxels_model = m_voxels_models[i++];

        // attach to the body
        voxels_model.link_index = indexer(voxels_config.link_name);

        // initialize sphere models
        voxels_model.spheres.resize(voxels_config.spheres.size());
        for (size_t j = 0; j < voxels_model.spheres.size(); ++j) {
            const CollisionSphereConfig& sphere_config = voxels_config.spheres[j];
            CollisionSphereModel& sphere_model = voxels_model.spheres[j];
            sphere_model.name = sphere_config.name;
            sphere_model.center = Eigen::Vector3d(sphere_config.x, sphere_config.y, sphere_config.z);
            sphere_model.radius = sphere_config.radius;
            sphere_model.priority = sphere_config.priority;
        }
    }
}

template <typename InputIt, typename Indexer>
void BodyCollisionModel::insertGroupModels(
    InputIt first, InputIt last,
    const Indexer& index)
{

}

} // namespace collision
} // namespace smpl

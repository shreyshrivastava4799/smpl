#ifndef SMPL_MEMORY_H
#define SMPL_MEMORY_H

#include <memory>
#include <utility>

namespace smpl {

template <class T, class... Args>
auto make_unique(Args&&... args) -> std::unique_ptr<T>
{
    return std::unique_ptr<T>(new T(args...));
}

} // namespace smpl

#endif


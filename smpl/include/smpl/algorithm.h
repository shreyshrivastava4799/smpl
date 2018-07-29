#ifndef SMPL_ALGORITHM_H
#define SMPL_ALGORITHM_H

#include <iterator>
#include <utility>

namespace smpl {

template <class BidirIt, class UnaryPredicate>
auto unordered_remove_if(
    BidirIt first,
    BidirIt last,
    UnaryPredicate p)
    -> BidirIt
{
    while (first != last) {
        if (p(*first)) {
            *first = std::move(*--last);
        } else {
            ++first;
        }
    }
    return last;
}

template <class BidirIt, class T>
auto unordered_remove(
    BidirIt first,
    BidirIt last,
    const T& value)
    -> BidirIt
{
    return unordered_remove_if(first, last,
            [&value](const typename std::iterator_traits<BidirIt>::value_type& v) {
                return v == value;
            });
}

} // namespace smpl

#endif

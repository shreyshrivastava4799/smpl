#ifndef SMPL_DEFER_H
#define SMPL_DEFER_H

namespace smpl {
namespace detail {

template <class Callable>
struct CallOnDestruct
{
    Callable c;
    CallOnDestruct(Callable c) : c(c) { }
    ~CallOnDestruct() { c(); }
};

template <class Callable>
CallOnDestruct<Callable> MakeCallOnDestruct(Callable c) {
    return CallOnDestruct<Callable>(c);
}

} // namespace detail
} // namespace smpl

// preprocessor magic to get a prefix to concatenate with the __LINE__ macro
#define SMPL_MAKE_LINE_IDENT_JOIN_(a, b) a##b
#define SMPL_MAKE_LINE_IDENT_JOIN(a, b) SMPL_MAKE_LINE_IDENT_JOIN_(a, b)
#define SMPL_MAKE_LINE_IDENT(prefix) SMPL_MAKE_LINE_IDENT_JOIN(prefix, __LINE__)

// create an obscurely named CallOnDestruct with an anonymous lambda that
// executes the given statement sequence
#define DEFER(fun) auto SMPL_MAKE_LINE_IDENT(tmp_call_on_destruct_) = ::smpl::detail::MakeCallOnDestruct([&](){ fun; })

#endif

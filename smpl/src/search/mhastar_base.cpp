#include <smpl/search/mhastar_base.h>

#include "mhastar_base_impl.h"

namespace smpl {

MHAStar::~MHAStar()
{
    Clear(this);
}

} // namespace smpl

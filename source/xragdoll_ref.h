#ifndef XRAGDOLL_REF_H
#define XRAGDOLL_REF_H
#pragma once

// How other resources name a Ragdoll resource (a skin's default ragdoll: a runtime binding, never a compile input)
#include "dependencies/xresource_guid/source/xresource_guid.h"

namespace xrsc
{
    inline static constexpr auto    ragdoll_type_guid_v     = xresource::type_guid(xresource::guid_generator::Instance64FromString("xRagdoll"));
    using                           ragdoll                 = xresource::def_guid<ragdoll_type_guid_v>;
}

#endif

#ifndef XRAGDOLL_XGPU_RSC_LOADER_H
#define XRAGDOLL_XGPU_RSC_LOADER_H
#pragma once

#include "dependencies/xresource_mgr/source/xresource_mgr.h"
#include "xragdoll.h"
#include "xragdoll_ref.h"

// The compiled Ragdoll at run time (Play and the editor's Drop): pure CPU data like the skeleton (no GPU buffers), one deserialize
template<>
struct xresource::loader< xrsc::ragdoll_type_guid_v >
{
    constexpr static inline auto            type_name_v         = L"Ragdoll";
    constexpr static inline auto            use_death_march_v   = false;
    using                                   data_type           = xragdoll::ragdoll;

    static data_type*                       Load        (xresource::mgr& Mgr, const full_guid& GUID);
    static void                             Destroy     (xresource::mgr& Mgr, data_type&& Data, const full_guid& GUID);
};

#endif

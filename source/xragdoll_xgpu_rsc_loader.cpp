#include "xragdoll.h"
#include "xragdoll_xgpu_rsc_loader.h"

#include "dependencies/xresource_guid/source/bridges/xresource_xproperty_bridge.h"

inline static auto s_RagdollRegistrations = xresource::common_registrations<xrsc::ragdoll_type_guid_v>{};

// A missing / not yet compiled resource is an expected case: null (every caller handles it)
xresource::loader< xrsc::ragdoll_type_guid_v >::data_type* xresource::loader< xrsc::ragdoll_type_guid_v >::Load(xresource::mgr& Mgr, const full_guid& GUID)
{
    std::wstring        Path     = Mgr.getResourcePath(GUID, type_name_v);
    xragdoll::ragdoll*  pRagdoll = nullptr;
    xserializer::stream Stream;
    if (auto Err = Stream.Load(Path, pRagdoll); Err) return nullptr;
    return pRagdoll;
}

// xserializer loads the struct and its arrays as ONE allocation (as the skeleton's loader)
void xresource::loader< xrsc::ragdoll_type_guid_v >::Destroy(xresource::mgr& Mgr, data_type&& Data, const full_guid& GUID)
{
    xserializer::default_memory_handler_v.Free(xserializer::mem_type{ .m_bUnique = true }, &Data);
}

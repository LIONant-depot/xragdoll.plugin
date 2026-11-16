#include "xragdoll_compiler.h"

#include "dependencies/xstrtool/source/xstrtool.h"

#include "../xragdoll_descriptor.h"
#include "../xragdoll_generate.h"
#include "../xragdoll.h"
#include "../xragdoll_details.h"
#include "plugins/xgeom_skin.plugin/source/xgeom_skin.h"

#include "dependencies/xproperty/source/xcore/my_properties.cpp"
#include "dependencies/xmath/source/bridge/xmath_to_xproperty.h"

#include <format>

namespace xragdoll_compiler
{
    struct implementation : xragdoll_compiler::instance
    {
        //--------------------------------------------------------------------------------------

        std::wstring CompiledPath(const wchar_t* pType, std::uint64_t Instance) const
        {
            return std::format(L"{}/WINDOWS/{}/{:02X}/{:02X}/{:X}", m_ProjectPaths.m_ResourcesPlatforms, pType, Instance & 0xff, (Instance >> 8) & 0xff, Instance);
        }

        // The compiled skin's LOD 0 of every mesh: bind-pose positions, triangles and the skeleton bone of each influence
        xerr ReadSkin(xrsc::skeleton& SkeletonRef)
        {
            xserializer::stream Stream; xgeom_skin::geom* pGeom = nullptr;
            if (auto Err = Stream.Load(CompiledPath(L"GeomSkin", m_Descriptor.m_Skin.m_Instance.m_Value), pGeom); Err)
                return xerr::create_f<state, "Could not load the compiled skin - has it been compiled yet?">(Err);
            SkeletonRef = pGeom->getSkeletonRef();

            const auto Verts = pGeom->getVertices(); const auto Clusters = pGeom->getClusters(); const auto CD = pGeom->getClusterData(); const auto Idx = pGeom->getIndices();
            m_Skin.m_Pos.resize(Verts.size()); m_Skin.m_W.resize(Verts.size());
            for (std::size_t c = 0; c < Clusters.size(); ++c)
            {
                const auto& Cl = Clusters[c]; const auto& D = CD[c]; const auto& T = D.m_PosTrasnlationAndWPADDING;
                for (std::uint32_t i = 0; i < Cl.m_nVertices; ++i)
                {
                    const auto& V = Verts[Cl.m_iVertex + i];
                    auto Dq = [](std::int16_t q) { return (q + 32768.0f) / 32767.5f - 1.0f; };
                    m_Skin.m_Pos[Cl.m_iVertex + i] = xmath::fvec3(Dq(V.m_XPos) * D.m_PosScaleAndBaseBoneIndex.m_X + T.m_X, Dq(V.m_YPos) * D.m_PosScaleAndBaseBoneIndex.m_Y + T.m_Y, Dq(V.m_ZPos) * D.m_PosScaleAndBaseBoneIndex.m_Z + T.m_Z);
                    const auto W = V.getWeights(); auto& O = m_Skin.m_W[Cl.m_iVertex + i];
                    for (int k = 0; k < 4; ++k) { O.m_Bone[k] = W[k] > 0 ? std::int32_t(pGeom->getVertexBone(c, V, k)) : -1; O.m_Weight[k] = W[k]; }
                }
            }
            for (const auto& M : pGeom->getMeshes())
            {
                if (M.m_nLODs == 0) continue;
                const auto& L = pGeom->getLODs()[M.m_iLOD];
                for (int s = L.m_iSubmesh; s < L.m_iSubmesh + L.m_nSubmesh; ++s)
                {
                    const auto& Sm = pGeom->getSubmeshes()[s];
                    for (int c = Sm.m_iCluster; c < Sm.m_iCluster + Sm.m_nCluster; ++c)
                    {
                        const auto& Cl = Clusters[c];
                        for (std::uint32_t i = 0; i < Cl.m_nIndices; ++i)
                        {
                            const std::uint32_t Local = Idx[Cl.m_iIndex + i];
                            if (Local >= Cl.m_nVertices) { xserializer::default_memory_handler_v.Free(xserializer::mem_type{ .m_bUnique = true }, pGeom); return xerr::create_f<state, "A skin index is outside its cluster">(); }
                            m_Skin.m_Idx.push_back(Cl.m_iVertex + Local);
                        }
                    }
                }
            }
            LogMessage(xresource_pipeline::msg_type::INFO, std::format("Skin LOD 0: {} vertices, {} triangles", Verts.size(), m_Skin.m_Idx.size() / 3));
            xserializer::default_memory_handler_v.Free(xserializer::mem_type{ .m_bUnique = true }, pGeom);
            return {};
        }

        // The skeleton: its bone manifest (names, rest, limits) and the compiled resource's inverse binds (what the skin is bound with)
        xerr ReadSkeleton(const xrsc::skeleton& Ref)
        {
            const auto Path = std::format(L"{}/{}", m_ProjectPaths.m_Project, xskeleton_desc::ManifestRelativePath(Ref.m_Instance.m_Value));
            xtextfile::stream File; xproperty::settings::context C;
            if (auto Err = File.Open(true, Path, xtextfile::file_type::TEXT); Err)
                return xerr::create_f<state, "Error opening the skeleton's bone manifest - has it been compiled yet?">(Err);
            if (auto Err = xproperty::sprop::serializer::Stream(File, m_Manifest, C); Err)
                return xerr::create_f<state, "Error reading the skeleton's bone manifest">(Err);

            xserializer::stream Stream; xskeleton::skeleton* pSkel = nullptr;
            if (auto Err = Stream.Load(CompiledPath(L"Skeleton", Ref.m_Instance.m_Value), pSkel); Err)
                return xerr::create_f<state, "Could not load the compiled skeleton">(Err);
            if (pSkel->m_nBones != m_Manifest.m_Bones.size())
            {
                xserializer::default_memory_handler_v.Free(xserializer::mem_type{ .m_bUnique = true }, pSkel);
                return xerr::create_f<state, "The compiled skeleton and its manifest disagree on the bone count">();
            }
            for (std::size_t i = 0; i < pSkel->m_nBones; ++i) m_InvBind.push_back(pSkel->m_pBones[i].m_InvBindPose);
            xserializer::default_memory_handler_v.Free(xserializer::mem_type{ .m_bUnique = true }, pSkel);
            return {};
        }

        //--------------------------------------------------------------------------------------

        xerr onCompile(void) noexcept override
        {
            displayProgressBar("Loading Descriptor", 0);
            {
                xproperty::settings::context    Context{};
                auto                            DescriptorFileName = std::format(L"{}/{}/Descriptor.txt", m_ProjectPaths.m_Project, m_InputSrcDescriptorPath);
                if (auto Err = m_Descriptor.Serialize(true, DescriptorFileName, Context); Err)
                    return Err;
            }
            {
                std::vector<std::string> Errors;
                m_Descriptor.Validate(Errors);
                if (not Errors.empty())
                {
                    for (auto& E : Errors) LogMessage(xresource_pipeline::msg_type::ERROR, std::move(E));
                    return xerr::create_f<state, "Validation Errors">();
                }
            }
            displayProgressBar("Loading Descriptor", 1);

            // inputs: the skin (compiled), its skeleton (manifest + compiled); the resource follows both
            xrsc::skeleton SkinSkeleton;
            if (auto Err = ReadSkin(SkinSkeleton); Err) return Err;
            const xrsc::skeleton Skeleton = m_Descriptor.m_Skeleton.empty() ? SkinSkeleton : m_Descriptor.m_Skeleton;
            if (Skeleton.m_Instance.m_Value != SkinSkeleton.m_Instance.m_Value)
            {
                LogMessage(xresource_pipeline::msg_type::ERROR, "The skeleton is not the one the skin is bound to");
                return xerr::create_f<state, "Skeleton mismatch">();
            }
            m_Dependencies.m_Resources.push_back({ m_Descriptor.m_Skin.m_Instance, xrsc::geom_skin_type_guid_v });
            m_Dependencies.m_Resources.push_back({ Skeleton.m_Instance, xrsc::skeleton_type_guid_v });
            m_Dependencies.m_Assets.push_back(xskeleton_desc::ManifestRelativePath(Skeleton.m_Instance.m_Value));
            if (auto Err = ReadSkeleton(Skeleton); Err) return Err;

            displayProgressBar("Fitting bodies", 0);
            std::vector<std::string> Warnings;
            const auto S = xragdoll_gen::FromManifest(m_Manifest, Warnings);
            // free joints are not a ragdoll (C63: a skeleton saved with its limits Off compiled one whose knees bent every way, and nothing failed)
            if (S.m_Limits.empty())
            {
                LogMessage(xresource_pipeline::msg_type::ERROR, "The skeleton's joint limits are Off: a ragdoll's joints come from them. Turn Joint Limits On in the Skeleton editor (Humanoid v2 for a person), compile the skeleton, then this ragdoll");
                return xerr::create_f<state, "The skeleton has no joint limits">();
            }
            const auto R = xragdoll_gen::Generate(S, m_Skin, m_InvBind, m_Descriptor, std::move(Warnings));
            displayProgressBar("Fitting bodies", 1);

            for (const auto& W : R.m_Details.m_Warnings) LogMessage(xresource_pipeline::msg_type::WARNING, W);
            for (const auto& L : R.m_Details.m_Report)   LogMessage(xresource_pipeline::msg_type::INFO, L);
            LogMessage(xresource_pipeline::msg_type::INFO, std::format("{} bodies, {} joints, {:.1f} kg, height {:.3f} m, {} filtered pairs", R.m_Details.m_nBodies, R.m_Details.m_Joints.size(), R.m_Details.m_TotalMass, R.m_Details.m_Height, R.m_Details.m_nFilterPairs));

            {
                xtextfile::stream File;
                if (auto Err = File.Open(false, std::format(L"{}\\Details.txt", m_ResourceLogPath), xtextfile::file_type::TEXT); Err)
                    return xerr::create_f<state, "Failed while opening the details.txt so it can't be saved">(Err);
                xproperty::settings::context C{}; auto Details = R.m_Details;
                if (auto Err = xproperty::sprop::serializer::Stream(File, Details, C); Err)
                    return xerr::create_f<state, "Failed while serializing details.txt">(Err);
            }
            if (!R.m_Details.m_bOk)
            {
                LogMessage(xresource_pipeline::msg_type::ERROR, "The body model could not be built: see the report in Details.txt");
                return xerr::create_f<state, "Generation failed">();
            }

            xragdoll_gen::ToResource(R, m_Final);
            for (auto& T : m_Target)
            {
                if (!T.m_bValid) continue;
                xserializer::stream Serializer;
                if (auto Err = Serializer.Save(T.m_DataPath, m_Final
                    , m_OptimizationType == optimization_type::O0 ? xserializer::compression_level::FAST : m_OptimizationType == optimization_type::O1 ? xserializer::compression_level::MEDIUM : xserializer::compression_level::HIGH); Err)
                    return xerr::create_f<state, "Failed to save the compiled ragdoll">(Err);
            }
            return {};
        }

        implementation() { m_Final.Initialize(); }
        ~implementation() { m_Final.Kill(); }

        xragdoll_desc::descriptor       m_Descriptor;
        xskeleton_desc::bone_manifest   m_Manifest;
        std::vector<xmath::fmat4>       m_InvBind;
        xragdoll_gen::skin_mesh         m_Skin;
        xragdoll::ragdoll               m_Final;
    };

    //------------------------------------------------------------------------------------

    std::unique_ptr<instance> instance::Create(void)
    {
        return std::make_unique<implementation>();
    }
}

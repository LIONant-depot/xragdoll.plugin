#ifndef XRAGDOLL_EDITOR_H
#define XRAGDOLL_EDITOR_H
#pragma once

// The Ragdoll resource editor (ragdoll design v2 section 10, R5 thin slice): the descriptor's properties, the skin's skeleton in the shared
// skeleton viewport with every body's collision shape drawn over its bones (capsule / sphere / box wireframes), the capability report the
// compiler wrote (Details.txt: the bodies, the joints with how well the solver holds the skeleton's limits, every approximation) and Drop.
// Drop runs the SAME runtime as Play (xlioncore_ragdoll_api.h: the activation, sub-steps and capture of xanimation_ragdoll_runtime.h, in a
// world of its own with a floor): the compiled ragdoll at its rest pose, lifted to a height, falls for a few seconds; the poses are recorded
// and played back (a recording: scrubbing does not simulate). The editor shows what is compiled: an edit shows after Compile.
#include "source/Tools/Editor/xeditor_descriptor_editor.h"
#include "source/Tools/Editor/xeditor_camera_toolbar.h"
#include "dependencies/xeditor/include/xeditor/commands.h"
#include "dependencies/xresource_pipeline_v2/source/editor/xresource_editor_inspector_pickers.h"
#include "dependencies/xresource_pipeline_v2/source/editor/xresource_editor_resources.h"
#include "plugins/xskeleton.plugin/source/Editor/xskeleton_viewport.h"
#include "plugins/xgeom_skin.plugin/source/xgeom_skin_xgpu_rsc_loader.h"
#include "plugins/xgeom_skin.plugin/source/xgeom_skin_xgpu_runtime.h"        // (the loaded skin: its skeleton reference)
#include "plugins/xragdoll.plugin/source/xragdoll_descriptor.h"
#include "plugins/xragdoll.plugin/source/xragdoll_details.h"
#include "plugins/xragdoll.plugin/source/xragdoll_body_graph.h"
#include "plugins/xragdoll.plugin/source/xragdoll_xgpu_rsc_loader.h"
#include "plugins/xragdoll.plugin/source/xragdoll_xgpu_rsc_loader.cpp"        // the resource loader: compiled once, in the host's translation unit
#include "dependencies/xAnimation/source/adapters/xanimation_resources.h"
#include "dependencies/xLIONCore/src/physics/xlioncore_ragdoll_api.h"
#include <algorithm>
#include <cmath>
#include <format>

namespace xragdoll_editor
{
    struct session : xeditor::descriptor_editor
    {
        using desc = xragdoll_desc::descriptor;

        // A recorded Drop: every bone's world frame per fixed step, and what the gate reads
        struct drop
        {
            float                                       m_Height = 0.5f, m_Dt = 1.0f / 60.0f;
            std::vector<std::vector<xanimation::rigid>> m_Frames;               // [frame][bone]
            std::vector<float>                          m_MaxAnchor;            // m, per frame
            float                                       m_AnchorP95 = 0, m_AnchorMax = 0, m_EnergyRatio = 0, m_RestAt = -1, m_Mass = 0;
            std::string                                 m_Note;
            bool                                        m_bOk = false;
        };

        // Drop [-Height m] [-Show s]: drops the compiled ragdoll (the runtime of Play), returns the numbers; Show: the recording's time shown
        struct drop_cmd : xundo::query_command_base
        {
            session& m_S;
            drop_cmd(xundo::system& System, session& S) noexcept : query_command_base(System, "Drop", nullptr), m_S(S) { RegisterArguments(); }
            const char* getCommandHelp() const noexcept override { return "Drops the compiled ragdoll from its rest pose (the same runtime as Play) and records it. Usage: Drop [-Height metres above the floor] [-Show seconds into the recording]"; }
            void RegisterArguments() noexcept override { m_hHeight = m_Parser.addOption("Height", "metres", false, 1); m_hShow = m_Parser.addOption("Show", "seconds", false, 1); }
            std::string Query() noexcept override
            {
                std::string H, Sh;
                if (xeditor::cmd_util::GetArg(m_Parser, m_hHeight, H) && !H.empty()) m_S.m_DropHeight = std::stof(H);
                m_S.Drop();
                if (xeditor::cmd_util::GetArg(m_Parser, m_hShow, Sh) && !Sh.empty()) { m_S.m_PlayT = std::stof(Sh); m_S.m_bPlaying = false; }
                return m_S.Summary();
            }
            xcmdline::parser::handle m_hHeight, m_hShow;
        };

        // RagdollInfo: what is loaded and shown (for the tools)
        struct info_cmd : xundo::query_command_base
        {
            session& m_S;
            info_cmd(xundo::system& System, session& S) noexcept : query_command_base(System, "RagdollInfo", nullptr), m_S(S) {}
            const char* getCommandHelp() const noexcept override { return "What the Ragdoll editor shows: the compiled ragdoll, its skeleton, the report's totals and the last Drop. Usage: RagdollInfo"; }
            void RegisterArguments() noexcept override {}
            std::string Query() noexcept override
            {
                m_S.Resolve();
                if (!m_S.m_pRagdoll) return "not loaded: " + m_S.m_Error;
                const auto& R = *m_S.m_pRagdoll; const auto& D = m_S.m_Details;
                float Shapes = 0; for (std::uint32_t i = 0; i < R.m_nShapes; ++i) Shapes += R.m_pShapes[i].m_Radius + R.m_pShapes[i].m_HalfSegment;
                return std::format("bodies {} joints {} mass {:.2f} kg bones {} shapes {:.5f} m anchor gap at rest {:.3f} mm; report ok {} height {:.3f} m warnings {}\n{}"
                    , R.m_nBodies, R.m_nJoints, R.m_TotalMass, R.m_nBones, Shapes, 1000 * m_S.m_Graph.m_RestAnchorGap, D.m_bOk ? 1 : 0, D.m_Height, D.m_Warnings.size(), m_S.Summary());
            }
        };

        xskeleton_editor::skeleton_viewport             m_Viewport;
        xeditor::camera_toolbar                         m_CameraToolbar;
        xeditor::camera_cmds                            m_CameraCmds;
        drop_cmd                                        m_DropCmd;
        info_cmd                                        m_InfoCmd;
        bool                                            m_bReady = false;

        // what is shown: the compiled ragdoll, the skin's skeleton, the body graph the runtime simulates (ToBodyGraph, as Play)
        xrsc::ragdoll                                   m_RagdollRef;
        bool                                            m_bRagdollRef = false;   // m_RagdollRef was asked for (released in LetGo)
        xrsc::geom_skin                                 m_SkinRef;
        xrsc::skeleton                                  m_SkeletonRef;
        const xragdoll::ragdoll*                        m_pRagdoll = nullptr;
        const xskeleton::skeleton*                      m_pSkeleton = nullptr;
        xanimation::skeleton_data                       m_Skel;
        std::vector<xanimation::rigid>                  m_Rest;             // rigid model frames at rest (metres)
        xragdoll::runtime_graph                         m_Graph;
        std::string                                     m_Error;
        bool                                            m_bResolved = false;
        bool                                            m_bCompiling = false;   // between a compile's start and its end nothing is loaded (the files are being replaced)
        float                                           m_Lowest = 0, m_Height = 1.7f;     // the rest pose's lowest shape point and size
        xragdoll_desc::details                          m_Details;

        // Drop and its playback
        drop                                            m_Drop;
        float                                           m_DropHeight = 0.5f, m_PlayT = 0;
        bool                                            m_bPlaying = false, m_bShowShapes = true;
        double                                          m_LastTime = 0;
        int                                             m_HoverBody = -1, m_SelBody = -1;
        std::vector<xskeleton_editor::bone_world>       m_BoneWorlds;
        std::set<int>                                   m_SelBones;

        session(xresource::full_guid Guid, xresource_editor::library::guid LibraryGuid, xgpu::device* pDevice) noexcept
            : descriptor_editor("Ragdoll", Guid, LibraryGuid, pDevice)
            , m_CameraCmds(m_Undo, m_Viewport.m_Scene.Camera())
            , m_DropCmd(m_Undo, *this), m_InfoCmd(m_Undo, *this)
        {
            m_RagdollRef.m_Instance = Guid.m_Instance;
            xresource_editor::WireResourcePickerCallbacks(m_DescriptorInspector.m_Inspector);
            LoadDetails();
            AddPanel("Report",           dock::right,  [this] { RenderReport(); });
            if (pDevice) m_CameraToolbar.Init(m_Document.m_Guid.m_Instance.m_Value);
            AddPanel("Ragdoll Viewport", dock::center, [this] { RenderViewport(); }, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
            AddPanel("Description",      dock::left,   [this] { m_DescriptorInspector.Show(); });
            m_Viewport.m_pOpen = &m_bOpen;
            m_bReady = pDevice && m_Viewport.Init(*pDevice);
        }

        ~session() noexcept override { LetGo(); m_Viewport.Release(); }

        desc* Desc() noexcept { return m_Document.isLoaded() ? static_cast<desc*>(m_Document.m_pDescriptor.get()) : nullptr; }

        void LoadDetails() noexcept
        {
            m_Details = {};
            xtextfile::stream File;
            if (auto Err = File.Open(true, m_Document.m_LogPath + L"\\Details.txt", {}); Err) return;
            xproperty::settings::context Context;
            if (auto Err = xproperty::sprop::serializer::Stream(File, m_Details, Context); Err) m_Details = {};
        }

        void LetGo() noexcept
        {
            if (m_bRagdollRef) xresource::g_Mgr.ReleaseRef(m_RagdollRef);
            if (!m_SkeletonRef.empty()) xresource::g_Mgr.ReleaseRef(m_SkeletonRef);
            m_bRagdollRef = false;
            if (!m_SkinRef.empty()) xresource::g_Mgr.ReleaseRef(m_SkinRef);
            m_RagdollRef.m_Instance = m_Document.m_Guid.m_Instance;
            m_SkinRef.clear(); m_SkeletonRef.clear();
            m_pRagdoll = nullptr; m_pSkeleton = nullptr; m_bResolved = false;
        }

        void OnDescriptorReplaced() noexcept override { }
        void OnCompileStarted()     noexcept override { LetGo(); m_Drop = {}; m_bCompiling = true; }
        void OnCompiled()           noexcept override { LoadDetails(); LetGo(); m_Drop = {}; m_bCompiling = false; }
        void OnCompileFailed()      noexcept override { LoadDetails(); LetGo(); m_Drop = {}; m_bCompiling = false; }

        // The compiled ragdoll, the skeleton it was made for (the descriptor's, or the skin's own) and the body graph the runtime simulates
        void Resolve() noexcept
        {
            if (m_bCompiling) { m_Error = "Compiling..."; return; }
            if (m_bResolved) return;
            LetGo();
            m_bResolved = true; m_Error.clear();
            auto* pDesc = Desc();
            if (!pDesc) { m_Error = "The descriptor could not be read."; return; }
            if (pDesc->m_Skin.empty()) { m_Error = "Set the Skin in the Description panel, then Compile."; return; }
            m_pRagdoll = xresource::g_Mgr.getResource(m_RagdollRef); m_bRagdollRef = true;
            if (!m_pRagdoll) { m_Error = "Not compiled yet: Compile (the bodies are generated from the skin)."; return; }
            m_SkeletonRef = pDesc->m_Skeleton;
            if (m_SkeletonRef.empty())
            {
                m_SkinRef.m_Instance = pDesc->m_Skin.m_Instance;
                auto* pSkin = xresource::g_Mgr.getResource(m_SkinRef);
                if (!pSkin) { m_SkinRef.clear(); m_Error = "The skin could not be loaded (compile it first)."; return; }
                m_SkeletonRef = pSkin->getSkeletonRef();
            }
            m_pSkeleton = m_SkeletonRef.empty() ? nullptr : xresource::g_Mgr.getResource(m_SkeletonRef);
            if (!m_pSkeleton) { m_Error = "The skeleton could not be loaded (compile it first)."; return; }
            if (m_pSkeleton->getBones().size() != m_pRagdoll->m_nBones) { m_Error = "The ragdoll was compiled for another skeleton (bone count): Compile again."; return; }

            m_Skel = xanimation::MakeSkeleton(*m_pSkeleton);
            const auto n = std::size_t(m_pRagdoll->m_nBones);
            std::vector<float> Sc(n); m_Rest.resize(n);
            xanimation::RigidBoneWorlds(m_Skel.view(), m_Skel.m_Rest, {}, 1.0f, m_Rest, Sc);
            m_Graph = xragdoll::ToBodyGraph(*m_pRagdoll, m_Rest);
            if (!m_Graph.m_Graph.m_bOk) { m_Error = "Its joints name bodies it does not have: Compile again."; return; }
            // how low the shapes reach at rest (the floor is put under them) and how big the character is (the camera)
            float Lo = 1e30f, Hi = -1e30f;
            for (const auto& B : m_Graph.m_Graph.m_Bodies)
                for (const auto& S : B.m_Shapes)
                {
                    const auto W = m_Rest[std::size_t(B.m_Bone)] * B.m_BoneToBody * S.m_Local;
                    const float r = S.m_Kind == xanimation::shape_kind::BOX ? S.m_HalfExtents.Length() : S.m_Radius + S.m_HalfSegment;
                    Lo = std::min(Lo, W.m_P.m_Y - r); Hi = std::max(Hi, W.m_P.m_Y + r);
                }
            m_Lowest = Lo < Hi ? Lo : 0.0f; m_Height = Lo < Hi ? Hi - Lo : 1.7f;
            m_Viewport.m_Scene.m_Center   = xmath::fvec3(0, 0.5f * m_Height, 0);
            m_Viewport.m_Scene.m_Radius   = 0.75f * m_Height;
            m_Viewport.m_Scene.m_bReframe = true;
        }

        //----------------------------------------------------------------------------------------
        // Drop: the runtime of Play, in a world of its own
        //----------------------------------------------------------------------------------------

        void Drop() noexcept
        {
            Resolve();
            m_Drop = {}; m_PlayT = 0; m_bPlaying = true;
            if (!m_pRagdoll || !m_Graph.m_Graph.m_bOk) { m_Drop.m_Note = m_Error; return; }
            const float Dt = m_Drop.m_Dt; constexpr float Seconds = 5.0f, g = 9.81f;
            m_Drop.m_Height = m_DropHeight;
            const auto n = std::size_t(m_pRagdoll->m_nBones);
            // the rest pose standing on the floor (its lowest shape point on y = 0), lifted by the drop height; no motion (a still start)
            const xanimation::rigid E{ xmath::fvec3(0, m_DropHeight - m_Lowest, 0), xmath::fquat::fromIdentity() };
            std::vector<xanimation::rigid> W(n); std::vector<float> Sc(n);
            xanimation::RigidBoneWorlds(m_Skel.view(), m_Skel.m_Rest, E, 1.0f, W, Sc);

            auto* pWorld = xlioncore::physics::CreateRagdollWorld(0.0f, g);
            xlioncore::physics::ragdoll_params Params;
            Params.m_Size = 2 * (m_pRagdoll->m_Capsule.m_HalfSegment + m_pRagdoll->m_Capsule.m_Radius);
            if (Params.m_Size <= 0.01f) Params.m_Size = m_Height;
            auto* pR = xlioncore::physics::ActivateRagdoll(*pWorld, m_Graph.m_Graph, m_Rest, W, W, Dt, m_Graph.m_Filter, Params, &m_Drop.m_Note);
            if (!pR) { xlioncore::physics::DestroyRagdollWorld(pWorld); m_Drop.m_Note = "the ragdoll could not be activated"; return; }

            xlioncore::physics::ragdoll_stats S0; xlioncore::physics::RagdollStats(*pR, S0);
            m_Drop.m_Mass = S0.m_Mass;
            const float Sleep = 0.05f * std::sqrt(Params.m_Size / 1.7f);
            float Ratio = 0;
            const int nSteps = int(Seconds / Dt);
            m_Drop.m_Frames.reserve(std::size_t(nSteps) + 1);
            m_Drop.m_Frames.push_back(W); m_Drop.m_MaxAnchor.push_back(0);
            for (int k = 1; k <= nSteps; ++k)
            {
                xlioncore::physics::StepRagdollWorld(*pWorld, Dt);
                std::vector<xanimation::rigid> Out(n);
                xlioncore::physics::PoseRagdoll(*pR, m_Skel.view(), 1.0f, W, Out);
                m_Drop.m_Frames.push_back(std::move(Out));
                xlioncore::physics::ragdoll_stats St; xlioncore::physics::RagdollStats(*pR, St);
                m_Drop.m_MaxAnchor.push_back(St.m_MaxAnchor);
                // no energy made: KE <= at the switch + the drop's m g h, + 10 % + 1 J (the headless test's and Play's bound)
                const float Bound = 1.1f * (S0.m_KE + St.m_Mass * g * std::max(0.0f, S0.m_ComY - St.m_ComY)) + 1.0f;
                Ratio = std::max(Ratio, St.m_KE / Bound);
                if (m_Drop.m_RestAt < 0 && k * Dt > 0.25f && (St.m_nAwake == 0 || St.m_MaxSpeed < Sleep)) m_Drop.m_RestAt = k * Dt;
            }
            xlioncore::physics::DestroyRagdollWorld(pWorld);
            auto A = m_Drop.m_MaxAnchor; std::sort(A.begin(), A.end());
            m_Drop.m_AnchorP95 = A[std::min(A.size() - 1, std::size_t(0.95 * double(A.size())))];
            m_Drop.m_AnchorMax = A.back();
            m_Drop.m_EnergyRatio = Ratio;
            m_Drop.m_bOk = true;
        }

        std::string Summary() const noexcept
        {
            if (!m_Drop.m_bOk) return m_Drop.m_Note.empty() ? "no drop yet" : "drop failed: " + m_Drop.m_Note;
            return std::format("drop {:.2f} m: {} frames, mass {:.2f} kg, anchors p95 {:.2f} mm max {:.2f} mm, energy <= {:.2f} of the bound, at rest {}{}"
                , m_Drop.m_Height, m_Drop.m_Frames.size(), m_Drop.m_Mass, 1000 * m_Drop.m_AnchorP95, 1000 * m_Drop.m_AnchorMax, m_Drop.m_EnergyRatio
                , m_Drop.m_RestAt >= 0 ? std::format("{:.2f} s", m_Drop.m_RestAt) : std::string("never (5 s)"), m_Drop.m_Note.empty() ? "" : "; " + m_Drop.m_Note);
        }

        //----------------------------------------------------------------------------------------
        // Drawing
        //----------------------------------------------------------------------------------------

        // One shape's wireframe at its world frame W, into the viewport's lines
        void EmitShape(const xanimation::body_shape& S, const xanimation::rigid& W, std::uint32_t C) noexcept
        {
            auto& V = m_Viewport.m_ExtraLines;
            auto Seg = [&](const xmath::fvec3& a, const xmath::fvec3& b) { xskeleton_editor::EmitSegment(V, W.Apply(a), W.Apply(b), C, C); };
            constexpr int N = 20; constexpr float Pi = 3.14159265f;
            // an arc around Center in the plane of U, V, from angle A0 to A1
            auto Arc = [&](const xmath::fvec3& Center, const xmath::fvec3& U, const xmath::fvec3& Vv, float R, float A0, float A1)
            {
                for (int i = 0; i < N; ++i)
                {
                    const float a = A0 + (A1 - A0) * float(i) / N, b = A0 + (A1 - A0) * float(i + 1) / N;
                    Seg(Center + U * (R * std::cos(a)) + Vv * (R * std::sin(a)), Center + U * (R * std::cos(b)) + Vv * (R * std::sin(b)));
                }
            };
            const xmath::fvec3 X(1, 0, 0), Y(0, 1, 0), Z(0, 0, 1);
            if (S.m_Kind == xanimation::shape_kind::CAPSULE)
            {
                const float r = S.m_Radius, h = S.m_HalfSegment;
                const xmath::fvec3 Top = Y * h, Bot = Y * -h;
                Arc(Top, X, Z, r, 0, 2 * Pi); Arc(Bot, X, Z, r, 0, 2 * Pi);
                Seg(Top + X * r, Bot + X * r); Seg(Top - X * r, Bot - X * r); Seg(Top + Z * r, Bot + Z * r); Seg(Top - Z * r, Bot - Z * r);
                Arc(Top, X, Y, r, 0, Pi); Arc(Top, Z, Y, r, 0, Pi); Arc(Bot, X, Y, r, Pi, 2 * Pi); Arc(Bot, Z, Y, r, Pi, 2 * Pi);
            }
            else if (S.m_Kind == xanimation::shape_kind::SPHERE)
            {
                const xmath::fvec3 O(0, 0, 0);
                Arc(O, X, Y, S.m_Radius, 0, 2 * Pi); Arc(O, Y, Z, S.m_Radius, 0, 2 * Pi); Arc(O, Z, X, S.m_Radius, 0, 2 * Pi);
            }
            else
            {
                const auto& H = S.m_HalfExtents;
                auto Pt = [&](int i) { return xmath::fvec3((i & 1) ? H.m_X : -H.m_X, (i & 2) ? H.m_Y : -H.m_Y, (i & 4) ? H.m_Z : -H.m_Z); };
                for (int i = 0; i < 8; ++i) for (int b = 1; b < 8; b <<= 1) if (!(i & b)) Seg(Pt(i), Pt(i | b));
            }
        }

        // The bones shown now: the Drop's recording at the playback time, else the rest pose (standing on the floor)
        const std::vector<xanimation::rigid>* ShownBones(std::vector<xanimation::rigid>& Tmp) noexcept
        {
            if (m_Drop.m_bOk && !m_Drop.m_Frames.empty())
            {
                const int k = std::clamp(int(m_PlayT / m_Drop.m_Dt + 0.5f), 0, int(m_Drop.m_Frames.size()) - 1);
                return &m_Drop.m_Frames[std::size_t(k)];
            }
            Tmp.resize(m_Rest.size());
            for (std::size_t i = 0; i < m_Rest.size(); ++i) Tmp[i] = xanimation::rigid{ xmath::fvec3(0, -m_Lowest, 0), xmath::fquat::fromIdentity() } * m_Rest[i];
            return &Tmp;
        }

        void RenderViewport() noexcept
        {
            Resolve();
            // the Drop row: height, Drop, the playback of the recording
            ImGui::SetNextItemWidth(90); ImGui::DragFloat("height (m)", &m_DropHeight, 0.01f, 0.0f, 50.0f, "%.2f");
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("How high above the floor the rest pose's lowest point starts");
            ImGui::SameLine();
            if (ImGui::Button("Drop")) Drop();
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Drops the compiled ragdoll from its rest pose with the runtime Play uses, records 5 s and plays it back");
            ImGui::SameLine(); ImGui::Checkbox("shapes", &m_bShowShapes);
            if (m_Drop.m_bOk)
            {
                const float Len = float(m_Drop.m_Frames.size() - 1) * m_Drop.m_Dt;
                const double Now = ImGui::GetTime();
                if (m_bPlaying) { m_PlayT += float(Now - m_LastTime); if (m_PlayT >= Len) { m_PlayT = Len; m_bPlaying = false; } }
                m_LastTime = Now;
                ImGui::SameLine(); if (ImGui::Button(m_bPlaying ? "Pause" : "Play")) { if (!m_bPlaying && m_PlayT >= Len) m_PlayT = 0; m_bPlaying = !m_bPlaying; }
                ImGui::SameLine(); if (ImGui::Button("Rest")) { m_Drop = {}; m_bPlaying = false; }
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("Back to the rest pose (the recording is dropped)");
                ImGui::SameLine(); ImGui::SetNextItemWidth(std::max(120.0f, ImGui::GetContentRegionAvail().x));
                if (ImGui::SliderFloat("##t", &m_PlayT, 0, Len, "RECORDED DROP: %.2f s")) m_bPlaying = false;
                ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.4f, 1.0f), "PLAYBACK of a recording (scrubbing does not simulate)");
                ImGui::SameLine(); ImGui::TextDisabled("%s", Summary().c_str());
            }
            else if (!m_Drop.m_Note.empty()) { ImGui::SameLine(); ImGui::TextColored(ImVec4(1, 0.5f, 0.4f, 1), "%s", m_Drop.m_Note.c_str()); }
            m_CameraToolbar.Render(ImGui::GetContentRegionAvail(), m_Viewport.m_Scene.m_State, [this] { RenderViewportBody(); });
        }

        void RenderViewportBody() noexcept
        {
            auto* pHost   = xeditor::host::current();
            auto* pWindow = pHost ? pHost->find<xgpu::window>() : nullptr;
            if (!m_bReady || !pWindow) { ImGui::TextDisabled("The 3D view needs a GPU device (open from the editor)."); return; }
            if (!m_pRagdoll || !m_pSkeleton || !m_Graph.m_Graph.m_bOk) { ImGui::TextWrapped("%s", m_Error.empty() ? "Nothing to show." : m_Error.c_str()); return; }

            std::vector<xanimation::rigid> Tmp;
            const auto& Bones = *ShownBones(Tmp);
            m_BoneWorlds.resize(Bones.size());
            for (std::size_t i = 0; i < Bones.size(); ++i) m_BoneWorlds[i] = { Bones[i].m_P, Bones[i].m_Q * xmath::fvec3(1, 0, 0), Bones[i].m_Q * xmath::fvec3(0, 1, 0) };

            const int iLit = m_HoverBody >= 0 ? m_HoverBody : m_SelBody;
            m_SelBones.clear();
            if (iLit >= 0 && iLit < int(m_Graph.m_Graph.m_Bodies.size())) m_SelBones.insert(m_Graph.m_Graph.m_Bodies[std::size_t(iLit)].m_Bone);
            if (m_bShowShapes)
                for (int b = 0; b < int(m_Graph.m_Graph.m_Bodies.size()); ++b)
                {
                    const auto& B = m_Graph.m_Graph.m_Bodies[std::size_t(b)];
                    const auto BW = Bones[std::size_t(B.m_Bone)] * B.m_BoneToBody;
                    const std::uint32_t C = b == iLit ? IM_COL32(255, 230, 60, 255) : IM_COL32(90, 230, 255, 255);
                    for (const auto& S : B.m_Shapes) EmitShape(S, BW * S.m_Local, C);
                }
            xskeleton_editor::viewport_item Items[1] = { { .m_pSkeleton = m_pSkeleton, .m_Bones = m_BoneWorlds, .m_NormalColor = IM_COL32(230, 230, 230, 255), .m_pSelected = &m_SelBones, .m_bLeadsToSelected = false, .m_bOutlineOnly = m_bShowShapes } };
            m_Viewport.Render(Items);

            // the body under the mouse: the bone the pick finds, posed from its body
            m_HoverBody = -1;
            if (m_Viewport.Over())
            {
                int iBone = -1; float T = 0;
                if (m_Viewport.PickBone(Items[0], ImGui::GetMousePos(), iBone, T) && iBone >= 0 && iBone < int(m_Graph.m_Graph.m_BoneBody.size()))
                {
                    m_HoverBody = m_Graph.m_Graph.m_BoneBody[std::size_t(iBone)];
                    if (m_HoverBody >= 0)
                    {
                        const auto* pRow = m_HoverBody < int(m_Details.m_Bodies.size()) ? &m_Details.m_Bodies[std::size_t(m_HoverBody)] : nullptr;
                        if (pRow) ImGui::SetTooltip("%s (%s)\n%s\n%.2f kg", pRow->m_Bone.c_str(), pRow->m_Slot.c_str(), pRow->m_Shape.c_str(), pRow->m_Mass);
                        if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) m_SelBody = m_HoverBody;
                    }
                }
            }
        }

        // The capability report: totals, warnings, the bodies, the joints (how well the solver holds the skeleton's limits), the approximations
        void RenderReport() noexcept
        {
            Resolve();
            const auto& D = m_Details;
            if (D.m_Bodies.empty()) { ImGui::TextWrapped("No report yet: Compile (the compiler writes it)."); return; }
            ImGui::Text("%s: %d bodies, %.1f kg, %.3f m tall", D.m_bOk ? "OK" : "NOT OK", D.m_nBodies, D.m_TotalMass, D.m_Height);
            ImGui::TextDisabled("follow %d, animated %d, excluded %d, never-collide pairs %d", D.m_nFollows, D.m_nAnimated, D.m_nExcluded, D.m_nFilterPairs);
            ImGui::TextDisabled("movement capsule: %s", D.m_Capsule.c_str());
            for (const auto& W : D.m_Warnings) ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.3f, 1.0f), "! %s", W.c_str());
            int iHover = -1;
            if (ImGui::CollapsingHeader("Bodies", ImGuiTreeNodeFlags_DefaultOpen) && ImGui::BeginTable("##Bodies", 6, ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_Resizable))
            {
                for (const char* H : { "Bone", "Slot", "Shape", "kg", "Coverage", "Outside p95" }) ImGui::TableSetupColumn(H);
                ImGui::TableHeadersRow();
                for (int i = 0; i < int(D.m_Bodies.size()); ++i)
                {
                    const auto& R = D.m_Bodies[std::size_t(i)];
                    ImGui::TableNextRow();
                    if (i == m_HoverBody) ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg1, IM_COL32(90, 140, 210, 110));
                    ImGui::TableNextColumn();
                    if (ImGui::Selectable(R.m_Bone.c_str(), m_SelBody == i, ImGuiSelectableFlags_SpanAllColumns)) m_SelBody = m_SelBody == i ? -1 : i;
                    if (ImGui::IsItemHovered()) { iHover = i; ImGui::SetTooltip("origin: %s; excess p95 %.1f mm; %.1f %% of the skin", R.m_Origin.c_str(), 1000 * R.m_ExcessP95, 100 * R.m_Share); }
                    ImGui::TableNextColumn(); ImGui::TextUnformatted(R.m_Slot.c_str());
                    ImGui::TableNextColumn(); ImGui::TextUnformatted(R.m_Shape.c_str());
                    ImGui::TableNextColumn(); ImGui::Text("%.2f", R.m_Mass);
                    ImGui::TableNextColumn(); ImGui::Text("%.0f %%", 100 * R.m_Coverage);
                    ImGui::TableNextColumn(); ImGui::Text("%.1f mm", 1000 * R.m_OutsideP95);
                }
                ImGui::EndTable();
            }
            if (ImGui::CollapsingHeader("Joints", ImGuiTreeNodeFlags_DefaultOpen) && ImGui::BeginTable("##Joints", 6, ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_Resizable))
            {
                for (const char* H : { "Bone", "Solver", "Mapping", "Stiffer", "Past limit", "Max past" }) ImGui::TableSetupColumn(H);
                ImGui::TableHeadersRow();
                for (const auto& J : D.m_Joints)
                {
                    ImGui::TableNextRow();
                    ImGui::TableNextColumn(); ImGui::TextUnformatted(J.m_Bone.c_str());
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip("parent: %s\n%s\nStiffer: of the rotations the skeleton allows, the share the solver rejects.\nPast limit: of the rotations the solver allows, the share outside the skeleton's limit.", J.m_Parent.c_str(), J.m_Note.c_str());
                    ImGui::TableNextColumn(); ImGui::TextUnformatted(J.m_Solver.c_str());
                    ImGui::TableNextColumn();
                    if (J.m_Mapping == "exact") ImGui::TextColored(ImVec4(0.5f, 1.0f, 0.6f, 1.0f), "exact"); else ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.3f, 1.0f), "%s", J.m_Mapping.c_str());
                    ImGui::TableNextColumn(); ImGui::Text("%.0f %%", J.m_FalseRejectPct);
                    ImGui::TableNextColumn(); ImGui::Text("%.0f %%", J.m_FalseAcceptPct);
                    ImGui::TableNextColumn(); ImGui::Text("%.1f deg", J.m_MaxExcessDeg);
                }
                ImGui::EndTable();
            }
            if (ImGui::CollapsingHeader("Approximations and removed articulations"))
                for (const auto& L : D.m_Report) ImGui::TextWrapped("%s", L.c_str());
            m_HoverBody = iHover >= 0 ? iHover : m_HoverBody;
        }
    };

    inline const xeditor::auto_register_resource_editor g_Registration
    { xragdoll_desc::resource_type_guid_v
    , [](xresource::full_guid Guid, xresource_editor::library::guid LibraryGuid, xgpu::device* pDevice) -> std::unique_ptr<xeditor::resource_editor>
      { return std::make_unique<session>(Guid, LibraryGuid, pDevice); }
    };
}

#endif

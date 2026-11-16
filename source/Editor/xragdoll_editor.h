#ifndef XRAGDOLL_EDITOR_H
#define XRAGDOLL_EDITOR_H
#pragma once

// The Ragdoll resource editor (ragdoll design v2 section 10, R5 thin slice): the descriptor's properties, the skin's skeleton in the shared
// skeleton viewport with every body's collision shape drawn over its bones (capsule / sphere / box wireframes), the capability report the
// compiler wrote (Details.txt: the bodies, the joints with how well the solver holds the skeleton's limits, every approximation) and Drop.
// Drop runs the SAME runtime as Play (xlioncore_ragdoll_api.h: the activation, sub-steps and capture of xanimation_ragdoll_runtime.h, in a
// world of its own with a floor): the compiled ragdoll at its rest pose, lifted to a height, falls for a few seconds; the poses are recorded
// and played back (a recording: scrubbing does not simulate). The view shows the descriptor: a shape edited with the handles shows (and drops) at once, without a compile.
#include "source/Tools/Editor/xeditor_descriptor_editor.h"
#include "source/Tools/Editor/xeditor_camera_toolbar.h"
#include "dependencies/xeditor/include/xeditor/commands.h"
#include "dependencies/xresource_pipeline_v2/source/editor/xresource_editor_inspector_pickers.h"
#include "dependencies/xresource_pipeline_v2/source/editor/xresource_editor_resources.h"
#include "plugins/xskeleton.plugin/source/Editor/xskeleton_viewport.h"
#include "plugins/xskeleton.plugin/source/Editor/xskeleton_bone_tree.h"         // the Skeleton editor's bone tree (shared)
#include "plugins/xlevel.plugin/source/Editor/xlevel_viewport_tools.h"       // the Level editor's shape handles (view, handle::Axis, Label): reused for the bodies' shapes
#include "plugins/xgeom_skin.plugin/source/xgeom_skin_xgpu_rsc_loader.h"
#include "plugins/xgeom_skin.plugin/source/xgeom_skin_xgpu_runtime.h"        // (the loaded skin: its skeleton reference)
#include "plugins/xragdoll.plugin/source/xragdoll_descriptor.h"
#include "plugins/xragdoll.plugin/source/xragdoll_details.h"
#include "plugins/xragdoll.plugin/source/xragdoll_body_graph.h"
#include "plugins/xragdoll.plugin/source/xragdoll_xgpu_rsc_loader.h"
#include "plugins/xragdoll.plugin/source/xragdoll_xgpu_rsc_loader.cpp"        // the resource loader: compiled once, in the host's translation unit
#include "dependencies/xAnimation/source/adapters/xanimation_resources.h"
#include "dependencies/xLIONCore/src/physics/xlioncore_ragdoll_api.h"
#include "dependencies/xLIONCore/src/game/xlioncore_game.h"                 // game_time::kScaleSteps: the speeds the Level editor offers
#include "dependencies/xAnimation/source/retarget/xanimation_mirror.h"       // FindMirrorBone: the L/R pair by name (the Retarget editor's smart mirror)
#include "source/Tools/Editor/xeditor_play_transport.h"                      // the Level editor's Play/Stop/Step/Pause and speed (shared)
#include "source/Tools/Editor/xeditor_scale_reference.h"                     // the Skeleton editor's human scale reference
#include "dependencies/xeditor_tools/src/xeditor_tools_picking.h"       // ray against sphere / box: the hand picks the colliders
#include "source/Tools/Editor/xeditor_transform_gizmo.h"                     // the previews' Move / Rotate gizmo, its tools and keys (Manipulate: on a shape)
#include "plugins/xgeom_skin.plugin/source/Editor/xgeom_skin_editor_under.h"    // the skin drawn under the bones (shared with the Animation Package editor)
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
                return std::format("bodies {} joints {} mass {:.2f} kg bones {} shapes {:.5f} m anchor gap at rest {:.3f} mm; report ok {} height {:.3f} m warnings {}\n{}\nview: over {} hover bone {} body {} grab body {}"
                    , R.m_nBodies, R.m_nJoints, R.m_TotalMass, R.m_nBones, Shapes, 1000 * m_S.m_Graph.m_RestAnchorGap, D.m_bOk ? 1 : 0, D.m_Height, D.m_Warnings.size(), m_S.Summary(), m_S.m_Viewport.Over() ? 1 : 0, m_S.m_HoverBone, m_S.m_HoverBody, m_S.m_GrabBody);
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
        float                                           m_DropHeight = 0.5f, m_PlayT = 0, m_Speed = 1.0f;
        bool                                            m_bPlaying = false, m_bShowShapes = true, m_bShowBones = true;
        // the live simulation (Play) and the hand that pulls it (a left drag on a bone while it runs)
        struct sim
        {
            xlioncore::physics::ragdoll_world*          m_pWorld = nullptr;
            xlioncore::physics::ragdoll*                m_pR     = nullptr;
            std::vector<xanimation::rigid>              m_Start;            // the bones at the start (lifted rest pose): the animated input
            xlioncore::physics::ragdoll_stats           m_S0;
            float                                       m_Sleep = 0, m_Accum = 0;
        };
        static constexpr float                          kGravity = 9.81f, kMaxLive = 120.0f;
        sim                                             m_Sim;
        bool                                            m_bDragged = false;
        float                                           m_PullHertz = 12.0f;         // the hand's stiffness (the Pull slider)
        std::vector<xanimation::rigid>                  m_LastBones;        // the bones shown this frame (the hand picks the colliders posed by them)
        int                                             m_GrabBody = -1;
        xmath::fvec3                                    m_GrabLocal{}, m_GrabTarget{}, m_GrabNormal{};
        bool                                            m_bMirror = true;               // a shape edit changes its left/right twin too
        xeditor::transform_gizmo::state                 m_Gizmo;                        // the shapes' Move / Rotate gizmo (the other editors' tools and keys: Q W E R)
        xeditor::transform_gizmo::actions               m_GizmoActions{ m_Gizmo };
        // the skin under the bones (the Skin button): the Skin Geom editor's own preview, with the bones' camera
        xgeom_skin_editor::skin_under                   m_SkinUnder;
        xgeom_skin_editor::preview::runtime::geom*      m_pSkinGeom = nullptr;
        std::vector<xmath::fmat4>                       m_SkinWorld;
        std::vector<xmath::fmat4>                       m_SkinRest;
        xgpu::device*                                   m_pDevice = nullptr;
        bool                                            m_bShowSkin = false;
        bool                                            m_bFramed = false;              // the camera framed the character once: a Compile never moves it again
        std::vector<xanimation::body_shape>             m_CompiledShapes;               // each body's first shape as compiled (an undone hand edit goes back to it)
        double                                          m_LastTime = 0;
        int                                             m_HoverBody = -1, m_SelBody = -1;
        int                                             m_SelBone = -1, m_HoverBone = -1;     // the bone picked in the tree or the view: its body is the selected body
        xskeleton_editor::bone_tree                     m_BoneTree;
        std::vector<xskeleton_editor::bone_tree::row>   m_BoneRows;
        bool                                            m_bHandleHot = false, m_bHandleDrag = false;     // a shape handle is hovered / dragged (the view's camera and picking stay out)
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
            if (pDevice) m_CameraToolbar.Init(m_Document.m_Guid.m_Instance.m_Value, 500.0f, 330.0f);     // with the gizmo's tools, Human and Skin
            m_pDevice = pDevice;
            AddPanel("Ragdoll Viewport", dock::center, [this] { RenderViewport(); }, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
            AddPanel("Bones",            dock::left,   [this] { RenderBones(); });
            AddPanel("Description",      dock::left,   [this] { m_DescriptorInspector.Show(); });
            m_Viewport.m_pOpen = &m_bOpen;
            m_bReady = pDevice && m_Viewport.Init(*pDevice);
        }

        ~session() noexcept override { EndSim(); LetGo(); m_Viewport.Release(); m_SkinUnder.Release(); }

        void RegisterActions(ximgui::actions::context& Ctx) noexcept override { Ctx.Scope(m_GizmoActions, "Preview"); }     // Q W E R, as every preview

        int LayoutVersion() const noexcept override { return 1; }    // 1: the Bones panel (the shared bone tree)

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
            m_pRagdoll = nullptr; m_pSkeleton = nullptr; m_pSkinGeom = nullptr; m_bResolved = false;
        }

        void OnDescriptorReplaced() noexcept override { }
        void OnCompileStarted()     noexcept override { EndSim(); LetGo(); m_Drop = {}; m_bCompiling = true; }
        void OnCompiled()           noexcept override { EndSim(); LoadDetails(); LetGo(); m_Drop = {}; m_bCompiling = false; }
        void OnCompileFailed()      noexcept override { EndSim(); LoadDetails(); LetGo(); m_Drop = {}; m_bCompiling = false; }

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
            m_SkinRef.m_Instance = pDesc->m_Skin.m_Instance;        // (also drawn: the Skin button)
            m_pSkinGeom = xresource::g_Mgr.getResource(m_SkinRef);
            m_SkeletonRef = pDesc->m_Skeleton;
            if (m_SkeletonRef.empty())
            {
                if (!m_pSkinGeom) { m_Error = "The skin could not be loaded (compile it first)."; return; }
                m_SkeletonRef = m_pSkinGeom->getSkeletonRef();
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
            if (!m_bFramed) { m_Viewport.m_Scene.m_bReframe = true; m_bFramed = true; }
            m_CompiledShapes.assign(m_Graph.m_Graph.m_Bodies.size(), {});
            for (std::size_t b = 0; b < m_CompiledShapes.size(); ++b) if (!m_Graph.m_Graph.m_Bodies[b].m_Shapes.empty()) m_CompiledShapes[b] = m_Graph.m_Graph.m_Bodies[b].m_Shapes[0];
            ShowDescShapes();
        }

        // The shapes shown follow the descriptor, not the last compile: a bone's hand-set shape (its override's ShapeSet) replaces the
        // compiled one, as the compiler will (xragdoll_generate.h); without one, the compiled shape. An edit, its undo and redo show at once.
        void ShowDescShapes() noexcept
        {
            auto* pDesc = Desc();
            if (!pDesc || m_CompiledShapes.size() != m_Graph.m_Graph.m_Bodies.size()) return;
            for (std::size_t b = 0; b < m_CompiledShapes.size() && b < m_Details.m_Bodies.size(); ++b)
            {
                auto& B = m_Graph.m_Graph.m_Bodies[b];
                if (B.m_Shapes.empty()) continue;
                auto& S = B.m_Shapes[0];
                S = m_CompiledShapes[b];
                const auto& Name = m_Details.m_Bodies[b].m_Bone;
                auto It = std::find_if(pDesc->m_Overrides.begin(), pDesc->m_Overrides.end(), [&](const auto& O) { return O.m_Bone == Name; });
                if (It == pDesc->m_Overrides.end() || !It->m_bShapeSet) continue;
                S.m_Kind = It->m_Shape == xragdoll_desc::shape_choice::CAPSULE ? xanimation::shape_kind::CAPSULE : It->m_Shape == xragdoll_desc::shape_choice::SPHERE ? xanimation::shape_kind::SPHERE : xanimation::shape_kind::BOX;
                S.m_Local = { It->m_ShapeCentre, It->m_ShapeRotation.NormalizeSafeCopy() };
                S.m_Radius = std::max(It->m_ShapeRadius, 0.005f); S.m_HalfSegment = std::max(It->m_ShapeHalfSegment, 0.0f); S.m_HalfExtents = It->m_ShapeHalfExtents;
            }
        }

        // The body of the selected body's left/right twin (by the bone names, as the Retarget editor's smart mirror); -1: none
        int MirrorBody(int iBody) const noexcept
        {
            if (iBody < 0 || iBody >= int(m_Graph.m_Graph.m_Bodies.size())) return -1;
            const auto& Names = m_Details.m_BoneNames;
            std::vector<std::string_view> V(Names.begin(), Names.end());
            const int iBone = xanimation::FindMirrorBone(V, m_Graph.m_Graph.m_Bodies[std::size_t(iBody)].m_Bone);
            const int iTwin = BodyOf(iBone);
            return iTwin >= 0 && iTwin != iBody && m_Graph.m_Graph.m_Bodies[std::size_t(iTwin)].m_Bone == iBone ? iTwin : -1;
        }

        //----------------------------------------------------------------------------------------
        // Drop: the runtime of Play, in a world of its own
        //----------------------------------------------------------------------------------------

        // The simulation: Begin makes the world and activates the ragdoll lifted to the drop height; Step runs one fixed step (the hand's
        // pull first) and records the pose; End frees the world and sums the recording up. Drop (the command, the gate) runs 5 s at once;
        // Play runs it live, a step per 1/60 s of the playback, so the hand can pull it.
        bool BeginSim() noexcept
        {
            EndSim();
            Resolve();
            m_Drop = {}; m_PlayT = 0; m_bDragged = false; m_GrabBody = -1;
            if (!m_pRagdoll || !m_Graph.m_Graph.m_bOk) { m_Drop.m_Note = m_Error; return false; }
            const float Dt = m_Drop.m_Dt;
            m_Drop.m_Height = m_DropHeight;
            const auto n = std::size_t(m_pRagdoll->m_nBones);
            // the rest pose standing on the floor (its lowest shape point on y = 0), lifted by the drop height; no motion (a still start)
            const xanimation::rigid E{ xmath::fvec3(0, m_DropHeight - m_Lowest, 0), xmath::fquat::fromIdentity() };
            m_Sim.m_Start.assign(n, {}); std::vector<float> Sc(n);
            xanimation::RigidBoneWorlds(m_Skel.view(), m_Skel.m_Rest, E, 1.0f, m_Sim.m_Start, Sc);

            m_Sim.m_pWorld = xlioncore::physics::CreateRagdollWorld(0.0f, kGravity);
            xlioncore::physics::ragdoll_params Params;
            Params.m_Size = 2 * (m_pRagdoll->m_Capsule.m_HalfSegment + m_pRagdoll->m_Capsule.m_Radius);
            if (Params.m_Size <= 0.01f) Params.m_Size = m_Height;
            m_Sim.m_pR = xlioncore::physics::ActivateRagdoll(*m_Sim.m_pWorld, m_Graph.m_Graph, m_Rest, m_Sim.m_Start, m_Sim.m_Start, Dt, m_Graph.m_Filter, Params, &m_Drop.m_Note);
            if (!m_Sim.m_pR) { EndSim(); m_Drop.m_Note = "the ragdoll could not be activated"; return false; }
            xlioncore::physics::RagdollStats(*m_Sim.m_pR, m_Sim.m_S0);
            m_Drop.m_Mass = m_Sim.m_S0.m_Mass;
            m_Sim.m_Sleep = 0.05f * std::sqrt(Params.m_Size / 1.7f);
            m_Sim.m_Accum = 0;
            m_Drop.m_Frames.push_back(m_Sim.m_Start); m_Drop.m_MaxAnchor.push_back(0);
            m_Drop.m_bOk = true;
            return true;
        }

        void StepSim() noexcept
        {
            if (!m_Sim.m_pR) return;
            const float Dt = m_Drop.m_Dt;
            if (m_GrabBody >= 0) { xlioncore::physics::PullRagdollBody(*m_Sim.m_pR, m_GrabBody, m_GrabLocal, m_GrabTarget, m_PullHertz); m_bDragged = true; }
            else xlioncore::physics::ReleaseRagdollPull(*m_Sim.m_pR);
            xlioncore::physics::StepRagdollWorld(*m_Sim.m_pWorld, Dt);
            std::vector<xanimation::rigid> Out(m_Sim.m_Start.size());
            xlioncore::physics::PoseRagdoll(*m_Sim.m_pR, m_Skel.view(), 1.0f, m_Sim.m_Start, Out);
            m_Drop.m_Frames.push_back(std::move(Out));
            xlioncore::physics::ragdoll_stats St; xlioncore::physics::RagdollStats(*m_Sim.m_pR, St);
            m_Drop.m_MaxAnchor.push_back(St.m_MaxAnchor);
            // no energy made: KE <= at the switch + the drop's m g h, + 10 % + 1 J (the headless test's and Play's bound); a pull adds energy: not counted then
            const auto& S0 = m_Sim.m_S0;
            const float Bound = 1.1f * (S0.m_KE + St.m_Mass * kGravity * std::max(0.0f, S0.m_ComY - St.m_ComY)) + 1.0f;
            if (!m_bDragged) m_Drop.m_EnergyRatio = std::max(m_Drop.m_EnergyRatio, St.m_KE / Bound);
            const float T = float(m_Drop.m_Frames.size() - 1) * Dt;
            if (m_Drop.m_RestAt < 0 && T > 0.25f && (St.m_nAwake == 0 || St.m_MaxSpeed < m_Sim.m_Sleep)) m_Drop.m_RestAt = T;
        }

        void EndSim() noexcept
        {
            if (m_Sim.m_pWorld) xlioncore::physics::DestroyRagdollWorld(m_Sim.m_pWorld);
            m_Sim.m_pWorld = nullptr; m_Sim.m_pR = nullptr; m_GrabBody = -1;
            if (m_Drop.m_MaxAnchor.empty()) return;
            auto A = m_Drop.m_MaxAnchor; std::sort(A.begin(), A.end());
            m_Drop.m_AnchorP95 = A[std::min(A.size() - 1, std::size_t(0.95 * double(A.size())))];
            m_Drop.m_AnchorMax = A.back();
        }

        // Drop (the command): 5 s at once, then played back
        void Drop() noexcept
        {
            if (!BeginSim()) return;
            for (int k = 0; k < int(5.0f / m_Drop.m_Dt); ++k) StepSim();
            EndSim();
            m_bPlaying = true;
        }

        // Play's live simulation: the steps the playback time asks for (at most 8 a frame: a slow frame slows it down, never explodes it)
        void RunLive(float Seconds) noexcept
        {
            if (!m_Sim.m_pR) return;
            m_Sim.m_Accum += Seconds;
            for (int i = 0; i < 8 && m_Sim.m_Accum >= m_Drop.m_Dt; ++i) { StepSim(); m_Sim.m_Accum -= m_Drop.m_Dt; }
            m_Sim.m_Accum = std::min(m_Sim.m_Accum, m_Drop.m_Dt);
            m_PlayT = float(m_Drop.m_Frames.size() - 1) * m_Drop.m_Dt;
            if (m_PlayT >= kMaxLive) { m_bPlaying = false; EndSim(); }     // ponytail: the recording stops at kMaxLive (memory); Stop/Reset to go again
        }

        std::string Summary() const noexcept
        {
            if (!m_Drop.m_bOk) return m_Drop.m_Note.empty() ? "no drop yet" : "drop failed: " + m_Drop.m_Note;
            if (m_Sim.m_pR) return std::format("live: {:.1f} s, mass {:.2f} kg{}; left-drag a bone to pull it", float(m_Drop.m_Frames.size() - 1) * m_Drop.m_Dt, m_Drop.m_Mass
                                              , m_bDragged ? ", pulled (the energy check is off)" : "");
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
            // The play row, in the Level editor's language (xeditor_play_transport.h): the speed, Play (drops the ragdoll with the runtime
            // Play uses and plays the recording) / Stop (back to the rest pose), Step / Pause, Resume. Left: the height and the editing switches.
            const float Len = m_Drop.m_bOk ? float(m_Drop.m_Frames.size() - 1) * m_Drop.m_Dt : 0.0f;
            const double Now = ImGui::GetTime();
            if (m_Drop.m_bOk && m_bPlaying)
            {
                const float Dt = std::min(float(Now - m_LastTime), 0.1f) * m_Speed;
                if (m_Sim.m_pR) RunLive(Dt);          // live: the simulation runs at the playback speed
                else { m_PlayT += Dt; if (m_PlayT >= Len) { m_PlayT = Len; m_bPlaying = false; } }
            }
            m_LastTime = Now;
            namespace pt = xeditor::play_transport;
            const pt::layout L;
            ImGui::BeginDisabled(m_Drop.m_bOk);
            ImGui::SetNextItemWidth(70); ImGui::DragFloat("##Height", &m_DropHeight, 0.01f, 0.0f, 50.0f, "%.2f m");
            ImGui::EndDisabled();
            if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) xeditor::hint::Draw({ "Drop height", "How high above the floor the rest pose's lowest point starts when Play drops it" });
            ImGui::SameLine(); ImGui::Checkbox("Bones", &m_bShowBones);
            if (ImGui::IsItemHovered()) xeditor::hint::Draw({ "Bones", "Draw the skeleton. Hidden, the character is still picked and dragged by its colliders" });
            ImGui::SameLine(); ImGui::Checkbox("Shapes", &m_bShowShapes);
            ImGui::SameLine(); ImGui::SetNextItemWidth(80); ImGui::SliderFloat("##Pull", &m_PullHertz, 1.0f, 30.0f, "pull %.1f Hz", ImGuiSliderFlags_Logarithmic);
            if (ImGui::IsItemHovered()) xeditor::hint::Draw({ "Pull", "How stiffly the hand holds a body while the simulation runs (a left drag on it): the frequency of a mouse joint's spring (Box2D's), its force at most four times the ragdoll's weight" });
            ImGui::SameLine(); ImGui::Checkbox("Mirror", &m_bMirror);
            if (ImGui::IsItemHovered()) xeditor::hint::Draw({ "Mirror", "A shape edit changes its left/right twin the same way (the pair is found by the bone names)" });
            if (m_SelBody >= 0 && m_SelBody < int(m_Graph.m_Graph.m_Bodies.size()) && !m_Graph.m_Graph.m_Bodies[std::size_t(m_SelBody)].m_Shapes.empty())
            {   // the selected body's collider: capsule, sphere or box
                const auto K = m_Graph.m_Graph.m_Bodies[std::size_t(m_SelBody)].m_Shapes[0].m_Kind;
                int iKind = K == xanimation::shape_kind::CAPSULE ? 0 : K == xanimation::shape_kind::SPHERE ? 1 : 2;
                ImGui::SameLine(); ImGui::SetNextItemWidth(80);
                ImGui::BeginDisabled(m_Drop.m_bOk);
                if (ImGui::Combo("##Collider", &iKind, "Capsule\0Sphere\0Box\0")) SetShapeKind(m_SelBody, iKind == 0 ? xanimation::shape_kind::CAPSULE : iKind == 1 ? xanimation::shape_kind::SPHERE : xanimation::shape_kind::BOX);
                ImGui::EndDisabled();
                if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) xeditor::hint::Draw({ "Collider", "The selected body's shape: a capsule, a sphere or a box (sized to the one it replaces; with Mirror, its twin too)" });
            }
            pt::SpeedLeftOfPair(m_Speed, xlioncore::game_time::kScaleSteps, L, "the playback");
            pt::actions A;
            A.m_bBusy     = m_bCompiling;
            A.m_WhyNotPlay = m_pRagdoll && m_Graph.m_Graph.m_bOk ? std::string{} : (m_Error.empty() ? std::string("nothing compiled") : m_Error);
            A.m_pPlayHint = "Drops the ragdoll from its rest pose and simulates it live with the runtime the game's Play uses; left-drag a bone to pull it";
            A.m_pStopHint = "Back to the rest pose (the simulation ends)";
            A.m_pStepHint = "One simulation step (starting it if needed)";
            A.m_Play   = [this] { m_bPlaying = BeginSim(); };
            A.m_Stop   = [this] { EndSim(); m_Drop = {}; m_bPlaying = false; m_PlayT = 0; };
            A.m_Pause  = [this] { m_bPlaying = false; };
            A.m_Resume = [this, Len] { if (m_Sim.m_pR) m_PlayT = float(m_Drop.m_Frames.size() - 1) * m_Drop.m_Dt; else if (m_PlayT >= Len) m_PlayT = 0; m_bPlaying = true; };
            A.m_Step   = [this] { if (!m_Drop.m_bOk && !BeginSim()) return; m_bPlaying = false;
                                  if (m_Sim.m_pR) { StepSim(); m_PlayT = float(m_Drop.m_Frames.size() - 1) * m_Drop.m_Dt; }
                                  else m_PlayT = std::min(m_PlayT + m_Drop.m_Dt, float(m_Drop.m_Frames.size() - 1) * m_Drop.m_Dt); };
            pt::Buttons(!m_Drop.m_bOk ? pt::state::Stopped : m_bPlaying ? pt::state::Playing : pt::state::Paused, A, L);
            ImGui::SameLine(); ImGui::BeginDisabled(!m_Drop.m_bOk);
            if (ImGui::Button("Reset")) { const bool bWas = m_bPlaying; m_bPlaying = BeginSim() && bWas; }
            ImGui::EndDisabled();
            if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) xeditor::hint::Draw({ "Reset", "The ragdoll back to its start pose (the rest pose at the drop height); the simulation goes on from there (Stop: back to the rest pose)" });
            if (m_Drop.m_bOk)
            {
                ImGui::SetNextItemWidth(std::max(120.0f, ImGui::GetContentRegionAvail().x));
                if (ImGui::SliderFloat("##t", &m_PlayT, 0, Len, "%.2f s")) m_bPlaying = false;
                if (ImGui::IsItemHovered()) xeditor::hint::Draw({ "Time", "Where the recording is: a recording plays back, scrubbing does not simulate" });
                ImGui::TextDisabled("%s", Summary().c_str());
            }
            else if (!m_Drop.m_Note.empty()) ImGui::TextColored(ImVec4(1, 0.5f, 0.4f, 1), "%s", m_Drop.m_Note.c_str());
            m_CameraToolbar.Render(ImGui::GetContentRegionAvail(), m_Viewport.m_Scene.m_State, [this] { RenderViewportBody(); }
                                  , [this](bool bHorizontal, float ButtonHeight)
                                    {
                                        xeditor::transform_gizmo::Buttons(m_Gizmo, bHorizontal, ButtonHeight, "the selected body's shape");
                                        xeditor::scale_reference::Button(bHorizontal, ButtonHeight);
                                        xgeom_skin_editor::skin_under::Button(m_bShowSkin, bHorizontal, ButtonHeight, "The skin, lit, under the bones and the shapes");
                                    });
        }

        // The skin under the bones (m_Viewport.m_Under): the Skin Geom editor's preview, posed by the bones shown, seen by the same camera
        void RenderSkinUnder(const std::vector<xanimation::rigid>& Bones, const ImVec2& Min, const ImVec2& Avail) noexcept
        {
            if (!m_pSkinGeom || !m_pSkeleton) return;
            m_SkinWorld.resize(Bones.size());
            // the skin's own rest worlds (the Skin editor's: they carry the skeleton's scale, the bind space) moved by each bone's change from rest
            xgpu::tools::editors::ComputeRestBoneWorlds(*m_pSkeleton, m_SkinRest);
            if (m_SkinRest.size() != Bones.size() || m_Rest.size() != Bones.size()) return;
            for (std::size_t i = 0; i < Bones.size(); ++i)
            {
                const auto D = Bones[i] * m_Rest[i].Inverse();
                m_SkinWorld[i] = xmath::fmat4(xmath::fvec3(1, 1, 1), D.m_Q, D.m_P) * m_SkinRest[i];
            }
            m_SkinUnder.Render(*m_pSkinGeom, *m_pSkeleton, m_SkinWorld, m_Viewport, Min, Avail, m_bOpen);
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

            if (m_BoneTree.m_bTreeHovered) m_HoverBody = BodyOf(m_HoverBone);       // a row hovered in the tree lights its body
            if (!m_bHandleDrag) ShowDescShapes();                                   // (a drag shows its live shape until the release writes it)
            const int iLit = m_HoverBody >= 0 ? m_HoverBody : m_SelBody;
            m_SelBones.clear();
            if (m_SelBone >= 0) m_SelBones.insert(m_SelBone);
            if (m_bShowShapes)
                for (int b = 0; b < int(m_Graph.m_Graph.m_Bodies.size()); ++b)
                {
                    const auto& B = m_Graph.m_Graph.m_Bodies[std::size_t(b)];
                    const auto BW = Bones[std::size_t(B.m_Bone)] * B.m_BoneToBody;
                    const std::uint32_t C = b == iLit ? IM_COL32(255, 230, 60, 255) : IM_COL32(90, 230, 255, 255);
                    for (const auto& S : B.m_Shapes) EmitShape(S, BW * S.m_Local, C);
                }
            // drawn exactly as the Skeleton editor draws it: the style's colours, fills and shadow, the selected and hovered bone
            xskeleton_editor::viewport_item Items[1] = { { .m_pSkeleton = m_pSkeleton, .m_Bones = m_BoneWorlds, .m_pSelected = &m_SelBones, .m_Hovered = m_HoverBone } };
            m_Viewport.m_bGizmoHot = m_bHandleHot; m_Viewport.m_bGizmoDrag = m_bHandleDrag;
            if (m_bShowSkin && !m_pSkinGeom && !m_SkinRef.empty()) m_pSkinGeom = xresource::g_Mgr.getResource(m_SkinRef);     // (still loading when Resolve asked)
            if (m_bShowSkin && m_pSkinGeom && m_SkinUnder.Ready(m_pDevice)) m_Viewport.m_Under = [this, &Bones](const ImVec2& Min, const ImVec2& Avail) { RenderSkinUnder(Bones, Min, Avail); };
            else m_Viewport.m_Under = nullptr;
            xskeleton_editor::viewport_item Drawn[1] = { Items[0] };
            if (!m_bShowBones) Drawn[0].m_pSkeleton = nullptr;     // hidden: not drawn; picking still uses Items (and the colliders)
            m_Viewport.Render(Drawn);
            m_Viewport.m_Under = nullptr;
            if (!m_BoneWorlds.empty())
            {   // the human scale reference beside the character, as the Skeleton editor has it
                xmath::fvec3 BoxMin = m_BoneWorlds[0].m_Position, BoxMax = BoxMin;
                for (const auto& W : m_BoneWorlds) { BoxMin = BoxMin.Min(W.m_Position); BoxMax = BoxMax.Max(W.m_Position); }
                xeditor::scale_reference::DrawBesideBox(*ImGui::GetWindowDrawList(), m_Viewport.m_Scene.m_View.getW2C(), m_Viewport.m_Scene.m_View.getPosition(), m_Viewport.RectMin(), m_Viewport.RectSize(), BoxMin, BoxMax, 1.0f);
            }
            ShapeHandles(Bones);
            m_LastBones = Bones;
            Grab(Items[0]);

            // the body under the mouse: the bone the pick finds, posed from its body
            m_HoverBody = -1; if (!m_BoneTree.m_bTreeHovered) m_HoverBone = -1;
            if (m_Viewport.Over() && !m_bHandleHot && m_GrabBody < 0)
            {
                int iBone = -1; float T = 0;
                auto& Sv = m_Viewport.m_Scene.m_View;
                { const int iB = PickBody(Bones, Sv.getPosition(), Sv.RayFromScreen(ImGui::GetMousePos().x, ImGui::GetMousePos().y), T); if (iB >= 0) iBone = m_Graph.m_Graph.m_Bodies[std::size_t(iB)].m_Bone; }     // the colliders: easier to hit than the wedges
                if ((iBone >= 0 || m_Viewport.PickBone(Items[0], ImGui::GetMousePos(), iBone, T)) && iBone >= 0 && iBone < int(m_Graph.m_Graph.m_BoneBody.size()))
                {
                    m_HoverBody = m_Graph.m_Graph.m_BoneBody[std::size_t(iBone)]; m_HoverBone = iBone;
                    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) m_SelBone = iBone;
                    if (m_HoverBody >= 0)
                    {
                        const auto* pRow = m_HoverBody < int(m_Details.m_Bodies.size()) ? &m_Details.m_Bodies[std::size_t(m_HoverBody)] : nullptr;
                        if (pRow) ImGui::SetTooltip("%s (%s)\n%s\n%.2f kg", pRow->m_Bone.c_str(), pRow->m_Slot.c_str(), pRow->m_Shape.c_str(), pRow->m_Mass);
                        if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) m_SelBody = m_HoverBody;
                    }
                }
            }
        }

        // The hand: while the simulation runs, a left drag on a bone pulls its body (a critically damped spring to the mouse, PullRagdollBody)
        // on the plane through the grabbed point facing the camera, as a physics editor's mouse joint. The camera stays out of the drag.
        void Grab(const xskeleton_editor::viewport_item& It) noexcept
        {
            if (!m_Sim.m_pR) { m_GrabBody = -1; return; }
            auto& Sv = m_Viewport.m_Scene.m_View;
            const ImVec2 Mouse = ImGui::GetMousePos();
            const xmath::fvec3 Eye = Sv.getPosition(), Dir = Sv.RayFromScreen(Mouse.x, Mouse.y);
            xanimation::rigid F;
            if (m_GrabBody < 0)
            {
                if (!m_Viewport.Over() || m_bHandleHot || !ImGui::IsMouseClicked(ImGuiMouseButton_Left)) return;
                float T = 0;
                const int iBody = PickBody(m_LastBones, Eye, Dir, T);
                if (iBody < 0 || !xlioncore::physics::RagdollBodyFrame(*m_Sim.m_pR, iBody, F)) return;
                const xmath::fvec3 Hit = Eye + Dir * T;
                m_GrabBody = iBody; m_GrabLocal = F.Inverse().Apply(Hit); m_GrabNormal = Dir; m_GrabTarget = Hit;
            }
            if (!ImGui::IsMouseDown(ImGuiMouseButton_Left) || !xlioncore::physics::RagdollBodyFrame(*m_Sim.m_pR, m_GrabBody, F)) { m_GrabBody = -1; return; }
            const float d = Dir.Dot(m_GrabNormal);
            if (std::abs(d) > 1e-4f) m_GrabTarget = Eye + Dir * ((m_GrabTarget - Eye).Dot(m_GrabNormal) / d);
            m_bHandleHot = m_bHandleDrag = true;
            ImVec2 A, B;
            if (m_Viewport.Project(F.Apply(m_GrabLocal), A) && m_Viewport.Project(m_GrabTarget, B))
            {
                auto& DL = *ImGui::GetWindowDrawList();
                DL.AddLine(A, B, IM_COL32(255, 230, 60, 255), 2.0f);
                DL.AddCircleFilled(A, 4.0f, IM_COL32(255, 230, 60, 255));
                DL.AddCircle(B, 6.0f, IM_COL32(255, 230, 60, 255), 0, 2.0f);
            }
        }

        // The body whose collider the ray hits first (T: the distance along Dir); -1: none. Each shape in its own frame: a sphere, a box
        // (xeditor_tools picking), a capsule as spheres along its segment (ponytail: 9 spheres, a true ray-capsule if the gaps ever show)
        int PickBody(const std::vector<xanimation::rigid>& Bones, const xmath::fvec3& Eye, const xmath::fvec3& Dir, float& T) const noexcept
        {
            namespace pk = xeditor_tools::picking;
            int iBest = -1; T = 1e30f;
            for (int b = 0; b < int(m_Graph.m_Graph.m_Bodies.size()); ++b)
            {
                const auto& B = m_Graph.m_Graph.m_Bodies[std::size_t(b)];
                if (B.m_Bone < 0 || B.m_Bone >= int(Bones.size())) continue;
                for (const auto& S : B.m_Shapes)
                {
                    const auto Wi = (Bones[std::size_t(B.m_Bone)] * B.m_BoneToBody * S.m_Local).Inverse();
                    const xmath::fvec3 O = Wi.Apply(Eye), D = Wi.m_Q * Dir;
                    float t = 0; bool bHit = false;
                    if (S.m_Kind == xanimation::shape_kind::BOX) bHit = pk::RayAABBIntersect(O, D, xmath::fvec3(0, 0, 0), S.m_HalfExtents, t);
                    else if (S.m_Kind == xanimation::shape_kind::SPHERE) bHit = pk::RaySphereIntersect(O, D, xmath::fvec3(0, 0, 0), S.m_Radius, t);
                    else for (int k = 0; k < 9; ++k)
                    {
                        float tk = 0;
                        if (pk::RaySphereIntersect(O, D, xmath::fvec3(0, S.m_HalfSegment * (float(k) / 4.0f - 1.0f), 0), S.m_Radius, tk) && (!bHit || tk < t)) { t = tk; bHit = true; }
                    }
                    if (bHit && t > 0 && t < T) { T = t; iBest = b; }
                }
            }
            return iBest;
        }

        // The body a bone is posed from (its own, or the nearest above it); -1: none
        int BodyOf(int iBone) const noexcept { return iBone >= 0 && iBone < int(m_Graph.m_Graph.m_BoneBody.size()) ? m_Graph.m_Graph.m_BoneBody[std::size_t(iBone)] : -1; }

        // The skeleton's bones in the shared bone tree (the Skeleton editor's): a bone selects its body; a body bone says so
        void RenderBones() noexcept
        {
            Resolve();
            const auto& Names = m_Details.m_BoneNames;
            if (!m_pSkeleton || Names.size() != m_Skel.m_Parent.size()) { ImGui::TextDisabled("No bones yet: Compile."); return; }
            if (m_BoneRows.size() != Names.size())
            {
                m_BoneRows.assign(Names.size(), {});
                for (int i = 0; i < int(Names.size()); ++i) m_BoneRows[std::size_t(i)] = { .m_Name = Names[std::size_t(i)], .m_iParent = m_Skel.m_Parent[std::size_t(i)], .m_iBone = i };
            }
            for (int i = 0; i < int(Names.size()); ++i)
            {
                auto& R = m_BoneRows[std::size_t(i)];
                const int Body = BodyOf(i);
                const bool bOwn = Body >= 0 && m_Graph.m_Graph.m_Bodies[std::size_t(Body)].m_Bone == i;
                R.m_Label = bOwn ? Names[std::size_t(i)] + "  (body)" : std::string{};
                R.m_TextColor = bOwn ? 0 : ImGui::GetColorU32(ImGuiCol_TextDisabled);
            }
            const std::set<int> Sel = m_SelBone >= 0 ? std::set<int>{ m_SelBone } : std::set<int>{};
            m_BoneTree.Follow(m_BoneRows, Sel, m_HoverBone);
            if (ImGui::BeginTable("##Bones", 1, ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY))
            {
                xskeleton_editor::bone_tree::hooks H;
                H.m_Select = [this](int iBone, bool, bool) { m_SelBone = iBone; m_SelBody = BodyOf(iBone); };
                m_BoneTree.Render(m_BoneRows, Sel, m_HoverBone, H);
                ImGui::EndTable();
            }
        }

        // The selected body's shape handles (the Level editor's collider handles, xlevel_viewport_tools.h: the same dots, drag and readout):
        // a capsule's radius (4 around) and length (its 2 tips), a sphere's radius, a box's half extents. The drag edits the shown shape live;
        // the release writes it to the bone's override (ShapeSet: the shape drawn by hand replaces the fitted one) as one undo step (no compile).
        // Rest pose only (a recording's playback is not edited).
        void ShapeHandles(const std::vector<xanimation::rigid>& Bones) noexcept
        {
            namespace vt = xlevel::viewport_tools;
            m_bHandleHot = false;
            const bool bWasDrag = m_bHandleDrag; m_bHandleDrag = false;
            if (!m_bShowShapes || m_Drop.m_bOk || m_SelBody < 0 || m_SelBody >= int(m_Graph.m_Graph.m_Bodies.size())) return;
            auto& B = m_Graph.m_Graph.m_Bodies[std::size_t(m_SelBody)];
            if (B.m_Shapes.empty()) return;
            auto& S = B.m_Shapes[0];
            const auto W = Bones[std::size_t(B.m_Bone)] * B.m_BoneToBody * S.m_Local;
            auto& Sv = m_Viewport.m_Scene.m_View;
            vt::view V;
            V.m_W2C = Sv.getW2C(); V.m_Min = m_Viewport.RectMin(); V.m_Size = m_Viewport.RectSize(); V.m_Eye = Sv.getPosition();
            V.m_RayDir = [&Sv](float x, float y) { return Sv.RayFromScreen(x, y); };
            int iTwin = m_bMirror ? MirrorBody(m_SelBody) : -1;
            namespace tg = xeditor::transform_gizmo;
            if (m_Gizmo.m_Tool == tg::TOOL_MOVE || m_Gizmo.m_Tool == tg::TOOL_ROTATE)
            {   // Move / Rotate: the previews' gizmo on the shape's frame; its twin is placed as its mirror image
                const auto BW = Bones[std::size_t(B.m_Bone)] * B.m_BoneToBody;
                xmath::fmat4 M(xmath::fvec3(1, 1, 1), W.m_Q, W.m_P);
                const auto R = tg::Manipulate(m_Gizmo, Sv, m_Viewport.RectMin(), m_Viewport.RectSize(), M);
                m_bHandleHot = R.m_bInteracting; m_bHandleDrag = m_Gizmo.m_bWasUsing;
                if (R.m_bChanged) S.m_Local = BW.Inverse() * xanimation::rigid{ M.ExtractPosition(), M.ExtractRotation().NormalizeCopy() };
                if (iTwin >= 0 && m_bHandleDrag) MirrorPlace(m_SelBody, iTwin, Bones);
                if (R.m_bEnded) CommitShape(m_SelBody, iTwin);
                return;
            }
            const xmath::fvec3 X = W.m_Q * xmath::fvec3(1, 0, 0), Y = W.m_Q * xmath::fvec3(0, 1, 0), Z = W.m_Q * xmath::fvec3(0, 0, 1);
            const ImGuiID Base = 0x5D000000u + ImGuiID(m_SelBody) * 16u;
            auto One = [&](int Slot, const xmath::fvec3& Dir, float T, auto&& Set)
            {
                const auto H = vt::handle::Axis(V, Base + ImGuiID(Slot), W.m_P, Dir, T);
                m_bHandleHot |= H.m_bHovered || H.m_bActive; m_bHandleDrag |= H.m_bActive;
                if (H.m_bChanged) { char Text[64]; Set(H.m_T, Text); vt::handle::Label(H, Text); }
            };
            if (S.m_Kind == xanimation::shape_kind::CAPSULE)
            {
                const xmath::fvec3 Around[4] = { X, X * -1.0f, Z, Z * -1.0f };
                for (int i = 0; i < 4; ++i) One(i, Around[i], S.m_Radius, [&](float T, char* Txt) { S.m_Radius = std::clamp(T, 0.005f, 1.0f); std::snprintf(Txt, 64, "Radius %.3f", S.m_Radius); });
                for (int s = 0; s < 2; ++s) One(4 + s, s ? Y * -1.0f : Y, S.m_HalfSegment + S.m_Radius
                                               , [&](float T, char* Txt) { S.m_HalfSegment = std::max(T - S.m_Radius, 0.0f); std::snprintf(Txt, 64, "Length %.3f", 2 * (S.m_HalfSegment + S.m_Radius)); });
            }
            else if (S.m_Kind == xanimation::shape_kind::SPHERE)
            {
                const xmath::fvec3 Ax[6] = { X, X * -1.0f, Y, Y * -1.0f, Z, Z * -1.0f };
                for (int i = 0; i < 6; ++i) One(i, Ax[i], S.m_Radius, [&](float T, char* Txt) { S.m_Radius = std::clamp(T, 0.005f, 1.0f); std::snprintf(Txt, 64, "Radius %.3f", S.m_Radius); });
            }
            else
            {
                const xmath::fvec3 Ax[3] = { X, Y, Z }; float* H[3] = { &S.m_HalfExtents.m_X, &S.m_HalfExtents.m_Y, &S.m_HalfExtents.m_Z };
                for (int a = 0; a < 3; ++a) for (int s = 0; s < 2; ++s)
                    One(a * 2 + s, s ? Ax[a] * -1.0f : Ax[a], *H[a], [&, a](float T, char* Txt) { *H[a] = std::clamp(T, 0.005f, 1.0f); std::snprintf(Txt, 64, "Half %.3f", *H[a]); });
            }
            // Mirror: the twin takes the same sizes (its own place and turn stay: the handles only resize)
            if (iTwin >= 0 && !m_Graph.m_Graph.m_Bodies[std::size_t(iTwin)].m_Shapes.empty())
            {
                auto& T = m_Graph.m_Graph.m_Bodies[std::size_t(iTwin)].m_Shapes[0];
                if (T.m_Kind == S.m_Kind) { T.m_Radius = S.m_Radius; T.m_HalfSegment = S.m_HalfSegment; T.m_HalfExtents = S.m_HalfExtents; }
                else iTwin = -1;
            }
            if (bWasDrag && !m_bHandleDrag) CommitShape(m_SelBody, iTwin);
        }

        // The twin's shape as the mirror image of iBody's, in model space: reflected across the plane between the two bones, on the axis
        // they are apart along (left/right), then into the twin's body frame
        void MirrorPlace(int iBody, int iTwin, const std::vector<xanimation::rigid>& Bones) noexcept
        {
            const auto& Bb = m_Graph.m_Graph.m_Bodies[std::size_t(iBody)];
            auto&       Bt = m_Graph.m_Graph.m_Bodies[std::size_t(iTwin)];
            if (Bb.m_Shapes.empty() || Bt.m_Shapes.empty()) return;
            const auto BWb = Bones[std::size_t(Bb.m_Bone)] * Bb.m_BoneToBody, BWt = Bones[std::size_t(Bt.m_Bone)] * Bt.m_BoneToBody;
            auto M = BWb * Bb.m_Shapes[0].m_Local;
            const auto Pa = Bones[std::size_t(Bb.m_Bone)].m_P, Pb = Bones[std::size_t(Bt.m_Bone)].m_P;
            const xmath::fvec3 D = Pb - Pa;
            const int a = std::abs(D.m_X) >= std::abs(D.m_Y) && std::abs(D.m_X) >= std::abs(D.m_Z) ? 0 : std::abs(D.m_Y) >= std::abs(D.m_Z) ? 1 : 2;
            auto& Q = M.m_Q;
            if (a == 0) { M.m_P.m_X = Pa.m_X + Pb.m_X - M.m_P.m_X; Q = xmath::fquat(Q.m_X, -Q.m_Y, -Q.m_Z, Q.m_W); }
            if (a == 1) { M.m_P.m_Y = Pa.m_Y + Pb.m_Y - M.m_P.m_Y; Q = xmath::fquat(-Q.m_X, Q.m_Y, -Q.m_Z, Q.m_W); }
            if (a == 2) { M.m_P.m_Z = Pa.m_Z + Pb.m_Z - M.m_P.m_Z; Q = xmath::fquat(-Q.m_X, -Q.m_Y, Q.m_Z, Q.m_W); }
            auto& T = Bt.m_Shapes[0];
            const auto& S = Bb.m_Shapes[0];
            T.m_Kind = S.m_Kind; T.m_Radius = S.m_Radius; T.m_HalfSegment = S.m_HalfSegment; T.m_HalfExtents = S.m_HalfExtents;
            T.m_Local = BWt.Inverse() * M;
        }

        // A body's collider becomes a capsule, a sphere or a box, sized to the one it replaces (same centre and turn); its twin too (Mirror)
        void SetShapeKind(int iBody, xanimation::shape_kind Kind) noexcept
        {
            const int iTwin = m_bMirror ? MirrorBody(iBody) : -1;
            for (int b : { iBody, iTwin })
            {
                if (b < 0 || m_Graph.m_Graph.m_Bodies[std::size_t(b)].m_Shapes.empty()) continue;
                auto& S = m_Graph.m_Graph.m_Bodies[std::size_t(b)].m_Shapes[0];
                if (S.m_Kind == Kind) continue;
                // the old one's half sizes along its own axes (Y: a capsule's length)
                xmath::fvec3 H = S.m_Kind == xanimation::shape_kind::BOX ? S.m_HalfExtents
                               : S.m_Kind == xanimation::shape_kind::CAPSULE ? xmath::fvec3(S.m_Radius, S.m_HalfSegment + S.m_Radius, S.m_Radius)
                               : xmath::fvec3(S.m_Radius, S.m_Radius, S.m_Radius);
                if (Kind == xanimation::shape_kind::BOX) S.m_HalfExtents = H;
                else if (Kind == xanimation::shape_kind::SPHERE) S.m_Radius = std::max({ H.m_X, H.m_Y, H.m_Z });
                else { S.m_Radius = std::max(std::max(H.m_X, H.m_Z), 0.005f); S.m_HalfSegment = std::max(H.m_Y - S.m_Radius, 0.0f); }
                S.m_Kind = Kind;
            }
            CommitShape(iBody, iTwin);
        }

        // The edited shape (and its mirrored twin) into the bones' overrides, one undo step (SnapshotEdit). No compile: the view shows the
        // descriptor's shapes (ShowDescShapes) and Play drops them; the compiled resource follows when the user compiles or saves.
        void CommitShape(int iBody, int iTwin) noexcept
        {
            auto* pDesc = Desc();
            if (!pDesc || iBody >= int(m_Details.m_Bodies.size())) return;
            xproperty::settings::context C;
            const auto Before = xproperty::ui::undo::SnapshotToString(*pDesc->getProperties(), pDesc, C);
            std::string Label = m_Details.m_Bodies[std::size_t(iBody)].m_Bone;
            for (int b : { iBody, iTwin })
            {
                if (b < 0 || b >= int(m_Details.m_Bodies.size()) || m_Graph.m_Graph.m_Bodies[std::size_t(b)].m_Shapes.empty()) continue;
                const auto& S = m_Graph.m_Graph.m_Bodies[std::size_t(b)].m_Shapes[0];
                const std::string Name = m_Details.m_Bodies[std::size_t(b)].m_Bone;
                if (b == iTwin) Label += " + " + Name;
                auto It = std::find_if(pDesc->m_Overrides.begin(), pDesc->m_Overrides.end(), [&](const auto& O) { return O.m_Bone == Name; });
                auto& O = It != pDesc->m_Overrides.end() ? *It : pDesc->m_Overrides.emplace_back();
                O.m_Bone = Name; O.m_bShapeSet = true;
                O.m_Shape = S.m_Kind == xanimation::shape_kind::CAPSULE ? xragdoll_desc::shape_choice::CAPSULE : S.m_Kind == xanimation::shape_kind::SPHERE ? xragdoll_desc::shape_choice::SPHERE : xragdoll_desc::shape_choice::BOX;
                O.m_ShapeCentre = S.m_Local.m_P; O.m_ShapeRotation = S.m_Local.m_Q; O.m_ShapeRadius = S.m_Radius; O.m_ShapeHalfSegment = S.m_HalfSegment; O.m_ShapeHalfExtents = S.m_HalfExtents;
            }
            const auto After = xproperty::ui::undo::SnapshotToString(*pDesc->getProperties(), pDesc, C);
            xproperty::ui::undo::ApplySnapshotFromString(*pDesc->getProperties(), pDesc, Before, C);     // the command applies it (and undoes it)
            m_Undo.Execute(std::format("SnapshotEdit -Label {} -Before {} -After {}", xeditor::Quote(std::format("Shape {}", Label)), xeditor::Quote(Before), xeditor::Quote(After)));
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

#ifndef XRAGDOLL_GENERATE_H
#define XRAGDOLL_GENERATE_H
#pragma once

// The body model generator (ragdoll design v2 sections 3, 5, 6, 7): skeleton + skin + descriptor -> bodies, shapes, joints, mass, filter,
// capsule and the capability report. Shared by the compiler and the editor (R5); no file access here (the caller reads the manifest and
// the skin). Deterministic: no random generator, fixed orders.
//
// Policy (v1):
//  - Body bones: the skeleton's slots (xanimation's name classifier): hips, every spine bone, neck, head, upper/lower arm, hand, thigh,
//    calf, foot (the first bone of each slot and side down the hierarchy). A candidate with less than MinBodyShare of the skin follows the
//    body above it (significance); the hips always has a body (the root body).
//  - Bones between two bodies (a clavicle, a dropped neck) follow the body above (removed articulations, reported). Fingers / toes follow
//    their hand / foot by the named policies (reported), else keep their animation. Every other bone (helpers, twist, hair) keeps its
//    animation relative to its body. Bones above the hips keep their animation (the entity root follows the hips body: R4).
//  - One shape per body, fitted in the body's frame (= its bone's frame) to the skin samples dominated by the bones it poses: head a
//    sphere, hand and foot a box, the rest a capsule (limbs along the bone toward the next body, hips and spine along their own long axis).
//  - Mass from the shapes' volumes x density, or the total mass shared by volume; inertia of the shape about its centre.
//  - Filter: jointed pairs never collide (the solver's collideConnected off); other pairs that overlap at rest are excluded and listed.
#include "dependencies/xAnimation/source/physical/xanimation_body_model.h"
#include "dependencies/xAnimation/source/retarget/xanimation_bonemap.h"
#include "dependencies/xcollision/source/xcollision.h"
#include "plugins/xskeleton.plugin/source/xskeleton_manifest_limits.h"
#include "xragdoll.h"
#include "xragdoll_descriptor.h"
#include "xragdoll_details.h"
#include <algorithm>
#include <format>
#include <string>
#include <vector>

namespace xragdoll_gen
{
    using vec3  = xmath::fvec3;
    using quat  = xmath::fquat;
    using rigid = xanimation::rigid;

    // The skeleton as the ragdoll sees it: rigid frames (the uniform scale folded into positions, metres), its limits (empty: Off)
    struct skeleton_in
    {
        std::vector<std::string>                m_Names;
        std::vector<std::int32_t>               m_Parent;
        std::vector<std::uint32_t>              m_NameHash;
        xanimation::pose                        m_Rest;         // rigid local rest (scale 1)
        std::vector<rigid>                      m_World;        // rigid model-space rest frames
        std::vector<float>                      m_WorldScale;   // each bone's uniform world scale at rest
        std::vector<xanimation::joint_limit>    m_Limits;
        int count(void) const noexcept { return int(m_Parent.size()); }
    };

    // The fitting skin: model-space bind-pose positions, triangles, up to 4 influences per vertex (skeleton bone indices)
    struct skin_mesh
    {
        std::vector<vec3>                       m_Pos;
        std::vector<std::uint32_t>              m_Idx;
        std::vector<xcollision::vertex_weights> m_W;
    };

    inline skeleton_in FromManifest(const xskeleton_desc::bone_manifest& M, std::vector<std::string>& Warnings)
    {
        skeleton_in S; const int N = int(M.m_Bones.size());
        xanimation::pose Raw; Raw.resize(N);
        for (int i = 0; i < N; ++i)
        {
            const auto& B = M.m_Bones[i];
            S.m_Names.push_back(B.m_Name); S.m_Parent.push_back(B.m_iParent); S.m_NameHash.push_back(B.m_NameHash);
            Raw.m_Position[i] = B.m_RestPosition; Raw.m_Rotation[i] = quat(B.m_RestRotX, B.m_RestRotY, B.m_RestRotZ, B.m_RestRotW).NormalizeSafeCopy(); Raw.m_Scale[i] = B.m_RestScale;
            const auto& s = B.m_RestScale;
            if (std::abs(s.m_X - s.m_Y) > 1e-3f * std::abs(s.m_X) || std::abs(s.m_X - s.m_Z) > 1e-3f * std::abs(s.m_X))
                Warnings.push_back(std::format("{} has a non-uniform rest scale ({}, {}, {}): treated as uniform", B.m_Name, s.m_X, s.m_Y, s.m_Z));
        }
        const xanimation::skeleton_view View{ S.m_Parent };
        std::vector<quat> R; std::vector<vec3> P, Sc; xanimation::details::WorldRPS(View, Raw, R, P, Sc);
        S.m_Rest.resize(N); S.m_World.resize(N); S.m_WorldScale.resize(N);
        for (int i = 0; i < N; ++i)
        {
            S.m_World[i] = { P[i], R[i].NormalizeSafeCopy() };
            S.m_WorldScale[i] = Sc[i].m_X;
            const int p = S.m_Parent[i];
            const rigid L = p < 0 ? S.m_World[i] : S.m_World[p].Inverse() * S.m_World[i];
            S.m_Rest.m_Position[i] = L.m_P; S.m_Rest.m_Rotation[i] = L.m_Q; S.m_Rest.m_Scale[i] = vec3(1, 1, 1);
        }
        S.m_Limits = xskeleton_desc::ManifestLimits(M);
        if (S.m_Limits.empty()) Warnings.push_back("The skeleton's joint limits are Off: every joint is free (a ragdoll needs them: turn them on in the Skeleton editor)");
        return S;
    }

    struct result
    {
        xanimation::body_graph                  m_Graph;
        std::vector<int>                        m_Slot;         // per body: xanimation::bone_slot
        std::vector<xragdoll::shape>            m_Shapes;       // one per body, body order
        std::vector<xragdoll::filter_pair>      m_Filter;
        xragdoll::capsule                       m_Capsule;
        xragdoll_desc::details                  m_Details;
        std::uint64_t                           m_Signature = 0;
        float                                   m_WorldScale = 1;
    };

    namespace details
    {
        inline const char* SlotName(xanimation::bone_slot s) noexcept
        {
            static const char* N[] = { "", "root", "hips", "spine", "neck", "head", "clavicle", "upperarm", "lowerarm", "hand", "thigh", "calf", "foot", "toe" };
            return N[int(s)];
        }
        inline quat FromTo(const vec3& A, const vec3& B) noexcept
        {
            const vec3 a = A.NormalizeSafeCopy(), b = B.NormalizeSafeCopy();
            const float c = std::clamp(a.Dot(b), -1.0f, 1.0f);
            vec3 Ax = a.Cross(b);
            if (Ax.LengthSq() < 1e-12f) { if (c > 0) return quat::fromIdentity(); Ax = std::abs(a.m_X) < 0.9f ? a.Cross(vec3(1, 0, 0)) : a.Cross(vec3(0, 1, 0)); }
            return xanimation::details::AxisAngle(Ax.NormalizeSafeCopy(), std::acos(c));
        }
        inline quat FromAxes(vec3 X, vec3 Y) noexcept      // the rotation whose x and y are X and Y (made orthonormal, right-handed)
        {
            X = X.NormalizeSafeCopy(); Y = (Y - X * X.Dot(Y)).NormalizeSafeCopy();
            const vec3 Z = X.Cross(Y);
            return quat::LookRotation(Z, Y).NormalizeSafeCopy();
        }
        inline xragdoll::rigid ToFile(const rigid& R) noexcept { return { { R.m_P.m_X, R.m_P.m_Y, R.m_P.m_Z }, { R.m_Q.m_X, R.m_Q.m_Y, R.m_Q.m_Z, R.m_Q.m_W } }; }
        inline rigid FromFile(const xragdoll::rigid& R) noexcept { return { vec3(R.m_P[0], R.m_P[1], R.m_P[2]), quat(R.m_Q[0], R.m_Q[1], R.m_Q[2], R.m_Q[3]) }; }

        // a shape (in its body's frame) placed by the body's frame into xcollision's form
        struct placed { int m_Kind = 0; xcollision::capsule m_C; xcollision::sphere m_S; xcollision::box m_B; };
        inline placed Place(const xragdoll::shape& Sh, const rigid& Frame) noexcept
        {
            placed O; O.m_Kind = Sh.m_Kind; const rigid W = Frame * FromFile(Sh.m_Local);
            if (Sh.m_Kind == 0) { const vec3 D = W.m_Q * vec3(0, Sh.m_HalfSegment, 0); O.m_C = { W.m_P - D, W.m_P + D, Sh.m_Radius }; }
            else if (Sh.m_Kind == 1) O.m_S = { W.m_P, Sh.m_Radius };
            else O.m_B = { W.m_P, { W.m_Q * vec3(1, 0, 0), W.m_Q * vec3(0, 1, 0), W.m_Q * vec3(0, 0, 1) }, vec3(Sh.m_HalfExtents[0], Sh.m_HalfExtents[1], Sh.m_HalfExtents[2]) };
            return O;
        }
        inline float Dist(const placed& P, const vec3& X) noexcept { return P.m_Kind == 0 ? xcollision::Distance(P.m_C, X) : P.m_Kind == 1 ? xcollision::Distance(P.m_S, X) : xcollision::Distance(P.m_B, X); }
        inline std::vector<vec3> Surface(const placed& P, int N) { return P.m_Kind == 0 ? xcollision::SurfacePoints(P.m_C, N) : P.m_Kind == 1 ? xcollision::SurfacePoints(P.m_S, N) : xcollision::SurfacePoints(P.m_B, N); }
        // the deepest penetration of B's surface into A or A's into B (0: apart)
        inline float Overlap(const placed& A, const placed& B) noexcept
        {
            float D = 0;
            for (const auto& p : Surface(B, 128)) D = std::min(D, Dist(A, p));
            for (const auto& p : Surface(A, 128)) D = std::min(D, Dist(B, p));
            return -D;
        }

        inline void Inertia(xragdoll::shape& Sh, float m, float I[3]) noexcept      // principal moments about the shape's centre, in its frame
        {
            if (Sh.m_Kind == 1) { I[0] = I[1] = I[2] = 0.4f * m * Sh.m_Radius * Sh.m_Radius; return; }
            if (Sh.m_Kind == 2)
            {
                const float x = Sh.m_HalfExtents[0], y = Sh.m_HalfExtents[1], z = Sh.m_HalfExtents[2];
                I[0] = m / 3 * (y * y + z * z); I[1] = m / 3 * (x * x + z * z); I[2] = m / 3 * (x * x + y * y); return;
            }
            const float r = Sh.m_Radius, h = 2 * Sh.m_HalfSegment, Vc = 3.14159265f * r * r * h, Vs = 4.0f / 3.0f * 3.14159265f * r * r * r;
            const float mc = m * Vc / (Vc + Vs), ms = m * Vs / (Vc + Vs);
            I[1] = mc * r * r / 2 + ms * 0.4f * r * r;                                                       // about the segment (y)
            I[0] = I[2] = mc * (h * h / 12 + r * r / 4) + ms * (0.4f * r * r + h * h / 4 + 3 * h * r / 8);    // across it
        }
        inline float Volume(const xragdoll::shape& Sh) noexcept
        {
            if (Sh.m_Kind == 1) return 4.0f / 3.0f * 3.14159265f * Sh.m_Radius * Sh.m_Radius * Sh.m_Radius;
            if (Sh.m_Kind == 2) return 8 * Sh.m_HalfExtents[0] * Sh.m_HalfExtents[1] * Sh.m_HalfExtents[2];
            return 3.14159265f * Sh.m_Radius * Sh.m_Radius * (2 * Sh.m_HalfSegment + 4.0f / 3.0f * Sh.m_Radius);
        }
        inline std::string Describe(const xragdoll::shape& Sh)
        {
            if (Sh.m_Kind == 0) return std::format("capsule r {:.3f} length {:.3f}", Sh.m_Radius, 2 * (Sh.m_HalfSegment + Sh.m_Radius));
            if (Sh.m_Kind == 1) return std::format("sphere r {:.3f}", Sh.m_Radius);
            return std::format("box {:.3f} x {:.3f} x {:.3f}", 2 * Sh.m_HalfExtents[0], 2 * Sh.m_HalfExtents[1], 2 * Sh.m_HalfExtents[2]);
        }
    }

    // InvBind: the skin's inverse bind matrices (the skeleton's), one per bone; empty: the bind pose is the rest pose.
    inline result Generate(const skeleton_in& S, const skin_mesh& Skin, std::span<const xmath::fmat4> InvBind, const xragdoll_desc::descriptor& D, std::vector<std::string> Warnings = {})
    {
        using namespace xanimation;
        result R; auto& Det = R.m_Details;
        const int nB = S.count();
        const skeleton_view View{ S.m_Parent };
        auto Nm = [&](int b) { return b >= 0 && b < nB ? S.m_Names[b] : std::string("?"); };
        auto Fail = [&](std::string Msg) { Det.m_bOk = false; Det.m_Report.push_back("error: " + Msg); Det.m_Warnings.insert(Det.m_Warnings.end(), Warnings.begin(), Warnings.end()); return R; };

        R.m_Signature = 1469598103934665603ull;
        for (int i = 0; i < nB; ++i) { R.m_Signature = (R.m_Signature ^ S.m_NameHash[i]) * 1099511628211ull; R.m_Signature = (R.m_Signature ^ std::uint64_t(std::uint32_t(S.m_Parent[i]))) * 1099511628211ull; }
        R.m_WorldScale = nB ? S.m_WorldScale[0] : 1.0f;
        if (Skin.m_Pos.empty() || Skin.m_Idx.size() < 3) return Fail("the skin has no triangles");
        if (!InvBind.empty() && int(InvBind.size()) != nB) return Fail("one inverse bind matrix per bone is needed");

        // the skin's samples, each with the bone that dominates it
        const auto Samples = xcollision::SampleSurface(Skin.m_Pos, Skin.m_Idx, 40000);
        const auto Assigned = xcollision::AssignByDominantWeight(Samples, Skin.m_Idx, Skin.m_W, nB);
        if (Assigned.m_Unassigned) Warnings.push_back(std::format("{} of {} skin samples have no bone weight", Assigned.m_Unassigned, Samples.size()));

        // slots
        std::vector<std::string_view> Names(S.m_Names.begin(), S.m_Names.end());
        const auto Slots = xanimation::details::BuildSlots(View, Names, S.m_Rest);
        std::vector<bone_slot> SlotOf(nB, bone_slot::NONE);
        std::vector<char> Candidate(nB, 0);
        int Hips = -1;
        for (const auto& [Key, Bones] : Slots.m_Bones)
        {
            for (int b : Bones) SlotOf[b] = Key.first;
            switch (Key.first)
            {
            case bone_slot::HIPS: Hips = Bones.front(); Candidate[Hips] = 1; break;
            case bone_slot::SPINE: for (int b : Bones) Candidate[b] = 1; break;
            case bone_slot::NECK: case bone_slot::HEAD: case bone_slot::UPPERARM: case bone_slot::LOWERARM: case bone_slot::HAND:
            case bone_slot::THIGH: case bone_slot::CALF: case bone_slot::FOOT: Candidate[Bones.front()] = 1; break;
            default: break;
            }
        }
        if (Hips < 0) return Fail("no hips bone found by name: a ragdoll needs a root body (name it, or override a bone's disposition to Body)");
        auto IsAncestor = [&](int a, int b) { for (int x = S.m_Parent[b]; x >= 0; x = S.m_Parent[x]) if (x == a) return true; return false; };
        // spine bones above the hips (some rigs) are not bodies: the hips is the root body
        for (int b = 0; b < nB; ++b) if (Candidate[b] && b != Hips && !IsAncestor(Hips, b)) Candidate[b] = 0;
        // a neck the arms hang from (3ds Max Biped: the clavicles are the neck's children) would carry the arms' weight and whip the head:
        // it follows the chest, and the head and the arms join the chest
        for (int n = 0; n < nB; ++n) if (Candidate[n] && SlotOf[n] == bone_slot::NECK)
            for (int a = 0; a < nB; ++a) if (Candidate[a] && SlotOf[a] == bone_slot::UPPERARM && IsAncestor(n, a))
            { Candidate[n] = 0; Det.m_Report.push_back(std::format("dropped body: the arms hang from {} in this rig: it follows the chest", Nm(n))); break; }

        auto FindOverride = [&](int b) -> const xragdoll_desc::bone_override* { for (const auto& O : D.m_Overrides) if (O.m_Bone == S.m_Names[b]) return &O; return nullptr; };
        for (const auto& O : D.m_Overrides) if (std::find(S.m_Names.begin(), S.m_Names.end(), O.m_Bone) == S.m_Names.end()) Warnings.push_back(std::format("override for '{}': no such bone in the skeleton (ignored)", O.m_Bone));

        auto Dispositions = [&](const std::vector<char>& Body)
        {
            std::vector<bone_disposition> Disp(nB, bone_disposition::ANIMATED);
            auto Under = [&](int b, bone_slot Slot) { for (int x = S.m_Parent[b]; x >= 0; x = S.m_Parent[x]) { if (Body[x] && SlotOf[x] == Slot) return true; if (Body[x]) return false; } return false; };
            auto BodyBelow = [&](int b) { for (int c = b + 1; c < nB; ++c) if (Body[c] && IsAncestor(b, c)) return true; return false; };
            for (int b = 0; b < nB; ++b)
            {
                if (Body[b]) Disp[b] = bone_disposition::BODY;
                else if (b == Hips || !IsAncestor(Hips, b)) Disp[b] = bone_disposition::ANIMATED;      // above or beside the root body
                else if (BodyBelow(b)) Disp[b] = bone_disposition::FOLLOWS;                             // between two bodies
                else if (Under(b, bone_slot::HAND)) Disp[b] = D.m_bFingersFollow ? bone_disposition::FOLLOWS : bone_disposition::ANIMATED;
                else if (Under(b, bone_slot::FOOT)) Disp[b] = D.m_bToesFollow    ? bone_disposition::FOLLOWS : bone_disposition::ANIMATED;
                if (const auto* O = FindOverride(b); O && O->m_Disposition != xragdoll_desc::disposition::AUTO)
                    Disp[b] = static_cast<bone_disposition>(int(O->m_Disposition) - 1);
            }
            return Disp;
        };

        // significance: a candidate body with too little of the skin follows the body above it
        std::vector<char> Body = Candidate;
        {
            const auto G0 = BuildBodyGraph(View, S.m_Rest, S.m_Limits, Dispositions(Body), S.m_Names);
            std::vector<float> Share(G0.m_Bodies.size(), 0.0f);
            for (int b = 0; b < nB; ++b) if (G0.m_BoneBody[b] >= 0) Share[G0.m_BoneBody[b]] += Assigned.m_Share[b];
            for (std::size_t i = 0; i < G0.m_Bodies.size(); ++i)
            {
                const int b = G0.m_Bodies[i].m_Bone;
                const auto* O = FindOverride(b);
                if (b != Hips && Share[i] < D.m_MinBodyShare && !(O && O->m_Disposition == xragdoll_desc::disposition::BODY))
                {
                    Body[b] = 0;
                    Det.m_Report.push_back(std::format("dropped body: {} has {:.2f} % of the skin (below MinBodyShare {:.2f} %): it follows the body above", Nm(b), 100 * Share[i], 100 * D.m_MinBodyShare));
                }
            }
        }
        const auto Disp = Dispositions(Body);
        R.m_Graph = BuildBodyGraph(View, S.m_Rest, S.m_Limits, Disp, S.m_Names);
        auto& G = R.m_Graph;
        if (!G.m_bOk) { Det.m_Report.insert(Det.m_Report.end(), G.m_Report.begin(), G.m_Report.end()); return Fail("the body graph could not be built"); }
        const int nBodies = int(G.m_Bodies.size());

        // each body's samples, in its frame (the bind pose's: through the inverse bind when given)
        auto BoneLocal = [&](int b, const vec3& p)
        {
            if (InvBind.empty()) return S.m_World[b].Inverse().Apply(p);
            return (InvBind[b] * p) * S.m_WorldScale[b];
        };
        std::vector<std::vector<vec3>> Pts(nBodies);
        float BindRestGap = 0;
        for (int b = 0; b < nB; ++b)
        {
            const int Id = G.m_BoneBody[b]; if (Id < 0) continue;
            const int Owner = G.m_Bodies[Id].m_Bone;
            for (const auto& p : Assigned.m_Points[b])
            {
                const vec3 L = BoneLocal(Owner, p);
                Pts[Id].push_back(L);
                BindRestGap = std::max(BindRestGap, (S.m_World[Owner].Apply(L) - p).Length());
            }
        }
        if (BindRestGap > 0.01f) Warnings.push_back(std::format("the skin's bind pose differs from the skeleton's rest pose by up to {:.3f} m: the shapes are fitted in the bind pose", BindRestGap));

        // shapes
        R.m_Slot.assign(nBodies, 0); R.m_Shapes.assign(nBodies, {});
        std::vector<int> ChildBodies(nBodies, 0), OnlyChild(nBodies, -1);
        for (const auto& J : G.m_Joints) { ++ChildBodies[J.m_BodyA]; OnlyChild[J.m_BodyA] = J.m_BodyB; }
        std::vector<float> Volumes(nBodies, 0);
        for (int i = 0; i < nBodies; ++i)
        {
            auto& Bd = G.m_Bodies[i]; const int b = Bd.m_Bone; const auto Slot = SlotOf[b]; R.m_Slot[i] = int(Slot);
            auto& Sh = R.m_Shapes[i];
            const auto* O = FindOverride(b);
            int Kind = Slot == bone_slot::HEAD ? 1 : (Slot == bone_slot::HAND || Slot == bone_slot::FOOT) ? 2 : 0;
            if (O && O->m_Shape != xragdoll_desc::shape_choice::AUTO) Kind = int(O->m_Shape) - 1;
            Sh.m_Kind = std::uint8_t(Kind);

            xcollision::fit_settings Fs; Fs.m_Q = D.m_FitCoverage;
            const bool bTorso = Slot == bone_slot::HIPS || Slot == bone_slot::SPINE;
            if (!bTorso && ChildBodies[i] == 1)
                Fs.m_AxisHint = S.m_World[b].Inverse().Apply(S.m_World[G.m_Bodies[OnlyChild[i]].m_Bone].m_P);
            const auto& P = Pts[i];
            if (P.size() < 16)
            {
                Det.m_Warnings.push_back(std::format("{}: only {} skin samples: a small default sphere", Nm(b), P.size()));
                Sh.m_Kind = 1; Sh.m_Radius = 0.03f;
            }
            else if (Kind == 0)
            {
                const auto C = xcollision::FitCapsule(P, Fs);
                const vec3 Mid = (C.m_A + C.m_B) * 0.5f, Seg = C.m_B - C.m_A;
                Sh.m_Local = details::ToFile({ Mid, Seg.LengthSq() > 1e-12f ? details::FromTo(vec3(0, 1, 0), Seg) : quat::fromIdentity() });
                Sh.m_Radius = C.m_Radius; Sh.m_HalfSegment = 0.5f * Seg.Length();
            }
            else if (Kind == 1)
            {
                const auto Sp = xcollision::FitSphere(P, Fs);
                Sh.m_Local = details::ToFile({ Sp.m_Centre, quat::fromIdentity() }); Sh.m_Radius = Sp.m_Radius;
            }
            else
            {
                const auto Bx = xcollision::FitBox(P, Fs);
                Sh.m_Local = details::ToFile({ Bx.m_Centre, details::FromAxes(Bx.m_Axis[0], Bx.m_Axis[1]) });
                Sh.m_HalfExtents[0] = Bx.m_Half.m_X; Sh.m_HalfExtents[1] = Bx.m_Half.m_Y; Sh.m_HalfExtents[2] = Bx.m_Half.m_Z;
            }
            Volumes[i] = details::Volume(Sh);

            // diagnostics
            xragdoll_desc::details::body_row Row{ .m_Bone = Nm(b), .m_Slot = details::SlotName(Slot), .m_Shape = details::Describe(Sh)
                                                , .m_Origin = O ? "override" : "generated" };
            for (int c = 0; c < nB; ++c) if (G.m_BoneBody[c] == i) Row.m_Share += Assigned.m_Share[c];
            if (P.size() >= 16)
            {
                const auto Pl = details::Place(Sh, rigid{});
                xcollision::fit_report F;
                if (Pl.m_Kind == 0) F = xcollision::Measure(Pl.m_C, P); else if (Pl.m_Kind == 1) F = xcollision::Measure(Pl.m_S, P); else F = xcollision::Measure(Pl.m_B, P);
                Row.m_Coverage = F.m_Coverage; Row.m_OutsideP95 = F.m_OutsideP95; Row.m_ExcessP95 = F.m_ExcessP95;
            }
            Det.m_Bodies.push_back(std::move(Row));
        }

        // mass. A humanoid (hips, head and feet found): the body segments' share of the total from anthropometry (de Leva 1996, the mean
        // of its male and female tables), the total from the height by isometric scaling (65 kg at 1.70 m: a body k times taller weighs k^3
        // more) unless TotalMass is given. Shape volumes would weigh clothes, hair and boots as flesh (KindsLilly: 140 kg). Otherwise:
        // density x the shapes' volume, or TotalMass shared by volume.
        std::vector<float> Mass(nBodies, 0.0f);
        {
            float VolSum = 0; for (float v : Volumes) VolSum += v;
            auto Has = [&](bone_slot s) { for (int i = 0; i < nBodies; ++i) if (SlotOf[G.m_Bodies[i].m_Bone] == s) return true; return false; };
            const bool bHumanoid = Has(bone_slot::HIPS) && Has(bone_slot::HEAD) && Has(bone_slot::FOOT);
            float Height = 0;
            if (bHumanoid)
            {   // feet soles to the top of the head, along the up direction: the skin's extent
                int Head = -1; vec3 Feet(0, 0, 0); int nFeet = 0;
                for (int i = 0; i < nBodies; ++i) { const int b = G.m_Bodies[i].m_Bone; if (SlotOf[b] == bone_slot::HEAD) Head = b; if (SlotOf[b] == bone_slot::FOOT) { Feet = Feet + S.m_World[b].m_P; ++nFeet; } }
                const vec3 Up = (S.m_World[Head].m_P - Feet * (1.0f / float(nFeet))).NormalizeSafeCopy();
                float Lo = 1e30f, Hi = -1e30f; for (const auto& s : Samples) { const float t = s.m_P.Dot(Up); Lo = std::min(Lo, t); Hi = std::max(Hi, t); }
                Height = Hi - Lo;
            }
            if (bHumanoid && Height > 0)
            {
                // % of the whole body (de Leva 1996, male/female mean): hips = the lower trunk, the spine bodies share the middle + upper trunk
                // by volume, neck + head 6.8 (the neck a sixth), and per side upper arm, forearm, hand, thigh, shank, foot
                auto Fraction = [](bone_slot s) -> float
                {
                    switch (s)
                    {
                    case bone_slot::HIPS: return 11.7f;  case bone_slot::SPINE: return 31.2f;  case bone_slot::NECK: return 1.1f;  case bone_slot::HEAD: return 5.7f;
                    case bone_slot::UPPERARM: return 2.63f; case bone_slot::LOWERARM: return 1.5f; case bone_slot::HAND: return 0.585f;
                    case bone_slot::THIGH: return 14.47f; case bone_slot::CALF: return 4.57f; case bone_slot::FOOT: return 1.33f;
                    default: return 0.0f;
                    }
                };
                const float Total = D.m_TotalMass > 0 ? D.m_TotalMass : 65.0f * std::pow(Height / 1.70f, 3.0f);
                // each slot's fraction goes to the bodies that pose its bones (a dropped neck's share goes to the chest), split by volume
                for (const auto& [Key, Bones] : Slots.m_Bones)
                {
                    const float F = Fraction(Key.first); if (F == 0) continue;
                    std::vector<int> Targets; float V = 0;
                    const std::vector<int> Use = Key.first == bone_slot::SPINE ? Bones : std::vector<int>{ Bones.front() };
                    for (int b : Use) if (const int Id = G.m_BoneBody[b]; Id >= 0 && std::find(Targets.begin(), Targets.end(), Id) == Targets.end()) { Targets.push_back(Id); V += Volumes[Id]; }
                    for (int Id : Targets) Mass[Id] += Total * F / 100.0f * (V > 0 ? Volumes[Id] / V : 1.0f / float(Targets.size()));
                }
                float Sum = 0; for (float m : Mass) Sum += m;
                for (int i = 0; i < nBodies; ++i) if (Mass[i] == 0) Det.m_Warnings.push_back(std::format("{}: no anthropometric share (not a humanoid slot): it gets 0.5 % of the mass", Nm(G.m_Bodies[i].m_Bone)));
                for (auto& m : Mass) if (m == 0) m = 0.005f * Total;
                Sum = 0; for (float m : Mass) Sum += m;
                for (auto& m : Mass) m *= Total / Sum;
                Det.m_Report.push_back(std::format("mass: humanoid, {:.1f} kg {} (height {:.3f} m), shared by anthropometric segment fractions (de Leva 1996)", Total, D.m_TotalMass > 0 ? "(TotalMass)" : "from the height (65 kg at 1.70 m, isometric)", Height));
            }
            else
            {
                const float Scale = D.m_TotalMass > 0 && VolSum > 0 ? D.m_TotalMass / (D.m_Density * VolSum) : 1.0f;
                for (int i = 0; i < nBodies; ++i) Mass[i] = D.m_Density * Volumes[i] * Scale;
                Det.m_Report.push_back(std::format("mass: not a humanoid, density {:.0f} kg/m^3 x the shapes' volume{}", D.m_Density, D.m_TotalMass > 0 ? ", scaled to TotalMass" : ""));
            }
        }
        for (int i = 0; i < nBodies; ++i)
        {
            auto& Bd = G.m_Bodies[i]; auto& Sh = R.m_Shapes[i];
            Bd.m_Mass = Mass[i];
            const rigid L = details::FromFile(Sh.m_Local);
            Bd.m_Com = L.m_P; Bd.m_InertiaFrame = L.m_Q;
            float I[3]; details::Inertia(Sh, Bd.m_Mass, I); Bd.m_Inertia = vec3(I[0], I[1], I[2]);
            Bd.m_Shapes.push_back({ .m_Kind = shape_kind(Sh.m_Kind), .m_Local = L, .m_Radius = Sh.m_Radius, .m_HalfSegment = Sh.m_HalfSegment
                                  , .m_HalfExtents = vec3(Sh.m_HalfExtents[0], Sh.m_HalfExtents[1], Sh.m_HalfExtents[2]) });
            Det.m_Bodies[i].m_Mass = Bd.m_Mass;
            Det.m_TotalMass += Bd.m_Mass;
        }
        // connected-body mass ratios past 10:1 make a stiff chain jitter
        for (const auto& J : G.m_Joints)
        {
            const float a = G.m_Bodies[J.m_BodyA].m_Mass, b = G.m_Bodies[J.m_BodyB].m_Mass;
            if (std::max(a, b) > 10 * std::min(a, b)) Det.m_Warnings.push_back(std::format("mass ratio {:.1f}:1 between {} and {}", std::max(a, b) / std::max(1e-6f, std::min(a, b)), Nm(G.m_Bodies[J.m_BodyA].m_Bone), Nm(G.m_Bodies[J.m_BodyB].m_Bone)));
        }

        // joints: the adapter's measured agreement with the skeleton's limit
        for (const auto& J : G.m_Joints)
        {
            static const char* SolverName[] = { "none", "weld", "revolute", "spherical" };
            xragdoll_desc::details::joint_row Row{ .m_Bone = Nm(J.m_Bone), .m_Parent = Nm(G.m_Bodies[J.m_BodyA].m_Bone)
                                                 , .m_Solver = SolverName[int(J.m_Adapter.m_Type)], .m_Mapping = MappingName(J.m_Adapter.m_Mapping), .m_Note = J.m_Adapter.m_Note };
            if (!S.m_Limits.empty() && S.m_Limits[J.m_Bone].m_Type != joint_type::FREE && S.m_Limits[J.m_Bone].m_Type != joint_type::FIXED)
            {
                const auto M = MeasureAdapter(S.m_Limits[J.m_Bone], J.m_Adapter);
                Row.m_FalseRejectPct = M.m_FalseRejectPct; Row.m_FalseAcceptPct = M.m_FalseAcceptPct; Row.m_MaxExcessDeg = M.m_MaxExcessDeg;
            }
            else if (S.m_Limits.empty() || S.m_Limits[J.m_Bone].m_Type == joint_type::FREE) Row.m_Note = Row.m_Note.empty() ? "free: no limit" : Row.m_Note;
            // the rest pose (the joint rotation is the identity there) inside the solver joint: a ragdoll activated at rest must not snap
            if (!SolverAllows(J.m_Adapter, quat::fromIdentity())) Det.m_Warnings.push_back(std::format("{}: the rest pose is outside the solver joint (it would snap when activated)", Nm(J.m_Bone)));
            Det.m_Joints.push_back(std::move(Row));
        }

        // filter: pairs overlapping at rest (jointed pairs never collide anyway); self-collision off: every pair
        {
            std::vector<details::placed> Placed;
            for (int i = 0; i < nBodies; ++i) Placed.push_back(details::Place(R.m_Shapes[i], S.m_World[G.m_Bodies[i].m_Bone] * G.m_Bodies[i].m_BoneToBody));
            auto Jointed = [&](int a, int b) { for (const auto& J : G.m_Joints) if ((J.m_BodyA == a && J.m_BodyB == b) || (J.m_BodyA == b && J.m_BodyB == a)) return true; return false; };
            int nOverlap = 0;
            for (int a = 0; a < nBodies; ++a) for (int b = a + 1; b < nBodies; ++b)
            {
                if (Jointed(a, b)) continue;
                if (!D.m_bSelfCollision) { R.m_Filter.push_back({ std::uint16_t(a), std::uint16_t(b) }); continue; }
                if (const float Depth = details::Overlap(Placed[a], Placed[b]); Depth > 0.002f)
                {
                    R.m_Filter.push_back({ std::uint16_t(a), std::uint16_t(b) }); ++nOverlap;
                    Det.m_Report.push_back(std::format("pair excluded (overlaps at rest by {:.3f} m): {} - {}", Depth, Nm(G.m_Bodies[a].m_Bone), Nm(G.m_Bodies[b].m_Bone)));
                }
            }
            if (!D.m_bSelfCollision) Det.m_Report.push_back("self-collision off: bodies never collide with each other");
        }

        // the movement capsule: the core region (no arms), feet to the top of the head, along the up direction at rest
        {
            vec3 Up(0, 1, 0);
            int Head = -1; vec3 Feet(0, 0, 0); int nFeet = 0;
            for (int i = 0; i < nBodies; ++i)
            {
                const int b = G.m_Bodies[i].m_Bone;
                if (SlotOf[b] == bone_slot::HEAD) Head = b;
                if (SlotOf[b] == bone_slot::FOOT) { Feet = Feet + S.m_World[b].m_P; ++nFeet; }
            }
            if (Head >= 0 && nFeet) { const vec3 U = S.m_World[Head].m_P - Feet * (1.0f / float(nFeet)); if (U.LengthSq() > 1e-6f) Up = U.NormalizeSafeCopy(); }
            else Det.m_Warnings.push_back("no head or no feet: the capsule stands along +y");
            std::vector<vec3> Core;
            for (int b = 0; b < nB; ++b)
            {
                const int Id = G.m_BoneBody[b]; if (Id < 0) continue;
                const auto Sl = SlotOf[G.m_Bodies[Id].m_Bone];
                if (Sl == bone_slot::UPPERARM || Sl == bone_slot::LOWERARM || Sl == bone_slot::HAND) continue;
                Core.insert(Core.end(), Assigned.m_Points[b].begin(), Assigned.m_Points[b].end());
            }
            xcollision::fit_settings Fs; Fs.m_Q = 0.9f; Fs.m_LengthQ = 1.0f; Fs.m_CapInset = 1.0f; Fs.m_AxisHint = Up;
            const auto C = xcollision::FitCapsule(Core, Fs);
            const vec3 Mid = (C.m_A + C.m_B) * 0.5f;
            R.m_Capsule.m_Local = details::ToFile({ Mid, details::FromTo(vec3(0, 1, 0), Up) });
            R.m_Capsule.m_Radius = C.m_Radius; R.m_Capsule.m_HalfSegment = 0.5f * (C.m_B - C.m_A).Length();
            Det.m_Height = 2 * (R.m_Capsule.m_HalfSegment + R.m_Capsule.m_Radius);
            Det.m_Capsule = std::format("radius {:.3f}, height {:.3f}, centre ({:.3f}, {:.3f}, {:.3f})", C.m_Radius, Det.m_Height, Mid.m_X, Mid.m_Y, Mid.m_Z);
        }

        for (auto d : Disp) { Det.m_nBodies += d == bone_disposition::BODY; Det.m_nFollows += d == bone_disposition::FOLLOWS; Det.m_nAnimated += d == bone_disposition::ANIMATED; Det.m_nExcluded += d == bone_disposition::EXCLUDED; }
        Det.m_nFilterPairs = int(R.m_Filter.size());
        Det.m_Report.insert(Det.m_Report.end(), G.m_Report.begin(), G.m_Report.end());
        Det.m_Warnings.insert(Det.m_Warnings.end(), Warnings.begin(), Warnings.end());
        Det.m_bOk = true;
        return R;
    }

    // The compiled resource from a result (the caller owns it: Kill)
    inline void ToResource(const result& R, xragdoll::ragdoll& Out)
    {
        Out.Initialize();
        const auto& G = R.m_Graph;
        Out.m_nBones = std::uint32_t(G.m_Disposition.size());
        Out.m_pDisposition = new std::uint8_t[Out.m_nBones]; Out.m_pBoneBody = new std::int16_t[Out.m_nBones];
        for (std::uint32_t i = 0; i < Out.m_nBones; ++i) { Out.m_pDisposition[i] = std::uint8_t(G.m_Disposition[i]); Out.m_pBoneBody[i] = std::int16_t(G.m_BoneBody[i]); }
        Out.m_nBodies = Out.m_nShapes = std::uint32_t(G.m_Bodies.size());
        Out.m_pBodies = new xragdoll::body[Out.m_nBodies]; Out.m_pShapes = new xragdoll::shape[Out.m_nShapes];
        for (std::uint32_t i = 0; i < Out.m_nBodies; ++i)
        {
            const auto& B = G.m_Bodies[i]; auto& O = Out.m_pBodies[i];
            O = {}; O.m_Bone = B.m_Bone; O.m_BoneToBody = details::ToFile(B.m_BoneToBody); O.m_iShape = i; O.m_nShapes = 1; O.m_Mass = B.m_Mass;
            O.m_Com[0] = B.m_Com.m_X; O.m_Com[1] = B.m_Com.m_Y; O.m_Com[2] = B.m_Com.m_Z;
            O.m_Inertia[0] = B.m_Inertia.m_X; O.m_Inertia[1] = B.m_Inertia.m_Y; O.m_Inertia[2] = B.m_Inertia.m_Z;
            O.m_InertiaFrame[0] = B.m_InertiaFrame.m_X; O.m_InertiaFrame[1] = B.m_InertiaFrame.m_Y; O.m_InertiaFrame[2] = B.m_InertiaFrame.m_Z; O.m_InertiaFrame[3] = B.m_InertiaFrame.m_W;
            Out.m_pShapes[i] = R.m_Shapes[i];
            Out.m_TotalMass += B.m_Mass;
        }
        Out.m_nJoints = std::uint32_t(G.m_Joints.size());
        Out.m_pJoints = new xragdoll::joint[Out.m_nJoints];
        for (std::uint32_t i = 0; i < Out.m_nJoints; ++i)
        {
            const auto& J = G.m_Joints[i]; const auto& A = J.m_Adapter; auto& O = Out.m_pJoints[i];
            O = {}; O.m_BodyA = J.m_BodyA; O.m_BodyB = J.m_BodyB; O.m_Bone = J.m_Bone; O.m_FrameA = details::ToFile(J.m_FrameA); O.m_FrameB = details::ToFile(J.m_FrameB);
            O.m_Type = std::uint8_t(A.m_Type); O.m_Mapping = std::uint8_t(A.m_Mapping);
            O.m_Flags = std::uint8_t((A.m_bSwing ? xragdoll::joint::flag_swing_v : 0) | (A.m_bTwist ? xragdoll::joint::flag_twist_v : 0) | (A.m_bHinge ? xragdoll::joint::flag_hinge_v : 0));
            O.m_SwingCx = A.m_SwingCx; O.m_SwingCy = A.m_SwingCy; O.m_SwingRx = A.m_SwingRx; O.m_SwingRy = A.m_SwingRy;
            O.m_TwistLo = A.m_TwistLo; O.m_TwistHi = A.m_TwistHi; O.m_HingeLo = A.m_HingeLo; O.m_HingeHi = A.m_HingeHi;
        }
        Out.m_nFilter = std::uint32_t(R.m_Filter.size());
        Out.m_pFilter = new xragdoll::filter_pair[Out.m_nFilter];
        std::copy(R.m_Filter.begin(), R.m_Filter.end(), Out.m_pFilter);
        Out.m_Capsule = R.m_Capsule; Out.m_WorldScale = R.m_WorldScale; Out.m_SkeletonSignature = R.m_Signature;
    }
}

#endif

#ifndef XRAGDOLL_BODY_GRAPH_H
#define XRAGDOLL_BODY_GRAPH_H
#pragma once

// The compiled Ragdoll back to the body model the runtime simulates (xanimation::body_graph: ragdoll design v2 section 8, one path for
// the editor's Drop and Play): the inverse of xragdoll_generate.h's ToResource. The file keeps the SOLVER frames; the spring's target (the
// rest rotation seen through the solver frames, OffA^-1 OffB) is not stored: it is the solver frames' relative rotation at the rest pose,
// so it is measured here from the skeleton's rest (RestModel: rigid model-space bone frames, metres). The joint anchors must meet at that
// rest: the worst gap is returned (a rest that differs from the one the ragdoll was compiled with shows up there).
#include "dependencies/xAnimation/source/physical/xanimation_body_model.h"
#include "xragdoll.h"
#include <algorithm>
#include <span>
#include <utility>
#include <vector>

namespace xragdoll
{
    inline xanimation::rigid FromFile(const rigid& R) noexcept
    {
        return { xmath::fvec3(R.m_P[0], R.m_P[1], R.m_P[2]), xmath::fquat(R.m_Q[0], R.m_Q[1], R.m_Q[2], R.m_Q[3]) };
    }

    struct runtime_graph
    {
        xanimation::body_graph              m_Graph;
        std::vector<std::pair<int, int>>    m_Filter;
        float                               m_RestAnchorGap = 0;    // m: the worst joint anchor gap at the rest pose (should be ~0)
    };

    inline runtime_graph ToBodyGraph(const ragdoll& R, std::span<const xanimation::rigid> RestModel)
    {
        using namespace xanimation;
        runtime_graph Out; auto& G = Out.m_Graph;
        G.m_Disposition.resize(R.m_nBones); G.m_BoneBody.resize(R.m_nBones);
        for (std::uint32_t i = 0; i < R.m_nBones; ++i) { G.m_Disposition[i] = bone_disposition(R.m_pDisposition[i]); G.m_BoneBody[i] = R.m_pBoneBody[i]; }
        G.m_Bodies.resize(R.m_nBodies);
        for (std::uint32_t i = 0; i < R.m_nBodies; ++i)
        {
            const auto& F = R.m_pBodies[i]; auto& B = G.m_Bodies[i];
            B.m_Bone = F.m_Bone; B.m_BoneToBody = FromFile(F.m_BoneToBody); B.m_Mass = F.m_Mass;
            B.m_Com = xmath::fvec3(F.m_Com[0], F.m_Com[1], F.m_Com[2]);
            B.m_Inertia = xmath::fvec3(F.m_Inertia[0], F.m_Inertia[1], F.m_Inertia[2]);
            B.m_InertiaFrame = xmath::fquat(F.m_InertiaFrame[0], F.m_InertiaFrame[1], F.m_InertiaFrame[2], F.m_InertiaFrame[3]);
            for (std::uint32_t s = F.m_iShape; s < F.m_iShape + F.m_nShapes && s < R.m_nShapes; ++s)
            {
                const auto& S = R.m_pShapes[s];
                B.m_Shapes.push_back({ shape_kind(S.m_Kind), FromFile(S.m_Local), S.m_Radius, S.m_HalfSegment, xmath::fvec3(S.m_HalfExtents[0], S.m_HalfExtents[1], S.m_HalfExtents[2]) });
            }
        }
        auto BodyRest = [&](int b) { const auto& B = G.m_Bodies[std::size_t(b)]; return RestModel[std::size_t(B.m_Bone)] * B.m_BoneToBody; };
        G.m_Joints.resize(R.m_nJoints);
        for (std::uint32_t i = 0; i < R.m_nJoints; ++i)
        {
            const auto& F = R.m_pJoints[i]; auto& J = G.m_Joints[i];
            J.m_BodyA = F.m_BodyA; J.m_BodyB = F.m_BodyB; J.m_Bone = F.m_Bone; J.m_FrameA = FromFile(F.m_FrameA); J.m_FrameB = FromFile(F.m_FrameB);
            auto& A = J.m_Adapter;
            A.m_Type = solver_joint_type(F.m_Type); A.m_Mapping = mapping_kind(F.m_Mapping);
            A.m_bCone = (F.m_Flags & joint::flag_cone_v) != 0; A.m_bTwist = (F.m_Flags & joint::flag_twist_v) != 0; A.m_bHinge = (F.m_Flags & joint::flag_hinge_v) != 0;
            A.m_Cone = F.m_Cone; A.m_TwistLo = F.m_TwistLo; A.m_TwistHi = F.m_TwistHi; A.m_HingeLo = F.m_HingeLo; A.m_HingeHi = F.m_HingeHi;
            if (std::size_t(std::max(F.m_BodyA, F.m_BodyB)) >= G.m_Bodies.size() || F.m_BodyA < 0 || F.m_BodyB < 0) { G.m_bOk = false; continue; }
            const auto SA = BodyRest(F.m_BodyA) * J.m_FrameA, SB = BodyRest(F.m_BodyB) * J.m_FrameB;
            A.m_OffA = xmath::fquat::fromIdentity();
            A.m_OffB = (SA.m_Q.ConjugateCopy() * SB.m_Q).NormalizeSafeCopy();      // the spring's target: OffA^-1 OffB
            Out.m_RestAnchorGap = std::max(Out.m_RestAnchorGap, (SA.m_P - SB.m_P).Length());
        }
        for (std::uint32_t i = 0; i < R.m_nFilter; ++i) Out.m_Filter.push_back({ int(R.m_pFilter[i].m_A), int(R.m_pFilter[i].m_B) });
        return Out;
    }
}

#endif

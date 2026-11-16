#ifndef XRAGDOLL_RUNTIME_H
#define XRAGDOLL_RUNTIME_H
#pragma once

// The compiled Ragdoll resource: the body model of one skeleton + skin (ragdoll design v2). Flat arrays, the same content as
// xanimation::body_graph (physical/xanimation_body_model.h), which the runtime builds from it (one path for the editor's Drop and Play).
// Frames: a rigid is P (metres) + Q (x, y, z, w); a body's frame is its bone's frame * BoneToBody; shapes, COM, inertia and joint frames
// are in the body's frame. A bone's frame here is RIGID (the skeleton's uniform scale folded into positions: metres).
#include "dependencies/xserializer/source/xserializer.h"
#include <cstdint>
#include <cstring>

namespace xragdoll
{
    struct rigid
    {
        float   m_P[3] = { 0, 0, 0 };
        float   m_Q[4] = { 0, 0, 0, 1 };
    };

    struct shape
    {
        std::uint8_t    m_Kind          = 0;            // xanimation::shape_kind: 0 capsule (segment along local y), 1 sphere, 2 box
        rigid           m_Local;                        // in the body's frame
        float           m_Radius        = 0;            // capsule, sphere
        float           m_HalfSegment   = 0;            // capsule
        float           m_HalfExtents[3]= { 0, 0, 0 };  // box
    };

    struct body
    {
        std::int32_t    m_Bone          = -1;
        rigid           m_BoneToBody;
        std::uint32_t   m_iShape        = 0, m_nShapes = 0;
        float           m_Mass          = 0;            // kg
        float           m_Com[3]        = { 0, 0, 0 };  // body frame
        float           m_Inertia[3]    = { 0, 0, 0 };  // principal moments about the COM (kg m^2)
        float           m_InertiaFrame[4] = { 0, 0, 0, 1 };
    };

    struct joint
    {
        std::int32_t    m_BodyA = -1, m_BodyB = -1;     // parent body, child body
        std::int32_t    m_Bone  = -1;                   // the skeleton bone whose joint (to its parent) this is
        rigid           m_FrameA, m_FrameB;             // the SOLVER frames in each body's frame
        std::uint8_t    m_Type      = 0;                // xanimation::solver_joint_type: 0 none, 1 weld, 2 revolute, 3 spherical
        std::uint8_t    m_Mapping   = 0;                // xanimation::mapping_kind: 0 exact, 1 approximation, 2 unsupported
        std::uint8_t    m_Flags     = 0;                // flag_swing_v | flag_twist_v | flag_hinge_v
        float           m_SwingCx = 0, m_SwingCy = 0, m_SwingRx = 0, m_SwingRy = 0;    // the swing ellipse in solver frame A (radians; R54)
        float           m_TwistLo = 0, m_TwistHi = 0, m_HingeLo = 0, m_HingeHi = 0;     // radians
        float           m_Relaxed[4] = { 0, 0, 0, 1 };  // the skeleton's relaxed joint rotation in solver frame A (C64; identity: the rest)
        float           m_Tone = 1;                     // the passive spring's stiffness scale (C64)

        static constexpr std::uint8_t flag_swing_v = 1, flag_twist_v = 2, flag_hinge_v = 4;
    };

    // A pair of bodies that never collide (besides jointed pairs, which never do)
    struct filter_pair { std::uint16_t m_A = 0, m_B = 0; };

    // The movement capsule's defaults (no mass, never collides with its own bodies): in the character's model frame
    struct capsule
    {
        rigid           m_Local;                        // its segment along local y
        float           m_Radius = 0, m_HalfSegment = 0;
    };

    struct ragdoll
    {
        inline static constexpr auto xserializer_version_v = 3;        // 2: a spherical joint's swing is our ellipse (was a cone); 3: joints' relaxed rotation + tone (C64)

        inline          ragdoll     (void) noexcept = default;
        inline          ragdoll     (xserializer::stream&) noexcept {}
        inline void     Initialize  (void) noexcept { std::memset(this, 0, sizeof(*this)); }
        inline void     Kill        (void) noexcept
        {
            delete[] m_pDisposition; delete[] m_pBoneBody; delete[] m_pBodies; delete[] m_pShapes; delete[] m_pJoints; delete[] m_pFilter;
            Initialize();
        }

        std::uint8_t*   m_pDisposition  {nullptr};      // m_nBones: xanimation::bone_disposition (0 body, 1 follows, 2 animated, 3 excluded)
        std::int16_t*   m_pBoneBody     {nullptr};      // m_nBones: the body a bone is posed from (-1 none)
        body*           m_pBodies       {nullptr};      // bone order: a parent body before its children; body 0 is the root (the hips)
        shape*          m_pShapes       {nullptr};
        joint*          m_pJoints       {nullptr};      // one per body but the root
        filter_pair*    m_pFilter       {nullptr};
        std::uint32_t   m_nBones = 0, m_nBodies = 0, m_nShapes = 0, m_nJoints = 0, m_nFilter = 0;
        capsule         m_Capsule;
        float           m_TotalMass = 0;                // kg
        float           m_WorldScale = 1;               // the skeleton's uniform scale at the root (its frames here are rigid: metres)
        std::uint64_t   m_SkeletonSignature = 0;        // hash of the skeleton's bone names and parents when this was compiled
    };
}

namespace xserializer::io_functions
{
    template<> inline xerr SerializeIO<xragdoll::rigid>(xserializer::stream& S, const xragdoll::rigid& R) noexcept
    {
        xerr Err; false || (Err = S.Serialize(R.m_P)) || (Err = S.Serialize(R.m_Q)); return Err;
    }
    template<> inline xerr SerializeIO<xragdoll::shape>(xserializer::stream& S, const xragdoll::shape& V) noexcept
    {
        xerr Err;
        false || (Err = S.Serialize(V.m_Kind)) || (Err = SerializeIO(S, V.m_Local)) || (Err = S.Serialize(V.m_Radius)) || (Err = S.Serialize(V.m_HalfSegment)) || (Err = S.Serialize(V.m_HalfExtents));
        return Err;
    }
    template<> inline xerr SerializeIO<xragdoll::body>(xserializer::stream& S, const xragdoll::body& V) noexcept
    {
        xerr Err;
        false || (Err = S.Serialize(V.m_Bone)) || (Err = SerializeIO(S, V.m_BoneToBody)) || (Err = S.Serialize(V.m_iShape)) || (Err = S.Serialize(V.m_nShapes))
              || (Err = S.Serialize(V.m_Mass)) || (Err = S.Serialize(V.m_Com)) || (Err = S.Serialize(V.m_Inertia)) || (Err = S.Serialize(V.m_InertiaFrame));
        return Err;
    }
    template<> inline xerr SerializeIO<xragdoll::joint>(xserializer::stream& S, const xragdoll::joint& V) noexcept
    {
        xerr Err;
        false || (Err = S.Serialize(V.m_BodyA)) || (Err = S.Serialize(V.m_BodyB)) || (Err = S.Serialize(V.m_Bone)) || (Err = SerializeIO(S, V.m_FrameA)) || (Err = SerializeIO(S, V.m_FrameB))
              || (Err = S.Serialize(V.m_Type)) || (Err = S.Serialize(V.m_Mapping)) || (Err = S.Serialize(V.m_Flags))
              || (Err = S.Serialize(V.m_SwingCx)) || (Err = S.Serialize(V.m_SwingCy)) || (Err = S.Serialize(V.m_SwingRx)) || (Err = S.Serialize(V.m_SwingRy))
              || (Err = S.Serialize(V.m_TwistLo)) || (Err = S.Serialize(V.m_TwistHi)) || (Err = S.Serialize(V.m_HingeLo)) || (Err = S.Serialize(V.m_HingeHi))
              || (Err = S.Serialize(V.m_Relaxed[0])) || (Err = S.Serialize(V.m_Relaxed[1])) || (Err = S.Serialize(V.m_Relaxed[2])) || (Err = S.Serialize(V.m_Relaxed[3])) || (Err = S.Serialize(V.m_Tone));
        return Err;
    }
    template<> inline xerr SerializeIO<xragdoll::filter_pair>(xserializer::stream& S, const xragdoll::filter_pair& V) noexcept
    {
        xerr Err; false || (Err = S.Serialize(V.m_A)) || (Err = S.Serialize(V.m_B)); return Err;
    }
    template<> inline xerr SerializeIO<xragdoll::ragdoll>(xserializer::stream& S, const xragdoll::ragdoll& R) noexcept
    {
        xerr Err;
        false
            || (Err = S.Serialize(R.m_nBones))   || (Err = S.Serialize(R.m_pDisposition, R.m_nBones)) || (Err = S.Serialize(R.m_pBoneBody, R.m_nBones))
            || (Err = S.Serialize(R.m_nBodies))  || (Err = S.Serialize(R.m_pBodies, R.m_nBodies))
            || (Err = S.Serialize(R.m_nShapes))  || (Err = S.Serialize(R.m_pShapes, R.m_nShapes))
            || (Err = S.Serialize(R.m_nJoints))  || (Err = S.Serialize(R.m_pJoints, R.m_nJoints))
            || (Err = S.Serialize(R.m_nFilter))  || (Err = S.Serialize(R.m_pFilter, R.m_nFilter))
            || (Err = SerializeIO(S, R.m_Capsule.m_Local)) || (Err = S.Serialize(R.m_Capsule.m_Radius)) || (Err = S.Serialize(R.m_Capsule.m_HalfSegment))
            || (Err = S.Serialize(R.m_TotalMass)) || (Err = S.Serialize(R.m_WorldScale)) || (Err = S.Serialize(R.m_SkeletonSignature))
            ;
        return Err;
    }
}

#endif

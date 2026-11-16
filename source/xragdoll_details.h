#ifndef XRAGDOLL_DETAILS_H
#define XRAGDOLL_DETAILS_H
#pragma once

// The capability report (ragdoll design v2 sections 0 and 10): what the compiler made and every approximation it took, for the editor
// and for review. Written to the resource's log folder as Details.txt each time the resource compiles.
#include <string>
#include <vector>

namespace xragdoll_desc
{
    struct details
    {
        struct body_row
        {
            std::string  m_Bone;
            std::string  m_Slot;                // hips, spine, thigh ... ("" : not a known slot)
            std::string  m_Shape;               // capsule r 0.061 len 0.342 ...
            std::string  m_Origin;              // generated, or the user's override
            float        m_Mass        = 0;     // kg
            float        m_Share       = 0;     // of the skin's surface
            float        m_Coverage    = 0;     // of its samples inside the shape
            float        m_OutsideP95  = 0;     // m
            float        m_ExcessP95   = 0;     // m: how far the shape bulges past its skin

            XPROPERTY_DEF
            ( "body_row", body_row
            , obj_member<"Bone",       &body_row::m_Bone,       member_flags<flags::SHOW_READONLY> >
            , obj_member<"Slot",       &body_row::m_Slot,       member_flags<flags::SHOW_READONLY> >
            , obj_member<"Shape",      &body_row::m_Shape,      member_flags<flags::SHOW_READONLY> >
            , obj_member<"Origin",     &body_row::m_Origin,     member_flags<flags::SHOW_READONLY> >
            , obj_member<"Mass",       &body_row::m_Mass,       member_flags<flags::SHOW_READONLY> >
            , obj_member<"Share",      &body_row::m_Share,      member_flags<flags::SHOW_READONLY> >
            , obj_member<"Coverage",   &body_row::m_Coverage,   member_flags<flags::SHOW_READONLY> >
            , obj_member<"OutsideP95", &body_row::m_OutsideP95, member_flags<flags::SHOW_READONLY> >
            , obj_member<"ExcessP95",  &body_row::m_ExcessP95,  member_flags<flags::SHOW_READONLY> >
            )
        };

        struct joint_row
        {
            std::string  m_Bone;                // the child body's bone
            std::string  m_Parent;              // the parent body's bone
            std::string  m_Solver;              // weld, revolute, spherical
            std::string  m_Mapping;             // exact, approximation, unsupported
            std::string  m_Note;
            float        m_FalseRejectPct = 0;  // of the rotations the skeleton allows, the % the solver rejects (stiffer)
            float        m_FalseAcceptPct = 0;  // of the rotations the solver allows, the % outside the skeleton (bends past it)
            float        m_MaxExcessDeg   = 0;

            XPROPERTY_DEF
            ( "joint_row", joint_row
            , obj_member<"Bone",           &joint_row::m_Bone,           member_flags<flags::SHOW_READONLY> >
            , obj_member<"Parent",         &joint_row::m_Parent,         member_flags<flags::SHOW_READONLY> >
            , obj_member<"Solver",         &joint_row::m_Solver,         member_flags<flags::SHOW_READONLY> >
            , obj_member<"Mapping",        &joint_row::m_Mapping,        member_flags<flags::SHOW_READONLY> >
            , obj_member<"Note",           &joint_row::m_Note,           member_flags<flags::SHOW_READONLY> >
            , obj_member<"FalseRejectPct", &joint_row::m_FalseRejectPct, member_flags<flags::SHOW_READONLY> >
            , obj_member<"FalseAcceptPct", &joint_row::m_FalseAcceptPct, member_flags<flags::SHOW_READONLY> >
            , obj_member<"MaxExcessDeg",   &joint_row::m_MaxExcessDeg,   member_flags<flags::SHOW_READONLY> >
            )
        };

        std::vector<body_row>     m_Bodies;
        std::vector<joint_row>    m_Joints;
        std::vector<std::string>  m_Report;         // removed articulations, approximations, excluded pairs, errors (each line says which)
        std::vector<std::string>  m_Warnings;
        float                     m_TotalMass   = 0;
        float                     m_Height      = 0;    // m: feet to the top of the head at rest
        std::string               m_Capsule;            // the movement capsule
        int                       m_nBodies = 0, m_nFollows = 0, m_nAnimated = 0, m_nExcluded = 0, m_nFilterPairs = 0;
        bool                      m_bOk = false;

        XPROPERTY_DEF
        ( "details", details
        , obj_member<"Bodies",      &details::m_Bodies,      member_flags<flags::SHOW_READONLY> >
        , obj_member<"Joints",      &details::m_Joints,      member_flags<flags::SHOW_READONLY> >
        , obj_member<"Report",      &details::m_Report,      member_flags<flags::SHOW_READONLY> >
        , obj_member<"Warnings",    &details::m_Warnings,    member_flags<flags::SHOW_READONLY> >
        , obj_member<"TotalMass",   &details::m_TotalMass,   member_flags<flags::SHOW_READONLY> >
        , obj_member<"Height",      &details::m_Height,      member_flags<flags::SHOW_READONLY> >
        , obj_member<"Capsule",     &details::m_Capsule,     member_flags<flags::SHOW_READONLY> >
        , obj_member<"nBodies",     &details::m_nBodies,     member_flags<flags::SHOW_READONLY> >
        , obj_member<"nFollows",    &details::m_nFollows,    member_flags<flags::SHOW_READONLY> >
        , obj_member<"nAnimated",   &details::m_nAnimated,   member_flags<flags::SHOW_READONLY> >
        , obj_member<"nExcluded",   &details::m_nExcluded,   member_flags<flags::SHOW_READONLY> >
        , obj_member<"nFilterPairs",&details::m_nFilterPairs,member_flags<flags::SHOW_READONLY> >
        , obj_member<"Ok",          &details::m_bOk,         member_flags<flags::SHOW_READONLY> >
        )
    };
    XPROPERTY_REG(details)
    XPROPERTY_REG2(ragdoll_body_row_, details::body_row)
    XPROPERTY_REG2(ragdoll_joint_row_, details::joint_row)
}

#endif

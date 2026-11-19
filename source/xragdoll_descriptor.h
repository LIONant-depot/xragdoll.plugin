#ifndef XRAGDOLL_DESCRIPTOR_H
#define XRAGDOLL_DESCRIPTOR_H
#pragma once

// What the user edits and saves for a Ragdoll (ragdoll design v2 section 2). Compile inputs: the skin it is fitted to (its compiled LOD 0:
// positions in the bind pose and the skeleton's bone indices), the skin's skeleton (its bone manifest: names, rest pose, joint limits) and
// this descriptor. Everything not named here is generated; the user's corrections are the overrides, kept by bone NAME (they survive a
// skin or skeleton being compiled again).
#include "plugins/xskeleton.plugin/source/xskeleton_xgpu_rsc_loader.h"
#include "plugins/xgeom_skin.plugin/source/xgeom_skin_xgpu_rsc_loader.h"
#include "xragdoll_ref.h"
#include <array>
#include <string>
#include <vector>

namespace xragdoll_desc
{
    inline static constexpr auto    resource_type_guid_v    = xrsc::ragdoll_type_guid_v;

    // A bone's disposition (xanimation::bone_disposition) with Auto = the generator decides
    enum class disposition : std::uint8_t { AUTO, BODY, FOLLOWS, ANIMATED, EXCLUDED };
    static constexpr auto disposition_v = std::array
    { xproperty::settings::enum_item("Auto",     disposition::AUTO)
    , xproperty::settings::enum_item("Body",     disposition::BODY)
    , xproperty::settings::enum_item("Follows",  disposition::FOLLOWS)
    , xproperty::settings::enum_item("Animated", disposition::ANIMATED)
    , xproperty::settings::enum_item("Excluded", disposition::EXCLUDED)
    };

    enum class shape_choice : std::uint8_t { AUTO, CAPSULE, SPHERE, BOX };
    static constexpr auto shape_choice_v = std::array
    { xproperty::settings::enum_item("Auto",    shape_choice::AUTO)
    , xproperty::settings::enum_item("Capsule", shape_choice::CAPSULE)
    , xproperty::settings::enum_item("Sphere",  shape_choice::SPHERE)
    , xproperty::settings::enum_item("Box",     shape_choice::BOX)
    };

    // A correction of the generated body model for one bone, by bone NAME
    struct bone_override
    {
        std::string     m_Bone          = {};
        disposition     m_Disposition   = disposition::AUTO;
        shape_choice    m_Shape         = shape_choice::AUTO;
        // the shape drawn by hand in the editor (its handles), in the body's frame, instead of the one fitted to the skin
        bool            m_bShapeSet     = false;
        xmath::fvec3    m_ShapeCentre   = xmath::fvec3(0, 0, 0);
        xmath::fquat    m_ShapeRotation = xmath::fquat::fromIdentity();   // a capsule's segment along its local y
        float           m_ShapeRadius   = 0;                              // capsule, sphere
        float           m_ShapeHalfSegment = 0;                           // capsule
        xmath::fvec3    m_ShapeHalfExtents = xmath::fvec3(0, 0, 0);       // box

        XPROPERTY_DEF
        ( "BoneOverride", bone_override
        , obj_member<"Bone",        &bone_override::m_Bone >
        , obj_member<"Disposition", &bone_override::m_Disposition, member_enum_span<disposition_v>,  member_help<"Body: owns a rigid body. Follows: rigid with the body above it (a movable joint here is a removed articulation, listed in the report). Animated: keeps its animation relative to the body above it. Excluded: not driven by the simulation"> >
        , obj_member<"Shape",       &bone_override::m_Shape,       member_enum_span<shape_choice_v>, member_help<"The body's collision shape (a body bone only)"> >
        , obj_member<"ShapeSet",    &bone_override::m_bShapeSet,   member_help<"The shape below was set by hand (the viewport's handles) instead of fitted to the skin"> >
        , obj_member<"ShapeCentre", &bone_override::m_ShapeCentre >
        , obj_member<"ShapeRotation", &bone_override::m_ShapeRotation >
        , obj_member<"ShapeRadius", &bone_override::m_ShapeRadius >
        , obj_member<"ShapeHalfSegment", &bone_override::m_ShapeHalfSegment >
        , obj_member<"ShapeHalfExtents", &bone_override::m_ShapeHalfExtents >
        )
    };
    XPROPERTY_REG(bone_override)

    struct descriptor : xresource_pipeline::descriptor::base
    {
        using parent = xresource_pipeline::descriptor::base;

        void SetupFromSource(std::string_view FileName) override {}

        void Validate(std::vector<std::string>& Errors) const noexcept override
        {
            if (m_Skin.empty())         Errors.push_back("The skin reference is NULL: a ragdoll is fitted to a skin");
            if (m_TotalMass < 0)        Errors.push_back("The total mass is negative");
            if (m_Density <= 0)         Errors.push_back("The density must be positive");
            if (m_FitCoverage <= 0.1f || m_FitCoverage > 1) Errors.push_back("The fit coverage must be in (0.1, 1]");
        }

        xrsc::geom_skin                 m_Skin              = {};       // the skin the bodies are fitted to
        xrsc::skeleton                  m_Skeleton          = {};       // empty: the skin's own skeleton
        float                           m_TotalMass         = 0;        // kg; 0: density x the bodies' volume
        float                           m_Density           = 985;      // kg/m^3 (a human body floats just under 1000)
        float                           m_FitCoverage       = 0.9f;     // the share of a body's skin its shape reaches (slimmer bodies fall better)
        float                           m_MinBodyShare      = 0.004f;   // a candidate body with less of the skin than this follows the body above it
        bool                            m_bFingersFollow    = true;
        bool                            m_bToesFollow       = true;
        bool                            m_bSelfCollision    = true;     // limbs collide with the torso and each other (jointed pairs and pairs overlapping at rest never)
        bool                            m_bDetailed         = false;    // E66: a toe body per foot (hinged at the ball) and a clavicle body per side
        std::vector<bone_override>      m_Overrides         = {};

        XPROPERTY_VDEF
        ( "Ragdoll", descriptor
        , obj_member<"Skin",            &descriptor::m_Skin,            member_help<"The skin the bodies are fitted to (its LOD 0 in the bind pose)"> >
        , obj_member<"Skeleton",        &descriptor::m_Skeleton,        member_help<"Empty: the skin's own skeleton. It is the only authority for the joint frames and limits"> >
        , obj_member<"TotalMass",       &descriptor::m_TotalMass,       member_help<"kg. 0: a humanoid weighs 65 kg at 1.70 m scaled by the cube of its height (4x taller: 64x heavier); another creature, the density times its bodies' volume. A humanoid's bodies share it by anthropometric segment fractions, another creature's by volume"> >
        , obj_member<"Density",         &descriptor::m_Density,         member_help<"kg/m^3: the mass of a creature that is not a humanoid, when TotalMass is 0"> >
        , obj_member<"FitCoverage",     &descriptor::m_FitCoverage,     member_help<"The share of each body's skin its shape reaches (0.9: a little slimmer than the skin, which falls better)"> >
        , obj_member<"MinBodyShare",    &descriptor::m_MinBodyShare,    member_help<"A candidate body (a neck, a spine bone) with less than this share of the skin follows the body above it"> >
        , obj_member<"FingersFollow",   &descriptor::m_bFingersFollow,  member_help<"Fingers are rigid with the hand (their joints are listed as removed articulations)"> >
        , obj_member<"ToesFollow",      &descriptor::m_bToesFollow,     member_help<"Toes are rigid with the foot"> >
        , obj_member<"SelfCollision",   &descriptor::m_bSelfCollision,  member_help<"Bodies collide with each other, except jointed pairs and pairs that overlap at rest"> >
        , obj_member<"Detailed",        &descriptor::m_bDetailed,       member_help<"Detailed humanoid: each foot is a heel body and a toe body hinged at the ball (the feet roll over the ball), and each shoulder has a clavicle body between the chest and the upper arm"> >
        , obj_member<"Overrides",       &descriptor::m_Overrides >
        )
    };
    XPROPERTY_VREG(descriptor)

    //--------------------------------------------------------------------------------------

    struct factory final : xresource_pipeline::factory_base
    {
        using xresource_pipeline::factory_base::factory_base;

        std::unique_ptr<xresource_pipeline::descriptor::base> CreateDescriptor(void) const noexcept override { return std::make_unique<descriptor>(); }
        xresource::type_guid        ResourceTypeGUID(void) const noexcept override { return resource_type_guid_v; }
        const char*                 ResourceTypeName(void) const noexcept override { return "Ragdoll"; }
        const xproperty::type::object& ResourceXPropertyObject(void) const noexcept override { return *xproperty::getObjectByType<descriptor>(); }
    };

    inline static factory g_Factory{};
}

#endif

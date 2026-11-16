# xragdoll.plugin

The **Ragdoll** resource: the physical body of a character. It is fitted to a skin and takes its joints from the skin's skeleton:

- **Bodies**: one rigid body per main bone (hips, spine, head, upper and lower arms, hands, thighs, calves, feet), each with one collision
  shape (capsule, sphere or box) fitted to the skin around it. Other bones follow a body (fingers, toes, clavicles) or keep their animation
  (helpers, hair). The compile report lists every joint the ragdoll does not move.
- **Joints**: from the skeleton's joint limits (turn them on in the Skeleton editor). Each one says whether the physics joint is exact or an
  approximation, and how far it differs from the skeleton's limit.
- **Mass**: in real units, so weight feels right at any size. A humanoid weighs 65 kg at 1.70 m, scaled by the cube of its height, shared
  between the bodies like a human body; set `TotalMass` for a heavier or lighter character. Other creatures: density times volume.
- **Collision filter**: jointed bodies never collide; bodies that overlap at rest are excluded from each other.
- **Movement capsule**: the defaults for a character controller's capsule.

Corrections are kept by bone name in `Overrides` (a bone's disposition and shape), so they survive the skin or the skeleton being compiled
again.

- `source/xragdoll_descriptor.h`: what the user edits and saves.
- `source/xragdoll.h`: the compiled resource.
- `source/xragdoll_generate.h`: the generator (skeleton + skin + descriptor -> the body model and its report).
- `source/compiler/`: the compiler. The report is written to the resource's log folder as `Details.txt`.

The body model and the physics adapter live in the [xAnimation](https://github.com/LIONant-depot/xAnimation) library, the shape fitting
in [xcollision](https://github.com/LIONant-depot/xcollision).

## License
MIT.

# Coordinate and unit contract

The public Godot API uses metres, seconds, kilograms, radians, newtons and
newton-metres. MuJoCo generalized-force entries are labelled `N_or_Nm`
because translational degrees of freedom use N and rotational degrees of
freedom use N·m.

Godot and XR Human use a right-handed `+X right, +Y up, -Z forward` frame.
MuJoCo is right-handed and conventionally Z-up. The bridge applies the proper
rotation (determinant +1):

```
MuJoCo (x, y, z) -> Godot (x, z, -y)
Godot  (x, y, z) -> MuJoCo (x, -z, y)
```

Orientations are converted as matrices (`C R C^-1`), rather than by relabeling
quaternion components. MuJoCo stores quaternions scalar-first (`w x y z`);
Godot exposes them scalar-last in its constructor (`x y z w`). No quaternion
storage order crosses the GDScript API in this milestone.

`get_joint_frame()` reports the joint anchor in metres and the axis as a unit
vector, both in Godot coordinates. Body and site transforms are world/rig-from-
local transforms. A future exporter must fail on unsupported or ambiguous
shape/joint conventions; silently reflecting or guessing an axis is forbidden.

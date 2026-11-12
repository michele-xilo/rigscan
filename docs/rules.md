# Rules

Every finding carries a rule ID. IDs are stable: they are never reused or renumbered. A finding is **critical**
(the robot is broken or will behave wrongly) or a **warning** (probably a mistake, but the file can load).
`rigscan scan --statistics` counts findings per rule.

| Rule | Severity | Formats | Flags |
|---|---|---|---|
| [`RS001`](#rs001) | critical | URDF, MJCF | a joint names a link that is not defined |
| [`RS002`](#rs002) | critical | URDF, MJCF | two links with the same name |
| [`RS003`](#rs003) | critical | URDF, MJCF | two joints with the same name |
| [`RS004`](#rs004) | critical | URDF, MJCF | a link with more than one parent joint |
| [`RS005`](#rs005) | critical | URDF, MJCF | a kinematic loop |
| [`RS006`](#rs006) | critical | URDF, MJCF | more than one root link |
| [`RS007`](#rs007) | critical | URDF, MJCF | a robot with no links |
| [`RS008`](#rs008) | critical | all | negative link mass |
| [`RS009`](#rs009) | warning | all | zero mass on a link a joint moves |
| [`RS010`](#rs010) | critical | all | negative principal moment of inertia |
| [`RS011`](#rs011) | warning | all | principal moments that break the triangle inequality |
| [`RS012`](#rs012) | critical | all | joint `lower` limit above `upper` |
| [`RS013`](#rs013) | critical | URDF | a revolute or prismatic joint with no limit, or a limit missing `effort` or `velocity` |
| [`RS014`](#rs014) | critical | URDF, MJCF | a mesh file that does not exist |
| [`RS015`](#rs015) | critical | URDF | a `<mimic>` of a joint that does not exist, or of itself |
| [`RS016`](#rs016) | critical | MJCF | a `class` or `childclass` that names no default class |
| [`RS017`](#rs017) | critical | MJCF | a mesh geom naming an asset that is not defined |
| [`RS018`](#rs018) | critical | MJCF | an `<include>` file that does not exist |
| [`RS019`](#rs019) | critical | USD | a joint that connects a body to itself |
| [`RS020`](#rs020) | warning | USD | a rigid body with no mass and no collision shape |
| [`RS021`](#rs021) | warning | USD | joints but no articulation root |
| [`RS022`](#rs022) | critical | USD | a reference, payload or sublayer that cannot be found |

`RS004`-`RS007` do not run on USD: UsdPhysics joins bodies in a graph, so closed chains and free bodies are valid.
A file that cannot be parsed at all (a bad number, truncated XML) is reported as an error with its line and column,
and `rigscan scan` exits 2.

## RS001

A joint's `parent` or `child` names a link that does not exist, or has none. Simulators either refuse the file
or silently drop the joint.

```xml
<joint name="elbow" type="revolute"><parent link="upper_arm"/><child link="forarm"/></joint>
<!-- forarm is not a link: the child was misspelled -->
```

Fix: use the name of a defined link, or define the missing one.

## RS002

Two links share a name, so a joint cannot say which one it means. Rename one.

## RS003

Two joints share a name, so a controller or a `<mimic>` cannot address one of them. Rename one.

## RS004

A link is the child of two joints. A rigid body has one pose, so a tree needs exactly one parent per link. Break
the closed chain with a different mechanism (MuJoCo `equality`, a URDF loop-closure convention).

## RS005

Following parent to child comes back to where it started. A URDF must be a tree.

```xml
<joint name="j1" ...><parent link="a"/><child link="b"/></joint>
<joint name="j2" ...><parent link="b"/><child link="a"/></joint>
```

## RS006

More than one link has no parent, so the file describes separate robots (or a link was left unconnected). Join them
with a joint, or split the file.

## RS007

The robot has no links. Usually the wrong file or a truncated one.

## RS008

A link's mass is negative. No physics engine can use it. Fix the sign, or the unit conversion that produced it.

## RS009

A link that a joint moves has zero mass: it has no inertia, so the dynamics are singular or divided by zero. A
massless frame on a fixed joint is fine and is not reported.

```xml
<link name="forearm"><inertial><mass value="0"/>...</inertial></link>
```

Fix: give it the mass of the part, or merge it into its parent with a fixed joint.

## RS010

A principal moment of inertia (an eigenvalue of the inertia tensor) is negative, which no real body has.

## RS011

The principal moments must satisfy `a + b >= c` for every pair. A body with `ixx = iyy = 0.1`, `izz = 0.5`
cannot exist: its inertia was probably typed in the wrong order or in the wrong units. The file loads, so this is a
warning.

## RS012

`lower` is above `upper`, so the joint has no allowed range. Swap them, or check the unit (URDF radians, MJCF
degrees by default, USD degrees for revolute joints).

## RS013

A revolute or prismatic joint has no `<limit>`, or the limit has no `effort` or `velocity`. The URDF schema
requires both, and tools such as Pinocchio refuse the file without them. A `continuous` joint needs none.

```xml
<joint name="j" type="revolute"><parent link="a"/><child link="b"/><limit lower="-1" upper="1"/></joint>
<!-- add effort="..." velocity="..." -->
```

## RS014

A mesh file is not where the description says. The CLI resolves relative paths from the file's directory, and
`package://name/...` from the nearest parent directory called `name`; a package it cannot locate is not reported.

## RS015

`<mimic joint="...">` names a joint that does not exist, or the joint it sits in.

## RS016

A `class` or `childclass` names a default class that is not defined, so the settings it was meant to apply are
silently skipped.

```xml
<body childclass="missing"><geom type="sphere" size="0.1"/></body>
```

## RS017

A geom has `mesh="ghost"` but no `<mesh name="ghost">` asset exists.

## RS018

An `<include file="...">` points at a file that is not there, so part of the model is missing.

## RS019

A joint's `body0` and `body1` are the same body (or both are the world). It constrains nothing.

## RS020

A rigid body has no mass (`PhysicsMassAPI`) and no collision shape, so the engine cannot compute a mass for it.
It loads, with a default mass, so this is a warning.

## RS021

The stage has joints but no `PhysicsArticulationRootAPI`, so the joints are solved as loose constraints rather
than as one articulation. Many importers need the root.

## RS022

A `references`, `payloads` or `subLayers` path could not be found, so part of the stage is missing.

```usda
def Xform "robot" (references = @gone.usda@) {}
```

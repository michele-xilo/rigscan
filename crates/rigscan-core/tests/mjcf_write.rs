use rigscan_core::{
    parse_mjcf, parse_urdf, write_mjcf, write_urdf, Format, Geometry, IncludeLoader, Included,
    JointKind, Robot, Role, Shape, Span, Unmapped,
};
use std::cell::RefCell;
use std::collections::HashSet;
use std::path::{Path, PathBuf};

struct NoFiles;

impl IncludeLoader for NoFiles {
    fn load(&self, _: &str, _: Option<&str>) -> Option<Included> {
        None
    }
}

fn from_mjcf(text: &str) -> Robot {
    parse_mjcf(text, &NoFiles).unwrap().to_robot().unwrap()
}

/// What `lost` names, for one robot: the pairs of `what` and element name.
fn reported(lost: &[Unmapped], what: &str) -> HashSet<String> {
    lost.iter()
        .filter(|u| u.what == what)
        .filter_map(|u| u.name.clone())
        .collect()
}

/// The robot as it must come back after a conversion and the way back, given everything the two
/// conversions reported as lost. Each rule removes exactly what a loss entry says was dropped, so
/// a difference that was not reported still fails the comparison. Besides that:
/// - source format, spans and the loss list are not part of the comparison;
/// - a shape for both roles is split, as URDF has to write it twice;
/// - MJCF nests bodies and has no name for a weld, so links and joints are sorted and the name of a
///   fixed joint is not compared;
/// - MJCF always has a world to weld a root link to, so a `world` link the source did not have
///   and its welds are dropped.
fn essence(mut r: Robot, lost: &[Unmapped], had_world: bool) -> Robot {
    let (mimic, effort, velocity, limit) = (
        reported(lost, "mimic"),
        reported(lost, "limit/effort"),
        reported(lost, "limit/velocity"),
        reported(lost, "limit"),
    );
    let capsule = reported(lost, "capsule");
    let dynamics = reported(lost, "dynamics");
    let kinds: HashSet<String> = lost
        .iter()
        .filter(|u| u.what.starts_with("joint/"))
        .filter_map(|u| u.name.clone())
        .collect();
    r.format = Format::Mjcf;
    r.span = Span::NONE;
    r.unmapped.clear();
    for l in &mut r.links {
        l.span = Span::NONE;
        if let Some(i) = &mut l.inertial {
            i.span = Span::NONE;
        }
        let mut shapes = Vec::new();
        for mut s in l.shapes.drain(..) {
            s.span = Span::NONE;
            if capsule.contains(&l.name) && matches!(s.geometry, Geometry::Capsule { .. }) {
                continue;
            }
            if s.role == Role::Both {
                shapes.push(Shape {
                    role: Role::Visual,
                    ..s.clone()
                });
                s.role = Role::Collision;
            }
            shapes.push(s);
        }
        l.shapes = shapes;
    }
    for j in &mut r.joints {
        j.span = Span::NONE;
        if mimic.contains(&j.name) {
            j.mimic = None;
        }
        if let Some(m) = &mut j.mimic {
            m.span = Span::NONE;
        }
        if let Some(l) = &mut j.limit {
            if effort.contains(&j.name) {
                l.effort = None;
            }
            if velocity.contains(&j.name) {
                l.velocity = None;
            }
        }
        if limit.contains(&j.name) {
            j.limit = None;
            if j.kind == JointKind::Revolute {
                j.kind = JointKind::Continuous;
            }
        }
        if dynamics.contains(&j.name) {
            j.dynamics = None;
        }
        if kinds.contains(&j.name) {
            j.kind = JointKind::Fixed;
            j.axis = None;
            j.limit = None;
            j.dynamics = None;
        }
        if j.limit.as_ref().is_some_and(|l| {
            l.lower.is_none() && l.upper.is_none() && l.effort.is_none() && l.velocity.is_none()
        }) {
            j.limit = None;
        }
        if j.dynamics
            .as_ref()
            .is_some_and(|d| d.damping.is_none() && d.friction.is_none())
        {
            j.dynamics = None;
        }
        if j.kind == JointKind::Fixed {
            j.name.clear();
        }
    }
    if !had_world {
        r.links.retain(|l| l.name != "world");
        r.joints
            .retain(|j| !(j.kind == JointKind::Fixed && j.parent == "world"));
    }
    r.links.sort_by(|a, b| a.name.cmp(&b.name));
    r.joints
        .sort_by(|a, b| (&a.child, &a.parent, &a.name).cmp(&(&b.child, &b.parent, &b.name)));
    r
}

/// A -> B -> A: `a` written as MJCF and read back must be the same robot, apart from what the
/// conversion reported as lost.
fn assert_urdf_round_trip(label: &str, a: &Robot) {
    let first = write_mjcf(a);
    let b = from_mjcf(&first.text);
    let second = write_urdf(&b);
    let back =
        parse_urdf(&second.text).unwrap_or_else(|e| panic!("{label}: {}\n{}", e.0, second.text));
    let lost: Vec<_> = first.lost.iter().chain(&second.lost).cloned().collect();
    let had_world = a.links.iter().any(|l| l.name == "world");
    assert_eq!(
        essence(a.clone(), &lost, had_world),
        essence(back, &lost, had_world),
        "{label}\n{}",
        first.text
    );
    // The text is a fixed point: converting what was written gives the same text again.
    assert_eq!(
        write_mjcf(&b).text,
        first.text,
        "{label}: not a fixed point"
    );
}

/// The same from the MJCF side: URDF in the middle, MJCF at both ends.
fn assert_mjcf_round_trip(label: &str, a: &Robot) {
    let first = write_urdf(a);
    let b = parse_urdf(&first.text).unwrap_or_else(|e| panic!("{label}: {}\n{}", e.0, first.text));
    let second = write_mjcf(&b);
    let back = parse_mjcf(&second.text, &NoFiles)
        .unwrap_or_else(|e| panic!("{label}: {}\n{}", e.0, second.text))
        .to_robot()
        .unwrap();
    let lost: Vec<_> = first.lost.iter().chain(&second.lost).cloned().collect();
    let had_world = a.links.iter().any(|l| l.name == "world");
    assert_eq!(
        essence(a.clone(), &lost, had_world),
        essence(back, &lost, had_world),
        "{label}\n{}",
        second.text
    );
    assert_eq!(
        write_urdf(&b).text,
        first.text,
        "{label}: not a fixed point"
    );
}

const URDF: &str = r#"<?xml version="1.0"?>
<robot name="r">
  <link name="base"/>
  <link name="arm">
    <inertial>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <mass value="2"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="2" iyz="0" izz="3"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="1 2 3"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0.1" rpy="0 0.5 0"/>
      <geometry>
        <mesh filename="package://r/arm.stl" scale="2 2 2"/>
      </geometry>
    </collision>
  </link>
  <link name="hand"/>
  <joint name="j" type="revolute">
    <parent link="base"/>
    <child link="arm"/>
    <origin xyz="0 0 1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1"/>
    <dynamics damping="0.1" friction="0.2"/>
  </joint>
  <joint name="k" type="prismatic">
    <parent link="arm"/>
    <child link="hand"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
  </joint>
</robot>
"#;

const MJCF: &str = r#"<mujoco model="r">
  <compiler angle="radian" eulerseq="XYZ"/>
  <asset>
    <mesh name="mesh0" file="package://r/arm.stl" scale="2 2 2"/>
  </asset>
  <worldbody>
    <body name="base">
      <body name="arm" pos="0 0 1">
        <inertial pos="0 0 0.5" mass="2" diaginertia="1 2 3"/>
        <joint name="j" type="hinge" axis="0 0 1" range="-1 1" damping="0.1" frictionloss="0.2"/>
        <geom type="box" size="0.5 1 1.5" contype="0" conaffinity="0"/>
        <geom type="mesh" mesh="mesh0" pos="0 0 0.1" euler="0 0.5 0" group="3"/>
        <body name="hand" pos="0 0 0.5">
          <joint name="k" type="slide" axis="1 0 0"/>
        </body>
      </body>
    </body>
  </worldbody>
</mujoco>
"#;

#[test]
fn the_written_mjcf_has_one_fixed_layout() {
    let robot = parse_urdf(URDF).unwrap();
    assert_eq!(write_mjcf(&robot).text, MJCF);
}

#[test]
fn urdf_to_mjcf_and_back_is_the_same_robot() {
    assert_urdf_round_trip("pinned", &parse_urdf(URDF).unwrap());
}

#[test]
fn mjcf_to_urdf_and_back_is_the_same_robot() {
    let a = from_mjcf(
        r#"<mujoco model="m"><compiler angle="radian"/>
  <asset><mesh name="p" file="p.stl" scale="1 2 3"/></asset>
  <worldbody>
    <geom type="plane" size="1 1 1"/>
    <geom type="sphere" size="0.1" pos="1 0 0"/>
    <body name="a" pos="0 0 1" euler="0 0.5 0">
      <freejoint name="root"/>
      <inertial pos="0 0 0.1" mass="2" fullinertia="1 2 3 0.1 0.2 0.3"/>
      <geom type="box" size="1 1 1"/>
      <geom type="capsule" size="0.1 0.5" contype="0" conaffinity="0"/>
      <geom type="cylinder" size="0.1 0.5" group="3"/>
      <geom type="mesh" mesh="p"/>
      <body name="b" pos="0 1 0" axisangle="0 0 1 0.3">
        <joint name="j" type="hinge" axis="0 1 0" range="-1 1" damping="0.3"/>
        <joint name="k" type="slide" axis="1 0 0" range="0 1"/>
        <body name="c" pos="1 0 0"><joint name="m" type="hinge"/></body>
      </body>
    </body>
  </worldbody>
</mujoco>"#,
    );
    assert_mjcf_round_trip("mjcf", &a);
}

#[test]
fn what_mjcf_cannot_hold_is_reported() {
    let robot = parse_urdf(
        r#"<robot name="r">
  <link name="a"/><link name="b"/><link name="c"/><link name="d"/>
  <joint name="j" type="revolute"><parent link="a"/><child link="b"/><axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="5" velocity="6"/></joint>
  <joint name="weld" type="fixed"><parent link="b"/><child link="c"/></joint>
  <joint name="m" type="prismatic"><parent link="c"/><child link="d"/><limit lower="0" upper="1"/><mimic joint="j"/></joint>
</robot>"#,
    )
    .unwrap();
    let mut lost: Vec<_> = write_mjcf(&robot)
        .lost
        .iter()
        .map(|u| (u.what.clone(), u.name.clone().unwrap()))
        .collect();
    lost.sort();
    assert_eq!(
        lost,
        [
            ("joint/name".to_string(), "weld".to_string()),
            ("limit/effort".to_string(), "j".to_string()),
            ("limit/velocity".to_string(), "j".to_string()),
            ("mimic".to_string(), "m".to_string()),
        ]
    );
}

#[test]
fn a_joint_mjcf_cannot_hold_becomes_a_weld_and_is_reported() {
    let robot = parse_urdf(
        r#"<robot name="r"><link name="a"/><link name="b"/>
  <joint name="p" type="planar"><parent link="a"/><child link="b"/></joint></robot>"#,
    )
    .unwrap();
    let out = write_mjcf(&robot);
    assert!(!out.text.contains("<joint"), "{}", out.text);
    assert!(out.lost.iter().any(|u| u.what == "joint/planar"));
}

#[test]
fn an_inertial_with_only_a_mass_round_trips() {
    let a = parse_urdf(
        "<robot name=\"r\"><link name=\"a\"><inertial><mass value=\"1\"/></inertial></link></robot>",
    )
    .unwrap();
    assert_urdf_round_trip("mass only", &a);
}

fn lost_of(urdf: &str) -> Vec<(String, String)> {
    let mut lost: Vec<_> = write_mjcf(&parse_urdf(urdf).unwrap())
        .lost
        .iter()
        .map(|u| (u.what.clone(), u.name.clone().unwrap_or_default()))
        .collect();
    lost.sort();
    lost
}

fn lost(entries: &[(&str, &str)]) -> Vec<(String, String)> {
    entries
        .iter()
        .map(|(w, n)| (w.to_string(), n.to_string()))
        .collect()
}

#[test]
fn what_a_tree_cannot_hold_is_reported() {
    // MJCF nests each body in one parent: a second parent, an undefined link and a cycle
    // cannot be written.
    assert_eq!(
        lost_of(
            r#"<robot name="r"><link name="a"/><link name="b"/><link name="c"/><link name="x"/><link name="y"/>
  <joint name="j1" type="fixed"><parent link="a"/><child link="c"/></joint>
  <joint name="j2" type="fixed"><parent link="b"/><child link="c"/></joint>
  <joint name="j3" type="fixed"><parent link="a"/><child link="ghost"/></joint>
  <joint name="j4" type="fixed"><parent link="x"/><child link="y"/></joint>
  <joint name="j5" type="fixed"><parent link="y"/><child link="x"/></joint>
</robot>"#
        ),
        lost(&[
            ("joint/name", "j1"),
            ("joint/tree", "j2"),
            ("joint/undefined", "j3"),
            ("link", "x"),
            ("link", "y"),
        ])
    );
}

#[test]
fn a_link_named_twice_is_written_once_and_an_empty_root_is_one_element() {
    let robot =
        parse_urdf("<robot name=\"r\"><link name=\"a\"/><link name=\"a\"/></robot>").unwrap();
    assert_eq!(write_mjcf(&robot).text.matches("<body").count(), 1);
    assert!(write_mjcf(&robot).text.contains("<body name=\"a\"/>"));
}

#[test]
fn a_joint_with_no_place_for_its_values_is_reported() {
    assert_eq!(
        lost_of(
            r#"<robot name="r"><link name="a"/><link name="b"/><link name="c"/><link name="d"/>
  <joint name="j1" type="fixed"><parent link="a"/><child link="b"/><limit lower="0" upper="1"/><dynamics damping="1"/></joint>
  <joint name="j2" type="floating"><parent link="b"/><child link="c"/></joint>
  <joint name="j3" type="ball"><parent link="c"/><child link="d"/></joint>
</robot>"#
        ),
        lost(&[
            ("dynamics", "j1"),
            ("joint/ball", "j3"),
            ("joint/floating", "j2"),
            ("joint/name", "j1"),
            ("limit", "j1"),
        ])
    );
}

#[test]
fn a_bound_mjcf_cannot_keep_is_reported() {
    assert_eq!(
        lost_of(
            r#"<robot name="r"><link name="a"/><link name="b"/><link name="c"/><link name="d"/><link name="e"/><link name="f"/>
  <joint name="r1" type="revolute"><parent link="a"/><child link="b"/></joint>
  <joint name="r2" type="revolute"><parent link="b"/><child link="c"/><limit lower="0"/></joint>
  <joint name="c1" type="continuous"><parent link="c"/><child link="d"/><limit lower="0" upper="1"/></joint>
  <joint name="p1" type="prismatic"><parent link="d"/><child link="e"/><limit upper="1"/></joint>
  <joint name="p2" type="prismatic"><parent link="e"/><child link="f"/></joint>
</robot>"#
        ),
        lost(&[
            ("limit", "c1"),
            ("limit", "p1"),
            ("limit", "r1"),
            ("limit", "r2"),
        ])
    );
}

#[test]
fn joints_to_one_body_at_different_poses_are_reported() {
    assert_eq!(
        lost_of(
            r#"<robot name="r"><link name="a"/><link name="b"/>
  <joint name="j1" type="revolute"><parent link="a"/><child link="b"/><origin xyz="0 0 1"/><limit lower="0" upper="1"/></joint>
  <joint name="j2" type="revolute"><parent link="a"/><child link="b"/><origin xyz="0 0 2"/><limit lower="0" upper="1"/></joint>
</robot>"#
        ),
        lost(&[("joint/origin", "j2")])
    );
}

#[test]
fn a_capsule_and_a_shape_for_both_roles_are_written_plain() {
    let robot = from_mjcf(
        "<mujoco><worldbody><body name=\"a\"><geom type=\"capsule\" size=\"0.1 0.5\"/></body></worldbody></mujoco>",
    );
    assert!(write_mjcf(&robot)
        .text
        .contains("<geom type=\"capsule\" size=\"0.1 0.5\"/>"));
}

fn urdf_files(dir: &Path, out: &mut Vec<PathBuf>) {
    for entry in std::fs::read_dir(dir).unwrap().flatten() {
        let p = entry.path();
        if p.is_dir() {
            urdf_files(&p, out);
        } else if p.extension().is_some_and(|e| e == "urdf") {
            out.push(p);
        }
    }
}

/// None when the corpus is absent; RIGSCAN_REQUIRE_CORPUS=1 makes that a failure.
fn corpus(name: &str) -> Option<PathBuf> {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../corpus")
        .join(name);
    if dir.exists() {
        return Some(dir);
    }
    assert!(
        std::env::var_os("RIGSCAN_REQUIRE_CORPUS").is_none(),
        "corpus required but missing at {}",
        dir.display()
    );
    eprintln!("SKIPPED: corpus not found at {}", dir.display());
    None
}

#[test]
fn every_corpus_urdf_survives_a_trip_through_mjcf() {
    let Some(dir) = corpus("example-robot-data") else {
        return;
    };
    let mut files = Vec::new();
    urdf_files(&dir, &mut files);
    files.sort();
    for f in &files {
        let a = parse_urdf(&std::fs::read_to_string(f).unwrap()).unwrap();
        assert_urdf_round_trip(&f.display().to_string(), &a);
    }
    eprintln!("{} URDF files round-tripped through MJCF", files.len());
}

struct DiskLoader(PathBuf);

impl IncludeLoader for DiskLoader {
    fn load(&self, file: &str, from: Option<&str>) -> Option<Included> {
        let base = from.map_or(&*self.0, |f| Path::new(f).parent().unwrap());
        let path = base.join(file);
        let text = std::fs::read_to_string(&path).ok()?;
        Some(Included {
            path: path.display().to_string(),
            text,
        })
    }
}

fn mjcf_files(dir: &Path, out: &mut Vec<PathBuf>) {
    for entry in std::fs::read_dir(dir).unwrap().flatten() {
        let p = entry.path();
        if p.is_dir() {
            mjcf_files(&p, out);
        } else if p.extension().is_some_and(|e| e == "xml") {
            out.push(p);
        }
    }
}

#[test]
fn every_corpus_mjcf_survives_a_trip_through_urdf() {
    let Some(dir) = corpus("mujoco_menagerie") else {
        return;
    };
    let mut files = Vec::new();
    mjcf_files(&dir, &mut files);
    files.sort();
    let seen = RefCell::new(0usize);
    for f in &files {
        let text = std::fs::read_to_string(f).unwrap();
        if !text.contains("<mujoco") {
            continue;
        }
        let loader = DiskLoader(f.parent().unwrap().to_path_buf());
        // Files that are only included by another model do not parse on their own.
        let Ok(mjcf) = parse_mjcf(&text, &loader) else {
            continue;
        };
        let Ok(a) = mjcf.to_robot() else { continue };
        if a.links.len() < 2 {
            continue;
        }
        assert_mjcf_round_trip(&f.display().to_string(), &a);
        *seen.borrow_mut() += 1;
    }
    assert!(
        *seen.borrow() >= 50,
        "only {} models round-tripped",
        seen.borrow()
    );
    eprintln!("{} MJCF models round-tripped through URDF", seen.borrow());
}

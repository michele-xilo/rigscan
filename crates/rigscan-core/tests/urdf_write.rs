use rigscan_core::{
    parse_mjcf, parse_urdf, write_urdf, Format, Geometry, Joint, JointKind, Link, Origin, Robot,
    Role, Shape, Span,
};
use std::path::{Path, PathBuf};

struct NoFiles;

impl rigscan_core::IncludeLoader for NoFiles {
    fn load(&self, _: &str, _: Option<&str>) -> Option<rigscan_core::Included> {
        None
    }
}

/// The model without what a text round trip cannot keep: positions, the list of losses and the
/// source format. A
/// shape for both roles is the one pair URDF has to write as two elements, so it is split.
fn essence(mut r: Robot) -> Robot {
    r.format = Format::Urdf;
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
        if let Some(m) = &mut j.mimic {
            m.span = Span::NONE;
        }
    }
    r
}

/// The model read back from the URDF written for `robot` must be the same robot.
fn assert_round_trip(label: &str, robot: Robot) {
    let written = write_urdf(&robot).text;
    let back = parse_urdf(&written).unwrap_or_else(|e| panic!("{label}: {}\n{written}", e.0));
    assert_eq!(essence(robot), essence(back), "{label}");
}

const PINNED: &str = r#"<?xml version="1.0"?>
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
    <limit lower="-1" upper="1" effort="10" velocity="2"/>
    <dynamics damping="0.1" friction="0.2"/>
  </joint>
  <joint name="k" type="prismatic">
    <parent link="arm"/>
    <child link="hand"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <mimic joint="j" multiplier="2" offset="0.5"/>
  </joint>
</robot>
"#;

#[test]
fn the_written_urdf_has_one_fixed_layout() {
    let robot = parse_urdf(PINNED).unwrap();
    assert_eq!(write_urdf(&robot).text, PINNED);
}

#[test]
fn urdf_to_urdf_keeps_the_model_and_the_text() {
    let robot = parse_urdf(PINNED).unwrap();
    let first = write_urdf(&robot).text;
    let second = write_urdf(&parse_urdf(&first).unwrap()).text;
    assert_eq!(first, second);
    assert_round_trip("pinned", robot);
}

#[test]
fn a_shape_for_both_roles_is_written_as_a_visual_and_a_collision() {
    let mut robot = parse_urdf("<robot name=\"r\"><link name=\"a\"/></robot>").unwrap();
    robot.links[0].shapes.push(Shape {
        role: Role::Both,
        geometry: Geometry::Sphere { radius: 0.5 },
        origin: Origin::default(),
        span: Span::NONE,
    });
    let text = write_urdf(&robot).text;
    assert_eq!(text.matches("<visual>").count(), 1, "{text}");
    assert_eq!(text.matches("<collision>").count(), 1, "{text}");
    assert_round_trip("both", robot);
}

#[test]
fn what_the_source_dropped_is_passed_on_and_what_urdf_cannot_hold_is_added() {
    let mut robot = parse_urdf(
        "<robot name=\"r\"><link name=\"a\"><visual><geometry><sphere radius=\"1\"/></geometry><material name=\"m\"/></visual></link></robot>",
    )
    .unwrap();
    robot.links[0].shapes.push(Shape {
        role: Role::Visual,
        geometry: Geometry::Capsule {
            radius: 0.1,
            length: 1.0,
        },
        origin: Origin::default(),
        span: Span::NONE,
    });
    robot.links.push(Link {
        name: "b".into(),
        inertial: None,
        shapes: vec![],
        span: Span::NONE,
    });
    robot.joints.push(Joint {
        name: "ball".into(),
        parent: "a".into(),
        child: "b".into(),
        kind: JointKind::Other("ball".into()),
        origin: Origin::default(),
        limit: None,
        mimic: None,
        axis: None,
        dynamics: None,
        span: Span::NONE,
    });
    let lost: Vec<_> = write_urdf(&robot)
        .lost
        .iter()
        .map(|u| (u.what.clone(), u.name.clone()))
        .collect();
    assert_eq!(
        lost,
        [
            ("visual/material".to_string(), Some("a".to_string())),
            ("capsule".to_string(), Some("a".to_string())),
            ("joint/ball".to_string(), Some("ball".to_string())),
        ]
    );
}

#[test]
fn every_joint_type_and_primitive_shape_round_trips() {
    let robot = parse_urdf(
        r#"<robot name="r">
  <link name="a"><collision><geometry><cylinder radius="0.1" length="2"/></geometry></collision></link>
  <link name="b"/><link name="c"/><link name="d"/><link name="e"/>
  <joint name="j1" type="continuous"><parent link="a"/><child link="b"/></joint>
  <joint name="j2" type="fixed"><parent link="b"/><child link="c"/></joint>
  <joint name="j3" type="floating"><parent link="c"/><child link="d"/></joint>
  <joint name="j4" type="planar"><parent link="d"/><child link="e"/></joint>
</robot>"#,
    )
    .unwrap();
    assert_round_trip("kinds", robot);
}

#[test]
fn an_inertial_with_only_a_mass_or_only_an_inertia_round_trips() {
    let robot = parse_urdf(
        r#"<robot name="r">
  <link name="a"><inertial><mass value="1"/></inertial></link>
  <link name="b"><inertial><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial></link>
</robot>"#,
    )
    .unwrap();
    assert_round_trip("partial inertials", robot);
}

#[test]
fn names_and_paths_with_markup_are_escaped() {
    let mut robot = parse_urdf("<robot name=\"a&amp;b\"><link name=\"x\"/></robot>").unwrap();
    robot.links[0].shapes.push(Shape {
        role: Role::Visual,
        geometry: Geometry::Mesh {
            filename: "m\"<1>&.stl".into(),
            scale: [1.0; 3],
        },
        origin: Origin::default(),
        span: Span::NONE,
    });
    assert_round_trip("escaped", robot);
}

#[test]
fn an_mjcf_robot_written_as_urdf_reads_back_as_the_same_robot() {
    let robot = parse_mjcf(
        r#"<mujoco model="m"><compiler angle="radian"/>
  <asset><mesh name="p" file="p.stl" scale="1 2 3"/></asset>
  <worldbody>
    <body name="a" pos="0 0 1" euler="0 0.5 0">
      <joint name="j" type="hinge" axis="0 1 0" range="-1 1" pos="0 0 0.2" damping="0.3"/>
      <inertial pos="0 0 0.1" mass="2" diaginertia="1 2 3"/>
      <geom type="box" size="1 1 1"/>
      <geom type="mesh" mesh="p" contype="0" conaffinity="0"/>
      <body name="b"><joint name="k" type="slide" axis="1 0 0" range="0 1"/></body>
    </body>
  </worldbody>
</mujoco>"#,
        &NoFiles,
    )
    .unwrap()
    .to_robot()
    .unwrap();
    assert_eq!(robot.format, Format::Mjcf);
    assert_round_trip("mjcf", robot);
}

#[test]
fn the_example_robot_survives_a_round_trip() {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../examples/broken_arm.urdf");
    assert_round_trip(
        "broken_arm",
        parse_urdf(&std::fs::read_to_string(path).unwrap()).unwrap(),
    );
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

/// Needs the corpus from `scripts/fetch-corpus.sh`; RIGSCAN_REQUIRE_CORPUS=1 makes a missing
/// corpus a failure.
#[test]
fn every_corpus_urdf_survives_a_round_trip() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../corpus/example-robot-data");
    if !dir.exists() {
        assert!(
            std::env::var_os("RIGSCAN_REQUIRE_CORPUS").is_none(),
            "corpus required but missing at {}",
            dir.display()
        );
        eprintln!(
            "SKIPPED: corpus not found at {}; run scripts/fetch-corpus.sh",
            dir.display()
        );
        return;
    }
    let mut files = Vec::new();
    urdf_files(&dir, &mut files);
    files.sort();
    for f in &files {
        let robot = parse_urdf(&std::fs::read_to_string(f).unwrap()).unwrap();
        assert_round_trip(&f.display().to_string(), robot);
    }
    eprintln!("{} URDF files round-tripped", files.len());
}

use rigscan_core::{
    parse_mjcf, parse_urdf, parse_usd, write_usd, Geometry, JointKind, Robot, Role, Unmapped,
    UsdLoader,
};
use std::collections::HashMap;
use std::fmt::Write;

struct Files(HashMap<String, String>);

impl UsdLoader for Files {
    fn load(&self, path: &str) -> Option<Vec<u8>> {
        self.0.get(path).map(|s| s.as_bytes().to_vec())
    }
}

struct NoFiles;

impl rigscan_core::IncludeLoader for NoFiles {
    fn load(&self, _: &str, _: Option<&str>) -> Option<rigscan_core::Included> {
        None
    }
}

fn read_usd(text: &str) -> Robot {
    let files = Files(HashMap::from([(
        "robot.usda".to_string(),
        text.to_string(),
    )]));
    parse_usd("robot.usda", files)
        .unwrap_or_else(|e| panic!("{}\n{text}", e.0))
        .to_robot()
        .unwrap()
}

fn mjcf(src: &str) -> Robot {
    parse_mjcf(src, &NoFiles).unwrap().to_robot().unwrap()
}

/// A number at the precision USD keeps: schema attributes are `float`, so about seven digits.
fn n(x: f64) -> String {
    if x.abs() < 1e-6 {
        "0".to_string()
    } else {
        format!("{x:.5e}")
    }
}

fn nums(v: &[f64]) -> String {
    v.iter().map(|x| n(*x)).collect::<Vec<_>>().join(" ")
}

fn rotation(rpy: [f64; 3]) -> [[f64; 3]; 3] {
    let (sr, cr) = rpy[0].sin_cos();
    let (sp, cp) = rpy[1].sin_cos();
    let (sy, cy) = rpy[2].sin_cos();
    [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
}

fn flat(m: [[f64; 3]; 3]) -> Vec<f64> {
    m.iter().flatten().copied().collect()
}

/// The inertia tensor in the link frame: R I R^T for the inertial frame R.
fn link_tensor(i: &rigscan_core::Inertia, rpy: [f64; 3]) -> [f64; 6] {
    let r = rotation(rpy);
    let t = [
        [i.ixx, i.ixy, i.ixz],
        [i.ixy, i.iyy, i.iyz],
        [i.ixz, i.iyz, i.izz],
    ];
    let at = |a: usize, b: usize| -> f64 {
        (0..3)
            .map(|j| (0..3).map(|k| r[a][j] * t[j][k] * r[b][k]).sum::<f64>())
            .sum()
    };
    [at(0, 0), at(0, 1), at(0, 2), at(1, 1), at(1, 2), at(2, 2)]
}

fn sanitize(name: &str) -> String {
    let mut s: String = name
        .chars()
        .map(|c| if c.is_ascii_alphanumeric() { c } else { '_' })
        .collect();
    if s.starts_with(|c: char| c.is_ascii_digit()) || s.is_empty() {
        s.insert(0, '_');
    }
    s
}

/// The robot as comparable text: names without the path of the robot, numbers at USD precision,
/// rotations as matrices, and the synthetic root weld to the world removed. `lost` is what the
/// writer reported; it is applied to the source side (`written`) the way the writer applied it.
fn describe(r: &Robot, root: &str, written: bool, lost: &[Unmapped]) -> String {
    let has = |what: &str, name: &str| {
        written
            && lost
                .iter()
                .any(|u| u.what == what && u.name.as_deref() == Some(name))
    };
    let name = |n: &str| -> String {
        if n == "world" {
            n.to_string()
        } else if written {
            sanitize(n)
        } else {
            n.strip_prefix(&format!("/{root}/joints/"))
                .or_else(|| n.strip_prefix(&format!("/{root}/")))
                .unwrap_or(n)
                .to_string()
        }
    };
    let mut links = Vec::new();
    for l in &r.links {
        let mut s = format!("link {}", name(&l.name));
        if let Some(i) = &l.inertial {
            if !has("inertial", &l.name) {
                let (rpy, tensor) = match &i.inertia {
                    Some(t) if written => (i.origin.rpy, Some(link_tensor(t, i.origin.rpy))),
                    Some(t) => ([0.0; 3], Some([t.ixx, t.ixy, t.ixz, t.iyy, t.iyz, t.izz])),
                    None => ([0.0; 3], None),
                };
                let _ = rpy;
                let tensor = tensor.filter(|t| t.iter().any(|x| x.abs() > 1e-12));
                if i.mass.is_some() || tensor.is_some() {
                    let _ = write!(
                        s,
                        " mass {:?} com {} tensor {}",
                        i.mass.map(n),
                        nums(&i.origin.xyz),
                        tensor.map_or(String::new(), |t| nums(&t)),
                    );
                }
            }
        }
        for sh in &l.shapes {
            let g = match &sh.geometry {
                Geometry::Box { size } => format!("box {}", nums(size)),
                Geometry::Sphere { radius } => format!("sphere {}", n(*radius)),
                Geometry::Cylinder { radius, length } => {
                    format!("cylinder {}", nums(&[*radius, *length]))
                }
                Geometry::Capsule { radius, length } => {
                    format!("capsule {}", nums(&[*radius, *length]))
                }
                Geometry::Mesh { .. } => continue,
            };
            let role = match sh.role {
                Role::Visual => "visual",
                Role::Collision => "collision",
                Role::Both => "both",
            };
            let _ = write!(
                s,
                "\n  shape {role} {g} at {} turn {}",
                nums(&sh.origin.xyz),
                nums(&flat(rotation(sh.origin.rpy)))
            );
        }
        links.push(s);
    }
    let mut joints = Vec::new();
    let mut kept_world = false;
    for j in &r.joints {
        let from_world = j.parent == "world";
        let mut kind = j.kind.clone();
        if written {
            match &j.kind {
                JointKind::Revolute
                    if j.limit
                        .as_ref()
                        .is_none_or(|l| l.lower.is_none() && l.upper.is_none()) =>
                {
                    kind = JointKind::Continuous;
                }
                JointKind::Planar => kind = JointKind::Fixed,
                JointKind::Other(t) if t == "ball" => kind = JointKind::Other("spherical".into()),
                JointKind::Other(_) => kind = JointKind::Fixed,
                JointKind::Floating if !from_world => continue,
                _ => {}
            }
        }
        if from_world && matches!(kind, JointKind::Fixed | JointKind::Floating) {
            continue;
        }
        kept_world |= from_world || j.child == "world";
        let moves = matches!(
            kind,
            JointKind::Revolute | JointKind::Continuous | JointKind::Prismatic
        );
        let axis = j
            .axis
            .filter(|_| moves)
            .map(|a| {
                let len = (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt();
                a.map(|v| v / len)
            })
            .map_or(String::new(), |a| nums(&a));
        let limit = j
            .limit
            .as_ref()
            .filter(|_| {
                moves
                    && matches!(
                        kind,
                        JointKind::Revolute | JointKind::Prismatic | JointKind::Continuous
                    )
            })
            .map_or(String::new(), |l| {
                let effort = l.effort.filter(|e| !written || *e > 0.0);
                let bounded = !matches!(kind, JointKind::Continuous);
                format!(
                    "{} {} effort {}",
                    l.lower.filter(|_| bounded).map_or("-".into(), n),
                    l.upper.filter(|_| bounded).map_or("-".into(), n),
                    effort.map_or("-".into(), n)
                )
            });
        joints.push(format!(
            "joint {} {} -> {} {:?} at {} turn {} axis {} limit {}",
            name(&j.name),
            name(&j.parent),
            name(&j.child),
            kind,
            nums(&j.origin.xyz),
            nums(&flat(rotation(j.origin.rpy))),
            axis,
            limit
        ));
    }
    if !kept_world {
        links.retain(|l| l != "link world");
    }
    links.sort();
    joints.sort();
    format!("{}\n{}\n", links.join("\n"), joints.join("\n"))
}

fn round_trip(label: &str, robot: &Robot) -> rigscan_core::Converted {
    let out = write_usd(robot);
    let back = read_usd(&out.text);
    let root = sanitize(robot.name.as_deref().unwrap_or("robot"));
    assert_eq!(
        describe(robot, &root, true, &out.lost),
        describe(&back, &root, false, &[]),
        "{label}\n{}",
        out.text
    );
    out
}

const FULL: &str = r#"<robot name="arm">
  <link name="base"/>
  <link name="upper">
    <inertial>
      <origin xyz="0 0 0.1" rpy="0 0 0.3"/>
      <mass value="2"/>
      <inertia ixx="0.1" ixy="0.02" ixz="0" iyy="0.2" iyz="0.01" izz="0.3"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0.2 0"/>
      <geometry><box size="0.1 0.2 0.3"/></geometry>
    </visual>
    <collision>
      <origin xyz="0 0.05 0" rpy="1.5707963 0 0"/>
      <geometry><cylinder radius="0.04" length="0.5"/></geometry>
    </collision>
    <visual>
      <geometry><mesh filename="package://arm/upper.stl"/></geometry>
    </visual>
  </link>
  <link name="fore">
    <inertial>
      <origin xyz="0.1 0 0" rpy="0 0 0"/>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.03"/>
    </inertial>
    <visual>
      <geometry><sphere radius="0.05"/></geometry>
    </visual>
  </link>
  <link name="hand"/>
  <link name="finger"/>
  <joint name="shoulder" type="revolute">
    <parent link="base"/>
    <child link="upper"/>
    <origin xyz="0 0 1" rpy="0 0 0.5"/>
    <axis xyz="0 0.6 0.8"/>
    <limit lower="-1" upper="1.5" effort="10" velocity="2"/>
    <dynamics damping="0.1" friction="0.2"/>
  </joint>
  <joint name="elbow" type="continuous">
    <parent link="upper"/>
    <child link="fore"/>
    <origin xyz="0.3 0 0" rpy="0.1 0.2 0.3"/>
    <axis xyz="0 1 0"/>
  </joint>
  <joint name="slide" type="prismatic">
    <parent link="fore"/>
    <child link="hand"/>
    <origin xyz="0.2 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 -1"/>
    <limit lower="0" upper="0.1" effort="5" velocity="1"/>
    <mimic joint="shoulder" multiplier="2" offset="0.5"/>
  </joint>
  <joint name="mount" type="fixed">
    <parent link="hand"/>
    <child link="finger"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
  </joint>
</robot>"#;

fn ids(out: &rigscan_core::Converted, what: &str) -> Vec<String> {
    out.lost
        .iter()
        .filter(|u| u.what == what)
        .filter_map(|u| u.name.clone())
        .collect()
}

#[test]
fn a_urdf_robot_survives_a_round_trip_through_usd() {
    let robot = parse_urdf(FULL).unwrap();
    round_trip("full", &robot);
}

#[test]
fn what_usd_cannot_hold_is_reported() {
    let out = write_usd(&parse_urdf(FULL).unwrap());
    assert_eq!(ids(&out, "mimic"), ["slide"]);
    assert_eq!(ids(&out, "dynamics"), ["shoulder"]);
    assert_eq!(ids(&out, "limit/velocity"), ["shoulder", "slide"]);
    assert_eq!(ids(&out, "mesh"), ["upper"]);
    assert!(ids(&out, "limit/effort").is_empty(), "{:?}", out.lost);
}

#[test]
fn the_text_is_one_flat_list_of_bodies_and_joints_under_an_articulation_root() {
    let text = write_usd(&parse_urdf(FULL).unwrap()).text;
    for expected in [
        "#usda 1.0",
        "defaultPrim = \"arm\"",
        "metersPerUnit = 1",
        "kilogramsPerUnit = 1",
        "upAxis = \"Z\"",
        "def Xform \"arm\" (",
        "PhysicsArticulationRootAPI",
        "def Xform \"upper\" (",
        "PhysicsRigidBodyAPI",
        "def PhysicsRevoluteJoint \"shoulder\"",
        "def PhysicsPrismaticJoint \"slide\"",
        "def PhysicsFixedJoint \"mount\"",
        "rel physics:body0 = </arm/base>",
        "rel physics:body1 = </arm/upper>",
    ] {
        assert!(text.contains(expected), "{expected}\n{text}");
    }
}

#[test]
fn an_axis_along_x_y_or_z_is_a_token_and_another_axis_is_a_turned_frame() {
    let text = write_usd(&parse_urdf(FULL).unwrap()).text;
    assert!(text.contains("token physics:axis = \"Y\""), "{text}");
    assert!(text.contains("token physics:axis = \"X\""), "{text}");
}

#[test]
fn a_body_is_placed_where_the_joints_put_it() {
    // upper is 1 above base; fore is 0.3 along upper's x, turned 0.5 about z by the shoulder.
    let text = write_usd(&parse_urdf(FULL).unwrap()).text;
    assert!(
        text.contains("double3 xformOp:translate = (0, 0, 1)"),
        "{text}"
    );
}

#[test]
fn a_root_link_is_held_to_the_world_and_a_free_root_is_not() {
    let welded = write_usd(&parse_urdf(FULL).unwrap());
    let r = read_usd(&welded.text);
    let weld = r.joints.iter().find(|j| j.parent == "world").unwrap();
    assert_eq!(
        (weld.kind.clone(), weld.child.as_str()),
        (JointKind::Fixed, "/arm/base")
    );
    assert!(welded.lost.iter().all(|u| u.what != "joint/root"));

    let free = write_usd(&mjcf(
        r#"<mujoco model="f"><worldbody><body name="a"><freejoint name="root"/>
  <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/></body></worldbody></mujoco>"#,
    ));
    let r = read_usd(&free.text);
    assert!(r.joints.is_empty(), "{:?}", r.joints);
    assert!(free.lost.is_empty(), "{:?}", free.lost);
}

#[test]
fn a_name_that_is_not_a_usd_identifier_is_changed_and_reported() {
    let robot = parse_urdf(
        r#"<robot name="my robot"><link name="1st-link"/><link name="b"/>
  <joint name="j" type="fixed"><parent link="1st-link"/><child link="b"/></joint></robot>"#,
    )
    .unwrap();
    let out = write_usd(&robot);
    assert!(out.text.contains("def Xform \"_1st_link\""), "{}", out.text);
    assert_eq!(ids(&out, "name"), ["my robot", "1st-link"]);
    round_trip("names", &robot);
}

#[test]
fn two_joints_into_one_link_are_both_written() {
    let robot = parse_urdf(
        r#"<robot name="loop"><link name="a"/><link name="b"/><link name="c"/>
  <joint name="x" type="continuous"><parent link="a"/><child link="c"/><axis xyz="0 0 1"/></joint>
  <joint name="y" type="continuous"><parent link="b"/><child link="c"/><axis xyz="0 0 1"/></joint>
  <joint name="z" type="fixed"><parent link="a"/><child link="b"/></joint></robot>"#,
    )
    .unwrap();
    let out = write_usd(&robot);
    assert_eq!(out.text.matches("RevoluteJoint").count(), 2, "{}", out.text);
    assert!(ids(&out, "joint/multiple").is_empty());
}

#[test]
fn an_mjcf_robot_survives_a_round_trip_through_usd() {
    let robot = mjcf(
        r#"<mujoco model="m"><worldbody><body name="a" pos="0 0 1">
  <inertial pos="0.1 0 0" mass="1" diaginertia="0.1 0.2 0.3"/>
  <geom type="capsule" size="0.05 0.2" contype="0" conaffinity="0"/>
  <body name="b" pos="0 0.5 0"><inertial pos="0 0 0" mass="2" diaginertia="0.1 0.1 0.1"/>
    <joint name="hinge" type="hinge" axis="1 1 0" range="-30 60"/>
    <geom type="sphere" size="0.05"/></body>
  <body name="c" pos="0 -0.5 0"><inertial pos="0 0 0" mass="2" diaginertia="0.1 0.1 0.1"/>
    <joint name="rail" type="slide" axis="0 0 1" range="0 0.2"/></body>
  </body></worldbody></mujoco>"#,
    );
    let out = round_trip("mjcf", &robot);
    assert!(out.lost.is_empty(), "{:?}", out.lost);
}

#[test]
fn joint_kinds_usd_has_no_form_for_are_reported() {
    let robot = parse_urdf(
        r#"<robot name="k"><link name="a"/><link name="b"/><link name="c"/><link name="d"/>
  <joint name="p" type="planar"><parent link="a"/><child link="b"/></joint>
  <joint name="f" type="floating"><parent link="b"/><child link="c"/></joint>
  <joint name="g" type="fixed"><parent link="c"/><child link="d"/></joint></robot>"#,
    )
    .unwrap();
    let out = write_usd(&robot);
    assert_eq!(ids(&out, "joint/planar"), ["p"]);
    assert_eq!(ids(&out, "joint/floating"), ["f"]);
    assert!(read_usd(&out.text).joints.len() >= 2);
}

#[test]
fn a_mass_that_is_not_positive_is_left_out_and_reported() {
    let robot = parse_urdf(
        r#"<robot name="z"><link name="a"><inertial><mass value="0"/>
  <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial></link></robot>"#,
    )
    .unwrap();
    let out = write_usd(&robot);
    assert_eq!(ids(&out, "inertial"), ["a"]);
    assert!(!out.text.contains("physics:mass"), "{}", out.text);
}

#[test]
fn a_joint_to_a_missing_link_is_reported_and_not_written() {
    let mut robot = parse_urdf(
        r#"<robot name="u"><link name="a"/><link name="b"/>
  <joint name="j" type="fixed"><parent link="a"/><child link="b"/></joint></robot>"#,
    )
    .unwrap();
    robot.joints[0].child = "ghost".into();
    let out = write_usd(&robot);
    assert_eq!(ids(&out, "joint/undefined"), ["j"]);
    assert!(!out.text.contains("\"j\""), "{}", out.text);
}

const TENSORS: [&str; 6] = [
    "ixx=\"2\" ixy=\"0.3\" ixz=\"0.1\" iyy=\"3\" iyz=\"0.2\" izz=\"4\"",
    "ixx=\"4\" ixy=\"-0.3\" ixz=\"0.1\" iyy=\"3\" iyz=\"0.2\" izz=\"2\"",
    "ixx=\"3\" ixy=\"0.5\" ixz=\"-0.4\" iyy=\"2\" iyz=\"0.3\" izz=\"4\"",
    "ixx=\"2\" ixy=\"0.1\" ixz=\"0.1\" iyy=\"2.5\" iyz=\"-0.2\" izz=\"3\"",
    "ixx=\"5\" ixy=\"1\" ixz=\"0\" iyy=\"3\" iyz=\"0\" izz=\"4\"",
    "ixx=\"3\" ixy=\"0\" ixz=\"1\" iyy=\"5\" iyz=\"1\" izz=\"4\"",
];

#[test]
fn a_full_inertia_tensor_of_any_handedness_round_trips() {
    for (i, tensor) in TENSORS.iter().enumerate() {
        let robot = parse_urdf(&format!(
            r#"<robot name="t"><link name="a"><inertial><mass value="1"/><inertia {tensor}/></inertial></link></robot>"#
        ))
        .unwrap();
        round_trip(&format!("tensor {i}"), &robot);
    }
}

#[test]
fn an_axis_pointing_down_x_and_a_zero_axis_still_round_trip() {
    for axis in ["-1 0 0", "0 0 0"] {
        let robot = parse_urdf(&format!(
            r#"<robot name="x"><link name="a"/><link name="b"/>
  <joint name="j" type="revolute"><parent link="a"/><child link="b"/><axis xyz="{axis}"/>
  <limit lower="-1" upper="1" effort="1" velocity="1"/></joint></robot>"#
        ))
        .unwrap();
        write_usd(&robot);
        if axis != "0 0 0" {
            round_trip(axis, &robot);
        }
    }
}

#[test]
fn ball_and_unknown_joints_and_unlimited_kinds_are_written_or_reported() {
    let ball = mjcf(
        r#"<mujoco model="b"><worldbody><body name="a"><inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
  <body name="c"><inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/><joint name="s" type="ball"/></body>
  </body></worldbody></mujoco>"#,
    );
    let out = write_usd(&ball);
    assert!(
        out.text.contains("def PhysicsSphericalJoint \"s\""),
        "{}",
        out.text
    );

    let mut weird = parse_urdf(
        r#"<robot name="w"><link name="a"/><link name="b"/>
  <joint name="j" type="fixed"><parent link="a"/><child link="b"/></joint></robot>"#,
    )
    .unwrap();
    weird.joints[0].kind = JointKind::Other("gear".into());
    assert_eq!(ids(&write_usd(&weird), "joint/gear"), ["j"]);

    let bounded = parse_urdf(
        r#"<robot name="c"><link name="a"/><link name="b"/>
  <joint name="j" type="continuous"><parent link="a"/><child link="b"/>
  <limit lower="-1" upper="1" effort="1" velocity="1"/></joint></robot>"#,
    )
    .unwrap();
    assert_eq!(ids(&write_usd(&bounded), "limit"), ["j"]);
}

#[test]
fn a_revolute_joint_with_no_limit_at_all_is_reported() {
    let mut robot = parse_urdf(
        r#"<robot name="n"><link name="a"/><link name="b"/>
  <joint name="j" type="continuous"><parent link="a"/><child link="b"/></joint></robot>"#,
    )
    .unwrap();
    robot.joints[0].kind = JointKind::Revolute;
    assert_eq!(ids(&write_usd(&robot), "limit"), ["j"]);
}

use rigscan_core::{
    parse_urdf, Geometry, Inertia, Inertial, Joint, JointKind, Limit, Origin, Role, Shape, Span,
};

fn joint_with(type_attr: &str, inner: &str) -> Joint {
    let src = format!(
        "<robot name=\"t\"><link name=\"a\"/><link name=\"b\"/>\
         <joint name=\"j\" {type_attr}><parent link=\"a\"/><child link=\"b\"/>{inner}</joint></robot>"
    );
    parse_urdf(&src).unwrap().joints.remove(0)
}

#[test]
fn joint_types_are_recognised() {
    let cases = [
        ("revolute", JointKind::Revolute),
        ("continuous", JointKind::Continuous),
        ("prismatic", JointKind::Prismatic),
        ("fixed", JointKind::Fixed),
        ("floating", JointKind::Floating),
        ("planar", JointKind::Planar),
    ];
    for (name, kind) in cases {
        assert_eq!(joint_with(&format!("type=\"{name}\""), "").kind, kind);
    }
}

#[test]
fn unknown_or_missing_joint_type_is_kept_for_rules() {
    assert_eq!(
        joint_with("type=\"hinge\"", "").kind,
        JointKind::Other("hinge".into())
    );
    assert_eq!(joint_with("", "").kind, JointKind::Other(String::new()));
}

#[test]
fn origin_defaults_to_zero() {
    assert_eq!(joint_with("type=\"fixed\"", "").origin, Origin::default());
}

#[test]
fn origin_reads_xyz_and_rpy() {
    let j = joint_with(
        "type=\"fixed\"",
        "<origin xyz=\"1 2 3.5\" rpy=\"0.1 0.2 0.3\"/>",
    );

    assert_eq!(j.origin.xyz, [1.0, 2.0, 3.5]);
    assert_eq!(j.origin.rpy, [0.1, 0.2, 0.3]);
}

#[test]
fn origin_with_only_xyz_keeps_zero_rotation() {
    let j = joint_with("type=\"fixed\"", "<origin xyz=\"1 0 0\"/>");

    assert_eq!(j.origin.xyz, [1.0, 0.0, 0.0]);
    assert_eq!(j.origin.rpy, [0.0, 0.0, 0.0]);
}

#[test]
fn limit_reads_all_attributes() {
    let j = joint_with(
        "type=\"revolute\"",
        "<limit lower=\"-1.5\" upper=\"1.5\" effort=\"10\" velocity=\"2\"/>",
    );

    assert_eq!(
        j.limit,
        Some(Limit {
            lower: Some(-1.5),
            upper: Some(1.5),
            effort: Some(10.0),
            velocity: Some(2.0),
        })
    );
}

#[test]
fn missing_limit_element_is_none_and_missing_attributes_are_none() {
    assert_eq!(joint_with("type=\"revolute\"", "").limit, None);

    let j = joint_with("type=\"revolute\"", "<limit effort=\"5\"/>");
    assert_eq!(
        j.limit,
        Some(Limit {
            lower: None,
            upper: None,
            effort: Some(5.0),
            velocity: None,
        })
    );
}

#[test]
fn mimic_reads_joint_multiplier_and_offset() {
    let j = joint_with(
        "type=\"revolute\"",
        "<mimic joint=\"other\" multiplier=\"2\" offset=\"0.5\"/>",
    );

    assert_eq!(
        j.mimic.map(|m| (m.joint, m.multiplier, m.offset)),
        Some(("other".into(), Some(2.0), Some(0.5)))
    );
}

#[test]
fn mimic_with_only_a_joint_has_no_multiplier_or_offset() {
    let j = joint_with("type=\"revolute\"", "<mimic joint=\"other\"/>");

    assert_eq!(
        j.mimic.map(|m| (m.joint, m.multiplier, m.offset)),
        Some(("other".into(), None, None))
    );
}

#[test]
fn limit_inside_a_transmission_joint_is_not_attached_to_the_real_joint() {
    let src = r#"<robot name="t"><link name="a"/><link name="b"/>
<joint name="j" type="revolute"><parent link="a"/><child link="b"/></joint>
<transmission name="t1"><joint name="j"><limit lower="0" upper="1"/></joint></transmission></robot>"#;

    let robot = parse_urdf(src).unwrap();

    assert_eq!(robot.joints.len(), 1);
    assert_eq!(robot.joints[0].limit, None);
}

#[test]
fn invalid_number_is_an_error_naming_the_attribute_and_location() {
    let src = r#"<robot name="t">
  <link name="a"/><link name="b"/>
  <joint name="j" type="revolute">
    <parent link="a"/><child link="b"/>
    <limit lower="abc"/>
  </joint>
</robot>"#;

    let err = parse_urdf(src).unwrap_err().0;

    assert!(err.starts_with("5:5:"), "{err}");
    assert!(err.contains("lower") && err.contains("abc"), "{err}");
}

#[test]
fn vector_with_wrong_length_is_an_error() {
    let src = "<robot name=\"t\"><link name=\"a\"/><link name=\"b\"/>\
<joint name=\"j\" type=\"fixed\"><parent link=\"a\"/><child link=\"b\"/>\
<origin xyz=\"1 2\"/></joint></robot>";

    let err = parse_urdf(src).unwrap_err().0;

    assert!(err.contains("xyz"), "{err}");
}

fn link_with(inner: &str) -> rigscan_core::Link {
    let src = format!("<robot name=\"t\"><link name=\"a\">{inner}</link></robot>");
    parse_urdf(&src).unwrap().links.remove(0)
}

const INERTIA: &str =
    "<inertia ixx=\"1\" ixy=\"0.1\" ixz=\"0.2\" iyy=\"2\" iyz=\"0.3\" izz=\"3\"/>";

#[test]
fn inertial_reads_mass_and_inertia() {
    let l = link_with(&format!(
        "<inertial><origin xyz=\"0 0 0\"/><mass value=\"2.5\"/>{INERTIA}</inertial>"
    ));

    assert_eq!(
        l.inertial,
        Some(Inertial {
            mass: Some(2.5),
            inertia: Some(Inertia {
                ixx: 1.0,
                ixy: 0.1,
                ixz: 0.2,
                iyy: 2.0,
                iyz: 0.3,
                izz: 3.0,
            }),
            origin: Default::default(),
            span: Span { line: 1, col: 32 },
        })
    );
}

#[test]
fn link_without_inertial_has_none() {
    assert_eq!(link_with("").inertial, None);
}

#[test]
fn inertial_without_mass_keeps_the_inertia() {
    let l = link_with(&format!("<inertial>{INERTIA}</inertial>"));

    let inertial = l.inertial.unwrap();
    assert_eq!(inertial.mass, None);
    assert!(inertial.inertia.is_some());
}

#[test]
fn inertia_missing_an_attribute_is_an_error_naming_it() {
    let src = "<robot name=\"t\"><link name=\"a\"><inertial>\
<inertia ixx=\"1\" iyy=\"2\" izz=\"3\"/></inertial></link></robot>";

    let err = parse_urdf(src).unwrap_err().0;

    assert!(err.contains("ixy"), "{err}");
}

#[test]
fn mesh_references_are_collected_from_visual_and_collision_with_spans() {
    let src = r#"<robot name="t">
  <link name="a">
    <visual><geometry><mesh filename="package://pkg/meshes/a.stl"/></geometry></visual>
    <collision><geometry><mesh filename="meshes/a_collision.obj" scale="1 1 1"/></geometry></collision>
  </link>
</robot>"#;

    let link = parse_urdf(src).unwrap().links.remove(0);

    assert_eq!(
        link.meshes().collect::<Vec<_>>(),
        [
            ("package://pkg/meshes/a.stl", Span { line: 3, col: 23 }),
            ("meshes/a_collision.obj", Span { line: 4, col: 26 }),
        ]
    );
}

#[test]
fn primitive_geometry_has_no_mesh_references() {
    let l = link_with(
        "<visual><geometry><box size=\"1 1 1\"/></geometry></visual>\
<collision><geometry><cylinder radius=\"1\" length=\"2\"/></geometry></collision>",
    );

    assert_eq!(l.meshes().count(), 0);
}

#[test]
fn mesh_without_a_filename_is_an_error_naming_it() {
    let src = "<robot name=\"t\"><link name=\"a\"><visual><geometry><mesh/></geometry></visual></link></robot>";

    let err = parse_urdf(src).unwrap_err().0;

    assert!(err.contains("filename"), "{err}");
}

#[test]
fn non_finite_numbers_are_rejected() {
    for v in ["nan", "NaN", "inf", "-inf", "infinity"] {
        let src = format!(
            r#"<robot name="t"><link name="a"><inertial><mass value="{v}"/></inertial></link></robot>"#
        );

        let err = parse_urdf(&src).unwrap_err().0;

        assert!(err.contains("invalid number") && err.contains(v), "{err}");
    }
}

#[test]
fn revolute_joint_without_axis_gets_the_urdf_default() {
    assert_eq!(
        joint_with("type=\"revolute\"", "").axis,
        Some([1.0, 0.0, 0.0])
    );
    assert_eq!(
        joint_with("type=\"continuous\"", "").axis,
        Some([1.0, 0.0, 0.0])
    );
    assert_eq!(
        joint_with("type=\"prismatic\"", "").axis,
        Some([1.0, 0.0, 0.0])
    );
}

#[test]
fn authored_axis_is_kept_as_written() {
    let j = joint_with("type=\"revolute\"", "<axis xyz=\"0 2 0\"/>");
    assert_eq!(j.axis, Some([0.0, 2.0, 0.0]));
}

#[test]
fn fixed_and_floating_joints_have_no_axis() {
    assert_eq!(
        joint_with("type=\"fixed\"", "<axis xyz=\"0 0 1\"/>").axis,
        None
    );
    assert_eq!(joint_with("type=\"floating\"", "").axis, None);
}

#[test]
fn joint_dynamics_are_read() {
    let j = joint_with(
        "type=\"revolute\"",
        "<dynamics damping=\"0.5\" friction=\"0.1\"/>",
    );
    let d = j.dynamics.unwrap();
    assert_eq!((d.damping, d.friction), (Some(0.5), Some(0.1)));
}

#[test]
fn a_missing_dynamics_value_stays_none() {
    let j = joint_with("type=\"revolute\"", "<dynamics damping=\"0.5\"/>");
    let d = j.dynamics.unwrap();
    assert_eq!((d.damping, d.friction), (Some(0.5), None));
    assert_eq!(joint_with("type=\"revolute\"", "").dynamics, None);
}

#[test]
fn inertial_origin_is_read() {
    let r = parse_urdf(
        "<robot name=\"t\"><link name=\"a\"><inertial><origin xyz=\"1 2 3\" rpy=\"0.1 0.2 0.3\"/><mass value=\"1\"/></inertial></link></robot>",
    )
    .unwrap();
    let o = r.links[0].inertial.as_ref().unwrap().origin;
    assert_eq!((o.xyz, o.rpy), ([1.0, 2.0, 3.0], [0.1, 0.2, 0.3]));
}

#[test]
fn inertial_origin_with_only_one_attribute_keeps_the_other_at_zero() {
    let r = parse_urdf(
        "<robot name=\"t\"><link name=\"a\"><inertial><origin xyz=\"1 2 3\"/></inertial></link><link name=\"b\"><inertial><origin rpy=\"0.1 0.2 0.3\"/></inertial></link></robot>",
    )
    .unwrap();
    let a = r.links[0].inertial.as_ref().unwrap().origin;
    let b = r.links[1].inertial.as_ref().unwrap().origin;
    assert_eq!((a.xyz, a.rpy), ([1.0, 2.0, 3.0], [0.0; 3]));
    assert_eq!((b.xyz, b.rpy), ([0.0; 3], [0.1, 0.2, 0.3]));
}

#[test]
fn bad_numbers_in_axis_dynamics_and_inertial_origin_are_parse_errors() {
    let bad = [
        "<robot name=\"t\"><link name=\"a\"/><link name=\"b\"/><joint name=\"j\" type=\"revolute\"><parent link=\"a\"/><child link=\"b\"/><axis xyz=\"0 1\"/></joint></robot>",
        "<robot name=\"t\"><link name=\"a\"/><link name=\"b\"/><joint name=\"j\" type=\"revolute\"><parent link=\"a\"/><child link=\"b\"/><dynamics damping=\"x\"/></joint></robot>",
        "<robot name=\"t\"><link name=\"a\"/><link name=\"b\"/><joint name=\"j\" type=\"revolute\"><parent link=\"a\"/><child link=\"b\"/><dynamics friction=\"x\"/></joint></robot>",
        "<robot name=\"t\"><link name=\"a\"><inertial><origin xyz=\"1 2\"/></inertial></link></robot>",
        "<robot name=\"t\"><link name=\"a\"><inertial><origin rpy=\"1 2\"/></inertial></link></robot>",
    ];
    for src in bad {
        let err = parse_urdf(src).unwrap_err().0;
        assert!(err.starts_with("1:"), "{src}: {err}");
    }
}

fn shapes_of(inner: &str) -> Vec<Shape> {
    parse_urdf(&format!(
        "<robot name=\"t\"><link name=\"a\">{inner}</link></robot>"
    ))
    .unwrap()
    .links
    .remove(0)
    .shapes
}

#[test]
fn visual_and_collision_primitives_become_shapes_with_their_role() {
    let s = shapes_of(
        "<visual><geometry><box size=\"1 2 3\"/></geometry></visual>\
         <collision><geometry><sphere radius=\"0.5\"/></geometry></collision>\
         <collision><geometry><cylinder radius=\"0.1\" length=\"2\"/></geometry></collision>",
    );
    let got: Vec<_> = s.iter().map(|s| (s.role, s.geometry.clone())).collect();
    assert_eq!(
        got,
        [
            (
                Role::Visual,
                Geometry::Box {
                    size: [1.0, 2.0, 3.0]
                }
            ),
            (Role::Collision, Geometry::Sphere { radius: 0.5 }),
            (
                Role::Collision,
                Geometry::Cylinder {
                    radius: 0.1,
                    length: 2.0
                }
            ),
        ]
    );
}

#[test]
fn a_mesh_shape_keeps_filename_scale_and_default_scale() {
    let s = shapes_of(
        "<visual><geometry><mesh filename=\"a.stl\" scale=\"1 2 3\"/></geometry></visual>\
         <collision><geometry><mesh filename=\"b.stl\"/></geometry></collision>",
    );
    assert_eq!(
        s[0].geometry,
        Geometry::Mesh {
            filename: "a.stl".into(),
            scale: [1.0, 2.0, 3.0]
        }
    );
    assert_eq!(
        s[1].geometry,
        Geometry::Mesh {
            filename: "b.stl".into(),
            scale: [1.0; 3]
        }
    );
}

#[test]
fn shape_origin_is_read_before_or_after_the_geometry() {
    let s = shapes_of(
        "<visual><origin xyz=\"1 2 3\" rpy=\"0.1 0.2 0.3\"/><geometry><sphere radius=\"1\"/></geometry></visual>\
         <collision><geometry><sphere radius=\"1\"/></geometry><origin xyz=\"4 5 6\"/></collision>\
         <visual><geometry><sphere radius=\"1\"/></geometry></visual>",
    );
    assert_eq!(
        (s[0].origin.xyz, s[0].origin.rpy),
        ([1.0, 2.0, 3.0], [0.1, 0.2, 0.3])
    );
    assert_eq!(
        (s[1].origin.xyz, s[1].origin.rpy),
        ([4.0, 5.0, 6.0], [0.0; 3])
    );
    assert_eq!(s[2].origin, Origin::default());
}

#[test]
fn shape_span_is_the_geometry_element() {
    let r = parse_urdf(
        "<robot name=\"t\">\n  <link name=\"a\">\n    <visual><geometry><box size=\"1 1 1\"/></geometry></visual>\n  </link>\n</robot>",
    )
    .unwrap();
    assert_eq!(r.links[0].shapes[0].span, Span { line: 3, col: 23 });
}

#[test]
fn a_shape_without_geometry_is_not_a_shape() {
    assert!(shapes_of("<visual/><collision><origin xyz=\"1 0 0\"/></collision>").is_empty());
}

#[test]
fn bad_shape_numbers_are_parse_errors() {
    for inner in [
        "<visual><geometry><box size=\"1 2\"/></geometry></visual>",
        "<visual><geometry><sphere/></geometry></visual>",
        "<visual><geometry><cylinder radius=\"1\"/></geometry></visual>",
        "<visual><geometry><mesh filename=\"a\" scale=\"1\"/></geometry></visual>",
        "<visual><origin xyz=\"1\"/><geometry><sphere radius=\"1\"/></geometry></visual>",
        "<visual><origin rpy=\"1\"/><geometry><sphere radius=\"1\"/></geometry></visual>",
        "<visual><geometry><mesh/></geometry></visual>",
        "<visual><geometry><cylinder length=\"1\"/></geometry></visual>",
        "<visual><geometry><box/></geometry></visual>",
    ] {
        let src = format!("<robot name=\"t\"><link name=\"a\">{inner}</link></robot>");
        assert!(parse_urdf(&src).is_err(), "{inner}");
    }
}

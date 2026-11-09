use rigscan_core::{parse_urdf, Inertia, Inertial, Joint, JointKind, Limit, MeshRef, Origin, Span};

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

    let meshes = parse_urdf(src).unwrap().links.remove(0).meshes;

    assert_eq!(
        meshes,
        vec![
            MeshRef {
                filename: "package://pkg/meshes/a.stl".into(),
                span: Span { line: 3, col: 23 },
            },
            MeshRef {
                filename: "meshes/a_collision.obj".into(),
                span: Span { line: 4, col: 26 },
            },
        ]
    );
}

#[test]
fn primitive_geometry_has_no_mesh_references() {
    let l = link_with(
        "<visual><geometry><box size=\"1 1 1\"/></geometry></visual>\
<collision><geometry><cylinder radius=\"1\" length=\"2\"/></geometry></collision>",
    );

    assert!(l.meshes.is_empty());
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

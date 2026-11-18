use std::fs;
use std::path::{Path, PathBuf};
use std::process::{Command, Output};

const ARM: &str = r#"<robot name="arm">
  <link name="base"/>
  <link name="upper">
    <inertial>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <mass value="2"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.2" iyz="0" izz="0.3"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry><box size="0.1 0.2 0.3"/></geometry>
    </visual>
  </link>
  <joint name="shoulder" type="continuous">
    <parent link="base"/>
    <child link="upper"/>
    <origin xyz="0 0 1" rpy="0 0 0.5"/>
    <axis xyz="0 1 0"/>
  </joint>
</robot>"#;

const WITH_EFFORT: &str = r#"<robot name="arm">
  <link name="base"/>
  <link name="upper"/>
  <joint name="shoulder" type="revolute">
    <parent link="base"/>
    <child link="upper"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1" upper="1" effort="10" velocity="2"/>
  </joint>
</robot>"#;

const BROKEN: &str = r#"<robot name="bad">
  <link name="base"/>
  <joint name="j" type="fixed">
    <parent link="base"/>
    <child link="ghost"/>
  </joint>
</robot>"#;

fn tree(name: &str, files: &[(&str, &str)]) -> PathBuf {
    let root = Path::new(env!("CARGO_TARGET_TMPDIR")).join(format!("convert-{name}"));
    let _ = fs::remove_dir_all(&root);
    fs::create_dir_all(&root).unwrap();
    for (path, content) in files {
        fs::write(root.join(path), content).unwrap();
    }
    root
}

fn convert(dir: &Path, args: &[&str]) -> Output {
    Command::new(env!("CARGO_BIN_EXE_rigscan"))
        .current_dir(dir)
        .arg("convert")
        .args(args)
        .output()
        .unwrap()
}

fn stdout(o: &Output) -> String {
    String::from_utf8_lossy(&o.stdout).into_owned()
}

fn stderr(o: &Output) -> String {
    String::from_utf8_lossy(&o.stderr).into_owned()
}

#[test]
fn lossless_conversion_prints_the_text_and_exits_zero() {
    let root = tree("clean", &[("arm.urdf", ARM)]);
    let out = convert(&root, &["arm.urdf", "--to", "mjcf"]);

    assert_eq!(out.status.code(), Some(0), "{}", stderr(&out));
    assert!(stdout(&out).starts_with("<mujoco model=\"arm\">"));
    assert_eq!(stderr(&out), "");
}

#[test]
fn lost_data_is_listed_on_stderr_and_exits_one() {
    let root = tree("lossy", &[("arm.urdf", WITH_EFFORT)]);
    let out = convert(&root, &["arm.urdf", "--to", "mjcf"]);

    assert_eq!(out.status.code(), Some(1));
    assert!(stdout(&out).starts_with("<mujoco"));
    assert!(
        stderr(&out).contains("limit/effort: shoulder"),
        "{}",
        stderr(&out)
    );
}

#[test]
fn output_file_gets_the_text_and_stdout_stays_empty() {
    let root = tree("file", &[("arm.urdf", ARM)]);
    let out = convert(&root, &["arm.urdf", "--to", "mjcf", "-o", "arm.xml"]);

    assert_eq!(out.status.code(), Some(0), "{}", stderr(&out));
    assert_eq!(stdout(&out), "");
    let written = fs::read_to_string(root.join("arm.xml")).unwrap();
    assert!(written.starts_with("<mujoco model=\"arm\">"));
}

#[test]
fn same_format_in_and_out_is_an_error() {
    let root = tree("same", &[("arm.urdf", ARM)]);
    let out = convert(&root, &["arm.urdf", "--to", "urdf"]);

    assert_eq!(out.status.code(), Some(2));
    assert_eq!(stdout(&out), "");
    assert!(stderr(&out).contains("already URDF"), "{}", stderr(&out));
}

#[test]
fn input_that_does_not_parse_exits_two() {
    let root = tree("malformed", &[("bad.urdf", "<robot><link")]);
    let out = convert(&root, &["bad.urdf", "--to", "mjcf"]);

    assert_eq!(out.status.code(), Some(2));
    assert_eq!(stdout(&out), "");
    assert!(stderr(&out).starts_with("bad.urdf:"), "{}", stderr(&out));
}

#[test]
fn input_with_critical_findings_is_refused_with_the_findings() {
    let root = tree("critical", &[("bad.urdf", BROKEN)]);
    let out = convert(&root, &["bad.urdf", "--to", "mjcf", "-o", "bad.xml"]);

    assert_eq!(out.status.code(), Some(2));
    assert!(stderr(&out).contains("critical RS001"), "{}", stderr(&out));
    assert!(!root.join("bad.xml").exists());
}

#[test]
fn missing_input_exits_two() {
    let root = tree("missing", &[]);
    let out = convert(&root, &["nope.urdf", "--to", "mjcf"]);

    assert_eq!(out.status.code(), Some(2));
    assert!(stderr(&out).starts_with("nope.urdf:"), "{}", stderr(&out));
}

#[test]
fn urdf_to_mjcf_to_urdf_to_mjcf_to_urdf_settles_after_one_round() {
    let root = tree("round", &[("a.urdf", ARM)]);
    let step = |input: &str, to: &str, output: &str| {
        let out = convert(&root, &[input, "--to", to, "-o", output]);
        assert_eq!(out.status.code(), Some(0), "{}", stderr(&out));
        fs::read_to_string(root.join(output)).unwrap()
    };
    step("a.urdf", "mjcf", "b.xml");
    let a1 = step("b.xml", "urdf", "a1.urdf");
    step("a1.urdf", "mjcf", "b1.xml");
    let a2 = step("b1.xml", "urdf", "a2.urdf");

    assert_eq!(a1, a2);
    assert_eq!(
        fs::read_to_string(root.join("b.xml")).unwrap(),
        fs::read_to_string(root.join("b1.xml")).unwrap()
    );
}

#[test]
fn round_trip_keeps_the_robot_whole() {
    let root = tree("whole", &[("a.urdf", ARM)]);
    convert(&root, &["a.urdf", "--to", "mjcf", "-o", "b.xml"]);
    convert(&root, &["b.xml", "--to", "urdf", "-o", "a1.urdf"]);
    let back = fs::read_to_string(root.join("a1.urdf")).unwrap();

    let orig = rigscan_core::parse_urdf(ARM).unwrap();
    let after = rigscan_core::parse_urdf(&back).unwrap();
    // Names, frames, axes, limits and shapes survive; the MJCF reader adds a `world` link and
    // splits no shape here.
    let names = |r: &rigscan_core::Robot| {
        let mut n: Vec<String> = r.links.iter().map(|l| l.name.clone()).collect();
        n.retain(|n| n != "world");
        n.sort();
        n
    };
    assert_eq!(names(&orig), names(&after));
    let j = |r: &rigscan_core::Robot| r.joints.iter().find(|j| j.name == "shoulder").cloned();
    let (a, b) = (j(&orig).unwrap(), j(&after).unwrap());
    assert_eq!(
        (a.kind, a.origin, a.axis, a.limit),
        (b.kind, b.origin, b.axis, b.limit)
    );
}

#[test]
fn urdf_corpus_converts_to_mjcf_and_back() {
    let corpus = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../corpus/example-robot-data");
    if !corpus.exists() && std::env::var_os("RIGSCAN_REQUIRE_CORPUS").is_none() {
        eprintln!("skipped: corpus/example-robot-data is not present");
        return;
    }
    let root = tree("corpus", &[]);
    let mut converted = 0;
    for entry in ignore::Walk::new(&corpus).flatten() {
        let p = entry.path();
        if p.extension().is_none_or(|e| e != "urdf") {
            continue;
        }
        let out = convert(&root, &[p.to_str().unwrap(), "--to", "mjcf", "-o", "b.xml"]);
        // 2 is a file the linter refuses or that does not parse; 0 and 1 produced text.
        if out.status.code() == Some(2) {
            continue;
        }
        let back = convert(&root, &["b.xml", "--to", "urdf"]);
        assert!(
            // A written file the linter refuses (a joint MJCF left unlimited needs a URDF limit)
            // is a refusal, not a failure to read what was written.
            back.status.code().is_some_and(|c| c < 2) || stderr(&back).contains("not converted"),
            "{}: {}",
            p.display(),
            stderr(&back).lines().next().unwrap_or("")
        );
        converted += 1;
    }
    assert!(converted > 0, "no corpus file converted");
}

#[test]
fn menagerie_converts_to_urdf_and_back() {
    let corpus = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../corpus/mujoco_menagerie");
    if !corpus.exists() && std::env::var_os("RIGSCAN_REQUIRE_CORPUS").is_none() {
        eprintln!("skipped: corpus/mujoco_menagerie is not present");
        return;
    }
    let root = tree("menagerie", &[]);
    let mut converted = 0;
    for entry in ignore::Walk::new(&corpus).flatten() {
        let p = entry.path();
        if p.extension().is_none_or(|e| e != "xml") {
            continue;
        }
        let out = convert(
            &root,
            &[p.to_str().unwrap(), "--to", "urdf", "-o", "b.urdf"],
        );
        if out.status.code() == Some(2) {
            continue;
        }
        let back = convert(&root, &["b.urdf", "--to", "mjcf"]);
        assert!(
            // A written file the linter refuses (a joint MJCF left unlimited needs a URDF limit)
            // is a refusal, not a failure to read what was written.
            back.status.code().is_some_and(|c| c < 2) || stderr(&back).contains("not converted"),
            "{}: {}",
            p.display(),
            stderr(&back).lines().next().unwrap_or("")
        );
        converted += 1;
    }
    eprintln!("converted {converted}");
    assert!(converted > 0, "no corpus file converted");
}

#[test]
fn relative_mesh_paths_are_rewritten_for_an_output_in_another_directory() {
    let urdf = r#"<robot name="m"><link name="a"><visual><geometry><mesh filename="meshes/a.stl"/></geometry></visual></link></robot>"#;
    let root = tree("rebase", &[]);
    fs::create_dir_all(root.join("in/meshes")).unwrap();
    fs::create_dir_all(root.join("out")).unwrap();
    fs::write(root.join("in/m.urdf"), urdf).unwrap();
    fs::write(root.join("in/meshes/a.stl"), "solid").unwrap();

    let out = convert(&root, &["in/m.urdf", "--to", "mjcf", "-o", "out/m.xml"]);

    assert_eq!(out.status.code(), Some(0), "{}", stderr(&out));
    let written = fs::read_to_string(root.join("out/m.xml")).unwrap();
    assert!(written.contains("file=\"../in/meshes/a.stl\""), "{written}");
}

#[test]
fn a_missing_mesh_file_does_not_stop_the_conversion() {
    let urdf = r#"<robot name="m"><link name="a"><visual><geometry><mesh filename="gone.stl"/></geometry></visual></link></robot>"#;
    let root = tree("nomesh", &[("m.urdf", urdf)]);

    let out = convert(&root, &["m.urdf", "--to", "mjcf"]);

    assert_eq!(out.status.code(), Some(0), "{}", stderr(&out));
    assert!(stdout(&out).contains("file=\"gone.stl\""));
}

#[test]
fn output_in_a_missing_directory_exits_two() {
    let root = tree("nodir", &[("arm.urdf", ARM)]);
    let out = convert(&root, &["arm.urdf", "--to", "mjcf", "-o", "gone/arm.xml"]);

    assert_eq!(out.status.code(), Some(2));
    assert!(
        stderr(&out).starts_with("gone/arm.xml:"),
        "{}",
        stderr(&out)
    );
}

#[test]
fn losses_of_one_kind_share_a_line() {
    let two = WITH_EFFORT.replace(
        "</robot>",
        r#"<link name="fore"/>
  <joint name="elbow" type="revolute">
    <parent link="upper"/>
    <child link="fore"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1" upper="1" effort="10" velocity="2"/>
  </joint>
</robot>"#,
    );
    let root = tree("grouped", &[("arm.urdf", &two)]);
    let out = convert(&root, &["arm.urdf", "--to", "mjcf"]);

    assert!(
        stderr(&out).contains("lost limit/effort: shoulder, elbow"),
        "{}",
        stderr(&out)
    );
}

#[test]
fn package_mesh_paths_are_not_rewritten() {
    let urdf = r#"<robot name="m"><link name="a"><visual><geometry><mesh filename="package://p/a.stl"/></geometry></visual></link></robot>"#;
    let root = tree("package", &[]);
    fs::create_dir_all(root.join("in")).unwrap();
    fs::create_dir_all(root.join("out")).unwrap();
    fs::write(root.join("in/m.urdf"), urdf).unwrap();

    convert(&root, &["in/m.urdf", "--to", "mjcf", "-o", "out/m.xml"]);

    let written = fs::read_to_string(root.join("out/m.xml")).unwrap();
    assert!(written.contains("file=\"package://p/a.stl\""), "{written}");
}

#[test]
fn shapes_without_a_mesh_and_absolute_paths_survive_a_move() {
    let urdf = r#"<robot name="m"><link name="a">
      <visual><geometry><box size="1 1 1"/></geometry></visual>
      <visual><geometry><mesh filename="/abs/a.stl"/></geometry></visual>
    </link></robot>"#;
    let root = tree("moved", &[]);
    fs::create_dir_all(root.join("in")).unwrap();
    fs::create_dir_all(root.join("out")).unwrap();
    fs::write(root.join("in/m.urdf"), urdf).unwrap();

    convert(&root, &["in/m.urdf", "--to", "mjcf", "-o", "out/m.xml"]);

    let written = fs::read_to_string(root.join("out/m.xml")).unwrap();
    assert!(written.contains("type=\"box\""), "{written}");
    assert!(written.contains("file=\"/abs/a.stl\""), "{written}");
}

const WITH_MATERIAL: &str = r#"<robot name="arm">
  <link name="base"/>
  <link name="upper">
    <visual>
      <geometry><box size="1 1 1"/></geometry>
      <material name="red"/>
    </visual>
  </link>
  <joint name="shoulder" type="continuous">
    <parent link="base"/>
    <child link="upper"/>
    <axis xyz="0 1 0"/>
  </joint>
</robot>"#;

fn json(path: &Path) -> serde_json::Value {
    serde_json::from_str(&fs::read_to_string(path).unwrap()).unwrap()
}

#[test]
fn each_kind_of_loss_gets_its_meaning_and_a_total() {
    let root = tree("meaning", &[("arm.urdf", WITH_EFFORT)]);
    let out = convert(&root, &["arm.urdf", "--to", "mjcf"]);

    let err = stderr(&out);
    assert!(
        err.contains(
            "lost limit/effort: shoulder\n  MJCF has no effort limit on a joint; it was left out.\n"
        ),
        "{err}"
    );
    assert!(err.contains("lost limit/velocity: shoulder\n  "), "{err}");
    assert!(err.ends_with("2 kinds of loss, 2 elements\n"), "{err}");
}

#[test]
fn a_loss_from_the_reader_shows_its_position_and_a_generic_meaning() {
    let root = tree("reader-loss", &[("arm.urdf", WITH_MATERIAL)]);
    let out = convert(&root, &["arm.urdf", "--to", "mjcf"]);

    let err = stderr(&out);
    assert!(err.contains("lost visual/material: upper (6:7)\n"), "{err}");
    assert!(
        err.contains("  The neutral model has no field for it; it was not carried over.\n"),
        "{err}"
    );
    assert!(err.ends_with("1 kind of loss, 1 element\n"), "{err}");
}

#[test]
fn the_loss_report_is_written_as_json() {
    let root = tree("json", &[("arm.urdf", WITH_MATERIAL)]);
    let out = convert(
        &root,
        &["arm.urdf", "--to", "mjcf", "--loss-report", "loss.json"],
    );
    assert_eq!(out.status.code(), Some(1));

    let r = json(&root.join("loss.json"));
    assert_eq!(r["version"], 1);
    assert_eq!(r["input"], "arm.urdf");
    assert_eq!(r["from"], "urdf");
    assert_eq!(r["to"], "mjcf");
    assert_eq!(r["counts"]["visual/material"], 1);
    let lost = &r["lost"][0];
    assert_eq!(lost["id"], "visual/material");
    assert_eq!(lost["kind"], "dropped");
    assert_eq!(lost["name"], "upper");
    assert_eq!(lost["line"], 6);
    assert_eq!(lost["column"], 7);
    assert!(lost["summary"].as_str().unwrap().contains("neutral model"));
}

#[test]
fn a_loss_without_a_position_has_null_line_and_column() {
    let root = tree("json-nopos", &[("arm.urdf", WITH_EFFORT)]);
    convert(
        &root,
        &["arm.urdf", "--to", "mjcf", "--loss-report", "loss.json"],
    );

    let r = json(&root.join("loss.json"));
    let effort = r["lost"]
        .as_array()
        .unwrap()
        .iter()
        .find(|l| l["id"] == "limit/effort")
        .unwrap();
    assert_eq!(effort["kind"], "dropped");
    assert!(effort["line"].is_null() && effort["column"].is_null());
    assert_eq!(r["counts"]["limit/velocity"], 1);
}

#[test]
fn a_lossless_conversion_writes_an_empty_report() {
    let root = tree("json-clean", &[("arm.urdf", ARM)]);
    let out = convert(
        &root,
        &["arm.urdf", "--to", "mjcf", "--loss-report", "loss.json"],
    );
    assert_eq!(out.status.code(), Some(0));

    let r = json(&root.join("loss.json"));
    assert_eq!(r["lost"].as_array().unwrap().len(), 0);
    assert_eq!(r["counts"].as_object().unwrap().len(), 0);
}

#[test]
fn an_invented_value_is_marked_as_invented() {
    let root = tree("json-invented", &[("arm.xml", RANGED)]);
    convert(
        &root,
        &["arm.xml", "--to", "urdf", "--loss-report", "loss.json"],
    );

    let r = json(&root.join("loss.json"));
    let kinds: Vec<_> = r["lost"]
        .as_array()
        .unwrap()
        .iter()
        .filter(|l| l["id"] == "limit/effort/default")
        .map(|l| l["kind"].as_str().unwrap().to_string())
        .collect();
    assert_eq!(kinds, ["invented"]);
}

#[test]
fn a_refused_conversion_writes_no_report() {
    let root = tree("json-refused", &[("bad.urdf", BROKEN)]);
    let out = convert(
        &root,
        &["bad.urdf", "--to", "mjcf", "--loss-report", "loss.json"],
    );

    assert_eq!(out.status.code(), Some(2));
    assert!(!root.join("loss.json").exists());
}

#[test]
fn a_report_that_cannot_be_written_is_an_error() {
    let root = tree("json-unwritable", &[("arm.urdf", ARM)]);
    let out = convert(
        &root,
        &[
            "arm.urdf",
            "--to",
            "mjcf",
            "--loss-report",
            "missing/loss.json",
        ],
    );

    assert_eq!(out.status.code(), Some(2));
    assert!(
        stderr(&out).contains("missing/loss.json"),
        "{}",
        stderr(&out)
    );
}

const RANGED: &str = r#"<mujoco><worldbody><body name="a">
  <joint name="h" type="hinge" axis="0 0 1" range="-1 1"/>
</body></worldbody></mujoco>"#;

const ROTATED_TENSOR: &str = r#"<robot name="arm">
  <link name="base"/>
  <link name="upper">
    <inertial>
      <origin xyz="0 0 0.1" rpy="0 0 0.5"/>
      <mass value="2"/>
      <inertia ixx="0.1" ixy="0.01" ixz="0" iyy="0.2" iyz="0" izz="0.3"/>
    </inertial>
  </link>
  <joint name="shoulder" type="continuous">
    <parent link="base"/>
    <child link="upper"/>
    <axis xyz="0 1 0"/>
  </joint>
</robot>"#;

#[test]
fn a_rewritten_value_is_marked_as_changed() {
    let root = tree("json-changed", &[("arm.urdf", ROTATED_TENSOR)]);
    convert(
        &root,
        &["arm.urdf", "--to", "mjcf", "--loss-report", "loss.json"],
    );

    let r = json(&root.join("loss.json"));
    assert_eq!(r["lost"][0]["id"], "inertial/frame");
    assert_eq!(r["lost"][0]["kind"], "changed");
}

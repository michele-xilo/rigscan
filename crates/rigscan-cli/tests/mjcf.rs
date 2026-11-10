use std::fs;
use std::path::{Path, PathBuf};
use std::process::{Command, Output};

const HEALTHY: &str = r#"<mujoco model="ok">
  <worldbody>
    <body name="a"><joint type="slide" range="0 1"/></body>
  </worldbody>
</mujoco>"#;

const BROKEN: &str = r#"<mujoco model="bad">
  <worldbody>
    <body name="a"><joint name="j" type="slide" range="1 0"/></body>
  </worldbody>
</mujoco>"#;

fn tree(name: &str, files: &[(&str, &str)]) -> PathBuf {
    let root = Path::new(env!("CARGO_TARGET_TMPDIR")).join(format!("mjcf-{name}"));
    let _ = fs::remove_dir_all(&root);
    for (path, content) in files {
        let p = root.join(path);
        fs::create_dir_all(p.parent().unwrap()).unwrap();
        fs::write(p, content).unwrap();
    }
    root
}

fn run(dir: &Path, args: &[&str]) -> Output {
    Command::new(env!("CARGO_BIN_EXE_rigscan"))
        .current_dir(dir)
        .arg("scan")
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
fn healthy_mjcf_exits_zero_with_no_output() {
    let root = tree("healthy", &[("robot.xml", HEALTHY)]);
    let out = run(&root, &["robot.xml"]);
    assert_eq!(stdout(&out), "");
    assert_eq!(out.status.code(), Some(0), "{}", stderr(&out));
}

#[test]
fn finding_in_mjcf_is_printed_with_its_location_and_exits_one() {
    let root = tree("broken", &[("robot.xml", BROKEN)]);
    let out = run(&root, &["robot.xml"]);
    assert!(
        stdout(&out).starts_with("robot.xml:3:20: critical RS012:"),
        "{}",
        stdout(&out)
    );
    assert_eq!(out.status.code(), Some(1));
}

#[test]
fn includes_are_read_relative_to_the_including_file() {
    let root = tree(
        "include",
        &[
            (
                "scene.xml",
                "<mujoco>\n  <worldbody>\n    <include file=\"sub/arm.xml\"/>\n  </worldbody>\n</mujoco>",
            ),
            ("sub/arm.xml", "<mujoco><include file=\"slider.xml\"/></mujoco>"),
            (
                "sub/slider.xml",
                "<mujoco><body name=\"a\"><joint type=\"slide\" range=\"1 0\"/></body></mujoco>",
            ),
        ],
    );
    let out = run(&root, &["scene.xml"]);
    // The element lives in sub/slider.xml, so it is reported where the scanned file includes it.
    assert!(
        stdout(&out).starts_with("scene.xml:3:5: critical RS012:"),
        "{}",
        stdout(&out)
    );
}

#[test]
fn json_output_names_the_format() {
    let root = tree("json", &[("robot.xml", HEALTHY)]);
    let out = run(&root, &["--output-format", "json", "robot.xml"]);
    assert!(
        stdout(&out).contains("\"format\":\"mjcf\""),
        "{}",
        stdout(&out)
    );
}

#[test]
fn missing_mesh_file_is_reported_using_meshdir() {
    let root = tree(
        "mesh",
        &[(
            "robot.xml",
            "<mujoco>\n  <compiler meshdir=\"assets\"/>\n  <asset><mesh name=\"m\" file=\"m.stl\"/></asset>\n  <worldbody>\n    <body name=\"a\"><geom mesh=\"m\"/></body>\n  </worldbody>\n</mujoco>",
        )],
    );
    let out = run(&root, &["robot.xml"]);
    assert!(
        stdout(&out).contains(
            "robot.xml:5:20: critical RS014: link `a` mesh file `assets/m.stl` was not found"
        ),
        "{}",
        stdout(&out)
    );

    fs::create_dir_all(root.join("assets")).unwrap();
    fs::write(root.join("assets/m.stl"), "").unwrap();
    assert_eq!(run(&root, &["robot.xml"]).status.code(), Some(0));
}

#[test]
fn malformed_mjcf_exits_two() {
    let root = tree(
        "malformed",
        &[("robot.xml", "<mujoco><worldbody></mujoco>")],
    );
    let out = run(&root, &["robot.xml"]);
    assert_eq!(out.status.code(), Some(2));
    assert!(stderr(&out).contains("robot.xml"), "{}", stderr(&out));
}

#[test]
fn directory_scan_takes_mjcf_xml_and_skips_other_xml() {
    let root = tree(
        "dir",
        &[
            ("robot.xml", BROKEN),
            ("package.xml", "<package><name>x</name></package>"),
            ("launch/run.xml", "<launch/>"),
        ],
    );
    let out = run(&root, &["."]);
    assert_eq!(stdout(&out).lines().count(), 1, "{}", stdout(&out));
    assert!(stdout(&out).starts_with("robot.xml:3:20:"));
    assert!(!stderr(&out).contains("package.xml"), "{}", stderr(&out));
    assert_eq!(out.status.code(), Some(1));
}

#[test]
fn explicit_xml_that_is_not_mjcf_is_an_unsupported_file() {
    let root = tree("notmjcf", &[("package.xml", "<package/>")]);
    let out = run(&root, &["package.xml"]);
    assert_eq!(out.status.code(), Some(2));
    assert!(
        stderr(&out).contains("unsupported file type"),
        "{}",
        stderr(&out)
    );
}

#[test]
fn mjcf_rules_run_in_the_cli() {
    let dir = tree(
        "rules",
        &[(
            "robot.xml",
            "<mujoco>\n  <include file=\"gone.xml\"/>\n  <worldbody>\n    <body childclass=\"nope\"><geom type=\"mesh\" mesh=\"ghost\"/></body>\n  </worldbody>\n</mujoco>",
        )],
    );
    let o = run(&dir, &["robot.xml"]);
    let out = stdout(&o);
    assert!(out.contains("robot.xml:2:3: critical RS018"), "{out}");
    assert!(out.contains("robot.xml:4:5: critical RS016"), "{out}");
    assert!(out.contains("robot.xml:4:29: critical RS017"), "{out}");
    assert_eq!(o.status.code(), Some(1));
}

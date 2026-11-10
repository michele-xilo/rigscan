use std::fs;
use std::path::{Path, PathBuf};
use std::process::{Command, Output};

const ARM: &str = r#"#usda 1.0
(
    defaultPrim = "r"
)
def Xform "r" (prepend apiSchemas = ["PhysicsArticulationRootAPI"])
{
    def Xform "base" (prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"])
    {
        float physics:mass = 2
    }
    def Xform "arm" (prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"])
    {
        float physics:mass = 1
    }
    def PhysicsRevoluteJoint "shoulder"
    {
        rel physics:body0 = </r/base>
        rel physics:body1 = </r/arm>
        float physics:lowerLimit = LOWER
        float physics:upperLimit = UPPER
    }
}
"#;

fn arm(lower: &str, upper: &str) -> String {
    ARM.replace("LOWER", lower).replace("UPPER", upper)
}

fn tree(name: &str, files: &[(&str, &str)]) -> PathBuf {
    let root = Path::new(env!("CARGO_TARGET_TMPDIR")).join(format!("usd-{name}"));
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
fn healthy_usda_exits_zero_with_no_output() {
    let root = tree("healthy", &[("robot.usda", &arm("-90", "45"))]);
    let out = run(&root, &["robot.usda"]);
    assert_eq!(stdout(&out), "");
    assert_eq!(out.status.code(), Some(0), "{}", stderr(&out));
}

#[test]
fn finding_in_usd_is_printed_without_a_position() {
    let root = tree("broken", &[("robot.usda", &arm("45", "-90"))]);
    let out = run(&root, &["robot.usda"]);
    assert!(
        stdout(&out).starts_with("robot.usda: critical RS012:"),
        "{}",
        stdout(&out)
    );
    assert_eq!(out.status.code(), Some(1));
}

#[test]
fn full_output_has_no_source_line_for_usd() {
    let root = tree("full", &[("robot.usda", &arm("45", "-90"))]);
    let out = run(&root, &["--output-format", "full", "robot.usda"]);
    assert!(
        stdout(&out).contains("--> robot.usda\n"),
        "{}",
        stdout(&out)
    );
    assert!(!stdout(&out).contains(" | "), "{}", stdout(&out));
}

#[test]
fn json_names_the_format_and_has_null_positions() {
    let root = tree("json", &[("robot.usda", &arm("45", "-90"))]);
    let out = run(&root, &["--output-format", "json", "robot.usda"]);
    let s = stdout(&out);
    assert!(s.contains("\"format\":\"usd\""), "{s}");
    assert!(s.contains("\"line\":null,\"message\""), "{s}");
}

#[test]
fn binary_usdc_is_read() {
    let root = tree("usdc", &[]);
    fs::create_dir_all(&root).unwrap();
    fs::copy(
        Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/arm.usdc"),
        root.join("arm.usdc"),
    )
    .unwrap();
    let out = run(&root, &["arm.usdc"]);
    assert_eq!(stdout(&out), "");
    assert_eq!(out.status.code(), Some(0), "{}", stderr(&out));
}

#[test]
fn malformed_usda_exits_two() {
    let root = tree(
        "bad",
        &[(
            "robot.usda",
            "#usda 1.0\ndef Xform \"a\" {\n  float = = \n}\n",
        )],
    );
    let out = run(&root, &["robot.usda"]);
    assert_eq!(out.status.code(), Some(2));
    assert!(stderr(&out).contains("robot.usda"), "{}", stderr(&out));
}

#[test]
fn directory_scan_skips_a_layer_that_another_file_references() {
    let root = tree(
        "layers",
        &[
            (
                "scene.usda",
                "#usda 1.0\ndef \"w\" (references = @parts/arm.usda@)\n{\n}\n",
            ),
            ("parts/arm.usda", &arm("45", "-90")),
        ],
    );
    let out = run(&root, &["."]);
    // The finding is in the referenced layer; it is reported once, for the scene.
    let lines: Vec<String> = stdout(&out).lines().map(String::from).collect();
    assert_eq!(lines.len(), 1, "{lines:?}");
    assert!(
        lines[0].starts_with("scene.usda: critical RS012:"),
        "{lines:?}"
    );
}

#[test]
fn usd_files_are_found_by_a_directory_scan() {
    let root = tree(
        "dir",
        &[
            ("a.usda", &arm("-90", "45")),
            ("sub/b.usda", &arm("1", "0")),
        ],
    );
    let out = run(&root, &["."]);
    assert_eq!(stdout(&out).lines().count(), 1, "{}", stdout(&out));
    assert!(stdout(&out).starts_with("sub/b.usda:"), "{}", stdout(&out));
}

#[test]
fn usd_rules_run_in_the_cli() {
    let root = tree(
        "rules",
        &[(
            "robot.usda",
            r#"#usda 1.0
def Xform "a" (prepend apiSchemas = ["PhysicsRigidBodyAPI"]) {}
def PhysicsFixedJoint "j"
{
    rel physics:body0 = </a>
    rel physics:body1 = </a>
}
def "w" (references = @gone.usda@)
{
}
"#,
        )],
    );
    let out = run(&root, &["robot.usda"]);
    let s = stdout(&out);
    assert!(
        s.contains("robot.usda: critical RS019: joint `/j` connects `/a` to itself"),
        "{s}"
    );
    assert!(s.contains("robot.usda: warning RS020:"), "{s}");
    assert!(s.contains("robot.usda: warning RS021:"), "{s}");
    assert!(s.contains("robot.usda: critical RS022:"), "{s}");
    assert_eq!(out.status.code(), Some(1));
}

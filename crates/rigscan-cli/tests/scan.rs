use std::path::PathBuf;
use std::process::{Command, Output};

fn fixture(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures")
        .join(name)
}

fn scan(files: &[&str]) -> Output {
    Command::new(env!("CARGO_BIN_EXE_rigscan"))
        .arg("scan")
        .args(files.iter().map(|f| fixture(f)))
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
fn healthy_file_exits_zero_with_no_output() {
    let out = scan(&["healthy.urdf"]);

    assert_eq!(out.status.code(), Some(0));
    assert_eq!(stdout(&out), "");
}

#[test]
fn critical_finding_is_printed_with_location_and_exits_one() {
    let out = scan(&["broken.urdf"]);

    let path = fixture("broken.urdf");
    let expected = format!(
        "{}:3:3: critical RS001: joint `j1` child link `ghost` is not defined\n",
        path.display()
    );
    assert_eq!(stdout(&out), expected);
    assert_eq!(out.status.code(), Some(1));
}

#[test]
fn several_files_report_all_findings() {
    let out = scan(&["healthy.urdf", "broken.urdf"]);

    assert_eq!(stdout(&out).lines().count(), 1);
    assert_eq!(out.status.code(), Some(1));
}

#[test]
fn unsupported_extension_exits_two() {
    let out = scan(&["robot.txt"]);

    assert_eq!(out.status.code(), Some(2));
    assert!(stderr(&out).contains("unsupported file type"));
    assert!(stderr(&out).contains("robot.txt"));
}

#[test]
fn missing_file_exits_two_and_names_the_path() {
    let out = scan(&["does-not-exist.urdf"]);

    assert_eq!(out.status.code(), Some(2));
    assert!(stderr(&out).contains("does-not-exist.urdf"));
}

#[test]
fn malformed_xml_exits_two_and_names_the_path() {
    let out = scan(&["malformed.urdf"]);

    assert_eq!(out.status.code(), Some(2));
    assert!(stderr(&out).contains("malformed.urdf"));
}

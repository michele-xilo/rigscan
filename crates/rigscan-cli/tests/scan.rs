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

#[test]
fn file_without_robot_element_exits_two() {
    let out = scan(&["not-a-robot.urdf"]);

    assert_eq!(out.status.code(), Some(2));
    assert!(stderr(&out).contains("not-a-robot.urdf"));
}

fn scan_as(format: &str, files: &[&str]) -> Output {
    Command::new(env!("CARGO_BIN_EXE_rigscan"))
        .args(["scan", "--output-format", format])
        .args(files.iter().map(|f| fixture(f)))
        .output()
        .unwrap()
}

#[test]
fn report_format_lists_every_element_under_a_verdict_line() {
    let out = scan_as("report", &["healthy.urdf"]);

    let path = fixture("healthy.urdf");
    let expected = format!(
        "{}: 3 healthy, 0 warnings, 0 critical\n  healthy  link  base\n  healthy  link  arm\n  healthy  joint j1\n",
        path.display()
    );
    assert_eq!(stdout(&out), expected);
    assert_eq!(out.status.code(), Some(0));
}

#[test]
fn report_format_shows_findings_under_the_unhealthy_element() {
    let out = scan_as("report", &["broken.urdf"]);

    let path = fixture("broken.urdf").display().to_string();
    let expected = format!(
        "{path}: 1 healthy, 0 warnings, 1 critical\n  healthy  link  base\n  critical joint j1\n    {path}:3:3: critical RS001: joint `j1` child link `ghost` is not defined\n"
    );
    assert_eq!(stdout(&out), expected);
    assert_eq!(out.status.code(), Some(1));
}

#[test]
fn json_format_is_one_versioned_document() {
    let out = scan_as("json", &["healthy.urdf", "broken.urdf"]);

    let doc: serde_json::Value = serde_json::from_str(&stdout(&out)).unwrap();
    assert_eq!(doc["version"], 1);
    assert_eq!(doc["summary"]["healthy"], 4);
    assert_eq!(doc["summary"]["critical"], 1);
    let broken = &doc["files"][1];
    assert_eq!(broken["format"], "urdf");
    assert_eq!(broken["robot"], "broken");
    assert_eq!(broken["verdict"]["critical"], 1);
    assert_eq!(broken["elements"][1]["kind"], "joint");
    assert_eq!(broken["elements"][1]["name"], "j1");
    assert_eq!(broken["elements"][1]["status"], "critical");
    let finding = &broken["findings"][0];
    assert_eq!(finding["rule"], "RS001");
    assert_eq!(finding["severity"], "critical");
    assert_eq!(finding["line"], 3);
    assert_eq!(finding["column"], 3);
    assert_eq!(finding["element"]["kind"], "joint");
    assert_eq!(finding["element"]["name"], "j1");
    assert_eq!(out.status.code(), Some(1));
}

#[test]
fn json_format_reports_a_file_that_cannot_be_scanned() {
    let out = scan_as("json", &["malformed.urdf"]);

    let doc: serde_json::Value = serde_json::from_str(&stdout(&out)).unwrap();
    let file = &doc["files"][0];
    assert!(file["path"].as_str().unwrap().ends_with("malformed.urdf"));
    assert!(!file["error"].as_str().unwrap().is_empty());
    assert!(file.get("verdict").is_none());
    assert_eq!(out.status.code(), Some(2));
}

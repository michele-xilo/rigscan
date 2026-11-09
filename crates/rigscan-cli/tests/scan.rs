use std::path::{Path, PathBuf};
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

fn scan_with(extra: &[&str], files: &[&str], envs: &[(&str, &str)]) -> Output {
    Command::new(env!("CARGO_BIN_EXE_rigscan"))
        .arg("scan")
        .args(extra)
        .args(files.iter().map(|f| fixture(f)))
        .envs(envs.iter().copied())
        .output()
        .unwrap()
}

#[test]
fn clean_scan_says_all_checks_passed_on_stderr() {
    let out = scan_with(&[], &["healthy.urdf"], &[]);

    assert_eq!(stderr(&out), "All checks passed!\n");
    assert_eq!(stdout(&out), "");
}

#[test]
fn summary_counts_findings_and_files_on_stderr_and_leaves_stdout_alone() {
    let one = scan_with(&[], &["broken.urdf"], &[]);
    let two = scan_with(&[], &["broken.urdf", "healthy.urdf", "broken.urdf"], &[]);

    assert_eq!(stderr(&one), "Found 1 finding in 1 file.\n");
    assert_eq!(stderr(&two), "Found 2 findings in 2 files.\n");
    assert_eq!(stdout(&one).lines().count(), 1);
}

#[test]
fn unscannable_file_is_counted_and_blocks_the_all_clear() {
    let out = scan_with(&[], &["healthy.urdf", "malformed.urdf"], &[]);

    let err = stderr(&out);
    assert!(err.contains("1 file could not be scanned."), "{err}");
    assert!(!err.contains("All checks passed"), "{err}");
    assert_eq!(out.status.code(), Some(2));
}

#[test]
fn report_and_json_formats_print_no_summary() {
    for format in ["report", "json"] {
        let out = scan_with(&["--output-format", format], &["healthy.urdf"], &[]);

        assert_eq!(stderr(&out), "", "{format}");
    }
}

#[test]
fn quiet_hides_findings_and_prints_the_summary_only_when_something_is_wrong() {
    let broken = scan_with(&["--quiet"], &["broken.urdf"], &[]);
    let healthy = scan_with(&["-q"], &["healthy.urdf"], &[]);

    assert_eq!(stdout(&broken), "");
    assert_eq!(stderr(&broken), "Found 1 finding in 1 file.\n");
    assert_eq!(broken.status.code(), Some(1));
    assert_eq!(
        (stdout(&healthy), stderr(&healthy)),
        (String::new(), String::new())
    );
}

#[test]
fn colour_is_off_when_piped_and_on_with_always() {
    let piped = scan_with(&[], &["broken.urdf"], &[]);
    let always = scan_with(&["--color", "always"], &["broken.urdf"], &[]);
    let clean = scan_with(&["--color", "always"], &["healthy.urdf"], &[]);

    assert!(!stdout(&piped).contains('\x1b') && !stderr(&piped).contains('\x1b'));
    assert!(
        stdout(&always).contains("\x1b[31mcritical\x1b[0m"),
        "{}",
        stdout(&always)
    );
    assert_eq!(stderr(&clean), "\x1b[36mAll checks passed!\x1b[0m\n");
}

#[test]
fn no_color_variable_beats_auto_but_not_always() {
    let auto = scan_with(&["--color", "auto"], &["broken.urdf"], &[("NO_COLOR", "1")]);
    let always = scan_with(
        &["--color", "always"],
        &["broken.urdf"],
        &[("NO_COLOR", "1")],
    );

    assert!(!stdout(&auto).contains('\x1b'));
    assert!(stdout(&always).contains('\x1b'));
}

#[test]
fn report_format_colours_the_status_column() {
    let out = scan_with(
        &["--output-format", "report", "--color", "always"],
        &["broken.urdf"],
        &[],
    );

    let text = stdout(&out);
    assert!(
        text.contains("\x1b[36mhealthy\x1b[0m  link  base"),
        "{text}"
    );
    assert!(text.contains("\x1b[31mcritical\x1b[0m joint j1"), "{text}");
}

fn temp_file(name: &str, content: &str) -> PathBuf {
    let dir = PathBuf::from(env!("CARGO_TARGET_TMPDIR"));
    let path = dir.join(name);
    std::fs::write(&path, content).unwrap();
    path
}

fn scan_paths(extra: &[&str], paths: &[&Path]) -> Output {
    Command::new(env!("CARGO_BIN_EXE_rigscan"))
        .arg("scan")
        .args(extra)
        .args(paths)
        .output()
        .unwrap()
}

#[test]
fn full_format_shows_the_source_line_with_a_caret_under_the_column() {
    let out = scan_with(&["--output-format", "full"], &["broken.urdf"], &[]);

    let path = fixture("broken.urdf").display().to_string();
    let expected = format!(
        "critical[RS001]: joint `j1` child link `ghost` is not defined\n --> {path}:3:3\n  |\n3 |   <joint name=\"j1\" type=\"fixed\">\n  |   ^\n\n"
    );
    assert_eq!(stdout(&out), expected);
    assert_eq!(stderr(&out), "Found 1 finding in 1 file.\n");
    assert_eq!(out.status.code(), Some(1));
}

#[test]
fn caret_lines_up_after_a_tab_and_counts_characters_not_bytes() {
    let tabbed = temp_file(
        "tab.urdf",
        "<robot name=\"t\">\n\t<link name=\"a\"/>\n\t<joint name=\"j\" type=\"fixed\"><parent link=\"a\"/><child link=\"g\"/></joint>\n</robot>",
    );
    let accent = temp_file(
        "accent.urdf",
        "<robot name=\"t\">\n<link name=\"é\"/><joint name=\"j\" type=\"fixed\"><parent link=\"é\"/><child link=\"g\"/></joint>\n</robot>",
    );

    let t = scan_paths(&["--output-format", "full"], &[&tabbed]);
    let a = scan_paths(&["--output-format", "full"], &[&accent]);

    assert!(stdout(&t).ends_with("\n  | \t^\n\n"), "{:?}", stdout(&t));
    let expected = format!("\n  | {}^\n\n", " ".repeat(16));
    assert!(stdout(&a).ends_with(&expected), "{:?}", stdout(&a));
}

#[test]
fn full_format_colours_the_header_and_the_caret() {
    let out = scan_with(
        &["--output-format", "full", "--color", "always"],
        &["broken.urdf"],
        &[],
    );

    let text = stdout(&out);
    assert!(
        text.starts_with("\x1b[31mcritical[RS001]\x1b[0m: joint"),
        "{text:?}"
    );
    assert!(text.contains("  |   \x1b[31m^\x1b[0m\n"), "{text:?}");
}

#[test]
fn quiet_applies_to_the_full_format_too() {
    let out = scan_with(&["--output-format", "full", "-q"], &["broken.urdf"], &[]);

    assert_eq!(stdout(&out), "");
    assert_eq!(stderr(&out), "Found 1 finding in 1 file.\n");
}

#[test]
fn statistics_count_findings_per_rule_across_files_most_frequent_first() {
    let dup = temp_file(
        "dup.urdf",
        r#"<robot name="d"><link name="a"/><link name="a"/></robot>"#,
    );
    let broken = fixture("broken.urdf");
    let other = fixture("broken.urdf");

    let out = scan_paths(&["--statistics"], &[&dup, &broken, &other]);

    assert_eq!(
        stdout(&out),
        "    2  RS001  critical\n    1  RS002  critical\n    1  RS006  critical\n"
    );
    assert_eq!(out.status.code(), Some(1));
}

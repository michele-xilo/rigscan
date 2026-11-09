use std::fs;
use std::path::{Path, PathBuf};
use std::process::{Command, Output};

const HEALTHY: &str = r#"<robot name="ok"><link name="a"/></robot>"#;
const BROKEN: &str = r#"<robot name="bad"><link name="a"/>
<joint name="j" type="fixed"><parent link="a"/><child link="ghost"/></joint></robot>"#;

fn tree(name: &str, files: &[(&str, &str)]) -> PathBuf {
    let root = Path::new(env!("CARGO_TARGET_TMPDIR")).join(name);
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
fn directory_is_scanned_recursively_for_supported_files() {
    let root = tree(
        "recursive",
        &[
            ("a/ok.urdf", HEALTHY),
            ("a/b/broken.urdf", BROKEN),
            ("notes.txt", "not a robot"),
        ],
    );

    let out = run(&root, &["."]);

    assert_eq!(stdout(&out).lines().count(), 1, "{}", stdout(&out));
    assert!(stdout(&out).starts_with("a/b/broken.urdf:2:1: critical RS001"));
    assert_eq!(out.status.code(), Some(1));
}

#[test]
fn no_argument_scans_the_current_directory() {
    let root = tree("no-arg", &[("broken.urdf", BROKEN)]);

    let out = run(&root, &[]);

    assert!(
        stdout(&out).starts_with("broken.urdf:2:1: critical RS001"),
        "{}",
        stdout(&out)
    );
    assert_eq!(out.status.code(), Some(1));
}

#[test]
fn gitignored_files_are_skipped_even_outside_a_git_repository() {
    let root = tree(
        "gitignore",
        &[
            (".gitignore", "ignored/\n"),
            ("ignored/broken.urdf", BROKEN),
            ("ok.urdf", HEALTHY),
        ],
    );

    let out = run(&root, &["."]);

    assert_eq!(stdout(&out), "");
    assert_eq!(out.status.code(), Some(0));
}

#[test]
fn directory_without_supported_files_warns_and_exits_zero() {
    let root = tree("empty", &[("notes.txt", "x")]);

    let out = run(&root, &["."]);

    assert!(
        stderr(&out).contains("no supported files"),
        "{}",
        stderr(&out)
    );
    assert_eq!(out.status.code(), Some(0));
}

#[test]
fn findings_come_out_sorted_by_path() {
    let root = tree(
        "sorted",
        &[("z.urdf", BROKEN), ("a.urdf", BROKEN), ("m/b.urdf", BROKEN)],
    );

    let out = run(&root, &["."]);

    let paths: Vec<String> = stdout(&out)
        .lines()
        .map(|l| l.split(':').next().unwrap().to_string())
        .collect();
    assert_eq!(paths, ["a.urdf", "m/b.urdf", "z.urdf"]);
}

#[test]
fn parallel_and_single_thread_runs_print_the_same_bytes() {
    let files: Vec<(String, &str)> = (0..40)
        .map(|i| (format!("d{}/r{i}.urdf", i % 7), BROKEN))
        .collect();
    let refs: Vec<(&str, &str)> = files.iter().map(|(p, c)| (p.as_str(), *c)).collect();
    let root = tree("determinism", &refs);

    let parallel = run(&root, &["."]);
    let serial = Command::new(env!("CARGO_BIN_EXE_rigscan"))
        .current_dir(&root)
        .env("RAYON_NUM_THREADS", "1")
        .args(["scan", "."])
        .output()
        .unwrap();

    assert_eq!(stdout(&parallel).lines().count(), 40);
    assert_eq!(stdout(&parallel), stdout(&serial));
}

use std::fs;
use std::path::{Path, PathBuf};
use std::process::{Command, Output};

fn robot(mesh: &str) -> String {
    format!(
        "<robot name=\"r\"><link name=\"a\"><visual><geometry><mesh filename=\"{mesh}\"/></geometry></visual></link></robot>"
    )
}

fn tree(name: &str, files: &[(&str, String)]) -> PathBuf {
    let root = Path::new(env!("CARGO_TARGET_TMPDIR")).join(name);
    let _ = fs::remove_dir_all(&root);
    for (path, content) in files {
        let p = root.join(path);
        fs::create_dir_all(p.parent().unwrap()).unwrap();
        fs::write(p, content).unwrap();
    }
    root
}

fn scan(root: &Path, file: &str) -> Output {
    Command::new(env!("CARGO_BIN_EXE_rigscan"))
        .current_dir(root)
        .args(["scan", file])
        .output()
        .unwrap()
}

fn stdout(o: &Output) -> String {
    String::from_utf8_lossy(&o.stdout).into_owned()
}

#[test]
fn relative_mesh_that_exists_is_clean() {
    let root = tree(
        "mesh_rel_ok",
        &[
            ("r.urdf", robot("meshes/a.stl")),
            ("meshes/a.stl", String::new()),
        ],
    );

    let out = scan(&root, "r.urdf");

    assert_eq!(out.status.code(), Some(0), "{}", stdout(&out));
}

#[test]
fn relative_mesh_that_is_missing_is_reported_and_exits_one() {
    let root = tree("mesh_rel_missing", &[("r.urdf", robot("meshes/a.stl"))]);

    let out = scan(&root, "r.urdf");

    assert_eq!(out.status.code(), Some(1), "{}", stdout(&out));
    assert!(stdout(&out).contains("RS014"), "{}", stdout(&out));
}

#[test]
fn package_mesh_resolves_against_the_ancestor_named_like_the_package() {
    let root = tree(
        "mesh_pkg_ok",
        &[
            (
                "ws/mypkg/urdf/r.urdf",
                robot("package://mypkg/meshes/a.stl"),
            ),
            ("ws/mypkg/meshes/a.stl", String::new()),
        ],
    );

    let out = scan(&root, "ws/mypkg/urdf/r.urdf");

    assert_eq!(out.status.code(), Some(0), "{}", stdout(&out));
}

#[test]
fn package_mesh_missing_inside_a_found_package_is_reported() {
    let root = tree(
        "mesh_pkg_missing",
        &[(
            "ws/mypkg/urdf/r.urdf",
            robot("package://mypkg/meshes/a.stl"),
        )],
    );

    let out = scan(&root, "ws/mypkg/urdf/r.urdf");

    assert_eq!(out.status.code(), Some(1), "{}", stdout(&out));
    assert!(stdout(&out).contains("RS014"), "{}", stdout(&out));
}

#[test]
fn package_that_cannot_be_located_is_not_reported() {
    let root = tree(
        "mesh_pkg_unknown",
        &[("r.urdf", robot("package://elsewhere/meshes/a.stl"))],
    );

    let out = scan(&root, "r.urdf");

    assert_eq!(out.status.code(), Some(0), "{}", stdout(&out));
}

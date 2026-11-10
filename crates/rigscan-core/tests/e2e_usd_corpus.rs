//! End-to-end tests on newton-assets. The corpus is fetched by `scripts/fetch-usd-corpus.sh`.
//!
//! Only the stages no other file pulls in are linted: payloads, mesh libraries and props are
//! fragments of a robot, not robots on their own.

use rigscan_core::{lint_usd, parse_usd, UsdLoader};
use std::cell::RefCell;
use std::collections::{BTreeMap, HashSet};
use std::path::{Path, PathBuf};
use std::rc::Rc;

/// Findings the corpus is known to produce: file, rule, number of findings. The test fails if one
/// stops being reported or a new one appears. All are warnings.
const KNOWN_FINDINGS: [(&str, &str, usize); 3] = [
    // Joints a body to the world; the body itself carries no mass.
    (
        "apptronik_apollo/usd_structured/apptronik_apollo.usda",
        "RS020",
        1,
    ),
    // Seven ankle and imu bodies have neither mass nor a collider in this variant.
    (
        "unitree_g1/usd/g1_collision_geom_simplified_bigger_offset.usd",
        "RS020",
        7,
    ),
    // The articulation root is deleted and prepended again in one list op, which the openusd crate
    // applies in the wrong order.
    ("unitree_h1/usd/h1_minimal.usda", "RS021", 1),
];

fn corpus_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../corpus/newton_assets")
}

fn collect(dir: &Path, out: &mut Vec<PathBuf>) {
    for entry in std::fs::read_dir(dir).unwrap().flatten() {
        let p = entry.path();
        if p.file_name().is_some_and(|n| n == ".git") {
            continue;
        }
        if p.is_dir() {
            collect(&p, out);
        } else if p
            .extension()
            .is_some_and(|e| e == "usd" || e == "usda" || e == "usdc")
        {
            out.push(p);
        }
    }
}

/// None when the corpus is absent; RIGSCAN_REQUIRE_CORPUS=1 turns that into a failure so CI
/// cannot pass silently without it.
fn usd_files() -> Option<Vec<PathBuf>> {
    let dir = corpus_dir();
    if !dir.exists() {
        assert!(
            std::env::var_os("RIGSCAN_REQUIRE_CORPUS").is_none(),
            "corpus required but missing at {}",
            dir.display()
        );
        eprintln!(
            "SKIPPED: corpus not found at {}; run scripts/fetch-usd-corpus.sh",
            dir.display()
        );
        return None;
    }
    let mut paths = Vec::new();
    collect(&dir, &mut paths);
    paths.sort();
    Some(paths)
}

/// Reads layers from disk and remembers which were read.
struct DiskLoader(Rc<RefCell<HashSet<PathBuf>>>);

impl UsdLoader for DiskLoader {
    fn load(&self, path: &str) -> Option<Vec<u8>> {
        let bytes = std::fs::read(path).ok()?;
        let canonical = std::fs::canonicalize(path).ok()?;
        self.0.borrow_mut().insert(canonical);
        Some(bytes)
    }
}

#[test]
fn newton_assets_stages_load_with_only_known_findings() {
    let Some(files) = usd_files() else { return };
    let mut pulled_in = HashSet::new();
    let mut parsed = Vec::new();
    for path in &files {
        let read = Rc::new(RefCell::new(HashSet::new()));
        let usd = parse_usd(
            &path.canonicalize().unwrap().display().to_string(),
            DiskLoader(read.clone()),
        )
        .unwrap_or_else(|e| panic!("{}: {}", path.display(), e.0));
        let own = path.canonicalize().unwrap();
        pulled_in.extend(read.borrow().iter().filter(|p| **p != own).cloned());
        parsed.push((path, usd));
    }

    let stages: Vec<_> = parsed
        .iter()
        .filter(|(p, _)| !pulled_in.contains(&p.canonicalize().unwrap()))
        .collect();
    assert!(
        stages.len() >= 30,
        "only {} stages found; is the corpus complete?",
        stages.len()
    );

    let mut found: BTreeMap<(String, &str), usize> = BTreeMap::new();
    for (path, usd) in &stages {
        let robot = usd
            .to_robot()
            .unwrap_or_else(|e| panic!("{}: {}", path.display(), e.0));
        for f in lint_usd(usd, &robot) {
            let name = path
                .strip_prefix(corpus_dir())
                .unwrap()
                .display()
                .to_string();
            *found.entry((name, f.rule)).or_default() += 1;
        }
    }

    let known: BTreeMap<(String, &str), usize> = KNOWN_FINDINGS
        .iter()
        .map(|(f, r, n)| ((f.to_string(), *r), *n))
        .collect();
    assert_eq!(found, known, "findings differ from KNOWN_FINDINGS");
    eprintln!(
        "{} USD files, {} stages, {} fragments; corpus {}",
        files.len(),
        stages.len(),
        files.len() - stages.len(),
        std::fs::read_to_string(corpus_dir().join("SOURCE"))
            .map_or("unknown".into(), |s| s.trim_end().replace('\n', "@"))
    );
}

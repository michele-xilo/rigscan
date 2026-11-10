//! End-to-end tests on MuJoCo Menagerie. The corpus is fetched by `scripts/fetch-mjcf-corpus.sh`.
//!
//! Only the models are linted: Menagerie files that other files `<include>` are fragments (assets,
//! actuators, keyframes) and are not valid robots on their own.

use rigscan_core::{lint_mjcf, parse_mjcf, IncludeLoader, Included, Resolution, Resolver};
use std::cell::RefCell;
use std::collections::{BTreeMap, HashSet};
use std::path::{Component, Path, PathBuf};

/// Real problems in upstream files: file, rule, number of findings. The test fails if one stops
/// being reported or a new one appears. Both are keyframe-only files that no other file includes.
const KNOWN_FINDINGS: [(&str, &str, usize); 2] = [
    ("aloha/keyframe_no_act.xml", "RS007", 1),
    ("shadow_hand/keyframes.xml", "RS007", 1),
];

fn corpus_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../corpus/mujoco_menagerie")
}

fn collect(dir: &Path, out: &mut Vec<PathBuf>) {
    for entry in std::fs::read_dir(dir).unwrap().flatten() {
        let p = entry.path();
        if p.is_dir() {
            collect(&p, out);
        } else if p.extension().is_some_and(|e| e == "xml") {
            out.push(p);
        }
    }
}

/// None when the corpus is absent; RIGSCAN_REQUIRE_CORPUS=1 turns that into a failure so CI
/// cannot pass silently without it.
fn mjcf_files() -> Option<Vec<(PathBuf, String)>> {
    let dir = corpus_dir();
    if !dir.exists() {
        assert!(
            std::env::var_os("RIGSCAN_REQUIRE_CORPUS").is_none(),
            "corpus required but missing at {}",
            dir.display()
        );
        eprintln!(
            "SKIPPED: corpus not found at {}; run scripts/fetch-mjcf-corpus.sh",
            dir.display()
        );
        return None;
    }
    let mut paths = Vec::new();
    collect(&dir, &mut paths);
    paths.sort();
    Some(
        paths
            .into_iter()
            .map(|p| (p.clone(), std::fs::read_to_string(&p).unwrap()))
            .filter(|(_, src)| src.contains("<mujoco"))
            .collect(),
    )
}

/// `a/../b` to `b` without touching the file system, so the same file is one entry however it is reached.
fn normalize(p: &Path) -> PathBuf {
    let mut out = PathBuf::new();
    for c in p.components() {
        match c {
            Component::ParentDir => {
                out.pop();
            }
            Component::CurDir => {}
            c => out.push(c),
        }
    }
    out
}

/// Reads includes from disk, relative to the including file, and remembers which were read.
struct DiskLoader {
    main: PathBuf,
    included: RefCell<HashSet<PathBuf>>,
}

impl IncludeLoader for DiskLoader {
    fn load(&self, file: &str, from: Option<&str>) -> Option<Included> {
        let base = from.map_or(&*self.main, |f| Path::new(f).parent().unwrap());
        let path = normalize(&base.join(file));
        let text = std::fs::read_to_string(&path).ok()?;
        self.included.borrow_mut().insert(path.clone());
        Some(Included {
            path: path.display().to_string(),
            text,
        })
    }
}

struct Unknown;

impl Resolver for Unknown {
    fn resolve(&self, _uri: &str) -> Resolution {
        Resolution::Unknown
    }
}

#[test]
fn menagerie_models_load_with_only_known_findings() {
    let Some(files) = mjcf_files() else { return };
    let mut included = HashSet::new();
    let mut parsed = Vec::new();
    for (path, src) in &files {
        let loader = DiskLoader {
            main: path.parent().unwrap().to_path_buf(),
            included: RefCell::default(),
        };
        let mjcf =
            parse_mjcf(src, &loader).unwrap_or_else(|e| panic!("{}: {}", path.display(), e.0));
        included.extend(loader.included.into_inner());
        parsed.push((path, mjcf));
    }

    let models: Vec<_> = parsed
        .iter()
        .filter(|(p, _)| !included.contains(&normalize(p)))
        .collect();
    assert!(
        models.len() >= 100,
        "only {} models found; is the corpus complete?",
        models.len()
    );

    let mut found: BTreeMap<(String, &str), usize> = BTreeMap::new();
    for (path, mjcf) in &models {
        let robot = mjcf
            .to_robot()
            .unwrap_or_else(|e| panic!("{}: {}", path.display(), e.0));
        for f in lint_mjcf(mjcf, &robot, &Unknown) {
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
        "{} files with a <mujoco> root, {} models, {} fragments; corpus {}",
        files.len(),
        models.len(),
        files.len() - models.len(),
        std::fs::read_to_string(corpus_dir().join("SOURCE"))
            .map_or("unknown".into(), |s| s.trim_end().replace('\n', "@"))
    );
}

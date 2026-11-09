//! End-to-end tests on real robots. The corpus is fetched by `scripts/fetch-corpus.sh`.
//!
//! Run with `cargo test --release -p rigscan-core --test e2e_corpus -- --nocapture` to see the
//! performance report; debug-build timings are not meaningful.

use rigscan_core::{lint, parse_urdf, Robot, Span};
use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::{Duration, Instant};

const REPS: usize = 5;
const CASES_PER_MODE: usize = 25;
/// Real problems in upstream files; the test fails if one stops being reported.
const KNOWN_FINDINGS: [(&str, &str); 1] = [("ur_description/urdf/ur3.urdf", "RS007")];
const DEFAULT_SEED: u64 = 0x5249_4753_4341_4E31;

struct Corpus {
    files: Vec<(PathBuf, String)>,
}

fn corpus_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../corpus/example-robot-data")
}

fn collect(dir: &Path, out: &mut Vec<PathBuf>) {
    for entry in std::fs::read_dir(dir).unwrap().flatten() {
        let p = entry.path();
        if p.is_dir() {
            collect(&p, out);
        } else if p.extension().is_some_and(|e| e == "urdf") {
            out.push(p);
        }
    }
}

/// None when the corpus is absent; RIGSCAN_REQUIRE_CORPUS=1 turns that into a failure so CI
/// cannot pass silently without it.
fn load_corpus() -> Option<Corpus> {
    let dir = corpus_dir();
    if !dir.exists() {
        assert!(
            std::env::var_os("RIGSCAN_REQUIRE_CORPUS").is_none(),
            "corpus required but missing at {}",
            dir.display()
        );
        eprintln!(
            "SKIPPED: corpus not found at {}; run scripts/fetch-corpus.sh",
            dir.display()
        );
        return None;
    }
    let mut paths = Vec::new();
    collect(&dir, &mut paths);
    paths.sort();
    let files = paths
        .into_iter()
        .map(|p| {
            let src = std::fs::read_to_string(&p).unwrap();
            (p, src)
        })
        .collect();
    Some(Corpus { files })
}

fn command_output(prog: &str, args: &[&str]) -> String {
    Command::new(prog)
        .args(args)
        .output()
        .ok()
        .filter(|o| o.status.success())
        .map(|o| String::from_utf8_lossy(&o.stdout).trim().to_string())
        .unwrap_or_else(|| "unknown".into())
}

fn cpu() -> String {
    if let Ok(info) = std::fs::read_to_string("/proc/cpuinfo") {
        if let Some(line) = info.lines().find(|l| l.starts_with("model name")) {
            return line.split(':').nth(1).unwrap_or("unknown").trim().into();
        }
    }
    command_output("sysctl", &["-n", "machdep.cpu.brand_string"])
}

fn corpus_commit() -> String {
    std::fs::read_to_string(corpus_dir().join("SOURCE"))
        .map(|s| s.trim_end().replace('\n', "@"))
        .unwrap_or_else(|_| "unknown".into())
}

fn short(p: &Path) -> String {
    p.strip_prefix(corpus_dir().join("robots"))
        .unwrap_or(p)
        .display()
        .to_string()
}

fn best_of<T>(mut f: impl FnMut() -> T) -> Duration {
    (0..REPS)
        .map(|_| {
            let t = Instant::now();
            std::hint::black_box(f());
            t.elapsed()
        })
        .min()
        .unwrap()
}

#[test]
fn corpus_lints_clean_and_reports_performance() {
    let Some(corpus) = load_corpus() else { return };
    assert!(
        corpus.files.len() >= 50,
        "only {} files",
        corpus.files.len()
    );
    for needle in ["go1", "panda", "so101"] {
        assert!(
            corpus.files.iter().any(|(p, _)| short(p).contains(needle)),
            "{needle} missing from the corpus"
        );
    }

    let mut rows = Vec::new();
    let mut bytes = 0usize;
    let mut elements = 0usize;
    for (path, src) in &corpus.files {
        let robot = parse_urdf(src).unwrap_or_else(|e| panic!("{}: {}", short(path), e.0));
        let rules: Vec<&str> = lint(&robot).iter().map(|f| f.rule).collect();
        let name = short(path);
        let expected: Vec<&str> = KNOWN_FINDINGS
            .iter()
            .filter(|(file, _)| *file == name)
            .map(|(_, rule)| *rule)
            .collect();
        assert_eq!(
            rules, expected,
            "{name}: findings differ from the known ones"
        );
        let parse = best_of(|| parse_urdf(src));
        let lint_time = best_of(|| lint(&robot));
        bytes += src.len();
        elements += robot.links.len() + robot.joints.len();
        rows.push((short(path), src.len(), parse, lint_time));
    }

    let parse_total: Duration = rows.iter().map(|r| r.2).sum();
    let lint_total: Duration = rows.iter().map(|r| r.3).sum();
    let mb = bytes as f64 / 1e6;
    println!("--- rigscan e2e performance ---");
    println!(
        "corpus:  {} ({} files, {mb:.2} MB)",
        corpus_commit(),
        rows.len()
    );
    println!(
        "rigscan: {}  cpu: {}  {}  profile: {}",
        command_output("git", &["rev-parse", "--short", "HEAD"]),
        cpu(),
        command_output("rustc", &["--version"]),
        if cfg!(debug_assertions) {
            "debug (timings not meaningful)"
        } else {
            "release"
        }
    );
    println!(
        "parse:   {parse_total:?} total, {:.0} MB/s (best of {REPS})",
        mb / parse_total.as_secs_f64()
    );
    println!(
        "lint:    {lint_total:?} total, {:.0} elements/s over {elements} links and joints",
        elements as f64 / lint_total.as_secs_f64()
    );
    rows.sort_by_key(|r| std::cmp::Reverse(r.2));
    println!("slowest parses:");
    for (name, size, parse, _) in rows.iter().take(5) {
        println!("  {parse:>10.2?}  {:>7.1} KB  {name}", *size as f64 / 1e3);
    }
}

struct Rng(u64);

impl Rng {
    // SplitMix64: tiny, and the same sequence on every platform.
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    fn below(&mut self, n: usize) -> usize {
        (self.next() % n as u64) as usize
    }
}

#[derive(Clone, Copy, Debug)]
enum Mode {
    UndefinedLink,
    DuplicateLink,
    DuplicateJoint,
    MultiParent,
    Loop,
    ExtraRoot,
    InvalidNumber,
    TruncatedXml,
}

const MODES: [Mode; 8] = [
    Mode::UndefinedLink,
    Mode::DuplicateLink,
    Mode::DuplicateJoint,
    Mode::MultiParent,
    Mode::Loop,
    Mode::ExtraRoot,
    Mode::InvalidNumber,
    Mode::TruncatedXml,
];

impl Mode {
    /// The rule that must fire, or None for modes that must be a parse error.
    fn rule(self) -> Option<&'static str> {
        match self {
            Mode::UndefinedLink => Some("RS001"),
            Mode::DuplicateLink => Some("RS002"),
            Mode::DuplicateJoint => Some("RS003"),
            Mode::MultiParent => Some("RS004"),
            Mode::Loop => Some("RS005"),
            Mode::ExtraRoot => Some("RS006"),
            Mode::InvalidNumber | Mode::TruncatedXml => None,
        }
    }
}

fn offset(src: &str, at: Span) -> usize {
    let mut off = 0;
    for (i, line) in src.split_inclusive('\n').enumerate() {
        if i + 1 == at.line as usize {
            let col = line.char_indices().nth(at.col as usize - 1);
            return off + col.map_or(line.len(), |(b, _)| b);
        }
        off += line.len();
    }
    src.len()
}

fn append(src: &str, xml: &str) -> Option<String> {
    let at = src.rfind("</robot>")?;
    Some(format!("{}{xml}{}", &src[..at], &src[at..]))
}

fn root(robot: &Robot) -> Option<&str> {
    robot
        .links
        .iter()
        .map(|l| l.name.as_str())
        .find(|n| robot.joints.iter().all(|j| j.child != *n))
}

/// None when this file has no place to apply the mode.
fn corrupt(mode: Mode, src: &str, robot: &Robot, rng: &mut Rng) -> Option<String> {
    let joints = &robot.joints;
    match mode {
        Mode::UndefinedLink => {
            if joints.is_empty() {
                return None;
            }
            let j = &joints[rng.below(joints.len())];
            let from = offset(src, j.span);
            let needle = format!("<child link=\"{}\"", j.child);
            let at = from + src[from..].find(&needle)? + needle.len() - 1;
            Some(format!("{}_missing{}", &src[..at], &src[at..]))
        }
        Mode::DuplicateLink => {
            if robot.links.is_empty() {
                return None;
            }
            let l = &robot.links[rng.below(robot.links.len())];
            append(src, &format!("<link name=\"{}\"/>", l.name))
        }
        Mode::DuplicateJoint => {
            if joints.len() < 2 {
                return None;
            }
            let (a, b) = (
                &joints[rng.below(joints.len())],
                &joints[rng.below(joints.len())],
            );
            if a.name == b.name {
                return None;
            }
            let from = offset(src, b.span);
            let tag_end = from + src[from..].find('>')?;
            let needle = format!("name=\"{}\"", b.name);
            let at = from + src[from..tag_end].find(&needle)?;
            Some(format!(
                "{}name=\"{}\"{}",
                &src[..at],
                a.name,
                &src[at + needle.len()..]
            ))
        }
        Mode::MultiParent => {
            let child = &joints.get(rng.below(joints.len().max(1)))?.child;
            append(
                src,
                &format!(
                    "<joint name=\"rigscan_extra\" type=\"fixed\"><parent link=\"{}\"/><child link=\"{child}\"/></joint>",
                    root(robot)?
                ),
            )
        }
        Mode::Loop => {
            let from = &joints.get(rng.below(joints.len().max(1)))?.child;
            append(
                src,
                &format!(
                    "<joint name=\"rigscan_loop\" type=\"fixed\"><parent link=\"{from}\"/><child link=\"{}\"/></joint>",
                    root(robot)?
                ),
            )
        }
        Mode::ExtraRoot => append(src, "<link name=\"rigscan_orphan\"/>"),
        Mode::InvalidNumber => {
            let sites: Vec<usize> = ["<mass value=\"", "<inertia ixx=\""]
                .iter()
                .flat_map(|n| src.match_indices(n).map(move |(i, _)| i + n.len()))
                .collect();
            let at = *sites.get(rng.below(sites.len().max(1)))?;
            let end = at + src[at..].find('"')?;
            Some(format!("{}abc{}", &src[..at], &src[end..]))
        }
        Mode::TruncatedXml => {
            let mut cut = src.len() / 10 + rng.below(src.len() * 8 / 10);
            while !src.is_char_boundary(cut) {
                cut -= 1;
            }
            Some(src[..cut].to_string())
        }
    }
}

fn detected(mode: Mode, corrupted: &str) -> bool {
    match (parse_urdf(corrupted), mode.rule()) {
        (Ok(robot), Some(rule)) => lint(&robot).iter().any(|f| f.rule == rule),
        (Err(e), None) => matches!(mode, Mode::TruncatedXml) || e.0.contains("invalid number"),
        _ => false,
    }
}

#[test]
fn corruptions_are_detected_equally_across_failure_modes() {
    let Some(corpus) = load_corpus() else { return };
    let seed = std::env::var("RIGSCAN_SEED")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(DEFAULT_SEED);
    let robots: Vec<Robot> = corpus
        .files
        .iter()
        .map(|(p, s)| parse_urdf(s).unwrap_or_else(|e| panic!("{}: {}", short(p), e.0)))
        .collect();

    let mut rng = Rng(seed);
    let mut tally = [(0usize, 0usize); MODES.len()];
    let mut missed = Vec::new();
    for i in 0..MODES.len() * CASES_PER_MODE {
        let (m, mode) = (i % MODES.len(), MODES[i % MODES.len()]);
        let (idx, bad) = (0..1000)
            .find_map(|_| {
                let idx = rng.below(corpus.files.len());
                corrupt(mode, &corpus.files[idx].1, &robots[idx], &mut rng).map(|bad| (idx, bad))
            })
            .unwrap_or_else(|| panic!("no file in the corpus fits {mode:?}"));
        tally[m].0 += 1;
        if detected(mode, &bad) {
            tally[m].1 += 1;
        } else {
            missed.push(format!("{mode:?} on {}", short(&corpus.files[idx].0)));
        }
    }

    println!("--- rigscan corruption detection (seed {seed:#x}) ---");
    for (mode, (cases, hits)) in MODES.iter().zip(tally) {
        println!("  {:<15} {hits}/{cases} detected", format!("{mode:?}"));
    }
    assert!(
        missed.is_empty(),
        "undetected (seed {seed:#x}): {missed:#?}"
    );
}

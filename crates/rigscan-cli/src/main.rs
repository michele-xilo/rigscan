mod discover;
mod output;
mod resolve;

use clap::{Parser, Subcommand, ValueEnum};
use output::Scanned;
use rayon::prelude::*;
use rigscan_core::{
    lint_mjcf, lint_usd, lint_with, parse_mjcf, parse_urdf, parse_usd, write_mjcf, write_urdf,
    write_usd, Format, Geometry, Robot, Severity,
};
use std::cell::RefCell;
use std::collections::HashSet;
use std::io::{IsTerminal, Write};
use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::rc::Rc;

#[derive(Parser)]
#[command(version, about = "A health check for robot descriptions")]
struct Cli {
    /// More log detail on stderr: -v info, -vv debug, -vvv trace; RUST_LOG overrides
    #[arg(short, long, action = clap::ArgAction::Count, global = true)]
    verbose: u8,
    /// How log lines are written to stderr
    #[arg(long, value_enum, default_value_t = LogFormat::Human, global = true)]
    log_format: LogFormat,
    #[command(subcommand)]
    command: Command,
}

#[derive(Clone, Copy, ValueEnum)]
enum LogFormat {
    Human,
    /// One JSON object per line
    Json,
}

#[derive(Clone, Copy, ValueEnum)]
enum OutputFormat {
    /// One line per finding
    Text,
    /// Each finding with its source line and a caret under the column
    Full,
    /// Status of every link and joint, with a verdict line
    Report,
    /// One JSON document for all files
    Json,
}

#[derive(Clone, Copy, ValueEnum)]
enum ColorChoice {
    /// Colour when writing to a terminal, unless NO_COLOR is set
    Auto,
    Always,
    Never,
}

#[derive(Clone, Copy, ValueEnum)]
enum Target {
    Urdf,
    Mjcf,
    Usd,
}

#[derive(Subcommand)]
enum Command {
    /// Scan robot description files and report findings
    Scan {
        /// Files or directories to scan; defaults to the current directory
        paths: Vec<PathBuf>,
        #[arg(long, value_enum, default_value_t = OutputFormat::Text)]
        output_format: OutputFormat,
        #[arg(long, value_enum, default_value_t = ColorChoice::Auto)]
        color: ColorChoice,
        /// Print no findings; print the summary only when something is wrong (text format)
        #[arg(short, long)]
        quiet: bool,
        /// Count findings per rule instead of listing them (text and full formats)
        #[arg(long)]
        statistics: bool,
    },
    /// Convert a robot description to another format and list what was lost
    Convert {
        input: PathBuf,
        #[arg(long, value_enum)]
        to: Target,
        /// Write the result to this file instead of stdout
        #[arg(short, long)]
        output: Option<PathBuf>,
        /// Write what was lost to this file as JSON
        #[arg(long)]
        loss_report: Option<PathBuf>,
    },
}

/// Logs go to stderr only, so stdout stays exactly the findings. Span timings are printed when a
/// span closes, which is what gives each phase of each file its duration.
fn init_logging(verbose: u8, format: LogFormat) {
    use tracing_subscriber::fmt::format::FmtSpan;
    use tracing_subscriber::EnvFilter;
    let level = ["warn", "info", "debug", "trace"][usize::from(verbose.min(3))];
    let filter = EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new(level));
    let builder = tracing_subscriber::fmt()
        .with_env_filter(filter)
        .with_writer(std::io::stderr)
        .with_span_events(FmtSpan::CLOSE);
    match format {
        LogFormat::Human => builder.with_ansi(std::io::stderr().is_terminal()).init(),
        LogFormat::Json => builder.json().init(),
    }
}

fn main() -> ExitCode {
    let cli = Cli::parse();
    init_logging(cli.verbose, cli.log_format);
    match cli.command {
        Command::Scan {
            paths,
            output_format,
            color,
            quiet,
            statistics,
        } => scan(&paths, output_format, color, quiet, statistics),
        Command::Convert {
            input,
            to,
            output,
            loss_report,
        } => convert(&input, to, output.as_deref(), loss_report.as_deref()),
    }
}

/// Exit codes: 0 nothing lost, 1 something lost, 2 the input was refused or the output not written.
fn convert(
    input: &Path,
    to: Target,
    output: Option<&Path>,
    loss_report: Option<&Path>,
) -> ExitCode {
    let refuse = |msg: String| {
        eprintln!("{msg}");
        ExitCode::from(2)
    };
    let scanned = match scan_file(input) {
        Ok(s) => s,
        Err(msg) => return refuse(msg),
    };
    let (target, name) = match to {
        Target::Urdf => (Format::Urdf, "URDF"),
        Target::Mjcf => (Format::Mjcf, "MJCF"),
        Target::Usd => (Format::Usd, "USD"),
    };
    if scanned.format == target {
        return refuse(format!("{}: already {name}", input.display()));
    }
    let critical: Vec<_> = scanned
        .findings
        .iter()
        // Mesh files are only referenced, so one that is not on disk does not stop a conversion.
        .filter(|f| f.severity == Severity::Critical && f.rule != "RS014")
        .collect();
    if !critical.is_empty() {
        for f in &critical {
            eprintln!("{}", output::format_finding(input, f, false));
        }
        return refuse(format!(
            "{}: not converted, fix the findings above",
            input.display()
        ));
    }
    let mut robot = scanned.robot;
    if let Some(path) = output {
        rebase_meshes(&mut robot, input, path);
    }
    let converted = match to {
        Target::Urdf => write_urdf(&robot),
        Target::Mjcf => write_mjcf(&robot),
        Target::Usd => write_usd(&robot),
    };
    let written = match output {
        Some(path) => {
            std::fs::write(path, &converted.text).map_err(|e| format!("{}: {e}", path.display()))
        }
        None => std::io::stdout()
            .lock()
            .write_all(converted.text.as_bytes())
            .map_err(|e| e.to_string()),
    };
    if let Err(msg) = written {
        return refuse(msg);
    }
    eprint!("{}", output::loss_text(&converted.lost));
    if let Some(path) = loss_report {
        let doc = output::loss_json(input, scanned.format, target, &converted.lost);
        let text = serde_json::to_string_pretty(&doc).unwrap_or_default();
        if let Err(e) = std::fs::write(path, text + "\n") {
            return refuse(format!("{}: {e}", path.display()));
        }
    }
    ExitCode::from(u8::from(!converted.lost.is_empty()))
}

/// Mesh files are only referenced, so a relative path must still reach the file from where the
/// output is written. Paths with a scheme (`package://`) or already absolute are left alone.
fn rebase_meshes(robot: &mut Robot, input: &Path, output: &Path) {
    let dir = |p: &Path| {
        p.parent()
            .map(|d| {
                if d.as_os_str().is_empty() {
                    Path::new(".")
                } else {
                    d
                }
            })
            .and_then(|d| d.canonicalize().ok())
    };
    let (Some(from), Some(to)) = (dir(input), dir(output)) else {
        return;
    };
    if from == to {
        return;
    }
    let common = from
        .components()
        .zip(to.components())
        .take_while(|(a, b)| a == b)
        .count();
    let mut prefix = PathBuf::new();
    for _ in to.components().skip(common) {
        prefix.push("..");
    }
    prefix.extend(from.components().skip(common));
    for g in robot.links.iter_mut().flat_map(|l| &mut l.shapes) {
        if let Geometry::Mesh { filename, .. } = &mut g.geometry {
            if !filename.contains("://") && Path::new(filename.as_str()).is_relative() {
                *filename = prefix.join(&*filename).to_string_lossy().into_owned();
            }
        }
    }
}

/// Exit codes: 0 clean, 1 critical findings, 2 a file could not be scanned.
fn scan(
    paths: &[PathBuf],
    format: OutputFormat,
    color: ColorChoice,
    quiet: bool,
    statistics: bool,
) -> ExitCode {
    let found = discover::discover(paths);
    for e in &found.errors {
        eprintln!("{e}");
    }
    if found.files.is_empty() {
        eprintln!("no supported files found");
    }
    // Collected in input order, so a parallel run prints the same bytes as a serial one.
    let results: Vec<Result<Scanned, String>> =
        found.files.par_iter().map(|p| scan_file(p)).collect();
    let (files, results) = without_included(&found, results);
    let files = &files;
    let failed = !found.errors.is_empty() || results.iter().any(Result::is_err);
    let critical = results
        .iter()
        .flatten()
        .any(|s| s.findings.iter().any(|f| f.severity == Severity::Critical));

    let color_out = use_color(color, std::io::stdout().is_terminal());
    let color_err = use_color(color, std::io::stderr().is_terminal());
    let listing = matches!(format, OutputFormat::Text | OutputFormat::Full);
    let quiet_text = quiet && listing;
    let statistics = statistics && listing;
    if write_output(files, &results, format, (quiet_text, statistics), color_out).is_err() {
        // A closed pipe (`rigscan scan x | head`) is not worth a panic.
        return ExitCode::from(2);
    }
    if listing && !files.is_empty() {
        let failed_files = found.errors.len() + results.iter().filter(|r| r.is_err()).count();
        print_summary(&results, failed_files, quiet, color_err);
    }
    ExitCode::from(if failed {
        2
    } else if critical {
        1
    } else {
        0
    })
}

/// Drops the results for files found by walking a directory that another file includes: alone they
/// are fragments, not robots, and would report names only the including file defines. Only
/// includes from files that scanned fine count, so a file that fails (an include cycle) never
/// hides the files it includes.
fn without_included(
    found: &discover::Discovered,
    results: Vec<Result<Scanned, String>>,
) -> (Vec<PathBuf>, Vec<Result<Scanned, String>>) {
    let included: HashSet<&Path> = results
        .iter()
        .flatten()
        .flat_map(|s| s.included.iter().map(PathBuf::as_path))
        .collect();
    let is_fragment = |p: &PathBuf| {
        !found.named.contains(p)
            && p.canonicalize()
                .is_ok_and(|c| included.contains(c.as_path()))
    };
    let keep: Vec<bool> = found.files.iter().map(|p| !is_fragment(p)).collect();
    let files = found
        .files
        .iter()
        .zip(&keep)
        .filter(|(_, k)| **k)
        .map(|(p, _)| p.clone())
        .collect();
    let results = results
        .into_iter()
        .zip(&keep)
        .filter(|(_, k)| **k)
        .map(|(r, _)| r)
        .collect();
    (files, results)
}

/// NO_COLOR (when non-empty) turns auto off, as does a dumb terminal; `always` ignores both.
fn use_color(choice: ColorChoice, is_terminal: bool) -> bool {
    match choice {
        ColorChoice::Always => true,
        ColorChoice::Never => false,
        ColorChoice::Auto => {
            is_terminal
                && std::env::var_os("NO_COLOR").is_none_or(|v| v.is_empty())
                && std::env::var("TERM").map_or(true, |t| t != "dumb")
        }
    }
}

fn plural(n: usize, one: &str, many: &str) -> String {
    format!("{n} {}", if n == 1 { one } else { many })
}

/// On stderr so stdout stays exactly the findings.
fn print_summary(
    results: &[Result<Scanned, String>],
    failed_files: usize,
    quiet: bool,
    color: bool,
) {
    let findings: usize = results.iter().flatten().map(|s| s.findings.len()).sum();
    let with_findings = results
        .iter()
        .flatten()
        .filter(|s| !s.findings.is_empty())
        .count();
    if findings > 0 {
        let line = format!(
            "Found {} in {}.",
            plural(findings, "finding", "findings"),
            plural(with_findings, "file", "files")
        );
        eprintln!("{}", output::problem(color, &line));
    }
    if failed_files > 0 {
        let line = format!(
            "{} could not be scanned.",
            plural(failed_files, "file", "files")
        );
        eprintln!("{}", output::problem(color, &line));
    }
    if findings == 0 && failed_files == 0 && !quiet {
        eprintln!("{}", output::healthy(color, "All checks passed!"));
    }
}

fn write_output(
    files: &[PathBuf],
    results: &[Result<Scanned, String>],
    format: OutputFormat,
    (quiet, statistics): (bool, bool),
    color: bool,
) -> std::io::Result<()> {
    let mut stdout = std::io::stdout().lock();
    if let OutputFormat::Json = format {
        let pairs: Vec<(&Path, &Result<Scanned, String>)> =
            files.iter().map(PathBuf::as_path).zip(results).collect();
        return writeln!(stdout, "{}", output::json_document(&pairs));
    }
    if statistics {
        for e in results.iter().filter_map(|r| r.as_ref().err()) {
            eprintln!("{e}");
        }
        let scanned: Vec<&Scanned> = results.iter().flatten().collect();
        return output::write_statistics(&mut stdout, &scanned, color);
    }
    for (path, result) in files.iter().zip(results) {
        match result {
            Ok(_) if quiet => {}
            Ok(s) => match format {
                OutputFormat::Report => output::write_report(&mut stdout, path, s, color)?,
                OutputFormat::Full => output::write_full(&mut stdout, path, s, color)?,
                _ => output::write_text(&mut stdout, path, s, color)?,
            },
            Err(msg) => eprintln!("{msg}"),
        }
    }
    Ok(())
}

#[tracing::instrument(level = "debug", skip_all, fields(path = %path.display()))]
fn scan_file(path: &Path) -> Result<Scanned, String> {
    let format =
        detect_format(path).ok_or_else(|| format!("{}: unsupported file type", path.display()))?;
    // A USD layer may be binary, and the stage reads its own files through the loader.
    let src = match format {
        Format::Usd => String::new(),
        _ => std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?,
    };
    let resolver = resolve::FsResolver::for_file(path);
    let at = |e: rigscan_core::ParseError| format!("{}: {}", path.display(), e.0);
    let (robot, findings, included) = match format {
        Format::Urdf => {
            let robot = parse_urdf(&src).map_err(at)?;
            let findings = lint_with(&robot, &resolver);
            (robot, findings, Vec::new())
        }
        Format::Mjcf => {
            let loader = resolve::FsLoader::for_file(path);
            let mjcf = parse_mjcf(&src, &loader).map_err(at)?;
            let robot = mjcf.to_robot().map_err(at)?;
            let findings = lint_mjcf(&mjcf, &robot, &resolver);
            let included = loader
                .into_read()
                .iter()
                .filter_map(|p| p.canonicalize().ok())
                .collect();
            (robot, findings, included)
        }
        Format::Usd => {
            let read = Rc::new(RefCell::new(Vec::new()));
            let loader = resolve::FsUsdLoader { read: read.clone() };
            let usd = parse_usd(&path.display().to_string(), loader).map_err(at)?;
            let robot = usd.to_robot().map_err(at)?;
            let findings = lint_usd(&usd, &robot);
            let root = path.canonicalize().ok();
            let included = read
                .borrow()
                .iter()
                .filter_map(|p| p.canonicalize().ok())
                .filter(|p| Some(p) != root.as_ref())
                .collect();
            (robot, findings, included)
        }
    };
    tracing::debug!(
        links = robot.links.len(),
        joints = robot.joints.len(),
        findings = findings.len(),
        "scanned"
    );
    Ok(Scanned {
        src,
        robot,
        format,
        findings,
        included,
    })
}

pub fn detect_format(path: &Path) -> Option<Format> {
    match path.extension()?.to_str()?.to_ascii_lowercase().as_str() {
        "urdf" => Some(Format::Urdf),
        "usda" | "usdc" | "usd" => Some(Format::Usd),
        // `.xml` is also ROS launch files and package manifests.
        "xml" if looks_like_mjcf(path) => Some(Format::Mjcf),
        _ => None,
    }
}

/// Whether the start of the file has a `<mujoco` root, without reading a large file whole.
fn looks_like_mjcf(path: &Path) -> bool {
    use std::io::Read;
    let mut head = Vec::new();
    let read = std::fs::File::open(path).and_then(|f| f.take(16 * 1024).read_to_end(&mut head));
    read.is_ok() && head.windows(7).any(|w| w == b"<mujoco")
}

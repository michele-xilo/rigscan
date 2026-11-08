use clap::{Parser, Subcommand};
use rigscan_core::{lint, parse_urdf, Finding, Format, Severity};
use std::io::Write;
use std::path::{Path, PathBuf};
use std::process::ExitCode;

#[derive(Parser)]
#[command(version, about = "A health check for robot descriptions")]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Scan robot description files and report findings
    Scan {
        #[arg(required = true)]
        files: Vec<PathBuf>,
    },
}

fn main() -> ExitCode {
    match Cli::parse().command {
        Command::Scan { files } => scan(&files),
    }
}

/// Exit codes: 0 clean, 1 critical findings, 2 a file could not be scanned.
fn scan(files: &[PathBuf]) -> ExitCode {
    let mut stdout = std::io::stdout().lock();
    let mut critical = false;
    let mut failed = false;
    for path in files {
        match scan_file(path) {
            Ok(findings) => {
                for f in &findings {
                    critical |= f.severity == Severity::Critical;
                    // A closed pipe (`rigscan scan x | head`) is not worth a panic.
                    if writeln!(stdout, "{}", format_finding(path, f)).is_err() {
                        return ExitCode::from(2);
                    }
                }
            }
            Err(msg) => {
                eprintln!("{msg}");
                failed = true;
            }
        }
    }
    ExitCode::from(if failed {
        2
    } else if critical {
        1
    } else {
        0
    })
}

fn scan_file(path: &Path) -> Result<Vec<Finding>, String> {
    let format =
        detect_format(path).ok_or_else(|| format!("{}: unsupported file type", path.display()))?;
    let src = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
    let robot = match format {
        Format::Urdf => parse_urdf(&src),
    }
    .map_err(|e| format!("{}: {}", path.display(), e.0))?;
    Ok(lint(&robot))
}

fn detect_format(path: &Path) -> Option<Format> {
    match path.extension()?.to_str()?.to_ascii_lowercase().as_str() {
        "urdf" => Some(Format::Urdf),
        _ => None,
    }
}

fn format_finding(path: &Path, f: &Finding) -> String {
    let severity = match f.severity {
        Severity::Warning => "warning",
        Severity::Critical => "critical",
    };
    format!(
        "{}:{}:{}: {severity} {}: {}",
        path.display(),
        f.span.line,
        f.span.col,
        f.rule,
        f.message
    )
}

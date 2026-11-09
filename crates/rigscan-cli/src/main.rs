mod output;

use clap::{Parser, Subcommand, ValueEnum};
use output::Scanned;
use rigscan_core::{lint, parse_urdf, Format, Severity};
use std::io::Write;
use std::path::{Path, PathBuf};
use std::process::ExitCode;

#[derive(Parser)]
#[command(version, about = "A health check for robot descriptions")]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Clone, Copy, ValueEnum)]
enum OutputFormat {
    /// One line per finding
    Text,
    /// Status of every link and joint, with a verdict line
    Report,
    /// One JSON document for all files
    Json,
}

#[derive(Subcommand)]
enum Command {
    /// Scan robot description files and report findings
    Scan {
        #[arg(required = true)]
        files: Vec<PathBuf>,
        #[arg(long, value_enum, default_value_t = OutputFormat::Text)]
        output_format: OutputFormat,
    },
}

fn main() -> ExitCode {
    match Cli::parse().command {
        Command::Scan {
            files,
            output_format,
        } => scan(&files, output_format),
    }
}

/// Exit codes: 0 clean, 1 critical findings, 2 a file could not be scanned.
fn scan(files: &[PathBuf], format: OutputFormat) -> ExitCode {
    let results: Vec<Result<Scanned, String>> = files.iter().map(|p| scan_file(p)).collect();
    let failed = results.iter().any(Result::is_err);
    let critical = results
        .iter()
        .flatten()
        .any(|s| s.findings.iter().any(|f| f.severity == Severity::Critical));

    if write_output(files, &results, format).is_err() {
        // A closed pipe (`rigscan scan x | head`) is not worth a panic.
        return ExitCode::from(2);
    }
    ExitCode::from(if failed {
        2
    } else if critical {
        1
    } else {
        0
    })
}

fn write_output(
    files: &[PathBuf],
    results: &[Result<Scanned, String>],
    format: OutputFormat,
) -> std::io::Result<()> {
    let mut stdout = std::io::stdout().lock();
    if let OutputFormat::Json = format {
        let pairs: Vec<(&Path, &Result<Scanned, String>)> =
            files.iter().map(PathBuf::as_path).zip(results).collect();
        return writeln!(stdout, "{}", output::json_document(&pairs));
    }
    for (path, result) in files.iter().zip(results) {
        match result {
            Ok(s) => match format {
                OutputFormat::Report => output::write_report(&mut stdout, path, s)?,
                _ => output::write_text(&mut stdout, path, s)?,
            },
            Err(msg) => eprintln!("{msg}"),
        }
    }
    Ok(())
}

fn scan_file(path: &Path) -> Result<Scanned, String> {
    let format =
        detect_format(path).ok_or_else(|| format!("{}: unsupported file type", path.display()))?;
    let src = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
    let robot = match format {
        Format::Urdf => parse_urdf(&src),
    }
    .map_err(|e| format!("{}: {}", path.display(), e.0))?;
    let findings = lint(&robot);
    Ok(Scanned {
        robot,
        format,
        findings,
    })
}

fn detect_format(path: &Path) -> Option<Format> {
    match path.extension()?.to_str()?.to_ascii_lowercase().as_str() {
        "urdf" => Some(Format::Urdf),
        _ => None,
    }
}

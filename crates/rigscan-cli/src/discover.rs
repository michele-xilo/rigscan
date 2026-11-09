//! Turning command line paths into the list of files to scan.

use crate::detect_format;
use ignore::WalkBuilder;
use std::path::PathBuf;

pub struct Discovered {
    pub files: Vec<PathBuf>,
    pub errors: Vec<String>,
}

/// Files named explicitly are always kept, so an unsupported one is reported later as an error.
/// Directories are walked for supported files, honouring `.gitignore` even outside a git
/// repository (an unpacked archive is a common case). No paths means the current directory.
pub fn discover(args: &[PathBuf]) -> Discovered {
    let current = [PathBuf::from(".")];
    let args = if args.is_empty() { &current } else { args };
    let mut found = Discovered {
        files: Vec::new(),
        errors: Vec::new(),
    };
    for arg in args {
        if !arg.is_dir() {
            found.files.push(arg.clone());
            continue;
        }
        let mut in_dir = Vec::new();
        for entry in WalkBuilder::new(arg).require_git(false).build() {
            match entry {
                Ok(e) if e.file_type().is_some_and(|t| t.is_file()) => {
                    let path = e.into_path();
                    if detect_format(&path).is_some() {
                        in_dir.push(path);
                    }
                }
                Ok(_) => {}
                Err(err) => found.errors.push(err.to_string()),
            }
        }
        // Sorted so the output does not depend on directory order or thread timing.
        in_dir.sort();
        found.files.extend(in_dir.into_iter().map(|p| {
            // "./a.urdf" reads as noise next to what a user typed.
            p.strip_prefix(".").map(PathBuf::from).unwrap_or(p)
        }));
    }
    found
}

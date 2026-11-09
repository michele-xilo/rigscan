//! Looks up mesh references on the file system, relative to the URDF being scanned.

use rigscan_core::{Resolution, Resolver};
use std::path::{Path, PathBuf};

pub struct FsResolver {
    dir: PathBuf,
}

impl FsResolver {
    pub fn for_file(file: &Path) -> Self {
        let abs = std::path::absolute(file).unwrap_or_else(|_| file.to_path_buf());
        FsResolver {
            dir: abs.parent().map(Path::to_path_buf).unwrap_or_default(),
        }
    }
}

fn exists(path: PathBuf) -> Resolution {
    if path.exists() {
        Resolution::Found
    } else {
        Resolution::Missing
    }
}

impl Resolver for FsResolver {
    /// ROS finds `package://name/...` through a package index that is not available here, so a package
    /// is taken to be the nearest ancestor directory called `name`, which is how a checked-out
    /// package or workspace is laid out. Without one the answer is `Unknown`, never `Missing`.
    fn resolve(&self, uri: &str) -> Resolution {
        if let Some(rest) = uri.strip_prefix("package://") {
            let Some((package, file)) = rest.split_once('/') else {
                return Resolution::Unknown;
            };
            match self
                .dir
                .ancestors()
                .find(|d| d.file_name().is_some_and(|n| n == package))
            {
                Some(root) => exists(root.join(file)),
                None => Resolution::Unknown,
            }
        } else if let Some(path) = uri.strip_prefix("file://") {
            exists(PathBuf::from(path))
        } else if uri.contains("://") {
            Resolution::Unknown
        } else {
            exists(self.dir.join(uri))
        }
    }
}

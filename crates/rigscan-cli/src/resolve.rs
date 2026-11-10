//! Looks up mesh references on the file system, relative to the URDF being scanned.

use rigscan_core::{IncludeLoader, Included, Resolution, Resolver};
use std::cell::RefCell;
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

/// Reads `<include>`d MJCF files from disk.
pub struct FsLoader {
    dir: PathBuf,
    /// Every file read, so the caller knows which files are only parts of another.
    read: RefCell<Vec<PathBuf>>,
}

impl FsLoader {
    pub fn for_file(file: &Path) -> Self {
        FsLoader {
            dir: file.parent().map(Path::to_path_buf).unwrap_or_default(),
            read: RefCell::default(),
        }
    }

    pub fn into_read(self) -> Vec<PathBuf> {
        self.read.into_inner()
    }
}

impl IncludeLoader for FsLoader {
    /// MuJoCo reads an include relative to the file that contains it, so a nested include
    /// starts from the directory of `from`.
    fn load(&self, file: &str, from: Option<&str>) -> Option<Included> {
        let base = from
            .and_then(|f| Path::new(f).parent())
            .unwrap_or(&self.dir);
        let path = base.join(file);
        let text = std::fs::read_to_string(&path).ok()?;
        self.read.borrow_mut().push(path.clone());
        Some(Included {
            path: path.display().to_string(),
            text,
        })
    }
}

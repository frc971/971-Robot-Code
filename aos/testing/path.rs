use std::path::{Path, PathBuf};

/// Returns the correct path for a data file within AOS, accounting for whether AOS is
/// imported as an external repository.
pub fn artifact_path(path: &Path) -> PathBuf {
    Path::new("..")
        .join(Path::new(env!("AOS_REPO_NAME")))
        .join(path)
}

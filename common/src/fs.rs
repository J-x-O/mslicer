//! Writing files so that a crash or a full disk leaves either the old file
//! or the new one, never a truncated one.

use std::{
    fs::{self, File},
    io::{self, BufWriter, Write},
    path::{Path, PathBuf},
};

/// Writes `path` through `write` into a temporary file next to it, then
/// renames that over `path`. On an error the temporary file is removed and
/// `path` is left as it was.
pub fn write_atomic(
    path: &Path,
    write: impl FnOnce(&mut BufWriter<File>) -> io::Result<()>,
) -> io::Result<()> {
    let tmp = temp_path(path);
    let result = (|| {
        let mut out = BufWriter::new(File::create(&tmp)?);
        write(&mut out)?;
        out.flush()?;
        let file = out.into_inner().map_err(|e| e.into_error())?;
        file.sync_all()?;
        drop(file);
        fs::rename(&tmp, path)
    })();
    if result.is_err() {
        let _ = fs::remove_file(&tmp);
    }
    result
}

/// `dir/.name.tmp` for `dir/name`.
fn temp_path(path: &Path) -> PathBuf {
    let mut name = std::ffi::OsString::from(".");
    name.push(path.file_name().unwrap_or_else(|| "file".as_ref()));
    name.push(".tmp");
    path.with_file_name(name)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_failed_write_keeps_the_old_file() {
        let dir = std::env::temp_dir().join(format!("mslicer-fs-{}", std::process::id()));
        fs::create_dir_all(&dir).unwrap();
        let path = dir.join("project.mslicer");
        fs::write(&path, b"old").unwrap();

        let failed = write_atomic(&path, |out| {
            out.write_all(b"half")?;
            Err(io::Error::other("disk full"))
        });
        assert!(failed.is_err());
        assert_eq!(fs::read(&path).unwrap(), b"old");
        assert!(!temp_path(&path).exists(), "the temporary file is gone");

        write_atomic(&path, |out| out.write_all(b"new")).unwrap();
        assert_eq!(fs::read(&path).unwrap(), b"new");
        fs::remove_dir_all(&dir).unwrap();
    }
}

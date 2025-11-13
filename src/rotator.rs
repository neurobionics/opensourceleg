use std::fs::{File, OpenOptions};
use std::io::{self, Write};
use std::path::{PathBuf};
use std::sync::Mutex;
use tracing_subscriber::fmt::MakeWriter;

pub struct RotatingFileWriter {
    inner: Mutex<Rotator>,
}

impl RotatingFileWriter {
    pub fn new(base_path: PathBuf, max_size: u64, backup_count: u64) -> Self {
        Self {
            inner: Mutex::new(Rotator::new(base_path, max_size, backup_count).expect("Error creating new Rotator"))
        }
    }
}
struct Rotator {
    base_path: PathBuf,
    current_file: Option<File>,
    current_size: u64,
    max_size: u64,
    index: usize,
    backup_count: u64
}

impl Rotator {
    fn new(base_path: PathBuf, max_size: u64, backup_count: u64) -> io::Result<Self> {
        Ok(Self {
            base_path,
            current_file: None,
            current_size: 0,
            max_size,
            index: 0,
            backup_count
        })
    }

    fn ensure_file(&mut self) -> io::Result<()> {
        if self.current_file.is_none() {
            let file = OpenOptions::new().create(true).append(true).open(&self.base_path)?;
            self.current_size = file.metadata()?.len();
            self.current_file = Some(file);
        }
        Ok(())
    }

    fn maybe_rotate(&mut self) -> io::Result<()> {
        if self.current_size < self.max_size {
            // Don't rotate yet
            return Ok(())
        }

        let rotated_path = if self.backup_count == 0 {
            // Backup infinitely
            self.index += 1;
            self.base_path.with_file_name(format!(
                "{}_{}.log",
                self.base_path.file_stem().expect("Couldn't get file stem").to_string_lossy(),
                self.index
            ))
        }
        else{
            // Rotate
            self.index = (self.index + 1) % self.backup_count as usize;
            self.base_path.with_file_name(format!(
                "{}_{}.log",
                self.base_path.file_stem().expect("Couldn't get file stem").to_string_lossy(),
                self.index
            ))
        };

        if self.base_path.exists() {
            std::fs::rename(&self.base_path, rotated_path)?;
        }

        self.current_size = 0;
        return Ok(())
    }

    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        if self.max_size > 0 {
            self.maybe_rotate()?;
        }

        self.ensure_file()?; //lazily create file
        let file = self.current_file.as_mut().expect("File should be ensured");
        let written = file.write(buf)?;
        self.current_size += written as u64;
        Ok(written)
    }

    fn flush(&mut self) -> io::Result<()> {
        if let Some(file) = self.current_file.as_mut() {
            file.flush()
        } else {
            Ok(())
        }
    }
}

impl Write for RotatingFileWriter {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        self.inner.lock().unwrap().write(buf)
    }

    fn flush(&mut self) -> io::Result<()> {
        self.inner.lock().unwrap().flush()
    }
}

impl<'a> MakeWriter<'a> for RotatingFileWriter {
    type Writer = Self;

    fn make_writer(&self) -> Self::Writer {
        Self {
            inner: Mutex::new(self.inner.lock().unwrap().clone()),
        }
    }
}

impl Clone for Rotator {
    fn clone(&self) -> Self {
        let cloned_file = if self.current_file.is_some() {
            //open a new handle to the same file
            Some(
                OpenOptions::new()
                    .create(true)
                    .append(true)
                    .open(&self.base_path)
                    .expect("Failed to reopen file in Clone"),
            )
        } else {
            None
        };

        Self {
            base_path: self.base_path.clone(),
            current_file: cloned_file,
            current_size: self.current_size,
            max_size: self.max_size,
            index: self.index,
            backup_count: self.backup_count,
        }
    }
}

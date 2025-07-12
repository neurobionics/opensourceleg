use std::{collections::HashMap, fs::File, io::{BufWriter, Write}, path::PathBuf, sync::Mutex};
use chrono::Utc;
use serde_json::{Value};
use tracing::warn;
use tracing_appender::non_blocking::{NonBlocking, WorkerGuard};

pub struct Record {
    variables: HashMap::<String, Value>,
    writer: NonBlocking,
    _guard: WorkerGuard
}

impl Record {
    pub fn new(path: PathBuf) -> Self{
        let parent_dir = path.parent().unwrap_or(std::path::Path::new("."));
        let file_name = path.file_name().unwrap().to_str().unwrap();
        let file_appender = tracing_appender::rolling::never(parent_dir, file_name);
        let (non_blocking_writer, guard) = tracing_appender::non_blocking(file_appender);

        Self {
            variables: HashMap::<String, Value>::new(),
            writer: non_blocking_writer,
            _guard: guard
        }
    }

    pub fn insert_variable(&mut self, key: String, value: Value) {
        let prev_value = self.variables.insert(key, value);

        if let Some(old) = prev_value {
            warn!("{} is being overwritten", old);
        }
    }

    pub fn record_variables(&mut self) {
        if self.variables.is_empty() {
            return;
        }

        let record = serde_json::json!({
            "timestamp": Utc::now(),
            "variables": self.variables
        });

        let json = serde_json::to_string(&record).expect("json serialization failed");
        if let Err(e) = writeln!(self.writer, "{json}") {
            eprintln!("Failed to write to file: {e}");
        }
        self.variables.clear();
    }
    
    pub fn flush_buffer(&mut self) {
        self.writer.flush().expect("Error flushing writer");
    }
}
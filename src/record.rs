use std::{collections::HashMap, io::{Write}, path::PathBuf};
use chrono::Utc;
use pyo3::{PyObject, Python};
use serde_json::{Map, Value};
use tracing::{error, warn};
use tracing_appender::non_blocking::{NonBlocking, WorkerGuard};

use crate::logger::downcast;

pub struct Record {
    variables: HashMap::<String, Value>,
    writer: NonBlocking,
    _guard: WorkerGuard,
    functions: HashMap<String, PyObject>
}

impl Record {
    pub fn new(path: PathBuf) -> Self {
        let parent_dir = path.parent().unwrap_or(std::path::Path::new("."));
        let file_name = path.file_name().unwrap().to_str().unwrap();
        let file_appender = tracing_appender::rolling::never(parent_dir, file_name);
        let (non_blocking_writer, guard) = tracing_appender::non_blocking(file_appender);

        Self {
            variables: HashMap::<String, Value>::new(),
            writer: non_blocking_writer,
            _guard: guard,
            functions: HashMap::<String, PyObject>::new()
        }
    }

    pub fn insert_variable(&mut self, key: String, value: Value) {
        let prev_value = self.variables.insert(key, value);

        if let Some(old) = prev_value {
            warn!("{} is being overwritten", old);
        }
    }

    pub fn insert_function(&mut self, name: String, function: pyo3::Py<pyo3::PyAny>) {
        if self.functions.contains_key(&name) {
            warn!("{} is being overwritten", name);
        }
        self.functions.insert(name, function);
    }

    pub fn remove_function(&mut self, name: String) {
        if !self.functions.contains_key(&name) {
            error!("There is no function with the id {}", name);
            return;
        }
        self.functions.remove(&name);
    }

    pub fn record_variables(&mut self) {
        if self.variables.is_empty() && self.functions.is_empty() {
            return;
        }

        let mut function_results = Map::new();

        Python::with_gil(|py| {
            for (name, func) in &self.functions {
                match func.call0(py) {
                    Ok(val) => {
                        let bound_val = val.bind(py);
                        let json_val = downcast(bound_val.clone());
                        function_results.insert(name.clone(), json_val);
                    }
                    Err(e) => {
                        eprintln!("Function '{name}' call failed: {e}");
                    }
                }
            }
        });

        let record = serde_json::json!({
            "timestamp": Utc::now(),
            "variables": self.variables,
            "functions": function_results
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
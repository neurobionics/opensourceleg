use std::{collections::HashMap, fs::{File}, io::{Write}, sync::Mutex};
use serde_json::{Value};
use tracing::warn;

pub struct Record {
    variables: HashMap::<String, Value>,
    file: Mutex<File>
}

impl Record {
    pub fn new(path: String) -> Self{
        let file = File::create(path).expect("Could not create variable tracing file");
        Self {
            variables: HashMap::<String, Value>::new(),
            file: Mutex::new(file)
        }
    }

    pub fn insert_variable(&mut self, key: String, value: Value) {
        let prev_value = self.variables.insert(key, value);

        if let Some(old) = prev_value {
            warn!("{} is being overwritten", old);
        }
    }

    pub fn flush(&mut self) {
        let mut lock = self.file.lock().expect("variable log file lock poisoned");
        let json = serde_json::to_string(&self.variables).expect("json serialization failed");
        if let Err(e) = writeln!(lock, "{json}") {
            eprintln!("Failed to write to file: {e}");
        }
        self.variables.clear();
    }
}
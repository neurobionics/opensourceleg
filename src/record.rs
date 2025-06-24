use std::collections::HashMap;

use serde::{Deserialize, Serialize};
use serde_json::{Value};
use tracing::trace;

#[derive(Serialize, Deserialize, Debug)]
pub struct Record {
    label: String,
    variables: HashMap::<String, Value>
}

impl Record {
    pub fn new(label: String) -> Self{
        Self {
            label,
            variables: HashMap::<String, Value>::new(),
        }
    }

    pub fn insert_variable(&mut self, key: String, value: Value) {
        self.variables.insert(key, value);
    }

    pub fn flush(&mut self) {
        trace!(target: "variables", "{}",  serde_json::to_string(&self.variables).expect("error"));
        self.variables.clear();
    }
}
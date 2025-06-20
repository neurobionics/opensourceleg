use std::collections::HashMap;

use serde_json::{json, Value};
use tracing::trace;

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
        let json = json!({
            "label": self.label,
            "variables": self.variables
        });

        trace!(target: "variables", %json);

        self.variables.clear();
    }
}
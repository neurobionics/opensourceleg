use crate::record::Record;
use std::io::stdout;
use std::sync::Mutex;

use once_cell::sync::OnceCell;
use pyo3::types::{PyAnyMethods, PyDict, PyDictMethods};
use pyo3::{pyclass, pymethods, Bound, PyAny, PyResult};
use serde_json::Value;
use tracing::level_filters::LevelFilter;
use tracing::{error, info, trace, warn, Level};
use tracing::debug;
use tracing_appender::rolling;
use tracing_subscriber::{filter, fmt, layer, Registry};
use tracing_subscriber::prelude::*;
use tracing_subscriber::Layer;
use tracing_subscriber::fmt::time::{ChronoLocal};

static SUBSCRIBER: OnceCell<()> = OnceCell::new(); //OnceCell is thread-safe
static GUARD: OnceCell<tracing_appender::non_blocking::WorkerGuard> = OnceCell::new(); //OnceCell is thread-safe
static VAR_GUARD: OnceCell<tracing_appender::non_blocking::WorkerGuard> = OnceCell::new(); //OnceCell is thread-safe
static RECORD: OnceCell<Mutex<Record>> = OnceCell::new();

#[pyclass]
pub struct Logger{}

#[pymethods]
impl Logger {
    //#[new]
    #[staticmethod]
    #[pyo3(signature = (time_format = None, log_directory = None, log_name = None, print_stdout = false))]
    pub fn init(time_format: Option<String>,
                log_directory: Option<String>, log_name: Option<String>, print_stdout: bool){
        SUBSCRIBER.get_or_init(|| {
            let time = time_format.unwrap_or(String::from("%Y-%m-%d %H:%M:%S%.3f"));
            let dir = log_directory.unwrap_or(String::from("./logs"));
            let log_name = log_name.unwrap_or(String::from("logfile.log"));

            let mut layers = vec![];

            let base = fmt::layer()
                                                        .with_writer(std::io::stdout)
                                                        .with_level(true)
                                                        .with_target(false)
                                                        .with_line_number(false)
                                                        .with_ansi(true)
                                                        .with_timer(ChronoLocal::new(time.clone()));
            if print_stdout {
                layers.push(base.boxed());
            }
            let _ = RECORD.set(Mutex::new(Record::new(String::from("base"))));
            //Create file layer
            let file_appender = rolling::daily(dir, log_name);
            let (non_blocking_writer, _guard) = tracing_appender::non_blocking(file_appender);
            
            //Must keep guard in memory
            let _ = GUARD.set(_guard);
            let file_layer = fmt::layer()
                                                            .with_writer(non_blocking_writer)
                                                            .with_level(true)
                                                            .with_target(false)
                                                            .with_line_number(false)
                                                            .with_ansi(false)
                                                            .with_timer(ChronoLocal::new(time.clone()))
                                                            .with_filter(LevelFilter::TRACE);
            layers.push(file_layer.boxed());
            let filter = filter::Targets::new()
                                                        .with_target("variables" ,Level::TRACE);

            let var_fw = rolling::daily("logs", "variables.log");
            let (writer, guard) = tracing_appender::non_blocking(var_fw);
            let _ = VAR_GUARD.set(guard);
            let variable_layer = fmt::layer()
                                                                .with_writer(writer)
                                                                .with_ansi(false)
                                                                .with_level(true)
                                                                .with_target(false)
                                                                .with_line_number(false)
                                                                .with_filter(filter);
            layers.push(variable_layer.boxed());
            let subscriber = Registry::default().with(layers);

            tracing::subscriber::set_global_default(subscriber).expect("error");
        });
    }

    #[staticmethod]
    pub fn debug(msg: String) {
        debug!("{}", msg);
    }

    #[staticmethod]
    pub fn info(msg: String) {
        info!("{}", msg);
    }

    #[staticmethod]
    pub fn trace(msg: String) {
        trace!("{}", msg);
    }

    #[staticmethod]
    pub fn error(msg: String) {
        error!("{}", msg);
    }

    #[staticmethod]
    pub fn warn(msg: String) {
        warn!("{}", msg);
    }
    
    // pub fn trace_variable<T: Serialize>(name: String, value: T) {
    //     let val = serde_json::to_value(value).unwrap();
    //     if let Some(lock) = RECORD.get() {
    //         let mut record = lock.lock().unwrap();
    //         record.insert_variable(name, val);
    //     }
    // }

    #[staticmethod]
    pub fn trace_variables<'py>(dict: &Bound<'_, PyDict>) -> PyResult<()>{
        if let Some(lock) = RECORD.get() {
            let mut record = lock.lock().unwrap();

            for (key, val) in dict.iter() {
                let key_str = key.extract::<String>().expect("error");
                let json_value: Value = python_to_json_value(&val).expect("error");
                record.insert_variable(key_str, json_value);
            }
        }

        Ok(())
    }
    
    #[staticmethod]
    pub fn flush_record() {
        if let Some(lock) = RECORD.get() {
            let mut record = lock.lock().unwrap();
            record.flush();
        }
    }
}

fn python_to_json_value(value: &Bound<'_, PyAny>) -> Result<Value, String> {
    //handle nulls, bools, ints, floats, strings
    if value.is_none() {
        return Ok(Value::Null);
    }
    
    if let Ok(b) = value.extract::<bool>() {
        return Ok(Value::Bool(b));
    }
    
    if let Ok(i) = value.extract::<i64>() {
        return Ok(Value::Number(serde_json::Number::from(i)));
    }
    
    if let Ok(f) = value.extract::<f64>() {
        if f.is_finite() {
            if let Some(num) = serde_json::Number::from_f64(f) {
                return Ok(Value::Number(num));
            }
        }
        return Ok(Value::String(f.to_string()));
    }
    
    if let Ok(s) = value.extract::<String>() {
        return Ok(Value::String(s));
    }

    Ok(Value::Null)
}
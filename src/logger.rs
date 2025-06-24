use crate::record::Record;
use std::sync::Mutex;

use once_cell::sync::OnceCell;
use pyo3::types::{PyAnyMethods, PyDict, PyDictMethods, PyStringMethods};
use pyo3::{pyclass, pymethods, Bound, FromPyObject, PyAny, PyResult};
use serde_json::Value;
use tracing::level_filters::LevelFilter;
use tracing::{error, info, trace, warn, Level};
use tracing::debug;
use tracing_appender::non_blocking::{NonBlocking};
use tracing_appender::rolling;
use tracing_subscriber::fmt::format::{DefaultFields, Format};
use tracing_subscriber::{filter, fmt, Registry};
use tracing_subscriber::prelude::*;
use tracing_subscriber::Layer;
use tracing_subscriber::fmt::time::{ChronoLocal};

static SUBSCRIBER: OnceCell<()> = OnceCell::new(); //OnceCell is thread-safe
static GUARD: OnceCell<tracing_appender::non_blocking::WorkerGuard> = OnceCell::new(); //OnceCell is thread-safe
static VAR_GUARD: OnceCell<tracing_appender::non_blocking::WorkerGuard> = OnceCell::new(); //OnceCell is thread-safe
static STDOUT_GUARD: OnceCell<tracing_appender::non_blocking::WorkerGuard> = OnceCell::new();
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
            let _ = RECORD.set(Mutex::new(Record::new()));

            if print_stdout {
                layers.push(create_stdout_layer(time.clone()).boxed());
            }

            layers.push(create_file_layer(dir.clone(), log_name, time).boxed());
            layers.push(create_variable_layer(dir).boxed());
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

    #[staticmethod]
    pub fn trace_variables<'py>(dict: &Bound<'_, PyDict>) -> PyResult<()>{
        if let Some(lock) = RECORD.get() {
            let mut record = lock.lock().unwrap();
            for (key, val) in dict.iter() {
                let val_str = if let Ok(v) = val.extract::<bool>() {
                    serde_json::to_value(v).expect("Error converting to JSON Value")
                }
                else if let Ok(v) = val.extract::<i64>() {
                    serde_json::to_value(v).expect("Error converting to JSON Value")
                }
                else if let Ok(v) = val.extract::<f64>() {
                    serde_json::to_value(v).expect("Error converting to JSON Value")
                }
                else if let Ok(v) = val.extract::<&str>() {
                    serde_json::to_value(v).expect("Error converting to JSON Value")
                }
                else {
                    let repr = val.str()?.to_str()?.to_owned();
                    serde_json::to_value(repr).expect("Error converting to JSON Value")
                };

                let key_str = key.extract::<&str>()?.to_owned();
                record.insert_variable(key_str, val_str);
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

fn create_stdout_layer(time: String) -> tracing_subscriber::fmt::Layer<
    tracing_subscriber::Registry,
    tracing_subscriber::fmt::format::DefaultFields,
    Format<tracing_subscriber::fmt::format::Full, ChronoLocal>,
    tracing_appender::non_blocking::NonBlocking
> {

    let (non_blocking_stdout, guard_stdout) = tracing_appender::non_blocking(std::io::stdout());
    let _ = STDOUT_GUARD.set(guard_stdout);

    let base = fmt::layer()
                                                .with_writer(non_blocking_stdout)
                                                .with_level(true)
                                                .with_target(false)
                                                .with_line_number(false)
                                                .with_ansi(true)
                                                .with_timer(ChronoLocal::new(time));
    base
}

fn create_file_layer(dir: String, log_name: String, time: String) -> filter::Filtered<
    fmt::Layer<Registry, DefaultFields, Format<fmt::format::Full, ChronoLocal>, NonBlocking>,
    LevelFilter,
    Registry
>{
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
    file_layer
}

fn create_variable_layer(dir: String) -> filter::Filtered<
    fmt::Layer<Registry,DefaultFields, Format, NonBlocking>,
    filter::Targets,
    Registry
>{
    let filter = filter::Targets::new()
                                                .with_target("variables" ,Level::TRACE);

    let var_fw = rolling::daily(dir, "variables.log");
    let (writer, guard) = tracing_appender::non_blocking(var_fw);
    let _ = VAR_GUARD.set(guard);
    let variable_layer = fmt::layer()
                                                        .with_writer(writer)
                                                        .with_ansi(false)
                                                        .with_level(true)
                                                        .with_target(false)
                                                        .with_line_number(false)
                                                        .with_filter(filter);
    variable_layer                          
}
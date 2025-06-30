use crate::record::Record;
use crate::rotator::RotatingFileWriter;
use std::fs;
use std::path::Path;
use std::sync::Mutex;

use once_cell::sync::OnceCell;
use pyo3::types::{PyAnyMethods, PyDict, PyDictMethods, PyStringMethods};
use pyo3::{pyclass, pymethods, Bound, PyResult};
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

static GUARD: OnceCell<tracing_appender::non_blocking::WorkerGuard> = OnceCell::new(); //OnceCell is thread-safe
static STDOUT_GUARD: OnceCell<tracing_appender::non_blocking::WorkerGuard> = OnceCell::new();
static RECORD: OnceCell<Mutex<Record>> = OnceCell::new();

#[pyclass(name = "LogLevel")]
#[derive(Clone)]
pub enum PyLogLevel {
    TRACE,
    DEBUG,
    INFO,
    WARN,
    ERROR
}

impl From<PyLogLevel> for LevelFilter {
    fn from(level: PyLogLevel) -> Self {
        match level {
            PyLogLevel::TRACE => LevelFilter::TRACE,
            PyLogLevel::DEBUG => LevelFilter::DEBUG,
            PyLogLevel::INFO => LevelFilter::INFO,
            PyLogLevel::WARN => LevelFilter::WARN,
            PyLogLevel::ERROR => LevelFilter::ERROR
        }
    }
}

#[pyclass]
pub struct Logger{}

#[pymethods]
impl Logger {
    //#[new]
    #[staticmethod]
    #[pyo3(signature = (time_format = None, log_directory = None, log_name = None, print_stdout = false, file_max_bytes = 0, backup_count = 0,
                        stdout_level = None, logfile_level = None))]
    pub fn init(time_format: Option<String>,
                log_directory: Option<String>, log_name: Option<String>, print_stdout: bool,
                file_max_bytes: u64, backup_count: u64, stdout_level: Option<PyLogLevel>,
                logfile_level: Option<PyLogLevel>){

        let time = time_format.unwrap_or(String::from("%Y-%m-%d %H:%M:%S%.3f"));
        let dir = log_directory.unwrap_or(String::from("./logs"));
        let log_name = log_name.unwrap_or(String::from("logfile.log"));
        let console_level = stdout_level.map_or(LevelFilter::TRACE, |level|level.into());
        let log_level = logfile_level.map_or(LevelFilter::TRACE, |level| level.into());

        let mut layers = vec![];
        let path = Path::new(&dir).join("variables.log");

        if let Some(parent) = path.parent() {
            if parent.exists() {
                let _ = fs::remove_dir_all(parent);
            }
            let _ = fs::create_dir_all(parent);
        }

        let path_str = path.to_str().expect("error").to_owned();
        let _ = RECORD.set(Mutex::new(Record::new(path_str)));

        if print_stdout {
            layers.push(create_stdout_layer(time.clone(), console_level).boxed());
        }

        layers.push(create_file_layer(dir.clone(), log_name, time, file_max_bytes, backup_count, log_level).boxed());
        let subscriber = Registry::default().with(layers);

        tracing::subscriber::set_global_default(subscriber).expect("error");
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
    pub fn record() {
        if let Some(lock) = RECORD.get() {
            let mut record = lock.lock().unwrap();
            record.record_variables();
        }
    }

    #[staticmethod]
    pub fn flush_all() {
        if let Some(lock) = RECORD.get() {
            let mut record = lock.lock().unwrap();
            record.flush_buffer();
        }
    }
}

fn create_stdout_layer(time: String, level: LevelFilter) -> filter::Filtered<fmt::Layer<Registry, DefaultFields, Format<fmt::format::Full, ChronoLocal>, NonBlocking>, LevelFilter, Registry>{

    let (non_blocking_stdout, guard_stdout) = tracing_appender::non_blocking(std::io::stdout());
    let _ = STDOUT_GUARD.set(guard_stdout);

    let base = fmt::layer()
                                                
                                                .with_writer(non_blocking_stdout)
                                                .with_level(true)
                                                .with_target(false)
                                                .with_line_number(false)
                                                .with_ansi(true)
                                                .with_timer(ChronoLocal::new(time))
                                                .with_filter(level);
    base
}

fn create_file_layer(dir: String, log_name: String, time: String, file_max_size: u64, backup_count: u64, log_level: LevelFilter) -> filter::Filtered<
    fmt::Layer<Registry, DefaultFields, Format<fmt::format::Full, ChronoLocal>, NonBlocking>,
    LevelFilter,
    Registry
>{
    //Create file layer
    let file_appender = RotatingFileWriter::new(Path::new(&dir).join(String::from(log_name)).to_path_buf(), file_max_size, backup_count);
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
                                                    .with_filter(log_level);
    file_layer
}
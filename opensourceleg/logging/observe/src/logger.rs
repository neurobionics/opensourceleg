use crate::record::Record;
use crate::rotator::RotatingFileWriter;
use std::fs;
use std::path::Path;
use std::sync::{Mutex, OnceLock};

use chrono::Utc;
use once_cell::sync::OnceCell;
use pyo3::types::{PyAnyMethods, PyBool, PyDict, PyDictMethods, PyFloat, PyInt, PyList, PyString, PyStringMethods};
use pyo3::{pyclass, pymethods, Bound, PyAny, PyErr, PyResult, Python};
use serde::{Deserialize, Serialize};
use serde_json::Value;
use tracing::level_filters::LevelFilter;
use tracing::{error, info, trace, warn};
use tracing::debug;
use tracing_appender::non_blocking::{NonBlocking};
use tracing_subscriber::fmt::format::{DefaultFields, Format};
use tracing_subscriber::{filter, fmt, Registry};
use tracing_subscriber::prelude::*;
use tracing_subscriber::Layer;
use tracing_subscriber::fmt::time::{ChronoLocal};

static GUARD: OnceCell<tracing_appender::non_blocking::WorkerGuard> = OnceCell::new(); //OnceCell is thread-safe
static STDOUT_GUARD: OnceCell<tracing_appender::non_blocking::WorkerGuard> = OnceCell::new();
static RECORD: OnceCell<Mutex<Record>> = OnceCell::new();
static SESSION: OnceLock<Mutex<bool>> = OnceLock::new();

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

#[derive(Serialize, Deserialize, Debug)]
struct StringMsg {
    data: String,
}

#[pyclass]
pub struct Logger{}

#[pymethods]
impl Logger {
    //#[new]
    #[staticmethod]
    #[pyo3(signature = (log_directory = None, log_name = None, print_stdout = false, file_max_bytes = 0, backup_count = 0,
                        stdout_level = None, logfile_level = None))]
    pub fn init(
                log_directory: Option<String>, log_name: Option<String>, print_stdout: bool,
                file_max_bytes: u64, backup_count: u64, stdout_level: Option<PyLogLevel>,
                logfile_level: Option<PyLogLevel>){

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

        let _ = RECORD.set(Mutex::new(Record::new(path)));

        if print_stdout {
            layers.push(create_stdout_layer(console_level).boxed());
        }

        layers.push(create_file_layer(dir.clone(), log_name, file_max_bytes, backup_count, log_level).boxed());
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
        let mut variables = Vec::new();
        for (key, val) in dict.iter() {
            let key_str = key.extract::<&str>()?.to_owned();
            let val_str = downcast(val);

            variables.push((key_str, val_str));
        }

        Python::with_gil(|py| {
            py.allow_threads(|| {
                if let Some(lock) = RECORD.get() {
                    let mut record = lock.lock().unwrap();
                    for (key, val) in variables {
                        record.insert_variable(key, val);
                    }
                }
            })
        });
        Ok(())
    }

    #[staticmethod]
    pub fn track_functions<'py>(dict: &Bound<'_, PyDict>) -> PyResult<()>{
        let mut record = RECORD.get().unwrap().lock().unwrap();
        for (name, function) in dict.iter() {
            if !function.is_callable() {
                return Err(PyErr::new::<pyo3::exceptions::PyTypeError, _>(
                    format!("Value for '{}' is not a callable function", name)
                ));
            }
            record.insert_function(name.to_string(), function.into());
        }
        Ok(())
    }

    #[staticmethod]
    pub fn untrack_functions<'py>(list: &Bound<'_, PyList>) -> PyResult<()>{
        let mut record = RECORD.get().unwrap().lock().unwrap();
        for item in list {
            let id: &str = item.extract()?;
            record.remove_function(id.to_string());
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

    #[staticmethod]
    fn start_ros_subscriber(config_file: String, key_expr: String) {
        let mutex = SESSION.get_or_init(|| Mutex::new(false));
        let mut guard = mutex.lock().unwrap();
        if *guard {
            return;
        }

        std::thread::spawn(|| {
            let rt = tokio::runtime::Runtime::new().unwrap();
            rt.block_on(async {
                let contents = fs::read_to_string(String::from(config_file)).expect("Error reading from file");
                let config = match zenoh::config::Config::from_json5(&contents) {
                    Ok(config) => {
                        info!("Loaded config from JSON5 parameter");
                        config
                    },
                    Err(e) => {
                        warn!("Failed to parse JSON5 config: {}, using default", e);
                        zenoh::config::Config::default()
                    }
                };
                let session = zenoh::open(config).await.unwrap();
                let subscriber = session
                                                .declare_subscriber(key_expr)
                                                .await
                                                .unwrap();
                info!("Zenoh subscriber started");

                while let Ok(sample) = subscriber.recv_async().await {
                    let payload = sample.payload().to_bytes().into_owned();

                    match cdr::deserialize::<StringMsg>(&payload) {
                        Ok(msg) => {
                            info!("Received ROS2 String on '{}': {}", sample.key_expr(), msg.data);
                        }
                        Err(e) => {
                            warn!(
                                "Failed to decode CDR for key '{}': {}. Raw: {:02X?}",
                                sample.key_expr(),
                                e,
                                payload
                            );
                        }
                    }
                }
            });
        });
        *guard = true;
    }
}

fn create_stdout_layer(level: LevelFilter) -> filter::Filtered<fmt::Layer<Registry, DefaultFields, Format, NonBlocking>, LevelFilter, Registry>{

    let (non_blocking_stdout, guard_stdout) = tracing_appender::non_blocking(std::io::stdout());
    let _ = STDOUT_GUARD.set(guard_stdout);

    let base = fmt::layer()
                                                
                                                .with_writer(non_blocking_stdout)
                                                .with_level(true)
                                                .with_target(false)
                                                .with_line_number(false)
                                                .with_ansi(true)
                                                .with_filter(level);
    base
}

fn create_file_layer(dir: String, log_name: String, file_max_size: u64, backup_count: u64, log_level: LevelFilter) -> filter::Filtered<
    fmt::Layer<Registry, DefaultFields, Format, NonBlocking>,
    LevelFilter,
    Registry
>{
    //Create file layer
    let file_appender = RotatingFileWriter::new(Path::new(&dir).join(String::from(log_name)).to_path_buf(), file_max_size, backup_count);
    let (non_blocking_writer, _guard) = tracing_appender::non_blocking(file_appender);
    
    //Must keep guard in memory
    let _ = GUARD.set(_guard);
    let file_layer: filter::Filtered<fmt::Layer<_, DefaultFields, Format, NonBlocking>, LevelFilter, _>  = fmt::layer()
                                                    .with_writer(non_blocking_writer)
                                                    .with_level(true)
                                                    .with_target(false)
                                                    .with_line_number(false)
                                                    .with_ansi(false)
                                                    .with_filter(log_level);
    file_layer
}

pub fn downcast(val: Bound<'_, PyAny>) -> Value {
    if let Ok(v) = val.downcast::<PyBool>() {
        serde_json::to_value(v.extract::<bool>().expect("Error converting to JSON Value")).expect("Error converting to JSON Value")
    }
    else if let Ok(v) = val.downcast::<PyInt>() {
        serde_json::to_value(v.extract::<i64>().expect("Error extracting i64 from PyInt")).expect("Error converting to JSON Value")
    }
    else if let Ok(v) = val.downcast::<PyFloat>() {
        serde_json::to_value(v.extract::<f64>().expect("Error extracting f64 from PyFloat")).expect("Error converting to JSON Value")
    }
    else if let Ok(v) = val.downcast::<PyString>() {
        serde_json::to_value(v.to_str().expect("Error converting PyString to &str")).expect("Error converting to JSON Value")
    }
    else {
        let repr = val.str().expect("error").to_str().expect("error").to_owned();
        serde_json::to_value(repr).expect("Error converting to JSON Value")
    }
}
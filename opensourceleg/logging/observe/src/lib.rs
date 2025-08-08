pub mod logger;
pub mod record;
pub mod rotator;
pub mod profiler;
pub mod core;
use pyo3::{prelude::*};

use crate::logger::{Logger, PyLogLevel};

#[pymodule]
fn observable(py: Python, m: &Bound<'_, PyModule>) -> PyResult<()>{
    m.add_class::<Logger>()?;
    m.add_class::<PyLogLevel>()?;
    m.add_class::<profiler::PyProfiler>()?;

    let atexit = py.import("atexit")?;
    let logger_class = m.getattr("Logger")?;
    let flush_all_method = logger_class.getattr("flush_all")?;
    atexit.call_method1("register", (flush_all_method,))?;
    Ok(())
}
pub mod logger;
pub mod record;
pub mod rotator;
use pyo3::{prelude::*};

use crate::logger::{Logger, PyLogLevel};

#[pymodule]
fn profiler(py: Python, m: &Bound<'_, PyModule>) -> PyResult<()>{
    m.add_class::<Logger>()?;
    m.add_class::<PyLogLevel>()?;

    let atexit = py.import("atexit")?;
    let logger_class = m.getattr("Logger")?;
    let flush_all_method = logger_class.getattr("flush_all")?;
    atexit.call_method1("register", (flush_all_method,))?;
    Ok(())
}
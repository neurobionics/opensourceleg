use pyo3::prelude::*;
use crate::logger::{Logger, PyLogLevel};
pub mod logger;
pub mod record;
pub mod rotator;
pub mod profiler;
pub mod core;
mod math;

/// High-performance math operations for robotics
#[pymodule]
fn opensourceleg_rs(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add("__version__", "3.1.0")?;

    // Add math submodule
    let math_module = PyModule::new(m.py(), "math")?;
    math::register_module(&math_module)?;
    m.add_submodule(&math_module)?;

    m.add_class::<Logger>()?;
    m.add_class::<PyLogLevel>()?;
    m.add_class::<profiler::PyProfiler>()?;

    let atexit = m.py().import("atexit")?;
    let logger_class = m.getattr("Logger")?;
    let flush_all_method = logger_class.getattr("flush_all")?;
    atexit.call_method1("register", (flush_all_method,))?;

    Logger::init(None, None, 0, 0, None, None);

    Ok(())
}

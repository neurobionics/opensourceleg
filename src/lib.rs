pub mod logger;
pub mod record;
pub mod rotator;
use pyo3::{prelude::*};

use crate::logger::{Logger, PyLogLevel};

#[pymodule]
fn profiler(m: &Bound<'_, PyModule>) -> PyResult<()>{
    m.add_class::<Logger>()?;
    m.add_class::<PyLogLevel>()?;
    Ok(())
}
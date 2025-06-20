pub mod logger;
pub mod record;
use pyo3::{prelude::*};

use crate::logger::Logger;

#[pymodule]
fn profiler(m: &Bound<'_, PyModule>) -> PyResult<()>{
    m.add_class::<Logger>()?;
    Ok(())
}
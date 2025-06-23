use pyo3::prelude::*;

mod math;

/// High-performance math operations for robotics
#[pymodule]
fn rust(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add("__version__", "3.1.0")?;

    // Add math submodule
    let math_module = PyModule::new_bound(m.py(), "math")?;
    math::register_module(&math_module)?;
    m.add_submodule(&math_module)?;

    Ok(())
}

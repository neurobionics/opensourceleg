use pyo3::prelude::*;
use nalgebra::{Vector3, UnitQuaternion};
use numpy::PyReadonlyArray1;

/// Fast quaternion to rotation matrix conversion
#[pyfunction]
fn quat_to_rotation_matrix(quat: PyReadonlyArray1<f64>) -> PyResult<Vec<Vec<f64>>> {
    let quat_slice = quat.as_slice()?;
    if quat_slice.len() != 4 {
        return Err(PyErr::new::<pyo3::exceptions::PyValueError, _>(
            "Quaternion must have 4 elements [w, x, y, z]"
        ));
    }

    let q = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
        quat_slice[0], quat_slice[1], quat_slice[2], quat_slice[3]
    ));

    let rot_matrix = q.to_rotation_matrix();
    let matrix = rot_matrix.matrix();

    Ok(vec![
        vec![matrix[(0, 0)], matrix[(0, 1)], matrix[(0, 2)]],
        vec![matrix[(1, 0)], matrix[(1, 1)], matrix[(1, 2)]],
        vec![matrix[(2, 0)], matrix[(2, 1)], matrix[(2, 2)]],
    ])
}

/// Fast vector cross product
#[pyfunction]
fn cross_product(a: PyReadonlyArray1<f64>, b: PyReadonlyArray1<f64>) -> PyResult<Vec<f64>> {
    let a_slice = a.as_slice()?;
    let b_slice = b.as_slice()?;

    if a_slice.len() != 3 || b_slice.len() != 3 {
        return Err(PyErr::new::<pyo3::exceptions::PyValueError, _>(
            "Vectors must have 3 elements"
        ));
    }

    let vec_a = Vector3::new(a_slice[0], a_slice[1], a_slice[2]);
    let vec_b = Vector3::new(b_slice[0], b_slice[1], b_slice[2]);
    let result = vec_a.cross(&vec_b);

    Ok(vec![result.x, result.y, result.z])
}

/// Fast magnitude calculation
#[pyfunction]
fn magnitude(vec: PyReadonlyArray1<f64>) -> PyResult<f64> {
    let vec_slice = vec.as_slice()?;
    let sum_squares: f64 = vec_slice.iter().map(|&x| x * x).sum();
    Ok(sum_squares.sqrt())
}

/// Normalize a vector
#[pyfunction]
fn normalize(vec: PyReadonlyArray1<f64>) -> PyResult<Vec<f64>> {
    let vec_slice = vec.as_slice()?;

    // Calculate magnitude directly from slice to avoid borrowing issues
    let sum_squares: f64 = vec_slice.iter().map(|&x| x * x).sum();
    let mag = sum_squares.sqrt();

    if mag == 0.0 {
        return Err(PyErr::new::<pyo3::exceptions::PyValueError, _>(
            "Cannot normalize zero vector"
        ));
    }

    Ok(vec_slice.iter().map(|&x| x / mag).collect())
}

pub fn register_module(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(quat_to_rotation_matrix, m)?)?;
    m.add_function(wrap_pyfunction!(cross_product, m)?)?;
    m.add_function(wrap_pyfunction!(magnitude, m)?)?;
    m.add_function(wrap_pyfunction!(normalize, m)?)?;
    Ok(())
}

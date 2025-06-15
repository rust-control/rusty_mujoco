//! ref: <https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#parse-and-compile>

#![allow(non_snake_case)]

use crate::{MjError, MjModel};

/// Parse XML file in MJCF or URDF format, compile it, return low-level model.
pub fn mj_loadXML(
    filename: impl AsRef<std::path::Path>,
) -> Result<MjModel, MjError> {
    let filename = std::ffi::CString::new(
        filename.as_ref().as_os_str().as_encoded_bytes()
    ).map_err(MjError::from_error)?;

    let mut error = MjError::init();
    let model_ptr = unsafe {
        let (err_ptr, err_len) = error.as_parts();
        crate::bindgen::mj_loadXML(
            filename.as_ptr(),
            std::ptr::null_mut(), // no vfs
            err_ptr,
            err_len,
        )
    };
    if model_ptr.is_null() {
        return Err(error);
    }

    Ok(MjModel(unsafe {*model_ptr}))
}

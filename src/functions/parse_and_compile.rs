//! <https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#parse-and-compile>

#![allow(non_snake_case)]

use crate::{MjError, MjSpec, MjModel, MjData};

/// Parse XML file in MJCF or URDF format, compile it, return low-level model.
pub fn mj_loadXML(filename: impl Into<Vec<u8>>) -> Result<MjModel, MjError> {
    let filename = std::ffi::CString::new(filename).map_err(MjError::from_error)?;

    let mut error = MjError::init();
    let model_ptr = unsafe {
        let (err_ptr, err_len) = error.as_parts();
        crate::bindgen::mj_loadXML(
            filename.as_ptr(),
            std::ptr::null(), // no vfs
            err_ptr,
            err_len,
        )
    };

    if model_ptr.is_null() {
        Err(error)
    } else {
        Ok((unsafe {*model_ptr}).into())
    }
}

/// Parse spec from XML file.
pub fn mj_parseXML(filename: impl Into<Vec<u8>>) -> Result<MjSpec, MjError> {
    let filename = std::ffi::CString::new(filename).map_err(MjError::from_error)?;

    let mut error = MjError::init();
    let model_ptr = unsafe {
        let (err_ptr, err_len) = error.as_parts();
        crate::bindgen::mj_parseXML(
            filename.as_ptr(),
            std::ptr::null(), // no vfs
            err_ptr,
            err_len,
        )
    };

    if model_ptr.is_null() {
        Err(error)
    } else {
        Ok((unsafe {*model_ptr}).into())
    }
}

/// Parse spec from XML string.
pub fn mj_parseXMLString(xml: &'static str) -> Result<MjSpec, MjError> {
    let xml = std::ffi::CString::new(xml).map_err(MjError::from_error)?;

    let mut error = MjError::init();
    let model_ptr = unsafe {
        let (err_ptr, err_len) = error.as_parts();
        crate::bindgen::mj_parseXMLString(
            xml.as_ptr(),
            std::ptr::null(), // no vfs
            err_ptr,
            err_len,
        )
    };

    if model_ptr.is_null() {
        Err(error)
    } else {
        Ok((unsafe {*model_ptr}).into())
    }
}

/// Compile mjSpec to mjModel. A spec can be edited and compiled multiple times,
/// returning a new mjModel instance that takes the edits into account.
/// If compilation fails, `mj_compile` returns NULL; the error can be read with `mjs_getError`.
pub fn mj_compile(s: &mut MjSpec) -> Option<MjModel> {
    let model_ptr = unsafe { crate::bindgen::mj_compile(s.as_mut(), std::ptr::null()) };

    if model_ptr.is_null() {
        None
    } else {
        Some((unsafe {*model_ptr}).into())
    }
}

/// Recompile spec to model, preserving the state. Like `mj_compile`, this function
/// compiles an mjSpec to an mjModel, with two differences.
/// First, rather than returning an entirely new model, it will
/// reallocate existing mjModel and mjData instances in-place.
/// Second, it will preserve the integration state, as given in the
/// provided mjData instance, while accounting for newly added or removed degrees
/// of freedom. This allows the user to continue simulation with the same
/// model and data struct pointers while editing the model programmatically.
/// 
/// `mj_recompile` returns 0 if compilation succeed. In the case of failure,
/// the given mjModel and mjData instances will be deleted; as in `mj_compile`,
/// the compilation error can be read with `mjs_getError`.
pub fn mj_recompile(s: &mut MjSpec, m: &mut MjModel, d: &mut MjData) -> Result<(), ()> {
    let status = unsafe {
        crate::bindgen::mj_recompile(
            s.as_mut(),
            std::ptr::null(),
            m.as_mut(),
            d.as_mut(),
        )
    };

    // <https://github.com/google-deepmind/mujoco/blob/f75772587ac08a71dc4fe3a9cd9fdffd6ec2e2be/src/user/user_api.cc#L92-L93>
    // return 0 on success
    if status == 0 {Ok(())} else {Err(())}
}

/// Update XML data structures with info from low-level model
/// created with `mj_loadXML`, save as MJCF.
/// 
/// Note that this function only saves models that have been loaded with `mj_loadXML`,
/// the legacy loading mechanism. See the [model editing](https://mujoco.readthedocs.io/en/stable/programming/modeledit.html#meoverview)
/// chapter to understand the difference between the
/// old and new model loading and saving mechanisms.
pub fn mj_saveLastXML(
    filename: impl Into<Vec<u8>>,
    m: &MjModel,
) -> Result<(), MjError> {
    let filename = std::ffi::CString::new(filename).map_err(MjError::from_error)?;

    let mut error = MjError::init();
    let status = unsafe {
        let (err_ptr, err_len) = error.as_parts();
        crate::bindgen::mj_saveLastXML(
            filename.as_ptr(),
            m.as_ref(),
            err_ptr,
            err_len,
        )
    };

    // <https://github.com/google-deepmind/mujoco/blob/84ad22a5905a7d0b4e2e67ca8bb13ea90b6f74ef/src/xml/xml_api.cc#L131-L134>
    // returns 1 if successful, 0 otherwise
    if status == 1 {Ok(())} else {Err(error)}
}

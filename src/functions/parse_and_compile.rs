//! # [Parse and compile](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#parse-and-compile)
//! 
//! The key function here is [`mj_loadXML`]. It invokes the built-in parser
//! and compiler, and either returns a pointer to a valid mjModel,
//! or NULL - in which case the user should check the error information
//! in the user-provided string. The model and all files referenced in it
//! can be loaded from disk.

use crate::{MjError, mjSpec, mjModel, mjData};

/// Parse XML file in MJCF or URDF format, compile it, return low-level model.
pub fn mj_loadXML(filename: impl Into<String>) -> Result<mjModel, MjError> {
    let filename = std::ffi::CString::new(filename.into()).map_err(MjError::from_error)?;

    let mut error = MjError::init();
    let c_ptr = unsafe {
        let (err_ptr, err_len) = error.as_parts();
        crate::bindgen::mj_loadXML(
            filename.as_ptr(),
            std::ptr::null(), // no vfs
            err_ptr,
            err_len,
        )
    };

    if c_ptr.is_null() {
        Err(error)
    } else {
        // SAFETY:
        // 
        // - `c_ptr` is valid pointer to `mjModel`
        // - when the returned `mjModel` is dropped, it calls `mj_deleteModel`.
        //   then and only then the `mjModel`'s memory will be freed by Rust.
        Ok(unsafe { std::ptr::read(c_ptr) })
    }
}

/// Parse spec from XML file.
pub fn mj_parseXML(filename: impl Into<String>) -> Result<mjSpec, MjError> {
    let filename = std::ffi::CString::new(filename.into()).map_err(MjError::from_error)?;

    let mut error = MjError::init();
    let c_ptr = unsafe {
        let (err_ptr, err_len) = error.as_parts();
        crate::bindgen::mj_parseXML(
            filename.as_ptr(),
            std::ptr::null(), // no vfs
            err_ptr,
            err_len,
        )
    };

    if c_ptr.is_null() {
        Err(error)
    } else {
        // SAFETY:
        // 
        // - `c_ptr` is valid pointer to `mjSpec`
        // - when the returned `mjModel` is dropped, it calls `mj_deleteSpec`.
        //   then and only then the `mjModel`'s memory will be freed by Rust.
        Ok(unsafe { std::ptr::read(c_ptr) })
    }
}

/// Parse spec from XML string.
pub fn mj_parseXMLString(xml: impl Into<String>) -> Result<mjSpec, MjError> {
    let xml = std::ffi::CString::new(xml.into()).map_err(MjError::from_error)?;

    let mut error = MjError::init();
    let c_ptr = unsafe {
        let (err_ptr, err_len) = error.as_parts();
        crate::bindgen::mj_parseXMLString(
            xml.as_ptr(),
            std::ptr::null(), // no vfs
            err_ptr,
            err_len,
        )
    };

    if c_ptr.is_null() {
        Err(error)
    } else {
        // SAFETY:
        // 
        // - `c_ptr` is valid pointer to `mjSpec`
        // - when the returned `mjModel` is dropped, it calls `mj_deleteSpec`.
        //   then and only then the `mjModel`'s memory will be freed by Rust.
        Ok(unsafe { std::ptr::read(c_ptr) })
    }
}

/// Compile mjSpec to mjModel. A spec can be edited and compiled multiple times,
/// returning a new mjModel instance that takes the edits into account.
/// If compilation fails, `mj_compile` returns NULL; the error can be read with `mjs_getError`.
pub fn mj_compile(s: &mut mjSpec) -> Option<mjModel> {
    let c_ptr = unsafe { crate::bindgen::mj_compile(s, std::ptr::null()) };

    if c_ptr.is_null() {
        None
    } else {
        // SAFETY:
        // 
        // - `c_ptr` is valid pointer to `mjModel`
        // - when the returned `mjModel` is dropped, it calls `mj_deleteModel`.
        //   then and only then the `mjModel`'s memory will be freed by Rust.
        Some(unsafe { std::ptr::read(c_ptr) })
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
pub fn mj_recompile(s: &mut mjSpec, m: &mut mjModel, d: &mut mjData) -> Result<(), ()> {
    let status = unsafe {
        crate::bindgen::mj_recompile(
            s,
            std::ptr::null(),
            m,
            d,
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
    filename: impl Into<String>,
    m: &mjModel,
) -> Result<(), MjError> {
    let filename = std::ffi::CString::new(filename.into()).map_err(MjError::from_error)?;

    let mut error = MjError::init();
    let status = unsafe {
        let (err_ptr, err_len) = error.as_parts();
        crate::bindgen::mj_saveLastXML(
            filename.as_ptr(),
            m,
            err_ptr,
            err_len,
        )
    };

    // <https://github.com/google-deepmind/mujoco/blob/84ad22a5905a7d0b4e2e67ca8bb13ea90b6f74ef/src/xml/xml_api.cc#L131-L134>
    // returns 1 if successful, 0 otherwise
    if status == 1 {Ok(())} else {Err(error)}
}

/// Free last XML model if loaded. Called internally at each load.
pub fn mj_freeLastXML() {
    unsafe {
        crate::bindgen::mj_freeLastXML();
    }
}

/// Save spec to XML string.
/// XML saving automatically compiles the spec before saving.
pub fn mj_saveXMLString(s: &mjSpec) -> Result<String, MjError> {
    fn proc_inner(
        s: &mjSpec,
        output_buffer: &mut Vec<u8>,
    ) -> (i32, MjError) {
        let mut error = MjError::init();
        let status = unsafe {
            let (err_ptr, err_len) = error.as_parts();
            crate::bindgen::mj_saveXMLString(
                s,
                output_buffer.as_mut_ptr() as *mut std::ffi::c_char,
                output_buffer.len() as i32,
                err_ptr,
                err_len,
            )
        };
        (status, error)
    }

    #[cold]
    #[inline(never)]
    fn retry(
        s: &mjSpec,
        output_buffer: &mut Vec<u8>,
    ) -> (i32, MjError) {
        proc_inner(s, output_buffer)
    }

    let mut output_buffer = vec![0u8; 1 << 12];
    let (mut status, mut error) = proc_inner(s, &mut output_buffer);

    // <https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-savexmlstring>
    // return 0 on success, -1 on failure.
    // If the length of the output buffer is too small, returns the required size.
    if status > 0 {
        output_buffer.extend(vec![0u8; status as usize - output_buffer.len()]);
        (status, error) = retry(s, &mut output_buffer);
    }
    if status == 0 {
        Ok(String::from_utf8(output_buffer).map_err(MjError::from_error)?)
    } else {
        #[cfg(debug_assertions)] {
            assert_eq!(status, -1, "unexpected status of `mj_saveXMLString`");
        }
        Err(error)
    }
}

/// Save spec to XML file. XML saving requires that the spec first be compiled.
pub fn mj_saveXML(
    s: &mjSpec,
    filename: impl Into<String>,
) -> Result<(), MjError> {
    let filename = std::ffi::CString::new(filename.into()).map_err(MjError::from_error)?;

    let mut error = MjError::init();
    let status = unsafe {
        let (err_ptr, err_len) = error.as_parts();
        crate::bindgen::mj_saveXML(
            s,
            filename.as_ptr(),
            err_ptr,
            err_len,
        )
    };

    // <https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-savexml>
    // return 0 on success, -1 otherwise
    if status == 0 {Ok(())} else {Err(error)}
}

//! # [Error and memory](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#error-and-memory)

/// Main error function; does not return to caller.
/* void mju_error(const char* msg, ...) mjPRINTFLIKE(1, 2); */
pub fn mju_error(msg: impl Into<String>) -> ! {
    let msg = std::ffi::CString::new(msg.into()).expect("Error message must be valid UTF-8");
    unsafe { crate::bindgen::mju_error(msg.as_ptr()) }
    unreachable!()
}

/// Main warning function; does not return to caller.
/* void mju_warning(const char* msg, ...) mjPRINTFLIKE(1, 2); */
pub fn mju_warning(msg: impl Into<String>) -> ! {
    let msg = std::ffi::CString::new(msg.into()).expect("Warning message must be valid UTF-8");
    unsafe { crate::bindgen::mju_warning(msg.as_ptr()) }
    unreachable!()
}

/// Clear user error and memory handlers.
/* void mju_clearHandlers(void); */
pub fn mju_clearHandlers() {
    unsafe { crate::bindgen::mju_clearHandlers() }
}

/// Allocate memory; byte-align on 64; pad size to multiple of 64.
/* void* mju_malloc(size_t size); */
pub fn mju_malloc(size: usize) -> *mut u8 {
    unsafe { crate::bindgen::mju_malloc(size) as *mut u8 }
}

/// Free memory, using free() by default.
/* void mju_free(void* ptr); */
pub fn mju_free(ptr: *mut u8) {
    unsafe { crate::bindgen::mju_free(ptr as *mut std::ffi::c_void) }
}

/// High-level warning function: count warnings in mjData, print only the first.
/* void mj_warning(mjData* d, int warning, int info); */
pub fn mj_warning(
    d: &mut crate::MjData,
    warning: crate::bindgen::mjtWarning,
    info: usize,
) {
    unsafe { crate::bindgen::mj_warning(d.as_mut(), warning as i32, info as i32) }
}

/// Write [datetime, type: message] to MUJOCO_LOG.TXT.
/* void mju_writeLog(const char* type, const char* msg); */
pub fn mju_writeLog(kind: impl Into<String>, msg: impl Into<String>) {
    let kind = std::ffi::CString::new(kind.into()).expect("Log kind must be valid UTF-8");
    let msg = std::ffi::CString::new(msg.into()).expect("Log message must be valid UTF-8");
    unsafe { crate::bindgen::mju_writeLog(kind.as_ptr(), msg.as_ptr()) }
}

/// Get compiler error message from spec.
/* const char* mjs_getError(mjSpec* s); */
pub fn mjs_getError(s: &mut crate::MjSpec) -> Option<String> {
    let c_ptr = unsafe { crate::bindgen::mjs_getError(s.as_mut()) };
    if c_ptr.is_null() {
        None
    } else {
        Some(unsafe { std::ffi::CStr::from_ptr(c_ptr).to_str().unwrap().to_owned() })
    }
}

/// Check if compiler error is a warning.
/* int mjs_isWarning(mjSpec* s); */
pub fn mjs_isWarning(s: &mut crate::MjSpec) -> bool {
    /*
    <https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mjs-isWarning>
    > Returns 1 if the error is a warning.
    */
    unsafe { crate::bindgen::mjs_isWarning(s.as_mut()) == 1 }
}

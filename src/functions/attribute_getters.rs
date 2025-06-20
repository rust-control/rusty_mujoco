//! <https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#attributegetters>

/// Get string contents.
pub fn mjs_getString(source: &crate::MjString) -> &str {
    let c_char_ptr = unsafe { crate::bindgen::mjs_getString(source.as_ref()) };
    let c_str = unsafe { std::ffi::CStr::from_ptr(c_char_ptr) };
    c_str.to_str().expect("`mjs_getString` returned non-UTF-8 bytes")
}

/// Get double array contents.
pub fn mjs_getDouble(source: &crate::MjDoubleVec) -> &[f64] {
    let mut size_buf = 0i32;
    let c_double_ptr = unsafe { crate::bindgen::mjs_getDouble(source.as_ref(), &mut size_buf) };
    unsafe { std::slice::from_raw_parts(c_double_ptr, size_buf as usize) }
}

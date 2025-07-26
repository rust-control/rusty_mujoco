//! # [Attribute setters](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#attribute-setters)

use crate::{
    mjtByte, mjByteVec, mjString, mjStringVec, mjIntVec, mjFloatVec, mjDoubleVec,
    mjsPlugin,
};

/// Copy buffer to destination.
pub fn mjs_setBuffer(dest: &mut mjByteVec, array: &[u8]) {
    unsafe {
        crate::bindgen::mjs_setBuffer(
            dest,
            array.as_ptr() as *const _,
            array.len() as i32,
        );
    }
}

/// Copy text to string.
pub fn mjs_setString(dest: &mut mjString, text: &str) {
    let text = std::ffi::CString::new(text).expect("`text` must not contain null bytes");
    unsafe { crate::bindgen::mjs_setString(dest, text.as_ptr()) }
}

/// Split text into entries and copy to string vector.
pub fn mjs_setStringVec(dest: &mut mjStringVec, text: &str) {
    let text = std::ffi::CString::new(text).expect("`text` must not contain null bytes");
    unsafe { crate::bindgen::mjs_setStringVec(dest, text.as_ptr()) }
}

/// Set entry in string vector.
pub fn mjs_setInStringVec(dest: &mut mjStringVec, i: usize, text: &str) -> mjtByte {
    let text = std::ffi::CString::new(text).expect("`text` must not contain null bytes");
    unsafe { crate::bindgen::mjs_setInStringVec(dest, i as i32, text.as_ptr()) }
}

/// Append text entry to string vector.
pub fn mjs_appendString(dest: &mut mjStringVec, text: &str) {
    let text = std::ffi::CString::new(text).expect("`text` must not contain null bytes");
    unsafe { crate::bindgen::mjs_appendString(dest, text.as_ptr()) }
}

/// Copy int array to vector.
pub fn mjs_setInt(dest: &mut mjIntVec, array: &[i32]) {
    unsafe { crate::bindgen::mjs_setInt(dest, array.as_ptr(), array.len() as i32); }
}

/// Append int array to vector of arrays.
pub fn mjs_appendIntVec(dest: &mut mjIntVec, array: &[i32]) {
    unsafe { crate::bindgen::mjs_appendIntVec(dest, array.as_ptr(), array.len() as i32); }
}

/// Copy float array to vector.
pub fn mjs_setFloat(dest: &mut mjFloatVec, array: &[f32]) {
    unsafe { crate::bindgen::mjs_setFloat(dest, array.as_ptr(), array.len() as i32); }
}

/// Append float array to vector of arrays.
pub fn mjs_appendFloatVec(dest: &mut mjFloatVec, array: &[f32]) {
    unsafe { crate::bindgen::mjs_appendFloatVec(dest, array.as_ptr(), array.len() as i32); }
}

/// Copy double array to vector.
pub fn mjs_setDouble(dest: &mut mjDoubleVec, array: &[f64]) {
    unsafe { crate::bindgen::mjs_setDouble(dest, array.as_ptr(), array.len() as i32); }
}

/// Set plugin attributes.
pub fn mjs_setPluginAttributes(plugin: &mut mjsPlugin, attributes: *mut std::ffi::c_void) {
    unsafe { crate::bindgen::mjs_setPluginAttributes(plugin, attributes); }
}

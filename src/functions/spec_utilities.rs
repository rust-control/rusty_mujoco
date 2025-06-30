//! # [Spec utilities](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#spec-utilities)

use crate::{
    mjSpec, mjsElement, mjsBody, mjsFrame, mjsDefault, mjtByte,
    mjsOrientation,
};

/*
mjs_setDefault
void mjs_setDefault(mjsElement* element, const mjsDefault* def);
Set element’s default.

mjs_setFrame
int mjs_setFrame(mjsElement* dest, mjsFrame* frame);
Set element’s enclosing frame, return 0 on success.

mjs_resolveOrientation
const char* mjs_resolveOrientation(double quat[4], mjtByte degree, const char* sequence,
                                   const mjsOrientation* orientation);
Resolve alternative orientations to quat, return error if any.

mjs_bodyToFrame
mjsFrame* mjs_bodyToFrame(mjsBody** body);
Transform body into a frame.

mjs_setUserValue
void mjs_setUserValue(mjsElement* element, const char* key, const void* data);
Set user payload, overriding the existing value for the specified key if present.

mjs_getUserValue
const void* mjs_getUserValue(mjsElement* element, const char* key);
Return user payload or NULL if none found.

mjs_deleteUserValue
void mjs_deleteUserValue(mjsElement* element, const char* key);
Delete user payload.
*/

/// Set element’s default.
/* void mjs_setDefault(mjsElement* element, const mjsDefault* def); */
pub fn mjs_setDefault(element: &mut mjsElement, def: &mjsDefault) {
    unsafe { crate::bindgen::mjs_setDefault(element, def); }
}

/// Set element’s enclosing frame.
/* int mjs_setFrame(mjsElement* dest, mjsFrame* frame); */
pub fn mjs_setFrame(dest: &mut mjsElement, frame: &mut mjsFrame) -> Result<(), ()> {
    let result = unsafe { crate::bindgen::mjs_setFrame(dest, frame) };
    /*
    <https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mjs-setframe>
    > return 0 on success
    */
    if result == 0 {Ok(())} else {Err(())}
}

/// Resolve alternative orientations to quat, return error if any.
/* const char* mjs_resolveOrientation(double quat[4], mjtByte degree, const char* sequence,
                                   const mjsOrientation* orientation); */
pub fn mjs_resolveOrientation(
    quat: &mut [f64; 4],
    degree: mjtByte,
    sequence: &str,
    orientation: &mjsOrientation,
) -> Result<(), String> {
    let c_sequence = std::ffi::CString::new(sequence).expect("Failed to create CString");
    let result = unsafe {
        crate::bindgen::mjs_resolveOrientation(
            quat,
            degree,
            c_sequence.as_ptr(),
            orientation,
        )
    };
    if !result.is_null() {
        let error_msg = unsafe { std::ffi::CStr::from_ptr(result) }
            .to_str()
            .expect("Failed to convert C string to Rust string")
            .to_owned();
        Err(error_msg)
    } else {
        Ok(())
    }
}

/// Transform body into a frame.
/* mjsFrame* mjs_bodyToFrame(mjsBody** body); */
pub fn mjs_bodyToFrame<'body>(body: &'body mut [&mut mjsBody]) -> &'body mut mjsFrame {
    let c_ptr = unsafe { crate::bindgen::mjs_bodyToFrame(body.as_mut_ptr()) };
    if c_ptr.is_null() {
        panic!("Failed to transform body into a frame");
    }
    unsafe { &mut *c_ptr }
}

/// Set user payload, overriding the existing value for the specified key if present.
/* void mjs_setUserValue(mjsElement* element, const char* key, const void* data); */
pub fn mjs_setUserValue(
    element: &mut mjsElement,
    key: &str,
    data: &std::ffi::c_void,
) {
    let c_key = std::ffi::CString::new(key).expect("Failed to create CString");
    unsafe {
        crate::bindgen::mjs_setUserValue(
            element,
            c_key.as_ptr(),
            data,
        );
    }
}

/// Return user payload or NULL if none found.
/* const void* mjs_getUserValue(mjsElement* element, const char* key); */
pub fn mjs_getUserValue(
    element: &mut mjsElement,
    key: &str,
) -> Option<*const std::ffi::c_void> {
    let key = std::ffi::CString::new(key).expect("Failed to create CString");
    let ptr = unsafe { crate::bindgen::mjs_getUserValue(element, key.as_ptr()) };
    if ptr.is_null() {None} else {Some(ptr)}
}

/// Delete user payload.
/* void mjs_deleteUserValue(mjsElement* element, const char* key); */
pub fn mjs_deleteUserValue(element: &mut mjsElement, key: &str) {
    let key = std::ffi::CString::new(key).expect("Failed to create CString");
    unsafe { crate::bindgen::mjs_deleteUserValue(element, key.as_ptr()); }
}

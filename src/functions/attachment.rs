//! # [Attachment](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#attachment)

use crate::{MjSpec, mjsElement, mjsBody, mjsDefault};

/// Attach child to a parent, return the attached element if success or `None` otherwise.
/* mjsElement* mjs_attach(mjsElement* parent, const mjsElement* child,
                       const char* prefix, const char* suffix); */
pub fn mjs_attach<'element>(
    parent: &'element mut mjsElement,
    child: &'element mjsElement,
    prefix: Option<&str>,
    suffix: Option<&str>,
) -> Option<&'element mut mjsElement> {
    let prefix_cstr = prefix.map(|s| std::ffi::CString::new(s).unwrap());
    let suffix_cstr = suffix.map(|s| std::ffi::CString::new(s).unwrap());

    let c_ptr = unsafe {
        crate::bindgen::mjs_attach(
            parent,
            child,
            prefix_cstr.as_ref().map_or(std::ptr::null(), |s| s.as_ptr()),
            suffix_cstr.as_ref().map_or(std::ptr::null(), |s| s.as_ptr()),
        )
    };

    if c_ptr.is_null() {None} else {Some(unsafe { &mut *c_ptr })}
}

/// Delete body and descendants from mjSpec, remove all references.
/* int mjs_detachBody(mjSpec* s, mjsBody* b); */
pub fn mjs_detachBody(s: &mut MjSpec, b: &mut mjsBody) -> Result<(), ()> {
    /*
    <https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mjs-detachbody>
    > return 0 on success
    */
    if unsafe { crate::bindgen::mjs_detachBody(s.as_mut_ptr(), b) } == 0 {Ok(())} else {Err(())}
}

/// Delete default class and descendants from mjSpec, remove all references.
/* int mjs_detachDefault(mjSpec* s, mjsDefault* d); */
pub fn mjs_detachDefault(s: &mut MjSpec, d: &mut mjsDefault) -> Result<(), ()> {
    /*
    <https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mjs-detachdefault>
    > return 0 on success
    */
    if unsafe { crate::bindgen::mjs_detachDefault(s.as_mut_ptr(), d) } == 0 {Ok(())} else {Err(())}
}

//! # [Attachment](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#attachment)

use crate::{mjSpec, mjsElement, mjsBody, mjsDefault};

/// Attach child to a parent, return the attached element if success or `None` otherwise.
/* mjsElement* mjs_attach(mjsElement* parent, const mjsElement* child,
                       const char* prefix, const char* suffix); */
pub fn mjs_attach(
    parent: &mut mjsElement,
    child: &mjsElement,
    prefix: Option<&str>,
    suffix: Option<&str>,
) -> Option<mjsElement> {
    todo!()
}

/// Delete body and descendants from mjSpec, remove all references.
/* int mjs_detachBody(mjSpec* s, mjsBody* b); */
pub fn mjs_detachBody(s: &mut mjSpec, b: &mut mjsBody) -> Result<(), ()> {
    /*
    <https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mjs-detachbody>
    > return 0 on success
    */
    if unsafe { crate::bindgen::mjs_detachBody(s, b) } == 0 {Ok(())} else {Err(())}
}

/// Delete default class and descendants from mjSpec, remove all references.
/* int mjs_detachDefault(mjSpec* s, mjsDefault* d); */
pub fn mjs_detachDefault(s: &mut mjSpec, d: &mut mjsDefault) -> Result<(), ()> {
    /*
    <https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mjs-detachdefault>
    > return 0 on success
    */
    if unsafe { crate::bindgen::mjs_detachDefault(s, d) } == 0 {Ok(())} else {Err(())}
}

//! # [Find and get utilities](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#find-and-get-utilities)

use crate::{
    MjSpec,
    mjsElement, mjsBody, mjsFrame, mjsDefault,
    mjtObj,
};

/// Get spec from element.
/* mjSpec* mjs_getSpec(mjsElement* element); */
pub fn mjs_getSpec(element: &mut mjsElement) -> Option<MjSpec> {
    let ptr = unsafe { crate::bindgen::mjs_getSpec(element) };
    if ptr.is_null() {None} else {Some(MjSpec(ptr)) }
}

/// Find spec (model asset) by name.
/* mjSpec* mjs_findSpec(mjSpec* spec, const char* name); */
pub fn mjs_findSpec<'spec>(spec: &'spec mut MjSpec, name: &str) -> Option<MjSpec> {
    let name = std::ffi::CString::new(name).expect("`name` must not contain null bytes");
    let ptr = unsafe { crate::bindgen::mjs_findSpec(spec.0, name.as_ptr()) };
    if ptr.is_null() {None} else {Some(MjSpec(ptr)) }
}

/// Find body in spec by name.
/* mjsBody* mjs_findBody(mjSpec* s, const char* name); */
pub fn mjs_findBody<'spec>(spec: &'spec mut MjSpec, name: &str) -> Option<&'spec mut mjsBody> {
    let name = std::ffi::CString::new(name).expect("`name` must not contain null bytes");
    let ptr = unsafe { crate::bindgen::mjs_findBody(spec.0, name.as_ptr()) };
    if ptr.is_null() {None} else {Some(unsafe { &mut *ptr }) }
}

/// Find element in spec by name.
/* mjsElement* mjs_findElement(mjSpec* s, mjtObj type, const char* name); */
pub fn mjs_findElement<'spec>(
    spec: &'spec mut MjSpec,
    type_: mjtObj,
    name: &str,
) -> Option<&'spec mut mjsElement> {
    let name = std::ffi::CString::new(name).expect("`name` must not contain null bytes");
    let ptr = unsafe { crate::bindgen::mjs_findElement(spec.0, type_, name.as_ptr()) };
    if ptr.is_null() {None} else {Some(unsafe { &mut *ptr }) }
}

/// Find child body by name.
/* mjsBody* mjs_findChild(mjsBody* body, const char* name); */
pub fn mjs_findChild<'body>(body: &'body mut mjsBody, name: &str) -> Option<&'body mut mjsBody> {
    let name = std::ffi::CString::new(name).expect("`name` must not contain null bytes");
    let ptr = unsafe { crate::bindgen::mjs_findChild(body, name.as_ptr()) };
    if ptr.is_null() {None} else {Some(unsafe { &mut *ptr }) }
}

/// Get parent body.
/* mjsBody* mjs_getParent(mjsElement* element); */
pub fn mjs_getParent<'body>(element: &'body mut mjsElement) -> Option<&'body mut mjsBody> {
    let ptr = unsafe { crate::bindgen::mjs_getParent(element) };
    if ptr.is_null() {None} else {Some(unsafe { &mut *ptr }) }
}

/// Get parent frame.
/* mjsFrame* mjs_getFrame(mjsElement* element); */
pub fn mjs_getFrame<'element>(element: &'element mut mjsElement) -> Option<&'element mut mjsFrame> {
    let ptr = unsafe { crate::bindgen::mjs_getFrame(element) };
    if ptr.is_null() {None} else {Some(unsafe { &mut *ptr }) }
}

/// Find frame by name.
/* mjsFrame* mjs_findFrame(mjSpec* s, const char* name); */
pub fn mjs_findFrame<'spec>(spec: &'spec mut MjSpec, name: &str) -> Option<&'spec mut mjsFrame> {
    let name = std::ffi::CString::new(name).expect("`name` must not contain null bytes");
    let ptr = unsafe { crate::bindgen::mjs_findFrame(spec.0, name.as_ptr()) };
    if ptr.is_null() {None} else {Some(unsafe { &mut *ptr }) }
}

/// Get default corresponding to an element.
/* mjsDefault* mjs_getDefault(mjsElement* element); */
pub fn mjs_getDefault<'element>(element: &'element mut mjsElement) -> Option<&'element mut mjsDefault> {
    let ptr = unsafe { crate::bindgen::mjs_getDefault(element) };
    if ptr.is_null() {None} else {Some(unsafe { &mut *ptr }) }
}

/// Find default in model by class name.
/* mjsDefault* mjs_findDefault(mjSpec* s, const char* classname); */
pub fn mjs_findDefault<'spec>(
    spec: &'spec mut MjSpec,
    classname: &str,
) -> Option<&'spec mut mjsDefault> {
    let classname = std::ffi::CString::new(classname).expect("`classname` must not contain null bytes");
    let ptr = unsafe { crate::bindgen::mjs_findDefault(spec.0, classname.as_ptr()) };
    if ptr.is_null() {None} else {Some(unsafe { &mut *ptr }) }
}

/// Get global default from model.
/* mjsDefault* mjs_getSpecDefault(mjSpec* s); */
pub fn mjs_getSpecDefault<'spec>(spec: &'spec mut MjSpec) -> Option<&'spec mut mjsDefault> {
    let ptr = unsafe { crate::bindgen::mjs_getSpecDefault(spec.0) };
    if ptr.is_null() {None} else {Some(unsafe { &mut *ptr })}
}

/// Get element id index.
/* int mjs_getId(mjsElement* element); */
pub fn mjs_getId(element: &mut mjsElement) -> usize {
    unsafe { crate::bindgen::mjs_getId(element) as usize }
}

/// Return body’s first child of given type. If `recurse` is true, also search the body’s subtree.
/* mjsElement* mjs_firstChild(mjsBody* body, mjtObj type, int recurse); */
pub fn mjs_firstChild<'body>(
    body: &'body mut mjsBody,
    type_: mjtObj,
    recurse: bool,
) -> Option<&'body mut mjsElement> {
    let ptr = unsafe { crate::bindgen::mjs_firstChild(body, type_, recurse as i32) };
    if ptr.is_null() {None} else {Some(unsafe { &mut *ptr }) }
}

/// Return body’s next child of the same type; return `None` if child is last. If `recurse` is true, also search the body’s subtree.
/* mjsElement* mjs_nextChild(mjsBody* body, mjsElement* child, int recurse); */
pub fn mjs_nextChild<'body>(
    body: &'body mut mjsBody,
    child: &mut mjsElement,
    recurse: bool,
) -> Option<&'body mut mjsElement> {
    let ptr = unsafe { crate::bindgen::mjs_nextChild(body, child, recurse as i32) };
    if ptr.is_null() {None} else {Some(unsafe { &mut *ptr }) }
}

/// Return spec’s first element of selected type.
/* mjsElement* mjs_firstElement(mjSpec* s, mjtObj type); */
pub fn mjs_firstElement<'spec>(
    spec: &'spec mut MjSpec,
    type_: mjtObj,
) -> Option<&'spec mut mjsElement> {
    let ptr = unsafe { crate::bindgen::mjs_firstElement(spec.0, type_) };
    if ptr.is_null() {None} else {Some(unsafe { &mut *ptr }) }
}

/// Return spec’s next element; return `None` if element is last.
/* mjsElement* mjs_nextElement(mjSpec* s, mjsElement* element); */
pub fn mjs_nextElement<'spec>(
    spec: &'spec mut MjSpec,
    element: &mut mjsElement,
) -> Option<&'spec mut mjsElement> {
    let ptr = unsafe { crate::bindgen::mjs_nextElement(spec.0, element) };
    if ptr.is_null() {None} else {Some(unsafe { &mut *ptr }) }
}

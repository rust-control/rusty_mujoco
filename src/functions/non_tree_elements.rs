//! # [Non-tree elements](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#non-tree-elements)

use crate::{mjsActuator, mjSpec, mjsDefault, mjsSensor, mjsFlex, mjsPair, mjsExclude, mjsEquality, mjsTendon, mjsWrap, mjsNumeric, mjsText, mjsTuple, mjsKey, mjsPlugin};

/// Add actuator to mjSpec, return actuator spec.
/* mjsActuator* mjs_addActuator(mjSpec* s, const mjsDefault* def); */
pub fn mjs_addActuator<'spec>(
    s: &'spec mut mjSpec,
    def: &mjsDefault,
) -> &'spec mut mjsActuator {
    unsafe { &mut *crate::bindgen::mjs_addActuator(s, def) }
}

/// Add sensor to mjSpec, return sensor spec.
/* mjsSensor* mjs_addSensor(mjSpec* s); */
pub fn mjs_addSensor<'spec>(s: &'spec mut mjSpec) -> &'spec mut mjsSensor {
    unsafe { &mut *crate::bindgen::mjs_addSensor(s) }
}

/// Add flex to mjSpec, return flex spec.
/* mjsFlex* mjs_addFlex(mjSpec* s); */
pub fn mjs_addFlex<'spec>(s: &'spec mut mjSpec) -> &'spec mut mjsFlex {
    unsafe { &mut *crate::bindgen::mjs_addFlex(s) }
}

/// Add contact pair to mjSpec, return pair spec.
/* mjsPair* mjs_addPair(mjSpec* s, const mjsDefault* def); */
pub fn mjs_addPair<'spec>(
    s: &'spec mut mjSpec,
    def: &mjsDefault,
) -> &'spec mut mjsPair {
    unsafe { &mut *crate::bindgen::mjs_addPair(s, def) }
}

/// Add excluded body pair to mjSpec, return exclude spec.
/* mjsExclude* mjs_addExclude(mjSpec* s); */
pub fn mjs_addExclude<'spec>(s: &'spec mut mjSpec) -> &'spec mut mjsExclude {
    unsafe { &mut *crate::bindgen::mjs_addExclude(s) }
}

/// Add equality to mjSpec, return equality spec.
/* mjsEquality* mjs_addEquality(mjSpec* s, const mjsDefault* def); */
pub fn mjs_addEquality<'spec>(
    s: &'spec mut mjSpec,
    def: &mjsDefault,
) -> &'spec mut mjsEquality {
    unsafe { &mut *crate::bindgen::mjs_addEquality(s, def) }
}

/// Add tendon to mjSpec, return tendon spec.
/* mjsTendon* mjs_addTendon(mjSpec* s, const mjsDefault* def); */
pub fn mjs_addTendon<'spec>( 
    s: &'spec mut mjSpec,
    def: &mjsDefault,
) -> &'spec mut mjsTendon {
    unsafe { &mut *crate::bindgen::mjs_addTendon(s, def) }
}

/// Wrap site using tendon, return wrap spec.
/* mjsWrap* mjs_wrapSite(mjsTendon* tendon, const char* name); */
pub fn mjs_wrapSite<'tendon>(
    tendon: &'tendon mut mjsTendon,
    name: &str,
) -> &'tendon mut mjsWrap {
    let c_name = std::ffi::CString::new(name).expect("`name` must not contain null bytes");
    unsafe { &mut *crate::bindgen::mjs_wrapSite(tendon, c_name.as_ptr()) }
}

/// Wrap geom using tendon, return wrap spec.
/* mjsWrap* mjs_wrapGeom(mjsTendon* tendon, const char* name, const char* sidesite); */
pub fn mjs_wrapGeom<'tendon>(
    tendon: &'tendon mut mjsTendon,
    name: &str,
    sidesite: &str,
) -> &'tendon mut mjsWrap {
    let c_name = std::ffi::CString::new(name).expect("`name` must not contain null bytes");
    let c_sidesite = std::ffi::CString::new(sidesite).expect("`sidesite` must not contain null bytes");
    unsafe { &mut *crate::bindgen::mjs_wrapGeom(tendon, c_name.as_ptr(), c_sidesite.as_ptr()) }
}

/// Wrap joint using tendon, return wrap spec.
/* mjsWrap* mjs_wrapJoint(mjsTendon* tendon, const char* name, double coef); */
pub fn mjs_wrapJoint<'tendon>(
    tendon: &'tendon mut mjsTendon,
    name: &str,
    coef: f64,
) -> &'tendon mut mjsWrap {
    let c_name = std::ffi::CString::new(name).expect("`name` must not contain null bytes");
    unsafe { &mut *crate::bindgen::mjs_wrapJoint(tendon, c_name.as_ptr(), coef) }
}

/// Wrap pulley using tendon, return wrap spec.
/* mjsWrap* mjs_wrapPulley(mjsTendon* tendon, double divisor); */
pub fn mjs_wrapPulley<'tendon>(
    tendon: &'tendon mut mjsTendon,
    divisor: f64,
) -> &'tendon mut mjsWrap {
    unsafe { &mut *crate::bindgen::mjs_wrapPulley(tendon, divisor) }
}

/// Add numeric to mjSpec, return numeric spec.
/* mjsNumeric* mjs_addNumeric(mjSpec* s); */
pub fn mjs_addNumeric<'spec>(s: &'spec mut mjSpec) -> &'spec mut mjsNumeric {
    unsafe { &mut *crate::bindgen::mjs_addNumeric(s) }
}

/// Add text to mjSpec, return text spec.
/* mjsText* mjs_addText(mjSpec* s); */
pub fn mjs_addText<'spec>(s: &'spec mut mjSpec) -> &'spec mut mjsText {
    unsafe { &mut *crate::bindgen::mjs_addText(s) }
}

/// Add tuple to mjSpec, return tuple spec.
/* mjsTuple* mjs_addTuple(mjSpec* s); */
pub fn mjs_addTuple<'spec>(s: &'spec mut mjSpec) -> &'spec mut mjsTuple {
    unsafe { &mut *crate::bindgen::mjs_addTuple(s) }
}

/// Add keyframe to mjSpec, return key spec.
/* mjsKey* mjs_addKey(mjSpec* s); */
pub fn mjs_addKey<'spec>(s: &'spec mut mjSpec) -> &'spec mut mjsKey {
    unsafe { &mut *crate::bindgen::mjs_addKey(s) }
}

/// Add plugin to mjSpec, return plugin spec.
/* mjsPlugin* mjs_addPlugin(mjSpec* s); */
pub fn mjs_addPlugin<'spec>(s: &'spec mut mjSpec) -> &'spec mut mjsPlugin {
    unsafe { &mut *crate::bindgen::mjs_addPlugin(s) }
}

/// Add default to mjSpec, return default spec.
/* mjsDefault* mjs_addDefault(mjSpec* s, const char* classname, const mjsDefault* parent); */
pub fn mjs_addDefault<'spec>(
    s: &'spec mut mjSpec,
    classname: &str,
    parent: &mjsDefault,
) -> &'spec mut mjsDefault {
    let c_classname = std::ffi::CString::new(classname).expect("`classname` must not contain null bytes");
    unsafe { &mut *crate::bindgen::mjs_addDefault(s, c_classname.as_ptr(), parent) }
}

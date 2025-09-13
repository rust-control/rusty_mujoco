//! # [Element initialization](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#element-initialization)

use crate::{
    MjSpec, 
    mjsOrientation, mjsBody, mjsFrame, mjsJoint, mjsGeom, mjsSite,
    mjsCamera, mjsLight, mjsFlex, mjsMesh, mjsHField, mjsSkin,
    mjsTexture, mjsMaterial, mjsPair, mjsEquality, mjsTendon,
    mjsActuator, mjsSensor, mjsNumeric, mjsText, mjsTuple,
    mjsKey, mjsPlugin,
};

/// Set default spec attributes.
/// 
/// **note**: This function is called in the `Default` implementation of [`mjSpec`].
/* void mjs_defaultSpec(mjSpec* spec); */
pub fn mjs_defaultSpec() -> MjSpec {
    let mut c = Box::<crate::bindgen::mjSpec>::new_uninit();
    unsafe { crate::bindgen::mjs_defaultSpec(c.as_mut_ptr()); }
    MjSpec::from_raw(Box::into_raw(unsafe { c.assume_init() }))
}

/// Set default orientation attributes.
/// 
/// **note**: This function is called in the `Default` implementation of [`mjsOrientation`].
/* void mjs_defaultOrientation(mjsOrientation* orient); */
pub fn mjs_defaultOrientation() -> mjsOrientation {
    let mut c = std::mem::MaybeUninit::<mjsOrientation>::uninit();
    unsafe { crate::bindgen::mjs_defaultOrientation(c.as_mut_ptr()); }
    unsafe { c.assume_init() }
}

/// Set default body attributes.
/// 
/// **note**: This function is called in the `Default` implementation of [`mjsBody`].
/* void mjs_defaultBody(mjsBody* body); */
pub fn mjs_defaultBody() -> mjsBody {
    let mut c = std::mem::MaybeUninit::<mjsBody>::uninit();
    unsafe { crate::bindgen::mjs_defaultBody(c.as_mut_ptr()); }
    unsafe { c.assume_init() }
}

/// Set default frame attributes.
/// 
/// **note**: This function is called in the `Default` implementation of [`mjsFrame`].
/* void mjs_defaultFrame(mjsFrame* frame); */
pub fn mjs_defaultFrame() -> mjsFrame {
    let mut c = std::mem::MaybeUninit::<mjsFrame>::uninit();
    unsafe { crate::bindgen::mjs_defaultFrame(c.as_mut_ptr()); }
    unsafe { c.assume_init() }
}

/// Set default joint attributes.
/// 
/// **note**: This function is called in the `Default` implementation of [`mjsJoint`].
/* void mjs_defaultJoint(mjsJoint* joint); */
pub fn mjs_defaultJoint() -> mjsJoint {
    let mut c = std::mem::MaybeUninit::<mjsJoint>::uninit();
    unsafe { crate::bindgen::mjs_defaultJoint(c.as_mut_ptr()); }
    unsafe { c.assume_init() }
}

/// Set default geom attributes.
/// 
/// **note**: This function is called in the `Default` implementation of [`mjsGeom`].
/* void mjs_defaultGeom(mjsGeom* geom); */
pub fn mjs_defaultGeom() -> mjsGeom {
    let mut c = std::mem::MaybeUninit::<mjsGeom>::uninit();
    unsafe { crate::bindgen::mjs_defaultGeom(c.as_mut_ptr()); }
    unsafe { c.assume_init() }
}   

/// Set default site attributes.
/// 
/// **note**: This function is called in the `Default` implementation of [`mjsSite`].
/* void mjs_defaultSite(mjsSite* site); */
pub fn mjs_defaultSite() -> mjsSite {
    let mut c = std::mem::MaybeUninit::<mjsSite>::uninit();
    unsafe { crate::bindgen::mjs_defaultSite(c.as_mut_ptr()); }
    unsafe { c.assume_init() }
}

/// Set default camera attributes.
/// 
/// **note**: This function is called in the `Default` implementation of [`mjsCamera`].
/* void mjs_defaultCamera(mjsCamera* camera); */
pub fn mjs_defaultCamera() -> mjsCamera {
    let mut c = std::mem::MaybeUninit::<mjsCamera>::uninit();
    unsafe { crate::bindgen::mjs_defaultCamera(c.as_mut_ptr()); }
    unsafe { c.assume_init() }
}

/// Set default light attributes.
/// 
/// **note**: This function is called in the `Default` implementation of [`mjsLight`].
/* void mjs_defaultLight(mjsLight* light); */
pub fn mjs_defaultLight() -> mjsLight {
    let mut c = std::mem::MaybeUninit::<mjsLight>::uninit();
    unsafe { crate::bindgen::mjs_defaultLight(c.as_mut_ptr()); }
    unsafe { c.assume_init() }
}

/// Set default flex attributes.
/// 
/// **note**: This function is called in the `Default` implementation of [`mjsFlex`].
/* void mjs_defaultFlex(mjsFlex* flex); */
pub fn mjs_defaultFlex() -> mjsFlex {
    let mut c = std::mem::MaybeUninit::<mjsFlex>::uninit();
    unsafe { crate::bindgen::mjs_defaultFlex(c.as_mut_ptr()); }
    unsafe { c.assume_init() }
}

/// Set default mesh attributes.
/// 
/// **note**: This function is called in the `Default` implementation of [`mjsMesh`].
/* void mjs_defaultMesh(mjsMesh* mesh); */
pub fn mjs_defaultMesh() -> mjsMesh {
    let mut c = std::mem::MaybeUninit::<mjsMesh>::uninit();
    unsafe { crate::bindgen::mjs_defaultMesh(c.as_mut_ptr()); }
    unsafe { c.assume_init() }
}

/// Set default height field attributes.
/// 
/// **note**: This function is called in the `Default` implementation of [`mjsHField`].
/* void mjs_defaultHField(mjsHField* hfield); */
pub fn mjs_defaultHField() -> mjsHField {
    let mut c = std::mem::MaybeUninit::<mjsHField>::uninit();
    unsafe { crate::bindgen::mjs_defaultHField(c.as_mut_ptr()); }
    unsafe { c.assume_init() }
}

/// Set default skin attributes.
//// **note**: This function is called in the `Default` implementation of [`mjsSkin`].
/* void mjs_defaultSkin(mjsSkin* skin); */
pub fn mjs_defaultSkin() -> mjsSkin {
    let mut c = std::mem::MaybeUninit::<mjsSkin>::uninit();
    unsafe { crate::bindgen::mjs_defaultSkin(c.as_mut_ptr()); }
    unsafe { c.assume_init() }
}

/// Set default texture attributes.
/// 
/// **note**: This function is called in the `Default` implementation of [`mjsTexture`].
/* void mjs_defaultTexture(mjsTexture* texture); */
pub fn mjs_defaultTexture() -> mjsTexture {
    let mut c = std::mem::MaybeUninit::<mjsTexture>::uninit();
    unsafe { crate::bindgen::mjs_defaultTexture(c.as_mut_ptr()); }
    unsafe { c.assume_init() }
}

/// Set default material attributes.
/// 
/// **note**: This function is called in the `Default` implementation of [`mjsMaterial`].
/* void mjs_defaultMaterial(mjsMaterial* material); */
pub fn mjs_defaultMaterial() -> mjsMaterial {
    let mut c = std::mem::MaybeUninit::<mjsMaterial>::uninit();
    unsafe { crate::bindgen::mjs_defaultMaterial(c.as_mut_ptr()); }
    unsafe { c.assume_init() }
}

/// Set default pair attributes.
/// 
/// **note**: This function is called in the `Default` implementation of [`mjsPair`].
/* void mjs_defaultPair(mjsPair* pair); */
pub fn mjs_defaultPair() -> mjsPair {
    let mut c = std::mem::MaybeUninit::<mjsPair>::uninit();
    unsafe { crate::bindgen::mjs_defaultPair(c.as_mut_ptr()); }
    unsafe { c.assume_init() }
}

/// Set default equality attributes.
/// 
/// **note**: This function is called in the `Default` implementation of [`mjsEquality`].
/* void mjs_defaultEquality(mjsEquality* equality); */
pub fn mjs_defaultEquality() -> mjsEquality {   
    let mut c = std::mem::MaybeUninit::<mjsEquality>::uninit();
    unsafe { crate::bindgen::mjs_defaultEquality(c.as_mut_ptr()); }
    unsafe { c.assume_init() }
}

/// Set default tendon attributes.
/// 
/// **note**: This function is called in the `Default` implementation of [`mjsTendon`].
/* void mjs_defaultTendon(mjsTendon* tendon); */
pub fn mjs_defaultTendon() -> mjsTendon {
    let mut c = std::mem::MaybeUninit::<mjsTendon>::uninit();
    unsafe { crate::bindgen::mjs_defaultTendon(c.as_mut_ptr()); }
    unsafe { c.assume_init() }
}

/// Set default actuator attributes.
/// 
/// **note**: This function is called in the `Default` implementation of [`mjsActuator`].
/* void mjs_defaultActuator(mjsActuator* actuator); */
pub fn mjs_defaultActuator() -> mjsActuator {
    let mut c = std::mem::MaybeUninit::<mjsActuator>::uninit();
    unsafe { crate::bindgen::mjs_defaultActuator(c.as_mut_ptr()); }
    unsafe { c.assume_init() }
}

/// Set default sensor attributes.
///
/// **note**: This function is called in the `Default` implementation of [`mjsSensor`].
/* void mjs_defaultSensor(mjsSensor* sensor); */
pub fn mjs_defaultSensor() -> mjsSensor {
    let mut c = std::mem::MaybeUninit::<mjsSensor>::uninit();
    unsafe { crate::bindgen::mjs_defaultSensor(c.as_mut_ptr()); }
    unsafe { c.assume_init() }
}

/// Set default numeric attributes.
/// 
/// **note**: This function is called in the `Default` implementation of [`mjsNumeric`].
/* void mjs_defaultNumeric(mjsNumeric* numeric); */
pub fn mjs_defaultNumeric() -> mjsNumeric {
    let mut c = std::mem::MaybeUninit::<mjsNumeric>::uninit();
    unsafe { crate::bindgen::mjs_defaultNumeric(c.as_mut_ptr()); }
    unsafe { c.assume_init() }
}

/// Set default text attributes.
/// 
/// **note**: This function is called in the `Default` implementation of [`mjsText`].
/* void mjs_defaultText(mjsText* text); */
pub fn mjs_defaultText() -> mjsText {
    let mut c = std::mem::MaybeUninit::<mjsText>::uninit();
    unsafe { crate::bindgen::mjs_defaultText(c.as_mut_ptr()); }
    unsafe { c.assume_init() }
}

/// Set default tuple attributes.
/// 
/// **note**: This function is called in the `Default` implementation of [`mjsTuple`].
/* void mjs_defaultTuple(mjsTuple* tuple); */
pub fn mjs_defaultTuple() -> mjsTuple {
    let mut c = std::mem::MaybeUninit::<mjsTuple>::uninit();
    unsafe { crate::bindgen::mjs_defaultTuple(c.as_mut_ptr()); }
    unsafe { c.assume_init() }
}

/// Set default keyframe attributes.
/// 
/// **note**: This function is called in the `Default` implementation of [`mjsKey`].
/* void mjs_defaultKey(mjsKey* key); */
pub fn mjs_defaultKey() -> mjsKey {
    let mut c = std::mem::MaybeUninit::<mjsKey>::uninit();
    unsafe { crate::bindgen::mjs_defaultKey(c.as_mut_ptr()); }
    unsafe { c.assume_init() }
}

/// Set default plugin attributes.
/// 
/// **note**: This function is called in the `Default` implementation of [`mjsPlugin`].
/* void mjs_defaultPlugin(mjsPlugin* plugin); */
pub fn mjs_defaultPlugin() -> mjsPlugin {
    let mut c = std::mem::MaybeUninit::<mjsPlugin>::uninit();
    unsafe { crate::bindgen::mjs_defaultPlugin(c.as_mut_ptr()); }
    unsafe { c.assume_init() }
}


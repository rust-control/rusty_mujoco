//! # [Element casting](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#element-casting)

use crate::{
    mjsElement, mjsBody, mjsGeom, mjsJoint, mjsSite, mjsCamera, mjsLight,
    mjsFrame, mjsActuator, mjsSensor, mjsFlex, mjsPair, mjsEquality,
    mjsExclude, mjsTendon, mjsNumeric, mjsText, mjsTuple, mjsKey,
    mjsMesh, mjsHField, mjsSkin, mjsTexture, mjsMaterial, mjsPlugin,
};

macro_rules! element_casting {
    ($( $name:ident: $type:ty; )*) => {
        $(
            /// Safely cast an element as the specified type, or return `None` if the element is not of that type.
            pub fn $name(element: &mut mjsElement) -> Option<&$type> {
                let c_ptr = unsafe { crate::bindgen::$name(element) };
                if c_ptr.is_null() {
                    None
                } else {
                    Some(unsafe { &*(c_ptr as *const $type) })
                }
            }
        )*
    };
}
element_casting! {
    mjs_asBody: mjsBody;
    mjs_asGeom: mjsGeom;
    mjs_asJoint: mjsJoint;
    mjs_asSite: mjsSite;
    mjs_asCamera: mjsCamera;
    mjs_asLight: mjsLight;
    mjs_asFrame: mjsFrame;
    mjs_asActuator: mjsActuator;
    mjs_asSensor: mjsSensor;
    mjs_asFlex: mjsFlex;
    mjs_asPair: mjsPair;
    mjs_asEquality: mjsEquality;
    mjs_asExclude: mjsExclude;
    mjs_asTendon: mjsTendon;
    mjs_asNumeric: mjsNumeric;
    mjs_asText: mjsText;
    mjs_asTuple: mjsTuple;
    mjs_asKey: mjsKey;
    mjs_asMesh: mjsMesh;
    mjs_asHField: mjsHField;
    mjs_asSkin: mjsSkin;
    mjs_asTexture: mjsTexture;
    mjs_asMaterial: mjsMaterial;
    mjs_asPlugin: mjsPlugin;
}

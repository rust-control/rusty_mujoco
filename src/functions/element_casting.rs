//! # [Element casting](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#element-casting)

use crate::{
    mjsElement, mjsBody, mjsGeom, mjsJoint, mjsSite, mjsCamera, mjsLight,
    mjsFrame, mjsActuator, mjsSensor, mjsFlex, mjsPair, mjsEquality,
    mjsExclude, mjsTendon, mjsNumeric, mjsText, mjsTuple, mjsKey,
    mjsMesh, mjsHField, mjsSkin, mjsTexture, mjsMaterial, mjsPlugin,
};

/*
mjs_asBody
mjsBody* mjs_asBody(mjsElement* element);
Safely cast an element as mjsBody, or return NULL if the element is not an mjsBody.

mjs_asGeom
mjsGeom* mjs_asGeom(mjsElement* element);
Safely cast an element as mjsGeom, or return NULL if the element is not an mjsGeom.

mjs_asJoint
mjsJoint* mjs_asJoint(mjsElement* element);
Safely cast an element as mjsJoint, or return NULL if the element is not an mjsJoint.

mjs_asSite
mjsSite* mjs_asSite(mjsElement* element);
Safely cast an element as mjsSite, or return NULL if the element is not an mjsSite.

mjs_asCamera
mjsCamera* mjs_asCamera(mjsElement* element);
Safely cast an element as mjsCamera, or return NULL if the element is not an mjsCamera.

mjs_asLight
mjsLight* mjs_asLight(mjsElement* element);
Safely cast an element as mjsLight, or return NULL if the element is not an mjsLight.

mjs_asFrame
mjsFrame* mjs_asFrame(mjsElement* element);
Safely cast an element as mjsFrame, or return NULL if the element is not an mjsFrame.

mjs_asActuator
mjsActuator* mjs_asActuator(mjsElement* element);
Safely cast an element as mjsActuator, or return NULL if the element is not an mjsActuator.

mjs_asSensor
mjsSensor* mjs_asSensor(mjsElement* element);
Safely cast an element as mjsSensor, or return NULL if the element is not an mjsSensor.

mjs_asFlex
mjsFlex* mjs_asFlex(mjsElement* element);
Safely cast an element as mjsFlex, or return NULL if the element is not an mjsFlex.

mjs_asPair
mjsPair* mjs_asPair(mjsElement* element);
Safely cast an element as mjsPair, or return NULL if the element is not an mjsPair.

mjs_asEquality
mjsEquality* mjs_asEquality(mjsElement* element);
Safely cast an element as mjsEquality, or return NULL if the element is not an mjsEquality.

mjs_asExclude
mjsExclude* mjs_asExclude(mjsElement* element);
Safely cast an element as mjsExclude, or return NULL if the element is not an mjsExclude.

mjs_asTendon
mjsTendon* mjs_asTendon(mjsElement* element);
Safely cast an element as mjsTendon, or return NULL if the element is not an mjsTendon.

mjs_asNumeric
mjsNumeric* mjs_asNumeric(mjsElement* element);
Safely cast an element as mjsNumeric, or return NULL if the element is not an mjsNumeric.

mjs_asText
mjsText* mjs_asText(mjsElement* element);
Safely cast an element as mjsText, or return NULL if the element is not an mjsText.

mjs_asTuple
mjsTuple* mjs_asTuple(mjsElement* element);
Safely cast an element as mjsTuple, or return NULL if the element is not an mjsTuple.

mjs_asKey
mjsKey* mjs_asKey(mjsElement* element);
Safely cast an element as mjsKey, or return NULL if the element is not an mjsKey.

mjs_asMesh
mjsMesh* mjs_asMesh(mjsElement* element);
Safely cast an element as mjsMesh, or return NULL if the element is not an mjsMesh.

mjs_asHField
mjsHField* mjs_asHField(mjsElement* element);
Safely cast an element as mjsHField, or return NULL if the element is not an mjsHField.

mjs_asSkin
mjsSkin* mjs_asSkin(mjsElement* element);
Safely cast an element as mjsSkin, or return NULL if the element is not an mjsSkin.

mjs_asTexture
mjsTexture* mjs_asTexture(mjsElement* element);
Safely cast an element as mjsTexture, or return NULL if the element is not an mjsTexture.

mjs_asMaterial
mjsMaterial* mjs_asMaterial(mjsElement* element);
Safely cast an element as mjsMaterial, or return NULL if the element is not an mjsMaterial.

mjs_asPlugin
mjsPlugin* mjs_asPlugin(mjsElement* element);
Safely cast an element as mjsPlugin, or return NULL if the element is not an mjsPlugin.
*/

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

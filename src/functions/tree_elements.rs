//! # [Tree elements](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#tree-elements)

use crate::{mjsBody, mjsDefault, mjsSite, mjsJoint, mjsGeom, mjsCamera, mjsLight, mjsFrame, mjsElement};

/// Add child body to body, return child.
/* mjsBody* mjs_addBody(mjsBody* body, const mjsDefault* def); */
pub fn mjs_addBody<'body>(
    body: &'body mut mjsBody,
    def: &mjsDefault,
) -> &'body mjsBody {
    unsafe { &*crate::bindgen::mjs_addBody(body, def) }
}

/// Add site to body, return site spec.
/* mjsSite* mjs_addSite(mjsBody* body, const mjsDefault* def); */
pub fn mjs_addSite<'body>(
    body: &'body mut mjsBody,
    def: &mjsDefault,
) -> &'body mut mjsSite {
    unsafe { &mut *crate::bindgen::mjs_addSite(body, def) }
}

/// Add joint to body.
/* mjsJoint* mjs_addJoint(mjsBody* body, const mjsDefault* def); */
pub fn mjs_addJoint<'body>(
    body: &'body mut mjsBody,
    def: &mjsDefault,
) -> &'body mut mjsJoint {
    unsafe { &mut *crate::bindgen::mjs_addJoint(body, def) }
}

/// Add freejoint to body.
/* mjsJoint* mjs_addFreeJoint(mjsBody* body); */
pub fn mjs_addFreeJoint<'body>(body: &'body mut mjsBody) -> &'body mut mjsJoint {
    unsafe { &mut *crate::bindgen::mjs_addFreeJoint(body) }
}

/// Add geom to body.
/* mjsGeom* mjs_addGeom(mjsBody* body, const mjsDefault* def); */
pub fn mjs_addGeom<'body>(
    body: &'body mut mjsBody,
    def: &mjsDefault,
) -> &'body mut mjsGeom {
    unsafe { &mut *crate::bindgen::mjs_addGeom(body, def) }
}

/// Add camera to body.
/* mjsCamera* mjs_addCamera(mjsBody* body, const mjsDefault* def); */
pub fn mjs_addCamera<'body>(
    body: &'body mut mjsBody,
    def: &mjsDefault,
) -> &'body mut mjsCamera {
    unsafe { &mut *crate::bindgen::mjs_addCamera(body, def) }
}

/// Add light to body.
/* mjsLight* mjs_addLight(mjsBody* body, const mjsDefault* def); */
pub fn mjs_addLight<'body>(
    body: &'body mut mjsBody,
    def: &mjsDefault,
) -> &'body mut mjsLight {
    unsafe { &mut *crate::bindgen::mjs_addLight(body, def) }
}

/// Add frame to body.
/* mjsFrame* mjs_addFrame(mjsBody* body, mjsFrame* parentframe); */
pub fn mjs_addFrame<'body>(
    body: &'body mut mjsBody,
    parentframe: &'body mut mjsFrame,
) -> &'body mut mjsFrame {
    unsafe { &mut *crate::bindgen::mjs_addFrame(body, parentframe) }
}

/// Delete object corresponding to the given element.
/* int mjs_delete(mjsElement* element); */
pub fn mjs_delete(element: &mut mjsElement) -> Result<(), ()> {
    /*
    <https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mjs-delete>
    > return 0 on success
    */
    if unsafe { crate::bindgen::mjs_delete(element) } == 0 {Ok(())} else {Err(())}
}

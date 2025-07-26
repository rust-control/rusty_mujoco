//! # [Assets](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#assets)

use crate::{mjsMesh, mjSpec, mjsDefault, mjsHField, mjsSkin, mjsTexture, mjsMaterial};

/// Add mesh to mjSpec.
pub fn mjs_addMesh<'spec>(
    s: &'spec mut mjSpec,
    def: &mjsDefault,
) -> &'spec mut mjsMesh {
    unsafe { &mut *crate::bindgen::mjs_addMesh(s, def) }
}

/// Add height field to mjSpec.
pub fn mjs_addHField<'spec>(s: &'spec mut mjSpec) -> &'spec mut mjsHField {
    unsafe { &mut *crate::bindgen::mjs_addHField(s) }
}

/// Add skin to mjSpec.
pub fn mjs_addSkin<'spec>(s: &'spec mut mjSpec) -> &'spec mut mjsSkin {
    unsafe { &mut *crate::bindgen::mjs_addSkin(s) }
}

/// Add texture to mjSpec.
pub fn mjs_addTexture<'spec>(s: &'spec mut mjSpec) -> &'spec mut mjsTexture {
    unsafe { &mut *crate::bindgen::mjs_addTexture(s) }
}

/// Add material to mjSpec.
pub fn mjs_addMaterial<'spec>(
    s: &'spec mut mjSpec,
    def: &mjsDefault,
) -> &'spec mut mjsMaterial {
    unsafe { &mut *crate::bindgen::mjs_addMaterial(s, def) }
}

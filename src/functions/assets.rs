//! # [Assets](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#assets)

use crate::{MjSpec, mjsMesh, mjsDefault, mjsHField, mjsSkin, mjsTexture, mjsMaterial};

/// Add mesh to mjSpec.
pub fn mjs_addMesh<'spec>(
    s: &'spec mut MjSpec,
    def: &mjsDefault,
) -> &'spec mut mjsMesh {
    unsafe { &mut *crate::bindgen::mjs_addMesh(s.0, def) }
}

/// Add height field to mjSpec.
pub fn mjs_addHField<'spec>(s: &'spec mut MjSpec) -> &'spec mut mjsHField {
    unsafe { &mut *crate::bindgen::mjs_addHField(s.0) }
}

/// Add skin to mjSpec.
pub fn mjs_addSkin<'spec>(s: &'spec mut MjSpec) -> &'spec mut mjsSkin {
    unsafe { &mut *crate::bindgen::mjs_addSkin(s.0) }
}

/// Add texture to mjSpec.
pub fn mjs_addTexture<'spec>(s: &'spec mut MjSpec) -> &'spec mut mjsTexture {
    unsafe { &mut *crate::bindgen::mjs_addTexture(s.0) }
}

/// Add material to mjSpec.
pub fn mjs_addMaterial<'spec>(
    s: &'spec mut MjSpec,
    def: &mjsDefault,
) -> &'spec mut mjsMaterial {
    unsafe { &mut *crate::bindgen::mjs_addMaterial(s.0, def) }
}

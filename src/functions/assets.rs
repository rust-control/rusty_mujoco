//! # [Assets](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#assets)

use crate::{MjSpec, mjsMesh, mjsDefault, mjsHField, mjsSkin, mjsTexture, mjsMaterial};

/// Add mesh to mjSpec.
pub fn mjs_addMesh<'spec>(
    s: &'spec mut MjSpec,
    def: &mjsDefault,
) -> &'spec mut mjsMesh {
    unsafe { &mut *crate::bindgen::mjs_addMesh(s.as_mut_ptr(), def) }
}

/// Add height field to mjSpec.
pub fn mjs_addHField<'spec>(s: &'spec mut MjSpec) -> &'spec mut mjsHField {
    unsafe { &mut *crate::bindgen::mjs_addHField(s.as_mut_ptr()) }
}

/// Add skin to mjSpec.
pub fn mjs_addSkin<'spec>(s: &'spec mut MjSpec) -> &'spec mut mjsSkin {
    unsafe { &mut *crate::bindgen::mjs_addSkin(s.as_mut_ptr()) }
}

/// Add texture to mjSpec.
pub fn mjs_addTexture<'spec>(s: &'spec mut MjSpec) -> &'spec mut mjsTexture {
    unsafe { &mut *crate::bindgen::mjs_addTexture(s.as_mut_ptr()) }
}

/// Add material to mjSpec.
pub fn mjs_addMaterial<'spec>(
    s: &'spec mut MjSpec,
    def: &mjsDefault,
) -> &'spec mut mjsMaterial {
    unsafe { &mut *crate::bindgen::mjs_addMaterial(s.as_mut_ptr(), def) }
}

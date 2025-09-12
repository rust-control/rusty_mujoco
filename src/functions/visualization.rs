//! # [Visualization](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#visualization)
//! 
//! The functions in this section implement abstract visualization.
//! The results are used by the OpenGL renderer, and can also be used by users
//! wishing to implement their own renderer, or hook up MuJoCo to
//! advanced rendering tools such as Unity or Unreal Engine.
//! See [simulate](https://mujoco.readthedocs.io/en/stable/programming/samples.html#sasimulate)
//! for illustration of how to use these functions.

use crate::{
    MjModel, MjData, MjvScene, mjvOption, mjvCamera, mjvPerturb, mjvGeom, mjvFigure,
    mjtGeom,
};

/// Set default visualization options.
/// 
/// **note**: [`mjvOption`] calls this function in its `Default` implementation.
/* void mjv_defaultOption(mjvOption* opt); */
pub fn mjv_defaultOption() -> mjvOption {
    let mut c = std::mem::MaybeUninit::<mjvOption>::uninit();
    unsafe { crate::bindgen::mjv_defaultOption(c.as_mut_ptr()); }
    unsafe { c.assume_init() }
}

/// Set default figure.
/// 
/// **note**: [`mjvFigure`] calls this function in its `Default` implementation.
/* void mjv_defaultFigure(mjvFigure* fig); */
pub fn mjv_defaultFigure() -> crate::mjvFigure {
    let mut c = std::mem::MaybeUninit::<mjvFigure>::uninit();
    unsafe { crate::bindgen::mjv_defaultFigure(c.as_mut_ptr()); }
    unsafe { c.assume_init() }
}

/// Initialize given geom fields when not `None`, set the rest to their default values.
/// 
/// **note**: [`mjvGeom`] calls this function with all default options in its `new` implementation.
/* void mjv_initGeom(mjvGeom* geom, int type, const mjtNum size[3],
                  const mjtNum pos[3], const mjtNum mat[9], const float rgba[4]); */
pub fn mjv_initGeom(
    type_: mjtGeom,
    size: Option<[f64; 3]>,
    pos: Option<[f64; 3]>,
    mat: Option<[f64; 9]>,
    rgba: Option<[f32; 4]>,
) -> mjvGeom {
    let mut c = std::mem::MaybeUninit::<mjvGeom>::uninit();
    unsafe {
        crate::bindgen::mjv_initGeom(
            c.as_mut_ptr(),
            type_.0 as i32,
            size.map_or(std::ptr::null(), |a| &a),
            pos.map_or(std::ptr::null(), |a| &a),
            mat.map_or(std::ptr::null(), |a| &a),
            rgba.map_or(std::ptr::null(), |a| &a),
        );
    }
    unsafe { c.assume_init() }
}

/// Set (type, size, pos, mat) for connector-type geom between given points.
/// Assume that `mjv_initGeom` (== `mjvGeom::new`) was already called to set all other properties.
/// Width of `mjGEOM_LINE` is denominated in pixels.
/* void mjv_connector(mjvGeom* geom, int type, mjtNum width,
                   const mjtNum from[3], const mjtNum to[3]); */
pub fn mjv_connector(
    geom: &mut mjvGeom,
    type_: crate::bindgen::mjtGeom,
    width: f64,
    from: [f64; 3],
    to: [f64; 3],
) {
    unsafe {
        crate::bindgen::mjv_connector(
            geom,
            type_.0 as i32,
            width,
            &from,
            &to,
        );
    }
}

/// Set default abstract scene.
/// 
/// **note**: [`MjvScene`] calls this function in its `Default` implementation.
/// 
/// **note**: Call [`mjv_makeScene`] to allocate resources in abstract scene
///           before starting a simulation with the `mjvScene`.
/// 
/* void mjv_defaultScene(mjvScene* scn); */
pub fn mjv_defaultScene() -> MjvScene {
    let mut c = std::mem::ManuallyDrop::new(std::mem::MaybeUninit::<crate::bindgen::mjvScene>::uninit());
    unsafe { crate::bindgen::mjv_defaultScene(c.as_mut_ptr()); }
    MjvScene::from_raw(c.as_mut_ptr())
}

/// Allocate resources in abstract scene.
/// 
/// **note**: [`mjvScene`] calls this function in its `new` implementation.
/* void mjv_makeScene(const mjModel* m, mjvScene* scn, int maxgeom); */
pub fn mjv_makeScene(
    model: &MjModel,
    scene: &mut MjvScene,
    maxgeom: usize,
) {
    unsafe {
        crate::bindgen::mjv_makeScene(
            model.as_ptr(),
            scene.as_mut_ptr(),
            maxgeom as i32,
        );
    }
}

/// Free abstract scene.
/// 
/// **note**: [`MjvScene`] calls this function in its `Drop` implementation.
/* void mjv_freeScene(mjvScene* scn); */
pub fn mjv_freeScene(scene: &mut MjvScene) {
    unsafe { crate::bindgen::mjv_freeScene(scene.as_mut_ptr()) }
}

/// Update entire scene given model state.
/* void mjv_updateScene(const mjModel* m, mjData* d, const mjvOption* opt,
                     const mjvPerturb* pert, mjvCamera* cam, int catmask, mjvScene* scn); */
pub fn mjv_updateScene(
    model: &MjModel,
    data: &mut MjData,
    opt: &mjvOption,
    pert: Option<&mjvPerturb>,
    cam: &mut mjvCamera,
    catmask: crate::bindgen::mjtCatBit,
    scene: &mut MjvScene,
) {
    unsafe {
        crate::bindgen::mjv_updateScene(
            model.as_ptr(),
            data.as_mut_ptr(),
            opt,
            pert.map_or(std::ptr::null(), |p| p),
            cam,
            catmask.0 as i32,
            scene.as_mut_ptr(),
        );
    }
}

/// Copy mjModel, skip large arrays not required for abstract visualization.
/* void mjv_copyModel(mjModel* dest, const mjModel* src); */
pub fn mjv_copyModel(dest: &mut MjModel, src: &MjModel) {
    unsafe { crate::bindgen::mjv_copyModel(dest.as_mut_ptr(), src.as_ptr()) }
}

/// Add geoms from selected categories.
/* void mjv_addGeoms(const mjModel* m, mjData* d, const mjvOption* opt,
                  const mjvPerturb* pert, int catmask, mjvScene* scn); */
pub fn mjv_addGeoms(
    model: &MjModel,
    data: &mut MjData,
    opt: &mjvOption,
    pert: &mjvPerturb,
    catmask: crate::bindgen::mjtCatBit,
    scene: &mut MjvScene,
) {
    unsafe {
        crate::bindgen::mjv_addGeoms(
            model.as_ptr(),
            data.as_mut_ptr(),
            opt,
            pert,
            catmask.0 as i32,
            scene.as_mut_ptr(),
        );
    }
}

/// Make list of lights.
/* void mjv_makeLights(const mjModel* m, const mjData* d, mjvScene* scn); */
pub fn mjv_makeLights(
    model: &MjModel,
    data: &MjData,
    scene: &mut MjvScene,
) {
    unsafe {
        crate::bindgen::mjv_makeLights(
            model.as_ptr(),
            data.as_ptr(),
            scene.as_mut_ptr(),
        );
    }
}

/// Update camera.
/* void mjv_updateCamera(const mjModel* m, const mjData* d, mjvCamera* cam, mjvScene* scn); */
pub fn mjv_updateCamera(
    model: &MjModel,
    data: &MjData,
    cam: &mut mjvCamera,
    scene: &mut MjvScene,
) {
    unsafe {
        crate::bindgen::mjv_updateCamera(
            model.as_ptr(),
            data.as_ptr(),
            cam,
            scene.as_mut_ptr(),
        );
    }
}

/// Update skins.
/* void mjv_updateSkin(const mjModel* m, const mjData* d, mjvScene* scn); */
pub fn mjv_updateSkin(
    model: &MjModel,
    data: &MjData,
    scene: &mut MjvScene,
) {
    unsafe {
        crate::bindgen::mjv_updateSkin(
            model.as_ptr(),
            data.as_ptr(),
            scene.as_mut_ptr(),
        );
    }
}

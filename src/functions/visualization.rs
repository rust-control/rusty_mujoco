//! # [Visualization](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#visualization)
//! 
//! The functions in this section implement abstract visualization.
//! The results are used by the OpenGL renderer, and can also be used by users
//! wishing to implement their own renderer, or hook up MuJoCo to
//! advanced rendering tools such as Unity or Unreal Engine.
//! See [simulate](https://mujoco.readthedocs.io/en/stable/programming/samples.html#sasimulate)
//! for illustration of how to use these functions.

use crate::{MjModel, MjData, MjvOption, MjvScene, MjvCamera, MjvPerturb, MjvGeom};

/// Set default visualization options.
/// 
/// **note**: `rusty_mujoco::MjvOption` calls this function in its `Default` implementation.
/* void mjv_defaultOption(mjvOption* opt); */
pub fn mjv_defaultOption() -> MjvOption {
    let mut c = crate::bindgen::mjvOption::default();
    unsafe { crate::bindgen::mjv_defaultOption(&mut c); }
    c.into()
}
impl Default for MjvOption {
    fn default() -> Self {
        mjv_defaultOption()
    }
}

/// Set default figure.
/// 
/// **note**: `rusty_mujoco::MjvFigure` calls this function in its `Default` implementation.
/* void mjv_defaultFigure(mjvFigure* fig); */
pub fn mjv_defaultFigure() -> crate::MjvFigure {
    let mut c = crate::bindgen::mjvFigure::default();
    unsafe { crate::bindgen::mjv_defaultFigure(&mut c); }
    c.into()
}
impl Default for crate::MjvFigure {
    fn default() -> Self {
        mjv_defaultFigure()
    }
}

/// Initialize given geom fields when not `None`, set the rest to their default values.
/// 
/// **note**: `rusty_mujoco::MjvGeom` calls this function with all default options in its `new` implementation.
/* void mjv_initGeom(mjvGeom* geom, int type, const mjtNum size[3],
                  const mjtNum pos[3], const mjtNum mat[9], const float rgba[4]); */
pub fn mjv_initGeom(
    type_: crate::bindgen::mjtGeom,
    size: Option<[f64; 3]>,
    pos: Option<[f64; 3]>,
    mat: Option<[f64; 9]>,
    rgba: Option<[f32; 4]>,
) -> MjvGeom {
    let mut c = crate::bindgen::mjvGeom::default();
    unsafe {
        crate::bindgen::mjv_initGeom(
            &mut c,
            type_.0 as i32,
            size.map_or(std::ptr::null(), |a| &a),
            pos.map_or(std::ptr::null(), |a| &a),
            mat.map_or(std::ptr::null(), |a| &a),
            rgba.map_or(std::ptr::null(), |a| &a),
        );
    }
    c.into()
}
impl MjvGeom {
    pub fn new(type_: crate::bindgen::mjtGeom) -> Self {
        mjv_initGeom(type_, None, None, None, None)
    }
}

/// Set (type, size, pos, mat) for connector-type geom between given points.
/// Assume that `mjv_initGeom` (== `MjvGeom::new`) was already called to set all other properties.
/// Width of `mjGEOM_LINE` is denominated in pixels.
/* void mjv_connector(mjvGeom* geom, int type, mjtNum width,
                   const mjtNum from[3], const mjtNum to[3]); */
pub fn mjv_connector(
    geom: &mut MjvGeom,
    type_: crate::bindgen::mjtGeom,
    width: f64,
    from: [f64; 3],
    to: [f64; 3],
) {
    unsafe {
        crate::bindgen::mjv_connector(
            geom.as_mut(),
            type_.0 as i32,
            width,
            &from,
            &to,
        );
    }
}

/// Set default abstract scene.
/// 
/// **note**: `rusty_mujoco::MjvScene` calls this function in its `Default` implementation.
/// 
/// **note**: Call [`mjv_makeScene`] to allocate resources in abstract scene
///           before starting a simulation with the `MjvScene`.
/// 
/* void mjv_defaultScene(mjvScene* scn); */
pub fn mjv_defaultScene() -> MjvScene {
    let mut c = crate::bindgen::mjvScene::default();
    unsafe { crate::bindgen::mjv_defaultScene(&mut c); }
    c.into()
}
impl Default for MjvScene {
    /// **note**: Call [`mjv_makeScene`] to allocate resources in abstract scene
    ///           before starting a simulation with the `MjvScene`.
    fn default() -> Self {
        mjv_defaultScene()
    }
}

/// Allocate resources in abstract scene.
/// 
/// **note**: `rusty_mujoco::MjvScene` calls this function in its `new` implementation.
/* void mjv_makeScene(const mjModel* m, mjvScene* scn, int maxgeom); */
pub fn mjv_makeScene(
    model: &MjModel,
    scene: &mut MjvScene,
    maxgeom: usize,
) {
    unsafe {
        crate::bindgen::mjv_makeScene(
            model.as_ref(),
            scene.as_mut(),
            maxgeom as i32,
        );
    }
}
impl MjvScene {
    /// Create a new abstract scene with resources allocated for `maxgeom` geoms.
    /// 
    /// This internally calls:
    /// 
    /// 1. [`mjv_defaultScene`] to set default values for the scene.
    /// 2. [`mjv_makeScene`] to allocate resources in the scene.
    pub fn new(model: &MjModel, maxgeom: usize) -> Self {
        let mut scene = MjvScene::default();
        mjv_makeScene(model, &mut scene, maxgeom);
        scene
    }
}

/// Free abstract scene.
/// 
/// **note**: `rusty_mujoco::MjvScene` calls this function in its `Drop` implementation.
/* void mjv_freeScene(mjvScene* scn); */
pub fn mjv_freeScene(scene: &mut MjvScene) {
    unsafe { crate::bindgen::mjv_freeScene(scene.as_mut()) }
}
impl Drop for MjvScene {
    fn drop(&mut self) {
        mjv_freeScene(self);
    }
}

/// Update entire scene given model state.
/* void mjv_updateScene(const mjModel* m, mjData* d, const mjvOption* opt,
                     const mjvPerturb* pert, mjvCamera* cam, int catmask, mjvScene* scn); */
pub fn mjv_updateScene(
    model: &MjModel,
    data: &mut MjData,
    opt: &MjvOption,
    pert: &MjvPerturb,
    cam: &mut MjvCamera,
    catmask: crate::bindgen::mjtCatBit,
    scene: &mut MjvScene,
) {
    unsafe {
        crate::bindgen::mjv_updateScene(
            model.as_ref(),
            data.as_mut(),
            opt.as_ref(),
            pert.as_ref(),
            cam.as_mut(),
            catmask.0 as i32,
            scene.as_mut(),
        );
    }
}

/// Copy mjModel, skip large arrays not required for abstract visualization.
/* void mjv_copyModel(mjModel* dest, const mjModel* src); */
pub fn mjv_copyModel(dest: &mut MjModel, src: &MjModel) {
    unsafe { crate::bindgen::mjv_copyModel(dest.as_mut(), src.as_ref()) }
}

/// Add geoms from selected categories.
/* void mjv_addGeoms(const mjModel* m, mjData* d, const mjvOption* opt,
                  const mjvPerturb* pert, int catmask, mjvScene* scn); */
pub fn mjv_addGeoms(
    model: &MjModel,
    data: &mut MjData,
    opt: &MjvOption,
    pert: &MjvPerturb,
    catmask: crate::bindgen::mjtCatBit,
    scene: &mut MjvScene,
) {
    unsafe {
        crate::bindgen::mjv_addGeoms(
            model.as_ref(),
            data.as_mut(),
            opt.as_ref(),
            pert.as_ref(),
            catmask.0 as i32,
            scene.as_mut(),
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
            model.as_ref(),
            data.as_ref(),
            scene.as_mut(),
        );
    }
}

/// Update camera.
/* void mjv_updateCamera(const mjModel* m, const mjData* d, mjvCamera* cam, mjvScene* scn); */
pub fn mjv_updateCamera(
    model: &MjModel,
    data: &MjData,
    cam: &mut MjvCamera,
    scene: &mut MjvScene,
) {
    unsafe {
        crate::bindgen::mjv_updateCamera(
            model.as_ref(),
            data.as_ref(),
            cam.as_mut(),
            scene.as_mut(),
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
            model.as_ref(),
            data.as_ref(),
            scene.as_mut(),
        );
    }
}

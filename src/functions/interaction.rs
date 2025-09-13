//! # [Interaction](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#interaction)
//! 
//! These functions implement abstract mouse interactions, allowing control over cameras and perturbations.
//! Their use is well illustrated in [simulate](https://mujoco.readthedocs.io/en/stable/programming/samples.html#sasimulate).

use crate::{MjModel, MjData, MjvScene, mjvCamera, mjvGLCamera, mjvPerturb, mjvOption};
use crate::{ObjectId, obj};

/// Set default camera.
/// 
/// **note**: [`mjvCamera`] calls this function in its `Default` implementation.
/* void mjv_defaultCamera(mjvCamera* cam); */
pub fn mjv_defaultCamera() -> mjvCamera {
    let mut cam = std::mem::MaybeUninit::<mjvCamera>::uninit();
    unsafe { crate::bindgen::mjv_defaultCamera(cam.as_mut_ptr()) };
    unsafe { cam.assume_init() }
}

/// Set default free camera.
/* void mjv_defaultFreeCamera(const mjModel* m, mjvCamera* cam); */
pub fn mjv_defaultFreeCamera(m: &MjModel) -> mjvCamera {
    let mut cam = std::mem::MaybeUninit::<mjvCamera>::uninit();
    unsafe { crate::bindgen::mjv_defaultFreeCamera(m.as_ptr(), cam.as_mut_ptr()) };
    unsafe { cam.assume_init() }
}

/// Set default perturbation.
/// 
/// **note**: [`mjvPerturb`] calls this function in its `Default` implementation.
/* void mjv_defaultPerturb(mjvPerturb* pert); */
pub fn mjv_defaultPerturb() -> mjvPerturb {
    let mut pert = std::mem::MaybeUninit::<mjvPerturb>::uninit();
    unsafe { crate::bindgen::mjv_defaultPerturb(pert.as_mut_ptr()) };
    unsafe { pert.assume_init() }
}

/// Transform pose from room to model space,
/// returning `(modelpos, modelquat)`.
/* void mjv_room2model(mjtNum modelpos[3], mjtNum modelquat[4], const mjtNum roompos[3],
                    const mjtNum roomquat[4], const mjvScene* scn); */
pub fn mjv_room2model(
    roompos: [f64; 3],
    roomquat: [f64; 4],
    scn: &MjvScene,
) -> ([f64; 3], [f64; 4]) {
    let mut modelpos = [0.0; 3];
    let mut modelquat = [0.0; 4];

    unsafe {
        crate::bindgen::mjv_room2model(
            &mut modelpos,
            &mut modelquat,
            &roompos,
            &roomquat,
            scn.as_ptr(),
        );
    }

    (modelpos, modelquat)
}

/// Transform pose from model to room space,
/// returning `(roompos, roomquat)`.
/* void mjv_model2room(mjtNum roompos[3], mjtNum roomquat[4], const mjtNum modelpos[3],
                    const mjtNum modelquat[4], const mjvScene* scn); */
pub fn mjv_model2room(
    modelpos: [f64; 3],
    modelquat: [f64; 4],
    scn: &MjvScene,
) -> ([f64; 3], [f64; 4]) {
    let mut roompos = [0.0; 3];
    let mut roomquat = [0.0; 4];

    unsafe {
        crate::bindgen::mjv_model2room(
            &mut roompos,
            &mut roomquat,
            &modelpos,
            &modelquat,
            scn.as_ptr(),
        );
    }

    (roompos, roomquat)
}

/// Get camera info in model space; average left and right OpenGL cameras,
/// returning `(headpos, forward, up)`.
/* void mjv_cameraInModel(mjtNum headpos[3], mjtNum forward[3], mjtNum up[3],
                       const mjvScene* scn); */
pub fn mjv_cameraInModel(scn: &MjvScene) -> ([f64; 3], [f64; 3], [f64; 3]) {
    let mut headpos = [0.0; 3];
    let mut forward = [0.0; 3];
    let mut up = [0.0; 3];

    unsafe {
        crate::bindgen::mjv_cameraInModel(
            &mut headpos,
            &mut forward,
            &mut up,
            scn.as_ptr(),
        );
    }

    (headpos, forward, up)
}

/// Get camera info in room space; average left and right OpenGL cameras,
/// returning `(headpos, forward, up)`.
/* void mjv_cameraInRoom(mjtNum headpos[3], mjtNum forward[3], mjtNum up[3],
                      const mjvScene* scn); */
pub fn mjv_cameraInRoom(scn: &MjvScene) -> ([f64; 3], [f64; 3], [f64; 3]) {
    let mut headpos = [0.0; 3];
    let mut forward = [0.0; 3];
    let mut up = [0.0; 3];

    unsafe {
        crate::bindgen::mjv_cameraInRoom(
            &mut headpos,
            &mut forward,
            &mut up,
            scn.as_ptr(),
        );
    }

    (headpos, forward, up)
}

/// Get frustum height at unit distance from camera; average left and right OpenGL cameras.
/* mjtNum mjv_frustumHeight(const mjvScene* scn); */
pub fn mjv_frustumHeight(scn: &MjvScene) -> f64 {
    unsafe { crate::bindgen::mjv_frustumHeight(scn.as_ptr()) }
}

/// Rotate 3D vec in horizontal plane by angle between (0,1) and (forward_x,forward_y).
/* void mjv_alignToCamera(mjtNum res[3], const mjtNum vec[3], const mjtNum forward[3]); */
pub fn mjv_alignToCamera(vec: [f64; 3], forward: [f64; 3]) -> [f64; 3] {
    let mut res = [0.0; 3];
    unsafe {
        crate::bindgen::mjv_alignToCamera(
            &mut res,
            &vec,
            &forward,
        );
    }
    res
}

/// Move camera with mouse.
/* void mjv_moveCamera(const mjModel* m, int action, mjtNum reldx, mjtNum reldy,
                    const mjvScene* scn, mjvCamera* cam); */
pub fn mjv_moveCamera(
    m: &MjModel,
    action: crate::bindgen::mjtMouse,
    reldx: f64,
    reldy: f64,
    scn: &MjvScene,
    cam: &mut mjvCamera,
) {
    unsafe {
        crate::bindgen::mjv_moveCamera(
            m.as_ptr(),
            action.0 as i32,
            reldx,
            reldy,
            scn.as_ptr(),
            cam,
        );
    }
}

/// Move perturb object with mouse.
/* void mjv_movePerturb(const mjModel* m, const mjData* d, int action, mjtNum reldx,
                     mjtNum reldy, const mjvScene* scn, mjvPerturb* pert); */
pub fn mjv_movePerturb(
    m: &MjModel,
    d: &MjData,
    action: crate::bindgen::mjtMouse,
    reldx: f64,
    reldy: f64,
    scn: &MjvScene,
    pert: &mut mjvPerturb,
) {
    unsafe {
        crate::bindgen::mjv_movePerturb(
            m.as_ptr(),
            d.as_ptr(),
            action.0 as i32,
            reldx,
            reldy,
            scn.as_ptr(),
            pert,
        );
    }
}

/// Move model with mouse.
/* void mjv_moveModel(const mjModel* m, int action, mjtNum reldx, mjtNum reldy,
                   const mjtNum roomup[3], mjvScene* scn); */
pub fn mjv_moveModel(
    m: &MjModel,
    action: crate::bindgen::mjtMouse,
    reldx: f64,
    reldy: f64,
    roomup: [f64; 3],
    scn: &mut MjvScene,
) {
    unsafe {
        crate::bindgen::mjv_moveModel(
            m.as_ptr(),
            action.0 as i32,
            reldx,
            reldy,
            &roomup,
            scn.as_mut_ptr(),
        );
    }
}

/// Copy perturb pos,quat from selected body; set scale for perturbation.
/* void mjv_initPerturb(const mjModel* m, mjData* d, const mjvScene* scn, mjvPerturb* pert); */
pub fn mjv_initPerturb(m: &MjModel, d: &mut MjData, scn: &MjvScene, pert: &mut mjvPerturb) {
    unsafe {
        crate::bindgen::mjv_initPerturb(m.as_ptr(), d.as_mut_ptr(), scn.as_ptr(), pert);
    }
}

/// Set perturb pos,quat in d->mocap when selected body is mocap, and in d->qpos otherwise.
/// Write d->qpos only if flg_paused and subtree root for selected body has free joint.
/* void mjv_applyPerturbPose(const mjModel* m, mjData* d, const mjvPerturb* pert,
                          int flg_paused); */
pub fn mjv_applyPerturbPose(
    m: &MjModel,
    d: &mut MjData,
    pert: &mjvPerturb,
    flg_paused: bool,
) {
    unsafe {
        crate::bindgen::mjv_applyPerturbPose(
            m.as_ptr(),
            d.as_mut_ptr(),
            pert,
            if flg_paused { 1 } else { 0 },
        );
    }
}

/// Set perturb force,torque in d->xfrc_applied, if selected body is dynamic.
/* void mjv_applyPerturbForce(const mjModel* m, mjData* d, const mjvPerturb* pert); */
pub fn mjv_applyPerturbForce(m: &MjModel, d: &mut MjData, pert: &mjvPerturb) {
    unsafe {
        crate::bindgen::mjv_applyPerturbForce(m.as_ptr(), d.as_mut_ptr(), pert);
    }
}

/// Return the average of two OpenGL cameras.
/* mjvGLCamera mjv_averageCamera(const mjvGLCamera* cam1, const mjvGLCamera* cam2); */
pub fn mjv_averageCamera(cam1: &mjvGLCamera, cam2: &mjvGLCamera) -> mjvGLCamera {
    unsafe {
        crate::bindgen::mjv_averageCamera(
            cam1,
            cam2,
        )
    }
}

mod mjv_select {
    use super::*;
    pub struct SelectResult {
        pub geomid: Option<ObjectId<obj::Geom>>,
        pub flexid: Option<ObjectId<obj::Flex>>,
        pub skinid: Option<ObjectId<obj::Skin>>,
        pub selpnt: [f64; 3],
    }
}

/// This function is used for mouse selection, relying on ray intersections.
/// 
/// `aspectratio` is the viewport width/height.
/// `relx` and `rely` are the relative coordinates of the 2D point of interest
/// in the viewport (usually mouse cursor).
/// 
/// The function returns the id of the geom under the specified 2D point,
/// or `None` if there is no geom
/// (note that they skybox if present is not a model geom).
/// 
/// The 3D coordinates of the clicked point are returned (`selpnt`). See
/// [simulate](https://mujoco.readthedocs.io/en/stable/programming/samples.html#sasimulate)
/// for an illustration.
/* int mjv_select(const mjModel* m, const mjData* d, const mjvOption* vopt,
               mjtNum aspectratio, mjtNum relx, mjtNum rely,
               const mjvScene* scn, mjtNum selpnt[3],
               int geomid[1], int flexid[1], int skinid[1]); */
pub fn mjv_select(
    m: &MjModel,
    d: &MjData,
    vopt: &mjvOption,
    aspectratio: f64,
    relx: f64,
    rely: f64,
    scn: &MjvScene,
) -> mjv_select::SelectResult {
    let mut geomid = [-1];
    let mut flexid = [-1];
    let mut skinid = [-1];
    let mut selpnt = [0.0; 3];

    unsafe {
        crate::bindgen::mjv_select(
            m.as_ptr(),
            d.as_ptr(),
            vopt,
            aspectratio,
            relx,
            rely,
            scn.as_ptr(),
            &mut selpnt,
            &mut geomid,
            &mut flexid,
            &mut skinid,
        );
    }

    mjv_select::SelectResult {
        geomid: if geomid[0] < 0 {None} else {Some(unsafe { ObjectId::new_unchecked(geomid[0] as usize) })},
        flexid: if flexid[0] < 0 {None} else {Some(unsafe { ObjectId::new_unchecked(geomid[0] as usize) })},
        skinid: if skinid[0] < 0 {None} else {Some(unsafe { ObjectId::new_unchecked(geomid[0] as usize) })},
        selpnt,
    }
}

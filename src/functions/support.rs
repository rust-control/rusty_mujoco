/* https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#support */

//! These are support functions that need access to mjModel and mjData, unlike the utility functions which do not need such access. Support functions are called within the simulator but some of them can also be useful for custom computations, and are documented in more detail below.

use crate::{MjModel, MjData, MjContact, ObjectId, obj};

/// Returns the number of mjtNum⁠s required for a given state specification. The bits of the integer spec correspond to element fields of mjtState.
pub fn mj_stateSize(m: &MjModel, spec: crate::bindgen::mjtState) -> usize {
    unsafe { crate::bindgen::mj_stateSize(m.as_ref(), spec as u32) as usize }
}

/// Copy concatenated state components specified by spec from d into state. The bits of the integer spec correspond to element fields of mjtState. Fails with mju_error if spec is invalid.
pub fn mj_getState(
    m: &MjModel,
    d: &MjData,
    state: &mut [f64],
    spec: crate::bindgen::mjtState,
) {
    #[cfg(debug_assertions)] {
        assert_eq!(state.len(), mj_stateSize(m, spec));
    }
    unsafe {
        crate::bindgen::mj_getState(
            m.as_ref(),
            d.as_ref(),
            state.as_mut_ptr(),
            spec as u32,
        )
    }
}

/// Copy concatenated state components specified by spec from state into d. The bits of the integer spec correspond to element fields of mjtState. Fails with mju_error if spec is invalid.
pub fn mj_setState(
    m: &MjModel,
    d: &mut MjData,
    state: &[f64],
    spec: crate::bindgen::mjtState,
) {
    #[cfg(debug_assertions)] {
        assert_eq!(state.len(), mj_stateSize(m, spec));
    }
    unsafe {
        crate::bindgen::mj_setState(
            m.as_ref(),
            d.as_mut(),
            state.as_ptr(),
            spec as u32,
        )
    }
}

/// Copy current state to the k-th model keyframe.
pub fn mj_setKeyframe(m: &mut MjModel, d: &mut MjData, k: usize) {
    unsafe { crate::bindgen::mj_setKeyframe(m.as_mut(), d.as_mut(), k as i32) }
}

/// Add contact to d->contact list
pub fn mj_addContact(m: &MjModel, d: &mut MjData, con: &MjContact) -> Result<(), ()> {
    let res = unsafe { crate::bindgen::mj_addContact(m.as_ref(), d.as_mut(), con.as_ref()) };
    /*
        https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-addcontact
        > return 0 if success; 1 if buffer full.
    */
    if res == 0 {Ok(())} else {Err(())}
}

/// Determine type of friction cone.
pub fn mj_isPyramidal(m: &MjModel) -> bool {
    unsafe { crate::bindgen::mj_isPyramidal(m.as_ref()) != 0 }
}

/// Determine type of constraint Jacobian.
pub fn mj_isSparse(m: &MjModel) -> bool {
    unsafe { crate::bindgen::mj_isSparse(m.as_ref()) != 0 }
}

/// Determine type of solver (PGS is dual, CG and Newton are primal).
pub fn mj_isDual(m: &MjModel) -> bool {
    unsafe { crate::bindgen::mj_isDual(m.as_ref()) != 0 }
}

/// This function multiplies the constraint Jacobian mjData.efc_J by a vector. Note that the Jacobian can be either dense or sparse; the function is aware of this setting. Multiplication by J maps velocities from joint space to constraint space.
pub fn mj_mulJacVec(m: &MjModel, d: &MjData, mut vec: Vec<f64>) -> Vec<f64> {
    #[cfg(debug_assertions)] {
        assert_eq!(vec.len(), m.nv() as usize);
    }
    let mut res = vec![0.0; d.nefc()];
    unsafe {
        crate::bindgen::mj_mulJacVec(
            m.as_ref(),
            d.as_ref(),
            vec.as_mut_ptr(),
            res.as_mut_ptr(),
        );
    }
    res
}

/// Same as mj_mulJacVec but multiplies by the transpose of the Jacobian. This maps forces from constraint space to joint space.
pub fn mj_mulJacTVec(m: &MjModel, d: &MjData, mut vec: Vec<f64>) -> Vec<f64> {
    #[cfg(debug_assertions)] {
        assert_eq!(vec.len(), d.nefc() as usize);
    }
    let mut res = vec![0.0; m.nv() as usize];
    unsafe {
        crate::bindgen::mj_mulJacTVec(
            m.as_ref(),
            d.as_ref(),
            vec.as_mut_ptr(),
            res.as_mut_ptr(),
        );
    }
    res
}

/// This function computes an end-effector kinematic Jacobian, describing
/// the local linear relationship between the degrees-of-freedom and
/// a given point.
/// 
/// Given a body specified by its ObjectId (body) and
/// a 3D point in the world frame (point) treated as attached to the body,
/// the Jacobian has both translational (jacp) and rotational (jacr) components.
/// 
/// <!--
///     Passing NULL for either pointer will skip that part of the computation.
/// 
///     Rust compiler's optimization will enable like `let (jacp, _) = mj_jac(...)`
///     to perform the same skipping as passing NULL.
/// -->
/// 
/// Each component is a 3-by-`nv` matrix.
/// Each row of this matrix is the gradient of the corresponding
/// coordinate of the specified point with respect to the degrees-of-freedom.
/// The frame with respect to which the Jacobian is computed is
/// centered at the body center-of-mass but aligned with the world frame.
/// The minimal pipeline stages required for Jacobian computations to be consistent
/// with the current generalized positions mjData.qpos are mj_kinematics followed by mj_comPos.
pub fn mj_jac(
    m: &MjModel,
    d: &mut MjData,
    body: ObjectId<obj::Body>,
    point: [f64; 3],
) -> (Vec<f64>, Vec<f64>) {
    let mut jacp = vec![0.0; 3 * m.nv() as usize];
    let mut jacr = vec![0.0; 3 * m.nv() as usize];
    unsafe {
        crate::bindgen::mj_jac(
            m.as_ref(),
            d.as_mut(),
            jacp.as_mut_ptr(),
            jacr.as_mut_ptr(),
            point.as_ptr(),
            body.index as i32,
        );
    }
    (jacp, jacr)
}

/// This and the remaining variants of the Jacobian function call mj_jac internally, with the center of the body, geom or site. They are just shortcuts; the same can be achieved by calling mj_jac directly.
pub fn mj_jacBody(
    m: &MjModel,
    d: &mut MjData,
    body: ObjectId<obj::Body>,
) -> (Vec<f64>, Vec<f64>) {
    let mut jacp = vec![0.0; 3 * m.nv() as usize];
    let mut jacr = vec![0.0; 3 * m.nv() as usize];
    unsafe {
        crate::bindgen::mj_jacBody(
            m.as_ref(),
            d.as_mut(),
            jacp.as_mut_ptr(),
            jacr.as_mut_ptr(),
            body.index as i32,
        )
    };
    (jacp, jacr)
}

/// Compute body center-of-mass end-effector Jacobian.
pub fn mj_jacBodyCom(
    m: &MjModel,
    d: &mut MjData,
    body: ObjectId<obj::Body>,
) -> (Vec<f64>, Vec<f64>) {
    let mut jacp = vec![0.0; 3 * m.nv() as usize];
    let mut jacr = vec![0.0; 3 * m.nv() as usize];
    unsafe {
        crate::bindgen::mj_jacBodyCom(
            m.as_ref(),
            d.as_mut(),
            jacp.as_mut_ptr(),
            jacr.as_mut_ptr(),
            body.index as i32,
        )
    };
    (jacp, jacr)
}

// Compute geom end-effector Jacobian.
pub fn mj_jacGeom(
    m: &MjModel,
    d: &mut MjData,
    geom: ObjectId<obj::Geom>,
) -> (Vec<f64>, Vec<f64>) {
    let mut jacp = vec![0.0; 3 * m.nv() as usize];
    let mut jacr = vec![0.0; 3 * m.nv() as usize];
    unsafe {
        crate::bindgen::mj_jacGeom(
            m.as_ref(),
            d.as_mut(),
            jacp.as_mut_ptr(),
            jacr.as_mut_ptr(),
            geom.index as i32,
        )
    };
    (jacp, jacr)
}

/// Compute site end-effector Jacobian.
pub fn mj_jacSite(
    m: &MjModel,
    d: &mut MjData,
    site: ObjectId<obj::Site>,
) -> (Vec<f64>, Vec<f64>) {
    let mut jacp = vec![0.0; 3 * m.nv() as usize];
    let mut jacr = vec![0.0; 3 * m.nv() as usize];
    unsafe {
        crate::bindgen::mj_jacSite(
            m.as_ref(),
            d.as_mut(),
            jacp.as_mut_ptr(),
            jacr.as_mut_ptr(),
            site.index as i32,
        )
    };
    (jacp, jacr)
}

/// Compute translation end-effector Jacobian of point, and rotation Jacobian of axis.
pub fn mj_jacPointAxis(
    m: &MjModel,
    d: &mut MjData,
    body: ObjectId<obj::Body>,
    point: [f64; 3],
    axis: [f64; 3],
) -> (Vec<f64>, Vec<f64>) {
    let mut jacp = vec![0.0; 3 * m.nv() as usize];
    let mut jacr = vec![0.0; 3 * m.nv() as usize];
    unsafe {
        crate::bindgen::mj_jacPointAxis(
            m.as_ref(),
            d.as_mut(),
            jacp.as_mut_ptr(),
            jacr.as_mut_ptr(),
            point.as_ptr(),
            axis.as_ptr(),
            body.index as i32,
        )
    };
    (jacp, jacr)
}


/// This function computes the time-derivative of an end-effector kinematic Jacobian computed by mj_jac. The minimal pipeline stages required for computation to be consistent with the current generalized positions and velocities mjData.{qpos, qvel} are mj_kinematics, mj_comPos,
pub fn mj_jacDot(
    m: &MjModel,
    d: &mut MjData,
    body: ObjectId<obj::Body>,
    point: [f64; 3],
) -> (Vec<f64>, Vec<f64>) {
    let mut jacp = vec![0.0; 3 * m.nv() as usize];
    let mut jacr = vec![0.0; 3 * m.nv() as usize];
    unsafe {
        crate::bindgen::mj_jacDot(
            m.as_ref(),
            d.as_mut(),
            jacp.as_mut_ptr(),
            jacr.as_mut_ptr(),
            point.as_ptr(),
            body.index as i32,
        )
    };
    (jacp, jacr)
}


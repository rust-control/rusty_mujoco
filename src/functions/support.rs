//! # [Support](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#support)
//! 
//! These are support functions that need access to mjModel and mjData,
//! unlike the utility functions which do not need such access.
//! Support functions are called within the simulator but
//! some of them can also be useful for custom computations.

use crate::{obj, mjContact, mjData, mjModel, ObjectId};

/// Returns the number of mjtNum⁠s required for a given state specification. The bits of the integer spec correspond to element fields of mjtState.
pub fn mj_stateSize(m: &mjModel, spec: crate::bindgen::mjtState) -> usize {
    unsafe { crate::bindgen::mj_stateSize(m, spec.0 as u32) as _ }
}

/// Copy concatenated state components specified by spec from d into state. The bits of the integer spec correspond to element fields of mjtState. Fails with mju_error if spec is invalid.
pub fn mj_getState(
    m: &mjModel,
    d: &mjData,
    state: &mut [f64],
    spec: crate::bindgen::mjtState,
) {
    #[cfg(debug_assertions)] {
        assert_eq!(state.len(), mj_stateSize(m, spec));
    }
    unsafe {
        crate::bindgen::mj_getState(
            m,
            d,
            state.as_mut_ptr(),
            spec.0 as u32,
        )
    }
}

/// Copy concatenated state components specified by spec from state into d. The bits of the integer spec correspond to element fields of mjtState. Fails with mju_error if spec is invalid.
pub fn mj_setState(
    m: &mjModel,
    d: &mut mjData,
    state: &[f64],
    spec: crate::bindgen::mjtState,
) {
    #[cfg(debug_assertions)] {
        assert_eq!(state.len(), mj_stateSize(m, spec));
    }
    unsafe {
        crate::bindgen::mj_setState(
            m,
            d,
            state.as_ptr(),
            spec.0 as u32,
        )
    }
}

/// Copy current state to the k-th model keyframe.
pub fn mj_setKeyframe(m: &mut mjModel, d: &mut mjData, k: usize) {
    unsafe { crate::bindgen::mj_setKeyframe(m, d, k as i32) }
}

/// Add contact to d->contact list
pub fn mj_addContact(m: &mjModel, d: &mut mjData, con: &mjContact) -> Result<(), ()> {
    let res = unsafe { crate::bindgen::mj_addContact(m, d, con) };
    /*
        https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-addcontact
        > return 0 if success; 1 if buffer full.
    */
    if res == 0 {Ok(())} else {Err(())}
}

/// Determine type of friction cone.
pub fn mj_isPyramidal(m: &mjModel) -> bool {
    unsafe { crate::bindgen::mj_isPyramidal(m) != 0 }
}

/// Determine type of constraint Jacobian.
pub fn mj_isSparse(m: &mjModel) -> bool {
    unsafe { crate::bindgen::mj_isSparse(m) != 0 }
}

/// Determine type of solver (PGS is dual, CG and Newton are primal).
pub fn mj_isDual(m: &mjModel) -> bool {
    unsafe { crate::bindgen::mj_isDual(m) != 0 }
}

/// This function multiplies the constraint Jacobian mjData.efc_J by a vector. Note that the Jacobian can be either dense or sparse; the function is aware of this setting. Multiplication by J maps velocities from joint space to constraint space.
pub fn mj_mulJacVec(m: &mjModel, d: &mjData, mut vec: Vec<f64>) -> Vec<f64> {
    #[cfg(debug_assertions)] {
        assert_eq!(vec.len(), m.nv());
    }
    let mut res = vec![0.0; d.nefc()];
    unsafe {
        crate::bindgen::mj_mulJacVec(
            m,
            d,
            vec.as_mut_ptr(),
            res.as_mut_ptr(),
        );
    }
    res
}

/// Same as mj_mulJacVec but multiplies by the transpose of the Jacobian. This maps forces from constraint space to joint space.
pub fn mj_mulJacTVec(m: &mjModel, d: &mjData, mut vec: Vec<f64>) -> Vec<f64> {
    #[cfg(debug_assertions)] {
        assert_eq!(vec.len(), d.nefc());
    }
    let mut res = vec![0.0; m.nv()];
    unsafe {
        crate::bindgen::mj_mulJacTVec(
            m,
            d,
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
    m: &mjModel,
    d: &mut mjData,
    body: ObjectId<obj::Body>,
    point: [f64; 3],
) -> (Vec<f64>, Vec<f64>) {
    let mut jacp = vec![0.0; 3 * m.nv() as usize];
    let mut jacr = vec![0.0; 3 * m.nv() as usize];
    unsafe {
        crate::bindgen::mj_jac(
            m,
            d,
            jacp.as_mut_ptr(),
            jacr.as_mut_ptr(),
            &point,
            body.index() as i32,
        );
    }
    (jacp, jacr)
}

/// This and the remaining variants of the Jacobian function call mj_jac internally, with the center of the body, geom or site. They are just shortcuts; the same can be achieved by calling mj_jac directly.
pub fn mj_jacBody(
    m: &mjModel,
    d: &mut mjData,
    body: ObjectId<obj::Body>,
) -> (Vec<f64>, Vec<f64>) {
    let mut jacp = vec![0.0; 3 * m.nv() as usize];
    let mut jacr = vec![0.0; 3 * m.nv() as usize];
    unsafe {
        crate::bindgen::mj_jacBody(
            m,
            d,
            jacp.as_mut_ptr(),
            jacr.as_mut_ptr(),
            body.index() as i32,
        )
    };
    (jacp, jacr)
}

/// Compute body center-of-mass end-effector Jacobian.
pub fn mj_jacBodyCom(
    m: &mjModel,
    d: &mut mjData,
    body: ObjectId<obj::Body>,
) -> (Vec<f64>, Vec<f64>) {
    let mut jacp = vec![0.0; 3 * m.nv() as usize];
    let mut jacr = vec![0.0; 3 * m.nv() as usize];
    unsafe {
        crate::bindgen::mj_jacBodyCom(
            m,
            d,
            jacp.as_mut_ptr(),
            jacr.as_mut_ptr(),
            body.index() as i32,
        )
    };
    (jacp, jacr)
}

// Compute geom end-effector Jacobian.
pub fn mj_jacGeom(
    m: &mjModel,
    d: &mut mjData,
    geom: ObjectId<obj::Geom>,
) -> (Vec<f64>, Vec<f64>) {
    let mut jacp = vec![0.0; 3 * m.nv() as usize];
    let mut jacr = vec![0.0; 3 * m.nv() as usize];
    unsafe {
        crate::bindgen::mj_jacGeom(
            m,
            d,
            jacp.as_mut_ptr(),
            jacr.as_mut_ptr(),
            geom.index() as i32,
        )
    };
    (jacp, jacr)
}

/// Compute site end-effector Jacobian.
pub fn mj_jacSite(
    m: &mjModel,
    d: &mut mjData,
    site: ObjectId<obj::Site>,
) -> (Vec<f64>, Vec<f64>) {
    let mut jacp = vec![0.0; 3 * m.nv() as usize];
    let mut jacr = vec![0.0; 3 * m.nv() as usize];
    unsafe {
        crate::bindgen::mj_jacSite(
            m,
            d,
            jacp.as_mut_ptr(),
            jacr.as_mut_ptr(),
            site.index() as i32,
        )
    };
    (jacp, jacr)
}

/// Compute translation end-effector Jacobian of point, and rotation Jacobian of axis.
pub fn mj_jacPointAxis(
    m: &mjModel,
    d: &mut mjData,
    body: ObjectId<obj::Body>,
    point: [f64; 3],
    axis: [f64; 3],
) -> (Vec<f64>, Vec<f64>) {
    let mut jacp = vec![0.0; 3 * m.nv() as usize];
    let mut jacr = vec![0.0; 3 * m.nv() as usize];
    unsafe {
        crate::bindgen::mj_jacPointAxis(
            m,
            d,
            jacp.as_mut_ptr(),
            jacr.as_mut_ptr(),
            &point,
            &axis,
            body.index() as i32,
        )
    };
    (jacp, jacr)
}


/// This function computes the time-derivative of an end-effector kinematic Jacobian computed by mj_jac. The minimal pipeline stages required for computation to be consistent with the current generalized positions and velocities mjData.{qpos, qvel} are mj_kinematics, mj_comPos,
pub fn mj_jacDot(
    m: &mjModel,
    d: &mut mjData,
    body: ObjectId<obj::Body>,
    point: [f64; 3],
) -> (Vec<f64>, Vec<f64>) {
    let mut jacp = vec![0.0; 3 * m.nv() as usize];
    let mut jacr = vec![0.0; 3 * m.nv() as usize];
    unsafe {
        crate::bindgen::mj_jacDot(
            m,
            d,
            jacp.as_mut_ptr(),
            jacr.as_mut_ptr(),
            &point,
            body.index() as i32,
        )
    };
    (jacp, jacr)
}

/// his function computes the 3 x nv angular momentum matrix *H(q)*,
/// providing the linear mapping from generalized velocities to subtree angular momentum.
/// More precisely if *h* is the subtree angular momentum of body id in mjData.subtree_angmom (reported by the subtreeangmom sensor)
/// and *qdot* ​is the generalized velocity mjData.qvel, then *h = H qdot*
pub fn mj_angmomMat(
    m: &mjModel,
    d: &mut mjData,
    body: ObjectId<obj::Body>,
) -> Vec<f64> {
    let mut res = vec![0.0; 3 * m.nv()];
    unsafe {
        crate::bindgen::mj_angmomMat(
            m,
            d,
            res.as_mut_ptr(),
            body.index() as i32,
        )
    };
    res
}

/// Get id of object with the specified mjtObj type and name, returns `None` if id not found.
/// 
/// ## Example
/// 
/// ```
/// use rusty_mujoco::obj;
/// 
/// # fn main() {
/// let model = rusty_mujoco::mj_loadXML(
///     "path/to/model.xml"
/// ).unwrap();
/// let body_id = rusty_mujoco::mj_name2id::<obj::Body>(
///     &model,
///     "body_name"
/// );
/// println!("Body ID: {:?}", body_id);
/// # }
/// ```
pub fn mj_name2id<O: crate::Obj>(
    m: &mjModel,
    name: &str,
) -> Option<ObjectId<O>> {
    let c_name = std::ffi::CString::new(name).expect("`name` contains null bytes");
    let index = unsafe {
        crate::bindgen::mj_name2id(m, O::TYPE.0 as i32, c_name.as_ptr())
    };
    if index < 0 {None} else {Some(unsafe { ObjectId::<O>::new_unchecked(index as usize) })}
}

/// Get name of object with the specified mjtObj type and id, returns `None` if name not found.
pub fn mj_id2name<O: crate::Obj>(
    m: &mjModel,
    id: ObjectId<O>,
) -> String {
    let c_name = unsafe {
        crate::bindgen::mj_id2name(m, O::TYPE.0 as i32, id.index() as i32)
    };
    #[cfg(debug_assertions)] {
        assert!(!c_name.is_null(), "`ObjectId` is always expected to have a valid index for the corresponding object type");
    }
    unsafe {std::ffi::CStr::from_ptr(c_name).to_str().unwrap().to_owned()}
}

/// Convert sparse inertia matrix M into full (i.e. dense) matrix.
/// `M` must be of the same size as mjData.`qM`.
/// Returned one is of size `nv x nv`,
pub fn mj_fullM(
    m: &mjModel,
    M: &[f64],
) -> Vec<f64> {
    #[cfg(debug_assertions)] {
        assert_eq!(M.len(), m.nM());
    }
    let mut res = vec![0.0; m.nv() * m.nv()];
    unsafe {
        crate::bindgen::mj_fullM(
            m,
            res.as_mut_ptr(),
            M.as_ptr(),
        );
    }
    res
}

/// This function multiplies the joint-space inertia matrix stored in mjData.qM by a vector.
/// 
/// qM has a custom sparse format that the user should not attempt to manipulate directly.
/// Alternatively one can convert qM to a dense matrix with `mj_fullM`
/// and then user regular matrix-vector multiplication,
/// but this is slower because it no longer benefits from sparsity.
pub fn mj_mulM(
    m: &mjModel,
    d: &mjData,
    mut vec: Vec<f64>,
) -> Vec<f64> {
    #[cfg(debug_assertions)] {
        assert_eq!(vec.len(), m.nv());
    }
    let mut res = vec![0.0; m.nv()];
    unsafe {
        crate::bindgen::mj_mulM(
            m,
            d,
            vec.as_mut_ptr(),
            res.as_mut_ptr(),
        );
    }
    res
}

/// Multiply vector by (inertia matrix)^(1/2).
pub fn mj_mulM2(
    m: &mjModel,
    d: &mjData,
    mut vec: Vec<f64>,
) -> Vec<f64> {
    #[cfg(debug_assertions)] {
        assert_eq!(vec.len(), m.nv());
    }
    let mut res = vec![0.0; m.nv()];
    unsafe {
        crate::bindgen::mj_mulM2(
            m,
            d,
            vec.as_mut_ptr(),
            res.as_mut_ptr(),
        );
    }
    res
}

/// Add inertia matrix to destination matrix.
/// Destination can be sparse or dense when all int are `None`.
/* void mj_addM(const mjModel* m, mjData* d, mjtNum* dst, int* rownnz, int* rowadr, int* colind); */
pub fn mj_addM(
    m: &mjModel,
    d: &mut mjData,
    dst: &mut [f64],
    rownnz: Option<&mut [i32]>,
    rowadr: Option<&mut [i32]>,
    colind: Option<&mut [i32]>,
) {
    #[cfg(debug_assertions)] {
        match (rownnz.as_ref(), rowadr.as_ref(), colind.as_ref()) {
            (Some(rownnz), Some(rowadr), Some(_)) => {
                assert_eq!(rownnz.len(), m.nv());
                assert_eq!(rowadr.len(), m.nv());
            }
            (None, None, None) => {
                assert_eq!(dst.len(), m.nv() * m.nv());
            }
            _ => panic!("If any of rownnz, rowadr or colind is specified, all must be specified"),
        }
    }
    unsafe {
        crate::bindgen::mj_addM(
            m,
            d,
            dst.as_mut_ptr(),
            rownnz.map_or(std::ptr::null_mut(), |x| x.as_mut_ptr()),
            rowadr.map_or(std::ptr::null_mut(), |x| x.as_mut_ptr()),
            colind.map_or(std::ptr::null_mut(), |x| x.as_mut_ptr()),
        );
    }
}

/// This function can be used to apply a Cartesian force and torque to a point on a body,
/// and add the result to the vector mjData.qfrc_applied of all applied forces.
/// Note that the function requires a pointer to this vector,
/// because sometimes we want to add the result to a different vector.
/* void mj_applyFT(
    const mjModel* m, mjData* d,
    const mjtNum force[3], const mjtNum torque[3], const mjtNum point[3],
    int body, mjtNum* qfrc_target); */
pub fn mj_applyFT(
    m: &mjModel,
    d: &mut mjData,
    body: ObjectId<obj::Body>,
    point: [f64; 3],
    force: [f64; 3],
    torque: [f64; 3],
    qfrc_target: &mut [f64],
) {
    unsafe {
        crate::bindgen::mj_applyFT(
            m,
            d,
            &point,
            &force,
            &torque,
            body.index() as i32,
            qfrc_target.as_mut_ptr(),
        );
    }
}

/// Compute object 6D velocity (rot:lin) in object-centered frame, world/local orientation.
/* void mj_objectVelocity(const mjModel* m, const mjData* d,
                       int objtype, int objid, mjtNum res[6], int flg_local); */
pub fn mj_objectVelocity<O: crate::id::Obj>(
    m: &mjModel,
    d: &mjData,
    object: ObjectId<O>,
    local: bool,
) -> [f64; 6] {
    let mut res = [0.0; 6];
    unsafe {
        crate::bindgen::mj_objectVelocity(
            m,
            d,
            O::TYPE.0 as i32,
            object.index() as i32,
            &mut res,
            local as i32,
        );
    }
    res
}

/// Compute object 6D acceleration (rot:lin) in object-centered frame,
/// world/local orientation. If acceleration or force sensors are
/// not present in the model, mj_rnePostConstraint must be manually called
/// in order to calculate mjData.cacc – the total body acceleration,
/// including contributions from the constraint solver.
/* void mj_objectAcceleration(const mjModel* m, const mjData* d,
                           int objtype, int objid, mjtNum res[6], int flg_local); */
pub fn mj_objectAcceleration<O: crate::id::Obj>(
    m: &mjModel,
    d: &mjData,
    object: ObjectId<O>,
    local: bool,
) -> [f64; 6] {
    let mut res = [0.0; 6];
    unsafe {
        crate::bindgen::mj_objectAcceleration(
            m,
            d,
            O::TYPE.0 as i32,
            object.index() as i32,
            &mut res,
            local as i32,
        );
    }
    res
}

/// Returns the smallest signed distance between two geoms and the segment from geom1 to geom2.
/// Returned distances are bounded from above by distmax.
/// 
/// If no collision of distance smaller than distmax is found, the function will return distmax and fromto, if given, will be set to (0, 0, 0, 0, 0, 0).
/* mjtNum mj_geomDistance(const mjModel* m, const mjData* d, int geom1, int geom2,
                       mjtNum distmax, mjtNum fromto[6]); */
pub fn mj_geomDistance(
    m: &mjModel,
    d: &mjData,
    geom1: ObjectId<obj::Geom>,
    geom2: ObjectId<obj::Geom>,
    distmax: f64,
) -> (f64, [f64; 6]) {
    let mut fromto = [0.0; 6];
    let dist = unsafe {
        crate::bindgen::mj_geomDistance(
            m,
            d,
            geom1.index() as i32,
            geom2.index() as i32,
            distmax,
            &mut fromto,
        )
    };
    (dist, fromto)
}

/// Extract 6D force:torque given contact id, in the contact frame.
/* void mj_contactForce(const mjModel* m, const mjData* d, int id, mjtNum result[6]); */
pub fn mj_contactForce(
    m: &mjModel,
    d: &mjData,
    contact_id: usize,
) -> [f64; 6] {
    let mut result = [0.0; 6];
    unsafe {
        crate::bindgen::mj_contactForce(
            m,
            d,
            contact_id as i32,
            &mut result,
        );
    }
    result
}

/// This function subtracts two vectors in the format of qpos
/// (and divides the result by dt), while respecting the properties of quaternions.
/// Recall that unit quaternions represent spatial orientations.
/// They are points on the unit sphere in 4D.
/// 
/// The tangent to that sphere is a 3D plane of rotational velocities.
/// Thus when we subtract two quaternions in the right way,
/// the result is a 3D vector and not a 4D vector.
/// Thus the output qvel has dimensionality nv while the inputs have dimensionality nq.
/*
void mj_differentiatePos(const mjModel* m, mjtNum* qvel, mjtNum dt,
                         const mjtNum* qpos1, const mjtNum* qpos2);
*/
pub fn mj_differentiatePos(
    m: &mjModel,
    qpos1: &[f64],
    qpos2: &[f64],
    dt: f64,
) -> Vec<f64> {
    #[cfg(debug_assertions)] {
        assert_eq!(qpos1.len(), m.nq());
        assert_eq!(qpos2.len(), m.nq());
    }
    let mut qvel = vec![0.0; m.nv()];
    unsafe {
        crate::bindgen::mj_differentiatePos(
            m,
            qvel.as_mut_ptr(),
            dt,
            qpos1.as_ptr(),
            qpos2.as_ptr(),
        );
    }
    qvel
}

/// This is the opposite of mj_differentiatePos. It adds a vector in the format of qvel (scaled by dt) to a vector in the format of qpos.
/* void mj_integratePos(const mjModel* m, mjtNum* qpos, const mjtNum* qvel, mjtNum dt); */
pub fn mj_integratePos(
    m: &mjModel,
    qpos: &mut [f64],
    qvel: &[f64],
    dt: f64,
) {
    #[cfg(debug_assertions)] {
        assert_eq!(qpos.len(), m.nq());
        assert_eq!(qvel.len(), m.nv());
    }
    unsafe {
        crate::bindgen::mj_integratePos(
            m,
            qpos.as_mut_ptr(),
            qvel.as_ptr(),
            dt,
        );
    }
}

/// Normalize all quaternions in qpos-type vector.
/* void mj_normalizeQuat(const mjModel* m, mjtNum* qpos); */
pub fn mj_normalizeQuat(
    m: &mjModel,
    qpos: &mut [f64],
) {
    #[cfg(debug_assertions)] {
        assert_eq!(qpos.len(), m.nq());
    }
    unsafe {
        crate::bindgen::mj_normalizeQuat(m, qpos.as_mut_ptr());
    }
}

/// Map from body local to global Cartesian coordinates, sameframe takes values from mjtSameFrame.
/* void mj_local2Global(mjData* d, mjtNum xpos[3], mjtNum xmat[9], const mjtNum pos[3],
                     const mjtNum quat[4], int body, mjtByte sameframe); */
pub fn mj_local2Global(
    d: &mut mjData,
    xpos: &mut [f64; 3],
    xmat: &mut [f64; 9],
    pos: &[f64; 3],
    quat: &[f64; 4],
    body: ObjectId<obj::Body>,
    sameframe: crate::bindgen::mjtSameFrame,
) {
    unsafe {
        crate::bindgen::mj_local2Global(
            d,
            xpos,
            xmat,
            pos,
            quat,
            body.index() as i32,
            sameframe.0 as u8,
        );
    }
}

/// Sum all body masses.
pub fn mj_getTotalmass(m: &mjModel) -> f64 {
    unsafe { crate::bindgen::mj_getTotalmass(m) }
}

/// Scale body masses and inertias to achieve specified total mass.
/* void mj_setTotalmass(mjModel* m, mjtNum newmass); */
pub fn mj_setTotalmass(
    m: &mut mjModel,
    newmass: f64,
) {
    unsafe { crate::bindgen::mj_setTotalmass(m, newmass) }
}

/// Return a config attribute value of a plugin instance;
/// `None`: invalid plugin instance ID or attribute name
/* const char* mj_getPluginConfig(const mjModel* m, int plugin_id, const char* attrib); */
pub fn mj_getPluginConfig(
    m: &mjModel,
    plugin_id: ObjectId<obj::Plugin>,
    attrib: &str,
) -> Option<String> {
    let c_attrib = std::ffi::CString::new(attrib).expect("`attrib` contains null bytes");
    let c_str = unsafe {
        crate::bindgen::mj_getPluginConfig(m, plugin_id.index() as i32, c_attrib.as_ptr())
    };
    if c_str.is_null() {
        None
    } else {
        Some(unsafe { std::ffi::CStr::from_ptr(c_str).to_str().unwrap().to_owned() })
    }
}

/// Load a dynamic library. The dynamic library is assumed to register one or more plugins.
pub fn mj_loadPluginLibrary(
    path: &str,
) {
    let c_path = std::ffi::CString::new(path).expect("`path` contains null bytes");
    unsafe {
        crate::bindgen::mj_loadPluginLibrary(c_path.as_ptr());
    }
}

/// Return version number: 1.0.2 is encoded as 102.
pub fn mj_version() -> u32 {
    unsafe { crate::bindgen::mj_version() as u32}
}

/// Return the current version of MuJoCo as a string.
pub fn mj_versionString() -> String {
    let c_str = unsafe { crate::bindgen::mj_versionString() };
    if c_str.is_null() {
        String::new()
    } else {
        unsafe { std::ffi::CStr::from_ptr(c_str).to_str().unwrap().to_owned() }
    }
}

//! # [Sub components](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#sub-components)
//! 
//! These are sub-components of the simulation pipeline, called internally from the components above.

/// Evaluate position-dependent sensors.
/* void mj_sensorPos(const mjModel* m, mjData* d); */
pub fn mj_sensorPos(m: &crate::mjModel, d: &mut crate::mjData) {
    unsafe { crate::bindgen::mj_sensorPos(m.as_ptr(), d.as_mut_ptr()) }
}

/// Evaluate velocity-dependent sensors.
/* void mj_sensorVel(const mjModel* m, mjData* d); */
pub fn mj_sensorVel(m: &crate::mjModel, d: &mut crate::mjData) {
    unsafe { crate::bindgen::mj_sensorVel(m.as_ptr(), d.as_mut_ptr()) }
}

/// Evaluate acceleration and force-dependent sensors.
/* void mj_sensorAcc(const mjModel* m, mjData* d); */
pub fn mj_sensorAcc(m: &crate::mjModel, d: &mut crate::mjData) {
    unsafe { crate::bindgen::mj_sensorAcc(m.as_ptr(), d.as_mut_ptr()) }
}

/// Evaluate position-dependent energy (potential).
/* void mj_energyPos(const mjModel* m, mjData* d); */
pub fn mj_energyPos(m: &crate::mjModel, d: &mut crate::mjData) {
    unsafe { crate::bindgen::mj_energyPos(m.as_ptr(), d.as_mut_ptr()) }
}

/// Evaluate velocity-dependent energy (kinetic).
/* void mj_energyVel(const mjModel* m, mjData* d); */
pub fn mj_energyVel(m: &crate::mjModel, d: &mut crate::mjData) {
    unsafe { crate::bindgen::mj_energyVel(m.as_ptr(), d.as_mut_ptr()) }
}

/// Check qpos, reset if any element is too big or nan.
/* void mj_checkPos(const mjModel* m, mjData* d); */
pub fn mj_checkPos(m: &crate::mjModel, d: &mut crate::mjData) {
    unsafe { crate::bindgen::mj_checkPos(m.as_ptr(), d.as_mut_ptr()) }
}

/// Check qvel, reset if any element is too big or nan.
/* void mj_checkVel(const mjModel* m, mjData* d); */
pub fn mj_checkVel(m: &crate::mjModel, d: &mut crate::mjData) {
    unsafe { crate::bindgen::mj_checkVel(m.as_ptr(), d.as_mut_ptr()) }
}

/// Check qacc, reset if any element is too big or nan.
/* void mj_checkAcc(const mjModel* m, mjData* d); */
pub fn mj_checkAcc(m: &crate::mjModel, d: &mut crate::mjData) {
    unsafe { crate::bindgen::mj_checkAcc(m.as_ptr(), d.as_mut_ptr()) }
}

/// Run forward kinematics.
/* void mj_kinematics(const mjModel* m, mjData* d); */
pub fn mj_kinematics(m: &crate::mjModel, d: &mut crate::mjData) {
    unsafe { crate::bindgen::mj_kinematics(m.as_ptr(), d.as_mut_ptr()) }
}

/// Map inertias and motion dofs to global frame centered at CoM.
/* void mj_comPos(const mjModel* m, mjData* d); */
pub fn mj_comPos(m: &crate::mjModel, d: &mut crate::mjData) {
    unsafe { crate::bindgen::mj_comPos(m.as_ptr(), d.as_mut_ptr()) }
}

/// Compute camera and light positions and orientations.
/* void mj_camlight(const mjModel* m, mjData* d); */
pub fn mj_camlight(m: &crate::mjModel, d: &mut crate::mjData) {
    unsafe { crate::bindgen::mj_camlight(m.as_ptr(), d.as_mut_ptr()) }
}

/// Compute flex-related quantities.
/* void mj_flex(const mjModel* m, mjData* d); */
pub fn mj_flex(m: &crate::mjModel, d: &mut crate::mjData) {
    unsafe { crate::bindgen::mj_flex(m.as_ptr(), d.as_mut_ptr()) }
}

/// Compute tendon lengths, velocities and moment arms.
/* void mj_tendon(const mjModel* m, mjData* d); */
pub fn mj_tendon(m: &crate::mjModel, d: &mut crate::mjData) {
    unsafe { crate::bindgen::mj_tendon(m.as_ptr(), d.as_mut_ptr()) }
}

/// Compute actuator transmission lengths and moments.
/* void mj_transmission(const mjModel* m, mjData* d); */
pub fn mj_transmission(m: &crate::mjModel, d: &mut crate::mjData) {
    unsafe { crate::bindgen::mj_transmission(m.as_ptr(), d.as_mut_ptr()) }
}

/// Run composite rigid body inertia algorithm (CRB).
/* void mj_crb(const mjModel* m, mjData* d); */
pub fn mj_crb(m: &crate::mjModel, d: &mut crate::mjData) {
    unsafe { crate::bindgen::mj_crb(m.as_ptr(), d.as_mut_ptr()) }
}

/// Compute sparse _L^T D L_ factorizaton of inertia matrix.
/* void mj_factorM(const mjModel* m, mjData* d); */
pub fn mj_factorM(m: &crate::mjModel, d: &mut crate::mjData) {
    unsafe { crate::bindgen::mj_factorM(m.as_ptr(), d.as_mut_ptr()) }
}

/// Solve linear system _M x = y_ using factorization: _x = (L^TDL)^{-1} y_
/* void mj_solveM(const mjModel* m, mjData* d, mjtNum* x, const mjtNum* y, int n); */
pub fn mj_solveM(
    m: &crate::mjModel,
    d: &mut crate::mjData,
    x: &mut [f64],
    y: &[f64],
    n: usize,
) {
    assert_eq!(x.len(), m.nv());
    assert_eq!(y.len(), m.nv());
    unsafe {
        crate::bindgen::mj_solveM(
            m.as_ptr(),
            d.as_mut_ptr(),
            x.as_mut_ptr(),
            y.as_ptr(),
            n as i32,
        );
    }
}

/// Half of linear solve: _x = \sqrt{D^{-1}} (L^T)^{-1} y_
/* void mj_solveM2(const mjModel* m, mjData* d, mjtNum* x, const mjtNum* y,
                const mjtNum* sqrtInvD, int n); */
pub fn mj_solveM2(
    m: &crate::mjModel,
    d: &mut crate::mjData,
    x: &mut [f64],
    y: &[f64],
    sqrt_inv_d: &[f64],
    n: usize,
) {
    assert_eq!(x.len(), m.nv());
    assert_eq!(y.len(), m.nv());
    assert_eq!(sqrt_inv_d.len(), m.nv());
    unsafe {
        crate::bindgen::mj_solveM2(
            m.as_ptr(),
            d.as_mut_ptr(),
            x.as_mut_ptr(),
            y.as_ptr(),
            sqrt_inv_d.as_ptr(),
            n as i32,
        );
    }
}

/// Compute cvel, cdof_dot.
/* void mj_comVel(const mjModel* m, mjData* d); */
pub fn mj_comVel(m: &crate::mjModel, d: &mut crate::mjData) {
    unsafe { crate::bindgen::mj_comVel(m.as_ptr(), d.as_mut_ptr()) }
}

/// Compute qfrc_passive from spring-dampers, gravity compensation and fluid forces.
/* void mj_passive(const mjModel* m, mjData* d); */
pub fn mj_passive(m: &crate::mjModel, d: &mut crate::mjData) {
    unsafe { crate::bindgen::mj_passive(m.as_ptr(), d.as_mut_ptr()) }
}

/// Sub-tree linear velocity and angular momentum:
/// compute `subtree_linvel`, `subtree_angmom`.
/// This function is triggered automatically if the subtree
/// [velocity](https://mujoco.readthedocs.io/en/stable/XMLreference.html#sensor-subtreelinvel)
/// or [momentum](https://mujoco.readthedocs.io/en/stable/XMLreference.html#sensor-subtreeangmom)
/// sensors are present in the model. It is also triggered for
/// [user sensors](https://mujoco.readthedocs.io/en/stable/XMLreference.html#sensor-user)
/// of [stage](https://mujoco.readthedocs.io/en/stable/XMLreference.html#sensor-user-needstage)
/// “vel”.
/* void mj_subtreeVel(const mjModel* m, mjData* d); */
pub fn mj_subtreeVel(m: &crate::mjModel, d: &mut crate::mjData) {
    unsafe { crate::bindgen::mj_subtreeVel(m.as_ptr(), d.as_mut_ptr()) }
}

/// Recursive Newton Euler: compute _M(q) \ddot{q} + C(q, \dot{q})_.
/// `flg_acc = false` removes the inertial term (i.e. assumes _\ddot{q} = 0_).
/* void mj_rne(const mjModel* m, mjData* d, int flg_acc, mjtNum* result); */
pub fn mj_rne(
    m: &crate::mjModel,
    d: &mut crate::mjData,
    flg_acc: bool,
) -> Vec<f64> {
    let mut result = vec![0.0; m.nv()];
    unsafe {
        crate::bindgen::mj_rne(
            m.as_ptr(),
            d.as_mut_ptr(),
            flg_acc as i32,
            result.as_mut_ptr()
        );
    }
    result
}

/// Recursive Newton Euler with final computed forces and accelerations.
/// Computes three body-level `nv x 6` arrays, all defined in the subtreecom-based
/// [c-frame](https://mujoco.readthedocs.io/en/stable/APIreference/APItypes.html#tynotescom)
/// and arranged in `[rotation(3), translation(3)]` order.
/// 
/// - `cacc`: Body acceleration, required for [`mj_objectAcceleration`](crate::mj_objectAcceleration).
/// - `cfrc_int`: Interaction force with the parent body.
/// - `cfrc_ext`: External force acting on the body.
/// 
/// This function is triggered automatically if the following sensors are present in the model:
/// 
/// - [accelerometer](https://mujoco.readthedocs.io/en/stable/XMLreference.html#sensor-accelerometer)
/// - [force](https://mujoco.readthedocs.io/en/stable/XMLreference.html#sensor-force)
/// - [torque](https://mujoco.readthedocs.io/en/stable/XMLreference.html#sensor-torque)
/// - [framelinacc](https://mujoco.readthedocs.io/en/stable/XMLreference.html#sensor-framelinacc)
/// - [frameangacc](https://mujoco.readthedocs.io/en/stable/XMLreference.html#sensor-frameangacc)
/// 
/// It is also triggered for [user sensors](https://mujoco.readthedocs.io/en/stable/XMLreference.html#sensor-user)
/// of [stage](https://mujoco.readthedocs.io/en/stable/XMLreference.html#sensor-user-needstage)
/// “acc”.
/* void mj_rnePostConstraint(const mjModel* m, mjData* d); */
pub fn mj_rnePostConstraint(m: &crate::mjModel, d: &mut crate::mjData) {
    unsafe { crate::bindgen::mj_rnePostConstraint(m.as_ptr(), d.as_mut_ptr()) }
}

/// Run collision detection.
/* void mj_collision(const mjModel* m, mjData* d); */
pub fn mj_collision(m: &crate::mjModel, d: &mut crate::mjData) {
    unsafe { crate::bindgen::mj_collision(m.as_ptr(), d.as_mut_ptr()) }
}

/// Construct constraints.
/* void mj_makeConstraint(const mjModel* m, mjData* d); */
pub fn mj_makeConstraint(m: &crate::mjModel, d: &mut crate::mjData) {
    unsafe { crate::bindgen::mj_makeConstraint(m.as_ptr(), d.as_mut_ptr()) }
}

/// Find constraint islands.
/* void mj_island(const mjModel* m, mjData* d); */
pub fn mj_island(m: &crate::mjModel, d: &mut crate::mjData) {
    unsafe { crate::bindgen::mj_island(m.as_ptr(), d.as_mut_ptr()) }
}

/// Compute inverse constraint inertia efc_AR.
/* void mj_projectConstraint(const mjModel* m, mjData* d); */
pub fn mj_projectConstraint(m: &crate::mjModel, d: &mut crate::mjData) {
    unsafe { crate::bindgen::mj_projectConstraint(m.as_ptr(), d.as_mut_ptr()) }
}

/// Compute efc_vel, efc_aref.
/* void mj_referenceConstraint(const mjModel* m, mjData* d); */
pub fn mj_referenceConstraint(m: &crate::mjModel, d: &mut crate::mjData) {
    unsafe { crate::bindgen::mj_referenceConstraint(m.as_ptr(), d.as_mut_ptr()) }
}

/// Compute `efc_state`, `efc_force`, `qfrc_constraint`, and (optionally) cone Hessians.
/// 
/// returns `s(jar) where jar = Jac*qacc - aref`.
/* void mj_constraintUpdate(const mjModel* m, mjData* d, const mjtNum* jar,
                         mjtNum cost[1], int flg_coneHessian); */
pub fn mj_constraintUpdate(
    m: &crate::mjModel,
    d: &mut crate::mjData,
    jar: &[f64],
    flg_cone_hessian: bool,
) -> f64 {
    assert_eq!(jar.len(), m.nv());
    let mut cost = [0.0];
    unsafe {
        crate::bindgen::mj_constraintUpdate(
            m.as_ptr(),
            d.as_mut_ptr(),
            jar.as_ptr(),
            &mut cost,
            flg_cone_hessian as i32,
        );
    }
    cost[0]
}

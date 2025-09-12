//! # [Sub components](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#sub-components)
//! 
//! These are sub-components of the simulation pipeline, called internally from the components above.

/// Evaluate position-dependent sensors.
/* void mj_sensorPos(const mjModel* m, mjData* d); */
pub fn mj_sensorPos(m: &crate::MjModel, d: &mut crate::MjData) {
    unsafe { crate::bindgen::mj_sensorPos(m.0, d.0) }
}

/// Evaluate velocity-dependent sensors.
/* void mj_sensorVel(const mjModel* m, mjData* d); */
pub fn mj_sensorVel(m: &crate::MjModel, d: &mut crate::MjData) {
    unsafe { crate::bindgen::mj_sensorVel(m.0, d.0) }
}

/// Evaluate acceleration and force-dependent sensors.
/* void mj_sensorAcc(const mjModel* m, mjData* d); */
pub fn mj_sensorAcc(m: &crate::MjModel, d: &mut crate::MjData) {
    unsafe { crate::bindgen::mj_sensorAcc(m.0, d.0) }
}

/// Evaluate position-dependent energy (potential).
/* void mj_energyPos(const mjModel* m, mjData* d); */
pub fn mj_energyPos(m: &crate::MjModel, d: &mut crate::MjData) {
    unsafe { crate::bindgen::mj_energyPos(m.0, d.0) }
}

/// Evaluate velocity-dependent energy (kinetic).
/* void mj_energyVel(const mjModel* m, mjData* d); */
pub fn mj_energyVel(m: &crate::MjModel, d: &mut crate::MjData) {
    unsafe { crate::bindgen::mj_energyVel(m.0, d.0) }
}

/// Check qpos, reset if any element is too big or nan.
/* void mj_checkPos(const mjModel* m, mjData* d); */
pub fn mj_checkPos(m: &crate::MjModel, d: &mut crate::MjData) {
    unsafe { crate::bindgen::mj_checkPos(m.0, d.0) }
}

/// Check qvel, reset if any element is too big or nan.
/* void mj_checkVel(const mjModel* m, mjData* d); */
pub fn mj_checkVel(m: &crate::MjModel, d: &mut crate::MjData) {
    unsafe { crate::bindgen::mj_checkVel(m.0, d.0) }
}

/// Check qacc, reset if any element is too big or nan.
/* void mj_checkAcc(const mjModel* m, mjData* d); */
pub fn mj_checkAcc(m: &crate::MjModel, d: &mut crate::MjData) {
    unsafe { crate::bindgen::mj_checkAcc(m.0, d.0) }
}

/// Run forward kinematics.
/* void mj_kinematics(const mjModel* m, mjData* d); */
pub fn mj_kinematics(m: &crate::MjModel, d: &mut crate::MjData) {
    unsafe { crate::bindgen::mj_kinematics(m.0, d.0) }
}

/// Map inertias and motion dofs to global frame centered at CoM.
/* void mj_comPos(const mjModel* m, mjData* d); */
pub fn mj_comPos(m: &crate::MjModel, d: &mut crate::MjData) {
    unsafe { crate::bindgen::mj_comPos(m.0, d.0) }
}

/// Compute camera and light positions and orientations.
/* void mj_camlight(const mjModel* m, mjData* d); */
pub fn mj_camlight(m: &crate::MjModel, d: &mut crate::MjData) {
    unsafe { crate::bindgen::mj_camlight(m.0, d.0) }
}

/// Compute flex-related quantities.
/* void mj_flex(const mjModel* m, mjData* d); */
pub fn mj_flex(m: &crate::MjModel, d: &mut crate::MjData) {
    unsafe { crate::bindgen::mj_flex(m.0, d.0) }
}

/// Compute tendon lengths, velocities and moment arms.
/* void mj_tendon(const mjModel* m, mjData* d); */
pub fn mj_tendon(m: &crate::MjModel, d: &mut crate::MjData) {
    unsafe { crate::bindgen::mj_tendon(m.0, d.0) }
}

/// Compute actuator transmission lengths and moments.
/* void mj_transmission(const mjModel* m, mjData* d); */
pub fn mj_transmission(m: &crate::MjModel, d: &mut crate::MjData) {
    unsafe { crate::bindgen::mj_transmission(m.0, d.0) }
}

/// Run composite rigid body inertia algorithm (CRB).
/* void mj_crb(const mjModel* m, mjData* d); */
pub fn mj_crb(m: &crate::MjModel, d: &mut crate::MjData) {
    unsafe { crate::bindgen::mj_crb(m.0, d.0) }
}

/// Compute sparse _L^T D L_ factorizaton of inertia matrix.
/* void mj_factorM(const mjModel* m, mjData* d); */
pub fn mj_factorM(m: &crate::MjModel, d: &mut crate::MjData) {
    unsafe { crate::bindgen::mj_factorM(m.0, d.0) }
}

/// Solve linear system _M x = y_ using factorization: _x = (L^TDL)^{-1} y_
/* void mj_solveM(const mjModel* m, mjData* d, mjtNum* x, const mjtNum* y, int n); */
pub fn mj_solveM(
    m: &crate::MjModel,
    d: &mut crate::MjData,
    x: &mut [f64],
    y: &[f64],
    n: usize,
) {
    assert_eq!(x.len(), m.nv());
    assert_eq!(y.len(), m.nv());
    unsafe {
        crate::bindgen::mj_solveM(
            m.0,
            d.0,
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
    m: &crate::MjModel,
    d: &mut crate::MjData,
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
            m.0,
            d.0,
            x.as_mut_ptr(),
            y.as_ptr(),
            sqrt_inv_d.as_ptr(),
            n as i32,
        );
    }
}

/// Compute cvel, cdof_dot.
/* void mj_comVel(const mjModel* m, mjData* d); */
pub fn mj_comVel(m: &crate::MjModel, d: &mut crate::MjData) {
    unsafe { crate::bindgen::mj_comVel(m.0, d.0) }
}

/// Compute qfrc_passive from spring-dampers, gravity compensation and fluid forces.
/* void mj_passive(const mjModel* m, mjData* d); */
pub fn mj_passive(m: &crate::MjModel, d: &mut crate::MjData) {
    unsafe { crate::bindgen::mj_passive(m.0, d.0) }
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
pub fn mj_subtreeVel(m: &crate::MjModel, d: &mut crate::MjData) {
    unsafe { crate::bindgen::mj_subtreeVel(m.0, d.0) }
}

/// Recursive Newton Euler: compute _M(q) \ddot{q} + C(q, \dot{q})_.
/// `flg_acc = false` removes the inertial term (i.e. assumes _\ddot{q} = 0_).
/* void mj_rne(const mjModel* m, mjData* d, int flg_acc, mjtNum* result); */
pub fn mj_rne(
    m: &crate::MjModel,
    d: &mut crate::MjData,
    flg_acc: bool,
) -> Vec<f64> {
    let mut result = vec![0.0; m.nv()];
    unsafe {
        crate::bindgen::mj_rne(
            m.0,
            d.0,
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
pub fn mj_rnePostConstraint(m: &crate::MjModel, d: &mut crate::MjData) {
    unsafe { crate::bindgen::mj_rnePostConstraint(m.0, d.0) }
}

/// Run collision detection.
/* void mj_collision(const mjModel* m, mjData* d); */
pub fn mj_collision(m: &crate::MjModel, d: &mut crate::MjData) {
    unsafe { crate::bindgen::mj_collision(m.0, d.0) }
}

/// Construct constraints.
/* void mj_makeConstraint(const mjModel* m, mjData* d); */
pub fn mj_makeConstraint(m: &crate::MjModel, d: &mut crate::MjData) {
    unsafe { crate::bindgen::mj_makeConstraint(m.0, d.0) }
}

/// Find constraint islands.
/* void mj_island(const mjModel* m, mjData* d); */
pub fn mj_island(m: &crate::MjModel, d: &mut crate::MjData) {
    unsafe { crate::bindgen::mj_island(m.0, d.0) }
}

/// Compute inverse constraint inertia efc_AR.
/* void mj_projectConstraint(const mjModel* m, mjData* d); */
pub fn mj_projectConstraint(m: &crate::MjModel, d: &mut crate::MjData) {
    unsafe { crate::bindgen::mj_projectConstraint(m.0, d.0) }
}

/// Compute efc_vel, efc_aref.
/* void mj_referenceConstraint(const mjModel* m, mjData* d); */
pub fn mj_referenceConstraint(m: &crate::MjModel, d: &mut crate::MjData) {
    unsafe { crate::bindgen::mj_referenceConstraint(m.0, d.0) }
}

/// Compute `efc_state`, `efc_force`, `qfrc_constraint`, and (optionally) cone Hessians.
/// 
/// returns `s(jar) where jar = Jac*qacc - aref`.
/* void mj_constraintUpdate(const mjModel* m, mjData* d, const mjtNum* jar,
                         mjtNum cost[1], int flg_coneHessian); */
pub fn mj_constraintUpdate(
    m: &crate::MjModel,
    d: &mut crate::MjData,
    jar: &[f64],
    flg_cone_hessian: bool,
) -> f64 {
    assert_eq!(jar.len(), m.nv());
    let mut cost = [0.0];
    unsafe {
        crate::bindgen::mj_constraintUpdate(
            m.0,
            d.0,
            jar.as_ptr(),
            &mut cost,
            flg_cone_hessian as i32,
        );
    }
    cost[0]
}

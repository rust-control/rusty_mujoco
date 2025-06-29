//! # [Components](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#components)
//! 
//! These are components of the simulation pipeline, called internally from `mj_step`, `mj_forward` and `mj_inverse`. It is unlikely that the user will need to call them.

/// Run position-dependent computations.
/* void mj_fwdPosition(const mjModel* m, mjData* d); */
pub fn mj_fwdPosition(m: &crate::mjModel, d: &mut crate::mjData) {
    unsafe { crate::bindgen::mj_fwdPosition(m, d) }
}

/// Run velocity-dependent computations.
/* void mj_fwdVelocity(const mjModel* m, mjData* d); */
pub fn mj_fwdVelocity(m: &crate::mjModel, d: &mut crate::mjData) {
    unsafe { crate::bindgen::mj_fwdVelocity(m, d) }
}

/// Compute actuator force qfrc_actuator.
/* void mj_fwdActuation(const mjModel* m, mjData* d); */
pub fn mj_fwdActuation(m: &crate::mjModel, d: &mut crate::mjData) {
    unsafe { crate::bindgen::mj_fwdActuation(m, d) }
}

/// Add up all non-constraint forces, compute qacc_smooth.
/* void mj_fwdAcceleration(const mjModel* m, mjData* d); */
pub fn mj_fwdAcceleration(m: &crate::mjModel, d: &mut crate::mjData) {
    unsafe { crate::bindgen::mj_fwdAcceleration(m, d) }
}

/// Run selected constraint solver.
/* void mj_fwdConstraint(const mjModel* m, mjData* d); */
pub fn mj_fwdConstraint(m: &crate::mjModel, d: &mut crate::mjData) {
    unsafe { crate::bindgen::mj_fwdConstraint(m, d) }
}

/// Euler integrator, semi-implicit in velocity.
/* void mj_Euler(const mjModel* m, mjData* d); */
pub fn mj_Euler(m: &crate::mjModel, d: &mut crate::mjData) {
    unsafe { crate::bindgen::mj_Euler(m, d) }
}

/// Runge-Kutta explicit order-N integrator.
/* void mj_RungeKutta(const mjModel* m, mjData* d, int N); */
pub fn mj_RungeKutta(m: &crate::mjModel, d: &mut crate::mjData, n: usize) {
    unsafe { crate::bindgen::mj_RungeKutta(m, d, n as i32) }
}

/// Integrates the simulation state using an implicit-in-velocity integrator (either “implicit” or “implicitfast”, see Numerical Integration), and advances simulation time. See mjdata.h for fields computed by this function.
/* void mj_implicit(const mjModel* m, mjData* d); */
pub fn mj_implicit(m: &crate::mjModel, d: &mut crate::mjData) {
    unsafe { crate::bindgen::mj_implicit(m, d) }
}

/// Run position-dependent computations in inverse dynamics.
/* void mj_invPosition(const mjModel* m, mjData* d); */
pub fn mj_invPosition(m: &crate::mjModel, d: &mut crate::mjData) {
    unsafe { crate::bindgen::mj_invPosition(m, d) }
}

/// Run velocity-dependent computations in inverse dynamics.
/* void mj_invVelocity(const mjModel* m, mjData* d); */
pub fn mj_invVelocity(m: &crate::mjModel, d: &mut crate::mjData) {
    unsafe { crate::bindgen::mj_invVelocity(m, d) }
}

/// Apply the analytical formula for inverse constraint dynamics.
/* void mj_invConstraint(const mjModel* m, mjData* d); */
pub fn mj_invConstraint(m: &crate::mjModel, d: &mut crate::mjData) {
    unsafe { crate::bindgen::mj_invConstraint(m, d) }
}

/// Compare forward and inverse dynamics, save results in fwdinv.
/* void mj_compareFwdInv(const mjModel* m, mjData* d); */
pub fn mj_compareFwdInv(m: &crate::mjModel, d: &mut crate::mjData) {
    unsafe { crate::bindgen::mj_compareFwdInv(m, d) }
}

//! # [Main simulation](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#main-simulation)
//! 
//! These are the main entry points to the simulator. Most users will only need to call `mj_step`, which computes everything and advanced the simulation state by one time step. Controls and applied forces must either be set in advance (in mjData.{ctrl, qfrc_applied, xfrc_applied}), or a control callback mjcb_control must be installed which will be called just before the controls and applied forces are needed. Alternatively, one can use `mj_step1` and `mj_step2` which break down the simulation pipeline into computations that are executed before and after the controls are needed; in this way one can set controls that depend on the results from `mj_step1`. Keep in mind though that the RK4 solver does not work with `mj_step1`/2. See Simulation pipeline for a more detailed description.
//! 
//! `mj_forward` performs the same computations as `mj_step` but without the integration. It is useful after loading or resetting a model (to put the entire mjData in a valid state), and also for out-of-order computations that involve sampling or finite-difference approximations.
//! 
//! `mj_inverse` runs the inverse dynamics, and writes its output in mjData.qfrc_inverse. Note that mjData.qacc must be set before calling this function. Given the state (qpos, qvel, act), `mj_forward` maps from force to acceleration, while `mj_inverse` maps from acceleration to force. Mathematically these functions are inverse of each other, but numerically this may not always be the case because the forward dynamics rely on a constraint optimization algorithm which is usually terminated early. The difference between the results of forward and inverse dynamics can be computed with the function `mj_compareFwdInv`, which can be thought of as another solver accuracy check (as well as a general sanity check).
//! 
//! The skip version of `mj_forward` and `mj_inverse` are useful for example when qpos was unchanged but qvel was changed (usually in the context of finite differencing). Then there is no point repeating the computations that only depend on qpos. Calling the dynamics with skipstage = mjSTAGE_POS will achieve these savings.

use crate::{MjModel, MjData};

/// Advance simulation, use control callback to obtain external force and control.
pub fn mj_step(m: &MjModel, d: &mut MjData) {
    unsafe { crate::bindgen::mj_step(m.as_ref(), d.as_mut()) }
}

/// Advance simulation in two steps: before external force and control is set by user.
pub fn mj_step1(m: &MjModel, d: &mut MjData) {
    unsafe { crate::bindgen::mj_step1(m.as_ref(), d.as_mut()) }
}

/// Advance simulation in two steps: after external force and control is set by user.
pub fn mj_step2(m: &MjModel, d: &mut MjData) {
    unsafe { crate::bindgen::mj_step2(m.as_ref(), d.as_mut()) }
}

/// Forward dynamics: same as mj_step but do not integrate in time.
pub fn mj_forward(m: &MjModel, d: &mut MjData) {
    unsafe { crate::bindgen::mj_forward(m.as_ref(), d.as_mut()) }
}

/// Inverse dynamics: qacc must be set before calling.
pub fn mj_inverse(m: &MjModel, d: &mut MjData) {
    unsafe { crate::bindgen::mj_inverse(m.as_ref(), d.as_mut()) }
}

/// Forward dynamics with skip
pub fn mj_forwardSkip(
    m: &MjModel,
    d: &mut MjData,
    skipstage: crate::bindgen::mjtStage,
    skipsensor: bool,
) {
    unsafe {
        crate::bindgen::mj_forwardSkip(
            m.as_ref(),
            d.as_mut(),
            skipstage.0 as i32,
            skipsensor as i32,
    )}
}

/// Inverse dynamics with skip
pub fn mj_inverseSkip(
    m: &MjModel,
    d: &mut MjData,
    skipstage: crate::bindgen::mjtStage,
    skipsensor: bool,
) {
    unsafe {
        crate::bindgen::mj_inverseSkip(
            m.as_ref(),
            d.as_mut(),
            skipstage.0 as i32,
            skipsensor as i32,
        )
    }
}

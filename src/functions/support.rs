/* https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#support */

//! These are support functions that need access to mjModel and mjData, unlike the utility functions which do not need such access. Support functions are called within the simulator but some of them can also be useful for custom computations, and are documented in more detail below.

use crate::{MjModel, MjData, MjContact};

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



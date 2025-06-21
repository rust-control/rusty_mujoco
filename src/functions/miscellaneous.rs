/* https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#miscellaneous */

/// Muscle active force, prm = (range[2], force, scale, lmin, lmax, vmax, fpmax, fvmax).
/* mjtNum mju_muscleGain(mjtNum len, mjtNum vel, const mjtNum lengthrange[2],
                      mjtNum acc0, const mjtNum prm[9]); */
pub fn mju_muscleGain(
    len: f64,
    vel: f64,
    lengthrange: [f64; 2],
    acc0: f64,
    prm: [f64; 9],
) -> f64 {
    unsafe {
        crate::bindgen::mju_muscleGain(
            len,
            vel,
            lengthrange.as_ptr(),
            acc0,
            prm.as_ptr(),
        )
    }
}

/// Muscle passive force, prm = (range[2], force, scale, lmin, lmax, vmax, fpmax, fvmax).
/* mjtNum mju_muscleBias(mjtNum len, const mjtNum lengthrange[2],
                      mjtNum acc0, const mjtNum prm[9]); */
pub fn mju_muscleBias(
    len: f64,
    lengthrange: [f64; 2],
    acc0: f64,
    prm: [f64; 9],
) -> f64 {
    unsafe {
        crate::bindgen::mju_muscleBias(len, lengthrange.as_ptr(), acc0, prm.as_ptr())
    }
}

/// Muscle activation dynamics, prm = (tau_act, tau_deact, smoothing_width).
/* mjtNum mju_muscleDynamics(mjtNum ctrl, mjtNum act, const mjtNum prm[3]); */
pub fn mju_muscleDynamics(ctrl: f64, act: f64, prm: [f64; 3]) -> f64 {
    unsafe { crate::bindgen::mju_muscleDynamics(ctrl, act, prm.as_ptr()) }
}

/// Convert contact force to pyramid representation.
/* void mju_encodePyramid(mjtNum* pyramid, const mjtNum* force, const mjtNum* mu, int dim); */
pub fn mju_encodePyramid<const MU_DIM: usize>(
    force: [f64; 3],
    mu: [f64; MU_DIM],
) -> [f64; 3] {
    #[cfg(debug_assertions)] {
        assert!(matches!(MU_DIM, 1 | 2), "mu must have 1 or 2 dimensions");
    }
    let mut pyramid = [0.0; 3];
    unsafe {
        crate::bindgen::mju_encodePyramid(
            pyramid.as_mut_ptr(),
            force.as_ptr(),
            mu.as_ptr(),
            MU_DIM as i32,
        );
    }
    pyramid
}

/// Convert pyramid representation to contact force.
/* void mju_decodePyramid(mjtNum* force, const mjtNum* pyramid, const mjtNum* mu, int dim); */
pub fn mju_decodePyramid<const MU_DIM: usize>(
    pyramid: [f64; 3],
    mu: [f64; MU_DIM],
) -> [f64; 3] {
    #[cfg(debug_assertions)] {
        assert!(matches!(MU_DIM, 1 | 2), "mu must have 1 or 2 dimensions");
    }
    let mut force = [0.0; 3];
    unsafe {
        crate::bindgen::mju_decodePyramid(
            force.as_mut_ptr(),
            pyramid.as_ptr(),
            mu.as_ptr(),
            MU_DIM as i32,
        );
    }
    force
}

/// Integrate spring-damper analytically, return pos(dt).
/* mjtNum mju_springDamper(mjtNum pos0, mjtNum vel0, mjtNum Kp, mjtNum Kv, mjtNum dt); */
pub fn mju_springDamper(pos0: f64, vel0: f64, Kp: f64, Kv: f64, dt: f64) -> f64 {
    unsafe { crate::bindgen::mju_springDamper(pos0, vel0, Kp, Kv, dt) }
}

/*

Skip following functions as they are not needed for Rust:

- `mju_min` -> `f64::min`
- `mju_max` -> `f64::max`
- `mju_clip` -> `f64::clamp`
- `mju_sign` -> `f64::signum`
- `mju_round` -> `f64::round` + `as {int}`

*/

/*

Skip following functions as they are obvious to be more efficient
when directly implemented in Rust:

- `mju_type2Str` -> `rusty_mujoco::ObjType::to_str(&self) -> &'static str`
- `mju_str2Type` -> `rusty_mujoco::ObjType::from_str(s: &str) -> Self`

*/

/// Return human readable number of bytes using standard letter suffix.
/* const char* mju_writeNumBytes(size_t nbytes); */
pub fn mju_writeNumBytes(nbytes: usize) -> String {
    let c_ptr = unsafe { crate::bindgen::mju_writeNumBytes(nbytes) };
    #[cfg(debug_assertions)] {
        assert!(!c_ptr.is_null(), "`mju_writeNumBytes` unexpectedly returned a null pointer");
    }
    unsafe { std::ffi::CStr::from_ptr(c_ptr).to_str().unwrap().to_owned() }
}

/// Construct a warning message given the warning type and info.
/* const char* mju_warningText(int warning, size_t info); */
pub fn mju_warningText(warning: crate::bindgen::mjtWarning, info: usize) -> String {
    let c_ptr = unsafe { crate::bindgen::mju_warningText(warning as i32, info) };
    #[cfg(debug_assertions)] {
        assert!(!c_ptr.is_null(), "`mju_warningText` unexpectedly returned a null pointer");
    }
    unsafe { std::ffi::CStr::from_ptr(c_ptr).to_str().unwrap().to_owned() }
}

/// Return 1 if nan or abs(x)>mjMAXVAL, 0 otherwise. Used by check functions.
/* int mju_isBad(mjtNum x); */
pub fn mju_isBad(x: f64) -> bool {
    unsafe { crate::bindgen::mju_isBad(x) != 0 }
}

/*

Skip following functions as they are not needed for Rust:

- `mju_isZero` -> `[f64]::iter()` + `.all(|&a| a == 0.0)`
- `mju_standardNormal` -> `rand` crate
- `mju_{f2n, n2f, d2n, n2d}` -> `bindgen` crate already handles conversions around `mjtNum`
- `mju_insertionSort`, `mju_insertionSortInt` -> `slice::sort*`
- `mju_strncpy` -> Rust has built-in string handling

*/

/// Generate Halton sequence.
/* mjtNum mju_Halton(int index, int base); */
pub fn mju_Halton(index: usize, base: usize) -> f64 {
    unsafe { crate::bindgen::mju_Halton(index as i32, base as i32) }
}

/// Twice continuously differentiable sigmoid function using a quintic polynomial:
/// 
/// _s(x) =_
/// 
/// - _0_ if _x <= 0_
/// - _1_ if _1 <= x_
/// - _6x^5 - 15x^4 + 10x^3_ otherwise
/// 
/* mjtNum mju_sigmoid(mjtNum x); */
pub fn mju_sigmoid(x: f64) -> f64 {
    unsafe { crate::bindgen::mju_sigmoid(x) }
}

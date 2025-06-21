/* https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#initialization */

//! This section contains functions that load/initialize the model or other data structures. Their use is well illustrated in the code samples.

use crate::{MjLrOpt, MjOption, MjVisual, MjModel, MjData, MjSpec};

/// Set default options for length range computation.
/* void mj_defaultLROpt(mjLROpt* opt); */
pub fn mj_defaultLROpt() -> MjLrOpt {
    let mut c = crate::bindgen::mjLROpt::default();
    unsafe { crate::bindgen::mj_defaultLROpt(&mut c) };
    MjLrOpt::from(c)
}

/// Set solver parameters to default values.
/* void mj_defaultSolRefImp(mjtNum* solref, mjtNum* solimp); */
pub fn mj_defaultSolRefImp() -> (
    [f64; crate::bindgen::mjNREF as usize],
    [f64; crate::bindgen::mjNIMP as usize],
) {
    let mut solref = [0.0; crate::bindgen::mjNREF as usize];
    let mut solimp = [0.0; crate::bindgen::mjNIMP as usize];
    unsafe { crate::bindgen::mj_defaultSolRefImp(solref.as_mut_ptr(), solimp.as_mut_ptr()) };
    (solref, solimp)
}

/// Set physics options to default values.
/* void mj_defaultOption(mjOption* opt); */
pub fn mj_defaultOption() -> MjOption {
    let mut c = crate::bindgen::mjOption::default();
    unsafe { crate::bindgen::mj_defaultOption(&mut c) };
    MjOption::from(c)
}

/// Set visual options to default values.
/* void mj_defaultVisual(mjVisual* vis); */
pub fn mj_defaultVisual() -> MjVisual {
    let mut c = crate::bindgen::mjVisual::default();
    unsafe { crate::bindgen::mj_defaultVisual(&mut c) };
    MjVisual::from(c)
}

/// Copy `MjModel`.
/// Unsafely overwrite `dest` if its `Some(&mut _)` with returning `None`,
/// otherwise allocate new and return it.
/* mjModel* mj_copyModel(mjModel* dest, const mjModel* src); */
pub fn mj_copyModel(dest: Option<&mut MjModel>, src: &MjModel) -> Option<MjModel> {
    match dest {
        Some(dest) => {
            unsafe { crate::bindgen::mj_copyModel(dest.as_mut(), src.as_ref()) };
            None
        }
        None => {
            let mut c = crate::bindgen::mjModel::default();
            unsafe { crate::bindgen::mj_copyModel(&mut c, src.as_ref()) };
            Some(MjModel::from(c))
        }
    }
}

/// Save model to binary MJB file or memory buffer; buffer has precedence when given.
/// 
/// The size of the buffer must be at least `mj_sizeModel(m)`.
/* void mj_saveModel(const mjModel* m, const char* filename, void* buffer, int buffer_sz); */
pub fn mj_saveModel(
    m: &MjModel,
    filename: Option<impl Into<String>>,
    buffer: Option<&mut [u8]>,
) {
    let filename = filename.map(|s| std::ffi::CString::new(s.into()).unwrap());
    unsafe {
        crate::bindgen::mj_saveModel(
            m.as_ref(),
            filename.map_or(std::ptr::null(), |cstr| cstr.as_ptr()),
            buffer.map_or(std::ptr::null_mut(), |b| b.as_mut_ptr()),
            buffer.map_or(0, |b| b.len() as i32),
        );
    }
}

//! # [Initialization](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#initialization)
//! 
//! This section contains functions that load/initialize the model or other data structures. Their use is well illustrated in the code samples.

use crate::{ObjectId, obj};
use crate::MjError;

/// Set default options for length range computation.
/// 
/// **note**: [`mjLROpt`] calls this function in its `Default` implementation.
/* void mj_defaultLROpt(mjLROpt* opt); */
pub fn mj_defaultLROpt() -> crate::mjLROpt {
    let mut c = std::mem::MaybeUninit::<crate::mjLROpt>::uninit();
    unsafe { crate::bindgen::mj_defaultLROpt(c.as_mut_ptr()) };
    unsafe { c.assume_init() }
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
/// 
/// **note**: [`mjOption`] calls this function in its `Default` implementation.
/* void mj_defaultOption(mjOption* opt); */
pub fn mj_defaultOption() -> crate::mjOption {
    let mut c = std::mem::MaybeUninit::<crate::mjOption>::uninit();
    unsafe { crate::bindgen::mj_defaultOption(c.as_mut_ptr()) };
    unsafe { c.assume_init() }
}

/// Set visual options to default values.
/// 
/// **note**: [`mjVisual`] calls this function in its `Default` implementation.
/* void mj_defaultVisual(mjVisual* vis); */
pub fn mj_defaultVisual() -> crate::mjVisual {
    let mut c = std::mem::MaybeUninit::<crate::mjVisual>::uninit();
    unsafe { crate::bindgen::mj_defaultVisual(c.as_mut_ptr()) };
    unsafe { c.assume_init() }
}

/// Copy `mjModel`.
/// Unsafely overwrite `dest` if its `Some(&mut _)` with returning `None`,
/// otherwise allocate new and return it.
/// 
/// ## SAFETY
/// 
/// When `dest` is `Some(&mut _)`, `src` and it **MUST** have the same sizes structure.
/// (e.g. same `nq`, `nv`, `nbody`, ...)
/// 
/* mjModel* mj_copyModel(mjModel* dest, const mjModel* src); */
pub unsafe fn mj_copyModel(dest: Option<&mut crate::MjModel>, src: &crate::MjModel) -> Option<crate::MjModel> {
    match dest {
        Some(dest) => {
            unsafe { crate::bindgen::mj_copyModel(dest.as_mut_ptr(), src.as_ptr()) };
            None
        }
        None => {
            let mut c = std::mem::MaybeUninit::<crate::bindgen::mjModel>::uninit();
            unsafe { crate::bindgen::mj_copyModel(c.as_mut_ptr(), src.as_ptr()) };
            Some(crate::MjModel::from_raw(c.as_mut_ptr()))
        }
    }
}

/// Save model to binary MJB file or memory buffer; buffer has precedence when given.
/// 
/// The size of the buffer must be at least `mj_sizeModel(m)`.
/* void mj_saveModel(const mjModel* m, const char* filename, void* buffer, int buffer_sz); */
pub fn mj_saveModel(
    m: &crate::MjModel,
    filename: Option<impl Into<String>>,
    buffer: Option<&mut [u8]>,
) {
    let filename = filename.map(|s| std::ffi::CString::new(s.into()).unwrap());
    let buffer_sz = buffer.as_ref().map_or(0, |b| b.len() as i32);
    unsafe {
        crate::bindgen::mj_saveModel(
            m.as_ptr(),
            filename.map_or(std::ptr::null(), |cstr| cstr.as_ptr()),
            buffer.map_or(std::ptr::null_mut(), |b| b.as_mut_ptr() as *mut std::ffi::c_void),
            buffer_sz,
        );
    }
}

/// Load model from binary MJB file.
/* mjModel* mj_loadModel(const char* filename, const mjVFS* vfs); */
pub fn mj_loadModel(
    filename: impl Into<String>,
) -> crate::MjModel {
    let filename = std::ffi::CString::new(filename.into()).unwrap();
    let c_ptr = unsafe {
        crate::bindgen::mj_loadModel(
            filename.as_ptr(),
            std::ptr::null(), // no vfs
        )
    };
    #[cfg(debug_assertions)] {
        assert!(!c_ptr.is_null(), "Failed to load model from file: {}", filename.to_string_lossy());
    }
    crate::MjModel::from_raw(c_ptr)
}

/// Free memory allocation in model.
/// 
/// **note**: [`mjModel`] calls this function in its `Drop` implementation.
/* void mj_deleteModel(mjModel* m); */
pub fn mj_deleteModel(m: &mut crate::MjModel) {
    unsafe { crate::bindgen::mj_deleteModel(m.as_mut_ptr()) };
}

/// Return size of buffer needed to hold model.
/* int mj_sizeModel(const mjModel* m); */
pub fn mj_sizeModel(m: &crate::MjModel) -> usize {
    unsafe { crate::bindgen::mj_sizeModel(m.as_ptr()) as usize }
}

/// Allocate mjData corresponding to given model. If the model buffer is unallocated
/// the initial configuration will not be set.
/* mjData* mj_makeData(const mjModel* m); */
pub fn mj_makeData(m: &crate::MjModel) -> crate::MjData {
    let c_ptr = unsafe { crate::bindgen::mj_makeData(m.as_ptr()) };
    #[cfg(debug_assertions)] {
        assert!(!c_ptr.is_null(), "Failed to allocate mjData for model");
    }
    crate::MjData::from_raw(c_ptr)
}

/// Copy mjData. `m` is only required to contain the size fields from MJMODEL_INTS.
/// 
/// This overwrites `dest` with `src`, without checking if their corresponding models
/// have the same sizes structure.
/// 
/// ## SAFETY
///
/// `dest` and `src` **MUST** have the same sizes structure.
/// (e.g. same `nq`, `nv`, `nbody`, ...)
/// 
/* mjData* mj_copyData(mjData* dest, const mjModel* m, const mjData* src); */
pub unsafe fn mj_copyData(dest: &mut crate::MjData, m: &crate::MjModel, src: &crate::MjData) {
    unsafe { crate::bindgen::mj_copyData(dest.as_mut_ptr(), m.as_ptr(), src.as_ptr()) };
}

/// Reset data to defaults.
/* void mj_resetData(const mjModel* m, mjData* d); */
pub fn mj_resetData(m: &crate::MjModel, d: &mut crate::MjData) {
    unsafe { crate::bindgen::mj_resetData(m.as_ptr(), d.as_mut_ptr()) }
}

/// Reset data to defaults, fill everything else with debug_value.
/* void mj_resetDataDebug(const mjModel* m, mjData* d, unsigned char debug_value); */
pub fn mj_resetDataDebug(m: &crate::MjModel, d: &mut crate::MjData, debug_value: u8) {
    unsafe { crate::bindgen::mj_resetDataDebug(m.as_ptr(), d.as_mut_ptr(), debug_value) }
}

/// Reset data. If `key` is given, set fields from specified keyframe.
/* void mj_resetDataKeyframe(const mjModel* m, mjData* d, int key); */
pub fn mj_resetDataKeyframe(
    m: &crate::MjModel,
    d: &mut crate::MjData,
    key: Option<ObjectId<obj::Key>>,
) {
    unsafe {
        crate::bindgen::mj_resetDataKeyframe(
            m.as_ptr(),
            d.as_mut_ptr(),
            key.map_or(-1, |k| k.index() as i32)
        )
    }
}

/// Mark a new frame on the mjData stack.
/* void mj_markStack(mjData* d); */
pub fn mj_markStack(d: &mut crate::MjData) {
    unsafe { crate::bindgen::mj_markStack(d.as_mut_ptr()) }
}

/// Free the current mjData stack frame. All pointers returned by `mj_stackAlloc` since the last call to `mj_markStack` must no longer be used afterwards.
/* void mj_freeStack(mjData* d); */
pub fn mj_freeStack(d: &mut crate::MjData) {
    unsafe { crate::bindgen::mj_freeStack(d.as_mut_ptr()) }
}

/// Allocate a number of bytes on mjData stack at a specific alignment. Call `mju_error` on stack overflow.
/* void* mj_stackAllocByte(mjData* d, size_t bytes, size_t alignment); */
pub fn mj_stackAllocByte(d: &mut crate::MjData, bytes: usize, alignment: usize) -> *mut u8 {
    unsafe { crate::bindgen::mj_stackAllocByte(d.as_mut_ptr(), bytes, alignment) as *mut u8 }
}

/// Allocate array of `mjtNums` on mjData stack. Call `mju_error` on stack overflow.
/* mjtNum* mj_stackAllocNum(mjData* d, size_t size); */
pub fn mj_stackAllocNum(d: &mut crate::MjData, size: usize) -> *mut f64 {
    unsafe { crate::bindgen::mj_stackAllocNum(d.as_mut_ptr(), size) as *mut f64 }
}

/// Allocate array of `ints` on mjData stack. Call `mju_error` on stack overflow.
/* int* mj_stackAllocInt(mjData* d, size_t size); */
pub fn mj_stackAllocInt(d: &mut crate::MjData, size: usize) -> *mut i32 {
    unsafe { crate::bindgen::mj_stackAllocInt(d.as_mut_ptr(), size) as *mut i32 }
}

/// Free memory allocation in mjData.
/// 
/// **note**: [`mjData`] calls this function in its `Drop` implementation.
/* void mj_deleteData(mjData* d); */
pub fn mj_deleteData(d: &mut crate::MjData) {
    unsafe { crate::bindgen::mj_deleteData(d.as_mut_ptr()) };
}

/// Reset all callbacks to NULL pointers (NULL is the default).
/* void mj_resetCallbacks(void); */
pub fn mj_resetCallbacks() {
    unsafe { crate::bindgen::mj_resetCallbacks() };
}

/// Set constant fields of mjModel, corresponding to `qpos0` configuration.
/* void mj_setConst(mjModel* m, mjData* d); */
pub fn mj_setConst(m: &mut crate::MjModel, d: &mut crate::MjData) {
    unsafe { crate::bindgen::mj_setConst(m.as_mut_ptr(), d.as_mut_ptr()) };
}

/// Set actuator length range for specified actuator.
/* int mj_setLengthRange(mjModel* m, mjData* d, int index,
                      const mjLROpt* opt, char* error, int error_sz); */
pub fn mj_setLengthRange(
    m: &mut crate::MjModel,
    d: &mut crate::MjData,
    index: ObjectId<obj::Actuator>,
    opt: &crate::mjLROpt,
) -> Result<(), MjError> {
    let mut error = MjError::init();
    let status = unsafe {
        let (err_ptr, err_len) = error.as_parts();
        crate::bindgen::mj_setLengthRange(
            m.as_mut_ptr(),
            d.as_mut_ptr(),
            index.index() as i32,
            opt,
            err_ptr,
            err_len,
        )
    };
    
    /*
    <https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-setlengthrange>
    > return 1 if ok, 0 if error.
    */
    if status == 1 {
        Ok(())
    } else {
        Err(error)
    }
}

/// Create empty spec.
/// 
/// **note**: [`mjSpec`] calls this function in its `Default` implementation.
/* mjSpec* mj_makeSpec(void); */
pub fn mj_makeSpec() -> crate::MjSpec {
    let c_ptr = unsafe { crate::bindgen::mj_makeSpec() };
    #[cfg(debug_assertions)] {
        assert!(!c_ptr.is_null(), "Failed to create empty mjSpec");
    }
    crate::MjSpec::from_raw(c_ptr)
}

/// Copy spec.
/// 
/// **note**: [`mjSpec`] calls this function in its `Clone` implementation.
/* mjSpec* mj_copySpec(const mjSpec* s); */
pub fn mj_copySpec(s: &crate::MjSpec) -> crate::MjSpec {
    let c_ptr = unsafe { crate::bindgen::mj_copySpec(s.as_ptr()) };
    #[cfg(debug_assertions)] {
        assert!(!c_ptr.is_null(), "Failed to copy mjSpec");
    }
    crate::MjSpec::from_raw(c_ptr)
}

/// Free memory allocation in mjSpec.
/// 
/// **note**: [`mjSpec`] calls this function in its `Drop` implementation.
/* void mj_deleteSpec(mjSpec* s); */
pub fn mj_deleteSpec(s: &mut crate::MjSpec) {
    unsafe { crate::bindgen::mj_deleteSpec(s.as_mut_ptr()) };
}

/// Activate plugin.
/* int mjs_activatePlugin(mjSpec* s, const char* name); */
pub fn mjs_activatePlugin(s: &mut crate::MjSpec, name: impl Into<String>) -> Result<(), MjError> {
    let name = std::ffi::CString::new(name.into()).unwrap();
    let status = unsafe { crate::bindgen::mjs_activatePlugin(s.as_mut_ptr(), name.as_ptr()) };
    
    /*
    <https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mjs-activateplugin>
    > Returns 0 on success.
    */
    if status == 0 {
        Ok(())
    } else {
        Err(MjError::from_error("Failed to activate plugin"))
    }
}

/// Turn deep copy on or off attach.
/* int mjs_setDeepCopy(mjSpec* s, int deepcopy); */
pub fn mjs_setDeepCopy(s: &mut crate::MjSpec, deepcopy: bool) -> Result<(), MjError> {
    let status = unsafe { crate::bindgen::mjs_setDeepCopy(s.as_mut_ptr(), deepcopy as i32) };
    
    /*
    <https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mjs-setdeepcopy>
    > Returns 0 on success.
    */
    if status == 0 {
        Ok(())
    } else {
        Err(MjError::from_error("Failed to set deep copy option"))
    }
}

const BUFFER_SIZE: usize = 1 << 10;

pub struct MjError {
    buffer: [u8; BUFFER_SIZE],
}

impl std::fmt::Debug for MjError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let msg = String::from_utf8_lossy(&self.buffer);
        write!(f, "MjError({msg})")
    }
}
impl std::fmt::Display for MjError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let msg = String::from_utf8_lossy(&self.buffer);
        write!(f, "MjError({msg})")
    }
}
impl std::error::Error for MjError {}

impl MjError {
    pub(crate) fn init() -> Self {
        Self {
            buffer: [0u8; BUFFER_SIZE],
        }
    }

    pub(crate) fn as_parts(&mut self) -> (*mut std::ffi::c_char, i32) {
        let ptr = self.buffer.as_mut_ptr();
        let len = self.buffer.len();
        (ptr as *mut std::ffi::c_char, len as i32)
    }
}

impl MjError {
    pub(crate) fn from_error(error: impl std::fmt::Display) -> Self {
        let error = error.to_string();
        let error = error.as_bytes();
        let mut mj_error = Self::init();
        if error.len() < BUFFER_SIZE {
            mj_error.buffer[..error.len()].copy_from_slice(error);
            mj_error.buffer[error.len()] = b'\0'; // null-terminate
        } else {
            mj_error.buffer[..(BUFFER_SIZE - 1)].copy_from_slice(&error[..(BUFFER_SIZE - 1)]);
            mj_error.buffer[BUFFER_SIZE - 1] = b'\0'; // null-terminate
        }
        mj_error
    }
}

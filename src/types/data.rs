/// This is the main data structure holding the simulation state. It is the workspace where all functions read their modifiable inputs and write their outputs.
pub struct MjData(pub(super) crate::bindgen::mjData);

// constant sizes
impl MjData {
    /// size of the arena in bytes (inclusive of the stack)
    pub fn narena(&self) -> usize {
        self.0.narena
    }
    /// size of main buffer in bytes
    pub fn nbuffer(&self) -> usize {
        self.0.nbuffer
    }
    /// number of plugin instances
    pub fn nplugin(&self) -> usize {
        self.0.nplugin as usize
    }
}

// stack pointer
impl MjData {
    /// first available byte in stack
    pub fn pstack(&self) -> usize {
        self.0.pstack
    }
    /// value of pstack when mj_markStack was last called
    pub fn pbase(&self) -> usize {
        self.0.pbase
    }
}

// arena pointer
impl MjData {
    /// first byte in arena
    pub fn parena(&self) -> usize {
        self.0.parena
    }
}

// memory utilization statistics
impl MjData {
    /// maximum stack allocation in bytes
    pub fn maxuse_stack(&self) -> usize {
      self.0.maxuse_stack
    }
    /// maximum arena allocation in bytes
    pub fn maxuse_arena(&self) -> usize {
      self.0.maxuse_arena
    }
    /// maximum stack allocation per thread in bytes
    pub fn maxuse_threadstack(&self) -> [usize; crate::bindgen::mjMAXTHREAD as usize] {
        self.0.maxuse_threadstack
    }
    /// maximum number of contacts
    pub fn maxuse_con(&self) -> usize {
        self.0.maxuse_con as usize
    }
    /// maximum number of scalar constraints
    pub fn maxuse_efc(&self) -> usize {
        self.0.maxuse_efc as usize
    }
}

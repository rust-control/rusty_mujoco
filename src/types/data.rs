use super::*;
use crate::bindgen::{mjNSOLVER, mjNISLAND, mjtTimer::mjNTIMER, mjtWarning::mjNWARNING};

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

// solver statistics
/*
  mjSolverStat  solver[mjNISLAND*mjNSOLVER];  // solver statistics per island, per iteration
  int           solver_niter[mjNISLAND];      // number of solver iterations, per island
  int           solver_nnz[mjNISLAND];        // number of nonzeros in Hessian or efc_AR, per island
  mjtNum        solver_fwdinv[2];             // forward-inverse comparison: qfrc, efc
*/
impl MjData {
    /// solver statistics per island, per iteration
    pub fn solver(&self) -> [MjSolverStat; mjNISLAND as usize * mjNSOLVER as usize] {
        self.0.solver.map(MjSolverStat)
    }
    /// number of solver iterations, per island
    pub fn solver_niter(&self) -> [usize; mjNISLAND as usize] {
        self.0.solver_niter.map(|x| x as usize)
    }
    /// number of nonzeros in Hessian or efc_AR, per island
    pub fn solver_nnz(&self) -> [usize; mjNISLAND as usize] {
        self.0.solver_nnz.map(|x| x as usize)
    }
    /// forward-inverse comparison: qfrc, efc
    pub fn solver_fwdinv(&self) -> [f64; 2] {
        self.0.solver_fwdinv
    }
}


// diagnostics
/*
  mjWarningStat warning[mjNWARNING];          // warning statistics
  mjTimerStat   timer[mjNTIMER];              // timer statistics
*/
impl MjData {
    /// warning statistics
    pub fn warning(&self) -> [MjWarningStat; mjNWARNING as usize] {
        self.0.warning.map(MjWarningStat)
    }
    /// timer statistics
    pub fn timer(&self) -> [MjTimerStat; mjNTIMER as usize] {
        self.0.timer.map(MjTimerStat)
    }
}

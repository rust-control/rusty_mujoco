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

// variable sizes
impl MjData {
    /// number of detected contacts
    pub fn ncon(&self) -> usize {self.0.ncon as usize}
    /// number of equality constraints
    pub fn ne(&self) -> usize {self.0.ne as usize}
    /// number of friction constraints
    pub fn nf(&self) -> usize {self.0.nf as usize}
    /// number of limit constraints
    pub fn nl(&self) -> usize {self.0.nl as usize}
    /// number of constraints
    pub fn nefc(&self) -> usize {self.0.nefc as usize}
    /// number of non-zeros in constraint Jacobian
    #[allow(non_snake_case)]
    pub fn nJ(&self) -> usize {self.0.nJ as usize}
    /// number of non-zeros in constraint inverse inertia matrix
    #[allow(non_snake_case)]
    pub fn nA(&self) -> usize {self.0.nA as usize}
    /// number of detected constraint islands
    pub fn nisland(&self) -> usize {self.0.nisland as usize}
}

// global properties
/*
  mjtNum  time;              // simulation time
  mjtNum  energy[2];         // potential, kinetic energy
*/
impl MjData {
    /// simulation time
    pub fn time(&self) -> f64 {
        self.0.time
    }
    /// potential, kinetic energy
    pub fn energy(&self) -> [f64; 2] {
        self.0.energy
    }
}

macro_rules! impl_buffer_slices {
    ($($name:ident: [$T:ty; $size_note:literal] = $description:literal;)*) => {
        impl MjData {
            $(
                #[allow(non_snake_case)]
                #[doc = $description]
                #[doc = "\n"]
                #[doc = "SAFETY: `len` must not exceed `"]
                #[doc = $size_note]
                #[doc = "` of corresponded MjModel."]
                pub unsafe fn $name(&self, len: usize) -> &[$T] {
                    unsafe { std::slice::from_raw_parts(self.0.$name, len) }
                }
            )*
        }
    };
}
impl_buffer_slices! {
  // state
  qpos: [f64; "nq * 1"]              = "position";
  qvel: [f64; "nv * 1"]              = "velocity";
  act: [f64; "na * 1"]               = "actuator activation";
  qacc_warmstart: [f64; "nv * 1"]    = "acceleration used for warmstart";
  plugin_state: [f64; "npluginstate * 1"]      = "plugin state";

  // control
  ctrl: [f64; "nu * 1"]              = "control";
  qfrc_applied: [f64; "nv * 1"]      = "applied generalized force";
  xfrc_applied: [f64; "nbody * 6"]      = "applied Cartesian force/torque";
  eq_active: [u8; "neq * 1"]        = "enable/disable constraints";

  // mocap data
  mocap_pos: [f64; "nmocap * 3"]         = "positions of mocap bodies";
  mocap_quat: [f64; "nmocap * 4"]        = "orientations of mocap bodies";

  // dynamics
  qacc: [f64; "nv * 1"]              = "acceleration";
  act_dot: [f64; "na * 1"]           = "time-derivative of actuator activation";

  // user data
  userdata: [f64; "nuserdata * 1"]          = "user data, not touched by engine";

  // sensors
  sensordata: [f64; "nsensordata * 1"]        = "sensor data array";

  // plugins
  plugin: [i32; "nplugin * 1"]         = "copy of m->plugin, required for deletion";
  /* handle specially in later...
    uintptr_t* plugin_data;    // pointer to plugin-managed data structure         (nplugin * 1)
  */

  //-------------------- POSITION dependent

  // computed by mj_fwdPosition/mj_kinematics
  xpos: [f64; "nbody * 3"]              = "Cartesian position of body frame";
  xquat: [f64; "nbody * 4"]             = "Cartesian orientation of body frame";
  xmat: [f64; "nbody * 9"]              = "Cartesian orientation of body frame";
  xipos: [f64; "nbody * 3"]             = "Cartesian position of body com";
  ximat: [f64; "nbody * 9"]             = "Cartesian orientation of body inertia";
  xanchor: [f64; "njnt * 3"]           = "Cartesian position of joint anchor";
  xaxis: [f64; "njnt * 3"]             = "Cartesian joint axis";
  geom_xpos: [f64; "ngeom * 3"]         = "Cartesian geom position";
  geom_xmat: [f64; "ngeom * 9"]         = "Cartesian geom orientation";
  site_xpos: [f64; "nsite * 3"]         = "Cartesian site position";
  site_xmat: [f64; "nsite * 9"]         = "Cartesian site orientation";
  cam_xpos: [f64; "ncam * 3"]          = "Cartesian camera position";
  cam_xmat: [f64; "ncam * 9"]          = "Cartesian camera orientation";
  light_xpos: [f64; "nlight * 3"]        = "Cartesian light position";
  light_xdir: [f64; "nlight * 3"]        = "Cartesian light direction";

  // computed by mj_fwdPosition/mj_comPos
  subtree_com: [f64; "nbody * 3"]       = "center of mass of each subtree";
  cdof: [f64; "nv * 6"]              = "com-based motion axis of each dof (rot:lin)";
  cinert: [f64; "nbody * 10"]            = "com-based body inertia and mass";

  // computed by mj_fwdPosition/mj_flex
  flexvert_xpos: [f64; "nflexvert * 3"]     = "Cartesian flex vertex positions";
  flexelem_aabb: [f64; "nflexelem * 6"]     = "flex element bounding boxes (center, size)";
  flexedge_J_rownnz: [i32; "nflexedge * 1"] = "number of non-zeros in Jacobian row";
  flexedge_J_rowadr: [i32; "nflexedge * 1"] = "row start address in colind array";
  flexedge_J_colind: [i32; "nflexedge x nv"] = "column indices in sparse Jacobian";
  flexedge_J: [f64; "nflexedge x nv"]        = "flex edge Jacobian";
  flexedge_length: [f64; "nflexedge * 1"]   = "flex edge lengths";

  // computed by mj_fwdPosition/mj_tendon
  ten_wrapadr: [i32; "ntendon * 1"]       = "start address of tendon's path";
  ten_wrapnum: [i32; "ntendon * 1"]       = "number of wrap points in path";
  ten_J_rownnz: [i32; "ntendon * 1"]      = "number of non-zeros in Jacobian row";
  ten_J_rowadr: [i32; "ntendon * 1"]      = "row start address in colind array";
  ten_J_colind: [i32; "ntendon x nv"]      = "column indices in sparse Jacobian";
  ten_J: [f64; "ntendon x nv"]             = "tendon Jacobian";
  ten_length: [f64; "ntendon * 1"]        = "tendon lengths";
  wrap_obj: [i32; "nwrap x 2"]          = "geom id; -1: site; -2: pulley";
  wrap_xpos: [f64; "nwrap * 6"]         = "Cartesian 3D points in all paths";

  // computed by mj_fwdPosition/mj_transmission
  actuator_length: [f64; "nu * 1"]   = "actuator lengths";
  moment_rownnz: [i32; "nu * 1"]     = "number of non-zeros in actuator_moment row";
  moment_rowadr: [i32; "nu * 1"]     = "row start address in colind array";
  moment_colind: [i32; "nJmom * 1"]     = "column indices in sparse Jacobian";
  actuator_moment: [f64; "nJmom * 1"]   = "actuator moments";

  // computed by mj_fwdPosition/mj_makeM
  crb: [f64; "nbody * 10"]               = "com-based composite inertia and mass";
  qM: [f64; "nM * 1"]                = "inertia (sparse)";

  // computed by mj_fwdPosition/mj_factorM
  qLD: [f64; "nC * 1"]               = "L'*D*L factorization of M (sparse)";
  qLDiagInv: [f64; "nv * 1"]         = "1/diag(D)";

  // computed by mj_collisionTree
   bvh_aabb_dyn: [f64; "nbvhdynamic * 6"]     = "global bounding box (center, size)";
  bvh_active: [u8; "nbvh * 1"]       = "was bounding volume checked for collision";

  //-------------------- POSITION, VELOCITY dependent

  // computed by mj_fwdVelocity
  flexedge_velocity: [f64; "nflexedge * 1"] = "flex edge velocities";
  ten_velocity: [f64; "ntendon * 1"]      = "tendon velocities";
  actuator_velocity: [f64; "nu * 1"] = "actuator velocities";

  // computed by mj_fwdVelocity/mj_comVel
  cvel: [f64; "nbody * 6"]              = "com-based velocity (rot:lin)";
  cdof_dot: [f64; "nv * 6"]          = "time-derivative of cdof (rot:lin)";

  // computed by mj_fwdVelocity/mj_rne (without acceleration)
  qfrc_bias: [f64; "nv * 1"]         = "C(qpos,qvel)";

  // computed by mj_fwdVelocity/mj_passive
  qfrc_spring: [f64; "nv * 1"]       = "passive spring force";
  qfrc_damper: [f64; "nv * 1"]       = "passive damper force";
  qfrc_gravcomp: [f64; "nv * 1"]     = "passive gravity compensation force";
  qfrc_fluid: [f64; "nv * 1"]        = "passive fluid force";
  qfrc_passive: [f64; "nv * 1"]      = "total passive force";

  // computed by mj_sensorVel/mj_subtreeVel if needed
  subtree_linvel: [f64; "nbody * 3"]    = "linear velocity of subtree com";
  subtree_angmom: [f64; "nbody * 3"]    = "angular momentum about subtree com";

  // computed by mj_Euler or mj_implicit
  qH: [f64; "nC * 1"]                = "L'*D*L factorization of modified M";
  qHDiagInv: [f64; "nv * 1"]         = "1/diag(D) of modified M";

  // computed by mj_resetData
  B_rownnz: [i32; "nbody * 1"]          = "body-dof: non-zeros in each row";
  B_rowadr: [i32; "nbody * 1"]          = "body-dof: address of each row in B_colind";
  B_colind: [i32; "nB * 1"]          = "body-dof: column indices of non-zeros";
  M_rownnz: [i32; "nv * 1"]          = "reduced inertia: non-zeros in each row";
  M_rowadr: [i32; "nv * 1"]          = "reduced inertia: address of each row in ";
  M_colind: [i32; "nC * 1"]          = "reduced inertia: column indices of non-zeros";
  mapM2M: [i32; "nC * 1"]            = "index mapping from qM to M";
  D_rownnz: [i32; "nv * 1"]          = "full inertia: non-zeros in each row";
  D_rowadr: [i32; "nv * 1"]          = "full inertia: address of each row in D_colind";
  D_diag: [i32; "nv * 1"]            = "full inertia: index of diagonal element";
  D_colind: [i32; "nD * 1"]          = "full inertia: column indices of non-zeros";
  mapM2D: [i32; "nD * 1"]            = "index mapping from qM to D";
  mapD2M: [i32; "nM * 1"]            = "index mapping from D to qM";

  // computed by mj_implicit/mj_derivative
  qDeriv: [f64; "nD * 1"]            = "d (passive + actuator - bias) / d qvel";

  // computed by mj_implicit/mju_factorLUSparse
  qLU: [f64; "nD * 1"]               = "sparse LU of (qM - dt*qDeriv)";

  //-------------------- POSITION, VELOCITY, CONTROL/ACCELERATION dependent

  // computed by mj_fwdActuation
  actuator_force: [f64; "nu * 1"]    = "actuator force in actuation space";
  qfrc_actuator: [f64; "nv * 1"]     = "actuator force";

  // computed by mj_fwdAcceleration
  qfrc_smooth: [f64; "nv * 1"]       = "net unconstrained force";
  qacc_smooth: [f64; "nv * 1"]       = "unconstrained acceleration";

  // computed by mj_fwdConstraint/mj_inverse
  qfrc_constraint: [f64; "nv * 1"]   = "constraint force";

  // computed by mj_inverse
  qfrc_inverse: [f64; "; should equal"]      = "net ";
                             // qfrc_applied + J'*xfrc_applied + qfrc_actuator   (nv * 1)

  // computed by mj_sensorAcc/mj_rnePostConstraint if needed; rotation:translation format
  cacc: [f64; "nbody * 6"]              = "com-based acceleration";
  cfrc_int: [f64; "nbody * 6"]          = "com-based interaction force with parent";
  cfrc_ext: [f64; "nbody * 6"]          = "com-based external force on body";

  //-------------------- arena-allocated: POSITION dependent

  /* handle specially in later...
    // computed by mj_collision
    mjContact* contact;        // array of all detected contacts                   (ncon * 1)
  */

  // computed by mj_makeConstraint
  efc_type: [i32; "nefc * 1"]          = "constraint type (mjtConstraint)";
  efc_id: [i32; "nefc * 1"]            = "id of object of specified type";
  efc_J_rownnz: [i32; "nefc * 1"]      = "number of non-zeros in constraint Jacobian row";
  efc_J_rowadr: [i32; "nefc * 1"]      = "row start address in colind array";
  efc_J_rowsuper: [i32; "nefc * 1"]    = "number of subsequent rows in supernode";
  efc_J_colind: [i32; "nJ * 1"]      = "column indices in constraint Jacobian";
  efc_JT_rownnz: [i32; "nv * 1"]     = "number of non-zeros in constraint Jacobian row ";
  efc_JT_rowadr: [i32; "nv * 1"]     = "row start address in colind array              ";
  efc_JT_rowsuper: [i32; "nv * 1"]   = "number of subsequent rows in supernode         ";
  efc_JT_colind: [i32; "nJ * 1"]     = "column indices in constraint Jacobian          ";
  efc_J: [f64; "nJ * 1"]             = "constraint Jacobian";
  efc_JT: [f64; "nJ * 1"]            = "constraint Jacobian transposed";
  efc_pos: [f64; "nefc * 1"]           = "constraint position (equality, contact)";
  efc_margin: [f64; "nefc * 1"]        = "inclusion margin (contact)";
  efc_frictionloss: [f64; "nefc * 1"]  = "frictionloss (friction)";
  efc_diagApprox: [f64; "nefc * 1"]    = "approximation to diagonal of A";
  efc_KBIP: [f64; "nefc * 4"]          = "stiffness, damping, impedance, imp'";
  efc_D: [f64; "nefc * 1"]             = "constraint mass";
  efc_R: [f64; "nefc * 1"]             = "inverse constraint mass";
  tendon_efcadr: [i32; "ntendon * 1"]     = "first efc address involving tendon; -1: none";

  // computed by mj_island (island dof structure)
  dof_island: [i32; "nv * 1"]        = "island id of this dof; -1: none";
  island_dofadr: [i32; "nisland * 1"]     = "island start address in dof vector";

  // computed by mj_island (island constraint structure)
  efc_island: [i32; "nefc * 1"]        = "island id of this constraint";

  // computed by mj_projectConstraint (PGS solver)
  efc_AR_rownnz: [i32; "nefc * 1"]     = "number of non-zeros in AR";
  efc_AR_rowadr: [i32; "nefc * 1"]     = "row start address in colind array";
  efc_AR_colind: [i32; "nA * 1"]     = "column indices in sparse AR";
  efc_AR: [f64; "nA * 1"]            = "J*inv(M)*J' + R";

  //-------------------- arena-allocated: POSITION, VELOCITY dependent

  // computed by mj_fwdVelocity/mj_referenceConstraint
  efc_vel: [f64; "nefc * 1"]           = "velocity in constraint space: J*qvel";
  efc_aref: [f64; "nefc * 1"]          = "reference pseudo-acceleration";

  //-------------------- arena-allocated: POSITION, VELOCITY, CONTROL/ACCELERATION dependent

  // computed by mj_fwdConstraint/mj_inverse
  efc_b: [f64; "nefc * 1"]             = "linear cost term: J*qacc_smooth - aref";
  efc_state: [i32; "nefc * 1"]         = "constraint state (mjtConstraintState)";
  efc_force: [f64; "nefc * 1"]         = "constraint force in constraint space";
}

impl MjData {
    /// pointer to plugin-managed data structure
    /// 
    /// SAFETY: `len` must not exceed `nplugin` of corresponded MjModel.
    pub unsafe fn plugin_data(&self, len: usize) -> &[usize] {
        unsafe { std::slice::from_raw_parts(self.0.plugin_data, len) }
    }

    /// array of all detected contacts
    /// 
    /// SAFETY: `len` must not exceed `ncon` of corresponded MjModel.
    pub unsafe fn contact(&self, len: usize) -> Vec<MjContact> {
        (unsafe {std::slice::from_raw_parts(self.0.contact, len)})
            .iter()
            .copied()
            .map(MjContact)
            .collect()
    }
}

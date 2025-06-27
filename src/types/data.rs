use super::*;
use crate::bindgen::{mjNSOLVER, mjNISLAND, mjNTIMER, mjNWARNING};

wrapper! {
    /// This is the main data structure holding the simulation state. It is the workspace where all functions read their modifiable inputs and write their outputs.
    MjData of crate::bindgen::mjData
}

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
    /// set simulation time
    pub fn set_time(&mut self, time: f64) {
        self.0.time = time;
    }

    /// potential, kinetic energy
    pub fn energy(&self) -> [f64; 2] {
        self.0.energy
    }
}

macro_rules! impl_buffer_slices {
    ($($name:ident / $mut_name:ident: [$T:ty; $size_note:literal] = $description:literal;)*) => {
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

                #[allow(non_snake_case)]
                #[doc = "mutable "]
                #[doc = $description]
                #[doc = "\n"]
                #[doc = "SAFETY: `len` must not exceed `"]
                #[doc = $size_note]
                #[doc = "` of corresponded MjModel."]
                pub unsafe fn $mut_name(&mut self, len: usize) -> &mut [$T] {
                    unsafe { std::slice::from_raw_parts_mut(self.0.$name, len) }
                }
            )*
        }
    };
}
impl_buffer_slices! {
  // state
  qpos / qpos_mut: [f64; "nq * 1"]              = "position";
  qvel / qvel_mut: [f64; "nv * 1"]              = "velocity";
  act / act_mut: [f64; "na * 1"]               = "actuator activation";
  qacc_warmstart / qacc_warmstart_mut: [f64; "nv * 1"]    = "acceleration used for warmstart";
  plugin_state / plugin_state_mut: [f64; "npluginstate * 1"]      = "plugin state";

  // control
  ctrl / ctrl_mut: [f64; "nu * 1"]              = "control";
  qfrc_applied / qfrc_applied_mut: [f64; "nv * 1"]      = "applied generalized force";
  xfrc_applied / xfrc_applied_mut: [f64; "nbody * 6"]      = "applied Cartesian force/torque";
  eq_active / eq_active_mut: [u8; "neq * 1"]        = "enable/disable constraints";

  // mocap data
  mocap_pos / mocap_pos_mut: [f64; "nmocap * 3"]         = "positions of mocap bodies";
  mocap_quat / mocap_quat_mut: [f64; "nmocap * 4"]        = "orientations of mocap bodies";

  // dynamics
  qacc / qacc_mut: [f64; "nv * 1"]              = "acceleration";
  act_dot / act_dot_mut: [f64; "na * 1"]           = "time-derivative of actuator activation";

  // user data
  userdata / userdata_mut: [f64; "nuserdata * 1"]          = "user data, not touched by engine";

  // sensors
  sensordata / sensordata_mut: [f64; "nsensordata * 1"]        = "sensor data array";

  // plugins
  plugin / plugin_mut: [i32; "nplugin * 1"]         = "copy of m->plugin, required for deletion";
  /* handle specially in later...
    uintptr_t* plugin_data;    // pointer to plugin-managed data structure         (nplugin * 1)
  */

  //-------------------- POSITION dependent

  // computed by mj_fwdPosition/mj_kinematics
  xpos / xpos_mut: [f64; "nbody * 3"]              = "Cartesian position of body frame";
  xquat / xquat_mut: [f64; "nbody * 4"]             = "Cartesian orientation of body frame";
  xmat / xmat_mut: [f64; "nbody * 9"]              = "Cartesian orientation of body frame";
  xipos / xipos_mut: [f64; "nbody * 3"]             = "Cartesian position of body com";
  ximat / ximat_mut: [f64; "nbody * 9"]             = "Cartesian orientation of body inertia";
  xanchor / xanchor_mut: [f64; "njnt * 3"]           = "Cartesian position of joint anchor";
  xaxis / xaxis_mut: [f64; "njnt * 3"]             = "Cartesian joint axis";
  geom_xpos / geom_xpos_mut: [f64; "ngeom * 3"]         = "Cartesian geom position";
  geom_xmat / geom_xmat_mut: [f64; "ngeom * 9"]         = "Cartesian geom orientation";
  site_xpos / site_xpos_mut: [f64; "nsite * 3"]         = "Cartesian site position";
  site_xmat / site_xmat_mut: [f64; "nsite * 9"]         = "Cartesian site orientation";
  cam_xpos / cam_xpos_mut: [f64; "ncam * 3"]          = "Cartesian camera position";
  cam_xmat / cam_xmat_mut: [f64; "ncam * 9"]          = "Cartesian camera orientation";
  light_xpos / light_xpos_mut: [f64; "nlight * 3"]        = "Cartesian light position";
  light_xdir / light_xdir_mut: [f64; "nlight * 3"]        = "Cartesian light direction";

  // computed by mj_fwdPosition/mj_comPos
  subtree_com / subtree_com_mut: [f64; "nbody * 3"]       = "center of mass of each subtree";
  cdof / cdof_mut: [f64; "nv * 6"]              = "com-based motion axis of each dof (rot:lin)";
  cinert / cinert_mut: [f64; "nbody * 10"]            = "com-based body inertia and mass";

  // computed by mj_fwdPosition/mj_flex
  flexvert_xpos / flexvert_xpos_mut: [f64; "nflexvert * 3"]     = "Cartesian flex vertex positions";
  flexelem_aabb / flexelem_aabb_mut: [f64; "nflexelem * 6"]     = "flex element bounding boxes (center, size)";
  flexedge_J_rownnz / flexedge_J_rownnz_mut: [i32; "nflexedge * 1"] = "number of non-zeros in Jacobian row";
  flexedge_J_rowadr / flexedge_J_rowadr_mut: [i32; "nflexedge * 1"] = "row start address in colind array";
  flexedge_J_colind / flexedge_J_colind_mut: [i32; "nflexedge x nv"] = "column indices in sparse Jacobian";
  flexedge_J / flexedge_J_mut: [f64; "nflexedge x nv"]        = "flex edge Jacobian";
  flexedge_length / flexedge_length_mut: [f64; "nflexedge * 1"]   = "flex edge lengths";

  // computed by mj_fwdPosition/mj_tendon
  ten_wrapadr / ten_wrapadr_mut: [i32; "ntendon * 1"]       = "start address of tendon's path";
  ten_wrapnum / ten_wrapnum_mut: [i32; "ntendon * 1"]       = "number of wrap points in path";
  ten_J_rownnz / ten_J_rownnz_mut: [i32; "ntendon * 1"]      = "number of non-zeros in Jacobian row";
  ten_J_rowadr / ten_J_rowadr_mut: [i32; "ntendon * 1"]      = "row start address in colind array";
  ten_J_colind / ten_J_colind_mut: [i32; "ntendon x nv"]      = "column indices in sparse Jacobian";
  ten_J / ten_J_mut: [f64; "ntendon x nv"]             = "tendon Jacobian";
  ten_length / ten_length_mut: [f64; "ntendon * 1"]        = "tendon lengths";
  wrap_obj / wrap_obj_mut: [i32; "nwrap x 2"]          = "geom id; -1: site; -2: pulley";
  wrap_xpos / wrap_xpos_mut: [f64; "nwrap * 6"]         = "Cartesian 3D points in all paths";

  // computed by mj_fwdPosition/mj_transmission
  actuator_length / actuator_length_mut: [f64; "nu * 1"]   = "actuator lengths";
  moment_rownnz / moment_rownnz_mut: [i32; "nu * 1"]     = "number of non-zeros in actuator_moment row";
  moment_rowadr / moment_rowadr_mut: [i32; "nu * 1"]     = "row start address in colind array";
  moment_colind / moment_colind_mut: [i32; "nJmom * 1"]     = "column indices in sparse Jacobian";
  actuator_moment / actuator_moment_mut: [f64; "nJmom * 1"]   = "actuator moments";

  // computed by mj_fwdPosition/mj_makeM
  crb / crb_mut: [f64; "nbody * 10"]               = "com-based composite inertia and mass";
  qM / qM_mut: [f64; "nM * 1"]                = "inertia (sparse)";

  // computed by mj_fwdPosition/mj_factorM
  qLD / qLD_mut: [f64; "nC * 1"]               = "L'*D*L factorization of M (sparse)";
  qLDiagInv / qLDiagInv_mut: [f64; "nv * 1"]         = "1/diag(D)";

  // computed by mj_collisionTree
   bvh_aabb_dyn / bvh_aabb_dyn_mut: [f64; "nbvhdynamic * 6"]     = "global bounding box (center, size)";
  bvh_active / bvh_active_mut: [u8; "nbvh * 1"]       = "was bounding volume checked for collision";

  //-------------------- POSITION, VELOCITY dependent

  // computed by mj_fwdVelocity
  flexedge_velocity / flexedge_velocity_mut: [f64; "nflexedge * 1"] = "flex edge velocities";
  ten_velocity / ten_velocity_mut: [f64; "ntendon * 1"]      = "tendon velocities";
  actuator_velocity / actuator_velocity_mut: [f64; "nu * 1"] = "actuator velocities";

  // computed by mj_fwdVelocity/mj_comVel
  cvel / cvel_mut: [f64; "nbody * 6"]              = "com-based velocity (rot:lin)";
  cdof_dot / cdof_dot_mut: [f64; "nv * 6"]          = "time-derivative of cdof (rot:lin)";

  // computed by mj_fwdVelocity/mj_rne (without acceleration)
  qfrc_bias / qfrc_bias_mut: [f64; "nv * 1"]         = "C(qpos,qvel)";

  // computed by mj_fwdVelocity/mj_passive
  qfrc_spring / qfrc_spring_mut: [f64; "nv * 1"]       = "passive spring force";
  qfrc_damper / qfrc_damper_mut: [f64; "nv * 1"]       = "passive damper force";
  qfrc_gravcomp / qfrc_gravcomp_mut: [f64; "nv * 1"]     = "passive gravity compensation force";
  qfrc_fluid / qfrc_fluid_mut: [f64; "nv * 1"]        = "passive fluid force";
  qfrc_passive / qfrc_passive_mut: [f64; "nv * 1"]      = "total passive force";

  // computed by mj_sensorVel/mj_subtreeVel if needed
  subtree_linvel / subtree_linvel_mut: [f64; "nbody * 3"]    = "linear velocity of subtree com";
  subtree_angmom / subtree_angmom_mut: [f64; "nbody * 3"]    = "angular momentum about subtree com";

  // computed by mj_Euler or mj_implicit
  qH / qH_mut: [f64; "nC * 1"]                = "L'*D*L factorization of modified M";
  qHDiagInv / qHDiagInv_mut: [f64; "nv * 1"]         = "1/diag(D) of modified M";

  // computed by mj_resetData
  B_rownnz / B_rownnz_mut: [i32; "nbody * 1"]          = "body-dof: non-zeros in each row";
  B_rowadr / B_rowadr_mut: [i32; "nbody * 1"]          = "body-dof: address of each row in B_colind";
  B_colind / B_colind_mut: [i32; "nB * 1"]          = "body-dof: column indices of non-zeros";
  M_rownnz / M_rownnz_mut: [i32; "nv * 1"]          = "reduced inertia: non-zeros in each row";
  M_rowadr / M_rowadr_mut: [i32; "nv * 1"]          = "reduced inertia: address of each row in ";
  M_colind / M_colind_mut: [i32; "nC * 1"]          = "reduced inertia: column indices of non-zeros";
  mapM2M / mapM2M_mut: [i32; "nC * 1"]            = "index mapping from qM to M";
  D_rownnz / D_rownnz_mut: [i32; "nv * 1"]          = "full inertia: non-zeros in each row";
  D_rowadr / D_rowadr_mut: [i32; "nv * 1"]          = "full inertia: address of each row in D_colind";
  D_diag / D_diag_mut: [i32; "nv * 1"]            = "full inertia: index of diagonal element";
  D_colind / D_colind_mut: [i32; "nD * 1"]          = "full inertia: column indices of non-zeros";
  mapM2D / mapM2D_mut: [i32; "nD * 1"]            = "index mapping from qM to D";
  mapD2M / mapD2M_mut: [i32; "nM * 1"]            = "index mapping from D to qM";

  // computed by mj_implicit/mj_derivative
  qDeriv / qDeriv_mut: [f64; "nD * 1"]            = "d (passive + actuator - bias) / d qvel";

  // computed by mj_implicit/mju_factorLUSparse
  qLU / qLU_mut: [f64; "nD * 1"]               = "sparse LU of (qM - dt*qDeriv)";

  //-------------------- POSITION, VELOCITY, CONTROL/ACCELERATION dependent

  // computed by mj_fwdActuation
  actuator_force / actuator_force_mut: [f64; "nu * 1"]    = "actuator force in actuation space";
  qfrc_actuator / qfrc_actuator_mut: [f64; "nv * 1"]     = "actuator force";

  // computed by mj_fwdAcceleration
  qfrc_smooth / qfrc_smooth_mut: [f64; "nv * 1"]       = "net unconstrained force";
  qacc_smooth / qacc_smooth_mut: [f64; "nv * 1"]       = "unconstrained acceleration";

  // computed by mj_fwdConstraint/mj_inverse
  qfrc_constraint / qfrc_constraint_mut: [f64; "nv * 1"]   = "constraint force";

  // computed by mj_inverse
  qfrc_inverse / qfrc_inverse_mut: [f64; "; should equal"]      = "net ";
                             // qfrc_applied + J'*xfrc_applied + qfrc_actuator   (nv * 1)

  // computed by mj_sensorAcc/mj_rnePostConstraint if needed; rotation:translation format
  cacc / cacc_mut: [f64; "nbody * 6"]              = "com-based acceleration";
  cfrc_int / cfrc_int_mut: [f64; "nbody * 6"]          = "com-based interaction force with parent";
  cfrc_ext / cfrc_ext_mut: [f64; "nbody * 6"]          = "com-based external force on body";

  //-------------------- arena-allocated: POSITION dependent

  /* handle specially in later...
    // computed by mj_collision
    mjContact* contact;        // array of all detected contacts                   (ncon * 1)
  */

  // computed by mj_makeConstraint
  efc_type / efc_type_mut: [i32; "nefc * 1"]          = "constraint type (mjtConstraint)";
  efc_id / efc_id_mut: [i32; "nefc * 1"]            = "id of object of specified type";
  efc_J_rownnz / efc_J_rownnz_mut: [i32; "nefc * 1"]      = "number of non-zeros in constraint Jacobian row";
  efc_J_rowadr / efc_J_rowadr_mut: [i32; "nefc * 1"]      = "row start address in colind array";
  efc_J_rowsuper / efc_J_rowsuper_mut: [i32; "nefc * 1"]    = "number of subsequent rows in supernode";
  efc_J_colind / efc_J_colind_mut: [i32; "nJ * 1"]      = "column indices in constraint Jacobian";
  efc_JT_rownnz / efc_JT_rownnz_mut: [i32; "nv * 1"]     = "number of non-zeros in constraint Jacobian row ";
  efc_JT_rowadr / efc_JT_rowadr_mut: [i32; "nv * 1"]     = "row start address in colind array              ";
  efc_JT_rowsuper / efc_JT_rowsuper_mut: [i32; "nv * 1"]   = "number of subsequent rows in supernode         ";
  efc_JT_colind / efc_JT_colind_mut: [i32; "nJ * 1"]     = "column indices in constraint Jacobian          ";
  efc_J / efc_J_mut: [f64; "nJ * 1"]             = "constraint Jacobian";
  efc_JT / efc_JT_mut: [f64; "nJ * 1"]            = "constraint Jacobian transposed";
  efc_pos / efc_pos_mut: [f64; "nefc * 1"]           = "constraint position (equality, contact)";
  efc_margin / efc_margin_mut: [f64; "nefc * 1"]        = "inclusion margin (contact)";
  efc_frictionloss / efc_frictionloss_mut: [f64; "nefc * 1"]  = "frictionloss (friction)";
  efc_diagApprox / efc_diagApprox_mut: [f64; "nefc * 1"]    = "approximation to diagonal of A";
  efc_KBIP / efc_KBIP_mut: [f64; "nefc * 4"]          = "stiffness, damping, impedance, imp'";
  efc_D / efc_D_mut: [f64; "nefc * 1"]             = "constraint mass";
  efc_R / efc_R_mut: [f64; "nefc * 1"]             = "inverse constraint mass";
  tendon_efcadr / tendon_efcadr_mut: [i32; "ntendon * 1"]     = "first efc address involving tendon; -1: none";

  // computed by mj_island (island dof structure)
  dof_island / dof_island_mut: [i32; "nv * 1"]        = "island id of this dof; -1: none";
  island_dofadr / island_dofadr_mut: [i32; "nisland * 1"]     = "island start address in dof vector";

  // computed by mj_island (island constraint structure)
  efc_island / efc_island_mut: [i32; "nefc * 1"]        = "island id of this constraint";

  // computed by mj_projectConstraint (PGS solver)
  efc_AR_rownnz / efc_AR_rownnz_mut: [i32; "nefc * 1"]     = "number of non-zeros in AR";
  efc_AR_rowadr / efc_AR_rowadr_mut: [i32; "nefc * 1"]     = "row start address in colind array";
  efc_AR_colind / efc_AR_colind_mut: [i32; "nA * 1"]     = "column indices in sparse AR";
  efc_AR / efc_AR_mut: [f64; "nA * 1"]            = "J*inv(M)*J' + R";

  //-------------------- arena-allocated: POSITION, VELOCITY dependent

  // computed by mj_fwdVelocity/mj_referenceConstraint
  efc_vel / efc_vel_mut: [f64; "nefc * 1"]           = "velocity in constraint space: J*qvel";
  efc_aref / efc_aref_mut: [f64; "nefc * 1"]          = "reference pseudo-acceleration";

  //-------------------- arena-allocated: POSITION, VELOCITY, CONTROL/ACCELERATION dependent

  // computed by mj_fwdConstraint/mj_inverse
  efc_b / efc_b_mut: [f64; "nefc * 1"]             = "linear cost term: J*qacc_smooth - aref";
  efc_state / efc_state_mut: [i32; "nefc * 1"]         = "constraint state (mjtConstraintState)";
  efc_force / efc_force_mut: [f64; "nefc * 1"]         = "constraint force in constraint space";
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
    pub unsafe fn contact(&self, len: usize) -> &[MjContact] {
        let slice: &[crate::bindgen::mjContact] = unsafe { std::slice::from_raw_parts(self.0.contact, len) };
        // SAFETY: `MjContact` is just a **wrapper** struct of `crate::bindgen::mjContact`
        unsafe { std::mem::transmute(slice) }
    }
}

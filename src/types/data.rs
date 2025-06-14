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
    ($($name:ident: [$T:ty; $size:ident $(* $mul_lit:literal)? $(* $mul_const:ident)? $(x $mul:ident)?] = $description:literal;)*) => {
        impl MjModel {
            $(
                #[allow(non_snake_case)]
                #[doc = $description]
                pub fn $name(&self) -> &[$T] {
                    unsafe { std::slice::from_raw_parts(self.0.$name, self.$size() $(* $mul_lit)? $(* $mul_const as usize)? $(* self.$mul())?) }
                }
            )*
        }
    };
}
impl_buffer_slices! {
  // state
  mjtNum* qpos;              // position                                         (nq * 1)
  mjtNum* qvel;              // velocity                                         (nv * 1)
  mjtNum* act;               // actuator activation                              (na * 1)
  mjtNum* qacc_warmstart;    // acceleration used for warmstart                  (nv * 1)
  mjtNum* plugin_state;      // plugin state                                     (npluginstate * 1)

  // control
  mjtNum* ctrl;              // control                                          (nu * 1)
  mjtNum* qfrc_applied;      // applied generalized force                        (nv * 1)
  mjtNum* xfrc_applied;      // applied Cartesian force/torque                   (nbody * 6)
  mjtByte* eq_active;        // enable/disable constraints                       (neq * 1)

  // mocap data
  mjtNum* mocap_pos;         // positions of mocap bodies                        (nmocap * 3)
  mjtNum* mocap_quat;        // orientations of mocap bodies                     (nmocap * 4)

  // dynamics
  mjtNum* qacc;              // acceleration                                     (nv * 1)
  mjtNum* act_dot;           // time-derivative of actuator activation           (na * 1)

  // user data
  mjtNum* userdata;          // user data, not touched by engine                 (nuserdata * 1)

  // sensors
  mjtNum* sensordata;        // sensor data array                                (nsensordata * 1)

  // plugins
  int*       plugin;         // copy of m->plugin, required for deletion         (nplugin * 1)
  uintptr_t* plugin_data;    // pointer to plugin-managed data structure         (nplugin * 1)

  //-------------------- POSITION dependent

  // computed by mj_fwdPosition/mj_kinematics
  mjtNum* xpos;              // Cartesian position of body frame                 (nbody * 3)
  mjtNum* xquat;             // Cartesian orientation of body frame              (nbody * 4)
  mjtNum* xmat;              // Cartesian orientation of body frame              (nbody * 9)
  mjtNum* xipos;             // Cartesian position of body com                   (nbody * 3)
  mjtNum* ximat;             // Cartesian orientation of body inertia            (nbody * 9)
  mjtNum* xanchor;           // Cartesian position of joint anchor               (njnt * 3)
  mjtNum* xaxis;             // Cartesian joint axis                             (njnt * 3)
  mjtNum* geom_xpos;         // Cartesian geom position                          (ngeom * 3)
  mjtNum* geom_xmat;         // Cartesian geom orientation                       (ngeom * 9)
  mjtNum* site_xpos;         // Cartesian site position                          (nsite * 3)
  mjtNum* site_xmat;         // Cartesian site orientation                       (nsite * 9)
  mjtNum* cam_xpos;          // Cartesian camera position                        (ncam * 3)
  mjtNum* cam_xmat;          // Cartesian camera orientation                     (ncam * 9)
  mjtNum* light_xpos;        // Cartesian light position                         (nlight * 3)
  mjtNum* light_xdir;        // Cartesian light direction                        (nlight * 3)

  // computed by mj_fwdPosition/mj_comPos
  mjtNum* subtree_com;       // center of mass of each subtree                   (nbody * 3)
  mjtNum* cdof;              // com-based motion axis of each dof (rot:lin)      (nv * 6)
  mjtNum* cinert;            // com-based body inertia and mass                  (nbody * 10)

  // computed by mj_fwdPosition/mj_flex
  mjtNum* flexvert_xpos;     // Cartesian flex vertex positions                  (nflexvert * 3)
  mjtNum* flexelem_aabb;     // flex element bounding boxes (center, size)       (nflexelem * 6)
  int*    flexedge_J_rownnz; // number of non-zeros in Jacobian row              (nflexedge * 1)
  int*    flexedge_J_rowadr; // row start address in colind array                (nflexedge * 1)
  int*    flexedge_J_colind; // column indices in sparse Jacobian                (nflexedge x nv)
  mjtNum* flexedge_J;        // flex edge Jacobian                               (nflexedge x nv)
  mjtNum* flexedge_length;   // flex edge lengths                                (nflexedge * 1)

  // computed by mj_fwdPosition/mj_tendon
  int*    ten_wrapadr;       // start address of tendon's path                   (ntendon * 1)
  int*    ten_wrapnum;       // number of wrap points in path                    (ntendon * 1)
  int*    ten_J_rownnz;      // number of non-zeros in Jacobian row              (ntendon * 1)
  int*    ten_J_rowadr;      // row start address in colind array                (ntendon * 1)
  int*    ten_J_colind;      // column indices in sparse Jacobian                (ntendon x nv)
  mjtNum* ten_J;             // tendon Jacobian                                  (ntendon x nv)
  mjtNum* ten_length;        // tendon lengths                                   (ntendon * 1)
  int*    wrap_obj;          // geom id; -1: site; -2: pulley                    (nwrap x 2)
  mjtNum* wrap_xpos;         // Cartesian 3D points in all paths                 (nwrap * 6)

  // computed by mj_fwdPosition/mj_transmission
  mjtNum* actuator_length;   // actuator lengths                                 (nu * 1)
  int*    moment_rownnz;     // number of non-zeros in actuator_moment row       (nu * 1)
  int*    moment_rowadr;     // row start address in colind array                (nu * 1)
  int*    moment_colind;     // column indices in sparse Jacobian                (nJmom * 1)
  mjtNum* actuator_moment;   // actuator moments                                 (nJmom * 1)

  // computed by mj_fwdPosition/mj_makeM
  mjtNum* crb;               // com-based composite inertia and mass             (nbody * 10)
  mjtNum* qM;                // inertia (sparse)                                 (nM * 1)
  mjtNum* M;                 // reduced inertia (compressed sparse row)          (nC * 1)

  // computed by mj_fwdPosition/mj_factorM
  mjtNum* qLD;               // L'*D*L factorization of M (sparse)               (nC * 1)
  mjtNum* qLDiagInv;         // 1/diag(D)                                        (nv * 1)

  // computed by mj_collisionTree
  mjtNum*  bvh_aabb_dyn;     // global bounding box (center, size)               (nbvhdynamic * 6)
  mjtByte* bvh_active;       // was bounding volume checked for collision        (nbvh * 1)

  //-------------------- POSITION, VELOCITY dependent

  // computed by mj_fwdVelocity
  mjtNum* flexedge_velocity; // flex edge velocities                             (nflexedge * 1)
  mjtNum* ten_velocity;      // tendon velocities                                (ntendon * 1)
  mjtNum* actuator_velocity; // actuator velocities                              (nu * 1)

  // computed by mj_fwdVelocity/mj_comVel
  mjtNum* cvel;              // com-based velocity (rot:lin)                     (nbody * 6)
  mjtNum* cdof_dot;          // time-derivative of cdof (rot:lin)                (nv * 6)

  // computed by mj_fwdVelocity/mj_rne (without acceleration)
  mjtNum* qfrc_bias;         // C(qpos,qvel)                                     (nv * 1)

  // computed by mj_fwdVelocity/mj_passive
  mjtNum* qfrc_spring;       // passive spring force                             (nv * 1)
  mjtNum* qfrc_damper;       // passive damper force                             (nv * 1)
  mjtNum* qfrc_gravcomp;     // passive gravity compensation force               (nv * 1)
  mjtNum* qfrc_fluid;        // passive fluid force                              (nv * 1)
  mjtNum* qfrc_passive;      // total passive force                              (nv * 1)

  // computed by mj_sensorVel/mj_subtreeVel if needed
  mjtNum* subtree_linvel;    // linear velocity of subtree com                   (nbody * 3)
  mjtNum* subtree_angmom;    // angular momentum about subtree com               (nbody * 3)

  // computed by mj_Euler or mj_implicit
  mjtNum* qH;                // L'*D*L factorization of modified M               (nC * 1)
  mjtNum* qHDiagInv;         // 1/diag(D) of modified M                          (nv * 1)

  // computed by mj_resetData
  int*    B_rownnz;          // body-dof: non-zeros in each row                  (nbody * 1)
  int*    B_rowadr;          // body-dof: address of each row in B_colind        (nbody * 1)
  int*    B_colind;          // body-dof: column indices of non-zeros            (nB * 1)
  int*    M_rownnz;          // reduced inertia: non-zeros in each row           (nv * 1)
  int*    M_rowadr;          // reduced inertia: address of each row in M_colind (nv * 1)
  int*    M_colind;          // reduced inertia: column indices of non-zeros     (nC * 1)
  int*    mapM2M;            // index mapping from qM to M                       (nC * 1)
  int*    D_rownnz;          // full inertia: non-zeros in each row              (nv * 1)
  int*    D_rowadr;          // full inertia: address of each row in D_colind    (nv * 1)
  int*    D_diag;            // full inertia: index of diagonal element          (nv * 1)
  int*    D_colind;          // full inertia: column indices of non-zeros        (nD * 1)
  int*    mapM2D;            // index mapping from qM to D                       (nD * 1)
  int*    mapD2M;            // index mapping from D to qM                       (nM * 1)

  // computed by mj_implicit/mj_derivative
  mjtNum* qDeriv;            // d (passive + actuator - bias) / d qvel           (nD * 1)

  // computed by mj_implicit/mju_factorLUSparse
  mjtNum* qLU;               // sparse LU of (qM - dt*qDeriv)                    (nD * 1)

  //-------------------- POSITION, VELOCITY, CONTROL/ACCELERATION dependent

  // computed by mj_fwdActuation
  mjtNum* actuator_force;    // actuator force in actuation space                (nu * 1)
  mjtNum* qfrc_actuator;     // actuator force                                   (nv * 1)

  // computed by mj_fwdAcceleration
  mjtNum* qfrc_smooth;       // net unconstrained force                          (nv * 1)
  mjtNum* qacc_smooth;       // unconstrained acceleration                       (nv * 1)

  // computed by mj_fwdConstraint/mj_inverse
  mjtNum* qfrc_constraint;   // constraint force                                 (nv * 1)

  // computed by mj_inverse
  mjtNum* qfrc_inverse;      // net external force; should equal:
                             // qfrc_applied + J'*xfrc_applied + qfrc_actuator   (nv * 1)

  // computed by mj_sensorAcc/mj_rnePostConstraint if needed; rotation:translation format
  mjtNum* cacc;              // com-based acceleration                           (nbody * 6)
  mjtNum* cfrc_int;          // com-based interaction force with parent          (nbody * 6)
  mjtNum* cfrc_ext;          // com-based external force on body                 (nbody * 6)

  //-------------------- arena-allocated: POSITION dependent

  // computed by mj_collision
  mjContact* contact;        // array of all detected contacts                   (ncon * 1)

  // computed by mj_makeConstraint
  int*    efc_type;          // constraint type (mjtConstraint)                  (nefc * 1)
  int*    efc_id;            // id of object of specified type                   (nefc * 1)
  int*    efc_J_rownnz;      // number of non-zeros in constraint Jacobian row   (nefc * 1)
  int*    efc_J_rowadr;      // row start address in colind array                (nefc * 1)
  int*    efc_J_rowsuper;    // number of subsequent rows in supernode           (nefc * 1)
  int*    efc_J_colind;      // column indices in constraint Jacobian            (nJ * 1)
  int*    efc_JT_rownnz;     // number of non-zeros in constraint Jacobian row T (nv * 1)
  int*    efc_JT_rowadr;     // row start address in colind array              T (nv * 1)
  int*    efc_JT_rowsuper;   // number of subsequent rows in supernode         T (nv * 1)
  int*    efc_JT_colind;     // column indices in constraint Jacobian          T (nJ * 1)
  mjtNum* efc_J;             // constraint Jacobian                              (nJ * 1)
  mjtNum* efc_JT;            // constraint Jacobian transposed                   (nJ * 1)
  mjtNum* efc_pos;           // constraint position (equality, contact)          (nefc * 1)
  mjtNum* efc_margin;        // inclusion margin (contact)                       (nefc * 1)
  mjtNum* efc_frictionloss;  // frictionloss (friction)                          (nefc * 1)
  mjtNum* efc_diagApprox;    // approximation to diagonal of A                   (nefc * 1)
  mjtNum* efc_KBIP;          // stiffness, damping, impedance, imp'              (nefc * 4)
  mjtNum* efc_D;             // constraint mass                                  (nefc * 1)
  mjtNum* efc_R;             // inverse constraint mass                          (nefc * 1)
  int*    tendon_efcadr;     // first efc address involving tendon; -1: none     (ntendon * 1)

  // computed by mj_island (island dof structure)
  int*    dof_island;        // island id of this dof; -1: none                  (nv * 1)
  int*    island_nv;         // number of dofs in this island                    (nisland * 1)
  int*    island_idofadr;    // island start address in idof vector              (nisland * 1)
  int*    island_dofadr;     // island start address in dof vector               (nisland * 1)
  int*    map_dof2idof;      // map from dof to idof                             (nv * 1)
  int*    map_idof2dof;      // map from idof to dof;  >= nidof: unconstrained   (nv * 1)

  // computed by mj_island (dofs sorted by island)
  mjtNum* ifrc_smooth;       // net unconstrained force                          (nidof * 1)
  mjtNum* iacc_smooth;       // unconstrained acceleration                       (nidof * 1)
  int*    iM_rownnz;         // inertia: non-zeros in each row                   (nidof * 1)
  int*    iM_rowadr;         // inertia: address of each row in iM_colind        (nidof * 1)
  int*    iM_colind;         // inertia: column indices of non-zeros             (nC * 1)
  mjtNum* iM;                // total inertia (sparse)                           (nC * 1)
  mjtNum* iLD;               // L'*D*L factorization of M (sparse)               (nC * 1)
  mjtNum* iLDiagInv;         // 1/diag(D)                                        (nidof * 1)
  mjtNum* iacc;              // acceleration                                     (nidof * 1)

  // computed by mj_island (island constraint structure)
  int*    efc_island;        // island id of this constraint                     (nefc * 1)
  int*    island_ne;         // number of equality constraints in island         (nisland * 1)
  int*    island_nf;         // number of friction constraints in island         (nisland * 1)
  int*    island_nefc;       // number of constraints in island                  (nisland * 1)
  int*    island_iefcadr;    // start address in iefc vector                     (nisland * 1)
  int*    map_efc2iefc;      // map from efc to iefc                             (nefc * 1)
  int*    map_iefc2efc;      // map from iefc to efc                             (nefc * 1)

  // computed by mj_island (constraints sorted by island)
  int*    iefc_type;         // constraint type (mjtConstraint)                  (nefc * 1)
  int*    iefc_id;           // id of object of specified type                   (nefc * 1)
  int*    iefc_J_rownnz;     // number of non-zeros in constraint Jacobian row   (nefc * 1)
  int*    iefc_J_rowadr;     // row start address in colind array                (nefc * 1)
  int*    iefc_J_rowsuper;   // number of subsequent rows in supernode           (nefc * 1)
  int*    iefc_J_colind;     // column indices in constraint Jacobian            (nJ * 1)
  int*    iefc_JT_rownnz;    // number of non-zeros in constraint Jacobian row T (nidof * 1)
  int*    iefc_JT_rowadr;    // row start address in colind array              T (nidof * 1)
  int*    iefc_JT_rowsuper;  // number of subsequent rows in supernode         T (nidof * 1)
  int*    iefc_JT_colind;    // column indices in constraint Jacobian          T (nJ * 1)
  mjtNum* iefc_J;            // constraint Jacobian                              (nJ * 1)
  mjtNum* iefc_JT;           // constraint Jacobian transposed                   (nJ * 1)
  mjtNum* iefc_frictionloss; // frictionloss (friction)                          (nefc * 1)
  mjtNum* iefc_D;            // constraint mass                                  (nefc * 1)
  mjtNum* iefc_R;            // inverse constraint mass                          (nefc * 1)

  // computed by mj_projectConstraint (PGS solver)
  int*    efc_AR_rownnz;     // number of non-zeros in AR                        (nefc * 1)
  int*    efc_AR_rowadr;     // row start address in colind array                (nefc * 1)
  int*    efc_AR_colind;     // column indices in sparse AR                      (nA * 1)
  mjtNum* efc_AR;            // J*inv(M)*J' + R                                  (nA * 1)

  //-------------------- arena-allocated: POSITION, VELOCITY dependent

  // computed by mj_fwdVelocity/mj_referenceConstraint
  mjtNum* efc_vel;           // velocity in constraint space: J*qvel             (nefc * 1)
  mjtNum* efc_aref;          // reference pseudo-acceleration                    (nefc * 1)

  //-------------------- arena-allocated: POSITION, VELOCITY, CONTROL/ACCELERATION dependent

  // computed by mj_fwdConstraint/mj_inverse
  mjtNum* efc_b;             // linear cost term: J*qacc_smooth - aref           (nefc * 1)
  mjtNum* iefc_aref;         // reference pseudo-acceleration                    (nefc * 1)
  int*    iefc_state;        // constraint state (mjtConstraintState)            (nefc * 1)
  mjtNum* iefc_force;        // constraint force in constraint space             (nefc * 1)
  int*    efc_state;         // constraint state (mjtConstraintState)            (nefc * 1)
  mjtNum* efc_force;         // constraint force in constraint space             (nefc * 1)
  mjtNum* ifrc_constraint;   // constraint force                                 (nidof * 1)
}

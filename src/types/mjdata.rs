//! # [mjData](https://mujoco.readthedocs.io/en/stable/APIreference/APItypes.html#mjdata)
//! 
//! This is the main data structure holding the simulation state. It is the workspace where all functions read their modifiable inputs and write their outputs.

pub use crate::bindgen::{
    mjtConstraint, mjtConstraintState,
    mjNSOLVER, mjNISLAND, mjNTIMER, mjNWARNING, mjMAXTHREAD,
};

resource_wrapper!(
    MjData for crate::bindgen::mjData;
    drop = crate::mj_deleteData;
);
fields_mapping!(MjData {
    scalars {
        // constant sizes
        narena: usize = "size of the arena in bytes (inclusive of the stack)";
        nbuffer: usize = "size of main buffer in bytes";
        nplugin: usize = "number of plugin instances";

        // stack pointer
        pstack: usize = "first available byte in stack";
        pbase: usize = "value of pstack when mj_markStack was last called";

        // arena pointer
        parena: usize = "first available byte in arena";

        // memory utilization statistics
        maxuse_stack: usize = "maximum stack allocation in bytes";
        maxuse_threadstack: [usize; mjMAXTHREAD] = "maximum stack allocation per thread in bytes";
        maxuse_arena: usize = "maximum arena allocation in bytes";
        maxuse_con: usize = "maximum number of contacts";
        maxuse_efc: usize = "maximum number of scalar constraints";

        // variable sizes
        ncon: usize = "number of detected contacts";
        ne: usize = "number of equality constraints";
        nf: usize = "number of friction constraints";
        nl: usize = "number of limit constraints";
        nefc: usize = "number of constraints";
        nJ: usize = "number of non-zeros in constraint Jacobian";
        nA: usize = "number of non-zeros in constraint inverse inertia matrix";
        nisland: usize = "number of detected constraint islands";

        // global properties
        time / set_time: f64 = "simulation time";
        energy: [f64; 2] = "potential, kinetic energy";

        signature: u64 = "compilation signature";
    }
    structs {
        // disgnostics
        warning: [crate::mjWarningStat; mjNWARNING] = "warning statistics";
        timer: [crate::mjTimerStat; mjNTIMER] = "timer statistics";
    }
});

// solver statistics
impl MjData {
    /// solver statistics per island, per iteration
    pub fn solver(&self) -> [crate::mjSolverStat; mjNISLAND as usize * mjNSOLVER as usize] {
        self.solver
    }
    /// number of solver iterations, per island
    pub fn solver_niter(&self) -> [usize; mjNISLAND as usize] {
        self.solver_niter.map(|x| x as usize)
    }
    /// number of nonzeros in Hessian or efc_AR, per island
    pub fn solver_nnz(&self) -> [usize; mjNISLAND as usize] {
        self.solver_nnz.map(|x| x as usize)
    }
    /// forward-inverse comparison: qfrc, efc
    pub fn solver_fwdinv(&self) -> [f64; 2] {
        self.solver_fwdinv
    }
}

use crate::{MjModel, ObjectId, obj, Joint};

impl MjData {
    pub fn ctrl(&self, id: ObjectId<obj::Actuator>) -> f64 {
        unsafe { self.ctrl.add(id.index()).read() }
    }
    pub fn set_ctrl(&mut self, id: ObjectId<obj::Actuator>, value: f64) {
        unsafe { self.ctrl.add(id.index()).write(value) };
    }

    pub fn act(&self, id: ObjectId<obj::Actuator>, model: &MjModel) -> Option<f64> {
        let offset = model.actuator_actadr(id)?;
        Some(unsafe { self.act.add(offset).read() })
    }
    /// Set the actuator activation value. `None` when the actuator is stateless.
    pub fn set_act(&mut self, id: ObjectId<obj::Actuator>, value: f64, model: &MjModel) -> Option<()> {
        let offset = model.actuator_actadr(id)?;
        unsafe { self.act.add(offset).write(value) };
        Some(())
    }

    pub fn qpos<J: Joint>(&self, id: ObjectId<J>, model: &MjModel) -> J::Qpos {
        let ptr = unsafe { self.qpos.add(model.jnt_qposadr(id)) };
        J::Qpos::try_from(unsafe { std::slice::from_raw_parts(ptr, J::QPOS_SIZE) }).ok().unwrap()
    }
    pub fn set_qpos<J: Joint>(&mut self, id: ObjectId<J>, qpos: J::Qpos, model: &MjModel) {
        let ptr = unsafe { self.qpos.add(model.jnt_qposadr(id)) };
        unsafe { std::ptr::copy_nonoverlapping(qpos.as_ref().as_ptr(), ptr, J::QPOS_SIZE) };
    }

    pub fn qvel<J: Joint>(&self, id: ObjectId<J>, model: &MjModel) -> J::Qvel {
        let ptr = unsafe { self.qvel.add(model.jnt_dofadr(id).expect("Currently we don't support `weld` in `joint::`, so this will not be None...")) };
        J::Qvel::try_from(unsafe { std::slice::from_raw_parts(ptr, J::QVEL_SIZE) }).ok().unwrap()
    }
    pub fn set_qvel<J: Joint>(&mut self, id: ObjectId<J>, qvel: J::Qvel, model: &MjModel) {
        let ptr = unsafe { self.qvel.add(model.jnt_dofadr(id).expect("Currently we don't support `weld` in `joint::`, so this will not be None...")) };
        unsafe { std::ptr::copy_nonoverlapping(qvel.as_ref().as_ptr(), ptr, J::QVEL_SIZE) };
    }

    pub fn qacc_warmstart(&self, id: ObjectId<obj::Dof>) -> f64 {
        unsafe { self.qacc_warmstart.add(id.index()).read() }
    }
    pub fn set_qacc_warmstart(&mut self, id: ObjectId<obj::Dof>, value: f64) {
        unsafe { self.qacc_warmstart.add(id.index()).write(value) };
    }

    pub fn plugin_state(&self, id: ObjectId<obj::Plugin>, model: &MjModel) -> Option<f64> {
        let offset = model.plugin_stateadr(id)?;
        Some(unsafe { self.plugin_state.add(offset).read() })
    }
    /// Set the plugin state value. Returns `None` when the plugin is stateless.
    pub fn set_plugin_state(&mut self, id: ObjectId<obj::Plugin>, value: f64, model: &MjModel) -> Option<()> {
        let offset = model.plugin_stateadr(id)?;
        unsafe { self.plugin_state.add(offset).write(value) };
        Some(())
    }

    pub fn qfrc_applied(&self, id: ObjectId<obj::Dof>) -> f64 {
        unsafe { self.qfrc_applied.add(id.index()).read() }
    }
    pub fn set_qfrc_applied(&mut self, id: ObjectId<obj::Dof>, value: f64) {
        unsafe { self.qfrc_applied.add(id.index()).write(value) };
    }

    pub fn xfrc_applied(&self, id: ObjectId<obj::Body>) -> [f64; 6] {
        let ptr = unsafe { self.xfrc_applied.add(id.index() * 6) };
        std::array::from_fn(|i| unsafe { ptr.add(i).read() })
    }
    pub fn set_xfrc_applied(&mut self, id: ObjectId<obj::Body>, value: [f64; 6]) {
        let ptr = unsafe { self.xfrc_applied.add(id.index() * 6) };
        unsafe { std::ptr::copy_nonoverlapping(value.as_ptr(), ptr, value.len()) };
    }

    pub fn eq_active(&self, id: ObjectId<obj::Equality>) -> bool {
        (unsafe { self.eq_active.add(id.index()).read() }) != 0
    }
    pub fn set_eq_active(&mut self, id: ObjectId<obj::Equality>, value: bool) {
        unsafe { self.eq_active.add(id.index()).write(if value {1} else {0}) };
    }

    /// `None` when the body is not a mocap body.
    pub fn mocap_pos(&self, id: ObjectId<obj::Body>, model: &MjModel) -> Option<[f64; 3]> {
        let offset = model.body_mocapid(id)?.index() * 3;
        Some(std::array::from_fn(|i| unsafe { self.mocap_pos.add(offset + i).read() }))
    }
    /// `None` when the body is not a mocap body.
    pub fn set_mocap_pos(&mut self, id: ObjectId<obj::Body>, pos: [f64; 3], model: &MjModel) -> Option<()> {
        let offset = model.body_mocapid(id)?.index() * 3;
        unsafe { std::ptr::copy_nonoverlapping(pos.as_ptr(), self.mocap_pos.add(offset), pos.len()) };
        Some(())
    }

    pub fn mocap_quat(&self, id: ObjectId<obj::Body>, model: &MjModel) -> Option<[f64; 4]> {
        let offset = model.body_mocapid(id)?.index() * 4;
        Some(std::array::from_fn(|i| unsafe { self.mocap_quat.add(offset + i).read() }))
    }
    /// `None` when the body is not a mocap body.
    pub fn set_mocap_quat(&mut self, id: ObjectId<obj::Body>, quat: [f64; 4], model: &MjModel) -> Option<()> {
        let offset = model.body_mocapid(id)?.index() * 4;
        unsafe { std::ptr::copy_nonoverlapping(quat.as_ptr(), self.mocap_quat.add(offset), quat.len()) };
        Some(())
    }
}

/*
 * TODO: provide idiomatic accessors also for the following buffers...
 */

// transmute: `u8` -> `bool`
macro_rules! buffer_slices_depending_on_model {
    ($($name:ident : [$T:ty; $size:ident $(* $lit:literal)? $(* $var:ident)?] = $description:literal;)*) => {
        #[allow(non_snake_case)]
        impl MjData {
            $(
                #[doc = $description]
                #[doc = "\n"]
                #[doc = "**SAFETY**: `model` must be exactly the same model as the one used to create this `mjData`."]
                pub unsafe fn $name(&self, model: &MjModel) -> &[$T] {
                    #[cfg(debug_assertions/* size check */)] {
                        #[allow(unnecessary_transmutes)]
                        let _: $T = unsafe { std::mem::transmute(*self.$name) };
                    }
                    unsafe { std::slice::from_raw_parts(self.$name as *const $T, model.$size()$(* $lit)?$(* model.$var())?) }
                }
            )*
        }
    };
    ($($name:ident / $mut_name:ident : [$T:ty; $size:ident $(* $mul:literal)?] = $description:literal;)*) => {
        #[allow(non_snake_case)]
        impl MjData {
            $(
                #[doc = $description]
                #[doc = "\n"]
                #[doc = "**SAFETY**: `model` must be exactly the same model as the one used to create this `mjData`."]
                pub unsafe fn $name(&self, model: &MjModel) -> &[$T] {
                    #[cfg(debug_assertions/* size check */)] {
                        #[allow(unnecessary_transmutes)]
                        let _: $T = unsafe { std::mem::transmute(*self.$name) };
                    }
                    unsafe { std::slice::from_raw_parts(self.$name as *const $T, model.$size()$(* $mul)?) }
                }

                #[doc = "mutable "]
                #[doc = $description]
                #[doc = "\n"]
                #[doc = "**SAFETY**: `model` must be exactly the same model as the one used to create this `mjData`."]
                pub unsafe fn $mut_name(&mut self, model: &MjModel) -> &mut [$T] {
                    #[cfg(debug_assertions/* size check */)] {
                        #[allow(unnecessary_transmutes)]
                        let _: $T = unsafe { std::mem::transmute(*self.$name) };
                    }
                    unsafe { std::slice::from_raw_parts_mut(self.$name as *mut $T, model.$size()$(* $mul)?) }
                }
            )*
        }
    };
}
// user-touchable buffers
buffer_slices_depending_on_model! {
    // user data
    userdata / userdata_mut: [f64; nuserdata] = "user data, not touched by engine (nuserdata x 1)";
}
// read-only buffers; for MuJoCo's internal use or output
buffer_slices_depending_on_model! {
    // dynamics
    qacc: [f64; nv] = "acceleration (nv x 1)";
    act_dot: [f64; na] = "time-derivative of actuator activation (na x 1)";

    // sensors
    sensordata: [f64; nsensordata] = "sensor data array (nsensordata x 1)";

    // plugins
    plugin: [i32; nplugin] = "copy of m->plugin, required for deletion";
    plugin_data: [usize; nplugin] = "pointer to plugin-managed data structure (nplugin x 1)";

    // computed by mj_fwdPosition/mj_kinematics
    xpos: [f64; nbody * 3] = "Cartesian position of body frame (nbody x 3)";
    xquat: [f64; nbody * 4] = "Cartesian orientation of body frame (nbody x 4)";
    xmat: [f64; nbody * 9] = "Cartesian orientation of body frame (nbody x 9)";
    xipos: [f64; nbody * 3] = "Cartesian position of body com (nbody x 3)";
    ximat: [f64; nbody * 9] = "Cartesian orientation of body inertia (nbody x 9)";
    xanchor: [f64; njnt * 3] = "Cartesian position of joint anchor (njnt x 3)";
    xaxis: [f64; njnt * 3] = "Cartesian joint axis (njnt x 3)";
    geom_xpos: [f64; ngeom * 3] = "Cartesian geom position (ngeom x 3)";
    geom_xmat: [f64; ngeom * 9] = "Cartesian geom orientation (ngeom x 9)";
    site_xpos: [f64; nsite * 3] = "Cartesian site position (nsite x 3)";
    site_xmat: [f64; nsite * 9] = "Cartesian site orientation (nsite x 9)";
    cam_xpos: [f64; ncam * 3] = "Cartesian camera position (ncam x 3)";
    cam_xmat: [f64; ncam * 9] = "Cartesian camera orientation (ncam x 9)";
    light_xpos: [f64; nlight * 3] = "Cartesian light position (nlight x 3)";
    light_xdir: [f64; nlight * 3] = "Cartesian light direction (nlight x 3)";

    // computed by mj_fwdPosition/mj_comPos
    subtree_com: [f64; nbody * 3] = "center of mass of each subtree (nbody x 3)";
    cdof: [f64; nv * 6] = "com-based motion axis of each dof (rot:lin) (nv x 6)";
    cinert: [f64; nbody * 10] = "com-based body inertia and mass (nbody x 10)";

    // computed by mj_fwdPosition/mj_flex
    flexvert_xpos: [f64; nflexvert * 3] = "Cartesian flex vertex positions (nflexvert x 3)";
    flexelem_aabb: [f64; nflexelem * 6] = "flex element bounding boxes (center, size) (nflexelem x 6)";
    flexedge_J: [f64; nflexedge * nv] = "flex edge Jacobian (nflexedge x nv)";
    flexedge_length: [f64; nflexedge * 1] = "flex edge lengths (nflexedge x 1)";

    // computed by mj_fwdPosition/mj_flex
    flexedge_J_rownnz: [i32; nflexedge * 1] = "number of non-zeros in Jacobian row (nflexedge x 1)";
    flexedge_J_rowadr: [i32; nflexedge * 1] = "row start address in colind array (nflexedge x 1)";
    flexedge_J_colind: [i32; nflexedge * nv] = "column indices in sparse Jacobian (nflexedge x nv)";

    // computed by mj_fwdPosition/mj_tendon
    ten_wrapadr: [i32; ntendon * 1] = "start address of tendon's path (ntendon x 1)";
    ten_wrapnum: [i32; ntendon * 1] = "number of wrap points in path (ntendon x 1)";
    ten_J_rownnz: [i32; ntendon * 1] = "number of non-zeros in Jacobian row (ntendon x 1)";
    ten_J_rowadr: [i32; ntendon * 1] = "row start address in colind array (ntendon x 1)";
    ten_J_colind: [i32; ntendon * nv] = "column indices in sparse Jacobian (ntendon x nv)";
    wrap_obj: [i32; nwrap * 2] = "geom id; -1: site; -2: pulley (nwrap x 2)";

    // computed by mj_fwdPosition/mj_tendon
    ten_J: [f64; ntendon * nv] = "tendon Jacobian (ntendon x nv)";
    ten_length: [f64; ntendon * 1] = "tendon lengths (ntendon x 1)";
    wrap_xpos: [f64; nwrap * 6] = "Cartesian 3D points in all paths (nwrap x 6)";

    // computed by mj_fwdPosition/mj_transmission
    actuator_length: [f64; nu * 1] = "actuator lengths (nu x 1)";
    actuator_moment: [f64; nJmom * 1] = "actuator moments (nJmom x 1)";

    // computed by mj_fwdPosition/mj_transmission
    moment_rownnz: [i32; nu * 1] = "number of non-zeros in actuator_moment row (nu x 1)";
    moment_rowadr: [i32; nu * 1] = "row start address in colind array (nu x 1)";
    moment_colind: [i32; nJmom * 1] = "column indices in sparse Jacobian (nJmom x 1)";

    // computed by mj_fwdPosition/mj_makeM
    crb: [f64; nbody * 10] = "com-based composite inertia and mass (nbody x 10)";
    qM: [f64; nM * 1] = "inertia (sparse) (nM x 1)";

    // computed by mj_fwdPosition/mj_factorM
    qLD: [f64; nC * 1] = "L'*D*L factorization of M (sparse) (nC x 1)";
    qLDiagInv: [f64; nv * 1] = "1/diag(D) (nv x 1)";

    // computed by mj_collisionTree
    bvh_aabb_dyn: [f64; nbvhdynamic * 6] = "global bounding box (center, size) (nbvhdynamic x 6)";
    bvh_active: [bool; nbvh * 1] = "was bounding volume checked for collision (nbvh x 1)";

    // computed by mj_fwdVelocity
    flexedge_velocity: [f64; nflexedge * 1] = "flex edge velocities (nflexedge x 1)";
    ten_velocity: [f64; ntendon * 1] = "tendon velocities (ntendon x 1)";
    actuator_velocity: [f64; nu * 1] = "actuator velocities (nu x 1)";

    // computed by mj_fwdVelocity/mj_comVel
    cvel: [f64; nbody * 6] = "com-based velocity (rot:lin) (nbody x 6)";
    cdof_dot: [f64; nv * 6] = "time-derivative of cdof (rot:lin) (nv x 6)";

    // computed by mj_fwdVelocity/mj_rne (without acceleration)
    qfrc_bias: [f64; nv * 1] = "C(qpos,qvel) (nv x 1)";

    // computed by mj_fwdVelocity/mj_passive
    qfrc_spring: [f64; nv * 1] = "passive spring force (nv x 1)";
    qfrc_damper: [f64; nv * 1] = "passive damper force (nv x 1)";
    qfrc_gravcomp: [f64; nv * 1] = "passive gravity compensation force (nv x 1)";
    qfrc_fluid: [f64; nv * 1] = "passive fluid force (nv x 1)";
    qfrc_passive: [f64; nv * 1] = "total passive force (nv x 1)";

    // computed by mj_sensorVel/mj_subtreeVel if needed
    subtree_linvel: [f64; nbody * 3] = "linear velocity of subtree com (nbody x 3)";
    subtree_angmom: [f64; nbody * 3] = "angular momentum about subtree com (nbody x 3)";

    // computed by mj_Euler or mj_implicit
    qH: [f64; nC * 1] = "L'*D*L factorization of modified M (nC x 1)";
    qHDiagInv: [f64; nv * 1] = "1/diag(D) of modified M (nv x 1)";

    // computed by mj_implicit/mj_derivative
    qDeriv: [f64; nD * 1] = "d (passive + actuator - bias) / d qvel (nD x 1)";

    // computed by mj_implicit/mju_factorLUSparse
    qLU: [f64; nD * 1] = "sparse LU of (qM - dt*qDeriv) (nD x 1)";

    // computed by mj_fwdActuation
    actuator_force: [f64; nu * 1] = "actuator force in actuation space (nu x 1)";
    qfrc_actuator: [f64; nv * 1] = "actuator force (nv x 1)";

    // computed by mj_fwdAcceleration
    qfrc_smooth: [f64; nv * 1] = "net unconstrained force (nv x 1)";
    qacc_smooth: [f64; nv * 1] = "unconstrained acceleration (nv x 1)";

    // computed by mj_fwdConstraint/mj_inverse
    qfrc_constraint: [f64; nv * 1] = "constraint force (nv x 1)";

    // computed by mj_inverse
    qfrc_inverse: [f64; nv * 1] = "net external force; should equal: qfrc_applied + J'*xfrc_applied + qfrc_actuator (nv x 1)";

    // computed by mj_sensorAcc/mj_rnePostConstraint if needed; rotation:translation format
    cacc: [f64; nbody * 6] = "com-based acceleration (rot:lin) (nbody x 6)";
    cfrc_int: [f64; nbody * 6] = "com-based interaction force with parent (rot:lin) (nbody x 6)";
    cfrc_ext: [f64; nbody * 6] = "com-based external force on body (rot:lin) (nbody x 6)";

    // computed by mj_makeConstraint
    tendon_efcadr: [i32; ntendon * 1] = "first efc address involving tendon; -1: none (ntendon x 1)";
    efc_JT_rownnz: [i32; nv * 1] = "number of non-zeros in constraint Jacobian row (nv x 1)";
    efc_JT_rowadr: [i32; nv * 1] = "row start address in colind array (nv x 1)";
    efc_JT_rowsuper: [i32; nv * 1] = "number of subsequent rows in supernode (nv x 1)";

    // computed by mj_resetData
    B_rownnz: [i32; nbody * 1] = "body-dof: non-zeros in each row (nbody x 1)";
    B_rowadr: [i32; nbody * 1] = "body-dof: address of each row in B_colind (nbody x 1)";
    B_colind: [i32; nB * 1] = "body-dof: column indices of non-zeros (nB x 1)";
    M_rownnz: [i32; nv * 1] = "reduced inertia: non-zeros in each row (nv x 1)";
    M_rowadr: [i32; nv * 1] = "reduced inertia: address of each row in M_colind (nv x 1)";
    M_colind: [i32; nC * 1] = "reduced inertia: column indices of non-zeros (nC x 1)";
    mapM2M: [i32; nC * 1] = "index mapping from qM to M (nC x 1)";
    D_rownnz: [i32; nv * 1] = "full inertia: non-zeros in each row (nv x 1)";
    D_rowadr: [i32; nv * 1] = "full inertia: address of each row in D_colind (nv x 1)";
    D_diag: [i32; nv * 1] = "full inertia: index of diagonal element in each row (nv x 1)";
    D_colind: [i32; nD * 1] = "full inertia: column indices of non-zeros (nD x 1)";
    mapM2D: [i32; nD * 1] = "index mapping from qM to D (nD x 1)";
    mapD2M: [i32; nM * 1] = "index mapping from D to qM (nM x 1)";

    // computed by mj_island (island dof structure)
    dof_island: [i32; nv * 1] = "island id of this dof; -1: none (nv x 1)";
}

// transmute: `i32` -> `mjtConstraint`, `mjtConstraintState`, `mjContact`
macro_rules! buffer_slices {
    ($($name:ident : [$T:ty; $size:ident $(* $lit:literal)?] = $description:literal;)*) => {
        #[allow(non_snake_case)]
        impl MjData {
            $(
                #[doc = $description]
                pub fn $name(&self) -> &[$T] {
                    #[cfg(debug_assertions/* size check */)] {
                        #[allow(unnecessary_transmutes)]
                        let _: $T = unsafe {std::mem::transmute(*self.$name)};
                    }
                    unsafe { std::slice::from_raw_parts(self.$name as *const $T, self.$size()) }
                }
            )*
        }
    };
}
buffer_slices! {
    // computed by mj_island (island dof structure)
    island_dofadr: [i32; nisland] = "island start address in dof vector (nisland x 1)";
    efc_island: [i32; nefc] = "island id of this constraint (nefc x 1)";

    // computed by mj_projectConstraint (PGS solver)
    efc_AR_rownnz: [i32; nefc] = "number of non-zeros in AR (nefc x 1)";
    efc_AR_rowadr: [i32; nefc] = "row start address in colind array (nefc x 1)";
    efc_AR_colind: [i32; nA] = "column indices in sparse AR (nA x 1)";
    efc_AR: [f64; nA] = "J*inv(M)*J' + R (nA x 1)";

    // computed by mj_fwdConstraint/mj_inverse
    efc_b: [f64; nefc] = "linear cost term: J*qacc_smooth - aref";
    efc_aref: [f64; nefc] = "reference pseudo-acceleration (nefc x 1)";
    efc_state: [mjtConstraintState; nefc] = "constraint state (mjtConstraintState)";
    efc_force: [f64; nefc] = "constraint force in constraint space";

    // computed by mj_makeConstraint
    efc_type: [mjtConstraint; nefc] = "constraint type (mjtConstraint)";
    efc_id: [i32; nefc] = "id of object of specified type";
    efc_J_rownnz: [i32; nefc] = "number of non-zeros in constraint Jacobian row (nefc x 1)";
    efc_J_rowadr: [i32; nefc] = "row start address in colind array (nefc x 1)";
    efc_J_rowsuper: [i32; nefc] = "number of subsequent rows in supernode (nefc x 1)";
    efc_J_colind: [i32; nJ] = "column indices in constraint Jacobian (nJ x 1)";
    efc_JT_colind: [i32; nJ] = "column indices in constraint Jacobian transposed (nJ x 1)";
    efc_J: [f64; nJ] = "constraint Jacobian (nJ x 1)";
    efc_JT: [f64; nJ] = "constraint Jacobian transposed (nJ x 1)";
    efc_pos: [f64; nefc] = "constraint position (equality, contact) (nefc x 1)";
    efc_margin: [f64; nefc] = "inclusion margin (contact) (nefc x 1)";
    efc_frictionloss: [f64; nefc] = "frictionloss (friction) (nefc x 1)";
    efc_diagApprox: [f64; nefc] = "approximation to diagonal of A (nefc x 1)";
    efc_KBIP: [f64; nefc * 4] = "stiffness, damping, impedance, imp' (nefc x 4)";
    efc_D: [f64; nefc] = "constraint mass (nefc x 1)";
    efc_R: [f64; nefc] = "inverse constraint mass (nefc x 1)";

    // computed by mj_collision
    contact: [crate::mjContact; ncon] = "array of all detected contacts";

    // computed by mj_fwdVelocity/mj_referenceConstraint
    efc_vel: [f64; nefc] = "velocity in constraint space: J*qvel (nefc x 1)";
}

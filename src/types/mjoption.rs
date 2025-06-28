//! # [mjOption](https://mujoco.readthedocs.io/en/stable/APIreference/APItypes.html#mjoption)
//! 
//! This is the data structure with simulation options. It corresponds to
//! the MJCF element [option](https://mujoco.readthedocs.io/en/stable/XMLreference.html#option).
//! One instance of it is embedded in [`mjModel`](crate::mjModel).

pub use crate::bindgen::{
    mjOption,
    mjtIntegrator, mjtCone, mjtJacobian, mjtSolver, mjtDisableBit, mjtEnableBit
};

derive_fields_mapping!(mjOption {
    scalars {
        // timing parameters
        timestep / set_timestep: f64 = "simulation timestep in seconds";
        apirate / set_apirate: f64 = "update rate for remote API in Hz";

        // solver parameters
        impratio / set_impratio: f64 = "ratio of friction-to-normal contact impedance";
        tolerance / set_tolerance: f64 = "main solver tolerance";
        ls_tolerance / set_ls_tolerance: f64 = "CG/Newton linesearch tolerance";
        noslip_tolerance / set_noslip_tolerance: f64 = "noslip solver tolerance";
        ccd_tolerance / set_ccd_tolerance: f64 = "convex collision solver tolerance";

        // physical constants
        gravity / set_gravity: [f64; 3] = "gravitational acceleration";
        wind / set_wind: [f64; 3] = "wind (for lift, drag and viscosity)";
        magnetic / set_magnetic: [f64; 3] = "global magnetic flux";
        density / set_density: f64 = "density of medium";
        viscosity / set_viscosity: f64 = "viscosity of medium";

        // override contact solver parameters (if enabled)
        o_margin / set_o_margin: f64 = "override contact solver's margin";
        o_solref / set_o_solref: [f64; 2] = "override contact solver's solref";
        o_solimp / set_o_solimp: [f64; 5] = "override contact solver's solimp";
        o_friction / set_o_friction: [f64; 5] = "override contact solver's friction";

        // discrete parameters
        iterations / set_iterations: usize = "maximum number of main solver iterations";
        ls_iterations / set_ls_iterations: usize = "maximum number of CG/Newton linesearch iterations";
        noslip_iterations / set_noslip_iterations: usize = "maximum number of noslip solver iterations";
        ccd_iterations / set_ccd_iterations: usize = "maximum number of convex collision solver iterations";
    }

    enums {
        integrator / set_integrator: mjtIntegrator = "integration mode (mjtIntegrator)";
        cone / set_cone: mjtCone = "type of friction cone (mjtCone)";
        jacobian / set_jacobian: mjtJacobian = "type of Jacobian (mjtJacobian)";
        solver / set_solver: mjtSolver = "solver algorithm (mjtSolver)";
        disableflags / set_disableflags: mjtDisableBit = "bit flags for disabling standard features";
        enableflags / set_enableflags: mjtEnableBit = "bit flags for enabling optional features";
        disableactuator / set_disableactuator: mjtDisableBit = "bit flags for disabling actuators by group id";
    }
});

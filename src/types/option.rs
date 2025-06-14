pub struct MjOption(pub(super) crate::bindgen::mjOption);

macro_rules! impl_parameters {
    ($($name:ident / $set_name:ident: $T:ty = $description:literal;)*) => {
        impl MjOption {
            $(
                #[doc = $description]
                pub fn $name(&self) -> $T {
                    self.0.$name
                }
                #[doc = "set "]
                #[doc = $description]
                pub fn $set_name(&mut self, value: $T) -> &mut Self {
                    self.0.$name = value;
                    self
                }
            )*
        }
    };
}
impl_parameters! {
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
}

macro_rules! impl_enum_settings {
    ($($name:ident / $set_name:ident: $T:ty = $description:literal;)*) => {
        impl MjOption {
            $(
                #[doc = $description]
                pub fn $name(&self) -> $T {
                    unsafe {std::mem::transmute(self.0.$name)}
                }
                #[doc = "set "]
                #[doc = $description]
                pub fn $set_name(&mut self, value: $T) -> &mut Self {
                    self.0.$name = value as i32;
                    self
                }
            )*
        }
    };
}
impl_enum_settings! {
    integrator / set_integrator: crate::bindgen::mjtIntegrator = "integration mode (mjtIntegrator)";
    cone / set_cone: crate::bindgen::mjtCone = "type of friction cone (mjtCone)";
    jacobian / set_jacobian: crate::bindgen::mjtJacobian = "type of Jacobian (mjtJacobian)";
    solver / set_solver: crate::bindgen::mjtSolver = "solver algorithm (mjtSolver)";
    disableflags / set_disableflags: crate::bindgen::mjtDisableBit = "bit flags for disabling standard features";
    enableflags / set_enableflags: crate::bindgen::mjtEnableBit = "bit flags for enabling optional features";
    disableactuator / set_disableactuator: crate::bindgen::mjtDisableBit = "bit flags for disabling actuators by group id";
}

macro_rules! impl_int_settings {
    ($($name:ident / $set_name:ident: usize = $description:literal;)*) => {
        impl MjOption {
            $(
                #[doc = $description]
                pub fn $name(&self) -> usize {
                    self.0.$name as usize
                }
                #[doc = "set "]
                #[doc = $description]
                pub fn $set_name(&mut self, value: usize) -> &mut Self {
                    self.0.$name = value as i32;
                    self
                }
            )*
        }
    };
}
impl_int_settings! {
    iterations / set_iterations: usize = "maximum number of main solver iterations";
    ls_iterations / set_ls_iterations: usize = "maximum number of CG/Newton linesearch iterations";
    noslip_iterations / set_noslip_iterations: usize = "maximum number of noslip solver iterations";
    ccd_iterations / set_ccd_iterations: usize = "maximum number of convex collision solver iterations";
}

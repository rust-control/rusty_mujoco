use crate::bindgen::mjStatistic;

macro_rules! impl_statistic_fields {
    ($($name:ident / $set_name:ident: $T:ty = $description:literal;)*) => {
        impl mjStatistic {
            $(
                #[doc = $description]
                pub fn $name(&self) -> $T {
                    self.$name
                }
                #[doc = "set "]
                #[doc = $description]
                pub fn $set_name(&mut self, value: $T) -> &mut Self {
                    self.$name = value;
                    self
                }
            )*
        }
    };
}
impl_statistic_fields! {
    meaninertia / set_meaninertia: f64 = "mean diagonal inertia of bodies";
    meanmass / set_meanmass: f64 = "mean mass of bodies";
    meansize / set_meansize: f64 = "mean size of bodies";
    extent / set_extent: f64 = "spatial extent of model";
    center / set_center: [f64; 3] = "center of model in world coordinates";
}

macro_rules! impl_getters {
    ($Stat:ty { $( $name:ident: $T:ty = $description:literal; )* }) => {
        impl $Stat {
            $(
                #[doc = $description]
                pub fn $name(&self) -> $T {
                    self.0.$name as $T
                }
            )*
        }
    };
}

/// This is the data structure holding information about one warning type. mjData.warning is a preallocated array of mjWarningStat data structures, one for each warning type.
pub struct MjWarningStat(pub(super) crate::bindgen::mjWarningStat);
impl_getters!(MjWarningStat{
    lastinfo: usize = "info from last warning";
    number: usize = "how many times was warning raised";
});

/// This is the data structure holding information about one timer. mjData.timer is a preallocated array of mjTimerStat data structures, one for each timer type.
pub struct MjTimerStat(pub(super) crate::bindgen::mjTimerStat);
impl_getters!(MjTimerStat {
    duration: f64 = "cumulative duration";
    number: usize = "how many times was timer called";
});

/// This is the data structure holding information about one solver iteration. mjData.solver is a preallocated array of mjSolverStat data structures, one for each iteration of the solver, up to a maximum of mjNSOLVER. The actual number of solver iterations is given by mjData.solver_niter.
pub struct MjSolverStat(pub(super) crate::bindgen::mjSolverStat);
impl_getters!(MjSolverStat {
    improvement: f64 = "cost reduction, scaled by 1/trace(M(qpos0))";
    gradient: f64 = "gradient norm (primal only, scaled)";
    lineslope: f64 = "slope in linesearch";
    nactive: usize = "number of active constraints";
    nchange: usize = "number of constraint state changes";
    neval: usize = "number of cost evaluations in line search";
    nupdate: usize = "number of Cholesky updates in line search";
});

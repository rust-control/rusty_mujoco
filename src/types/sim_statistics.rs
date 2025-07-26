//! # [Sim statistics](https://mujoco.readthedocs.io/en/latest/api.html#sim-statistics)
//! 
//! These structs are all embedded in [`mjData`](crate::mjData),
//! and collect simulation-related statistics.

pub use crate::bindgen::{mjWarningStat, mjTimerStat, mjSolverStat};

derive_fields_mapping!(mjWarningStat {
    scalars {
        lastinfo: i32 = "info from last warning";
        number: usize = "how many times was warning raised";
    }
});

derive_fields_mapping!(mjTimerStat {
    scalars {
        duration: f64 = "cumulative duration";
        number: usize = "how many times was timer called";
    }
});

derive_fields_mapping!(mjSolverStat {
    scalars {
        improvement: f64 = "cost reduction, scaled by 1/trace(M(qpos0))";
        gradient: f64 = "gradient norm (primal only, scaled)";
        lineslope: f64 = "slope in linesearch";
        nactive: usize = "number of active constraints";
        nchange: usize = "number of constraint state changes";
        neval: usize = "number of cost evaluations in line search";
        nupdate: usize = "number of Cholesky updates in line search";
    }
});

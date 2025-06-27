/*
struct mjLROpt_ {                 // options for mj_setLengthRange()
  // flags
  int mode;                       // which actuators to process (mjtLRMode)
  int useexisting;                // use existing length range if available
  int uselimit;                   // use joint and tendon limits if available

  // algorithm parameters
  mjtNum accel;                   // target acceleration used to compute force
  mjtNum maxforce;                // maximum force; 0: no limit
  mjtNum timeconst;               // time constant for velocity reduction; min 0.01
  mjtNum timestep;                // simulation timestep; 0: use mjOption.timestep
  mjtNum inttotal;                // total simulation time interval
  mjtNum interval;                // evaluation time interval (at the end)
  mjtNum tolrange;                // convergence tolerance (relative to range)
};
typedef struct mjLROpt_ mjLROpt;
*/
wrapper! {
    /// Options for configuring the automatic
    /// [actuator length-range computation](https://mujoco.readthedocs.io/en/stable/modeling.html#clengthrange).
    MjLrOpt of crate::bindgen::mjLROpt
}

impl MjLrOpt {
    /// which actuators to process
    pub fn mode(&self) -> crate::bindgen::mjtLRMode {
        // SAFETY: enum discriminator
        unsafe { std::mem::transmute(self.0.mode) }
    }
    /// Set which actuators to process
    pub fn set_mode(&mut self, value: crate::bindgen::mjtLRMode) -> &mut Self {
        self.0.mode = value.0 as i32;
        self
    }

    /// if use existing length range if available
    pub fn useexisting(&self) -> bool {
        self.0.useexisting != 0
    }
    /// Set if use existing length range if available
    pub fn set_useexisting(&mut self, value: bool) -> &mut Self {
        self.0.useexisting = value as i32;
        self
    }

    /// if use joint and tendon limits if available
    pub fn uselimit(&self) -> bool {
        self.0.uselimit != 0
    }
    /// Set if use joint and tendon limits if available
    pub fn set_uselimit(&mut self, value: bool) -> &mut Self {
        self.0.uselimit = value as i32;
        self
    }
}

impl MjLrOpt {
    /// target acceleration used to compute force
    pub fn accel(&self) -> f64 {
        self.0.accel
    }
    /// Set target acceleration used to compute force
    pub fn set_accel(&mut self, value: f64) -> &mut Self {
        self.0.accel = value;
        self
    }

    /// maximum force; 0: no limit
    pub fn maxforce(&self) -> f64 {
        self.0.maxforce
    }
    /// Set maximum force; 0: no limit
    pub fn set_maxforce(&mut self, value: f64) -> &mut Self {
        self.0.maxforce = value;
        self
    }

    /// time constant for velocity reduction; min 0.01
    pub fn timeconst(&self) -> f64 {
        self.0.timeconst
    }
    /// Set time constant for velocity reduction; min 0.01
    pub fn set_timeconst(&mut self, value: f64) -> &mut Self {
        self.0.timeconst = value;
        self
    }

    /// simulation timestep; 0: use mjOption.timestep
    pub fn timestep(&self) -> f64 {
        self.0.timestep
    }
    /// Set simulation timestep; 0: use mjOption.timestep
    pub fn set_timestep(&mut self, value: f64) -> &mut Self {
        self.0.timestep = value;
        self
    }

    /// total simulation time interval
    pub fn inttotal(&self) -> f64 {
        self.0.inttotal
    }
    /// Set total simulation time interval
    pub fn set_inttotal(&mut self, value: f64) -> &mut Self {
        self.0.inttotal = value;
        self
    }

    /// evaluation time interval (at the end)
    pub fn interval(&self) -> f64 {
        self.0.interval
    }
    /// Set evaluation time interval (at the end)
    pub fn set_interval(&mut self, value: f64) -> &mut Self {
        self.0.interval = value;
        self
    }

    /// convergence tolerance (relative to range)
    pub fn tolrange(&self) -> f64 {
        self.0.tolrange
    }
    /// Set convergence tolerance (relative to range
    pub fn set_tolrange(&mut self, value: f64) -> &mut Self {
        self.0.tolrange = value;
        self
    }
}

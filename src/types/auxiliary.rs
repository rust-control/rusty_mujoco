//! # [Auxiliary](https://mujoco.readthedocs.io/en/stable/APIreference/APItypes.html#auxiliary)
//! 
//! hese struct types are used in the engine and their names are prefixed
//! with `mj`.
//! 
//! [`mjVisual`] and [`mjStatistic`] are embedded in [`mjModel`](crate::mjModel),
//! [`mjContact`] is embedded in [`mjData`](crate::mjData).

pub use crate::bindgen::{
    mjtLRMode,
    mjNREF, mjNIMP,
};

use crate::{obj, ObjectId, VertexId, ElementId};

pub use crate::bindgen::mjVisual;
impl Default for mjVisual {
    fn default() -> Self {
        crate::mj_defaultVisual()
    }
}
fields_mapping!(mjVisual {
    structs {
        global / global_mut: crate::bindgen::mjVisual___bindgen_ty_1 = "global parameters";
        quality / quality_mut: crate::bindgen::mjVisual___bindgen_ty_2 = "rendering quality";
        headlight / headlight_mut: crate::bindgen::mjVisual___bindgen_ty_3 = "head light";
        map / map_mut: crate::bindgen::mjVisual___bindgen_ty_4 = "mapping";
        scale / scale_mut: crate::bindgen::mjVisual___bindgen_ty_5 = "scale of decor elements relative to mean body size";
        rgba / rgba_mut: crate::bindgen::mjVisual___bindgen_ty_6 = "color of decor elements";
    }
});
fields_mapping!(crate::bindgen::mjVisual___bindgen_ty_1 {
    scalars {
        fovy / set_fovy: f32 = "y field-of-view of free camera (orthographic ? length : degree)";
        ipd / set_ipd: f32 = "inter-pupilary distance for free camera";
        azimuth / set_azimuth: f32 = "initial azimuth of free camera (degrees)";
        elevation / set_elevation: f32 = "initial elevation of free camera (degrees)";
        linewidth / set_linewidth: f32 = "line width for wireframe and ray rendering";
        glow / set_glow: f32 = "glow coefficient for selected body";
        realtime / set_realtime: f32 = "initial real-time factor (1: real time)";
        offwidth / set_offwidth: i32 = "width of offscreen buffer";
        offheight / set_offheight: i32 = "height of offscreen buffer";
        ellipsoidinertia / set_ellipsoidinertia: i32 = "geom for inertia visualization (0: box, 1: ellipsoid)";
    }
});
impl crate::bindgen::mjVisual___bindgen_ty_1 {
    /// is the free camera orthographic
    pub fn orthographic(&self) -> bool {self.orthographic != 0}
    /// set if the free camera is orthographic
    pub fn set_orthographic(&mut self, value: bool) -> &mut Self {
        self.orthographic = value as i32;
        self
    }
    /// visualize active bounding volumes
    pub fn bvactive(&self) -> bool {self.bvactive != 0}
    /// set if visualize active bounding volumes
    pub fn set_bvactive(&mut self, value: bool) -> &mut Self {
        self.bvactive = value as i32;
        self
    }
}
fields_mapping!(crate::bindgen::mjVisual___bindgen_ty_2 {
    scalars {
        shadowsize / set_shadowsize: usize = "size of shadowmap texture";
        offsamples / set_offsamples: usize = "number of multisamples for offscreen rendering";
        numslices / set_numslices: usize = "number of slices for builtin geom drawing";
        numstacks / set_numstacks: usize = "number of stacks for builtin geom drawing";
        numquads / set_numquads: usize = "number of quads for box rendering";
    }
});
fields_mapping!(crate::bindgen::mjVisual___bindgen_ty_3 {
    scalars {
        ambient / set_ambient: [f32; 3] = "ambient rgb (alpha=1)";
        diffuse / set_diffuse: [f32; 3] = "diffuse rgb (alpha=1)";
        specular / set_specular: [f32; 3] = "specular rgb (alpha=1)";
    }
});
impl crate::bindgen::mjVisual___bindgen_ty_3 {
    /// is headlight active
    pub fn active(&self) -> bool {self.active != 0}
    /// set if headlight is active
    pub fn set_active(&mut self, value: bool) -> &mut Self {
        self.active = value as i32;
        self
    }
}
fields_mapping!(crate::bindgen::mjVisual___bindgen_ty_4 {
    scalars {
        stiffness / set_stiffness: f32 = "mouse perturbation stiffness (space->force)";
        stiffnessrot / set_stiffnessrot: f32 = "mouse perturbation stiffness (space->torque)";
        force / set_force: f32 = "from force units to space units";
        torque / set_torque: f32 = "from torque units to space units";
        alpha / set_alpha: f32 = "scale geom alphas when transparency is enabled";
        fogstart / set_fogstart: f32 = "OpenGL fog starts at fogstart * mjModel.stat.extent";
        fogend / set_fogend: f32 = "OpenGL fog ends at fogend * mjModel.stat.extent";
        znear / set_znear: f32 = "near clipping plane = znear * mjModel.stat.extent";
        zfar / set_zfar: f32 = "far clipping plane = zfar * mjModel.stat.extent";
        haze / set_haze: f32 = "haze ratio";
        shadowclip / set_shadowclip: f32 = "directional light: shadowclip * mjModel.stat.extent";
        shadowscale / set_shadowscale: f32 = "spot light: shadowscale * light.cutoff";
        actuatortendon / set_actuatortendon: f32 = "scale tendon width";
    }
});
fields_mapping!(crate::bindgen::mjVisual___bindgen_ty_5 {
    scalars {
        forcewidth / set_forcewidth: f32 = "width of force arrow";
        contactwidth / set_contactwidth: f32 = "contact width";
        contactheight / set_contactheight: f32 = "contact height";
        connect / set_connect: f32 = "autoconnect capsule width";
        com / set_com: f32 = "com radius";
        camera / set_camera: f32 = "camera object";
        light / set_light: f32 = "light object";
        selectpoint / set_selectpoint: f32 = "selection point";
        jointlength / set_jointlength: f32 = "joint length";
        jointwidth / set_jointwidth: f32 = "joint width";
        actuatorlength / set_actuatorlength: f32 = "actuator length";
        actuatorwidth / set_actuatorwidth: f32 = "actuator width";
        framelength / set_framelength: f32 = "bodyframe axis length";
        framewidth / set_framewidth: f32 = "bodyframe axis width";
        constraint / set_constraint: f32 = "constraint width";
        slidercrank / set_slidercrank: f32 = "slidercrank width";
        frustum / set_frustum: f32 = "frustum zfar plane";
    }
});
fields_mapping!(crate::bindgen::mjVisual___bindgen_ty_6 {
    scalars {
        fog / set_fog: [f32; 4] = "fog";
        haze / set_haze: [f32; 4] = "haze";
        force / set_force: [f32; 4] = "external force";
        inertia / set_inertia: [f32; 4] = "inertia box";
        joint / set_joint: [f32; 4] = "joint";
        actuator / set_actuator: [f32; 4] = "actuator, neutral";
        actuatornegative / set_actuatornegative: [f32; 4] = "actuator, negative limit";
        actuatorpositive / set_actuatorpositive: [f32; 4] = "actuator, positive limit";
        com / set_com: [f32; 4] = "center of mass";
        camera / set_camera: [f32; 4] = "camera object";
        light / set_light: [f32; 4] = "light object";
        selectpoint / set_selectpoint: [f32; 4] = "selection point";
        connect / set_connect: [f32; 4] = "auto connect";
        contactpoint / set_contactpoint: [f32; 4] = "contact point";
        contactforce / set_contactforce: [f32; 4] = "contact force";
        contactfriction / set_contactfriction: [f32; 4] = "contact friction force";
        contacttorque / set_contacttorque: [f32; 4] = "contact torque";
        contactgap / set_contactgap: [f32; 4] = "contact point in gap";
        rangefinder / set_rangefinder: [f32; 4] = "rangefinder ray";
        constraint / set_constraint: [f32; 4] = "constraint";
        slidercrank / set_slidercrank: [f32; 4] = "slidercrank";
        crankbroken / set_crankbroken: [f32; 4] = "used when crank must be stretched/broken";
        frustum / set_frustum: [f32; 4] = "camera frustum";
        bv / set_bv: [f32; 4] = "bounding volume";
    }
});

pub use crate::bindgen::mjStatistic;
fields_mapping!(mjStatistic {
    scalars {
        meaninertia / set_meaninertia: f64 = "mean diagonal inertia";
        meanmass / set_meanmass: f64 = "mean body mass";
        extent / set_extent: f64 = "spatial extent";
        center / set_center: [f64; 3] = "center of model";
    }
});

pub use crate::bindgen::mjContact;
fields_mapping!(mjContact {
    scalars {
        // contact parameters set by near-phase collision function
        dist: f64 = "distance between nearest points; negative means penetration";
        pos: [f64; 3] = "position of contact point: midpoint between geoms";
        frame: [f64; 9] = "normal is in [0-2], points from geom[0] to geom[1]";
        
        // contact parameters set by `mj_collideGeoms`
        includemargin: f64 = "include if dist < includemargin = margin - gap";
        friction: [f64; 5] = "tangent1, 2, spin, roll1, 2";
        solref: [f64; 2] = "constraint solver reference, normal direction";
        solreffriction: [f64; mjNREF] = "constraint solver reference, friction directions";
        solimp: [f64; mjNIMP] = "constraint solver impedance";

        // internal storage used by solver
        mu: f64 = "friction of regularized cone, set by mj_makeConstraint";
        H: [f64; 36] = "cone Hessian, set by mj_constraintUpdate";
    }
});
impl mjContact {
    /// contact space dimensionality: 1, 3, 4 or 6
    pub fn dim(&self) -> usize {self.dim as usize}
    
    /// geom ids; `None` for flex
    pub fn geom(&self) -> (Option<ObjectId<obj::Geom>>, Option<ObjectId<obj::Geom>>) {
        (
            if self.geom[0] >= 0 {Some(unsafe { ObjectId::new_unchecked(self.geom[0] as usize) })} else {None},
            if self.geom[1] >= 0 {Some(unsafe { ObjectId::new_unchecked(self.geom[1] as usize) })} else {None}
        )
    }
    /// flex ids; `None` for geom
    pub fn flex(&self) -> (Option<ObjectId<obj::Flex>>, Option<ObjectId<obj::Flex>>) {
        (
            if self.flex[0] >= 0 {Some(unsafe { ObjectId::new_unchecked(self.flex[0] as usize) })} else {None},
            if self.flex[1] >= 0 {Some(unsafe { ObjectId::new_unchecked(self.flex[1] as usize) })} else {None}
        )
    }

    /// element ids; `None` for geom or flex vertex
    pub fn elem(&self) -> (Option<ElementId<obj::Mesh>>, Option<ElementId<obj::Mesh>>) {
        (
            if self.elem[0] >= 0 {Some(unsafe { ElementId::<obj::Mesh>::new_unchecked(self.elem[0] as usize) })} else {None},
            if self.elem[1] >= 0 {Some(unsafe { ElementId::<obj::Mesh>::new_unchecked(self.elem[1] as usize) })} else {None}
        )
    }
    /// vertex ids; `None` for geom or flex element
    pub fn vert(&self) -> (Option<VertexId<obj::Mesh>>, Option<VertexId<obj::Mesh>>) {
        (
            if self.vert[0] >= 0 {Some(unsafe { VertexId::<obj::Mesh>::new_unchecked(self.vert[0] as usize) })} else {None},
            if self.vert[1] >= 0 {Some(unsafe { VertexId::<obj::Mesh>::new_unchecked(self.vert[1] as usize) })} else {None}
        )
    }
}
pub enum ContactExclude {
    Include,
    InGap,
    Fused,
    NoDofs,
}
impl mjContact {
    /// contact exclusion flag
    pub fn exclude(&self) -> ContactExclude {
        match self.exclude {
            0 => ContactExclude::Include,
            1 => ContactExclude::InGap,
            2 => ContactExclude::Fused,
            3 => ContactExclude::NoDofs,
            _ => panic!("Invalid contact exclusion flag: {}", self.exclude),
        }
    }

    /// address in efc; `None` means not included
    pub fn efc_address(&self) -> Option<usize> {
        if self.efc_address >= 0 {Some(self.efc_address as usize)} else {None}
    }
}

pub use crate::bindgen::mjLROpt;
impl Default for mjLROpt {
    fn default() -> Self {
        crate::mj_defaultLROpt()
    }
}
fields_mapping!(mjLROpt {
    scalars {
        accel / set_accel: f64 = "target acceleration used to compute force";
        maxforce / set_maxforce: f64 = "maximum force; 0: no limit";
        timeconst / set_timeconst: f64 = "time constant for velocity reduction; min 0.01";
        timestep / set_timestep: f64 = "simulation timestep; 0: use mjOption.timestep";
        inttotal / set_inttotal: f64 = "total simulation time interval";
        interval / set_interval: f64 = "evaluation time interval (at the end)";
        tolrange / set_tolrange: f64 = "convergence tolerance (relative to range)";
    }
    enums {
        mode / set_mode: mjtLRMode = "which actuators to process";
    }
});
impl mjLROpt {
    /// use existing length range if available
    pub fn useexisting(&self) -> bool {self.useexisting != 0}
    /// Set if use existing length range if available
    pub fn set_useexisting(&mut self, value: bool) -> &mut Self {
        self.useexisting = value as i32; self
    }

    /// use joint and tendon limits if available
    pub fn uselimit(&self) -> bool {self.uselimit != 0}
    /// Set if use joint and tendon limits if available
    pub fn set_uselimit(&mut self, value: bool) -> &mut Self {
        self.uselimit = value as i32; self
    }
}

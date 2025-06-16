newtype!(MjVisual of crate::bindgen::mjVisual);

macro_rules! visual_category {
    ($name:ident / $update_name:ident : $Category:ident ($bindgen_t:ty) {
        $($field:ident / $set_field:ident : $T:ty = $description:literal;)*
    } = $category_description:literal) => {
        impl MjVisual {
            #[doc = $category_description]
            pub fn $name(&self) -> $Category {
                $Category(self.0.$name)
            }
            #[doc = "update "]
            #[doc = $category_description]
            pub fn $update_name(&mut self, f: impl FnOnce(&mut $Category) -> &mut $Category) -> &mut Self {
                let mut category = self.$name();
                f(&mut category);
                self.0.$name = category.0;
                self
            }
        }

        #[doc = $category_description]
        pub struct $Category($bindgen_t);
        impl $Category {
            $(
                #[doc = $description]
                pub fn $field(&self) -> $T {
                    self.0.$field
                }
                #[doc = "set "]
                #[doc = $description]
                pub fn $set_field(&mut self, value: $T) -> &mut Self {
                    self.0.$field = value;
                    self
                }
            )*
        }
    };
}

visual_category!(global / update_global: MjVisualGlobal(crate::bindgen::mjVisual___bindgen_ty_1) {
    orthographic / set_orthographic: i32 = "is the free camera orthographic (0: no, 1: yes)";
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
    bvactive / set_bvactive: i32 = "visualize active bounding volumes (0: no, 1: yes)";
} = "global parameters");

visual_category!(quality / update_quality: MjVisualQuality(crate::bindgen::mjVisual___bindgen_ty_2) {
    shadowsize / set_shadowsize: i32 = "size of shadowmap texture";
    offsamples / set_offsamples: i32 = "number of multisamples for offscreen rendering";
    numslices / set_numslices: i32 = "number of slices for builtin geom drawing";
    numstacks / set_numstacks: i32 = "number of stacks for builtin geom drawing";
    numquads / set_numquads: i32 = "number of quads for box rendering";
} = "rendering quality");

visual_category!(headlight / update_headlight: MjVisualHeadlight(crate::bindgen::mjVisual___bindgen_ty_3) {
    ambient / set_ambient: [f32; 3] = "ambient rgb (alpha=1)";
    diffuse / set_diffuse: [f32; 3] = "diffuse rgb (alpha=1)";
    specular / set_specular: [f32; 3] = "specular rgb (alpha=1)";
    active / set_active: i32 = "is headlight active";
} = "head light");

visual_category!(map / update_map: MjVisualMap(crate::bindgen::mjVisual___bindgen_ty_4) {
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
} = "mapping");

visual_category!(scale / update_scale: MjVisualScale(crate::bindgen::mjVisual___bindgen_ty_5) {
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
} = "scale of decor elements relative to mean body size");

visual_category!(rgba / update_rgba: MjVisualRgba(crate::bindgen::mjVisual___bindgen_ty_6) {
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
    bvactive / set_bvactive: [f32; 4] = "active bounding volume";
} = "color of decor elements");

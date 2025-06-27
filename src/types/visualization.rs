//! # [Visualization]((https://mujoco.readthedocs.io/en/stable/APIreference/APItypes.html#visualisation))

use crate::{SegmentationId, ObjectId, mjtObj, obj};
pub use crate::bindgen::{
    mjNGROUP,
    mjMAXLINE,
    mjMAXLINEPNT,
    mjNVISFLAG,
    mjNRNDFLAG,
};

wrapper! {
    /// This is the data structure holding information about mouse perturbations.
    MjvPerturb of crate::bindgen::mjvPerturb
    /*
    struct mjvPerturb_ {              // object selection and perturbation
      int      select;                // selected body id; non-positive: none
      int      flexselect;            // selected flex id; negative: none
      int      skinselect;            // selected skin id; negative: none
      int      active;                // perturbation bitmask (mjtPertBit)
      int      active2;               // secondary perturbation bitmask (mjtPertBit)
      mjtNum   refpos[3];             // reference position for selected object
      mjtNum   refquat[4];            // reference orientation for selected object
      mjtNum   refselpos[3];          // reference position for selection point
      mjtNum   localpos[3];           // selection point in object coordinates
      mjtNum   localmass;             // spatial inertia at selection point
      mjtNum   scale;                 // relative mouse motion-to-space scaling (set by initPerturb)
    };
    typedef struct mjvPerturb_ mjvPerturb;
    */
}
impl MjvPerturb {
    /// selected body id
    pub fn select(&self) -> Option<ObjectId<obj::Body>> {
        if self.0.select <= 0 {
            None
        } else {
            Some(ObjectId::new(self.0.select as usize))
        }
    }
    /// set selected body id
    pub fn set_select(&mut self, select: Option<ObjectId<obj::Body>>) {
        self.0.select = select.map_or(-1, |id| id.index() as i32);
    }

    /// selected flex id
    pub fn flexselect(&self) -> Option<ObjectId<obj::Flex>> {
        if self.0.flexselect < 0 {
            None
        } else {
            Some(ObjectId::new(self.0.flexselect as usize))
        }
    }
    /// set selected flex id
    pub fn set_flexselect(&mut self, flexselect: Option<ObjectId<obj::Flex>>) {
        self.0.flexselect = flexselect.map_or(-1, |id| id.index() as i32);
    }

    /// selected skin id
    pub fn skinselect(&self) -> Option<ObjectId<obj::Skin>> {
        if self.0.skinselect < 0 {
            None
        } else {
            Some(ObjectId::new(self.0.skinselect as usize))
        }
    }
    /// set selected skin id
    pub fn set_skinselect(&mut self, skinselect: Option<ObjectId<obj::Skin>>) {
        self.0.skinselect = skinselect.map_or(-1, |id| id.index() as i32);
    }

    /// perturbation bitmask
    pub fn active(&self) -> crate::bindgen::mjtPertBit {
        // SAFETY: `mjtPertBit` is a bitmask, so it can be safely casted from `i32`
        unsafe { std::mem::transmute(self.0.active) }
    }
    /// set perturbation bitmask
    pub fn set_active(&mut self, active: crate::bindgen::mjtPertBit) {
        self.0.active = active.0 as i32;
    }

    /// secondary perturbation bitmask
    pub fn active2(&self) -> crate::bindgen::mjtPertBit {
        // SAFETY: `mjtPertBit` is a bitmask, so it can be safely casted from `i32`
        unsafe { std::mem::transmute(self.0.active2) }
    }
    /// set secondary perturbation bitmask
    pub fn set_active2(&mut self, active2: crate::bindgen::mjtPertBit) {
        self.0.active2 = active2.0 as i32;
    }

    /// selection point in object coordinates
    pub fn refpos(&self) -> [f64; 3] {
        self.0.refpos
    }
    /// set selection point in object coordinates
    pub fn set_refpos(&mut self, refpos: [f64; 3]) {
        self.0.refpos = refpos;
    }

    /// reference orientation for selected object
    pub fn refquat(&self) -> [f64; 4] {
        self.0.refquat
    }
    /// set reference orientation for selected object
    pub fn set_refquat(&mut self, refquat: [f64; 4]) {
        self.0.refquat = refquat;
    }

    /// reference position for selection point
    pub fn refselpos(&self) -> [f64; 3] {
        self.0.refselpos
    }
    /// set reference position for selection point
    pub fn set_refselpos(&mut self, refselpos: [f64; 3]) {
        self.0.refselpos = refselpos;
    }

    /// selection point in object coordinates
    pub fn localpos(&self) -> [f64; 3] {
        self.0.localpos
    }
    /// set selection point in object coordinates
    pub fn set_localpos(&mut self, localpos: [f64; 3]) {
        self.0.localpos = localpos;
    }

    /// spatial inertia at selection point
    pub fn localmass(&self) -> f64 {
        self.0.localmass
    }
    /// set spatial inertia at selection point
    pub fn set_localmass(&mut self, localmass: f64) {
        self.0.localmass = localmass;
    }

    /// relative mouse motion-to-space scaling
    pub fn scale(&self) -> f64 {
        self.0.scale
    }
    /// set relative mouse motion-to-space scaling
    pub fn set_scale(&mut self, scale: f64) {
        self.0.scale = scale;
    }
}

wrapper! {
    /// This is the data structure describing one abstract camera.
    MjvCamera of crate::bindgen::mjvCamera
    /*
    struct mjvCamera_ {               // abstract camera
      // type and ids
      int      type;                  // camera type (mjtCamera)
      int      fixedcamid;            // fixed camera id
      int      trackbodyid;           // body id to track

      // abstract camera pose specification
      mjtNum   lookat[3];             // lookat point
      mjtNum   distance;              // distance to lookat point or tracked body
      mjtNum   azimuth;               // camera azimuth (deg)
      mjtNum   elevation;             // camera elevation (deg)

      // orthographic / perspective
      int      orthographic;          // 0: perspective; 1: orthographic
    };
    typedef struct mjvCamera_ mjvCamera;
    */
}
impl MjvCamera {
    /// camera type
    pub fn type_(&self) -> crate::bindgen::mjtCamera {
        // SAFETY: `mjtCamera` is a bitmask, so it can be safely casted from `i32`
        unsafe { std::mem::transmute(self.0.type_) }
    }
    /// set camera type
    pub fn set_type(&mut self, type_: crate::bindgen::mjtCamera) {
        self.0.type_ = type_.0 as i32;
    }

    /// fixed camera id
    pub fn fixedcamid(&self) -> Option<ObjectId<obj::Camera>> {
        if self.0.fixedcamid < 0 {
            None
        } else {
            Some(ObjectId::new(self.0.fixedcamid as usize))
        }
    }
    /// set fixed camera id
    pub fn set_fixedcamid(&mut self, fixedcamid: Option<ObjectId<obj::Camera>>) {
        self.0.fixedcamid = fixedcamid.map_or(-1, |id| id.index() as i32);
    }

    /// body id to track
    pub fn trackbodyid(&self) -> Option<ObjectId<obj::Body>> {
        if self.0.trackbodyid < 0 {
            None
        } else {
            Some(ObjectId::new(self.0.trackbodyid as usize))
        }
    }
    /// set body id to track
    pub fn set_trackbodyid(&mut self, trackbodyid: Option<ObjectId<obj::Body>>) {
        self.0.trackbodyid = trackbodyid.map_or(-1, |id| id.index() as i32);
    }

    /// lookat point
    pub fn lookat(&self) -> [f64; 3] {
        self.0.lookat
    }
    /// set lookat point
    pub fn set_lookat(&mut self, lookat: [f64; 3]) {
        self.0.lookat = lookat;
    }

    /// distance to lookat point or tracked body
    pub fn distance(&self) -> f64 {
        self.0.distance
    }
    /// set distance to lookat point or tracked body
    pub fn set_distance(&mut self, distance: f64) {
        self.0.distance = distance;
    }

    /// camera azimuth (deg)
    pub fn azimuth(&self) -> f64 {
        self.0.azimuth
    }
    /// set camera azimuth (deg)
    pub fn set_azimuth(&mut self, azimuth: f64) {
        self.0.azimuth = azimuth;
    }

    /// camera elevation (deg)
    pub fn elevation(&self) -> f64 {
        self.0.elevation
    }
    /// set camera elevation (deg)
    pub fn set_elevation(&mut self, elevation: f64) {
        self.0.elevation = elevation;
    }

    /// if orthographic or perspective
    pub fn orthographic(&self) -> bool {
        self.0.orthographic != 0
    }
    /// set if orthographic or perspective
    pub fn set_orthographic(&mut self, orthographic: bool) {
        self.0.orthographic = orthographic as i32;
    }
}

wrapper! {
    /// This is the data structure describing one OpenGL camera.
    MjvGlCamera of crate::bindgen::mjvGLCamera
    /*
    struct mjvGLCamera_ {             // OpenGL camera
      // camera frame
      float    pos[3];                // position
      float    forward[3];            // forward direction
      float    up[3];                 // up direction

      // camera projection
      float    frustum_center;        // hor. center (left,right set to match aspect)
      float    frustum_width;         // width (not used for rendering)
      float    frustum_bottom;        // bottom
      float    frustum_top;           // top
      float    frustum_near;          // near
      float    frustum_far;           // far

      // orthographic / perspective
      int      orthographic;          // 0: perspective; 1: orthographic
    };
    typedef struct mjvGLCamera_ mjvGLCamera;
    */
}
impl MjvGlCamera {
    /// position
    pub fn pos(&self) -> [f32; 3] {
        self.0.pos
    }
    /// set position
    pub fn set_pos(&mut self, pos: [f32; 3]) {
        self.0.pos = pos;
    }

    /// forward direction
    pub fn forward(&self) -> [f32; 3] {
        self.0.forward
    }
    /// set forward direction
    pub fn set_forward(&mut self, forward: [f32; 3]) {
        self.0.forward = forward;
    }

    /// up direction
    pub fn up(&self) -> [f32; 3] {
        self.0.up
    }
    /// set up direction
    pub fn set_up(&mut self, up: [f32; 3]) {
        self.0.up = up;
    }

    /// horizontal center (left,right set to match aspect)
    pub fn frustum_center(&self) -> f32 {
        self.0.frustum_center
    }
    /// set horizontal center (left,right set to match aspect)
    pub fn set_frustum_center(&mut self, frustum_center: f32) {
        self.0.frustum_center = frustum_center;
    }

    /// width (not used for rendering)
    pub fn frustum_width(&self) -> f32 {
        self.0.frustum_width
    }
    /// set width (not used for rendering)
    pub fn set_frustum_width(&mut self, frustum_width: f32) {
        self.0.frustum_width = frustum_width;
    }

    /// bottom
    pub fn frustum_bottom(&self) -> f32 {
        self.0.frustum_bottom
    }
    /// set bottom
    pub fn set_frustum_bottom(&mut self, frustum_bottom: f32) {
        self.0.frustum_bottom = frustum_bottom;
    }

    /// top
    pub fn frustum_top(&self) -> f32 {
        self.0.frustum_top
    }
    /// set top
    pub fn set_frustum_top(&mut self, frustum_top: f32) {
        self.0.frustum_top = frustum_top;
    }

    /// near plane distance
    pub fn frustum_near(&self) -> f32 {
        self.0.frustum_near
    }
    /// set near plane distance
    pub fn set_frustum_near(&mut self, frustum_near: f32) {
        self.0.frustum_near = frustum_near;
    }

    /// far plane distance
    pub fn frustum_far(&self) -> f32 {
        self.0.frustum_far
    }
    /// set far plane distance
    pub fn set_frustum_far(&mut self, frustum_far: f32) {
        self.0.frustum_far = frustum_far;
    }

    /// if orthographic or perspective
    pub fn orthographic(&self) -> bool {
        self.0.orthographic != 0
    }
    /// set if orthographic or perspective
    pub fn set_orthographic(&mut self, orthographic: bool) {
        self.0.orthographic = orthographic as i32;
    }
}

wrapper! {
    /// This is the data structure describing one abstract visualization geom - which could correspond to a model geom or to a decoration element constructed by the visualizer.
    MjvGeom of crate::bindgen::mjvGeom
    /*
    struct mjvGeom_ {                 // abstract geom
      // type info
      int      type;                  // geom type (mjtGeom)
      int      dataid;                // mesh, hfield or plane id
      int      objtype;               // mujoco object type; mjOBJ_UNKNOWN for decor
      int      objid;                 // mujoco object id; -1 for decor
      int      category;              // visual category
      int      matid;                 // material id
      int      texcoord;              // mesh or flex geom has texture coordinates
      int      segid;                 // segmentation id

      // spatial transform
      float    size[3];               // size parameters
      float    pos[3];                // Cartesian position
      float    mat[9];                // Cartesian orientation

      // material properties
      float    rgba[4];               // color and transparency
      float    emission;              // emission coef
      float    specular;              // specular coef
      float    shininess;             // shininess coef
      float    reflectance;           // reflectance coef

      char     label[100];            // text label

      // transparency rendering (set internally)
      float    camdist;               // distance to camera (used by sorter)
      float    modelrbound;           // geom rbound from model, 0 if not model geom
      mjtByte  transparent;           // treat geom as transparent
    };
    typedef struct mjvGeom_ mjvGeom;
    */
}
impl MjvGeom {
    /// geom type
    pub fn type_(&self) -> crate::bindgen::mjtGeom {
        // SAFETY: `mjtGeom` is a bitmask, so it can be safely casted from `i32`
        unsafe { std::mem::transmute(self.0.type_) }
    }
    /// set geom type
    pub fn set_type(&mut self, type_: crate::bindgen::mjtGeom) {
        self.0.type_ = type_.0 as i32;
    }

    /// mesh, hfield or plane id
    pub fn dataid(&self) -> Option<ObjectId<obj::Mesh>> {
        if self.0.dataid < 0 {
            None
        } else {
            Some(ObjectId::new(self.0.dataid as usize))
        }
    }
    /// set mesh, hfield or plane id
    pub fn set_dataid(&mut self, dataid: Option<ObjectId<obj::Mesh>>) {
        self.0.dataid = dataid.map_or(-1, |id| id.index() as i32);
    }

    /// mujoco object type; `mjtObj::UNKNOWN` for decor
    pub fn objtype(&self) -> mjtObj {
        mjtObj(self.0.objtype as u32)
    }
    /// set mujoco object type; `mjtObj::UNKNOWN` for decor
    pub fn set_objtype(&mut self, objtype: mjtObj) {
        self.0.objtype = objtype.0 as i32;
    }

    /// mujoco object id
    pub fn objid(&self) -> Option<ObjectId<obj::Unknown>> {
        if self.0.objid < 0 {
            None
        } else {
            Some(ObjectId::new(self.0.objid as usize))
        }
    }
    /// set mujoco object id
    pub fn set_objid(&mut self, objid: Option<ObjectId<obj::Unknown>>) {
        self.0.objid = objid.map_or(-1, |id| id.index() as i32);
    }

    /// visual category
    pub fn category(&self) -> crate::bindgen::mjtCatBit {
        // SAFETY: `mjtCatBit` is a bitmask, so it can be safely casted from `i32`
        unsafe { std::mem::transmute(self.0.category) }
    }
    /// set visual category
    pub fn set_category(&mut self, category: crate::bindgen::mjtCatBit) {
        self.0.category = category.0 as i32;
    }

    /// material id
    pub fn matid(&self) -> Option<ObjectId<obj::Material>> {
        if self.0.matid < 0 {
            None
        } else {
            Some(ObjectId::new(self.0.matid as usize))
        }
    }
    /// set material id
    pub fn set_matid(&mut self, matid: Option<ObjectId<obj::Material>>) {
        self.0.matid = matid.map_or(-1, |id| id.index() as i32);
    }

    /// mesh or flex geom has texture coordinates
    pub fn texcoord(&self) -> bool {
        self.0.texcoord != 0
    }
    /// set mesh or flex geom has texture coordinates
    pub fn set_texcoord(&mut self, texcoord: bool) {
        self.0.texcoord = texcoord as i32;
    }

    /// segmentation id
    pub fn segid(&self) -> Option<SegmentationId> {
        if self.0.segid < 0 {
            None
        } else {
            Some(SegmentationId(self.0.segid as usize))
        }
    }
    /// set segmentation id
    pub fn set_segid(&mut self, segid: Option<SegmentationId>) {
        self.0.segid = segid.map_or(-1, |id| id.0 as i32);
    }

    /// size parameters
    pub fn size(&self) -> [f32; 3] {
        self.0.size
    }
    /// set size parameters
    pub fn set_size(&mut self, size: [f32; 3]) {
        self.0.size = size;
    }

    /// Cartesian position
    pub fn pos(&self) -> [f32; 3] {
        self.0.pos
    }
    /// set Cartesian position
    pub fn set_pos(&mut self, pos: [f32; 3]) {
        self.0.pos = pos;
    }

    /// Cartesian orientation
    pub fn mat(&self) -> [f32; 9] {
        self.0.mat
    }
    /// set Cartesian orientation
    pub fn set_mat(&mut self, mat: [f32; 9]) {
        self.0.mat = mat;
    }

    /// color and transparency
    pub fn rgba(&self) -> [f32; 4] {
        self.0.rgba
    }
    /// set color and transparency
    pub fn set_rgba(&mut self, rgba: [f32; 4]) {
        self.0.rgba = rgba;
    }

    /// emission coef
    pub fn emission(&self) -> f32 {
        self.0.emission
    }
    /// set emission coef
    pub fn set_emission(&mut self, emission: f32) {
        self.0.emission = emission;
    }

    /// specular coef
    pub fn specular(&self) -> f32 {
        self.0.specular
    }
    /// set specular coef
    pub fn set_specular(&mut self, specular: f32) {
        self.0.specular = specular;
    }

    /// shininess coef
    pub fn shininess(&self) -> f32 {
        self.0.shininess
    }
    /// set shininess coef
    pub fn set_shininess(&mut self, shininess: f32) {
        self.0.shininess = shininess;
    }

    /// reflectance coef
    pub fn reflectance(&self) -> f32 {
        self.0.reflectance
    }
    /// set reflectance coef
    pub fn set_reflectance(&mut self, reflectance: f32) {
        self.0.reflectance = reflectance;
    }

    /// text label
    pub fn label(&self) -> &str {
        let c_str = unsafe {std::ffi::CStr::from_ptr(self.0.label.as_ptr())};
        c_str.to_str().expect("Invalid UTF-8 in label")
    }
    /// set text label
    pub fn set_label(&mut self, label: &str) {
        let label_size = self.0.label.len();
        let i8_bytes = label.bytes().map(|u8| u8 as i8).collect::<Vec<_>>();
        self.0.label.copy_from_slice(&i8_bytes[..i8_bytes.len().min(label_size)]);
    }

    /// distance to camera (used by sorter)
    pub fn camdist(&self) -> f32 {
        self.0.camdist
    }
    /// set distance to camera (used by sorter)
    pub fn set_camdist(&mut self, camdist: f32) {
        self.0.camdist = camdist;
    }

    /// geom rbound from model
    pub fn modelrbound(&self) -> Option<f32> {
        if self.0.modelrbound == 0.0 {
            None
        } else {
            Some(self.0.modelrbound)
        }
    }
    /// set geom rbound from model
    pub fn set_modelrbound(&mut self, modelrbound: Option<f32>) {
        self.0.modelrbound = modelrbound.unwrap_or(0.0);
    }

    /// treat geom as transparent
    pub fn transparent(&self) -> bool {
        self.0.transparent != 0
    }
    /// set treat geom as transparent
    pub fn set_transparent(&mut self, transparent: bool) {
        self.0.transparent = transparent as u8;
    }
}

wrapper! {
    /// This is the data structure describing one OpenGL light.
    MjvLight of crate::bindgen::mjvLight
    /*
    struct mjvLight_ {                // OpenGL light
      float    pos[3];                // position rel. to body frame
      float    dir[3];                // direction rel. to body frame
      int      type;                  // type (mjtLightType)
      int      texid;                 // texture id for image lights
      float    attenuation[3];        // OpenGL attenuation (quadratic model)
      float    cutoff;                // OpenGL cutoff
      float    exponent;              // OpenGL exponent
      float    ambient[3];            // ambient rgb (alpha=1)
      float    diffuse[3];            // diffuse rgb (alpha=1)
      float    specular[3];           // specular rgb (alpha=1)
      mjtByte  headlight;             // headlight
      mjtByte  castshadow;            // does light cast shadows
      float    bulbradius;            // bulb radius for soft shadows
      float    intensity;             // intensity, in candelas
      float    range;                 // range of effectiveness
    };
    typedef struct mjvLight_ mjvLight;
    */
}
impl MjvLight {
    /// position relative to body frame
    pub fn pos(&self) -> [f32; 3] {
        self.0.pos
    }
    /// set position relative to body frame
    pub fn set_pos(&mut self, pos: [f32; 3]) {
        self.0.pos = pos;
    }

    /// direction relative to body frame
    pub fn dir(&self) -> [f32; 3] {
        self.0.dir
    }
    /// set direction relative to body frame
    pub fn set_dir(&mut self, dir: [f32; 3]) {
        self.0.dir = dir;
    }

    /// OpenGL attenuation (quadratic model)
    pub fn attenuation(&self) -> [f32; 3] {
        self.0.attenuation
    }
    /// set OpenGL attenuation (quadratic model)
    pub fn set_attenuation(&mut self, attenuation: [f32; 3]) {
        self.0.attenuation = attenuation;
    }

    /// OpenGL cutoff
    pub fn cutoff(&self) -> f32 {
        self.0.cutoff
    }
    /// set OpenGL cutoff
    pub fn set_cutoff(&mut self, cutoff: f32) {
        self.0.cutoff = cutoff;
    }

    /// OpenGL exponent
    pub fn exponent(&self) -> f32 {
        self.0.exponent
    }
    /// set OpenGL exponent
    pub fn set_exponent(&mut self, exponent: f32) {
        self.0.exponent = exponent;
    }

    /// ambient rgb (alpha=1)
    pub fn ambient(&self) -> [f32; 3] {
        self.0.ambient
    }
    /// set ambient rgb (alpha=1)
    pub fn set_ambient(&mut self, ambient: [f32; 3]) {
        self.0.ambient = ambient;
    }

    /// diffuse rgb (alpha=1)
    pub fn diffuse(&self) -> [f32; 3] {
        self.0.diffuse
    }
    /// set diffuse rgb (alpha=1)
    pub fn set_diffuse(&mut self, diffuse: [f32; 3]) {
        self.0.diffuse = diffuse;
    }

    /// specular rgb (alpha=1)
    pub fn specular(&self) -> [f32; 3] {
        self.0.specular
    }
    /// set specular rgb (alpha=1)
    pub fn set_specular(&mut self, specular: [f32; 3]) {
        self.0.specular = specular;
    }

    /// is headlight
    pub fn headlight(&self) -> bool {
        self.0.headlight != 0
    }
    /// set is headlight
    pub fn set_headlight(&mut self, headlight: bool) {
        self.0.headlight = headlight as u8;
    }

    /// does light cast shadows
    pub fn castshadow(&self) -> bool {
        self.0.castshadow != 0
    }
    /// set does light cast shadows
    pub fn set_castshadow(&mut self, castshadow: bool) {
        self.0.castshadow = castshadow as u8;
    }

    /// bulb radius for soft shadows
    pub fn bulbradius(&self) -> f32 {
        self.0.bulbradius
    }
    /// set bulb radius for soft shadows
    pub fn set_bulbradius(&mut self, bulbradius: f32) {
        self.0.bulbradius = bulbradius;
    }
}

wrapper! {
    /// This structure contains options that enable and disable the visualization of various elements.
    MjvOption of crate::bindgen::mjvOption
    /*
    struct mjvOption_ {                  // abstract visualization options
      int      label;                    // what objects to label (mjtLabel)
      int      frame;                    // which frame to show (mjtFrame)
      mjtByte  geomgroup[mjNGROUP];      // geom visualization by group
      mjtByte  sitegroup[mjNGROUP];      // site visualization by group
      mjtByte  jointgroup[mjNGROUP];     // joint visualization by group
      mjtByte  tendongroup[mjNGROUP];    // tendon visualization by group
      mjtByte  actuatorgroup[mjNGROUP];  // actuator visualization by group
      mjtByte  flexgroup[mjNGROUP];      // flex visualization by group
      mjtByte  skingroup[mjNGROUP];      // skin visualization by group
      mjtByte  flags[mjNVISFLAG];        // visualization flags (indexed by mjtVisFlag)
      int      bvh_depth;                // depth of the bounding volume hierarchy to be visualized
      int      flex_layer;               // element layer to be visualized for 3D flex
    };
    typedef struct mjvOption_ mjvOption;
    */
}
wrapper!(VisualizationFlags of [u8; mjNVISFLAG as usize]);
impl std::ops::Index<crate::bindgen::mjtVisFlag> for VisualizationFlags {
    type Output = bool;

    fn index(&self, index: crate::bindgen::mjtVisFlag) -> &Self::Output {
        let flags: &[u8; mjNVISFLAG as usize] = &self.0;
        let flags: &[bool; mjNVISFLAG as usize] = unsafe {std::mem::transmute(flags)};
        &flags[index.0 as usize]
    }
}
impl std::ops::IndexMut<crate::bindgen::mjtVisFlag> for VisualizationFlags {
    fn index_mut(&mut self, index: crate::bindgen::mjtVisFlag) -> &mut Self::Output {
        let flags: &mut [u8; mjNVISFLAG as usize] = &mut self.0;
        let flags: &mut [bool; mjNVISFLAG as usize] = unsafe {std::mem::transmute(flags)};
        &mut flags[index.0 as usize]
    }
}
impl MjvOption {
    /// what objects to label
    pub fn label(&self) -> crate::bindgen::mjtLabel {
        // SAFETY: `mjtLabel` is a bitmask, so it can be safely casted from `i32`
        unsafe { std::mem::transmute(self.0.label) }
    }
    /// set what objects to label
    pub fn set_label(&mut self, label: crate::bindgen::mjtLabel) {
        self.0.label = label.0 as i32;
    }

    /// which frame to show
    pub fn frame(&self) -> crate::bindgen::mjtFrame {
        // SAFETY: `mjtFrame` is a bitmask, so it can be safely casted from `i32`
        unsafe { std::mem::transmute(self.0.frame) }
    }
    /// set which frame to show
    pub fn set_frame(&mut self, frame: crate::bindgen::mjtFrame) {
        self.0.frame = frame.0 as i32;
    }

    /// geom visualization by group
    pub fn geomgroup(&self) -> [bool; mjNGROUP as usize] {
        self.0.geomgroup.map(|b| b != 0)
    }
    /// set geom visualization by group
    pub fn set_geomgroup(&mut self, geomgroup: [bool; mjNGROUP as usize]) {
        for (i, &b) in geomgroup.iter().enumerate() {
            self.0.geomgroup[i] = b as u8;
        }
    }

    /// site visualization by group
    pub fn sitegroup(&self) -> [bool; mjNGROUP as usize] {
        self.0.sitegroup.map(|b| b != 0)
    }
    /// set site visualization by group
    pub fn set_sitegroup(&mut self, sitegroup: [bool; mjNGROUP as usize]) {
        for (i, &b) in sitegroup.iter().enumerate() {
            self.0.sitegroup[i] = b as u8;
        }
    }

    /// joint visualization by group
    pub fn jointgroup(&self) -> [bool; mjNGROUP as usize] {
        self.0.jointgroup.map(|b| b != 0)
    }
    /// set joint visualization by group
    pub fn set_jointgroup(&mut self, jointgroup: [bool; mjNGROUP as usize]) {
        for (i, &b) in jointgroup.iter().enumerate() {
            self.0.jointgroup[i] = b as u8;
        }
    }

    /// tendon visualization by group
    pub fn tendongroup(&self) -> [bool; mjNGROUP as usize] {
        self.0.tendongroup.map(|b| b != 0)
    }
    /// set tendon visualization by group
    pub fn set_tendongroup(&mut self, tendongroup: [bool; mjNGROUP as usize]) {
        for (i, &b) in tendongroup.iter().enumerate() {
            self.0.tendongroup[i] = b as u8;
        }
    }

    /// actuator visualization by group
    pub fn actuatorgroup(&self) -> [bool; mjNGROUP as usize] {
        self.0.actuatorgroup.map(|b| b != 0)
    }
    /// set actuator visualization by group
    pub fn set_actuatorgroup(&mut self, actuatorgroup: [bool; mjNGROUP as usize]) {
        for (i, &b) in actuatorgroup.iter().enumerate() {
            self.0.actuatorgroup[i] = b as u8;
        }
    }

    /// flex visualization by group
    pub fn flexgroup(&self) -> [bool; mjNGROUP as usize] {
        self.0.flexgroup.map(|b| b != 0)
    }
    /// set flex visualization by group
    pub fn set_flexgroup(&mut self, flexgroup: [bool; mjNGROUP as usize]) {
        for (i, &b) in flexgroup.iter().enumerate() {
            self.0.flexgroup[i] = b as u8;
        }
    }

    /// skin visualization by group
    pub fn skingroup(&self) -> [bool; mjNGROUP as usize] {
        self.0.skingroup.map(|b| b != 0)
    }
    /// set skin visualization by group
    pub fn set_skingroup(&mut self, skingroup: [bool; mjNGROUP as usize]) {
        for (i, &b) in skingroup.iter().enumerate() {
            self.0.skingroup[i] = b as u8;
        }
    }

    /// visualization flags (indexed by `mjtVisFlag`)
    pub fn flags(&self) -> &VisualizationFlags {
        <&VisualizationFlags>::from(&self.0.flags)
    }
    /// mutate visualization flags (indexed by `mjtVisFlag`)
    pub fn flags_mut(&mut self) -> &mut VisualizationFlags {
        <&mut VisualizationFlags>::from(&mut self.0.flags)
    }

    /// depth of the bounding volume hierarchy to be visualized
    pub fn bvh_depth(&self) -> Option<usize> {
        if self.0.bvh_depth < 0 {
            None
        } else {
            Some(self.0.bvh_depth as usize)
        }
    }
    /// set depth of the bounding volume hierarchy to be visualized
    pub fn set_bvh_depth(&mut self, bvh_depth: Option<usize>) {
        self.0.bvh_depth = bvh_depth.map_or(-1, |depth| depth as i32);
    }

    /// element layer to be visualized for 3D flex
    pub fn flex_layer(&self) -> Option<usize> {
        if self.0.flex_layer < 0 {
            None
        } else {
            Some(self.0.flex_layer as usize)
        }
    }
    /// set element layer to be visualized for 3D flex
    pub fn set_flex_layer(&mut self, flex_layer: Option<usize>) {
        self.0.flex_layer = flex_layer.map_or(-1, |layer| layer as i32);
    }
}

wrapper! {
    /// This structure contains everything needed to render the 3D scene in OpenGL.
    MjvScene of crate::bindgen::mjvScene
    /*
    struct mjvScene_ {                // abstract scene passed to OpenGL renderer
      // abstract geoms
      int      maxgeom;               // size of allocated geom buffer
      int      ngeom;                 // number of geoms currently in buffer
      mjvGeom* geoms;                 // buffer for geoms (ngeom)
      int*     geomorder;             // buffer for ordering geoms by distance to camera (ngeom)

      // flex data
      int      nflex;                 // number of flexes
      int*     flexedgeadr;           // address of flex edges (nflex)
      int*     flexedgenum;           // number of edges in flex (nflex)
      int*     flexvertadr;           // address of flex vertices (nflex)
      int*     flexvertnum;           // number of vertices in flex (nflex)
      int*     flexfaceadr;           // address of flex faces (nflex)
      int*     flexfacenum;           // number of flex faces allocated (nflex)
      int*     flexfaceused;          // number of flex faces currently in use (nflex)
      int*     flexedge;              // flex edge data (2*nflexedge)
      float*   flexvert;              // flex vertices (3*nflexvert)
      float*   flexface;              // flex faces vertices (9*sum(flexfacenum))
      float*   flexnormal;            // flex face normals (9*sum(flexfacenum))
      float*   flextexcoord;          // flex face texture coordinates (6*sum(flexfacenum))
      mjtByte  flexvertopt;           // copy of mjVIS_FLEXVERT mjvOption flag
      mjtByte  flexedgeopt;           // copy of mjVIS_FLEXEDGE mjvOption flag
      mjtByte  flexfaceopt;           // copy of mjVIS_FLEXFACE mjvOption flag
      mjtByte  flexskinopt;           // copy of mjVIS_FLEXSKIN mjvOption flag

      // skin data
      int      nskin;                 // number of skins
      int*     skinfacenum;           // number of faces in skin (nskin)
      int*     skinvertadr;           // address of skin vertices (nskin)
      int*     skinvertnum;           // number of vertices in skin (nskin)
      float*   skinvert;              // skin vertex data (3*nskinvert)
      float*   skinnormal;            // skin normal data (3*nskinvert)

      // OpenGL lights
      int      nlight;                // number of lights currently in buffer
      mjvLight lights[mjMAXLIGHT];    // buffer for lights (nlight)

      // OpenGL cameras
      mjvGLCamera camera[2];          // left and right camera

      // OpenGL model transformation
      mjtByte  enabletransform;       // enable model transformation
      float    translate[3];          // model translation
      float    rotate[4];             // model quaternion rotation
      float    scale;                 // model scaling

      // OpenGL rendering effects
      int      stereo;                // stereoscopic rendering (mjtStereo)
      mjtByte  flags[mjNRNDFLAG];     // rendering flags (indexed by mjtRndFlag)

      // framing
      int      framewidth;            // frame pixel width; 0: disable framing
      float    framergb[3];           // frame color
    };
    typedef struct mjvScene_ mjvScene;
    */
}
wrapper!(RenderingFlags of [u8; mjNRNDFLAG as usize]);
impl std::ops::Index<crate::bindgen::mjtRndFlag> for RenderingFlags {
    type Output = bool;

    fn index(&self, index: crate::bindgen::mjtRndFlag) -> &Self::Output {
        let flags: &[u8; mjNRNDFLAG as usize] = &self.0;
        let flags: &[bool; mjNRNDFLAG as usize] = unsafe {std::mem::transmute(flags)};
        &flags[index.0 as usize]
    }
}
impl MjvScene {
    /* !!!!! READONLY !!!!! */

    /// size of allocated geom buffer
    pub fn maxgeom(&self) -> usize {
        self.0.maxgeom as usize
    }
    /// number of geoms currently in buffer
    pub fn ngeom(&self) -> usize {
        self.0.ngeom as usize
    }
    /// buffer for geoms (ngeom)
    pub fn geoms(&self) -> &[MjvGeom] {
        // SAFETY: `MjvGeom` is just a newtype of `mjvGeom`
        unsafe {std::slice::from_raw_parts(
            self.0.geoms as *const MjvGeom,
            self.ngeom()
        )}
    }
    /// buffer for ordering geoms by distance to camera (ngeom)
    pub fn geomorder(&self) -> &[i32] {
        // SAFETY: `geomorder` is a pointer to an array of `i32` with length `ngeom`
        unsafe {std::slice::from_raw_parts(self.0.geomorder, self.ngeom())}
    }

    /// number of flexes
    pub fn nflex(&self) -> usize {
        self.0.nflex as usize
    }
    /// address of flex edges (nflex)
    pub fn flexedgeadr(&self) -> &[i32] {
        // SAFETY: `flexedgeadr` is a pointer to an array of `i32` with length `nflex`
        unsafe {std::slice::from_raw_parts(self.0.flexedgeadr, self.nflex())}
    }
    /// number of edges in flex (nflex)
    pub fn flexedgenum(&self) -> &[i32] {
        // SAFETY: `flexedgenum` is a pointer to an array of `i32` with length `nflex`
        unsafe {std::slice::from_raw_parts(self.0.flexedgenum, self.nflex())}
    }
    /// address of flex vertices (nflex)
    pub fn flexvertadr(&self) -> &[i32] {
        // SAFETY: `flexvertadr` is a pointer to an array of `i32` with length `nflex`
        unsafe {std::slice::from_raw_parts(self.0.flexvertadr, self.nflex())}
    }
    /// number of vertices in flex (nflex)
    pub fn flexvertnum(&self) -> &[i32] {
        // SAFETY: `flexvertnum` is a pointer to an array of `i32` with length `nflex`
        unsafe {std::slice::from_raw_parts(self.0.flexvertnum, self.nflex())}
    }
    /// address of flex faces (nflex)
    pub fn flexfaceadr(&self) -> &[i32] {
        // SAFETY: `flexfaceadr` is a pointer to an array of `i32` with length `nflex`
        unsafe {std::slice::from_raw_parts(self.0.flexfaceadr, self.nflex())}
    }
    /// number of flex faces allocated (nflex)
    pub fn flexfacenum(&self) -> &[i32] {
        // SAFETY: `flexfacenum` is a pointer to an array of `i32` with length `nflex`
        unsafe {std::slice::from_raw_parts(self.0.flexfacenum, self.nflex())}
    }
    /// number of flex faces currently in use (nflex)
    pub fn flexfaceused(&self) -> &[i32] {
        // SAFETY: `flexfaceused` is a pointer to an array of `i32` with length `nflex`
        unsafe {std::slice::from_raw_parts(self.0.flexfaceused, self.nflex())}
    }
    /// flex edge data (2*nflexedge)
    pub fn flexedge(&self) -> &[i32] {
        // SAFETY: `flexedge` is a pointer to an array of `i32` with length `2 * sum(flexedgenum)`
        let nflexedge = self.flexedgenum().iter().sum::<i32>() as usize;
        unsafe {std::slice::from_raw_parts(self.0.flexedge, 2 * nflexedge)}
    }
    /// flex vertices (3*nflexvert)
    pub fn flexvert(&self) -> &[f32] {
        // SAFETY: `flexvert` is a pointer to an array of `f32` with length `3 * sum(flexvertnum)`
        let nflexvert = self.flexvertnum().iter().sum::<i32>() as usize;
        unsafe {std::slice::from_raw_parts(self.0.flexvert, 3 * nflexvert)}
    }
    /// flex faces vertices (9*sum(flexfacenum))
    pub fn flexface(&self) -> &[f32] {
        // SAFETY: `flexface` is a pointer to an array of `f32` with length `9 * sum(flexfacenum)`
        let nflexface = self.flexfacenum().iter().sum::<i32>() as usize;
        unsafe {std::slice::from_raw_parts(self.0.flexface, 9 * nflexface)}
    }
    /// flex face normals (9*sum(flexfacenum))
    pub fn flexnormal(&self) -> &[f32] {
        // SAFETY: `flexnormal` is a pointer to an array of `f32` with length `9 * sum(flexfacenum)`
        let nflexface = self.flexfacenum().iter().sum::<i32>() as usize;
        unsafe {std::slice::from_raw_parts(self.0.flexnormal, 9 * nflexface)}
    }
    /// flex face texture coordinates (6*sum(flexfacenum))
    pub fn flextexcoord(&self) -> &[f32] {
        // SAFETY: `flextexcoord` is a pointer to an array of `f32` with length `6 * sum(flexfacenum)`
        let nflexface = self.flexfacenum().iter().sum::<i32>() as usize;
        unsafe {std::slice::from_raw_parts(self.0.flextexcoord, 6 * nflexface)}
    }
    /// copy of `mjVIS_FLEXVERT` `mjvOption` flag
    pub fn flexvertopt(&self) -> bool {
        self.0.flexvertopt != 0
    }
    /// copy of `mjVIS_FLEXEDGE` `mjvOption` flag
    pub fn flexedgeopt(&self) -> bool {
        self.0.flexedgeopt != 0
    }
    /// copy of `mjVIS_FLEXFACE` `mjvOption` flag
    pub fn flexfaceopt(&self) -> bool {
        self.0.flexfaceopt != 0
    }
    /// copy of `mjVIS_FLEXSKIN` `mjvOption` flag
    pub fn flexskinopt(&self) -> bool {
        self.0.flexskinopt != 0
    }

    /// number of skins
    pub fn nskin(&self) -> usize {
        self.0.nskin as usize
    }
    /// number of faces in skin (nskin)
    pub fn skinfacenum(&self) -> &[i32] {
        // SAFETY: `skinfacenum` is a pointer to an array of `i32` with length `nskin`
        unsafe {std::slice::from_raw_parts(self.0.skinfacenum, self.nskin())}
    }
    /// address of skin vertices (nskin)
    pub fn skinvertadr(&self) -> &[i32] {
        // SAFETY: `skinvertadr` is a pointer to an array of `i32` with length `nskin`
        unsafe {std::slice::from_raw_parts(self.0.skinvertadr, self.nskin())}
    }
    /// number of vertices in skin (nskin)
    pub fn skinvertnum(&self) -> &[i32] {
        // SAFETY: `skinvertnum` is a pointer to an array of `i32` with length `nskin`
        unsafe {std::slice::from_raw_parts(self.0.skinvertnum, self.nskin())}
    }
    /// skin vertex data (3*nskinvert)
    pub fn skinvert(&self) -> &[f32] {
        // SAFETY: `skinvert` is a pointer to an array of `f32` with length `3 * nskinvert`
        let nskinvert = self.skinvertnum().iter().sum::<i32>() as usize;
        unsafe {std::slice::from_raw_parts(self.0.skinvert, 3 * nskinvert)}
    }
    /// skin normal data (3*nskinvert)
    pub fn skinnormal(&self) -> &[f32] {
        // SAFETY: `skinnormal` is a pointer to an array of `f32` with length `3 * nskinvert`
        let nskinvert = self.skinvertnum().iter().sum::<i32>() as usize;
        unsafe {std::slice::from_raw_parts(self.0.skinnormal, 3 * nskinvert)}
    }

    /// number of lights currently in buffer
    pub fn nlight(&self) -> usize {
        self.0.nlight as usize
    }
    /// buffer for lights (nlight)
    pub fn lights(&self) -> &[MjvLight] {
        // SAFETY: `MjvLight` is just a newtype of `mjvLight`
        unsafe {std::slice::from_raw_parts(
            self.0.lights.as_ptr() as *const MjvLight,
            self.nlight()
        )}
    }

    /// left and right camera
    pub fn camera(&self) -> [MjvGlCamera; 2] {
        self.0.camera.map(From::from)
    }

    /// enable model transformation
    pub fn enabletransform(&self) -> bool {
        self.0.enabletransform != 0
    }
    /// model translation
    pub fn translate(&self) -> [f32; 3] {
        self.0.translate
    }
    /// model quaternion rotation
    pub fn rotate(&self) -> [f32; 4] {
        self.0.rotate
    }
    /// model scaling
    pub fn scale(&self) -> f32 {
        self.0.scale
    }

    /// stereoscopic rendering
    pub fn stereo(&self) -> crate::bindgen::mjtStereo {
        // SAFETY: `mjtStereo` is a bitmask, so it can be safely casted from `i32`
        unsafe { std::mem::transmute(self.0.stereo) }
    }
    /// rendering flags (indexed by `mjtRndFlag`)
    pub fn flags(&self) -> &RenderingFlags {
        <&RenderingFlags>::from(&self.0.flags)
    }

    /// frame pixel width; 0: disable framing
    pub fn framewidth(&self) -> Option<usize> {
        if self.0.framewidth == 0 {
            None
        } else {
            Some(self.0.framewidth as usize)
        }
    }
    /// frame color
    pub fn framergb(&self) -> [f32; 3] {
        self.0.framergb
    }
}

wrapper! {
    /// This structure contains everything needed to render a 2D plot in OpenGL.
    /// The buffers for line points etc. are preallocated,
    /// and the user has to populate them before calling the function `mjr_figure`
    /// with this data structure as an argument.
    MjvFigure of crate::bindgen::mjvFigure
    /*
    struct mjvFigure_ {               // abstract 2D figure passed to OpenGL renderer
      // enable flags
      int     flg_legend;             // show legend
      int     flg_ticklabel[2];       // show grid tick labels (x,y)
      int     flg_extend;             // automatically extend axis ranges to fit data
      int     flg_barplot;            // isolated line segments (i.e. GL_LINES)
      int     flg_selection;          // vertical selection line
      int     flg_symmetric;          // symmetric y-axis

      // style settings
      float   linewidth;              // line width
      float   gridwidth;              // grid line width
      int     gridsize[2];            // number of grid points in (x,y)
      float   gridrgb[3];             // grid line rgb
      float   figurergba[4];          // figure color and alpha
      float   panergba[4];            // pane color and alpha
      float   legendrgba[4];          // legend color and alpha
      float   textrgb[3];             // text color
      float   linergb[mjMAXLINE][3];  // line colors
      float   range[2][2];            // axis ranges; (min>=max) automatic
      char    xformat[20];            // x-tick label format for sprintf
      char    yformat[20];            // y-tick label format for sprintf
      char    minwidth[20];           // string used to determine min y-tick width

      // text labels
      char    title[1000];            // figure title; subplots separated with 2+ spaces
      char    xlabel[100];            // x-axis label
      char    linename[mjMAXLINE][100];  // line names for legend

      // dynamic settings
      int     legendoffset;           // number of lines to offset legend
      int     subplot;                // selected subplot (for title rendering)
      int     highlight[2];           // if point is in legend rect, highlight line
      int     highlightid;            // if id>=0 and no point, highlight id
      float   selection;              // selection line x-value

      // line data
      int     linepnt[mjMAXLINE];     // number of points in line; (0) disable
      float   linedata[mjMAXLINE][2*mjMAXLINEPNT];  // line data (x,y)

      // output from renderer
      int     xaxispixel[2];          // range of x-axis in pixels
      int     yaxispixel[2];          // range of y-axis in pixels
      float   xaxisdata[2];           // range of x-axis in data units
      float   yaxisdata[2];           // range of y-axis in data units
    };
    typedef struct mjvFigure_ mjvFigure;
    */
}
impl MjvFigure {
    /// show legend
    pub fn flg_legend(&self) -> bool {
        self.0.flg_legend != 0
    }
    /// set show legend
    pub fn set_flg_legend(&mut self, flg_legend: bool) {
        self.0.flg_legend = flg_legend as i32;
    }

    /// show grid tick labels (x,y)
    pub fn flg_ticklabel(&self) -> [bool; 2] {
        self.0.flg_ticklabel.map(|b| b != 0)
    }
    /// set show grid tick labels (x,y)
    pub fn set_flg_ticklabel(&mut self, flg_ticklabel: [bool; 2]) {
        for (i, &b) in flg_ticklabel.iter().enumerate() {
            self.0.flg_ticklabel[i] = b as i32;
        }
    }

    /// automatically extend axis ranges to fit data
    pub fn flg_extend(&self) -> bool {
        self.0.flg_extend != 0
    }
    /// set automatically extend axis ranges to fit data
    pub fn set_flg_extend(&mut self, flg_extend: bool) {
        self.0.flg_extend = flg_extend as i32;
    }

    /// isolated line segments (i.e. GL_LINES)
    pub fn flg_barplot(&self) -> bool {
        self.0.flg_barplot != 0
    }
    /// set isolated line segments (i.e. GL_LINES)
    pub fn set_flg_barplot(&mut self, flg_barplot: bool) {
        self.0.flg_barplot = flg_barplot as i32;
    }

    /// vertical selection line
    pub fn flg_selection(&self) -> bool {
        self.0.flg_selection != 0
    }
    /// set vertical selection line
    pub fn set_flg_selection(&mut self, flg_selection: bool) {
        self.0.flg_selection = flg_selection as i32;
    }

    /// symmetric y-axis
    pub fn flg_symmetric(&self) -> bool {
        self.0.flg_symmetric != 0
    }
    /// set symmetric y-axis
    pub fn set_flg_symmetric(&mut self, flg_symmetric: bool) {
        self.0.flg_symmetric = flg_symmetric as i32;
    }

    /// line width
    pub fn linewidth(&self) -> f32 {
        self.0.linewidth
    }
    /// set line width
    pub fn set_linewidth(&mut self, linewidth: f32) {
        self.0.linewidth = linewidth;
    }

    /// grid line width
    pub fn gridwidth(&self) -> f32 {
        self.0.gridwidth
    }
    /// set grid line width
    pub fn set_gridwidth(&mut self, gridwidth: f32) {
        self.0.gridwidth = gridwidth;
    }

    /// number of grid points in (x,y)
    pub fn gridsize(&self) -> [usize; 2] {
        self.0.gridsize.map(|s| s as usize)
    }
    /// set number of grid points in (x,y)
    pub fn set_gridsize(&mut self, gridsize: [usize; 2]) {
        for (i, &s) in gridsize.iter().enumerate() {
            self.0.gridsize[i] = s as i32;
        }
    }

    /// grid line rgb
    pub fn gridrgb(&self) -> [f32; 3] {
        self.0.gridrgb
    }
    /// set grid line rgb
    pub fn set_gridrgb(&mut self, gridrgb: [f32; 3]) {
        self.0.gridrgb = gridrgb;
    }

    /// figure color and alpha
    pub fn figurergba(&self) -> [f32; 4] {
        self.0.figurergba
    }
    /// set figure color and alpha
    pub fn set_figurergba(&mut self, figurergba: [f32; 4]) {
        self.0.figurergba = figurergba;
    }

    /// pane color and alpha
    pub fn panergba(&self) -> [f32; 4] {
        self.0.panergba
    }
    /// set pane color and alpha
    pub fn set_panergba(&mut self, panergba: [f32; 4]) {
        self.0.panergba = panergba;
    }

    /// legend color and alpha
    pub fn legendrgba(&self) -> [f32; 4] {
        self.0.legendrgba
    }
    /// set legend color and alpha
    pub fn set_legendrgba(&mut self, legendrgba: [f32; 4]) {
        self.0.legendrgba = legendrgba;
    }

    /// text color
    pub fn textrgb(&self) -> [f32; 3] {
        self.0.textrgb
    }
    /// set text color
    pub fn set_textrgb(&mut self, textrgb: [f32; 3]) {
        self.0.textrgb = textrgb;
    }

    /// line color (`line` must be < `mjMAXLINE`)
    pub fn linergb(&self, line: usize) -> [f32; 3] {
        self.0.linergb[line]
    }
    /// set line color (`line` must be < `mjMAXLINE`)
    pub fn set_linergb(&mut self, line: usize, linergb: [f32; 3]) {
        self.0.linergb[line] = linergb;
    }

    /// axis ranges; (min>=max) automatic
    pub fn range(&self) -> [[f32; 2]; 2] {
        self.0.range
    }
    /// set axis ranges; (min>=max) automatic
    pub fn set_range(&mut self, range: [[f32; 2]; 2]) {
        self.0.range = range;
    }

    /// x-tick label format for sprintf
    pub fn xformat(&self) -> &str {
        // SAFETY: `xformat` is a C string, so it is null-terminated
        unsafe {std::ffi::CStr::from_ptr(self.0.xformat.as_ptr())}
            .to_str()
            .expect("Invalid UTF-8 in xformat")
    }
    /// set x-tick label format for sprintf
    pub fn set_xformat(&mut self, xformat: &str) {
        assert!(xformat.len() < self.0.xformat.len(), "xformat must be less than {} characters", self.0.xformat.len());
        for (i, &b) in xformat.as_bytes().iter().enumerate() {
            self.0.xformat[i] = b as i8;
        }
    }

    /// y-tick label format for sprintf
    pub fn yformat(&self) -> &str {
        // SAFETY: `yformat` is a C string, so it is null-terminated
        unsafe {std::ffi::CStr::from_ptr(self.0.yformat.as_ptr())}
            .to_str()
            .expect("Invalid UTF-8 in yformat")
    }
    /// set y-tick label format for sprintf
    pub fn set_yformat(&mut self, yformat: &str) {
        assert!(yformat.len() < self.0.yformat.len(), "yformat must be less than {} characters", self.0.yformat.len());
        for (i, &b) in yformat.as_bytes().iter().enumerate() {
            self.0.yformat[i] = b as i8;
        }
    }

    /// string used to determine min y-tick width
    pub fn minwidth(&self) -> &str {
        // SAFETY: `minwidth` is a C string, so it is null-terminated
        unsafe {std::ffi::CStr::from_ptr(self.0.minwidth.as_ptr())}
            .to_str()
            .expect("Invalid UTF-8 in minwidth")
    }
    /// set string used to determine min y-tick width
    pub fn set_minwidth(&mut self, minwidth: &str) {
        assert!(minwidth.len() < self.0.minwidth.len(), "minwidth must be less than {} characters", self.0.minwidth.len());
        for (i, &b) in minwidth.as_bytes().iter().enumerate() {
            self.0.minwidth[i] = b as i8;
        }
    }

    /// figure title; subplots separated with 2+ spaces
    pub fn title(&self) -> &str {
        // SAFETY: `title` is a C string, so it is null-terminated
        unsafe {std::ffi::CStr::from_ptr(self.0.title.as_ptr())}
            .to_str()
            .expect("Invalid UTF-8 in title")
    }
    /// set figure title; subplots separated with 2+ spaces
    pub fn set_title(&mut self, title: &str) {
        assert!(title.len() < self.0.title.len(), "title must be less than {} characters", self.0.title.len());
        for (i, &b) in title.as_bytes().iter().enumerate() {
            self.0.title[i] = b as i8;
        }
    }

    /// x-axis label
    pub fn xlabel(&self) -> &str {
        // SAFETY: `xlabel` is a C string, so it is null-terminated
        unsafe {std::ffi::CStr::from_ptr(self.0.xlabel.as_ptr())}
            .to_str()
            .expect("Invalid UTF-8 in xlabel")
    }
    /// set x-axis label
    pub fn set_xlabel(&mut self, xlabel: &str) {
        assert!(xlabel.len() < self.0.xlabel.len(), "xlabel must be less than {} characters", self.0.xlabel.len());
        for (i, &b) in xlabel.as_bytes().iter().enumerate() {
            self.0.xlabel[i] = b as i8;
        }
    }

    /// line name for legend (`line` must be < `mjMAXLINE`)
    pub fn linename(&self, line: usize) -> &str {
        // SAFETY: `linename` is a C string, so it is null-terminated
        unsafe {std::ffi::CStr::from_ptr(self.0.linename[line].as_ptr())}
            .to_str()
            .expect("Invalid UTF-8 in linename")
    }
    /// set line name for legend (`line` must be < `mjMAXLINE`)
    pub fn set_linename(&mut self, line: usize, linename: &str) {
        assert!(linename.len() < self.0.linename[line].len(), "linename must be less than {} characters", self.0.linename[line].len());
        for (i, &b) in linename.as_bytes().iter().enumerate() {
            self.0.linename[line][i] = b as i8;
        }
    }

    /// numer of lines to offset legend
    pub fn legendoffset(&self) -> usize {
        self.0.legendoffset as usize
    }
    /// set number of lines to offset legend
    pub fn set_legendoffset(&mut self, legendoffset: usize) {
        self.0.legendoffset = legendoffset as i32;
    }

    /// selected subplot (for title rendering)
    pub fn subplot(&self) -> usize {
        self.0.subplot as usize
    }
    /// set selected subplot (for title rendering)
    pub fn set_subplot(&mut self, subplot: usize) {
        self.0.subplot = subplot as i32;
    }

    /// highlight line if point is in legend rect
    pub fn highlight(&self) -> [usize; 2] {
        self.0.highlight.map(|h| h as usize)
    }
    /// set highlight line if point is in legend rect
    pub fn set_highlight(&mut self, highlight: [usize; 2]) {
        for (i, &h) in highlight.iter().enumerate() {
            self.0.highlight[i] = h as i32;
        }
    }

    /// highlight id if point is in legend rect
    pub fn highlightid(&self) -> Option<usize> {
        if self.0.highlightid < 0 {
            None
        } else {
            Some(self.0.highlightid as usize)
        }
    }
    /// set highlight id if point is in legend rect
    pub fn set_highlightid(&mut self, highlightid: Option<usize>) {
        self.0.highlightid = highlightid.map_or(-1, |id| id as i32);
    }

    /// selection line x-value
    pub fn selection(&self) -> f32 {
        self.0.selection
    }
    /// set selection line x-value
    pub fn set_selection(&mut self, selection: f32) {
        self.0.selection = selection;
    }

    /// number of points in line; `None`: disable (`line` must be < `mjMAXLINE`)
    pub fn linepnt(&self, line: usize) -> Option<usize> {
        if self.0.linepnt[line] == 0 {
            None
        } else {
            Some(self.0.linepnt[line] as usize)
        }
    }
    /// set number of points in line; `None`: disable (`line` must be < `mjMAXLINE`)
    pub fn set_linepnt(&mut self, line: usize, linepnt: Option<usize>) {
        self.0.linepnt[line] = linepnt.map_or(0, |pnt| pnt as i32);
    }

    /// line data (x,y) (`line` must be < `mjMAXLINE`)
    pub fn linedata(&self, line: usize) -> &[[f32; 2]; mjMAXLINEPNT as usize] {
        let flatten = &self.0.linedata[line];
        #[cfg(debug_assertions)] {
            assert_eq!(
                flatten.len(), 2 * mjMAXLINEPNT as usize,
                "linedata length mismatch: expected {}, got {}",
                2 * mjMAXLINEPNT as usize, flatten.len()
            );
        }
        // SAFETY: `linedata` is a pointer to an array of `f32` with length `2 * mjMAXLINEPNT`
        unsafe {std::mem::transmute(flatten)}
    }
    /// mutate line data (x,y) (`line` must be < `mjMAXLINE`)
    pub fn linedata_mut(&mut self, line: usize) -> &mut [[f32; 2]; mjMAXLINEPNT as usize] {
        let flatten = &mut self.0.linedata[line];
        #[cfg(debug_assertions)] {
            assert_eq!(
                flatten.len(), 2 * mjMAXLINEPNT as usize,
                "linedata length mismatch: expected {}, got {}",
                2 * mjMAXLINEPNT as usize, flatten.len()
            );
        }
        // SAFETY: `linedata` is a pointer to an array of `f32` with length `2 * mjMAXLINEPNT`
        unsafe {std::mem::transmute(flatten)}
    }

    /* !!!!! READONLY !!!!! */

    /// range of x-axis in pixels
    pub fn xaxispixel(&self) -> std::ops::Range<usize> {
        self.0.xaxispixel[0] as usize..self.0.xaxispixel[1] as usize
    }

    /// range of y-axis in pixels
    pub fn yaxispixel(&self) -> std::ops::Range<usize> {
        self.0.yaxispixel[0] as usize..self.0.yaxispixel[1] as usize
    }

    /// range of x-axis in data units
    pub fn xaxisdata(&self) -> std::ops::Range<f32> {
        self.0.xaxisdata[0]..self.0.xaxisdata[1]
    }

    /// range of y-axis in data units
    pub fn yaxisdata(&self) -> std::ops::Range<f32> {
        self.0.yaxisdata[0]..self.0.yaxisdata[1]
    }
}

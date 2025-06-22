//! # Visualization
//! 
//! <https://mujoco.readthedocs.io/en/stable/APIreference/APItypes.html#visualisation>
//! 
//! The names of these struct types are prefixed with mjv.
//! 
//! <https://mujoco.readthedocs.io/en/stable/APIreference/APItypes.html#tyvisenums>
//! 
//! The enums below are defined in mjvisualize.h.

use crate::{SegmentationId, ObjectId, ObjType, obj};

wrapper! {
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
    /// This is the data structure holding information about mouse perturbations.
    MjvPerturb of crate::bindgen::mjvPerturb
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
        self.0.active = active as i32;
    }

    /// secondary perturbation bitmask
    pub fn active2(&self) -> crate::bindgen::mjtPertBit {
        // SAFETY: `mjtPertBit` is a bitmask, so it can be safely casted from `i32`
        unsafe { std::mem::transmute(self.0.active2) }
    }
    /// set secondary perturbation bitmask
    pub fn set_active2(&mut self, active2: crate::bindgen::mjtPertBit) {
        self.0.active2 = active2 as i32;
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
    /// This is the data structure describing one abstract camera.
    MjvCamera of crate::bindgen::mjvCamera
}
impl MjvCamera {
    /// camera type
    pub fn type_(&self) -> crate::bindgen::mjtCamera {
        // SAFETY: `mjtCamera` is a bitmask, so it can be safely casted from `i32`
        unsafe { std::mem::transmute(self.0.type_) }
    }
    /// set camera type
    pub fn set_type(&mut self, type_: crate::bindgen::mjtCamera) {
        self.0.type_ = type_ as i32;
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
    /// This is the data structure describing one OpenGL camera.
    MjvGlCamera of crate::bindgen::mjvGLCamera
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
    /// This is the data structure describing one abstract visualization geom - which could correspond to a model geom or to a decoration element constructed by the visualizer.
    MjvGeom of crate::bindgen::mjvGeom
}
impl MjvGeom {
    /// geom type
    pub fn type_(&self) -> crate::bindgen::mjtGeom {
        // SAFETY: `mjtGeom` is a bitmask, so it can be safely casted from `i32`
        unsafe { std::mem::transmute(self.0.type_) }
    }
    /// set geom type
    pub fn set_type(&mut self, type_: crate::bindgen::mjtGeom) {
        self.0.type_ = type_ as i32;
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

    /// mujoco object type; `ObjType::Unknown` for decor
    pub fn objtype(&self) -> ObjType {
        // SAFETY: `mjtObj` is a bitmask, so it can be safely casted from `i32`
        unsafe { std::mem::transmute(self.0.objtype) }
    }
    /// set mujoco object type; `ObjType::Unknown` for decor
    pub fn set_objtype(&mut self, objtype: ObjType) {
        self.0.objtype = objtype as i32;
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
        self.0.category = category as i32;
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

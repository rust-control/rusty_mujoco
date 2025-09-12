//! # [Visualization]((https://mujoco.readthedocs.io/en/stable/APIreference/APItypes.html#visualisation))
//! 
//! The names of these struct types are prefixed with `mjv`.

pub use crate::bindgen::{
    mjtPertBit, mjtCamera, mjtCatBit, mjtGeom, mjtLabel, mjtFrame, mjtVisFlag, mjtStereo, mjtRndFlag,
    mjNGROUP, mjNVISFLAG, mjNRNDFLAG, mjMAXLIGHT, mjMAXLINE, mjMAXLINEPNT,
};

use crate::{mjtObj, obj, ObjectId, SegmentationId};

pub use crate::bindgen::mjvPerturb;
impl Default for mjvPerturb {
    fn default() -> Self {
        crate::mjv_defaultPerturb()
    }
}
fields_mapping!(mjvPerturb {
    scalars {
        refpos: [f64; 3] = "reference position for selected object";
        refquat: [f64; 4] = "reference orientation for selected object";
        refselpos: [f64; 3] = "reference position for selection point";
        localpos: [f64; 3] = "selection point in object coordinates";
        localmass: f64 = "spatial inertia at selection point";
        scale: f64 = "relative mouse motion-to-space scaling (set by initPerturb)";
    }
    enums {
        active / set_active: mjtPertBit = "perturbation bitmask";
        active2 / set_active2: mjtPertBit = "secondary perturbation bitmask";
    }
});
impl mjvPerturb {
    /// selected body id
    pub fn select(&self) -> Option<ObjectId<obj::Body>> {
        if self.select > 0 {Some(unsafe { ObjectId::new_unchecked(self.select as _) })} else {None}
    }
    /// set selected body id
    pub fn set_select(&mut self, select: Option<ObjectId<obj::Body>>) -> &mut Self {
        self.select = select.map_or(-1, |id| id.index() as i32); self
    }

    /// selected flex id
    pub fn flexselect(&self) -> Option<ObjectId<obj::Flex>> {
        if self.flexselect >= 0 {Some(unsafe { ObjectId::new_unchecked(self.flexselect as _) })} else {None}
    }
    /// set selected flex id
    pub fn set_flexselect(&mut self, flexselect: Option<ObjectId<obj::Flex>>) -> &mut Self {
        self.flexselect = flexselect.map_or(-1, |id| id.index() as i32); self
    }

    /// selected skin id
    pub fn skinselect(&self) -> Option<ObjectId<obj::Skin>> {
        if self.skinselect >= 0 {Some(unsafe { ObjectId::new_unchecked(self.skinselect as _) })} else {None}
    }
    /// set selected skin id
    pub fn set_skinselect(&mut self, skinselect: Option<ObjectId<obj::Skin>>) -> &mut Self {
        self.skinselect = skinselect.map_or(-1, |id| id.index() as i32); self
    }
}

pub use crate::bindgen::mjvCamera;
impl Default for mjvCamera {
    /// Create a new camera with default settings by [`mjv_defaultCamera`](crate::mjv_defaultCamera).
    /// 
    /// See [`mjvCamera::default_free`] for a free camera.
    fn default() -> Self {
        crate::mjv_defaultCamera()
    }
}
impl mjvCamera {
    /// Create a new free camera with default settings by [`mjv_defaultFreeCamera`](crate::mjv_defaultFreeCamera).
    /// 
    /// See [`mjvCamera::default`] for a camera with default settings.
    pub fn default_free(m: &crate::MjModel) -> Self {
        crate::mjv_defaultFreeCamera(m)
    }
}
fields_mapping!(mjvCamera {
    scalars {
        lookat / set_lookat: [f64; 3] = "lookat point";
        distance / set_distance: f64 = "distance to lookat point or tracked body";
        azimuth / set_azimuth: f64 = "camera azimuth (deg)";
        elevation / set_elevation: f64 = "camera elevation (deg)";
    }
    enums {
        type_ / set_type: mjtCamera = "camera type";
    }
});
impl mjvCamera {
    /// fixed camera id
    pub fn fixedcamid(&self) -> Option<ObjectId<obj::Camera>> {
        if self.type_ != mjtCamera::FIXED.0 as i32 {return None}
        if self.fixedcamid >= 0 {Some(unsafe { ObjectId::new_unchecked(self.fixedcamid as _) })} else {None}
    }
    /// set fixed camera id, with setting camera type to `mjtCamera::FIXED`.
    pub fn set_fixedcamid(&mut self, fixedcamid: ObjectId<obj::Camera>) -> &mut Self {
        self.type_ = mjtCamera::FIXED.0 as i32;
        self.fixedcamid = fixedcamid.index() as i32;
        self
    }

    /// body id to track
    pub fn trackbodyid(&self) -> Option<ObjectId<obj::Body>> {
        if self.type_ != mjtCamera::TRACKING.0 as i32 {return None}
        if self.trackbodyid >= 0 {Some(unsafe { ObjectId::new_unchecked(self.trackbodyid as _) })} else {None}
    }
    /// set body id to track, with setting camera type to `mjtCamera::TRACKING`.
    pub fn set_trackbodyid(&mut self, trackbodyid: ObjectId<obj::Body>) -> &mut Self {
        self.type_ = mjtCamera::TRACKING.0 as i32;
        self.trackbodyid = trackbodyid.index() as i32;
        self
    }

    pub fn orthographic(&self) -> bool {self.orthographic != 0}
    pub fn set_orthographic(&mut self, orthographic: bool) -> &mut Self {
        self.orthographic = orthographic as i32;
        self
    }
}

pub use crate::bindgen::mjvGLCamera;
fields_mapping!(mjvGLCamera {
    scalars {
        pos / set_pos: [f32; 3] = "position";
        forward / set_forward: [f32; 3] = "forward direction";
        up / set_up: [f32; 3] = "up direction";
        frustum_center / set_frustum_center: f32 = "horizontal center (left,right set to match aspect)";
        frustum_width / set_frustum_width: f32 = "width (not used for rendering)";
        frustum_bottom / set_frustum_bottom: f32 = "bottom";
        frustum_top / set_frustum_top: f32 = "top";
        frustum_near / set_frustum_near: f32 = "near plane distance";
        frustum_far / set_frustum_far: f32 = "far plane distance";
    }
});
impl mjvGLCamera {
    pub fn orthographic(&self) -> bool {
        self.orthographic != 0
    }
}

pub use crate::bindgen::mjvGeom;
impl mjvGeom {
    pub fn new(type_: crate::bindgen::mjtGeom) -> Self {
        crate::mjv_initGeom(type_, None, None, None, None)
    }
}
fields_mapping!(mjvGeom {
    scalars {
        size: [f32; 3] = "size parameters";
        pos: [f32; 3] = "Cartesian position";
        mat: [f32; 9] = "Cartesian orientation";
        rgba: [f32; 4] = "color and transparency";
        emission: f32 = "emission coef";
        specular: f32 = "specular coef";
        shininess: f32 = "shininess coef";
        reflectance: f32 = "reflectance coef";
        camdist: f32 = "distance to camera (used by sorter)";
        modelrbound: f32 = "geom rbound from model, 0 if not model geom";
    }
    enums {
        type_: mjtGeom = "geom type";
        category: mjtCatBit = "visual category";
    }
});
impl mjvGeom {
    /// mesh, hfield or plane id
    pub fn dataid(&self) -> Option<ObjectId<obj::Mesh>> {
        if self.dataid >= 0 {Some(unsafe { ObjectId::new_unchecked(self.dataid as _) })} else {None}
    }
    /// object type; `mjtObj::UNKNOWN` for decor
    pub fn objtype(&self) -> mjtObj {
        mjtObj(self.objtype as u32)
    }
    /// object id; `None` for decor
    pub fn objid(&self) -> Option<usize> {
        if self.objid >= 0 {Some(self.objid as usize)} else {None}
    }
    /// material id
    pub fn matid(&self) -> Option<ObjectId<obj::Material>> {
        if self.matid >= 0 {Some(unsafe { ObjectId::new_unchecked(self.matid as _) })} else {None}
    }
    /// segmentation id
    pub fn segid(&self) -> Option<SegmentationId> {
        if self.segid >= 0 {Some(unsafe { SegmentationId::new_unchecked(self.segid as usize) })} else {None}
    }
    /// if mesh or flex geom has texture coordinates
    pub fn texcoord(&self) -> bool {
        self.texcoord != 0
    }
    /// if geom is transparent
    pub fn transparent(&self) -> bool {
        self.transparent != 0
    }
    /// text label
    pub fn label(&self) -> &str {
        (unsafe { std::ffi::CStr::from_ptr(self.label.as_ptr()) })
            .to_str()
            .expect("Invalid UTF-8 in label")
    }
}

pub use crate::bindgen::mjvLight;
fields_mapping!(mjvLight {
    scalars {
        pos: [f32; 3] = "position relative to body frame";
        dir: [f32; 3] = "direction relative to body frame";
        attenuation: [f32; 3] = "OpenGL attenuation (quadratic model)";
        cutoff: f32 = "OpenGL cutoff";
        exponent: f32 = "OpenGL exponent";
        ambient: [f32; 3] = "ambient rgb (alpha=1)";
        diffuse: [f32; 3] = "diffuse rgb (alpha=1)";
        specular: [f32; 3] = "specular rgb (alpha=1)";
        bulbradius: f32 = "bulb radius for soft shadows";
    }
});
impl mjvLight {
    pub fn headlight(&self) -> bool {self.headlight != 0}
    pub fn castshadow(&self) -> bool {self.castshadow != 0}
}

pub use crate::bindgen::mjvOption;
impl Default for mjvOption {
    fn default() -> Self {
        crate::mjv_defaultOption()
    }
}
fields_mapping!(mjvOption {
    scalars {
        bvh_depth / set_bvh_depth: usize = "depth of the bounding volume hierarchy to be visualized";
        flex_layer / set_flex_layer: usize = "element layer to be visualized for 3D flex";
    }
    enums {
        label / set_label: mjtLabel = "what objects to label";
        frame / set_frame: mjtFrame = "which frame to show";
    }
});
impl mjvOption {
    /// site visualization by group
    pub fn sitegroup(&self) -> [bool; mjNGROUP] {self.sitegroup.map(|b| b != 0)}
    /// joint visualization by group
    pub fn jointgroup(&self) -> [bool; mjNGROUP] {self.jointgroup.map(|b| b != 0)}
    /// tendon visualization by group
    pub fn tendongroup(&self) -> [bool; mjNGROUP] {self.tendongroup.map(|b| b != 0)}
    /// actuator visualization by group
    pub fn actuatorgroup(&self) -> [bool; mjNGROUP] {self.actuatorgroup.map(|b| b != 0)}
    /// flex visualization by group
    pub fn flexgroup(&self) -> [bool; mjNGROUP] {self.flexgroup.map(|b| b != 0)}
    /// skin visualization by group
    pub fn skingroup(&self) -> [bool; mjNGROUP] {self.skingroup.map(|b| b != 0)}
}
pub struct VisualizationFlags([u8; mjNVISFLAG]);
impl std::ops::Index<mjtVisFlag> for VisualizationFlags {
    type Output = bool;
    fn index(&self, index: mjtVisFlag) -> &Self::Output {
        &(unsafe { std::mem::transmute::<_, &[bool; mjNVISFLAG]>(&self.0) })[index.0 as usize]
    }
}
impl std::ops::IndexMut<mjtVisFlag> for VisualizationFlags {
    fn index_mut(&mut self, index: mjtVisFlag) -> &mut Self::Output {
        &mut (unsafe { std::mem::transmute::<_, &mut [bool; mjNVISFLAG]>(&mut self.0) })[index.0 as usize]
    }
}
impl mjvOption {
    /// geom visualization by group
    pub fn geomgroup(&self) -> [bool; mjNGROUP] {self.geomgroup.map(|b| b != 0)}
    /// mutate geom visualization by group
    pub fn geomgroup_mut(&mut self) -> &mut [bool; mjNGROUP] {
        unsafe { std::mem::transmute(&mut self.geomgroup) }
    }
    /// flags
    pub fn flags(&self) -> &VisualizationFlags {
        // SAFETY: just a newtype
        unsafe { std::mem::transmute(&self.flags) }
    }
    /// mutate flags
    pub fn flags_mut(&mut self) -> &mut VisualizationFlags {
        // SAFETY: just a newtype
        unsafe { std::mem::transmute(&mut self.flags) }
    }
}

resource_wrapper!(
    MjvScene for crate::bindgen::mjvScene;
    drop = crate::mjv_freeScene;
);
impl MjvScene {
    /// Create a new abstract scene with resources allocated for `maxgeom` geoms.
    /// 
    /// This internally calls:
    /// 
    /// 1. [`mjv_defaultScene`](crate::mjv_defaultScene) to set default values for the scene.
    /// 2. [`mjv_makeScene`](crate::mjv_makeScene) to allocate resources in the scene.
    pub fn new(model: &crate::MjModel, maxgeom: usize) -> Self {
        let mut scene = MjvScene::default();
        crate::mjv_makeScene(model, &mut scene, maxgeom);
        scene
    }
}
impl Default for MjvScene {
    /// Internally calls [`mjv_defaultScene`](mjv_defaultScene).
    /// 
    /// **note**: Be sure to call [`mjv_makeScene`] for the returned `mjvScene` to allocate resources in abstract scene
    ///           before using it in rendering.
    fn default() -> Self {
        crate::mjv_defaultScene()
    }
}
fields_mapping!(MjvScene {
    scalars {
        maxgeom: usize = "size of allocated geom buffer";
        ngeom: usize = "number of geoms currently in buffer";
        nskin: usize = "number of skins";
        nlight: usize = "number of lights currently in buffer";
        nflex: usize = "number of flexes";
        framewidth / set_framewidth: i32 = "frame pixel width; 0: disable framing";
        framergb / set_framergb: [f32; 3] = "frame color";
        translate / set_translate: [f32; 3] = "model translation";
        rotate / set_rotate: [f32; 4] = "model quaternion rotation";
        scale / set_scale: f32 = "model scaling";
    }
    enums {
        stereo / set_stereo: mjtStereo = "stereoscopic rendering";
    }
    structs {
        lights: [mjvLight; mjMAXLIGHT] = "buffer for lights (nlight)";
        camera: [mjvGLCamera; 2] = "left and right camera";
    }
});
impl MjvScene {
    fn nflexedge(&self) -> usize {self.flexedgenum().iter().sum::<i32>() as usize}
    fn nflexface(&self) -> usize {self.flexfacenum().iter().sum::<i32>() as usize}
    fn nflexvert(&self) -> usize {self.flexvertnum().iter().sum::<i32>() as usize}
    fn nskinvert(&self) -> usize {self.skinvertnum().iter().sum::<i32>() as usize}
}
macro_rules! buffer_slices {
    ($($name:ident : [$T:ty; $size:ident $(* $mul:literal)?] = $description:literal;)*) => {
        impl MjvScene {
            $(
                #[doc = $description]
                pub fn $name(&self) -> &[$T] {
                    unsafe { std::slice::from_raw_parts(self.$name, self.$size() $(* $mul)?) }
                }
            )*
        }
    };
}
buffer_slices! {
    geoms: [mjvGeom; ngeom] = "buffer for geoms (ngeom)";
    geomorder: [i32; ngeom] = "buffer for ordering geoms by distance to camera (ngeom)";
    flexedgeadr: [i32; nflex] = "address of flex edges (nflex)";
    flexedgenum: [i32; nflex] = "number of edges in flex (nflex)";
    flexvertadr: [i32; nflex] = "address of flex vertices (nflex)";
    flexvertnum: [i32; nflex] = "number of vertices in flex (nflex)";
    flexfaceadr: [i32; nflex] = "address of flex faces (nflex)";
    flexfacenum: [i32; nflex] = "number of flex faces allocated (nflex)";
    flexfaceused: [i32; nflex] = "number of flex faces currently in use (nflex)";
    skinfacenum: [i32; nskin] = "number of faces in skin (nskin)";
    skinvertadr: [i32; nskin] = "address of skin vertices (nskin)";
    skinvertnum: [i32; nskin] = "number of vertices in skin (nskin)";
    flexedge: [i32; nflexedge * 2] = "flex edge data (2*nflexedge)";
    skinvert: [f32; nskinvert * 3] = "skin vertex data (3*nskinvert)";
    skinnormal: [f32; nskinvert * 3] = "skin normal data (3*nskinvert)";
    flexvert: [f32; nflexvert * 3] = "flex vertices (3*nflexvert)";
    flexface: [f32; nflexface * 9] = "flex faces vertices (9*sum(flexfacenum))";
    flexnormal: [f32; nflexface * 9] = "flex face normals (9*sum(flexfacenum))";
    flextexcoord: [f32; nflexface * 6] = "flex face texture coordinates (6*sum(flexfacenum))";
}
pub struct RenderingFlags([u8; mjNRNDFLAG as usize]);
impl std::ops::Index<mjtRndFlag> for RenderingFlags {
    type Output = bool;
    fn index(&self, index: mjtRndFlag) -> &Self::Output {
        &(unsafe { std::mem::transmute::<_, &[bool; mjNRNDFLAG as usize]>(&self.0) })[index.0 as usize]
    }
}
impl std::ops::IndexMut<mjtRndFlag> for RenderingFlags {
    fn index_mut(&mut self, index: mjtRndFlag) -> &mut Self::Output {
        &mut (unsafe { std::mem::transmute::<_, &mut [bool; mjNRNDFLAG as usize]>(&mut self.0) })[index.0 as usize]
    }
}
impl MjvScene {
    /// copy of mjVIS_FLEXVERT mjvOption flag
    pub fn flexvertopt(&self) -> bool {self.flexvertopt != 0}
    /// copy of mjVIS_FLEXEDGE mjvOption flag
    pub fn flexedgeopt(&self) -> bool {self.flexedgeopt != 0}
    /// copy of mjVIS_FLEXFACE mjvOption flag
    pub fn flexfaceopt(&self) -> bool {self.flexfaceopt != 0}
    /// copy of mjVIS_FLEXSKIN mjvOption flag
    pub fn flexskinopt(&self) -> bool {self.flexskinopt != 0}

    /// enable model transformation
    pub fn enabletransform(&self) -> bool {self.enabletransform != 0}
    /// set enable model transformation
    pub fn set_enabletransform(&mut self, enabletransform: bool) -> &mut Self {
        self.enabletransform = enabletransform as u8; self
    }

    /// rendering flags
    pub fn flags(&self) -> &RenderingFlags {
        // SAFETY: just a newtype
        unsafe { std::mem::transmute(&self.flags) }
    }
    /// mutate rendering flags
    pub fn flags_mut(&mut self) -> &mut RenderingFlags {
        // SAFETY: just a newtype
        unsafe { std::mem::transmute(&mut self.flags) }
    }
}

pub use crate::bindgen::mjvFigure;
impl Default for mjvFigure {
    fn default() -> Self {
        crate::mjv_defaultFigure()
    }
}
fields_mapping!(mjvFigure {
    boolean_flags {
        flg_legend / set_flg_legend = "show legend";
        flg_extend / set_flg_extend = "automatically extend axis ranges to fit data";
        flg_barplot / set_flg_barplot = "isolated line segments (i.e. GL_LINES)";
        flg_selection / set_flg_selection = "vertical selection line";
        flg_symmetric / set_flg_symmetric = "symmetric y-axis";
    }
    scalars {
        gridsize: [i32; 2] = "number of grid points in (x,y)";
        legendoffset / set_legendoffset: i32 = "number of lines to offset legend";
        subplot / set_subplot: i32 = "selected subplot (for title rendering)";
        highlight: [i32; 2] = "if point is in legend rect, highlight line";
        highlightid / set_highlightid: i32 = "if id>=0 and no point, highlight id";
        linewidth / set_linewidth: f32 = "line width";
        gridwidth / set_gridwidth: f32 = "grid line width";
        gridrgb / set_gridrgb: [f32; 3] = "grid line rgb";
        figurergba / set_figurergba: [f32; 4] = "figure color and alpha";
        panergba / set_panergba: [f32; 4] = "pane color and alpha";
        legendrgba / set_legendrgba: [f32; 4] = "legend color and alpha";
        textrgb / set_textrgb: [f32; 3] = "text color";
        selection / set_selection: f32 = "selection line x-value";
        range / set_range: [[f32; 2]; 2] = "axis ranges (x,y) in data units; (min>=max) automatic";
    }
    structs {
        linepnt / set_linepnt: [i32; mjMAXLINE] = "number of points in line; (0) disable";
        linergb / set_linergb: [[f32; 3]; mjMAXLINE] = "line colors (r,g,b) for legend";
    }
});
macro_rules! chars {
    ($($name:ident / $set_name:ident = $description:literal;)*) => {
        impl mjvFigure {
            $(
                #[doc = $description]
                pub fn $name(&self) -> &str {
                    (unsafe { std::ffi::CStr::from_ptr(self.$name.as_ptr()) })
                        .to_str().expect(concat!("Invalid UTF-8 in " , stringify!($name)))
                }
                #[doc = "set "]
                #[doc = $description]
                pub fn $set_name(&mut self, $name: &str) -> &mut Self {
                    let bytes = $name.as_bytes();
                    let (len, limit) = (bytes.len(), self.$name.len() - 1);
                    assert!(len <= limit, "{}: `{len}` is too long, max {limit} bytes", stringify!($name));
                    bytes.iter().enumerate().for_each(|(i, &b)| self.$name[i] = b as i8);
                    self.$name[bytes.len()] = 0; // null-terminate
                    self
                }
            )*
        }
    };
}
chars! {
    xformat / set_xformat = "x-tick label format for sprintf";
    yformat / set_yformat = "y-tick label format for sprintf";
    minwidth / set_minwidth = "string used to determine min y-tick width";
    title / set_title = "figure title; subplots separated with 2+ spaces";
    xlabel / set_xlabel = "x-axis label";
}
impl mjvFigure {
    /// line name for legend of index `index` \in `0..mjMAXLINE`
    pub fn linename(&self, index: usize) -> &str {
        (unsafe { std::ffi::CStr::from_ptr(self.linename[index].as_ptr()) }).to_str().expect("Invalid UTF-8 in line name")
    }
    /// set line name for legend of index `index` \in `0..mjMAXLINE`
    pub fn set_linename(&mut self, index: usize, name: &str) -> &mut Self {
        assert!(index < mjMAXLINE, "`set_linename`: too large index `{index}`, must be index < {mjMAXLINE}");
        let (len, limit) = (name.as_bytes().len(), self.linename[index].len() - 1);
        assert!(len <= limit, "Line name `{name}` is too long, max {limit} bytes");
        name.as_bytes().iter().enumerate().for_each(|(i, &b)| self.linename[index][i] = b as i8);
        self.linename[index][len] = 0; // null-terminate
        self
    }

    /// show grid tick labels (x,y)
    pub fn flg_ticklabel(&self) -> [bool; 2] {
        [self.flg_ticklabel[0] != 0, self.flg_ticklabel[1] != 0]
    }
    /// set show grid tick labels (x,y)
    pub fn set_flg_ticklabel(&mut self, x: bool, y: bool) -> &mut Self {
        self.flg_ticklabel[0] = x as i32;
        self.flg_ticklabel[1] = y as i32;
        self
    }

    /// range of x-axis in pixels
    pub fn xaxispixel(&self) -> std::ops::Range<usize> {
        self.xaxispixel[0] as usize..self.xaxispixel[1] as usize
    }
    /// range of y-axis in pixels
    pub fn yaxispixel(&self) -> std::ops::Range<usize> {
        self.yaxispixel[0] as usize..self.yaxispixel[1] as usize
    }
    /// range of x-axis in data units
    pub fn xaxisdata(&self) -> std::ops::Range<f32> {
        self.xaxisdata[0]..self.xaxisdata[1]
    }
    /// range of y-axis in data units
    pub fn yaxisdata(&self) -> std::ops::Range<f32> {
        self.yaxisdata[0]..self.yaxisdata[1]
    }
}

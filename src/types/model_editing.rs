//! # [Model Editing](https://mujoco.readthedocs.io/en/stable/APIreference/APItypes.html#model-editing)
//! 
//! The structs below are defined in mjspec.h and, with the exception of
//! the top level [`mjSpec`] struct, begin with the `mjs` prefix.
//! For more details, see the [Model Editing](https://mujoco.readthedocs.io/en/stable/programming/modeledit.html) chapter.

pub use crate::bindgen::{
    mjSpec, mjsElement, mjsCompiler,
    mjtInertiaFromGeom,
};

use crate::bindgen::{mjOption, mjVisual, mjStatistic, mjString, mjLROpt};

derive_fields_mapping!(mjSpec {
    boolean_flags {
        strippath / set_strippath = "automatically strip paths from mesh files";
        hasImplicitPluginElem = "already encountered an implicit plugin sensor/actuator";
    }
    scalars {
        memory / set_memory: usize = "number of bytes in arena+stack memory";
        nemax / set_nemax: usize = "max number of equality constraints";
        nuserdata / set_nuserdata: usize = "number of mjtNums in userdata";
        nuser_body / set_nuser_body: usize = "number of mjtNums in body_user";
        nuser_jnt / set_nuser_jnt: usize = "number of mjtNums in jnt_user";
        nuser_geom / set_nuser_geom: usize = "number of mjtNums in geom_user";
        nuser_site / set_nuser_site: usize = "number of mjtNums in site_user";
        nuser_cam / set_nuser_cam: usize = "number of mjtNums in cam_user";
        nuser_tendon / set_nuser_tendon: usize = "number of mjtNums in tendon_user";
        nuser_actuator / set_nuser_actuator: usize = "number of mjtNums in actuator_user";
        nuser_sensor / set_nuser_sensor: usize = "number of mjtNums in sensor_user";
        nkey / set_nkey: usize = "number of keyframes";
    }
    structs {
        compiler / compiler_mut: mjsCompiler = "compiler options";
        option / option_mut: mjOption = "physics options";
        visual / visual_mut: mjVisual = "visual options";
        stat / stat_mut: mjStatistic = "statistics override (if defined)";
    }
});
impl mjSpec {
    /// element type, do not modify
    pub fn element(&self) -> &mjsElement {unsafe { &*self.element }}

    /// model name
    pub fn modelname(&self) -> Option<&mjString> {
        if self.modelname.is_null() {None} else {Some(unsafe { &*self.modelname })}
    }
    /// mutable model name
    pub fn modelname_mut(&mut self) -> Option<&mut mjString> {
        if self.modelname.is_null() {None} else {Some(unsafe { &mut *self.modelname })}
    }

    /// comment at top of XML
    pub fn comment(&self) -> Option<&mjString> {
        if self.comment.is_null() {None} else {Some(unsafe { &*self.comment })}
    }
    /// mutable comment at top of XML
    pub fn comment_mut(&mut self) -> Option<&mut mjString> {
        if self.comment.is_null() {None} else {Some(unsafe { &mut *self.comment })}
    }

    /// path to model file
    pub fn modelfiledir(&self) -> Option<&mjString> {
        if self.modelfiledir.is_null() {None} else {Some(unsafe { &*self.modelfiledir })}
    }
    /// mutable path to model file
    pub fn modelfiledir_mut(&mut self) -> Option<&mut mjString> {
        if self.modelfiledir.is_null() {None} else {Some(unsafe { &mut *self.modelfiledir })}
    }

    /// mesh and hfield directory
    pub fn meshdir(&self) -> Option<&mjString> {
        if self.meshdir.is_null() {None} else {Some(unsafe { &*self.meshdir })}
    }
    /// mutable mesh and hfield directory
    pub fn meshdir_mut(&mut self) -> Option<&mut mjString> {
        if self.meshdir.is_null() {None} else {Some(unsafe { &mut *self.meshdir })}
    }

    /// texture directory
    pub fn texturedir(&self) -> Option<&mjString> {
        if self.texturedir.is_null() {None} else {Some(unsafe { &*self.texturedir })}
    }
    /// mutable texture directory
    pub fn texturedir_mut(&mut self) -> Option<&mut mjString> {
        if self.texturedir.is_null() {None} else {Some(unsafe { &mut *self.texturedir })}
    }
}

impl mjsElement {
    /// element type
    pub fn elemtype(&self) -> crate::mjtObj {
        self.elemtype
    }
    /// compilation signature
    pub fn signature(&self) -> u64 {
        self.signature
    }
}

derive_fields_mapping!(mjsCompiler {
    boolean_flags {
        autolimits / set_autolimits = "infer 'limited' attribute based on range";
        balanceinertia / set_balanceinertia = "automatically impose A + B >= C rule";
        fitaabb / set_fitaabb = "mesh fit to AABB instead of inertia box";
        degree / set_degree = "angles in radians or degrees";
        saveinertial / set_saveinertial = "save explicit inertial clause for all bodies to XML";
        discardvisual / set_discardvisual = "discard visual geoms in parser";
        fusestatic / set_fusestatic = "fuse static bodies with parent";
        usethread / set_usethread = "use multiple threads to speed up compiler";
    }
    scalars {
        boundmass / set_boundmass: f64 = "enforce minimum body mass";
        boundinertia / set_boundinertia: f64 = "enforce minimum body diagonal inertia";
        settotalmass / set_settotalmass: f64 = "rescale masses and inertias; <=0: ignore";
        inertiagrouprange / set_inertiagrouprange: [i32; 2] = "range of geom groups used to compute inertia";
        alignfree / set_alignfree: i32 = "align free joints with inertial frame";
    }
    enums {
        inertiafromgeom / set_inertiafromgeom: mjtInertiaFromGeom = "use geom inertias (mjtInertiaFromGeom)";
    }
    structs {
        LRopt / LRopt_mut: mjLROpt = "options for lengthrange computation";
    }
});
impl mjsCompiler {
    /// sequence for Euler rotations
    pub fn eulerseq(&self) -> [char; 3] {
        self.eulerseq.map(|i8| u8::try_from(i8).expect("unexpected char in `eulerseq`") as char)
    }
    /// set sequence for Euler rotations (**ASCII characters**)
    pub fn set_eulerseq(&mut self, seq: [char; 3]) -> &mut Self {
        assert!(seq.iter().all(char::is_ascii), "eulerseq must contain ASCII characters only");
        self.eulerseq = seq.map(|c| (c as u8).try_into().expect("unexpected char in `eulerseq`"));
        self
    }
}
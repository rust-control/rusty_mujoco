macro_rules! basic_accessors {
    ($T:ty {
        $(
            $fieldname:ident: $F:ty $(| $set:ident($S:ty) -> $R:ty)?,
        )*
    }) => {
        impl $T {
            $(
                #[inline(always)]
                pub fn $fieldname(&self) -> $F {
                    crate::util::AccessAs::<$F>::access_as(&self.$fieldname)
                }
                $(
                    #[inline(always)]
                    pub fn $set(&mut self, $fieldname: $S) -> $R {
                        crate::util::SetBy::<$S, $R>::set_by(&mut self.$fieldname, $fieldname)
                    }
                )?
            )*
        }
    };
}
macro_rules! slice_accessors {
    (@ $fieldname:ident: [$I:ty; $n:ident $(* $X:literal)?]) => {
        #[inline(always)]
        pub fn $fieldname(&self) -> &[$I] {
            unsafe { std::slice::from_raw_parts(self.$fieldname, (self.$n as usize) $(* $X)?) }
        }
    };
    (@ $fieldname:ident | $fieldname_mut:ident: [$I:ty; $n:ident $(* $X:literal)?]) => {
        #[inline(always)]
        pub fn $fieldname(&self) -> &[$I] {
            unsafe { std::slice::from_raw_parts(self.$fieldname, (self.$n as usize) $(* $X)?) }
        }
        #[inline(always)]
        pub fn $fieldname_mut(&mut self) -> &mut [$I] {
            unsafe { std::slice::from_raw_parts_mut(self.$fieldname, (self.$n as usize) $(* $X)?) }
        }
    };
    ($T:ty {
        $(
            $fieldname:ident $(| $fieldname_mut:ident)?: [$I:ty; $n:ident $(* $X:literal)?],
        )*
    }) => {
        impl $T {
            $(
                slice_accessors!(@ $fieldname $(| $fieldname_mut)?: [$I; $n $(* $X)?]);
            )*
        }
    };
}

use crate::{ObjectId, obj};
use std::slice::from_raw_parts as slice_from_raw;

trait IntExt: Sized {
    fn when_nonnegative<T>(self, f: impl FnOnce(Self) -> T) -> Option<T>;
    unsafe fn to_objectid_unchecked<B: crate::Obj>(self) -> ObjectId<B>;
}
impl IntExt for i32 {
    #[inline(always)]
    fn when_nonnegative<T>(self, f: impl FnOnce(Self) -> T) -> Option<T> {
        if self >= 0 { Some(f(self)) } else { None }
    }
    #[inline(always)]
    unsafe fn to_objectid_unchecked<B: crate::Obj>(self) -> ObjectId<B> {
        unsafe { ObjectId::new_unchecked(self as usize) }
    }
}

basic_accessors!(crate::mjLogMessage {
    subject: Result<&str, std::str::Utf8Error>,
    body: Result<&str, std::str::Utf8Error>,
    func: Result<&str, std::str::Utf8Error>,
    file: Result<&str, std::str::Utf8Error>,
});
basic_accessors!(crate::mjLogConfig {
    logfile: Result<&str, std::str::Utf8Error> | set_logfile(String) -> crate::Result<()>,
});
impl Drop for crate::mjModel {
    fn drop(&mut self) {
        unsafe { crate::functions::mj_deleteModel(self); }
    }
}
impl crate::mjModel {
    pub fn qpos0(&self) -> &[f64] {
        unsafe { slice_from_raw(self.qpos0, (self.nq as usize) * 1) }
    }
    pub fn qpos_spring(&self) -> &[f64] {
        unsafe { slice_from_raw(self.qpos0, (self.nq as usize) * 1) }
    }
    pub fn body_parentid(&self, id: ObjectId<obj::Body>) -> ObjectId<obj::Body> {
        unsafe { self.body_parentid.add(id.index()).read().to_objectid_unchecked() }
    }
    pub fn body_rootid(&self, id: ObjectId<obj::Body>) -> ObjectId<obj::Body> {
        unsafe { self.body_rootid.add(id.index()).read().to_objectid_unchecked() }
    }
    pub fn body_weldid(&self, id: ObjectId<obj::Body>) -> ObjectId<obj::Body> {
        unsafe { self.body_weldid.add(id.index()).read().to_objectid_unchecked() }
    }
    pub fn body_mocapid(&self, id: ObjectId<obj::Body>) -> Option<ObjectId<obj::Body>> {
        unsafe { self.body_mocapid.add(id.index()).read().when_nonnegative(|n| n.to_objectid_unchecked()) }
    }
    pub fn body_jntnum(&self, id: ObjectId<obj::Body>) -> usize {
        unsafe { self.body_jntnum.add(id.index()).read() as usize }
    }
    pub fn body_jntadr(&self, id: ObjectId<obj::Body>) -> Option<usize> {
        unsafe { self.body_jntadr.add(id.index()).read().when_nonnegative(|n| n as usize) }
    }
    pub fn body_dofnum(&self, id: ObjectId<obj::Body>) -> usize {
        unsafe { self.body_dofnum.add(id.index()).read() as usize }
    }
    pub fn body_dofadr(&self, id: ObjectId<obj::Body>) -> Option<usize> {
        unsafe { self.body_dofadr.add(id.index()).read().when_nonnegative(|n| n as usize) }
    }
    pub fn body_treeid(&self, id: ObjectId<obj::Body>) -> Option<ObjectId<obj::Body>> {
        unsafe { self.body_treeid.add(id.index()).read().when_nonnegative(|n| n.to_objectid_unchecked()) }
    }
    pub fn body_geomnum(&self, id: ObjectId<obj::Body>) -> usize {
        unsafe { self.body_geomnum.add(id.index()).read() as usize }
    }
    pub fn body_geomadr(&self, id: ObjectId<obj::Body>) -> Option<usize> {
        unsafe { self.body_geomadr.add(id.index()).read().when_nonnegative(|n| n as usize) }
    }
    pub fn body_simple(&self, id: ObjectId<obj::Body>) -> u8 {
        unsafe { self.body_simple.add(id.index()).read() }
    }
    pub fn body_sameframe(&self, id: ObjectId<obj::Body>) -> crate::mjtSameFrame {
        crate::mjtSameFrame(unsafe { self.body_sameframe.add(id.index()).read() as u32 })
    }
    pub fn body_pos(&self, id: ObjectId<obj::Body>) -> [f64; 3] {
        unsafe { *slice_from_raw(self.body_pos.add(id.index() * 3), 3).as_array().unwrap_unchecked() }
    }
    pub fn body_quat(&self, id: ObjectId<obj::Body>) -> [f64; 4] {
        unsafe { *slice_from_raw(self.body_quat.add(id.index() * 4), 4).as_array().unwrap_unchecked() }
    }
    pub fn body_ipos(&self, id: ObjectId<obj::Body>) -> [f64; 3] {
        unsafe { *slice_from_raw(self.body_ipos.add(id.index() * 3), 3).as_array().unwrap_unchecked() }
    }
    pub fn body_iquat(&self, id: ObjectId<obj::Body>) -> [f64; 4] {
        unsafe { *slice_from_raw(self.body_iquat.add(id.index() * 4), 4).as_array().unwrap_unchecked() }
    }
    pub fn body_mass(&self, id: ObjectId<obj::Body>) -> f64 {
        unsafe { self.body_mass.add(id.index()).read() }
    }
    pub fn body_subtreemass(&self, id: ObjectId<obj::Body>) -> f64 {
        unsafe { self.body_subtreemass.add(id.index()).read() }
    }
    pub fn body_inertia(&self, id: ObjectId<obj::Body>) -> [f64; 3] {
        unsafe { *slice_from_raw(self.body_inertia.add(id.index() * 3), 3).as_array().unwrap_unchecked() }
    }
    pub fn body_invweight0(&self, id: ObjectId<obj::Body>) -> [f64; 2] {
        unsafe { *slice_from_raw(self.body_invweight0.add(id.index() * 2), 2).as_array().unwrap_unchecked() }
    }
    pub fn body_gravcomp(&self, id: ObjectId<obj::Body>) -> f64 {
        unsafe { self.body_gravcomp.add(id.index()).read() }
    }
    pub fn body_margin(&self, id: ObjectId<obj::Body>) -> f64 {
        unsafe { self.body_margin.add(id.index()).read() }
    }
    pub fn body_user(&self, id: ObjectId<obj::Body>) -> &[f64] {
        unsafe { slice_from_raw(self.body_user.add(id.index() * self.nuser_body as usize), self.nuser_body as usize) }
    }
    pub fn body_plugin(&self, id: ObjectId<obj::Body>) -> Option<ObjectId<obj::Plugin>> {
        unsafe { self.body_plugin.add(id.index()).read().when_nonnegative(|n| n.to_objectid_unchecked()) }
    }
    pub fn body_contype(&self, id: ObjectId<obj::Body>) -> i32 {
        unsafe { self.body_contype.add(id.index()).read() }
    }
    pub fn body_conaffinity(&self, id: ObjectId<obj::Body>) -> i32 {
        unsafe { self.body_conaffinity.add(id.index()).read() }
    }
    pub fn body_bvhadr(&self, id: ObjectId<obj::Body>) -> usize {
        unsafe { self.body_bvhadr.add(id.index()).read() as usize }
    }
    pub fn body_bvhnum(&self, id: ObjectId<obj::Body>) -> usize {
        unsafe { self.body_bvhnum.add(id.index()).read() as usize }
    }
    pub fn bvh_depth(&self, id: usize) -> usize {
        (unsafe { slice_from_raw(self.bvh_depth, self.nbvh as usize) })[id] as usize
    }
    pub fn bvh_child(&self, id: usize) -> (usize, usize) {
        let s = unsafe { slice_from_raw(self.bvh_child, self.nbvh as usize * 2) };
        (s[id * 2] as usize, s[id * 2 + 1] as usize)
    }
    pub fn bvh_nodeid(&self, id: usize) -> Option<usize> {
        unsafe { self.bvh_nodeid.add(id).read().when_nonnegative(|n| n as usize) }
    }
    pub fn bvh_aabb(&self, id: usize) -> ([f64; 3], [f64; 3]) {
        let s = unsafe { slice_from_raw(self.bvh_aabb.add(id * 6), self.nbvhstatic as usize * 6) };
        ([s[0], s[1], s[2]], [s[3], s[4], s[5]])
    }
}

//! # [Ray casting](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#ray-casting)
//! 
//! Ray collisions, also known as ray casting, find the distance `x`
//! of a ray’s intersection with a geom, where a ray is a line emanating from
//! the 3D point `p` in the direction `v` i.e., `(p + x*v, x >= 0)`.
//! 
//! All functions in this family return the distance to the nearest geom surface,
//! or -1 if there is no intersection. Note that if `p` is inside a geom,
//! the ray will intersect the surface from the inside which still counts as
//! an intersection.
//! 
//! All ray collision functions rely on quantities computed by
//! [`mj_kinematics`](crate::mj_kinematics) (see [`mjData`](crate::MjData)),
//! so must be called after `mj_kinematics`, or functions that call it
//! (e.g. [`mj_fwdPosition`](crate::mj_fwdPosition)). The top level functions,
//! which intersect with all geoms types, are [`mj_ray`](crate::mj_ray)
//! which casts a single ray, and [`mj_multiRay`](crate::mj_multiRay)
//! which casts multiple rays from a single point.

use super::helper::array_flatslice;
use crate::{obj, ObjectId, VertexId};

/// Intersect multiple rays emanating from a single point.
/// Similar semantics to [`mj_ray`](crate::mj_ray), but `vec` is an array of
/// `(nray x 3)` directions.
/* void mj_multiRay(const mjModel* m, mjData* d, const mjtNum pnt[3], const mjtNum* vec,
                 const mjtByte* geomgroup, mjtByte flg_static, int bodyexclude,
                 int* geomid, mjtNum* dist, int nray, mjtNum cutoff); */
pub fn mj_multiRay<const N_RAY: usize>(
    m: &crate::MjModel,
    d: &mut crate::MjData,
    pnt: [f64; 3],
    vec: [[f64; 3]; N_RAY],
    geomgroup: Option<&[u8]>,
    flg_static: bool,
    bodyexclude: i32,
    cutoff: f64,
) -> Vec<Option<(ObjectId::<obj::Geom>, f64)>> {
    let vec: &[f64] = array_flatslice(&vec);

    let mut geomid = vec![-1; N_RAY];
    let mut dist = vec![-1.0; N_RAY];

    unsafe {
        crate::bindgen::mj_multiRay(
            m.as_ref(),
            d.as_mut(),
            pnt.as_ptr(),
            vec.as_ptr(),
            geomgroup.map_or(std::ptr::null(), |g| g.as_ptr()),
            flg_static as u8,
            bodyexclude,
            geomid.as_mut_ptr(),
            dist.as_mut_ptr(),
            N_RAY as i32,
            cutoff,
        )
    }

    Iterator::zip(geomid.into_iter(), dist.into_iter()).map(|(geomid, dist)| {
        if geomid < 0 || dist < 0.0 {
            None
        } else {
            Some((ObjectId::new(geomid as usize), dist))
        }
    }).collect()
}

/// Intersect ray `(pnt + x * vec, x >= 0)` with visible geoms, except geoms in `bodyexclude`.
/// 
/// Return `geomid` and distance (`x`) to nearest surface, or `None` if no intersection.
/// 
/// `geomgroup` is an array of length `mjNGROUP`, where `1` means the group should be included.
/// Pass `geomgroup = None` to skip group exclusion.
/// 
/// If `flg_static` is `false`, static geoms will be excluded.
/// 
/// `bodyexclude = None` can be used to indicate that all bodies are included.
/* mjtNum mj_ray(const mjModel* m, const mjData* d, const mjtNum pnt[3], const mjtNum vec[3],
              const mjtByte* geomgroup, mjtByte flg_static, int bodyexclude,
              int geomid[1]); */
pub fn mj_ray(
    m: &crate::MjModel,
    d: &mut crate::MjData,
    pnt: [f64; 3],
    vec: [f64; 3],
    geomgroup: Option<[bool; crate::bindgen::mjNGROUP as usize]>,
    flg_static: bool,
    bodyexclude: ObjectId<obj::Body>,
) -> Option<(ObjectId<obj::Geom>, f64)> {
    let geomgroup = geomgroup.map(|geomgroup| geomgroup.map(|bool| bool as u8));

    let mut geomid = [-1];
    
    let distance = unsafe {
        crate::bindgen::mj_ray(
            m.as_ref(),
            d.as_mut(),
            pnt.as_ptr(),
            vec.as_ptr(),
            geomgroup.map_or(std::ptr::null(), |gg| gg.as_ptr()),
            flg_static as u8,
            bodyexclude.index() as i32,
            geomid.as_mut_ptr(),
        )
    };
    
    if geomid[0] < 0 || distance < 0.0 {
        None
    } else {
        Some((ObjectId::new(geomid[0] as usize), distance))
    }
}

/// Intersect ray with hfield, return nearest distance or `None` if no intersection.
/* mjtNum mj_rayHfield(const mjModel* m, const mjData* d, int geomid,
                    const mjtNum pnt[3], const mjtNum vec[3]); */
pub fn mj_rayHfield(
    m: &crate::MjModel,
    d: &mut crate::MjData,
    geomid: ObjectId<obj::Geom>,
    pnt: [f64; 3],
    vec: [f64; 3],
) -> Option<f64> {
    let distance = unsafe {
        crate::bindgen::mj_rayHfield(
            m.as_ref(),
            d.as_mut(),
            geomid.index() as i32,
            pnt.as_ptr(),
            vec.as_ptr(),
        )
    };
    
    if distance < 0.0 {
        None
    } else {
        Some(distance)
    }
}

/// Intersect ray with mesh, return nearest distance or `None` if no intersection.
/* mjtNum mj_rayMesh(const mjModel* m, const mjData* d, int geomid,
                  const mjtNum pnt[3], const mjtNum vec[3]); */
pub fn mj_rayMesh(
    m: &crate::MjModel,
    d: &mut crate::MjData,
    geomid: ObjectId<obj::Geom>,
    pnt: [f64; 3],
    vec: [f64; 3],
) -> Option<f64> {
    let distance = unsafe {
        crate::bindgen::mj_rayMesh(
            m.as_ref(),
            d.as_mut(),
            geomid.index() as i32,
            pnt.as_ptr(),
            vec.as_ptr(),
        )
    };
    
    if distance < 0.0 {
        None
    } else {
        Some(distance)
    }
}

/// Intersect ray with pure geom, return nearest distance or `None` if no intersection.
/* mjtNum mju_rayGeom(const mjtNum pos[3], const mjtNum mat[9], const mjtNum size[3],
                   const mjtNum pnt[3], const mjtNum vec[3], int geomtype); */
pub fn mju_rayGeom(
    pos: [f64; 3],
    mat: [f64; 9],
    size: [f64; 3],
    pnt: [f64; 3],
    vec: [f64; 3],
    geomtype: crate::bindgen::mjtGeom,
) -> Option<f64> {
    let distance = unsafe {
        crate::bindgen::mju_rayGeom(
            pos.as_ptr(),
            mat.as_ptr(),
            size.as_ptr(),
            pnt.as_ptr(),
            vec.as_ptr(),
            geomtype as i32,
        )
    };
    
    if distance < 0.0 {
        None
    } else {
        Some(distance)
    }
}

/// Intersect ray with flex, return nearest distance or `None` if no intersection,
/// and also output nearest vertex id.
/* mjtNum mju_rayFlex(const mjModel* m, const mjData* d, int flex_layer, mjtByte flg_vert,
                   mjtByte flg_edge, mjtByte flg_face, mjtByte flg_skin, int flexid,
                   const mjtNum* pnt, const mjtNum* vec, int vertid[1]); */
pub fn mju_rayFlex(
    m: &crate::MjModel,
    d: &mut crate::MjData,
    flex_layer: Option<usize>,
    flg_vert: bool,
    flg_edge: bool,
    flg_face: bool,
    flg_skin: bool,
    flexid: ObjectId<obj::Flex>,
    pnt: [f64; 3],
    vec: [f64; 3],
) -> Option<(VertexId, f64)> {
    let mut vertid = [-1];
    
    let distance = unsafe {
        crate::bindgen::mju_rayFlex(
            m.as_ref(),
            d.as_mut(),
            flex_layer.map_or(-1, |l| l as i32),
            flg_vert as u8,
            flg_edge as u8,
            flg_face as u8,
            flg_skin as u8,
            flexid.index() as i32,
            pnt.as_ptr(),
            vec.as_ptr(),
            vertid.as_mut_ptr(),
        )
    };
    
    if vertid[0] < 0 || distance < 0.0 {
        None
    } else {
        Some((VertexId(vertid[0] as usize), distance))
    }
}

/// Intersect ray with skin, return nearest distance or `None` if no intersection,
/// and also output nearest vertex id.
/* mjtNum mju_raySkin(int nface, int nvert, const int* face, const float* vert,
                   const mjtNum pnt[3], const mjtNum vec[3], int vertid[1]); */
pub fn mju_raySkin<const N_FACE: usize, const N_VERT: usize>(
    face_indices: [[usize; 3]; N_FACE],
    vert_coordinates: [[f32; 3]; N_VERT],
    pnt: [f64; 3],
    vec: [f64; 3],
) -> Option<(VertexId, f64)> {
    let vert_coordinates: &[f32] = array_flatslice(&vert_coordinates);
    let face_indices: &[usize] = array_flatslice(&face_indices);
    let face_indices: Vec<i32> = face_indices.iter().map(|&i| i as i32).collect();

    let mut vertid = [-1];
    
    let distance = unsafe {
        crate::bindgen::mju_raySkin(
            N_FACE as i32,
            N_VERT as i32,
            face_indices.as_ptr(),
            vert_coordinates.as_ptr(),
            pnt.as_ptr(),
            vec.as_ptr(),
            vertid.as_mut_ptr(),
        )
    };
    
    if vertid[0] < 0 || distance < 0.0 {
        None
    } else {
        Some((VertexId(vertid[0] as usize), distance))
    }
}

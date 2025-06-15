use crate::bindgen::{mjNREF, mjNIMP};

/// This is the data structure holding information about one contact.
/// mjData.contact is a preallocated array of mjContact data structures,
/// populated at runtime with the contacts found by the collision detector.
/// Additional contact information is then filled-in by the simulator.
pub struct MjContact(pub(super) crate::bindgen::mjContact);

/*
struct mjContact_ {                // result of collision detection functions
  // contact parameters set by near-phase collision function
  mjtNum  dist;                    // distance between nearest points; neg: penetration
  mjtNum  pos[3];                  // position of contact point: midpoint between geoms
  mjtNum  frame[9];                // normal is in [0-2], points from geom[0] to geom[1]

  // contact parameters set by mj_collideGeoms
  mjtNum  includemargin;           // include if dist<includemargin=margin-gap
  mjtNum  friction[5];             // tangent1, 2, spin, roll1, 2
  mjtNum  solref[mjNREF];          // constraint solver reference, normal direction
  mjtNum  solreffriction[mjNREF];  // constraint solver reference, friction directions
  mjtNum  solimp[mjNIMP];          // constraint solver impedance

  // internal storage used by solver
  mjtNum  mu;                      // friction of regularized cone, set by mj_makeConstraint
  mjtNum  H[36];                   // cone Hessian, set by mj_constraintUpdate

  // contact descriptors set by mj_collideXXX
  int     dim;                     // contact space dimensionality: 1, 3, 4 or 6
  int     geom1;                   // id of geom 1; deprecated, use geom[0]
  int     geom2;                   // id of geom 2; deprecated, use geom[1]
  int     geom[2];                 // geom ids; -1 for flex
  int     flex[2];                 // flex ids; -1 for geom
  int     elem[2];                 // element ids; -1 for geom or flex vertex
  int     vert[2];                 // vertex ids;  -1 for geom or flex element

  // flag set by mj_setContact or mj_instantiateContact
  int     exclude;                 // 0: include, 1: in gap, 2: fused, 3: no dofs

  // address computed by mj_instantiateContact
  int     efc_address;             // address in efc; -1: not included
};
typedef struct mjContact_ mjContact;
*/

macro_rules! impl_getters {
    ($Struct:ty { $( $name:ident: $T:ty = $description:literal; )* }) => {
        impl $Struct {
            $(
                #[allow(non_snake_case)]
                #[doc = $description]
                pub fn $name(&self) -> $T {
                    self.0.$name as $T
                }
            )*
        }
    };
}
impl_getters!(MjContact {
    // contact parameters set by near-phase collision function
    dist: f64 = "distance between nearest points; negative means penetration";
    pos: [f64; 3] = "position of contact point: midpoint between geoms";
    frame: [f64; 9] = "normal is in [0-2], points from geom[0] to geom[1]";

    // contact parameters set by `mj_collideGeoms`
    includemargin: f64 = "include if dist < includemargin = margin - gap";
    friction: [f64; 5] = "tangent1, tangent2, spin, roll1, roll2";
    solref: [f64; mjNREF as usize] = "constraint solver reference, normal direction";
    solreffriction: [f64; mjNREF as usize] = "constraint solver reference, friction directions";
    solimp: [f64; mjNIMP as usize] = "constraint solver impedance";

    // internal storage used by solver
    mu: f64 = "friction of regularized cone, set by `mj_makeConstraint`";
    H: [f64; 36] = "cone Hessian, set by `mj_constraintUpdate`";

    /* wrap specially later...

    // contact descriptors set by `mj_collideXXX`
    dim: i32 = "contact space dimensionality: 1, 3, 4 or 6";
    geom: [i32; 2] = "geom ids; -1 for flex";
    flex: [i32; 2] = "flex ids; -1 for geom";
    elem: [i32; 2] = "element ids; -1 for geom or flex vertex";
    vert: [i32; 2] = "vertex ids; -1 for geom or flex element";

    // flag set by `mj_setContact` or `mj_instantiateContact`
    exclude: i32 = "0: include, 1: in gap, 2: fused, 3: no dofs";

    // address computed by `mj_instantiateContact`
    efc_address: i32 = "address in efc; -1 means not included";

    */
});

impl MjContact {
    /// contact space dimensionality: 1, 3, 4 or 6
    pub fn dim(&self) -> usize {
        self.0.dim as usize
    }

    /// geom ids
    pub fn geom(&self) -> [Option<usize>; 2] {
        self.0.geom.map(|id| if id < 0 { None } else { Some(id as usize) })
    }
}

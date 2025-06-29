pub(crate) fn array_flatslice<T, const M: usize, const N: usize>(
    arr: &[[T; N]; M]
) -> &[T] {
    // SAFETY: `[[T; N]; M]` has the same layout as `[T; M * N]`
    unsafe {std::slice::from_raw_parts(arr.as_ptr() as *const T, M * N)}
}

pub struct Rgb { pub r: f32, pub g: f32, pub b: f32 }
pub struct Rgba { pub r: f32, pub g: f32, pub b: f32, pub a: f32 }

pub struct Dimention<const N: usize>;

pub trait FrictionDimention {}
impl FrictionDimention for Dimention<1> {}
impl FrictionDimention for Dimention<2> {}

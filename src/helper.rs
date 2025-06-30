pub(crate) fn array_flatslice<T, const M: usize, const N: usize>(
    arr: &[[T; N]; M]
) -> &[T] {
    // SAFETY: `[[T; N]; M]` has the same layout as `[T; M * N]`
    unsafe {std::slice::from_raw_parts(arr.as_ptr() as *const T, M * N)}
}

pub(crate) fn copy_str_to_c_chararray<const N: usize>(
    s: &str,
    c_array: &mut [i8; N],
) {
    let c_str = std::ffi::CString::new(s).expect("string must not contain internal null bytes");
    let bytes = c_str.into_bytes_with_nul();
    assert!(bytes.len() <= N, "string must be less than {N} bytes long");
    bytes.iter().enumerate().for_each(|(i, &b)| {c_array[i] = b as i8});
    c_array[bytes.len()..].fill(0); // fill the rest with zeros
}

pub struct Rgb { pub r: f32, pub g: f32, pub b: f32 }
pub struct Rgba { pub r: f32, pub g: f32, pub b: f32, pub a: f32 }

pub struct Dimention<const N: usize>;

pub trait FrictionDimention {}
impl FrictionDimention for Dimention<1> {}
impl FrictionDimention for Dimention<2> {}

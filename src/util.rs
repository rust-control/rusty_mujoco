pub(crate) fn array_flatslice<T, const M: usize, const N: usize>(arr: &[[T; N]; M]) -> &[T] {
    // SAFETY: `[[T; N]; M]` has the same layout as `[T; M * N]`
    unsafe { std::slice::from_raw_parts(arr.as_ptr() as *const T, M * N) }
}

pub(crate) fn copy_str_to_c_chararray<const N: usize>(
    s: &str,
    c_array: &mut [std::ffi::c_char; N],
) {
    let c_str = std::ffi::CString::new(s).expect("string must not contain internal null bytes");
    let bytes = c_str.into_bytes_with_nul();
    assert!(bytes.len() <= N, "string must be less than {N} bytes long");
    bytes
        .iter()
        .enumerate()
        .for_each(|(i, &b)| c_array[i] = b as std::ffi::c_char);
    c_array[bytes.len()..].fill(0); // fill the rest with zeros
}

pub struct Rgb {
    pub r: f32,
    pub g: f32,
    pub b: f32,
}
pub struct Rgba {
    pub r: f32,
    pub g: f32,
    pub b: f32,
    pub a: f32,
}

pub struct Dimention<const N: usize>;

pub trait FrictionDimention {}

impl FrictionDimention for Dimention<1> {}
impl FrictionDimention for Dimention<2> {}

pub(crate) trait AccessAs<T> {
    fn access_as(self) -> T;
}
pub (crate) trait SetBy<T, R = ()> {
    fn set_by(&mut self, value: T) -> R;
}

impl<'a> AccessAs<Result<&'a str, std::str::Utf8Error>> for &'a *const ::core::ffi::c_char {
    fn access_as(self) -> Result<&'a str, std::str::Utf8Error> {
        unsafe { core::ffi::CStr::from_ptr(*self).to_str() }
    }
}

impl<'a, const N: usize> AccessAs<Result<&'a str, std::str::Utf8Error>> for &'a [::core::ffi::c_char; N] {
    fn access_as(self) -> Result<&'a str, std::str::Utf8Error> {
        unsafe { core::ffi::CStr::from_ptr(self.as_ptr()).to_str() }
    }
}
impl<'a, const N: usize> SetBy<String, crate::Result<()>> for [::core::ffi::c_char; N] {
    fn set_by(&mut self, value: String) -> Result<(), crate::MjError> {
        if value.len() >= N {
            return Err(crate::MjError::from_error("can't convert Rust string to C: inline string capacity overflow"));
        }
        value.as_bytes().iter().enumerate().for_each(|(i, b)| {
            self[i] = *b as ::core::ffi::c_char;
        });
        self[value.as_bytes().len()] = b'\0' as ::core::ffi::c_char;
        Ok(())
    }
}

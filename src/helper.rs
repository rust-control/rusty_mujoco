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

pub trait UiItemTypeUnion { type Union; }
pub struct UiItemType<const T: i32>;
impl UiItemTypeUnion for UiItemType<{crate::mjtItem::END.0}> { type Union = ();}
impl UiItemTypeUnion for UiItemType<{crate::mjtItem::SECTION.0}> { type Union = (); }
impl UiItemTypeUnion for UiItemType<{crate::mjtItem::SEPARATOR.0}> { type Union = (); }
impl UiItemTypeUnion for UiItemType<{crate::mjtItem::STATIC.0}> { type Union = (); }
impl UiItemTypeUnion for UiItemType<{crate::mjtItem::BUTTON.0}> { type Union = crate::mjuiItemSingle; }
impl UiItemTypeUnion for UiItemType<{crate::mjtItem::CHECKINT.0}> { type Union = crate::mjuiItemSingle; }
impl UiItemTypeUnion for UiItemType<{crate::mjtItem::CHECKBYTE.0}> { type Union = crate::mjuiItemSingle; }
impl UiItemTypeUnion for UiItemType<{crate::mjtItem::RADIO.0}> { type Union = crate::mjuiItemMulti; }
impl UiItemTypeUnion for UiItemType<{crate::mjtItem::RADIOLINE.0}> { type Union = crate::mjuiItemMulti; }
impl UiItemTypeUnion for UiItemType<{crate::mjtItem::SELECT.0}> { type Union = crate::mjuiItemMulti; }
impl UiItemTypeUnion for UiItemType<{crate::mjtItem::SLIDERINT.0}> { type Union = crate::mjuiItemSlider; }
impl UiItemTypeUnion for UiItemType<{crate::mjtItem::SLIDERNUM.0}> { type Union = crate::mjuiItemSlider; }
impl UiItemTypeUnion for UiItemType<{crate::mjtItem::EDITINT.0}> { type Union = crate::mjuiItemEdit; }
impl UiItemTypeUnion for UiItemType<{crate::mjtItem::EDITNUM.0}> { type Union = crate::mjuiItemEdit; }
impl UiItemTypeUnion for UiItemType<{crate::mjtItem::EDITFLOAT.0}> { type Union = crate::mjuiItemEdit; }
impl UiItemTypeUnion for UiItemType<{crate::mjtItem::EDITTXT.0}> { type Union = crate::mjuiItemEdit; }

fn _test<const T: i32, U>(u: U)
where
    const...
{}

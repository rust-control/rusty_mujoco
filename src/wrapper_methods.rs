macro_rules! basic_accessors {
    ($T:ty {
        $(
            $get:ident / $get_mut:ident: $F:ty,
        )*
    }) => {
        impl $T {
            $(
                pub fn $get(&self) -> &$F {
                    crate::util::AccessAs::access_as::<&F>(&self.$get)
                }
                pub fn $get_mut(&mut self) -> &mut $F {
                    crate::util::AccessAs::access_as::<&mut F>(&mut self.$get)
                }
            )*
        }
    };
}

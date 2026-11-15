macro_rules! basic_accessors {
    ($T:ty {
        $(
            $fieldname:ident: $F:ty $(| $set:ident($S:ty) -> $R:ty)?,
        )*
    }) => {
        impl $T {
            $(
                pub fn $fieldname(&self) -> $F {
                    crate::util::AccessAs::<$F>::access_as(&self.$fieldname)
                }
                $(
                    pub fn $set(&mut self, $fieldname: $S) -> $R {
                        crate::util::SetBy::<$S, $R>::set_by(&mut self.$fieldname, $fieldname)
                    }
                )?
            )*
        }
    };
}
macro_rules! slice_accessors {
    ($T:ty {
        $(
            $fieldname:ident $(| $fieldname_mut:ident)?: [$I:ty; $n:expr],
        )*
    }) => {
        impl $T {
            $(
                pub fn $fieldname(&self) -> &[$I] {
                    unsafe { std::slice::from_raw_parts(self.$fieldname, $n) }
                }
                $(
                    pub fn $fieldname_mut(&mut self) -> &mut [$I] {
                        unsafe { std::slice::from_raw_parts_mut(self.$fieldname, $n) }
                    }
                )?
            )*
        }
    };
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

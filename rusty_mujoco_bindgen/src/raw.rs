#[derive(Debug)]
struct TrimUnderscoreCallbacks;
impl bindgen::callbacks::ParseCallbacks for TrimUnderscoreCallbacks {
    fn item_name(&self, item_info: bindgen::callbacks::ItemInfo) -> Option<String> {
        /*
            This finally oversets non-suffixed name (like `mjData`)
            to/over its original name (like `mjData_`).
        */
        item_info.name.strip_suffix('_').map(str::to_owned)
    }
}

#[derive(Debug)]
struct MakeMjnVariantGlobalConstCallbacks;
impl bindgen::callbacks::ParseCallbacks for MakeMjnVariantGlobalConstCallbacks {
    fn enum_variant_behavior(
        &self,
        _enum_name: Option<&str>,
        original_variant_name: &str,
        _variant_value: bindgen::callbacks::EnumVariantValue,
    ) -> Option<bindgen::callbacks::EnumVariantCustomBehavior> {
        /*
            This generates const like:
            ```
            pub const mjNTEXROLE: mjtTextureRole = mjtTextureRole::mjNTEXROLE;
            ```
            at module top for `mjN*` variants.
        */
        original_variant_name
            .starts_with("mjN")
            .then_some(bindgen::callbacks::EnumVariantCustomBehavior::Constify)
    }
}

pub struct RawBindgen {
    /// `include` directory as it is in the official release tarball.
    /// It'll have a single child directory named `mujoco`.
    pub include_dir: std::path::PathBuf,
}

impl RawBindgen {
    pub fn generate(self) -> std::io::Result<String> {
        let mut buf = String::new();
        ::bindgen::builder()
            /* common */
            .header_contents("bindgen.h", "#include \"mujoco.h\"")
            .clang_args([
                format!("-I{}", self.include_dir.display()),
                format!("-I{}", self.include_dir.join("mujoco").display()),
                format!("-fparse-all-comments"),
            ])
            .use_core()
            .size_t_is_usize(true)
            .layout_tests(false)
            /* for types */
            .default_visibility(bindgen::FieldVisibilityKind::Public)
            .prepend_enum_name(false)
            .newtype_enum("_?mj.+[^B]+_?")
            .bitfield_enum("_?mj.+Bit_?")
            .allowlist_type("_?mj.*")
            .no_copy("mj(Model|Data|Spec|vScene|rContext)_") // manually impl Drop with each specific freeing function
            .parse_callbacks(Box::new(TrimUnderscoreCallbacks))
            .parse_callbacks(Box::new(MakeMjnVariantGlobalConstCallbacks))
            /* for functions */
            .array_pointers_in_arguments(true)
            .merge_extern_blocks(true)
            .allowlist_function("_?mj.*")
            .allowlist_var("_?mj.*")
            /* generate to intermediate buffer on memory */
            .parse_callbacks(Box::new(bindgen::CargoCallbacks::new()))
            .generate()
            .expect("failed to generate bindings")
            .write(unsafe { buf.as_mut_vec() }) // SAFETY: valid Rust code is UTF-8
            .expect("failed to write bindings to buffer");
        Ok(buf)
    }
}

use std::{env, io::BufRead, path::Path};

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
struct MakeMjnConstantsCallbacks;
impl bindgen::callbacks::ParseCallbacks for MakeMjnConstantsCallbacks {
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
            at module top for `mjtN*` variants.
        */
        original_variant_name.starts_with("mjN").then_some(bindgen::callbacks::EnumVariantCustomBehavior::Constify)
    }
}

fn main() {
    if option_env!("DOCS_RS").is_some() {
        return;
    }

    let mujoco_home = env::var("MUJOCO_HOME").expect("MUJOCO_HOME not set");
    let mujoco_lib = Path::new(&mujoco_home).join("lib").to_str().unwrap().to_owned();
    let mujoco_include = Path::new(&mujoco_home).join("include").to_str().unwrap().to_owned();
    let mujoco_include_mujoco = Path::new(&mujoco_include).join("mujoco").to_str().unwrap().to_owned();

    let bindgen_home = Path::new(&env!("CARGO_MANIFEST_DIR")).join("src");
    let bindgen_h = bindgen_home.join("bindgen.h").to_str().unwrap().to_owned();
    let bindgen_rs = bindgen_home.join("bindgen.rs").to_str().unwrap().to_owned();

    println!("cargo:rerun-if-changed={bindgen_h}");
    println!("cargo:rustc-link-search={mujoco_lib}");
    println!("cargo:rustc-link-lib=dylib=libmujoco");

    let mut bindings = Vec::new();
    bindgen::builder()
        .header(bindgen_h)
        .clang_args([format!("-I{mujoco_include}"), format!("-I{mujoco_include_mujoco}")])
        .use_core()
        .raw_line("#![allow(unused, non_camel_case_types, non_snake_case, non_upper_case_globals)]")
        .respect_cxx_access_specs(false)
        .default_visibility(bindgen::FieldVisibilityKind::PublicCrate)
        .newtype_enum("_?mjt.+[^Bit]")
        .bitfield_enum("_?mjt.+Bit_")
        .allowlist_type("_?mj.*")
        .allowlist_function("_?mj.*")
        .allowlist_var("_?mj.*")
        //.derive_default(true)
        .no_copy("mj(Model|Data|Spec|vScene|rContext)_")// impl Drop for them (using specific free-er functions)
        .size_t_is_usize(true)
        .array_pointers_in_arguments(true)
        .merge_extern_blocks(true)
        .prepend_enum_name(false)
        .layout_tests(false)
        .parse_callbacks(Box::new(TrimUnderscoreCallbacks))
        .parse_callbacks(Box::new(MakeMjnConstantsCallbacks))
        .parse_callbacks(Box::new(bindgen::CargoCallbacks::new()))
        .generate().expect("Failed to generate bindings")
        .write(Box::new(&mut bindings)).expect("Failed to write bindings to file");

    /*
        There seems to be no way to:
        
        - set `pub(crate)` visibility for newtype's inner field
        - make `mjtN.*` constants `usize` instead of original type
        - make `mjMAX.*` constants `usize` except for ones originally defined as
          `f(64|32)` which are limit values for something
        - hide only `mjtN.*` consts of newtype impl from user, and
          remove `mj{NAME}_` prefix of consts of newtype impl

        using bindgen, so we do them manually...
    */
    let bindings = bindings
        .lines()
        .map(Result::unwrap)
        .fold(Vec::with_capacity(bindings.len()), |mut new, line| {
            if line.starts_with("pub struct mjt") {
                /* sed -i -r 's/^pub struct mjt(.*)\(pub (.*)\);$/pub struct mjt\1\(pub\(crate\) \2\);/' */
                new.push(line.replace("(pub", "(pub(crate)"));
            } else if line.starts_with("pub const mjN") {
                /*
                    pub const mjNSomething: Something = Something::Value; // from enum newtype impl
                    pub const mjNSOMETHING: u32 = 42;
                */
                let mut line = line.split(' ');
                let _ = line.next(); // "pub"
                let _ = line.next(); // "const"
                let name = line.next().unwrap().strip_suffix(':').unwrap();
                let _ty = line.next().unwrap();
                let _ = line.next(); // "="
                let value = line.next().unwrap().strip_suffix(";").unwrap();
                new.push(if value.contains("::") {
                    format!("pub const {name}: usize = {value}.0 as usize;")
                } else {
                    format!("pub const {name}: usize = {value};")
                });
            } else if line.starts_with("pub const mjMAX") {
                /*
                    pub const mjMAXSOMETHING: f(64|32) = 1.0;
                    pub const mjMAXNSOMETHING: u.* = 42;
                */
                let mut line = line.split(' ');
                let _ = line.next(); // "pub"
                let _ = line.next(); // "const"
                let name = line.next().unwrap().strip_suffix(':').unwrap();
                let ty = line.next().unwrap();
                let _ = line.next(); // "="
                let value = line.next().unwrap().strip_suffix(";").unwrap();
                new.push(if ty.starts_with('f') {
                    format!("pub const {name}: {ty} = {value};")
                } else {
                    format!("pub const {name}: usize = {value};")
                });
            } else if line.starts_with("    pub const mjN") {
                new.push(line.replace("pub ", ""));
            } else if line.starts_with("    pub const mj") {
                /*
                    impl mjtObj {
                        pub const mjOBJ_UNKNOWN: mjtObj = mjtObj(0);
                        pub const mjOBJ_BODY: mjtObj = mjtObj(1);
                        pub const mjOBJ_XBODY: mjtObj = mjtObj(2);
                        ...
                        pub const mjNOBJECT: mjtObj = mjtObj(26);
                    }
                */
                let after_mj_prefix = line
                    .split_once('_')
                    .map_or(&*line, |(_, rest)| rest);
                let prefix_for_number = if after_mj_prefix.chars().next().unwrap().is_ascii_digit() {
                    // mjtTexture::2D / mjtFontScale::50~300
                    if after_mj_prefix.starts_with("2D") {"_"} else {"X"}
                } else {""};
                new.push(format!("    pub const {prefix_for_number}{after_mj_prefix}"));
            } else {
                new.push(line);
            }
            new
        });

    std::fs::write(&bindgen_rs, bindings.join("\n")).expect("Failed to write bindings to file");
}

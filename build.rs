fn main() {
    if option_env!("DOCS_RS").is_some() { return }

    if dbg!(!probe_mujoco_lib_with_libdir(None)) {
        if let Some(mujoco_lib_dir) = mujoco_lib_directory_from_env("MUJOCO_LIB") {
            assert!(
                dbg!(probe_mujoco_lib_with_libdir(Some(&mujoco_lib_dir))),
                "Failed to link with mujoco library even after setting `MUJOCO_LIB` environment variable!"
            );
            println!("cargo:rustc-link-search=native={}", mujoco_lib_dir.display());
        } else {
            panic!("\
                MuJoCo library not found. Make sure that the mujoco library is installed and, \
                if its location is non-standard, set the path via the `MUJOCO_LIB` environment variable.\
            ");
        }
    }
    println!("cargo:rustc-link-lib=dylib=mujoco");
    
    #[cfg(feature = "bindgen")]
    bindgen();
}

fn probe_mujoco_lib_with_libdir(libdir: Option<&std::path::Path>) -> bool {
    let crate_root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
    let vendor_dir = crate_root.join("vendor");
    
    let probe_c = crate_root.join("probe.c");
    let vendor_include = vendor_dir.join("include");
    let vendor_include_mujoco = vendor_include.join("mujoco");
    
    /*
     * `cc` crate is designed as:
     * 
     * > A library for Cargo build scripts to compile a set of C/C++/assembly/CUDA files
     *   into a static archive for Cargo to link into the crate being built.
     * 
     * However, here we need to probe whether linking with `mujoco` library to an executable
     * succeeds or not, so we manually invoke the compiler with appropriate arguments
     * using `cc` crate to get the compiler path and kind.
     */

    let cc = cc::Build::new().cargo_metadata(false).get_compiler();
    let cc_path = cc.path();
    let cc_args = if cc.is_like_gnu() || cc.is_like_clang() {
        let mut args = vec![
            probe_c.to_str().unwrap(),
            "-I", vendor_include.to_str().unwrap(),
            "-I", vendor_include_mujoco.to_str().unwrap(),
            "-o", "/dev/null",
        ];
        if let Some(libdir) = libdir {
            args.extend(["-L", libdir.to_str().unwrap()]);
        }
        args.extend(["-l", "mujoco"]);
        args
    } else if cc.is_like_msvc() || cc.is_like_clang_cl() {
        let mut args = vec![
            probe_c.to_str().unwrap(),
            "/I", vendor_include.to_str().unwrap(),
            "/I", vendor_include_mujoco.to_str().unwrap(),
            "/Fe:NUL",
        ];
        if let Some(libdir) = libdir {
            let libpath_arg = format!("/LIBPATH:{}", libdir.display());
            args.extend([libpath_arg.leak() as &_]); // will be released soon when build script ends
        }
        args.extend(["mujoco.lib"]);
        args
    } else {
        panic!("Unsupported compiler: {}", cc_path.display());
    };
    
    let stdio_strategy = || if cfg!(feature = "DEBUG") {
        std::process::Stdio::inherit()
    } else {
        std::process::Stdio::null()
    };
    std::process::Command::new(dbg!(cc_path))
        .args(dbg!(cc_args))
        .stdout(stdio_strategy())
        .stderr(stdio_strategy())
        .status()
        .unwrap_or_else(|err| panic!("Failed to invoke compiler to probe mujoco library: {err}"))
        .success()
}

/// Adds the given path to the library search path for linking,
/// with resolving the directory path from an environment variable `env`
/// that may be either the directory path or path of the library file itself.
fn mujoco_lib_directory_from_env(env: &'static str) -> Option<std::path::PathBuf> {
    let mujoco_lib = match std::env::var(env) {
        Ok(value) => value,
        Err(std::env::VarError::NotPresent) => return None,
        Err(std::env::VarError::NotUnicode(os_str)) => panic!("{env} contains invalid unicode: `{}`", os_str.to_string_lossy()),
    };
    let mujoco_lib = std::path::Path::new(&mujoco_lib)
        .canonicalize()
        .unwrap_or_else(|err| panic!("{env} is not a valid path: {err}"));
    
    Some(if mujoco_lib.is_dir() {
        mujoco_lib
    } else {
        mujoco_lib
            .parent()
            .unwrap_or_else(|| panic!("{env} must be a valid path to mujoco library file or directory containing it"))
            .to_owned()
    })
}

#[cfg(feature = "bindgen")]
fn bindgen() {
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
                at module top for `mjN*` variants.
            */
            original_variant_name.starts_with("mjN").then_some(bindgen::callbacks::EnumVariantCustomBehavior::Constify)
        }
    }

    /*
     * The hand-processing step after `bindgen` generation requires
     * `cargo fmt` (and then it's automatically applied to the
     * bindgen's raw output, and the hand-processing correctly works).
     * This is a **requirement** for the build script to continue.
     */
    assert!(
        std::process::Command::new("cargo")
            .args(["help", "fmt"])
            .stdout(std::process::Stdio::null())
            .status()
            .is_ok_and(|s| s.success()),
        "`cargo fmt` is not available; This build script can't continue without it."
    );

    let vendor_dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("vendor");
    let vendor_include = vendor_dir.join("include").to_str().unwrap().to_owned();
    let vendor_include_mujoco = vendor_dir.join("include").join("mujoco").to_str().unwrap().to_owned();

    let src_dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
    let bindgen_rs = src_dir.join("bindgen.rs").to_str().unwrap().to_owned();

    let mut bindings = Vec::new();
    bindgen::builder()
        .header_contents("bindgen.h", "#include \"mujoco.h\"")
        .clang_args([format!("-I{vendor_include}"), format!("-I{vendor_include_mujoco}")])
        .use_core()
        .raw_line("#![allow(unused, non_camel_case_types, non_snake_case, non_upper_case_globals)]")
        .respect_cxx_access_specs(false)
        .default_visibility(bindgen::FieldVisibilityKind::PublicCrate)
        .newtype_enum("_?mjt.+[^Bit]")
        .bitfield_enum("_?mjt.+Bit_")
        .allowlist_type("_?mj.*")
        .allowlist_function("_?mj.*")
        .allowlist_var("_?mj.*")
        .no_copy("mj(Model|Data|Spec|vScene|rContext)_")/* impl Drop with using specific freeing functions */
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
    let bindings = std::io::BufRead::lines(&*bindings)
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
                let after_mj_prefix = line.split_once('_').map_or(&*line, |(_, rest)| rest);
                new.push(if after_mj_prefix.starts_with("2D") {
                    format!("    pub const {}", after_mj_prefix.replace("2D", "D2"))
                } else if after_mj_prefix.chars().next().unwrap().is_ascii_digit() {
                    format!("    pub const X{after_mj_prefix}")
                } else {
                    format!("    pub const {after_mj_prefix}")
                });
            } else {
                new.push(line);
            }
            new
        });

    std::fs::write(&bindgen_rs, bindings.join("\n")).expect("Failed to write bindings to file");
}

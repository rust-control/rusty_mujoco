fn main() {
    if option_env!("DOCS_RS").is_some() {
        return;
    }

    if dbg!(!probe_mujoco_lib_with_libdir(None)) {
        if let Some(mujoco_lib_dir) = mujoco_lib_directory_from_env("MUJOCO_LIB") {
            assert!(
                dbg!(probe_mujoco_lib_with_libdir(Some(&mujoco_lib_dir))),
                "Failed to link with mujoco library even after setting `MUJOCO_LIB` environment variable!"
            );
            println!(
                "cargo:rustc-link-search=native={}",
                mujoco_lib_dir.display()
            );
        } else {
            panic!(
                "\
                MuJoCo library not found. Make sure that the mujoco library is installed and, \
                if its location is non-standard, set the path via the `MUJOCO_LIB` environment variable.\
            "
            );
        }
    }
    println!("cargo:rustc-link-lib=mujoco");

    #[cfg(feature = "bindgen")]
    ::rusty_mujoco_bindgen::bindgen();
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
     * succeeds or not, so we manually invoke the compiler with appropriate arguments,
     * using `cc` crate to get the compiler path and kind.
     */

    let cc = ::cc::Build::new().cargo_metadata(false).get_compiler();
    let cc_path = cc.path();
    let cc_args = if cc.is_like_gnu() || cc.is_like_clang() {
        let mut args = vec![
            probe_c.to_str().unwrap().to_string(),
            format!("-I{}", vendor_include.display()),
            format!("-I{}", vendor_include_mujoco.display()),
            "-o".to_string(),
            "/dev/null".to_string(),
        ];
        if let Some(libdir) = libdir {
            args.push(format!("-L{}", libdir.display()));
        }
        args.push("-lmujoco".to_string());
        args
    } else if cc.is_like_msvc() || cc.is_like_clang_cl() {
        let mut args = vec![
            probe_c.to_str().unwrap().to_string(),
            format!("/I{}", vendor_include.display()),
            format!("/I{}", vendor_include_mujoco.display()),
            "/Fe:NUL".to_string(),
        ];
        if let Some(libdir) = libdir {
            args.push(format!("/LIBPATH:{}", libdir.display()));
        }
        args.push("mujoco.lib".to_string());
        args
    } else {
        panic!("Unsupported compiler: {}", cc_path.display());
    };

    std::process::Command::new(dbg!(cc_path))
        .args(dbg!(cc_args))
        .stdout(std::process::Stdio::inherit())
        .stderr(std::process::Stdio::inherit())
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
        Err(std::env::VarError::NotUnicode(os_str)) => panic!(
            "{env} contains invalid unicode: `{}`",
            os_str.to_string_lossy()
        ),
    };
    let mujoco_lib = std::path::Path::new(&mujoco_lib);

    Some(if mujoco_lib.is_dir() {
        mujoco_lib.to_owned()
    } else if mujoco_lib.is_file() {
        mujoco_lib
            .parent()
            .unwrap_or_else(|| {
                panic!(
                    "{env} must be a valid path to mujoco library file or directory containing it"
                )
            })
            .to_owned()
    } else {
        panic!("{env} must be a valid path to mujoco library file or directory containing it")
    })
}

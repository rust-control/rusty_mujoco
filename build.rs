use std::{env, path::Path};

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

    bindgen::builder()
        .header(bindgen_h)
        .clang_args([format!("-I{mujoco_include}"), format!("-I{mujoco_include_mujoco}")])
        .rustified_enum("_?mjt.+")
        .bitfield_enum("_?mjt.+Bit")
        .allowlist_type("_?mj.*")
        .allowlist_function("_?mj.*")
        .allowlist_var("_?mj.*")
        .default_enum_style(bindgen::EnumVariation::NewType { is_bitfield: false, is_global: false })
        .derive_default(true)
        .size_t_is_usize(true)
        .raw_line("#![allow(unused, non_camel_case_types, non_snake_case, non_upper_case_globals)]")
        .parse_callbacks(Box::new(bindgen::CargoCallbacks::new()))
        .generate().expect("Failed to generate bindings")
        .write_to_file(bindgen_rs).expect("Failed to write bindings to file");
}

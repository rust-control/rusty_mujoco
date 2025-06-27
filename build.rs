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
        .derive_default(true)
        .size_t_is_usize(true)
        .array_pointers_in_arguments(true)
        .merge_extern_blocks(true)
        .parse_callbacks(Box::new(TrimUnderscoreCallbacks))
        .parse_callbacks(Box::new(bindgen::CargoCallbacks::new()))
        .generate().expect("Failed to generate bindings")
        .write(Box::new(&mut bindings)).expect("Failed to write bindings to file");

    /*
        There seems to be no way to set `pub(crate)` visibility for newtype inner fields
        using bindgen, so we do it manually...

        ```sh
        sed -i -r 's/^pub struct mjt(.*)\(pub (.*)\);$/pub struct mjt\1\(pub\(crate\) \2\);/'
        ```
    */
    let bindings = bindings
        .lines()
        .map(Result::unwrap)
        .fold(Vec::with_capacity(bindings.len()), |mut new, line| {
            if line.starts_with("pub struct mjt") {
                let new_line = line.replace("(pub ", "(pub(crate) ");
                new.push(new_line);
            } else {
                new.push(line.to_owned());
            }
            new
        });

    std::fs::write(&bindgen_rs, bindings.join("\n")).expect("Failed to write bindings to file");
}

mod bindings;
mod ast;

pub fn bindgen() {
    /*
     * The hand-processing step after `bindgen` generation requires
     * `cargo fmt` (and then it's automatically applied to the
     * bindgen's raw output, and the hand-processing correctly works).
     * This is a **requirement** for the build script to continue.
     */
    assert!(
        std::process::Command::new("rustfmt")
            .arg("TODO.rs")
            .stdout(std::process::Stdio::null())
            .status()
            .is_ok_and(|s| s.success()),
        "`rustfmt` is not available; This build script can't continue without it."
    );


    let mut binding_items = syn::parse_file(&bindings_buf).unwrap().items;

    // todo!()

    let bindings_buf = {
    };

    std::fs::write(
        std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("src")
            .join("bindgen.rs"),
        bindings_buf,
    )
    .expect("failed to write types bindings to file");

    std::process::Command::new("cargo")
        .arg("fmt")
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::null())
        .status()
        .ok();
}

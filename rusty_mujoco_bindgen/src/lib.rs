mod binding;
mod layers;

pub fn bindgen(
    include_dir: impl AsRef<std::path::Path>,
    out_mod_dir: impl AsRef<std::path::Path>,
) -> std::io::Result<()> {
    let (include_dir, out_mod_dir) = (include_dir.as_ref(), out_mod_dir.as_ref());
    if !std::fs::exists(include_dir)? {
        return Err(std::io::Error::new(
            std::io::ErrorKind::NotFound,
            format!("include directory `{}` not found", include_dir.display())
        ));
    }
    std::fs::remove_dir_all(out_mod_dir).ok();
    std::fs::create_dir_all(out_mod_dir)?;

    let b = binding::Binding::new(include_dir)?;
    let traits = layers::traits_layer(&b, "super::raw");
    let accessors = layers::accessors_layer(&b, "super::raw");
    let functions = layers::functions_layer(&b, "super::raw");

    std::fs::write(out_mod_dir.join("mod.rs"), "\
        mod raw;\n\
        mod traits;\n\
        mod accessors;\n\
        mod functions;\n\
        \n\
        pub use raw::*;\n\
        pub use functions::*;\n\
    ")?;
    std::fs::write(out_mod_dir.join("raw.rs"), b.to_string())?;
    std::fs::write(out_mod_dir.join("traits.rs"), traits.to_string())?;
    std::fs::write(out_mod_dir.join("accessors.rs"), accessors.to_string())?;
    std::fs::write(out_mod_dir.join("functions.rs"), functions.to_string())?;

    if std::process::Command::new("rustfmt")
        .args(["mod.rs", "traits.rs", "accessors.rs", "raw.rs", "functions.rs"].map(|f| out_mod_dir.join(f)))
        .stdin(std::process::Stdio::null())
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::null())
        .status()
        .is_err()
    {
        eprintln!("[WARNING] `rustfmt` failed; skipping formatting...");
    }

    Ok(())
}

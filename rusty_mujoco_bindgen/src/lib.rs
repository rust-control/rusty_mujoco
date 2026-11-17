mod bindings;

use self::bindings::Bindings;

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

    let mut b = Bindings::new(include_dir);

    for f in ["mod.rs", "raw.rs", "functions.rs", "types.rs"] {
        std::fs::write(out_mod_dir.join(f), "TODO")?;
    }
    if std::process::Command::new("rustfmt")
        .args([
            "mod.rs", "raw.rs", "functions.rs", "types.rs"
        ].map(|f| out_mod_dir.join(f)))
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::null())
        .status()
        .is_err()
    {
        eprintln!("[WARNING] `rustfmt` is not found; skipping format...");
    }

    Ok(())
}

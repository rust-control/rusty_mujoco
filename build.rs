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
     * succeeds or not, so we manually invoke the compiler with appropriate arguments,
     * using `cc` crate to get the compiler path and kind.
     */

    let cc = cc::Build::new().cargo_metadata(false).get_compiler();
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
    let vendor_include_mujoco = vendor_dir
        .join("include")
        .join("mujoco")
        .to_str()
        .unwrap()
        .to_owned();

    let mut bindings_buf = String::new();
    bindgen::builder()
        /* common */
        .header_contents("bindgen.h", "#include \"mujoco.h\"")
        .clang_args([
            format!("-I{vendor_include}"),
            format!("-I{vendor_include_mujoco}"),
        ])
        .use_core()
        .size_t_is_usize(true)
        .layout_tests(false)
        /* for types */
        .default_visibility(bindgen::FieldVisibilityKind::Public)
        .respect_cxx_access_specs(false)
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
        .write(unsafe { bindings_buf.as_mut_vec() }) // SAFETY: valid Rust code is UTF-8
        .expect("failed to write bindings to buffer");

    let bindings_buf = {
        use quote::ToTokens;
        let mut r#final = quote::quote! {
            #![allow(unused, non_camel_case_types, non_snake_case, non_upper_case_globals, clippy::all)]
        };
        for item in &mut syn::parse_file(&bindings_buf).unwrap().items {
            match item {
                syn::Item::Struct(syn::ItemStruct {
                    fields: syn::Fields::Unnamed(newtype_enum_inner),
                    ..
                }) => {
                    assert_eq!(newtype_enum_inner.unnamed.len(), 1);
                    assert!(matches!(
                        newtype_enum_inner.unnamed[0].vis,
                        syn::Visibility::Public(_)
                    ));
                    newtype_enum_inner.unnamed[0].vis = syn::Visibility::Inherited;
                }
                syn::Item::Struct(syn::ItemStruct {
                    vis,
                    ident,
                    fields: syn::Fields::Named(struct_fields),
                    ..
                }) => {
                    if ident.to_string().contains("__bindgen_ty") {
                        *vis = syn::parse_quote!(pub(crate));
                        struct_fields
                            .named
                            .iter_mut()
                            .for_each(|f| f.vis = syn::parse_quote!(pub(crate)));
                    } else {
                        for f in &mut struct_fields.named {
                            let (ident_str, ty_str) = (
                                f.ident.as_ref().unwrap().to_string(),
                                f.ty.to_token_stream().to_string(),
                            );
                            if ident_str.ends_with("id")
                                || matches!(f.ty, syn::Type::Ptr(_))
                                || ty_str.contains("unsafe extern \"C\"")
                                || ty_str.contains("__bindgen_ty")
                                || ty_str.contains("c_char")
                            {
                                f.vis = syn::parse_quote!(pub(crate));
                            }
                        }
                    }
                }
                syn::Item::Type(syn::ItemType { vis, ty, .. }) => {
                    if ty
                        .to_token_stream()
                        .to_string()
                        .contains("unsafe extern \"C\"")
                    {
                        *vis = syn::parse_quote!(pub(crate));
                    }
                }
                syn::Item::Union(u) => {
                    u.vis = syn::parse_quote!(pub(crate));
                }
                syn::Item::Const(x) if x.ident.to_string().starts_with("mjMAX") => {
                    if let syn::Expr::Lit(syn::ExprLit {
                        lit: syn::Lit::Int(intlit),
                        ..
                    }) = &*x.expr
                    {
                        let syn::ItemConst { ident, .. } = &*x;
                        *x = syn::parse_quote! {
                            pub const #ident: usize = #intlit;
                        };
                    }
                }
                syn::Item::Const(x) if x.ident.to_string().starts_with("mjN") => {
                    let syn::ItemConst { ident, .. } = x;
                    if let syn::Expr::Path(syn::ExprPath { path, .. }) = &*x.expr {
                        // generated by MakeMjnVariantGlobalConstCallbacks
                        *x = syn::parse_quote! {
                            pub const #ident: usize = #path.0 as usize;
                        };
                    } else if let syn::Expr::Lit(syn::ExprLit {
                        lit: syn::Lit::Int(intlit),
                        ..
                    }) = &*x.expr
                    {
                        *x = syn::parse_quote! {
                            pub const #ident: usize = #intlit;
                        };
                    }
                }
                syn::Item::Impl(syn::ItemImpl { items, .. }) => {
                    for impl_item in items.iter_mut() {
                        if let syn::ImplItem::Const(syn::ImplItemConst { vis, ident, .. }) =
                            impl_item
                        {
                            if ident.to_string().starts_with("mjN") {
                                // the targets of MakeMjnVariantGlobalConstCallbacks
                                *vis = syn::Visibility::Inherited;
                            } else if ident.to_string().starts_with("mj") {
                                let after_mj_prefix =
                                    ident.to_string().split_once('_').unwrap().1.to_owned();
                                *ident = if after_mj_prefix.starts_with("2D") {
                                    quote::format_ident!("{}", &after_mj_prefix.replace("2D", "D2"))
                                } else if after_mj_prefix.chars().next().unwrap().is_ascii_digit() {
                                    quote::format_ident!("X{after_mj_prefix}")
                                } else {
                                    quote::format_ident!("{after_mj_prefix}")
                                }
                            }
                        }
                    }
                    if items.iter().all(|i| matches!(i, syn::ImplItem::Const(_))) {
                        let ident_muts = items
                            .iter_mut()
                            .map(|i| match i {
                                syn::ImplItem::Const(syn::ImplItemConst { ident, .. }) => ident,
                                _ => unreachable!(),
                            })
                            .collect::<Vec<_>>();
                        if let Some(prefix) = ident_muts[0]
                            .to_string()
                            .rsplit_once('_')
                            .map(|(prefix, _)| prefix.to_owned())
                            && ident_muts
                                .iter()
                                .all(|i| i.to_string().starts_with(&prefix))
                        {
                            for i in ident_muts {
                                *i = quote::format_ident!(
                                    "{}",
                                    i.to_string()
                                        .strip_prefix(&prefix)
                                        .unwrap()
                                        .trim_start_matches('_')
                                );
                            }
                        }
                    }
                }
                syn::Item::ForeignMod(syn::ItemForeignMod { items, .. }) => {
                    for foreign_item in items {
                        match foreign_item {
                            syn::ForeignItem::Fn(syn::ForeignItemFn { vis, .. }) => {
                                *vis = syn::parse_quote!(pub(crate))
                            }
                            syn::ForeignItem::Static(syn::ForeignItemStatic { vis, .. }) => {
                                *vis = syn::parse_quote!(pub(crate))
                            }
                            _ => unreachable!("unexpected foreign item"),
                        }
                    }
                }
                _ => (),
            }
            item.to_tokens(&mut r#final);
        }
        r#final.to_string()
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

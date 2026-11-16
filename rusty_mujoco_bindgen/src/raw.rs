pub struct Bindings {
    pub globalattrs: Vec<::syn::Attribute>,
    pub constants: Vec<::syn::ItemConst>,
    pub typealiases: Vec<::syn::ItemType>,
    pub unions: Vec<::syn::ItemUnion>,
    pub enums: Vec<BindingEnum>,
    pub structs: Vec<BindingStruct>,
    pub functions: Vec<::syn::ForeignItemFn>,
    pub statics: Vec<::syn::ForeignItemStatic>,
}

pub struct BindingEnum(::syn::ItemStruct);

pub struct BindingStruct(::syn::ItemStruct);

pub struct BindingStructField<'s> {
    pub comment: Option<BindingStructFieldComment>,
    pub vis: &'s ::syn::Visibility,
    pub ident: &'s ::syn::Ident,
    pub ty: &'s ::syn::Type,
}

pub enum BindingStructFieldComment {
    Slice { raw: String, size_expr: ::syn::Expr },
    Varbatim(String),
}

impl BindingStruct {
    pub fn ident(&self) -> &::syn::Ident {
        &self.0.ident
    }

    pub fn fields(&self) -> impl Iterator<Item = BindingStructField<'_>> {
        self.0.fields.iter().map(|f| BindingStructField {
            comment: BindingStructFieldComment::extract(f.attrs.iter()),
            vis: &f.vis,
            ident: f.ident.as_ref().unwrap(),
            ty: &f.ty,
        })
    }
}

impl BindingStructFieldComment {
    fn extract<'a>(mut attrs: impl Iterator<Item = &'a ::syn::Attribute>) -> Option<Self> {
        attrs
            .find_map(|a| a
                .meta
                .require_name_value()
                .ok()
                .and_then(|nv| nv
                    .path
                    .get_ident()
                    .is_some_and(|i| i == "doc")
                    .then_some(&nv.value)
                )
            )
            .and_then(|doc_value_expr| match doc_value_expr {
                ::syn::Expr::Lit(::syn::ExprLit {
                    lit: ::syn::Lit::Str(s),
                    ..
                }) => Some(Self::parse_literal(s.value())),
                _ => None
            })
    }
    fn parse_literal(s: String) -> Self {
        let Some((_, p)) = s.strip_suffix(')').and_then(|s_| s_.rsplit_once('(')) else {
            return Self::Varbatim(s);
        };
        let mut tokens = p.split_whitespace();
        match (tokens.next(), tokens.next(), tokens.next(), tokens.next()) {
            (Some(lhs), Some("x"), Some(rhs), None) => {
                let (lhs_expr, rhs_expr): (::syn::Expr, ::syn::Expr) = (
                    ::syn::parse_quote!(lhs), ::syn::parse_quote!(rhs)
                );
                Self::Slice {
                    raw: s.clone(),
                    size_expr: if rhs.starts_with("mjN") || rhs.parse::<usize>().is_ok() {
                        ::syn::parse_quote! { self.#lhs_expr * #rhs_expr }
                    } else if rhs.chars().all(|c| matches!(c, 'a'..='z' | '_')) {
                        ::syn::parse_quote!  { self.#lhs_expr * self.#rhs_expr }
                    } else {
                        unreachable!("unexpected pattern of slice field comment: `{lhs} x {rhs}`")
                    }
                }
            },
            _ => Self::Varbatim(s)
        }
    }
}

impl Bindings {
    pub fn new(include_dir: impl AsRef<std::path::Path>) -> std::io::Result<Self> {
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

        let mut buf = Vec::new();
        ::bindgen::builder()
            /* common */
            .header_contents("bindgen.h", "#include \"mujoco.h\"")
            .clang_args([
                format!("-I{}", include_dir.as_ref().display()),
                format!("-I{}", include_dir.as_ref().join("mujoco").display()),
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
            .map_err(std::io::Error::other)?
            .write(&mut buf)?;

        let buf = String::from_utf8(buf)
            .map_err(std::io::Error::other)?;
        Self::parse_file(&buf)
    }

    fn parse_file(content: &str) -> std::io::Result<Self> {
        use ::quote::ToTokens;

        let syn_file = ::syn::parse_file(content)
            .map_err(std::io::Error::other)?;

        let mut this = Self {
            globalattrs: syn_file.attrs,
            constants: Vec::new(),
            typealiases: Vec::new(),
            enums: Vec::new(),
            unions: Vec::new(),
            structs: Vec::new(),
            functions: Vec::new(),
            statics: Vec::new(),
        };

        for item in syn_file.items {
            match item {
                ::syn::Item::Const(x) => this.constants.push(x),
                ::syn::Item::Type(x) => this.typealiases.push(x),
                ::syn::Item::Union(x) => this.unions.push(x),
                ::syn::Item::Struct(x) => match &x.fields {
                    ::syn::Fields::Named(_) => this.structs.push(BindingStruct(x)),
                    ::syn::Fields::Unnamed(_) => this.enums.push(BindingEnum(x)),
                    ::syn::Fields::Unit => unreachable!("unexpected unit struct")
                },
                ::syn::Item::ForeignMod(f) => for fitem in f.items {
                    match fitem {
                        ::syn::ForeignItem::Fn(x) => this.functions.push(x),
                        ::syn::ForeignItem::Static(x) => this.statics.push(x),
                        _ => unreachable!("unexpected foreign item in bindgen output: `{}`", fitem.to_token_stream())
                    }
                },
                _ => unreachable!("unexpected item in bindgen output: `{}`", item.to_token_stream())
            }
        }

        Ok(this)
    }
}

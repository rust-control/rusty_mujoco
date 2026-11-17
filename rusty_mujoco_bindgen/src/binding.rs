use quote::ToTokens;

pub struct Binding {
    pub globalattrs: Vec<::syn::Attribute>,
    pub constants: Vec<::syn::ItemConst>,
    pub typealiases: Vec<::syn::ItemType>,
    pub unions: Vec<::syn::ItemUnion>,
    pub enums: Vec<BindingEnum>,
    pub structs: Vec<BindingStruct>,
    pub foreign_statics: Vec<::syn::ForeignItemStatic>,
    pub foreign_functions: Vec<::syn::ForeignItemFn>,
}

pub struct BindingEnum {
    pub variant_consts: Vec<::syn::ImplItemConst>,
    pub bitflag_impls: Option<Box<[::syn::ItemImpl; 4]>>,
    pub attrs: Vec<::syn::Attribute>,
    pub vis: ::syn::Visibility,
    pub ident: ::syn::Ident,
    pub internal_vis: ::syn::Visibility,
    pub internal_type: ::syn::Type,
}

pub struct BindingStruct {
    pub attrs: Vec<::syn::Attribute>,
    pub vis: ::syn::Visibility,
    pub ident: ::syn::Ident,
    pub fields: Vec<BindingStructField>,
}

pub struct BindingStructField {
    pub comment: Option<BindingStructFieldComment>,
    pub vis: ::syn::Visibility,
    pub ident: ::syn::Ident,
    pub ty: ::syn::Type,
}

pub enum BindingStructFieldComment {
    Slice { text: String, size_expr: ::syn::Expr },
    Varbatim(String),
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
                    text: s.clone(),
                    size_expr: if rhs.starts_with("mjN") || rhs.parse::<usize>().is_ok() {
                        ::syn::parse_quote! { (self.#lhs_expr as usize) * #rhs_expr }
                    } else if rhs.chars().all(|c| matches!(c, 'a'..='z' | '_')) {
                        ::syn::parse_quote!  { (self.#lhs_expr as usize) * (self.#rhs_expr as usize) }
                    } else {
                        unreachable!("unexpected pattern of slice field comment: `{lhs} x {rhs}`")
                    }
                }
            },
            _ => Self::Varbatim(s)
        }
    }
}

impl Binding {
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

        let buf = String::from_utf8(buf).map_err(std::io::Error::other)?;
        let mut parsing_stack = Vec::new();
        let mut this = Self::parse_file(&buf, &mut parsing_stack)?;
        this.apply_rusty_mujoco_convention();
        Ok(this)
    }

    fn apply_rusty_mujoco_convention(&mut self) {
        for x in &mut self.typealiases {
            if x.ty.to_token_stream().to_string().contains("unsafe extern") {
                x.vis = ::syn::parse_quote!(pub(crate));
            }
        }
        for x in &mut self.unions {
            x.vis = ::syn::parse_quote!(pub(crate));
            if x.ident.to_string().contains("__bindgen_ty_") {
                x.ident = ::quote::format_ident!("{}", x.ident.to_string().replace("__bindgen_ty_", ""));
            }
        }
        for x in &mut self.constants {
            let ident_str = x.ident.to_string();
            if let ::syn::Expr::Lit(::syn::ExprLit { lit: ::syn::Lit::Int(_), .. }) = &*x.expr &&
                (ident_str.starts_with("mjMAX") || ident_str.starts_with("mjMIN") || ident_str.starts_with("mjN"))
            {
                *x.ty = ::syn::parse_quote!(usize);
            }
            if let ::syn::Expr::Path(::syn::ExprPath { path, .. }) = &*x.expr &&
                ident_str.starts_with("mjN")
            {/* the consitied enum variant by `MakeMjnVariantGlobalConstCallbacks` */
                *x.ty = ::syn::parse_quote!(usize);
                *x.expr = ::syn::parse_quote!(#path.0 as usize);
            }
        }
        for x in &mut self.structs {
            if x.ident.to_string().contains("__bindgen_ty_") {
                x.ident = ::quote::format_ident!("{}", x.ident.to_string().replace("__bindgen_ty_", ""));
            }
            for f in &mut x.fields {
                let ty_str = f.ty.to_token_stream().to_string();
                if ty_str.contains("__bindgen_ty_") {
                    f.ty = ::syn::parse_str(&ty_str.replace("__bindgen_ty_", "")).unwrap();
                }
                if f.ident.to_string().ends_with("id")
                    || (matches!(f.ty, ::syn::Type::Ptr(_)) && !ty_str.ends_with("c_void"))
                    || ty_str.contains("unsafe extern")
                    || ty_str.contains("c_char")
                {
                    f.vis = ::syn::parse_quote!(pub(crate));
                }
            }
        }
        for x in &mut self.enums {
            x.internal_vis = ::syn::parse_quote!(pub(crate));
            for v in &mut x.variant_consts {
                assert!(v.ident.to_string().starts_with("mj"));
                let after_mj_prefix = v.ident.to_string().split_once('_').unwrap().1.to_owned();
                v.ident = if after_mj_prefix.starts_with("2D") {
                    ::quote::format_ident!("{}", &after_mj_prefix.replace("2D", "D2"))
                } else if after_mj_prefix.chars().next().unwrap().is_ascii_digit() {
                    ::quote::format_ident!("X{after_mj_prefix}")
                } else {
                    ::quote::format_ident!("{after_mj_prefix}")
                };
            }
            if let Some(prefix_) = x.variant_consts[0].ident.to_string().rsplit_once('_').map(|(p, _)| p.to_owned()) &&
                x.variant_consts[1..].iter().all(|v| v.ident.to_string().starts_with(&prefix_))
            {
                for v in &mut x.variant_consts {
                    v.ident = ::quote::format_ident!(
                        "{}",
                        v.ident.to_string().strip_prefix(&prefix_).unwrap().trim_start_matches('_')
                    );
                }
            }
        }
        for x in &mut self.foreign_functions {
            x.vis = ::syn::parse_quote!(pub(crate));
        }
    }

    fn parse_file(content: &str, stack: &mut Vec<::syn::Item>) -> std::io::Result<Self> {
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
            foreign_functions: Vec::new(),
            foreign_statics: Vec::new(),
        };

        for item in syn_file.items {
            match item {
                ::syn::Item::Const(x) => this.constants.push(x),
                ::syn::Item::Type(x) => this.typealiases.push(x),
                ::syn::Item::Union(x) => this.unions.push(x),
                ::syn::Item::Struct(x) => this.dispatch_syn_item_struct(x, stack),
                ::syn::Item::ForeignMod(f) => for fitem in f.items {
                    match fitem {
                        ::syn::ForeignItem::Fn(x) => this.foreign_functions.push(x),
                        ::syn::ForeignItem::Static(x) => this.foreign_statics.push(x),
                        _ => unreachable!("unexpected foreign item in bindgen output: `{}`", fitem.to_token_stream())
                    }
                },
                x => stack.push(x),
            }
        }

        if let Some(x) = stack.pop() {
            return Err(std::io::Error::other(format!(
                "unhandled item remained: `{}`",
                x.to_token_stream()
            )));
        }

        Ok(this)
    }

    fn dispatch_syn_item_struct(&mut self, s: ::syn::ItemStruct, stack: &mut Vec<::syn::Item>) {
        use ::quote::ToTokens;

        let pop_impl_t = |t: &::syn::Ident, stack: &mut Vec<syn::Item>| -> ::syn::ItemImpl {
            match stack.pop() {
                Some(::syn::Item::Impl(i)) if 
                    i.trait_.is_none() &&
                    i.self_ty.to_token_stream().to_string() == t.to_string() => i,
                _ => panic!("expected `impl {t} {{ ... }}`"),
            }
        };
        let pop_impl_trait_for_t = |
            trait_name: &str,
            t: &::syn::Ident,
            stack: &mut Vec<syn::Item>
        | -> ::syn::ItemImpl {
            match stack.pop() {
                Some(::syn::Item::Impl(i)) if 
                    i.trait_.as_ref().is_some_and(|(p, _)| p.segments.last().unwrap().ident == trait_name) &&
                    i.self_ty.to_token_stream().to_string() == t.to_string() => i,
                _ => panic!("expected `impl {trait_name} for {t} {{ ... }}`"),
            }
        };

        match s.fields {
            ::syn::Fields::Named(f) => self.structs.push(BindingStruct {
                attrs: s.attrs,
                vis: s.vis,
                ident: s.ident,
                fields: f.named.into_iter().map(|f| BindingStructField {
                    comment: BindingStructFieldComment::extract(f.attrs.iter()),
                    vis: f.vis,
                    ident: f.ident.unwrap(),
                    ty: f.ty,
                }).collect(),
            }),
            ::syn::Fields::Unnamed(mut f) => {
                let Some(single_field) = f.unnamed.pop().and_then(|x| f.unnamed.pop().is_none().then_some(x)) else {
                    panic!("expected newtype struct for enum definition");
                };
                let (internal_vis, internal_type) = (single_field.vis, single_field.ty);
                let bitflag_impls = stack
                    .last()
                    .is_some_and(|l| matches!(l, ::syn::Item::Impl(::syn::ItemImpl { trait_: Some(_), .. })))
                    .then(|| Box::new([
                        pop_impl_trait_for_t("BiAndAssign", &s.ident, stack),
                        pop_impl_trait_for_t("BitAnd", &s.ident, stack),
                        pop_impl_trait_for_t("BitOrAssign", &s.ident, stack),
                        pop_impl_trait_for_t("BitOr", &s.ident, stack),
                    ]));
                let variant_consts = pop_impl_t(&s.ident, stack)
                    .items
                    .into_iter()
                    .filter_map(|i| match i {
                        ::syn::ImplItem::Const(c) => (!c.ident.to_string().starts_with("mjN")).then_some(c),
                        _ => panic!("unexpected item in `impl {}`", s.ident)
                    })
                    .collect();
                self.enums.push(BindingEnum {
                    internal_vis,
                    internal_type,
                    bitflag_impls,
                    variant_consts,
                    attrs: s.attrs,
                    vis: s.vis,
                    ident: s.ident,
                });
            },
            ::syn::Fields::Unit => unreachable!("unexpected unit struct: `{}`", s.ident)
        }
    }
}

impl ::quote::ToTokens for Binding {
    fn to_tokens(&self, t: &mut ::quote::__private::TokenStream) {
        self.globalattrs.iter().for_each(|x| x.to_tokens(t));
        self.constants.iter().for_each(|x| x.to_tokens(t));
        self.typealiases.iter().for_each(|x| x.to_tokens(t));
        self.unions.iter().for_each(|x| x.to_tokens(t));
        self.structs.iter().for_each(|x| x.to_tokens(t));
        self.enums.iter().for_each(|x| x.to_tokens(t));
        let mut foreign = ::quote::__private::TokenStream::new();
        self.foreign_statics.iter().for_each(|x| x.to_tokens(&mut foreign));
        self.foreign_functions.iter().for_each(|x| x.to_tokens(&mut foreign));
        t.extend(::quote::quote! {
            pub(crate) mod functions {
                use super::*;
                unsafe extern "C" {
                    #foreign
                }
            }
        });
    }
}

impl ::quote::ToTokens for BindingStruct {
    fn to_tokens(&self, t: &mut ::quote::__private::TokenStream) {
        let BindingStruct { attrs, vis, ident, fields } = self;
        t.extend(::quote::quote! {
            #(#attrs)*
            #vis struct #ident {
                #(#fields)*
            }
        });
    }
}

impl ::quote::ToTokens for BindingStructField {
    fn to_tokens(&self, t: &mut ::quote::__private::TokenStream) {
        let Self { comment, vis, ident, ty } = self;
        t.extend(::quote::quote! {
            #comment
            #vis #ident: #ty,
        });
    }
}

impl ::quote::ToTokens for BindingStructFieldComment {
    fn to_tokens(&self, t: &mut ::quote::__private::TokenStream) {
        match self {
            Self::Varbatim(x) => {
                let lit = ::syn::LitStr::new(x, ::quote::__private::Span::call_site());
                t.extend(::quote::quote! { #[doc = #lit] })
            }
            Self::Slice { text, size_expr: _ } => {
                let text = ::syn::LitStr::new(text, ::quote::__private::Span::call_site());
                t.extend(::quote::quote! { #[doc = #text] });
            }
        }
    }
}

impl ::quote::ToTokens for BindingEnum {
    fn to_tokens(&self, t: &mut ::quote::__private::TokenStream) {
        let BindingEnum { variant_consts, bitflag_impls, attrs, vis, ident, internal_vis, internal_type } = self;
        let bitflag_impls = bitflag_impls.as_deref().map_or(&[][..], |i| &i[..]);
        t.extend(::quote::quote! {
            #(#attrs)*
            #vis struct #ident(#internal_vis #internal_type);
            impl #ident {
                #(#variant_consts)*
            }
            #(#bitflag_impls)*
        });
    }
}

impl std::fmt::Display for Binding {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        ::quote::ToTokens::into_token_stream(self).fmt(f)
    }
}

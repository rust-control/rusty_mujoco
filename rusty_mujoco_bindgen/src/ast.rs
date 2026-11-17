pub struct Ast(Vec<::syn::Item>);

impl Ast {
    pub fn parse_file(content: String) -> std::io::Result<Self> {
        let syn_file = ::syn::parse_file(&content).map_err(std::io::Error::other)?;
        Ok(Self(syn_file.items))
    }

    pub fn __
        use quote::ToTokens;
        let mut r#final = quote::quote! {
            #![allow(unused, non_camel_case_types, non_snake_case, non_upper_case_globals, clippy::all)]
        };

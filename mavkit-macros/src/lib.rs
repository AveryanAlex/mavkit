use proc_macro::TokenStream;
use proc_macro2::TokenStream as TokenStream2;
use quote::{format_ident, quote};
use syn::{
    Attribute, DeriveInput, ExprPath, Field, Ident, LitInt, Token, Type, parse_macro_input,
    parse_quote,
};

/// Generates wire conversion functions for a mission command struct.
///
/// Annotate a struct with `#[mavkit_command(id = MAV_CMD_..., category = Nav)]`
/// and its fields with `#[param(N)]`, `#[wire_x]`, `#[wire_y]`, `#[wire_z]`,
/// or `#[position]` to generate `to_wire` and `from_wire` functions.
///
/// # Examples
///
/// ```ignore
/// #[mavkit_command(id = MAV_CMD_NAV_WAYPOINT, category = Nav)]
/// pub struct NavWaypoint {
///     #[position]
///     pub position: GeoPoint3d,
///     #[param(1)]
///     pub hold_time_s: f32,
///     #[param(2)]
///     pub acceptance_radius_m: f32,
///     #[param(3)]
///     pub pass_radius_m: f32,
///     #[param(4)]
///     pub yaw_deg: f32,
/// }
/// ```
///
/// Generates `nav_waypoint_to_wire(command: NavWaypoint) -> (MissionFrame, [f32; 4], i32, i32, f32)`
/// and the corresponding `nav_waypoint_from_wire` function.
#[proc_macro_attribute]
pub fn mavkit_command(attr: TokenStream, item: TokenStream) -> TokenStream {
    let input = parse_macro_input!(item as DeriveInput);
    let _cmd_attr = parse_macro_input!(attr as CommandAttr);

    match mavkit_command_impl(input) {
        Ok(tokens) => tokens.into(),
        Err(err) => err.to_compile_error().into(),
    }
}

/// Parsed struct-level `mavkit_command(id = ..., category = ...)` attribute.
struct CommandAttr {
    _id: ExprPath,
    _category: Ident,
}

impl syn::parse::Parse for CommandAttr {
    fn parse(input: syn::parse::ParseStream) -> syn::Result<Self> {
        let mut id: Option<ExprPath> = None;
        let mut category: Option<Ident> = None;

        while !input.is_empty() {
            let key: Ident = input.parse()?;
            input.parse::<Token![=]>()?;

            match key.to_string().as_str() {
                "id" => {
                    id = Some(input.parse()?);
                }
                "category" => {
                    category = Some(input.parse()?);
                }
                other => {
                    return Err(syn::Error::new(
                        key.span(),
                        format!("unknown attribute `{other}`, expected `id` or `category`"),
                    ));
                }
            }

            if !input.is_empty() {
                input.parse::<Token![,]>()?;
            }
        }

        Ok(CommandAttr {
            _id: id.ok_or_else(|| input.error("missing `id` attribute"))?,
            _category: category.ok_or_else(|| input.error("missing `category` attribute"))?,
        })
    }
}

fn mavkit_command_impl(input: DeriveInput) -> syn::Result<TokenStream2> {
    let struct_name = &input.ident;

    let fields = match &input.data {
        syn::Data::Struct(data) => match &data.fields {
            syn::Fields::Named(named) => &named.named,
            syn::Fields::Unit => {
                return Ok(generate_unit_command(input));
            }
            syn::Fields::Unnamed(_) => {
                return Err(syn::Error::new_spanned(
                    struct_name,
                    "mavkit_command does not support tuple structs",
                ));
            }
        },
        _ => {
            return Err(syn::Error::new_spanned(
                struct_name,
                "mavkit_command only supports structs",
            ));
        }
    };

    if fields.is_empty() {
        return Ok(generate_unit_command(input));
    }

    // Parse field annotations
    let mut parsed_fields = Vec::new();
    for field in fields {
        parsed_fields.push(parse_field(field)?);
    }

    let has_position = parsed_fields
        .iter()
        .any(|f| matches!(f.kind, FieldKind::Position));

    // Build the struct output with derives
    let struct_output = emit_struct_with_derives(&input);

    let snake_name = to_snake_case(&struct_name.to_string());
    let to_wire_fn = format_ident!("{snake_name}_to_wire");
    let from_wire_fn = format_ident!("{snake_name}_from_wire");

    let (to_wire_body, from_wire_body) = if has_position {
        generate_position_command(&parsed_fields, struct_name)?
    } else {
        generate_non_position_command(&parsed_fields, struct_name)?
    };

    Ok(quote! {
        #struct_output

        fn #to_wire_fn(command: #struct_name) -> (MissionFrame, [f32; 4], i32, i32, f32) {
            #to_wire_body
        }

        fn #from_wire_fn(
            frame: MissionFrame,
            params: [f32; 4],
            x: i32,
            y: i32,
            z: f32,
        ) -> #struct_name {
            #from_wire_body
        }
    })
}

/// Generate code for a unit command (no fields).
fn generate_unit_command(input: DeriveInput) -> TokenStream2 {
    let struct_name = &input.ident;
    let struct_output = emit_struct_with_derives(&input);

    let snake_name = to_snake_case(&struct_name.to_string());
    let to_wire_fn = format_ident!("{snake_name}_to_wire");
    let from_wire_fn = format_ident!("{snake_name}_from_wire");

    quote! {
        #struct_output

        fn #to_wire_fn(_command: #struct_name) -> (MissionFrame, [f32; 4], i32, i32, f32) {
            unit_command_to_wire()
        }

        fn #from_wire_fn(
            _frame: MissionFrame,
            _params: [f32; 4],
            _x: i32,
            _y: i32,
            _z: f32,
        ) -> #struct_name {
            #struct_name
        }
    }
}

// --- Field annotation parsing ---

enum FieldKind {
    /// `#[param(N)]` with optional via/from
    Param {
        index: usize,
        custom_encode: Option<Box<ExprPath>>,
        custom_decode: Option<Box<ExprPath>>,
    },
    /// `#[wire_x]`
    WireX,
    /// `#[wire_y]`
    WireY,
    /// `#[wire_z]`
    WireZ,
    /// `#[position]`
    Position,
}

struct ParsedField {
    ident: Ident,
    ty: Type,
    kind: FieldKind,
}

fn parse_field(field: &Field) -> syn::Result<ParsedField> {
    let ident = field.ident.clone().unwrap();
    let ty = field.ty.clone();

    let mut kind: Option<FieldKind> = None;

    for attr in &field.attrs {
        if let Some(k) = parse_field_attr(attr)? {
            if kind.is_some() {
                return Err(syn::Error::new_spanned(
                    attr,
                    "field cannot have multiple wire mapping attributes",
                ));
            }
            kind = Some(k);
        }
    }

    let kind = kind.ok_or_else(|| {
        syn::Error::new_spanned(
            &ident,
            format!(
                "field `{}` has no wire mapping attribute (#[param(N)], #[wire_x], #[wire_y], #[wire_z], or #[position])",
                ident
            ),
        )
    })?;

    Ok(ParsedField { ident, ty, kind })
}

fn parse_field_attr(attr: &Attribute) -> syn::Result<Option<FieldKind>> {
    let path = attr.path();

    if path.is_ident("position") {
        return Ok(Some(FieldKind::Position));
    }
    if path.is_ident("wire_x") {
        return Ok(Some(FieldKind::WireX));
    }
    if path.is_ident("wire_y") {
        return Ok(Some(FieldKind::WireY));
    }
    if path.is_ident("wire_z") {
        return Ok(Some(FieldKind::WireZ));
    }
    if path.is_ident("param") {
        return parse_param_attr(attr).map(Some);
    }

    // Not one of our attributes — ignore it (could be doc, serde, etc.)
    Ok(None)
}

/// Parse `#[param(N)]` or `#[param(N, via = encode, from = decode)]`.
fn parse_param_attr(attr: &Attribute) -> syn::Result<FieldKind> {
    attr.parse_args_with(|input: syn::parse::ParseStream| {
        let index_lit: LitInt = input.parse()?;
        let index: usize = index_lit.base10_parse()?;
        if !(1..=4).contains(&index) {
            return Err(syn::Error::new(
                index_lit.span(),
                "param index must be 1..=4",
            ));
        }

        let mut custom_encode = None;
        let mut custom_decode = None;

        if input.peek(Token![,]) {
            input.parse::<Token![,]>()?;

            // Parse `via = encode_fn, from = decode_fn`
            while !input.is_empty() {
                let key: Ident = input.parse()?;
                input.parse::<Token![=]>()?;

                match key.to_string().as_str() {
                    "via" => {
                        custom_encode = Some(Box::new(input.parse::<ExprPath>()?));
                    }
                    "from" => {
                        custom_decode = Some(Box::new(input.parse::<ExprPath>()?));
                    }
                    other => {
                        return Err(syn::Error::new(
                            key.span(),
                            format!("unknown param option `{other}`, expected `via` or `from`"),
                        ));
                    }
                }

                if !input.is_empty() {
                    input.parse::<Token![,]>()?;
                }
            }

            // Both or neither must be specified
            if custom_encode.is_some() != custom_decode.is_some() {
                return Err(input.error("`via` and `from` must both be specified"));
            }
        }

        Ok(FieldKind::Param {
            index: index - 1, // Convert to 0-based
            custom_encode,
            custom_decode,
        })
    })
}

// --- Code generation for position commands ---

fn generate_position_command(
    fields: &[ParsedField],
    struct_name: &Ident,
) -> syn::Result<(TokenStream2, TokenStream2)> {
    let position_field = fields
        .iter()
        .find(|f| matches!(f.kind, FieldKind::Position))
        .unwrap();
    let position_ident = &position_field.ident;

    // Build params array for to_wire
    let mut param_to_wire: [TokenStream2; 4] = [
        quote! { 0.0 },
        quote! { 0.0 },
        quote! { 0.0 },
        quote! { 0.0 },
    ];

    // Build from_wire field initializers
    let mut field_inits = Vec::new();
    field_inits.push(quote! {
        #position_ident: position_from_wire(frame, x, y, z)
    });

    for field in fields {
        let ident = &field.ident;
        match &field.kind {
            FieldKind::Position => {} // Handled above
            FieldKind::Param {
                index,
                custom_encode,
                custom_decode,
            } => {
                param_to_wire[*index] = param_to_wire_expr(field, custom_encode.as_deref());
                let from_expr = param_from_wire_expr(field, *index, custom_decode.as_deref());
                field_inits.push(quote! { #ident: #from_expr });
            }
            FieldKind::WireX | FieldKind::WireY | FieldKind::WireZ => {
                return Err(syn::Error::new_spanned(
                    ident,
                    "wire_x/wire_y/wire_z cannot be used together with #[position] (position owns x, y, z)",
                ));
            }
        }
    }

    let p0 = &param_to_wire[0];
    let p1 = &param_to_wire[1];
    let p2 = &param_to_wire[2];
    let p3 = &param_to_wire[3];

    let to_wire = quote! {
        position_command_to_wire(
            command.#position_ident,
            [#p0, #p1, #p2, #p3],
        )
    };

    let from_wire = quote! {
        #struct_name {
            #(#field_inits),*
        }
    };

    Ok((to_wire, from_wire))
}

// --- Code generation for non-position commands ---

fn generate_non_position_command(
    fields: &[ParsedField],
    struct_name: &Ident,
) -> syn::Result<(TokenStream2, TokenStream2)> {
    let mut param_to_wire: [TokenStream2; 4] = [
        quote! { 0.0 },
        quote! { 0.0 },
        quote! { 0.0 },
        quote! { 0.0 },
    ];
    let mut x_to_wire = quote! { 0 };
    let mut y_to_wire = quote! { 0 };
    let mut z_to_wire = quote! { 0.0 };

    let mut field_inits = Vec::new();

    for field in fields {
        let ident = &field.ident;
        match &field.kind {
            FieldKind::Param {
                index,
                custom_encode,
                custom_decode,
            } => {
                param_to_wire[*index] = param_to_wire_expr(field, custom_encode.as_deref());
                let from_expr = param_from_wire_expr(field, *index, custom_decode.as_deref());
                field_inits.push(quote! { #ident: #from_expr });
            }
            FieldKind::WireX => {
                x_to_wire = wire_x_to_wire_expr(field);
                let from_expr = wire_x_from_wire_expr(field);
                field_inits.push(quote! { #ident: #from_expr });
            }
            FieldKind::WireY => {
                y_to_wire = wire_y_to_wire_expr(field);
                let from_expr = wire_y_from_wire_expr(field);
                field_inits.push(quote! { #ident: #from_expr });
            }
            FieldKind::WireZ => {
                z_to_wire = wire_z_to_wire_expr(field);
                let from_expr = wire_z_from_wire_expr(field);
                field_inits.push(quote! { #ident: #from_expr });
            }
            FieldKind::Position => {
                return Err(syn::Error::new_spanned(
                    ident,
                    "internal error: position field in non-position path",
                ));
            }
        }
    }

    let p0 = &param_to_wire[0];
    let p1 = &param_to_wire[1];
    let p2 = &param_to_wire[2];
    let p3 = &param_to_wire[3];

    let to_wire = quote! {
        mission_command_to_wire(
            [#p0, #p1, #p2, #p3],
            #x_to_wire,
            #y_to_wire,
            #z_to_wire,
        )
    };

    let from_wire = quote! {
        #struct_name {
            #(#field_inits),*
        }
    };

    Ok((to_wire, from_wire))
}

// --- Type-aware conversion expressions ---

/// Build the expression for converting a field value to a param f32 (for to_wire).
fn param_to_wire_expr(field: &ParsedField, custom_encode: Option<&ExprPath>) -> TokenStream2 {
    let ident = &field.ident;

    if let Some(encode_fn) = custom_encode {
        return quote! { #encode_fn(command.#ident) };
    }

    let type_name = type_name_str(&field.ty);
    match type_name.as_str() {
        "f32" => quote! { command.#ident },
        "bool" => quote! { bool_to_param(command.#ident) },
        "u8" | "u16" => quote! { f32::from(command.#ident) },
        "u32" => quote! { command.#ident as f32 },
        "i8" => quote! { f32::from(command.#ident) },
        _ => quote! { command.#ident as f32 },
    }
}

/// Build the expression for converting a param f32 back to the field type (for from_wire).
fn param_from_wire_expr(
    field: &ParsedField,
    index: usize,
    custom_decode: Option<&ExprPath>,
) -> TokenStream2 {
    let idx = syn::Index::from(index);

    if let Some(decode_fn) = custom_decode {
        return quote! { #decode_fn(params[#idx]) };
    }

    let type_name = type_name_str(&field.ty);
    match type_name.as_str() {
        "f32" => quote! { params[#idx] },
        "bool" => quote! { bool_from_param(params[#idx]) },
        "u8" => quote! { u8_from_param(params[#idx]) },
        "u16" => quote! { u16_from_param(params[#idx]) },
        "u32" => quote! { u32_from_param(params[#idx]) },
        "i8" => quote! { i8_from_param(params[#idx]) },
        _ => quote! { params[#idx] },
    }
}

/// Build the expression for converting a field to wire x (i32).
fn wire_x_to_wire_expr(field: &ParsedField) -> TokenStream2 {
    let ident = &field.ident;
    let type_name = type_name_str(&field.ty);
    match type_name.as_str() {
        "i32" => quote! { command.#ident },
        "u32" => quote! { command.#ident as i32 },
        "u8" => quote! { i32::from(command.#ident) },
        "f32" => quote! { command.#ident as i32 },
        _ => quote! { command.#ident as i32 },
    }
}

/// Build the expression for converting wire x (i32) back to field type.
fn wire_x_from_wire_expr(field: &ParsedField) -> TokenStream2 {
    let type_name = type_name_str(&field.ty);
    match type_name.as_str() {
        "i32" => quote! { x },
        "u32" => quote! { x as u32 },
        "u8" => quote! { x as u8 },
        "f32" => quote! { x as f32 },
        _ => quote! { x },
    }
}

/// Build the expression for converting a field to wire y (i32).
fn wire_y_to_wire_expr(field: &ParsedField) -> TokenStream2 {
    let ident = &field.ident;
    let type_name = type_name_str(&field.ty);
    match type_name.as_str() {
        "i32" => quote! { command.#ident },
        "u32" => quote! { command.#ident as i32 },
        "u8" => quote! { i32::from(command.#ident) },
        "f32" => quote! { command.#ident as i32 },
        _ => quote! { command.#ident as i32 },
    }
}

/// Build the expression for converting wire y (i32) back to field type.
fn wire_y_from_wire_expr(field: &ParsedField) -> TokenStream2 {
    let type_name = type_name_str(&field.ty);
    match type_name.as_str() {
        "i32" => quote! { y },
        "u32" => quote! { y as u32 },
        "u8" => quote! { y as u8 },
        "f32" => quote! { y as f32 },
        _ => quote! { y },
    }
}

/// Build the expression for converting a field to wire z (f32).
fn wire_z_to_wire_expr(field: &ParsedField) -> TokenStream2 {
    let ident = &field.ident;
    let type_name = type_name_str(&field.ty);
    match type_name.as_str() {
        "f32" => quote! { command.#ident },
        "u8" => quote! { f32::from(command.#ident) },
        _ => quote! { command.#ident as f32 },
    }
}

/// Build the expression for converting wire z (f32) back to field type.
fn wire_z_from_wire_expr(field: &ParsedField) -> TokenStream2 {
    let type_name = type_name_str(&field.ty);
    match type_name.as_str() {
        "f32" => quote! { z },
        "u8" => quote! { z.round() as u8 },
        _ => quote! { z },
    }
}

// --- Struct output with derives ---

/// Emit the struct with `#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]`,
/// merging with any existing derives from the input.
fn emit_struct_with_derives(input: &DeriveInput) -> TokenStream2 {
    let name = &input.ident;
    let vis = &input.vis;
    let generics = &input.generics;

    let required_derives: Vec<syn::Path> = vec![
        parse_quote!(Debug),
        parse_quote!(Clone),
        parse_quote!(PartialEq),
        parse_quote!(Serialize),
        parse_quote!(Deserialize),
    ];

    // Collect existing derives from the input
    let mut existing_derives = Vec::new();
    let mut other_attrs = Vec::new();

    for attr in &input.attrs {
        if attr.path().is_ident("derive") {
            if let Ok(paths) = attr.parse_meta_fallback() {
                existing_derives.extend(paths);
            }
        } else {
            other_attrs.push(attr);
        }
    }

    // Merge: add required derives that aren't already present
    let mut all_derives = existing_derives.clone();
    for required in &required_derives {
        let already_present = existing_derives
            .iter()
            .any(|existing| path_matches(existing, required));
        if !already_present {
            all_derives.push(required.clone());
        }
    }

    // Strip our custom attributes from fields but keep others (doc, serde, etc.)
    let fields = match &input.data {
        syn::Data::Struct(data) => match &data.fields {
            syn::Fields::Named(named) => {
                let filtered: Vec<TokenStream2> = named
                    .named
                    .iter()
                    .map(|f| {
                        let field_vis = &f.vis;
                        let ident = &f.ident;
                        let ty = &f.ty;
                        let attrs: Vec<&Attribute> = f
                            .attrs
                            .iter()
                            .filter(|a| !is_mavkit_field_attr(a))
                            .collect();
                        quote! {
                            #(#attrs)*
                            #field_vis #ident: #ty
                        }
                    })
                    .collect();
                quote! { { #(#filtered),* } }
            }
            syn::Fields::Unit => quote! { ; },
            syn::Fields::Unnamed(_) => quote! { /* unreachable */ },
        },
        _ => quote! {},
    };

    quote! {
        #(#other_attrs)*
        #[derive(#(#all_derives),*)]
        #vis struct #name #generics #fields
    }
}

/// Check if an attribute is one of our field-level custom attributes.
fn is_mavkit_field_attr(attr: &Attribute) -> bool {
    let path = attr.path();
    path.is_ident("param")
        || path.is_ident("position")
        || path.is_ident("wire_x")
        || path.is_ident("wire_y")
        || path.is_ident("wire_z")
}

/// Compare two paths by their last segment (handles `Debug` vs `std::fmt::Debug`).
fn path_matches(a: &syn::Path, b: &syn::Path) -> bool {
    let a_last = a.segments.last().map(|s| &s.ident);
    let b_last = b.segments.last().map(|s| &s.ident);
    a_last == b_last
}

/// Extract the simple type name from a `Type` (e.g. `f32`, `bool`, `u8`).
/// Returns the last path segment as a string.
fn type_name_str(ty: &Type) -> String {
    if let Type::Path(type_path) = ty {
        if let Some(segment) = type_path.path.segments.last() {
            return segment.ident.to_string();
        }
    }
    String::new()
}

/// Convert `PascalCase` struct name to `snake_case`.
fn to_snake_case(name: &str) -> String {
    let mut result = String::new();
    for (i, ch) in name.chars().enumerate() {
        if ch.is_uppercase() {
            if i > 0 {
                result.push('_');
            }
            result.push(ch.to_lowercase().next().unwrap());
        } else {
            result.push(ch);
        }
    }
    result
}

/// Helper trait for parsing derive lists from attributes.
trait ParseMetaFallback {
    fn parse_meta_fallback(&self) -> syn::Result<Vec<syn::Path>>;
}

impl ParseMetaFallback for Attribute {
    fn parse_meta_fallback(&self) -> syn::Result<Vec<syn::Path>> {
        self.parse_args_with(|input: syn::parse::ParseStream| {
            let paths =
                syn::punctuated::Punctuated::<syn::Path, Token![,]>::parse_terminated(input)?;
            Ok(paths.into_iter().collect())
        })
    }
}

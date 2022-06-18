/// Defines a standard structure whose fields all have `with_fieldX` constructors.
///
/// This macro takes `StructName, {structfield1: type1, structfield2: type2, ...}` as arguments
/// and generates a structure:
///
/// ```compile_fail
/// struct StructName {
///     structfield1: type1,
///     structfield2: type2,
///     ...
/// }
///
/// impl StructName{
///     fn with_structfield1(self, new_structfield1: type1) -> Self {..}
///     fn with_structfield2(self, new_structfield2: type2) -> Self {..}
/// }
/// ```
///
#[macro_export]
macro_rules! create_struct_with {
    ($struct_name:ident, {$($varname:ident : $vartype:ty),*}) => {
        pub struct $struct_name {
            $(pub $varname: $vartype),*
        }

        paste! {
            impl $struct_name {
                $(
                    #[allow(dead_code)]
                    pub fn [<with_ $varname>](self, [<new_ $varname>]: $vartype) -> Self {
                        $struct_name {$varname: [<new_ $varname>], ..self}
                    }
                )*
            }
        }
    };
}

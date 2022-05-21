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
                    fn [<with_ $varname>](self, [<new_ $varname>]: $vartype) -> Self {
                        $struct_name {$varname: [<new_ $varname>], ..self}
                    }
                )*
            }
        }
    };
}

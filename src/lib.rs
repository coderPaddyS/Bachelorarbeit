#![feature(array_try_map)]
#![feature(let_chains)]
#![feature(negative_impls)]
#![feature(type_changing_struct_update)]
#![feature(generic_const_exprs)]
#![feature(iter_array_chunks)]
#![feature(path_add_extension)]

pub mod mesh;
pub use mesh::ClosedTriangleMesh;

pub mod projective_structure;

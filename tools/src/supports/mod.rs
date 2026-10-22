use common::id_type;

pub mod auto;
pub mod mesh;
pub mod parts;
pub mod raft;

pub use parts::{
    BaseConfig, BaseShape, Foot, Part, PartKind, PointKind, Shape, Support, SupportPoint,
    SupportSet,
};

id_type!(SupportId, u32);

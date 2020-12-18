#[allow(
    clippy::all,
    non_camel_case_types,
    non_upper_case_globals,
    non_snake_case,
    improper_ctypes
)]
mod bindings {
    use crate::c_types;
    include!(concat!(env!("OUT_DIR"), "/bindings.rs"));
}
pub use bindings::*;

pub const GFP_KERNEL: gfp_t = BINDINGS_GFP_KERNEL;


pub const REQ_FAILFAST_DEV: u32 = BINDINGS_REQ_FAILFAST_DEV;
pub const REQ_FAILFAST_TRANSPORT: u32 = BINDINGS_REQ_FAILFAST_TRANSPORT;
pub const REQ_FAILFAST_DRIVER: u32 = BINDINGS_REQ_FAILFAST_DRIVER;

pub const NVME_SC_DNR :u16 = BINDINGS_NVME_SC_DNR;
//pub nvme_max_retreis : u8 = nvme_max_retries;

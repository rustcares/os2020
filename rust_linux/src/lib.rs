#![no_std]
#![feature(allocator_api, alloc_error_handler)]

extern crate alloc;

use core::panic::PanicInfo;
use core::mem::size_of;

mod allocator;
pub mod bindings;
pub mod c_types;
pub mod chrdev;
mod error;
pub mod file_operations;
pub mod filesystem;
pub mod printk;
#[cfg(kernel_4_13_0_or_greater)]
pub mod random;
pub mod sysctl;
mod types;
pub mod user_ptr;

pub use crate::error::{Error, KernelResult};
pub use crate::types::{CStr, Mode};


// Use C functions in Rust //
extern "C" {

#[no_mangle]
	fn bug_helper() -> !;
#[no_mangle]
	fn nvme_init() -> c_types::c_int;
#[no_mangle]
    fn nvme_exit() -> !;
#[no_mangle]
    fn nvme_setup_flush( ns : *mut bindings::nvme_ns, cmnd : *mut bindings::nvme_command) -> core::ffi::c_void;
#[no_mangle]    
	fn nvme_setup_discard( ns : *mut bindings::nvme_ns, req : *mut bindings::request, cmnd : *mut bindings::nvme_command) -> bindings::blk_status_t;
#[no_mangle]
	fn nvme_setup_rw( ns : *mut bindings::nvme_ns, req : *mut bindings::request, cmnd : *mut bindings::nvme_command) -> bindings::blk_status_t;
#[no_mangle]
	fn nvme_req(req : *mut bindings::request ) -> *mut bindings::nvme_request;

#[no_mangle]
        fn nvme_pci_use_sgls(dev : *mut bindings::nvme_dev, req : *mut bindings::request) -> bool;
#[no_mangle]
        fn nvme_pci_iod_alloc_size(dev : *mut bindings::nvme_dev, size : u32, nseg : u32, use_sgl : bool) -> u32;
#[no_mangle]
        fn kmalloc_wrapper(size : c_types::c_size_t, flags : bindings::gfp_t) -> *mut c_types::c_void;


#[no_mangle]
    static mut nvme_max_retries: c_types::c_uchar;

}

pub fn nvme_init_fn() -> c_types::c_int {
    unsafe {
        return nvme_init()
    }
}

pub fn nvme_exit_fn() -> ! {
 unsafe {
        nvme_exit();
    }
}

/*
pub fn blk_mq_rq_to_pdu(rq : *mut bindings::request) -> *mut c_types::c_void {
    unsafe {
        let mut ret : *mut c_types::c_void = (rq.offset(1)) as *mut c_types::c_void;
        return ret;
    }
}
*/
pub fn blk_rq_nr_phys_segments(rq : *mut bindings::request) -> u16 {
    unsafe {
        if(((*rq).rq_flags & (1<<18)) != 0){
            return 1;
        }
        return (*rq).nr_phys_segments;
    }
}

pub fn blk_rq_payload_bytes(rq : *mut bindings::request) -> u32 {
    unsafe {
        if(((*rq).rq_flags & (1<<18)) != 0){
            return ((*rq).__bindgen_anon_3).special_vec.bv_len;
        }
        return (*rq).__data_len;
    }
}







//Export Rust Functions to C //
#[no_mangle]
extern "C" fn nvme_req_needs_retry( req : *mut bindings::request ) ->  u8 {

   unsafe {   	
	let nvme_req_ : *mut bindings::nvme_request = (req.offset(1)) as *mut bindings::nvme_request;

	if  ((*req).cmd_flags & ( bindings::req_flag_bits___REQ_FAILFAST_DEV | bindings::req_flag_bits___REQ_FAILFAST_TRANSPORT | bindings::req_flag_bits___REQ_FAILFAST_DRIVER)) != 0 {
		return 0;
	}
	else if  (*nvme_req_).status & ( bindings::NVME_SC_DNR as u16)!= 0 {
		return 0;
	}
	else if  (*nvme_req_).retries >= nvme_max_retries  {
		return 0;
	}
	return 1;
   }
}

#[no_mangle]
extern "C" fn nvme_setup_cmd( ns : *mut bindings::nvme_ns, req : *mut bindings::request, 
				cmd : *mut bindings::nvme_command) -> bindings::blk_status_t {
    unsafe {

	let mut ret : bindings::blk_status_t = bindings::BLK_STS_OK as u8;

	if ((*req).rq_flags & ((1 << 7) as bindings::req_flags_t)) == 0{
	    (*nvme_req(req)).retries = 0;
	    (*nvme_req(req)).flags = 0;
	    (*req).rq_flags |=   (1 << 7) as bindings::req_flags_t;
	}   
   
        let mut req_op = (*req).cmd_flags & bindings::REQ_OP_MASK;
        
	match req_op {           
            bindings::req_opf_REQ_OP_DRV_IN | bindings::req_opf_REQ_OP_DRV_OUT =>
            {
               bindings::memcpy(cmd as *mut c_types::c_void, (*nvme_req(req)).cmd as *mut c_types::c_void, core::mem::size_of::<bindings::nvme_command>() as u64);
            }
            bindings::req_opf_REQ_OP_FLUSH => 
            {
               nvme_setup_flush(ns, cmd);
            }
            bindings::req_opf_REQ_OP_WRITE_ZEROES | bindings::req_opf_REQ_OP_DISCARD =>
            {
                ret = nvme_setup_discard(ns, req, cmd);
            }
            bindings::req_opf_REQ_OP_READ | bindings::req_opf_REQ_OP_WRITE =>
            {
                ret = nvme_setup_rw(ns, req, cmd);
            }
            _ =>
            {
                println!("[LOG] : nvme_setup_cmd")
            }
        }

	((*cmd).__bindgen_anon_1).common.command_id = (*req).tag as u16;
        return ret;
    }	
}

/*
#[no_mangle]
extern "C" fn nvme_init_iod( rq : *mut bindings::request, dev : *mut bindings::nvme_dev) -> bindings::blk_status_t {
    unsafe{
        
        println!("LOG : init_iod entry point");
        let mut iod : *mut bindings::nvme_iod = (rq.offset(1)) as *mut bindings::nvme_iod;
        let mut nseg : i32 = blk_rq_nr_phys_segments(rq) as i32;
        let mut size : u32 = blk_rq_payload_bytes(rq);


        (*iod).use_sgl = nvme_pci_use_sgls(dev, rq);
        
        if (nseg > 2 || size > ( 2 * ((*dev).ctrl).page_size)) {
            let mut alloc_size : c_types::c_size_t = nvme_pci_iod_alloc_size(dev, size, nseg as u32, (*iod).use_sgl) as c_types::c_size_t;
            (*iod).sg = kmalloc_wrapper(alloc_size, bindings::GFP_ATOMIC) as *mut bindings::scatterlist;
            if((*iod).sg == (0 as *mut bindings::scatterlist)){
                return 9;
            }
        } else {
            //let mut tmp : *mut c_types::c_void = (*iod).inline_sg as *mut c_types::c_void;
            //(*iod).sg = tmp as *mut bindings::scatterlist;
            //    (*iod).sg = (*iod).inline_sg as mut bindings::scatterlist;
            println!("LOG : nvme_init_iod else is called!");
  //          (*iod).sg = iod.offset(core::mem::size_of::<bindings::nvme_iod>() as isize) as *mut bindings::scatterlist;

        }
        println!("LOG : init_iod exit point");
        (*iod).aborted = 0;
        (*iod).npages = -1;
        (*iod).nents = 0;
        (*iod).length = size as i32;

        return 0;
    }
}
*/


/*

이제 작동은 잘댐 

root의 build.rs에다가 함수 타입 엄청 짱박고, 한번 make 하셈
그런다음에 ag해서 찾고자하는 symbol 해서 debug/target ../bindings.rs 안에 생기면 그냥 bindings:: xxxx 로 쓰면댐


REQ_OP_WRITE_ZEROES 같은애는 bindings::req_opf_REQ_OP_WRITE_ZEROES 식으로 바껴서

ag를 통해서 찾아야함:





*/

#[macro_export]
macro_rules! kernel_module {
    ($module:ty, $($name:ident : $value:expr),*) => {
        static mut __MOD: Option<$module> = None;
        #[no_mangle]
        pub extern "C" fn init_module() -> $crate::c_types::c_int {
            match <$module as $crate::KernelModule>::init() {
                Ok(m) => {
                    unsafe {
			$crate::nvme_init_fn();
                        __MOD = Some(m);
                    }
                    return 0;
                }
                Err(e) => {
                    return e.to_kernel_errno();
                }
            }
        }

        #[no_mangle]
        pub extern "C" fn cleanup_module() {
            unsafe {
                // Invokes drop() on __MOD, which should be used for cleanup.
  
		$crate::nvme_exit_fn();
              __MOD = None;
            }
        }

        $(
            $crate::kernel_module!(@attribute $name, $value);
        )*
    };

    // TODO: The modinfo attributes below depend on the compiler placing
    // the variables in order in the .modinfo section, so that you end up
    // with b"key=value\0" in order in the section. This is a reasonably
    // standard trick in C, but I'm not sure that rustc guarantees it.
    //
    // Ideally we'd be able to use concat_bytes! + stringify_bytes! +
    // some way of turning a string literal (or at least a string
    // literal token) into a bytes literal, and get a single static
    // [u8; * N] with the whole thing, but those don't really exist yet.
    // Most of the alternatives (e.g. .as_bytes() as a const fn) give
    // you a pointer, not an array, which isn't right.

    (@attribute author, $value:expr) => {
        #[link_section = ".modinfo"]
        #[used]
        pub static AUTHOR_KEY: [u8; 7] = *b"author=";
        #[link_section = ".modinfo"]
        #[used]
        pub static AUTHOR_VALUE: [u8; $value.len()] = *$value;
        #[link_section = ".modinfo"]
        #[used]
        pub static AUTHOR_NUL: [u8; 1] = *b"\0";
    };

    (@attribute description, $value:expr) => {
        #[link_section = ".modinfo"]
        #[used]
        pub static DESCRIPTION_KEY: [u8; 12] = *b"description=";
        #[link_section = ".modinfo"]
        #[used]
        pub static DESCRIPTION_VALUE: [u8; $value.len()] = *$value;
        #[link_section = ".modinfo"]
        #[used]
        pub static DESCRIPTION_NUL: [u8; 1] = *b"\0";
    };

    (@attribute license, $value:expr) => {
        #[link_section = ".modinfo"]
        #[used]
        pub static LICENSE_KEY: [u8; 8] = *b"license=";
        #[link_section = ".modinfo"]
        #[used]
        pub static LICENSE_VALUE: [u8; $value.len()] = *$value;
        #[link_section = ".modinfo"]
        #[used]
        pub static LICENSE_NUL: [u8; 1] = *b"\0";
    };
}

/// KernelModule is the top level entrypoint to implementing a kernel module. Your kernel module
/// should implement the `init` method on it, which maps to the `module_init` macro in Linux C API.
/// You can use this method to do whatever setup or registration your module should do. For any
/// teardown or cleanup operations, your type may implement [`Drop`].
///
/// [`Drop`]: https://doc.rust-lang.org/stable/core/ops/trait.Drop.html
pub trait KernelModule: Sized + Sync {
    fn init() -> KernelResult<Self>;
}


#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    unsafe {
        bug_helper();
    }
}

#[global_allocator]
static ALLOCATOR: allocator::KernelAllocator = allocator::KernelAllocator;

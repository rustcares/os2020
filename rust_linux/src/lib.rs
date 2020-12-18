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
    fn bug_helper() -> !;
    fn nvme_init() -> c_types::c_int;
    fn nvme_exit() -> !;
    fn memcpy( dest : *mut core::ffi::c_void, src : *mut core::ffi::c_void, n : c_types::c_size_t) -> *mut core::ffi::c_void;
    fn nvme_setup_flush( ns : *mut bindings::nvme_ns, cmnd : *mut bindings::nvme_command) -> core::ffi::c_void;
    fn nvme_setup_discard( ns : *mut bindings::nvme_ns, req : *mut bindings::request, cmnd : *mut bindings::nvme_command) -> bindings::blk_status_t;
    fn nvme_setup_rw( ns : *mut bindings::nvme_ns, req : *mut bindings::request, cmnd : *mut bindings::nvme_command) -> bindings::blk_status_t;

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


//Export Rust Functions to C //
#[no_mangle]
extern "C" fn nvme_req_needs_retry( req : *mut bindings::request ) ->  u8 {

   unsafe {
   	
	let nvme_req_ : *mut bindings::nvme_request = (req.offset(1)) as *mut bindings::nvme_request;

	if  ((*req).cmd_flags & ( bindings::REQ_FAILFAST_DEV | bindings::REQ_FAILFAST_TRANSPORT | bindings::REQ_FAILFAST_DRIVER)) != 0 {
		return 0;
	}
	else if  (*nvme_req_).status & bindings::NVME_SC_DNR != 0 {
		return 0;
	}
	else if  (*nvme_req_).retries >= bindings::nvme_max_retries  {
		return 0;
	}
	return 1;

   }
}



#[no_mangle]
extern "C" fn nvme_setup_cmd( ns : *mut bindings::nvme_ns, req : *mut bindings::request, 
				cmd : *mut bindings::nvme_command) -> bindings::blk_status_t {
    unsafe {
	let mut ret = bindings::BLK_STS_OK;
	let mut nvme_req_ : *mut bindings::nvme_request = (req.offset(1)) as *mut bindings::nvme_request;

        let mut RQF_DONTPREP_ : u32 = 1<<7;

	if (((*req).rq_flags & RQF_DONTPREP_) == 0){
	    (*nvme_req_).retries = 0;
	    (*nvme_req_).flags = 0;
	    (*req).rq_flags |= RQF_DONTPREP_;
	}   
    
        let mut op_mask : u32 = 1<<8 - 1;

        let mut req_op = (*req).cmd_flags & op_mask;
        
	match req_op {           
            34 | 35 =>
            {
                memcpy(cmd as *mut core::ffi::c_void , (*nvme_req_).cmd as *mut core::ffi::c_void , core::mem::size_of::<bindings::request>());
            }
            2 => 
            {
                nvme_setup_flush(ns, cmd);
            }
            9 | 3 =>
            {
                ret = nvme_setup_discard(ns, req, cmd);
            }
            0 | 1 =>
            {
                ret = nvme_setup_rw(ns, req, cmd);
            }
            _ =>
            {
                println!("[LOG] : nvme_setup_cmd")
            }
        }

	let cmd__ : *mut bindings::nvme_common_command = cmd as *mut bindings::nvme_common_command;


        (*cmd__).command_id = (*req).tag as u16;
        //*cmd = (iriq).tag;
        return ret;
    }	

}


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

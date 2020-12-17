#![no_std]

extern crate alloc;

use alloc::borrow::ToOwned;
use alloc::string::String;

use linux_kernel_module::println;

struct NVMeModule {
    message: String,
}

impl linux_kernel_module::KernelModule for NVMeModule {

fn init() -> linux_kernel_module::KernelResult<Self> {
        println!("NVMeDriver Loaded!");
        Ok(NVMeModule {
            message: "Nvme Loaded!".to_owned(),
        })
    }


}

impl Drop for NVMeModule {
    fn drop(&mut self) {
        println!("NVMe Driver Module Unloaded!");
    }
}

linux_kernel_module::kernel_module!(
    NVMeModule,
    author: b"JyLee & SGLee",
    description: b"NVMe Linux Kernel Module Driver Written in Rust",
    license: b"GPL"
);

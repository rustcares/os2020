#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/random.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/version.h>


#include <uapi/linux/nvme_ioctl.h>

#include <linux/blk_types.h>
#include <linux/nvme.h>
#include <linux/compiler_types.h>

//NVME Support
#include "c_headers/nvme.h"
#include "c_headers/fabrics.h"



// Bindgen gets confused at certain things
const gfp_t BINDINGS_GFP_KERNEL = GFP_KERNEL;


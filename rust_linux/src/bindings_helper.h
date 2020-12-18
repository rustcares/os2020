#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/random.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/version.h>



#include <linux/blk_types.h>
#include <linux/nvme.h>

//NVME Support
#include "c_headers/nvme.h"
#include "c_headers/fabrics.h"



// Bindgen gets confused at certain things
const gfp_t BINDINGS_GFP_KERNEL = GFP_KERNEL;


const unsigned int BINDINGS_REQ_FAILFAST_DEV = REQ_FAILFAST_DEV;
const unsigned int BINDINGS_REQ_FAILFAST_TRANSPORT = REQ_FAILFAST_TRANSPORT;
const unsigned int BINDINGS_REQ_FAILFAST_DRIVER = REQ_FAILFAST_DRIVER;



const unsigned short BINDINGS_NVME_SC_DNR = NVME_SC_DNR;
extern  u8 nvme_max_retries;


//const unsigned int BINDINGS_REQ_FAILFAST_DRIVER = REQ_FAILFAST_DRIVER;


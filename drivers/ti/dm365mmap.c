/*
 *  Copyright 2009 by Texas Instruments Incorporated.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>
 */

/*
 * dm365mmap.c 
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/moduleparam.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/list.h>
#include <linux/cdev.h>
#include <linux/proc_fs.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/clk.h>

#include <asm/page.h>
#include <asm/uaccess.h>
#include <asm/pgtable.h>
#include <asm/io.h>
#include <asm/system.h>
#include <asm/memory.h>

#include <asm/irq.h>

#include <linux/version.h>
#include <linux/completion.h>
#include <mach/edma.h>

#define USE_UDEV	1

/*
 * The following macros control version-dependent code:
 * USE_CACHE_VOID_ARG - #define if dmac functions take "void *" parameters,
 *    otherwise unsigned long is used
 * USE_CLASS_SIMPLE - #define if Linux version contains "class_simple*",
 *    otherwise "class*" or "device*" is used (see USE_CLASS_DEVICE usage).
 * USE_CLASS_DEVICE - #define if Linux version contains "class_device*",
 *    otherwise "device*" or "class_simple*" is used (see USE_CLASS_SIMPLE
 *    usage).
 * If neither USE_CLASS_SIMPLE nor USE_CLASS_DEVICE is set, there is further
 *    kernel version checking embedded in the module init & exit functions.
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)

#warning *** not a warning *** Note: LINUX_VERSION_CODE >= 2.6.26

#undef USE_CLASS_DEVICE
#undef USE_CLASS_SIMPLE

#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18)

#warning *** not a warning *** Note: 2.6.26 > LINUX_VERSION_CODE >= 2.6.18

#define USE_CLASS_DEVICE
#undef USE_CLASS_SIMPLE

#else  /* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18) */

#warning *** not a warning *** Note: LINUX_VERSION_CODE < 2.6.18

#define USE_CLASS_SIMPLE
#undef USE_CLASS_DEVICE

#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18) */

/*#define ASQINT_ENABLE*/
/*#define MJCPCLK_ENABLE*/

typedef struct _edma_params {
    unsigned long src;
    unsigned long dst;
    unsigned int srcmode;
    unsigned int srcfifowidth;
    int     srcbidx;
    int     srccidx;
    unsigned int dstmode;
    unsigned int dstfifowidth;
    int     dstbidx;
    int     dstcidx;
    int     acnt;
    int     bcnt;
    int     ccnt;
    int     bcntrld;
    int     syncmode;
} edma_params;

#define DM365MMAP_IOCMEMCPY     0x7
#define DM365MMAP_IOCWAIT       0x8
#define DM365MMAP_IOCCLEAR_PENDING 0x9

#ifdef __DEBUG
#    define __D(fmt, args...) printk(KERN_DEBUG "DM365MMAP Debug: " fmt, ## args)
#else /*  */
#    define __D(fmt, args...)
#endif /*  */

#define __E(fmt, args...) printk(KERN_ERR "DM365MMAP Error: " fmt, ## args)

#define MAXTYPE(T) ((T) (((T)1 << ((sizeof(T) * 8) - 1) ^ ((T) -1))))

static int major = 0;

#if (USE_UDEV==1)
static struct class *dm365mmap_class;
#endif

static DECLARE_MUTEX(dm365mmap_reply_mutex);
static struct completion edmacompletion;

/* Forward declaration of system calls */
static int ioctl(struct inode *inode, struct file *filp, unsigned int cmd,
                 unsigned long args);
static int mmap(struct file *filp, struct vm_area_struct *vma);
static int open(struct inode *inode, struct file *filp);
static int release(struct inode *inode, struct file *filp);
static struct file_operations dm365mmap_fxns = { ioctl: ioctl, mmap: mmap, open: open, release:release
};

#ifdef MJCPCLK_ENABLE
/* Structure to hold mjcp clock info */
static struct clk *mjcp = NULL;
#endif


static irqreturn_t irq_handler(int irq, void *dev_id)
{
    /* Release the mutex, suggesting sequencer processing complete */
    up(&dm365mmap_reply_mutex);
    return IRQ_HANDLED;
}

static void memcpy_dma_irq_handler(unsigned lch, u16 ch_status, void *data)
{
    complete_all(&edmacompletion);
}

static int mmap(struct file *filp, struct vm_area_struct *vma)
{
    __D("mmap: vma->vm_start     = %#lx\n", vma->vm_start);
    __D("mmap: vma->vm_pgoff     = %#lx\n", vma->vm_pgoff);
    __D("mmap: vma->vm_end       = %#lx\n", vma->vm_end);
    __D("mmap: size              = %#lx\n", vma->vm_end - vma->vm_start);
    vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
    if (remap_pfn_range(vma, vma->vm_start,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18)
                        (vma->vm_pgoff >> PAGE_SHIFT),
#else /*  */
                        vma->vm_pgoff,
#endif /*  */
                        vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
        __E("mmap: failed remap_pfn_range\n");
        return -EAGAIN;
    }
    return 0;
}


static int ioctl(struct inode *inode, struct file *filp, unsigned int cmd,
                 unsigned long args)
{
    unsigned int __user *argp = (unsigned int __user *) args;
    edma_params edmaparams;
    int     edma_channel;
    int     retval = 0;
    struct edmacc_param p_ram;

    switch (cmd) {

            /*
             * Tries to clear any previously unaccounted interrupt.
             */
        case DM365MMAP_IOCCLEAR_PENDING:
            __D("Clear Pending Call received.\n");
            init_MUTEX_LOCKED(&dm365mmap_reply_mutex);
            break;

            /*
             * Blocks till the sequencer completion interrupt arrives.
             */
        case DM365MMAP_IOCWAIT:
            __D("WAIT Call received.\n");
            retval = down_interruptible(&dm365mmap_reply_mutex);
            break;

        case DM365MMAP_IOCMEMCPY:
            __D("MEMCPY ioctl received.\n");
            if (copy_from_user(&edmaparams, argp, sizeof(edmaparams))) {
                retval = -EFAULT;
                break;
            }

            edma_channel =
                edma_alloc_channel(EDMA_CHANNEL_ANY, &memcpy_dma_irq_handler,
                                   NULL, EVENTQ_DEFAULT);

            /* Handle Failure condition here */
            if (edma_channel < 0) {
                __E("Error in requesting Master channel = 0x%x\n",
                    edma_channel);
                retval = edma_channel;
                break;
            }

            edma_stop(edma_channel);

            init_completion(&edmacompletion);

            edma_set_src(edma_channel,
                         (dma_addr_t) edmaparams.src,
                         edmaparams.srcmode, edmaparams.srcfifowidth);

            edma_set_dest(edma_channel,
                          (dma_addr_t) edmaparams.dst,
                          edmaparams.dstmode, edmaparams.dstfifowidth);

            edma_set_src_index(edma_channel, edmaparams.srcbidx,
                               edmaparams.srccidx);
            edma_set_dest_index(edma_channel, edmaparams.dstbidx,
                                edmaparams.dstcidx);

            edma_set_transfer_params(edma_channel,
                                     edmaparams.acnt,
                                     edmaparams.bcnt,
                                     edmaparams.ccnt,
                                     edmaparams.bcntrld, edmaparams.syncmode);

            edma_read_slot(edma_channel, &p_ram);
            p_ram.opt |= TCINTEN | EDMA_TCC(edma_channel);
            edma_write_slot(edma_channel, &p_ram);

            edma_start(edma_channel);
            wait_for_completion(&edmacompletion);
            edma_stop(edma_channel);
            edma_free_channel(edma_channel);
            break;

        default:
            __E("Unknown ioctl received = %d.\n", cmd);
            retval = -EINVAL;
            break;
    }

    return retval;
}


static int open(struct inode *inode, struct file *filp)
{
    __D("open: called.\n");
    return 0;
}


static int release(struct inode *inode, struct file *filp)
{
    __D("close: called.");
    return 0;
}


int __init dm365mmap_init(void)
{

#ifdef MJCPCLK_ENABLE
    /* In the GIT kernel unused clocks are disabled. To run codec we need to 
     * enable mjcp clock.
     */
    mjcp = clk_get(NULL, "mjcp");
    if (IS_ERR(mjcp))
        printk(KERN_WARNING "unable to get MJCP clock\n");
    else
        clk_enable(mjcp);
#endif
    __D("** DM365MMAP kernel module built: " __DATE__ " " __TIME__ "\n");
    major = register_chrdev(0, "dm365mmap", &dm365mmap_fxns);
    if (major < 0) {
        __E("Failed to allocate major number.\n");
        return -ENODEV;
    }
    __D("Allocated major number: %d\n", major);
/*
#if (USE_UDEV==1)
    dm365mmap_class = class_create(THIS_MODULE, "dm365mmap");
    if (IS_ERR(dm365mmap_class)) {
        __E("Error creating dm365mmap device class.\n");
        return -EIO;
    }
    class_device_create(dm365mmap_class, NULL, MKDEV(major, 0), NULL,
                        "dm365mmap");

#endif
*/

#if (USE_UDEV==1)

#ifdef USE_CLASS_SIMPLE
    dm365mmap_class = class_simple_create(THIS_MODULE, "dm365mmap");
#else
    dm365mmap_class = class_create(THIS_MODULE, "dm365mmap");
#endif // USE_CLASS_SIMPLE

    if (IS_ERR(dm365mmap_class)) {
        __E("Error creating dm365mmap device class.\n");
        return -EIO;
    }

#ifdef USE_CLASS_SIMPLE
    class_simple_device_add(dm365mmap_class, MKDEV(major, 0), NULL, "dm365mmap");
#else
#ifdef USE_CLASS_DEVICE
    class_device_create(dm365mmap_class, NULL, MKDEV(major, 0), NULL,
                        "dm365mmap");
#else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
    device_create(dm365mmap_class, NULL, MKDEV(major, 0), NULL, "dm365mmap");
#else
    device_create(dm365mmap_class, NULL, MKDEV(major, 0), "dm365mmap");
#endif // LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
#endif // USE_CLASS_DEVICE
#endif // USE_CLASS_SIMPLE

#endif // USE_UDEV==1


    __D("Successfully initialized module\n");

#ifdef ASQINT_ENABLE
    if (request_irq(IRQ_ASQINT, irq_handler, 0, "seq_arm_interrupt", NULL)) {
        __D("Could not install ISR\n");
    }

#endif /*  */
    init_MUTEX_LOCKED(&dm365mmap_reply_mutex);
    return 0;
}


void __exit dm365mmap_exit(void)
{
#ifdef MJCPCLK_ENABLE
    /* if mjcp clock is enabled then free it */
    if (mjcp) {
        clk_disable(mjcp);
        clk_put(mjcp);
    }
#endif

    __D("In dm365mmap_exit()\n");
/*
#if (USE_UDEV==1)
    class_device_destroy(dm365mmap_class, MKDEV(major, 0));
    class_destroy(dm365mmap_class);

#endif
*/

#if (USE_UDEV==1)
#ifdef USE_CLASS_SIMPLE
    class_simple_device_remove(MKDEV(major, 0));
    class_simple_destroy(dm365mmap_class);
#else
#ifdef USE_CLASS_DEVICE
    class_device_destroy(dm365mmap_class, MKDEV(major, 0));
#else
    device_destroy(dm365mmap_class, MKDEV(major, 0));
#endif
    class_destroy(dm365mmap_class);
#endif
#endif // USE_UDEV


    __D("Unregistering character device dm365mmap\n");
    unregister_chrdev(major, "dm365mmap");

#ifdef ASQINT_ENABLE
    free_irq(IRQ_ASQINT, NULL);

#endif /*  */
    __D("dm365mmap unregistered\n");
}

module_init(dm365mmap_init);

module_exit(dm365mmap_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("DM365 mmap export to userland");


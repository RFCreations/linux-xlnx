#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>

#define DRIVER_NAME "userdma"

MODULE_AUTHOR("Patrick Stewart <patrick@rfcreations.com>");
MODULE_DESCRIPTION("Allocate DMA buffers for userspace");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");

static dev_t dev;
static struct class *dev_class = NULL;
struct cdev dev_cdev;
struct device *dev_char = NULL;

static const unsigned long attrs = DMA_ATTR_NO_KERNEL_MAPPING;

struct mapping {
  	dma_addr_t dma_addr;
    void* cpu_addr;
    size_t len;
};

static void map_close(struct vm_area_struct* vma) {
    struct mapping* m = vma->vm_private_data;
    printk(KERN_INFO "Free DMA mapping %lx -> %lx\n", vma->vm_start, (unsigned long)m->dma_addr);
    dma_free_attrs(dev_char, m->len, m->cpu_addr, m->dma_addr, attrs);
    kfree(m);
}

static struct vm_operations_struct vm_ops = {
    .close = map_close
};

static int map(struct file* file, struct vm_area_struct* vma) {
    int ret;
    struct mapping* m = kmalloc(sizeof(struct mapping), GFP_KERNEL | GFP_NOWAIT);
    if (!m) {
        printk(KERN_CRIT "Failed to allocate mapping\n");
        return -ENOMEM;
    }
    m->len = vma->vm_end - vma->vm_start;

    m->cpu_addr = dma_alloc_attrs(dev_char, m->len, &m->dma_addr, GFP_KERNEL, attrs);
    if (!m->cpu_addr) {
        printk(KERN_CRIT "Failed to allocate DMA\n");
        ret = -ENOMEM;
        goto cleanup1;
    }
    ret = dma_mmap_attrs(dev_char, vma, m->cpu_addr, m->dma_addr, m->len, attrs);
    if (ret) {
        printk(KERN_CRIT "Failed to allocate DMA\n");
        goto cleanup2;
    }

    vma->vm_private_data = m;
    vma->vm_ops = &vm_ops;
    file->private_data = m;

    printk(KERN_INFO "Userspace DMA mapping %lx -> %lx\n", vma->vm_start, (unsigned long)m->dma_addr);

    return 0;

cleanup2:
    dma_free_attrs(dev_char, m->len, m->cpu_addr, m->dma_addr, attrs);
cleanup1:
    kfree(m);
    return ret;
}

static long map_ioctl(struct file* file, unsigned int cmd, unsigned long arg) {
    struct mapping* m = file->private_data;
    if (m && cmd == _IOR(0xF0, 0, u64*)) {
        return put_user(m->dma_addr, (u64*)arg);
    }
    return -ENXIO;
}

static const struct file_operations dev_fops = {
    .owner = THIS_MODULE,
    .mmap = map,
    .unlocked_ioctl = map_ioctl
};

static int __init userdma_init(void)
{
    int ret = 0;

    // Set up character device creation
    ret = alloc_chrdev_region(&dev, 0, 1, DRIVER_NAME);
    if (ret < 0)
    {
        printk(KERN_CRIT "Failed to allocate character device region\n");
        return ret;
    }

    // Create character device class
    dev_class = class_create(THIS_MODULE, DRIVER_NAME);
    if (IS_ERR(dev_class))
    {
        printk(KERN_CRIT "Failed to create character device class\n");
        ret = PTR_ERR(dev_class);
        goto cleanup1;
    }

    cdev_init(&dev_cdev, &dev_fops);
    ret = cdev_add(&dev_cdev, dev, 1);
    if (ret != 0)
    {
        printk(KERN_ERR "Failed to add cdev\n");
        goto cleanup2;
    }

    dev_char = device_create(dev_class, NULL, dev, NULL, "dma");
    if (IS_ERR(dev_char))
    {
        printk(KERN_ERR "Failed to create character device\n");
        ret = PTR_ERR(dev_char);
        goto cleanup3;
    }

    ret = dma_set_coherent_mask(dev_char, DMA_BIT_MASK(32));
    if (ret) {
		printk(KERN_ERR "Failed to set DMA mask\n");
		goto cleanup4;
	}

    printk(KERN_INFO "Character device /dev/dma created (%d.%d)\n", MAJOR(dev), MINOR(dev));
    printk(KERN_DEBUG DRIVER_NAME " loaded\n");

    return 0;

cleanup4:
    device_destroy(dev_class, dev);
cleanup3:
    cdev_del(&dev_cdev);
cleanup2:
    class_destroy(dev_class);
cleanup1:
    unregister_chrdev_region(dev, 1);

    return ret;
}
module_init(userdma_init);

static void __exit userdma_exit(void)
{
    device_destroy(dev_class, dev);
    cdev_del(&dev_cdev);
    class_destroy(dev_class);
    unregister_chrdev_region(dev, 1);

    printk(KERN_DEBUG DRIVER_NAME " unloaded\n");
}
module_exit(userdma_exit);

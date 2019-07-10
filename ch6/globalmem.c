/*
 * a simple char device driver: globalmem without mutex
 *
 * Copyright (C) 2014 Barry Song  (baohua@kernel.org)
 *
 * Licensed under GPLv2 or later.
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include<linux/io.h>

#define GLOBALMEM_SIZE	0x2DD000		//0x31C6A0
#define GLOBALMEM_MAGIC 'globe'			//设备识别码
#define MEM_CLEAR _IO(GLOBALMEM_MAGIC,0)	//生成特殊的设备码
#define GLOBALMEM_MAJOR 230			//主要设备号
#define DEVICE_NUM 4	//最多支持设备数
unsigned int GLOBA_ADDR0[4]={0x400000C4,0x400000D4,0x40000400,0x40000408};		//存放物理地址的地址
#define GLOBA_SIZE0 0x400000C8		//存放长度
// #define barrier() _asm_ _volatile_("":::"memory")
static void *p_gpj0_base;

static int globalmem_major = GLOBALMEM_MAJOR;
module_param(globalmem_major, int, S_IRUGO);

struct globalmem_dev {
	struct cdev cdev;
	unsigned char mem[GLOBALMEM_SIZE];
	// struct mutex mutex;
};

struct globalmem_dev *globalmem_devp;

static int globalmem_open(struct inode *inode, struct file *filp)
{
	struct globalmem_dev *dev = container_of(inode->i_cdev, struct globalmem_dev, cdev);
	filp->private_data = dev;
	return 0;
}

static int globalmem_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static long globalmem_ioctl(struct file *filp, unsigned int cmd,
			    unsigned long arg)
{
	struct globalmem_dev *dev = filp->private_data;

	switch (cmd) {
	case MEM_CLEAR:
		// mutex_lock(&dev->mutex);
		memset(dev->mem, 0, GLOBALMEM_SIZE);
		// mutex_unlock(&dev->mutex);
		printk(KERN_INFO "globalmem is set to zero\n");
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static ssize_t globalmem_read(struct file *filp, char __user * buf, size_t size,
			      loff_t * ppos)
{
	unsigned long p = *ppos;
	unsigned int count = size;
	int ret = 0;
	struct globalmem_dev *dev = filp->private_data;

	if (p >= GLOBALMEM_SIZE)
		return 0;
	if (count > GLOBALMEM_SIZE - p)
		count = GLOBALMEM_SIZE - p;
	// mutex_lock(&dev->mutex);
	if (copy_to_user(buf, dev->mem + p, count)) {
		ret = -EFAULT;
	} else {
		*ppos += count;
		ret = count;
	}
	// mutex_unlock(&dev->mutex);
	return ret;
}


static ssize_t globalmem_write(struct file *filp, const char __user * buf,
			       size_t size, loff_t * ppos)
{
	volatile unsigned long p = *ppos;
	unsigned int count = size;
	u32 phys;
	int ret = 0;
	struct globalmem_dev *dev = filp->private_data;
	if (p >= GLOBALMEM_SIZE)
		return 0;
	if (count > GLOBALMEM_SIZE - p)
		count = GLOBALMEM_SIZE - p;
	// mutex_lock(&dev->mutex);
	if (copy_from_user(dev->mem + p, buf, count))
		ret = -EFAULT;
	else {
		*ppos += count;
		ret = count;
	}

	if (!request_mem_region(GLOBA_SIZE0, 8, "addr")){
        return -EINVAL;
		}
	p_gpj0_base = ioremap(GLOBA_SIZE0, 8);
	writel(*ppos, p_gpj0_base + 0);
	iounmap(p_gpj0_base);
	release_mem_region(GLOBA_SIZE0, 8);
	// mutex_unlock(&dev->mutex);
	printk(KERN_DEBUG "p=%p ppos=%d\n",p,*ppos);

	return ret;
}

static loff_t globalmem_llseek(struct file *filp, loff_t offset, int orig)
{
	loff_t ret = 0;
	switch (orig) {
	case 0:
		if (offset < 0) {
			ret = -EINVAL;
			break;
		}
		if ((unsigned int)offset > GLOBALMEM_SIZE) {
			ret = -EINVAL;
			break;
		}
		filp->f_pos = (unsigned int)offset;
		ret = filp->f_pos;
		break;
	case 1:
		if ((filp->f_pos + offset) > GLOBALMEM_SIZE) {
			ret = -EINVAL;
			break;
		}
		if ((filp->f_pos + offset) < 0) {
			ret = -EINVAL;
			break;	// mutex_init(&globalmem_dev->mutex);
		}
		filp->f_pos += offset;
		ret = filp->f_pos;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static const struct file_operations globalmem_fops = {
	.owner = THIS_MODULE,
	.llseek = globalmem_llseek,
	.read = globalmem_read,
	.write = globalmem_write,
	.unlocked_ioctl = globalmem_ioctl,
	.open = globalmem_open,
	.release = globalmem_release,
};

static void globalmem_setup_cdev(struct globalmem_dev *dev, int index)
{
	u32 phys;

	int err, devno = MKDEV(globalmem_major, index);
	cdev_init(&dev->cdev, &globalmem_fops);
	dev->cdev.owner = THIS_MODULE;
	err = cdev_add(&dev->cdev, devno, 1);
	if (err)
		printk(KERN_NOTICE "Error %d adding globalmem%d", err, index);
	phys=virt_to_phys(dev->mem);		//虚拟地址转化为物理地址
	printk("phys addr 0x%X\n",phys);
	if (!request_mem_region(GLOBA_ADDR0[index], 8, "addr")){
       	return -EINVAL;									//申请寄存器控制
	}
	p_gpj0_base = ioremap(GLOBA_ADDR0[index], 8);           //获得寄存器虚拟地址
	writel(phys, p_gpj0_base + 0);					//写寄存器
	iounmap(p_gpj0_base);							//关闭mmap
	release_mem_region(GLOBA_ADDR0[index], 8);
}

static int __init globalmem_init(void)
{
	int ret;
	int i;
	dev_t devno = MKDEV(globalmem_major, 0);

	if (globalmem_major)
		ret = register_chrdev_region(devno, 1, "globalmem");
	else {
		ret = alloc_chrdev_region(&devno, 0, 1, "globalmem");
		globalmem_major = MAJOR(devno);
	}
	if (ret < 0)
		return ret;

	globalmem_devp = kzalloc(sizeof(struct globalmem_dev), GFP_KERNEL);
	if (!globalmem_devp) {
		ret = -ENOMEM;
		goto fail_malloc;
	}
	// mutex_init(&globalmem_dev->mutex);
	for(i=0;i<DEVICE_NUM;i++)
		globalmem_setup_cdev(globalmem_devp+i, i);
	return 0;

 fail_malloc:
	unregister_chrdev_region(devno, DEVICE_NUM);
	return ret;
}
module_init(globalmem_init);

static void __exit globalmem_exit(void)
{
	int i;
	for(i=0;i<DEVICE_NUM;i++)
	cdev_del(&(globalmem_devp+i)->cdev);
	kfree(globalmem_devp);
	unregister_chrdev_region(MKDEV(globalmem_major, 0), 1);
}
module_exit(globalmem_exit);

MODULE_AUTHOR("lxl");
MODULE_LICENSE("Proprietary");
MODULE_VERSION("1.0");
MODULE_ALIAS("RAU-r");
// MODULE_DEVICE_TABLE("vavitel-RAU");

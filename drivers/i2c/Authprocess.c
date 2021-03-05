/*
 * This file is part of the C intelligent linux distribution
 * Copyright (c) 2019 Mitsubishi Electric India Pvt. Ltd., All Rights Reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/ioctl.h>
#include <linux/version.h>
#include <linux/kthread.h>
#include <linux/spinlock_types.h>
#include <asm/io.h>
#include <linux/ioport.h>
#include <linux/delay.h>

#define MAX_SIZE_I2C_RW_DATA	10

#define I2C_WR 					_IOWR('a','a',I2C_IORW_PKG*)
#define I2C_RD 					_IOWR('a','b',I2C_IORW_PKG*)

#define I2C_BASE_ADDR			0xFFC04000
#define FREERUNCNT_US_ADDR		(unsigned long)0xFF2000B0
#define I2C_SIZE				1024 * 4

typedef struct tagI2C_IORW_PKG
{
	unsigned short	usI2cAdr;
	unsigned short	usI2cSize;
	unsigned long 	ulI2cTimeout;
	unsigned char	ucI2cData[MAX_SIZE_I2C_RW_DATA];
	unsigned short	usRet;
} I2C_IORW_PKG;

I2C_IORW_PKG stI2cRwPkg;

void *ioptr_busAsic ;
void *ioptr_i2c;

dev_t dev = 0;
static struct class *dev_class;
static struct cdev etx_cdev;
 
static int __init etx_driver_init(void);
static void __exit etx_driver_exit(void);
static int etx_open(struct inode *inode, struct file *file);
static int etx_release(struct inode *inode, struct file *file);
static ssize_t etx_read(struct file *filp, char __user *buf, size_t len,loff_t * off);
static ssize_t etx_write(struct file *filp, const char *buf, size_t len, loff_t * off);
static long etx_ioctl(struct file *file, unsigned int cmd, unsigned long arg);

static struct file_operations fops =
{
        .owner          = THIS_MODULE,
        .read           = etx_read,
        .write          = etx_write,
        .open           = etx_open,
        .unlocked_ioctl = etx_ioctl,
        .release        = etx_release,
};

static int etx_open(struct inode *inode, struct file *file)
{
	printk(KERN_INFO "I2C Device File Opened...!!!\n");
	return 0;
}

static int etx_release(struct inode *inode, struct file *file)
{
	printk(KERN_INFO "I2C Device File Closed...!!!\n");
	return 0;
}

static ssize_t etx_read(struct file *filp, char __user *buf, size_t len, loff_t *off)
{
	printk(KERN_INFO "I2C Read Function\n");
	return 0;
}

static ssize_t etx_write(struct file *filp, const char __user *buf, size_t len, loff_t *off)
{
	printk(KERN_INFO "I2C Write function\n");
	return 0;
}

unsigned int uiInit (void)
{
	struct resource *res;

	res= request_mem_region (I2C_BASE_ADDR, I2C_SIZE, "i2c1");
	if (res == NULL)
	{
		printk(KERN_INFO "uiInit error\n");
		return -1;
    }
	ioptr_i2c = ioremap_nocache (I2C_BASE_ADDR, I2C_SIZE);

	res= request_mem_region (FREERUNCNT_US_ADDR, 32, "Bus ASIC");
	if (res == NULL)
    {
		printk(KERN_INFO "uiInit error\n");
		return -1;
    }
	ioptr_busAsic = ioremap_nocache (FREERUNCNT_US_ADDR, 32);

	return (0);
}

unsigned int uiMap(unsigned int uiaddress)
{

	if (	(uiaddress >= FREERUNCNT_US_ADDR ) &&
			(uiaddress < FREERUNCNT_US_ADDR + 32)
		)
	{
		return ((unsigned int)(ioptr_busAsic + uiaddress - FREERUNCNT_US_ADDR)) ;
	}
	else if(	(uiaddress >= I2C_BASE_ADDR) &&
				(uiaddress < (I2C_BASE_ADDR + I2C_SIZE))
			)
	{
		return ((unsigned int)(ioptr_i2c + uiaddress - I2C_BASE_ADDR));
	}
	else
	{
		printk(KERN_INFO "uiMap error\n");
		return ((unsigned int)-1);
	}
}

short sI2CRead(	unsigned short usI2cReadAdr,
				unsigned short usI2cReadSize,
				unsigned long ulWaitTime,
				unsigned long* pulReadDataBuf)
{
	int ix;
	unsigned char* pucReadDataBuf;
	pucReadDataBuf= (unsigned char*)pulReadDataBuf;

    (*((unsigned short*)uiMap(0x6C + 0xffc04000)))= 0x0;

    (*((unsigned short*)uiMap(0x04 + 0xffc04000)))= (usI2cReadAdr >> 9) & 0x007F;

    (*((unsigned short*)uiMap(0x6C + 0xffc04000)))= 0x1;

    for(ix= 0; ix < usI2cReadSize - 1; ix++)
    {
        (*((unsigned short*)uiMap(0x10 + 0xffc04000)))= 0x0100;

        while(!((*((unsigned short*)uiMap(0x34 + 0xffc04000))) & 0x4))
        {

        }

        pucReadDataBuf[ix]= (*((unsigned char*)uiMap(0x10 + 0xffc04000)));
    }

    (*((unsigned short*)uiMap(0x10 + 0xffc04000)))= 0x0300;

    while(!((*((unsigned short*)uiMap(0x34 + 0xffc04000))) & 0x4))
    {

    }

    pucReadDataBuf[ix]= (*((unsigned char*)uiMap(0x10 + 0xffc04000)));

    return 0;
}

short sI2CWrite(unsigned short usI2cWriteAdr,
				unsigned short usTxSize,
				unsigned long ulWaitTime,
				unsigned long* pulWriteDataBuf)
{
	int ix;
	unsigned char* pucWriteDataBuf;
	pucWriteDataBuf = (unsigned char*) pulWriteDataBuf;

    (*((unsigned short*)uiMap(0x6C + 0xffc04000)))= 0x0;

    (*((unsigned short*)uiMap(0x04 + 0xffc04000)))= (usI2cWriteAdr >> 9) & 0x007F;

    (*((unsigned short*)uiMap(0x6C + 0xffc04000)))= 0x1;

    for(ix= 0; ix < usTxSize - 1; ix++)
    {
    	 (*((unsigned short*)uiMap(0x10 + 0xffc04000)))= (pucWriteDataBuf[ix] & 0xff);

    	 while(!((*((unsigned short*)uiMap(0x34 + 0xffc04000))) & 0x10))
    	 {

         }
     }

    (*((unsigned short*)uiMap(0x10 + 0xffc04000)))= ((pucWriteDataBuf[ix] & 0xff) | 0x200);

    while(!((*((unsigned short*)uiMap(0x34 + 0xffc04000))) & 0x10))
    {

    }

	return 0;
}

static long etx_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	unsigned long flags;
	DEFINE_SPINLOCK(mLock);

	copy_from_user(&stI2cRwPkg, (int32_t*)arg, sizeof(stI2cRwPkg));

	spin_lock_irqsave(&mLock, flags);

	switch(cmd)
	{
		case I2C_WR:
		{
			stI2cRwPkg.usRet = sI2CWrite((unsigned short)stI2cRwPkg.usI2cAdr, (unsigned short)stI2cRwPkg.usI2cSize, (unsigned long)stI2cRwPkg.ulI2cTimeout , (unsigned long*)(&stI2cRwPkg.ucI2cData[0]));
			break;
		}
        case I2C_RD:
        {
            stI2cRwPkg.usRet = sI2CRead((unsigned short)stI2cRwPkg.usI2cAdr, (unsigned short)stI2cRwPkg.usI2cSize, (unsigned long)stI2cRwPkg.ulI2cTimeout, (unsigned long*)(&stI2cRwPkg.ucI2cData[0]));
            break;
        }
    }

	spin_unlock_irqrestore(&mLock, flags);

	copy_to_user((int32_t*)arg, &stI2cRwPkg, sizeof(stI2cRwPkg));

	return 0;
}

static int __init etx_driver_init(void)
{
	short sRetValue = 0;

	/*Allocating Major number*/
	if((alloc_chrdev_region(&dev, 0, 1, "etx_Dev")) <0)
	{
		printk(KERN_INFO "I2C Device Driver :: Cannot allocate major number\n");
		return -1;
	}

	printk(KERN_INFO "I2C Device Driver :: Major = %d  Minor = %d \n",MAJOR(dev), MINOR(dev));
 
	/*Creating cdev structure*/
	cdev_init(&etx_cdev,&fops);
	/*Adding device to the system*/
	if((cdev_add(&etx_cdev,dev,1)) < 0)
	{
		printk(KERN_INFO "I2C Device Driver :: Cannot add the device to the system\n");
		goto r_class;
	}
 
	/*Creating struct class*/
	if((dev_class = class_create(THIS_MODULE,"etx_class")) == NULL)
	{
		printk(KERN_INFO "I2C Device Driver :: Cannot create the struct class\n");
		goto r_class;
	}
 
	/*Creating device*/
	if((device_create(dev_class,NULL,dev,NULL,"etx_device")) == NULL)
	{
		printk(KERN_INFO "I2C Device Driver :: Cannot create the Device 1\n");
		goto r_device;
	}

	sRetValue = uiInit();
	if(sRetValue)
	{
		printk(KERN_INFO "uiInit error\n");
		return -1;
	}


	printk(KERN_INFO "I2C Device Driver Insert...Done!!!\n");

	return 0;
 
r_device:
        class_destroy(dev_class);
r_class:
        unregister_chrdev_region(dev,1);
        return -1;
}

void __exit etx_driver_exit(void)
{

	iounmap(ioptr_busAsic);
	release_mem_region(FREERUNCNT_US_ADDR, 32);

	iounmap(ioptr_i2c);
	release_mem_region(I2C_BASE_ADDR, I2C_SIZE);
	device_destroy(dev_class,dev);
	class_destroy(dev_class);
	cdev_del(&etx_cdev);
	unregister_chrdev_region(dev, 1);
    printk(KERN_INFO "I2C Device Driver Remove...Done!!!\n");
}

module_init(etx_driver_init);
module_exit(etx_driver_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("MEI:Chaitanya Bapat");
MODULE_DESCRIPTION("I2C Device Driver");

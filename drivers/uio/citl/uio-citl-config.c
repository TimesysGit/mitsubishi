/*
 * This file is part of the C intelligent linux distribution
 * Copyright (c) 2015 Mitsubishi Electric India Pvt. Ltd., All Rights Reserved.
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

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/uio_driver.h>
#include <asm/io.h>
#include <linux/ioport.h>
#include <linux/irq.h>

#define I2C_BASE_ADDR		(0xFFC04000)
#define	BUS_AD				(0xF0000000)
#define	BUS_SZ				(0x08000000)
#define	FPGA_AD				(0xFF200000)
#define FLASH_BASE_ADDR		(0xC0000000)
#define FLASH_SIZE			(0x04000000)

static struct uio_info 	*info;
static struct uio_info 	*info_fpga_register;
static struct uio_info 	*info_flash_area;
static struct uio_info 	*info_hpsI2C_area;
static struct device 	*dev;

static void
uio_citl_release (struct device *dev)
{
  pr_info ("releasing citl uio device\n");
}

static int __init
uio_citl_init (void)
{
  int cnt = 0;

  dev = kzalloc (sizeof(struct device), GFP_KERNEL);
  dev_set_name (dev, "FPGA Devices");
  dev->release = uio_citl_release;
  device_register (dev);

  info = kzalloc (sizeof(struct uio_info), GFP_KERNEL);
  info->name = "Bus ASIC";
  info->version = "0.0.1";
  info->mem[0].addr = BUS_AD;
  info->mem[0].memtype = UIO_MEM_PHYS;
  info->mem[0].size = BUS_SZ;

  if (uio_register_device (dev, info) < 0)
    {
      device_unregister (dev);
      kfree (dev);
      kfree (info);
      pr_info ("Failing to register UIO device %s \n", info->name);
      return -1;
    }
  pr_info ("Registered UIO device %s \n", info->name);

  info_fpga_register = kzalloc (sizeof(struct uio_info), GFP_KERNEL);
  info_fpga_register->name = "FPGA registers";
  info_fpga_register->version = "0.0.1";
  info_fpga_register->mem[0].addr = FPGA_AD;
  info_fpga_register->mem[0].memtype = UIO_MEM_PHYS;
  info_fpga_register->mem[0].size = 1024 * 1024 * 2;

  if (uio_register_device (dev, info_fpga_register) < 0)
    {
      device_unregister (dev);
      kfree (dev);
      kfree (info_fpga_register);
      pr_info ("Failing to register UIO device %s \n",
	       info_fpga_register->name);
      return -1;
    }
  pr_info ("Registered UIO device %s \n", info_fpga_register->name);

  info_flash_area = kzalloc (sizeof(struct uio_info), GFP_KERNEL);
  info_flash_area->name = "Flash Area";
  info_flash_area->version = "0.0.1";
  info_flash_area->mem[0].addr = FLASH_BASE_ADDR;
  info_flash_area->mem[0].memtype = UIO_MEM_PHYS;
  info_flash_area->mem[0].size = FLASH_SIZE;

  if (uio_register_device (dev, info_flash_area) < 0)
    {
      device_unregister (dev);
      kfree (dev);
      kfree (info_flash_area);
      pr_info ("Failing to register UIO device %s \n", info_flash_area->name);
      return -1;
    }
  pr_info ("Registered UIO device %s \n", info_flash_area->name);

  info_hpsI2C_area = kzalloc (sizeof(struct uio_info), GFP_KERNEL);
  info_hpsI2C_area->name = "HPS I2C";
  info_hpsI2C_area->version = "0.0.1";
  info_hpsI2C_area->mem[0].addr = I2C_BASE_ADDR;
  info_hpsI2C_area->mem[0].memtype = UIO_MEM_PHYS;
  info_hpsI2C_area->mem[0].size = 1024 * 1024 * 2;

  if (uio_register_device (dev, info_hpsI2C_area) < 0)
    {
      device_unregister (dev);
      kfree (dev);
      kfree (info_hpsI2C_area);
      pr_info ("Failing to register UIO device %s \n", info_flash_area->name);
      return -1;
    }
  pr_info ("Registered UIO device %s \n", info_hpsI2C_area->name);

  return 0;
}

static void __exit
uio_citl_exit (void)
{
  int cnt = 0;

  uio_unregister_device (info);
  uio_unregister_device (info_fpga_register);
  uio_unregister_device (info_flash_area);
  uio_unregister_device (info_hpsI2C_area);
  device_unregister (dev);

  kfree (info);
  kfree (info_fpga_register);
  kfree (info_flash_area);
  kfree (info_hpsI2C_area);

  kfree (dev);
}

module_init ( uio_citl_init);
module_exit ( uio_citl_exit);

MODULE_AUTHOR("Kunal Patwardhan");
MODULE_DESCRIPTION("UIO driver to map addresses to user space");
MODULE_LICENSE("GPL v2");

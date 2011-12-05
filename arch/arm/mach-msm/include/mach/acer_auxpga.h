/*
 * Acer FM volume adjusted by AUXPGA driver
 *
 *
 * Copyright (C) 2010 acer Corporation.
 *
 * Authors:
 *    Andyl Liu <Andyl_Liu@acer.com.tw>
 */

#ifndef __LINUX_ACER_AUXPGA_H
#define __LINUX_ACER_AUXPGA_H

// IOCTL
#define AUXPGA_IOCTL_MAGIC 'i'
#define IOC_MAXNR	2

#define AUXPGA_FM_VOLUME_CONTROL   _IO(AUXPGA_IOCTL_MAGIC, 1)
#define AUXPGA_GET_FM_VOLUME       _IO(AUXPGA_IOCTL_MAGIC, 2)

extern void auxpga_set_control(u32 index);
#endif

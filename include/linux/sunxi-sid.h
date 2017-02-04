/*
 * linux/sunxi-sid.h
 *
 * Copyright(c) 2014-2016 Allwinnertech Co., Ltd.
 *         http://www.allwinnertech.com
 *
 * Author: sunny <sunny@allwinnertech.com>
 *
 * allwinner sunxi soc chip version and chip id manager.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __SUNXI_MACH_SUNXI_CHIP_H
#define __SUNXI_MACH_SUNXI_CHIP_H

#define SUNXI_CHIP_REV(p, v)  (p + v)

#define SUNXI_CHIP_SUN8IW11   (0x17010000)
#define SUN8IW11P1_REV_A SUNXI_CHIP_REV(SUNXI_CHIP_SUN8IW11, 0x0000)
#define SUN8IW11P2_REV_A SUNXI_CHIP_REV(SUNXI_CHIP_SUN8IW11, 0x0001)
#define SUN8IW11P3_REV_A SUNXI_CHIP_REV(SUNXI_CHIP_SUN8IW11, 0x0011)
#define SUN8IW11P4_REV_A SUNXI_CHIP_REV(SUNXI_CHIP_SUN8IW11, 0x0101)

#define SUNXI_CHIP_SUN8IW10   (0x16990000)
#define SUN8IW10P1_REV_B SUNXI_CHIP_REV(SUNXI_CHIP_SUN8IW10, 0x1001)
#define SUN8IW10P2_REV_B SUNXI_CHIP_REV(SUNXI_CHIP_SUN8IW10, 0x1000)
#define SUN8IW10P3_REV_B SUNXI_CHIP_REV(SUNXI_CHIP_SUN8IW10, 0x1003)
#define SUN8IW10P4_REV_B SUNXI_CHIP_REV(SUNXI_CHIP_SUN8IW10, 0x1002)
#define SUN8IW10P5_REV_B SUNXI_CHIP_REV(SUNXI_CHIP_SUN8IW10, 0x100B)
#define SUN8IW10P6_REV_B SUNXI_CHIP_REV(SUNXI_CHIP_SUN8IW10, 0x100A)
#define SUN8IW10P7_REV_B SUNXI_CHIP_REV(SUNXI_CHIP_SUN8IW10, 0x1007)
#define SUN8IW10P8_REV_B SUNXI_CHIP_REV(SUNXI_CHIP_SUN8IW10, 0x1006)

#define SUNXI_CHIP_SUN50IW1   (0x16890000)
#define SUN50IW1P1_REV_A	SUNXI_CHIP_REV(SUNXI_CHIP_SUN50IW1, 0x0)

#define SUNXI_CHIP_SUN50IW2   (0x17180000)
#define SUN50IW2P1_REV_A	SUNXI_CHIP_REV(SUNXI_CHIP_SUN50IW2, 0x0)

unsigned int sunxi_get_soc_ver(void);
int sunxi_get_soc_chipid(u8 *chipid);
int sunxi_get_soc_chipid_str(char *chipid);
int sunxi_get_pmu_chipid(u8 *chipid);
int sunxi_get_serial(u8 *serial);
unsigned int sunxi_get_soc_bin(void);
int sunxi_soc_is_secure(void);
s32 sunxi_get_platform(s8 *buf, s32 size);

#endif  /* __SUNXI_MACH_SUNXI_CHIP_H */

/*
 * File: arch/arm/plat-omap/include/mach/vout.h
 *
 */

#ifndef __VOUT_H__
#define __VOUT_H__

#ifdef CONFIG_VIDEO_OMAP_VIDEOOUT
extern int omapvout_force_rotation(int plane, int enable, int rotation);
#else
static int omapvout_force_rotation(int plane, int enable, int rotation)
    {return 0; }
#endif

#endif /* __VOUT_H__ */


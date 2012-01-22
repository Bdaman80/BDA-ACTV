/*
 * cs48L10.h  --  CS48L10/CHAS DSP driver
 *
 * Copyright 2010 Cirrus Logic, Inc.
 *
 * Author: Georgi Vlaev, Nucleus Systems Ltd. <office@nucleusys.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#ifndef __LINUX_SPI_CS48L10_H__
#define __LINUX_SPI_CS48L10_H__

/*
    Platform data / DSP GPIOs
*/
struct cs48l10_platform_data
{
	int gpio_busy;
	int gpio_int;
	int gpio_reset;
};


/*
 * Userspace interface for /dev/csdsp
 */

#include <linux/types.h>
#include <linux/compiler.h>
#include <linux/ioctl.h>

#define CS48L10IO                       0xE0

#define CS48L10_INVALID_FORMAT          -1
#define CS48L10_FORMAT_DEFAULT          0
#define CS48L10_FORMAT_PCM              0x00000000
#define CS48L10_FORMAT_MP3              0x01000000
#define CS48L10_FORMAT_AMR_NB           0x02000000
#define CS48L10_FORMAT_AMR_WB           0x03000000
#define CS48L10_FORMAT_AAC              0x04000000
#define CS48L10_FORMAT_HE_AAC_V1        0x05000000
#define CS48L10_FORMAT_HE_AAC_V2        0x06000000
#define CS48L10_FORMAT_VORBIS           0x07000000
#define CS48L10_FORMAT_WMA              0xf0000000
#define CS48L10_FORMAT_MAIN_MASK        0xFF000000
#define CS48L10_FORMAT_SUB_MASK         0x00FFFFFF

struct cs48l10_audio_fmt {
	__u32 format;
	__u32 sample_rate;
	__u32 bitrate;
	__u8 channels;
};

struct cs48l10_hostif_config {
	__u32 packet_unit_size;
	__u32 data_buf_size;
	__u32 watermark_level;
};
/* stream format */
#define CS48L10_GET_STREAM_FMT  _IOR(CS48L10IO, 0x00,  \
	struct cs48l10_audio_fmt)
#define CS48L10_SET_STREAM_FMT  _IOW(CS48L10IO, 0x01,  \
	struct cs48l10_audio_fmt)

/* project/firmware info */
#define CS48L10_GET_PROJECT     _IO(CS48L10IO,   0x10)
#define CS48L10_SET_PROJECT     _IO(CS48L10IO,   0x11)
#define CS48L10_GET_HOSTIF_CONFIG     _IOR(CS48L10IO, 0x12, \
		    struct cs48l10_hostif_config)
#define CS48L10_SET_HOSTIF_CONFIG     _IOW(CS48L10IO, 0x13, \
		    struct cs48l10_hostif_config)

/* playback control */
#define CS48L10_PAUSE           _IO(CS48L10IO,   0x20)
#define CS48L10_RESUME          _IO(CS48L10IO,   0x21)
#define CS48L10_STOP            _IO(CS48L10IO,   0x22)
#define CS48L10_GET_DECODED_FRAMES      _IOR(CS48L10IO,   \
		    0x23, unsigned long)

/* volume control */
#define CS48L10_SET_MUTE        _IO(CS48L10IO,   0x30)
#define CS48L10_GET_MUTE        _IO(CS48L10IO,   0x31)
#define CS48L10_SET_VOLUME      _IO(CS48L10IO,   0x32)
#define CS48L10_GET_VOLUME      _IO(CS48L10IO,   0x33)


#endif /* __LINUX_SPI_CS48L10_H__ */

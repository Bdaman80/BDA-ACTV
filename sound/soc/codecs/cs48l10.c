/*
 * cs48L10.c  --  CS48L10 DSP driver
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
 * Contains code from the microcondeser loader / Cirrus Logic.
 *
 * 2010.10 - Initial release
 * 2011.05 - Added compressed audio interface
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/spi/cs48l10.h>
#include <linux/firmware.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/wakelock.h>
#include <linux/time.h>

#include <linux/io.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/hrtimer.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <linux/regulator/consumer.h>

#define CS48L10_CDEV

#define DRV_VERSION			"0.14"
#define CS48L10_SPI_WRITE_CMD		0x80
#define CS48L10_SPI_READ_CMD		0x81
#define CS48L10_SPI_BUSY_WAIT_USEC	(5)	/* usec */
#define CS48L10_SPI_INT_WAIT_MSEC	(2)	/* msec */

#define WAKELOCK_HOLD_THRESHOLD_MS      20000

#define MASTER_I2C              0x00
#define MASTER_SPI1             0x08
#define MASTER_SPI2             0x01
#define MASTER_SPI3             0x09
#define SLAVE_I2C               0x04
#define SLAVE_SPI               0x05

#define PREKICKSTART_CFG        0x00
#define KICKSTART_CFG           0x01
#define INITIAL_CFG		0x02
#define USER_SNAPSHOT1          0x03

/* Chas response codes */
#define CS48L10_BOOT_START 		0x00000001
#define CS48L10_BOOT_SUCCESS 		0x00000002
#define CS48L10_APP_START 		0x00000004
#define CS48L10_BOOT_READY		0x00000005
#define CS48L10_BOOT_ERROR_CHECKSUM 	0x000000FF
#define CS48L10_INVALID_BOOT_TYPE	0x000000FE
#define CS48L10_BOOT_FAILURE 		0x000000F8
#define CS48L10_APPLICATION_FAILURE 	0xF0000000	/* 0xF0[ULD_ID]0000 */

#define CS48L10_DEFAULT_PROJECT_ID	0
#define CS48L10_FIRMWARE_NAME		"cs48l10.bin"

#define FLASH_ADDRESS_INVALID (flash_address_t)0
#define FLASH_IMAGE_MARKER (dsp_word_t)0x1A2B3C4D

/* slots for OS/MPM/VPM/PPM */
#define MAX_ULDS_PER_SET 5

/** maximum number of characters that can be used for display names.
 *
 * MAX_DISPLAY_LENGTH%sizeof(dsp_word_t) must == 0, for checksum calculation to be correct
 */
#define MAX_DISPLAY_LENGTH 15

/** \defgroup flash_image
These are the data structures that are found in the actual flash image.
*/

#define INPUT_SRC_UNKNOWN 0
#define INPUT_SRC_ANALOG  1
#define INPUT_SRC_OPTICAL 2
#define INPUT_SRC_COAXIAL 3

#undef CS48L10_ASYNC_FWLOADER	/* module */

typedef unsigned char sample_rate_t;
typedef unsigned char input_src_t;
typedef u32 dsp_word_t;
typedef u32 firmware_address_t;

// DSP Boot Loader Write Messages:
const dsp_word_t Slave_Boot = 0x80000000;
const dsp_word_t Soft_Reset = 0x40000000;
const dsp_word_t Soft_Boot[] = { 0x81000009, 0x00000001 };

/** a flash image pointer to a .cfg set of words.
\ingroup flash_image
 */
typedef struct {
	dsp_word_t cfg_length;			/** in 32-bit words. */
	firmware_address_t cfg_ptr;	/** byte-address within the flash.*/
} __attribute__ ((packed)) cfg_ptr_t;

typedef struct {
	dsp_word_t uld_length;			/** in 32-bit words. */
	firmware_address_t uld_ptr;	/** byte-address within the flash.*/
} __attribute__ ((packed)) uld_ptr_t;

/** all the information about a specific project uld set in flash image.
 *  The uld_set list is of the form:
 *  @verbatim
+------------------------+------------------------+
| os_uld size (bytes)    | os_uld flash address   |
+------------------------+------------------------+
| mpm_uld size (bytes)   | mpm_uld flash address  |
+------------------------+------------------------+
| vpm_uld size (bytes)   | vpm_uld flash address  |
+------------------------+------------------------+
| ppm_uld size (bytes)   | ppm_uld flash address  |
+------------------------+------------------------+
| memory config          | memory map (version)   |
+------------------------+------------------------+
\ingroup flash_image
 */
typedef struct {
	uld_ptr_t uld_address[MAX_ULDS_PER_SET];
	dsp_word_t memory_configuration;	/*e.g. 2,4,6, or 8 */
	dsp_word_t memory_map;	/*e.g. 1,2,9 */
} __attribute__ ((packed)) uld_set_t;

typedef struct {
	input_src_t input_src;
	sample_rate_t input_fs;
	sample_rate_t output_fs;
	u8 reserved;
	uld_set_t uld_set;
	cfg_ptr_t snapshot;	/* first two snapshots are prekick and initial */
} __attribute__ ((packed)) project_t;

/** the project master list - a list of all info about all uld_sets in flash image.
 *  The list is of the form:
 *  @verbatim
+------------------------+
| num_projects (n)       |
+------------------------+
| max_snapshots_per_proj |
+------------------------+
| row_size (bytes)       |
+------------------------+----------------+-----------------+-----------------+-----------------+----------------+----+----------------+
| input_src_t 0          | uld_set_t 0    | kickstart ptr   | preKick cfg ptr | initial cfg ptr | snapshot 1 ptr | .. | snapshot n ptr |
+------------------------+----------------+-----------------+-----------------+-----------------+----------------+----+----------------+
| ...                    |...             | ...             | ...             | ...             |                | .. | ...            |
+------------------------+----------------+-----------------+-----------------+-----------------+----------------+----+----------------+
| input_src_t n-1        | uld_set_t n-1  | kickstart ptr   | preKick cfg ptr | initial cfg ptr | snapshot 1 ptr | .. | snapshot n ptr |
+------------------------+----------------+-----------------+-----------------+-----------------+----------------+----+----------------+

Each cfg ptr is of the form:
+------------+-------------+
| cfg_length | cfg_address |
+------------+-------------+
 *  @endverbatim
\ingroup flash_image
 */

typedef struct {
	dsp_word_t num_projects;
	dsp_word_t max_cfgs_per_project;
	dsp_word_t row_size;
	project_t project;
} __attribute__ ((packed)) project_master_list_t;

typedef char display_name_t[MAX_DISPLAY_LENGTH + 1];

/** data structure containing the displayable name for the project and all snapshots.
 *
 *  Names for the prekick, kickstart, and initial snapshots are NOT considered displayable.
 */
typedef struct {
	display_name_t project_name;
	display_name_t snapshot_name;
} __attribute__ ((packed)) project_display_t;

/* the project display text master list.  */
typedef struct {
	dsp_word_t num_projects;	/* should be the same as project_master_list.num_projects */
	dsp_word_t max_snapshot_names_per_project;	/* will be less than project master_list */
	dsp_word_t row_size;	/* bytes per row == bytes per project */
	project_display_t display;
} __attribute__ ((packed)) project_display_list_t;

/** the initial index part of the flash image.
\ingroup flash_image
 */
typedef struct {
	/** start marker of the flash image. */
	dsp_word_t start_marker;

	/** address of the end marker for the flash image. */
	firmware_address_t image_trailer_marker_ptr;	/* image_trailer_marker_t* */

	dsp_word_t image_checksum;	/* not currently used. */

	/** master pointer list of all ULDs in flash. */
	dsp_word_t project_master_list_size;	/* in bytes */
	firmware_address_t project_master_list_ptr;	/* project_master_list_t* */

	/** address of text display structure. */
	firmware_address_t project_text_display_ptr;	/* project_text_display_t* */

	/** words to use for HCMB boot message. If first word is zero, this is slave boot project */
	dsp_word_t hcmb_message[2];
} __attribute__ ((packed)) updateable_flash_image_index_t;

typedef struct {
    /** unique 32-bit marker flag denoting end of updatable flash image. */
	dsp_word_t image_marker;
} __attribute__ ((packed)) image_marker_t;

struct cs48l10_firmware {
	u32 firmware_size;
	const u8 *firmware_data;

	// Current project that is loaded
	u8 current_project;
	// Succefully booted project
	u8 active_project;

	// structure of flash addresses to DSP code (.uld) and configurations (.cfg)
	updateable_flash_image_index_t *pflash_image;
	project_master_list_t *pmaster_list;
	project_t *projects;
	/* display list*/
	project_display_list_t *pdisplay_list;
	project_display_t *project_names;
};


/* dsp cmd */
struct dsp_cmd {
	dsp_word_t		command;
	dsp_word_t		mask; /* match mask */
	dsp_word_t		value;
	struct completion	cmd_wait;
	struct list_head	link;
	int 			state;
};


/* dsp_audio_decoder */
struct dsp_audio_decoder;

struct dsp_audio_decoder_ops {
	/* TODO: Add prekickstart ? */
	int (*init)(struct dsp_audio_decoder *ad,
		struct cs48l10_audio_fmt *fmt);
	int (*resume)(struct dsp_audio_decoder *ad);
	int (*pause)(struct dsp_audio_decoder *ad);
	int (*stop)(struct dsp_audio_decoder *ad);
	int (*get_decoded_frames)(struct dsp_audio_decoder *ad,
		uint32_t *frames);
};

struct dsp_audio_decoder {
	u32 dsp_module_id;
	void *decoder_priv;
	struct cs48l10_audio_fmt fmt;
	struct dsp_audio_decoder_ops *ops;
};

/* cs48l10 private data */
struct cs48l10 {
	struct spi_device *spi;
	const struct firmware *osfw;	/* firmware class */
	struct cs48l10_firmware *fw;	/* parsed firmware */
	struct mutex lock;

	/* gpios from platform_data */
	int gpio_int;
	int gpio_busy;
	int gpio_reset;

	/* CDEV DSP Sound interface */
	struct cdev cdev;
	wait_queue_head_t wait;
	wait_queue_head_t rx_data_wait;
	int rx_data_available;
	struct wake_lock gpio_int_wake_wakelock;
	char *buffer;

	/* DSP */
	struct cs48l10_audio_fmt dsp_fmt; /* current stream format */
	u32 dsp_packet_unit_size;
	u32 dsp_chunk_size; /* dsp_packet_unit_size aligned */
	int unsol_timeout;
#define DSP_MODE_CMD	0
#define DSP_MODE_CMD_DATA	1
#define DSP_MODE_DATA		2
	int dsp_mode;
#define DSP_STATE_LOW_POWER	0
#define DSP_STATE_STOP		1
#define DSP_STATE_PLAY		2
#define DSP_STATE_PAUSE		3
	int dsp_state;

    struct delayed_work dsp_clk_req_disable_work;
	struct work_struct 	int_work;
	struct list_head	cmd_list;
	struct mutex		cmd_lock; /* cmd mutex */

	struct dsp_cmd	cached_unsol;
	int 		has_cached_unsol;

	/* DSP module rendering the current stream */
	u32 dsp_stream_mod_id;
	struct dsp_audio_decoder decoder;
};

extern void audio_clock_request(int request_clock);

int cs48l10_busy(struct cs48l10 *cs48l10);

static void dsp_configure_for_mono(struct cs48l10 *cs48l10);
static void dsp_configure_for_underrun_glitch(struct cs48l10 *cs48l10);
static void dsp_configure_watermark_level(struct cs48l10 *cs48l10, u32 bitrate);

static inline int spi_write_dma(struct spi_device *spi, const u8 *buf,
	size_t len)
{
	struct spi_transfer	t = {
		.tx_buf		= buf,
		.len		= len,
	};
	struct spi_message m;

	m.is_dma_mapped = 1;
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spi_sync(spi, &m);
}

/*
    SPI access. Fimrware loader requires a separate set of
    spi read/write functions.
*/


#define CS48L10_SPI_BOOT_SPEED		12000000 	/* firmware loading */
#define CS48L10_SPI_AUDIOIF_SPEED	24000000 	/* audio interface */
#define BUSY_RETRY	1000

/**
 * cs48l10_spi_speed() - Set new SPI speed for all spi transactions
 *
 * @spi: the spi device
 * @max_speed_hz: the new spi speed < 24 MHz
 * Note:
 *	Although we can set speed for each spi transcation, the current
 *	omap_mcspi driver ignores this setting (fixed in the newer versions)
 */
static int cs48l10_spi_speed(struct spi_device *spi, int max_speed_hz)
{
	int ret;

	if (spi->max_speed_hz == max_speed_hz &&
		spi->bits_per_word == 8 &&
		spi->mode == SPI_MODE_0)
			return 0;

	if (max_speed_hz > 24000000 || max_speed_hz < 2000000)
		max_speed_hz = CS48L10_SPI_BOOT_SPEED;

	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_0; 		/* Normal clock, rising edge */
	spi->max_speed_hz = max_speed_hz; 	/* Max clock is 24Mhz */

	ret = spi_setup(spi);
	if (ret < 0) {
		dev_err(&spi->dev, "spi setup (%u Hz) failed\n",
			max_speed_hz);
		return ret;
	}
	dev_info(&spi->dev, "New SPI speed: %u Hz\n", max_speed_hz);
	return 0;
}
/*
 * 	cs48l10_write_dsp_word()
 *		Writes single 32bit BE dsp_word_t
 */
static int cs48l10_write_dsp_word(struct cs48l10 *cs48l10, const u32 dsp_word)
{
	int ret;
	u8 buf[5];
	u8 *pbuf = &buf[1];

	buf[0] = CS48L10_SPI_WRITE_CMD;
	(*(u32 *) pbuf) = cpu_to_be32(dsp_word);

	if ((ret = spi_write(cs48l10->spi, buf, sizeof(buf))) < 0)
		dev_err(&cs48l10->spi->dev,
			": %s() write %d bytes failed: ret = %d\n",
			__FUNCTION__, sizeof(buf), ret);
	return ret;
}

/*
    cs48l10_read_dsp_word()
	Reads 32bit BE dsp_word_t
*/
static int cs48l10_read_dsp_word(struct cs48l10 *cs48l10, u32 * dsp_word)
{
	int ret;
	struct spi_message msg;
	struct spi_transfer tx, rx;
	u32 srx = 0;
	u8 stx = CS48L10_SPI_READ_CMD;

	memset(&tx, 0, sizeof(struct spi_transfer));
	memset(&rx, 0, sizeof(struct spi_transfer));

	/* tx xfer */
	tx.tx_buf = &stx;
	tx.len = sizeof(stx);
	tx.bits_per_word = 8;
	tx.speed_hz = cs48l10->spi->max_speed_hz;

	/* rx xfer */
	rx.rx_buf = &srx;
	rx.len = sizeof(srx);
	rx.bits_per_word = 8;
	rx.speed_hz = cs48l10->spi->max_speed_hz;

	spi_message_init(&msg);
	spi_message_add_tail(&tx, &msg);
	spi_message_add_tail(&rx, &msg);

	if ((ret = spi_sync(cs48l10->spi, &msg)) < 0) {
		dev_err(&cs48l10->spi->dev,
			": %s() read %d bytes failed: ret = %d\n",
			__FUNCTION__, sizeof(srx), ret);
		return ret;
	}
	dev_dbg(&cs48l10->spi->dev,
		": %s() result: %08x\n", __FUNCTION__, cpu_to_be32(srx));

	*dsp_word = cpu_to_be32(srx);
	return ret;
}

#define DSP_WORD_TRASFER_SIZE 1
/*
    cs48l10_write_dsp_word_ptr_buf()
	Writes a buffer of dsp_word_t.
*/

static int cs48l10_write_dsp_word_ptr_buf(struct cs48l10 *cs48l10,
					  const u8 * dsp_word_ptr,
					  int dsp_word_buf_len)
{
	int ret;
	u8 buf[(DSP_WORD_TRASFER_SIZE * sizeof(u32)) + 1];
	int buf_len = DSP_WORD_TRASFER_SIZE * sizeof(u32);

	if (buf_len > dsp_word_buf_len) {
		buf_len = dsp_word_buf_len;
	}

	buf[0] = CS48L10_SPI_WRITE_CMD;
	memcpy((u8 *) & buf[1], dsp_word_ptr, buf_len);

	buf_len++;
	if ((ret = spi_write(cs48l10->spi, buf, buf_len)) < 0)
		dev_err(&cs48l10->spi->dev,
			": %s() write %d bytes failed: ret = %d\n",
			__FUNCTION__, buf_len, ret);
	return ret;
}

/*
    cs48l10_write_buf()
	Write buffer (ULD, CFG), check for BUSY.
*/
static int cs48l10_write_buf(struct cs48l10 *cs48l10, const u8 * buf, int len)
{
	int ret;
	int write_len = 0;
	int chunk_len = DSP_WORD_TRASFER_SIZE * sizeof(u32);

	len &= ~3;
	write_len = len;

	while (write_len) {
		if (cs48l10_busy(cs48l10))
				return -EBUSY;

		if (write_len < chunk_len)
			chunk_len = write_len;

		ret = cs48l10_write_dsp_word_ptr_buf(cs48l10, buf, chunk_len);

		if (ret < 0)
			return ret;
		write_len -= chunk_len;
		buf += chunk_len;
	}
	return len;
}

static void endian_convert_last64(struct cs48l10 *cs48l10, u8 *buf, int len)
{
	int i = 0;
	u32  data;
	void *pbuf;

	while (i < len)
		i = i+64;
	if (i == len)
		return;
	i = i - 64;
	pbuf = &buf[i];

	for (; i < (len); i = i+4) {
		data = *(u32 *) &(buf[i]);
		(*(u32 *) pbuf) = cpu_to_be32(data);
		pbuf = pbuf + 4;
	}


}
static int dsp_write_audio_buf(struct cs48l10 *cs48l10, u8 *buf, int len);
/*
    cs48l10_write_uld_using_dma()
	Write ULD using DMA over SPI
*/
static int cs48l10_write_uld_using_dma(struct cs48l10 *cs48l10, const u8 * buf,
	int len)
{
	int ret;

	memcpy(&cs48l10->buffer[1], buf, len);
	cs48l10->buffer[0] = CS48L10_SPI_WRITE_CMD;
	/* the last 63 or less bytes which do not use DMA transfer needs to
	 * be endian converted because of the way the bytes are stored in the
	 * ULD file */
	endian_convert_last64(cs48l10, &cs48l10->buffer[0], len);

	/* Write firmware ULD data using dsp_write_audio_buf to use DMA write
	 * to DSP */
	ret = dsp_write_audio_buf(cs48l10, cs48l10->buffer, len + 1);
	if (ret < 0)
		return 0;

	return len;
}

/*
    cs48l10_write_then_read()
	Write DSP words, wait for CS48L10_nINT and then read.
*/
static int cs48l10_write_then_read(struct cs48l10 *cs48l10,
				const dsp_word_t *txwords, int txlen,
				dsp_word_t *rxwords, int rxlen)
{
	int ret, rxread = 0;

	cs48l10->rx_data_available = 0;

	/* Send the DSP command words */
	while (txlen--) {
		ret = cs48l10_write_dsp_word(cs48l10, *txwords);
		if (ret < 0)
			return ret;
		txwords++;
	}
	/* Wait for nINT */
	ret = wait_event_interruptible_timeout(cs48l10->rx_data_wait,
		!gpio_get_value(cs48l10->gpio_int), 5 *  HZ);
	if (ret < 0)
		return ret;

	if (!ret) {
		dev_err(&cs48l10->spi->dev,
			"%s(): Timeout while waiting for dsp response\n",
			__func__);
		return -ETIMEDOUT;
	}

	/* Read DSP response */
	while (rxlen--) {
		ret = cs48l10_read_dsp_word(cs48l10, rxwords);
		if (ret < 0)
			return ret;
		rxwords++; rxread++;
		if (gpio_get_value(cs48l10->gpio_int))
			break;
	}
	/* nINT still low ? */

	cs48l10->rx_data_available = 0;

	return rxread;
}

/*
    cs48l10_busy()
	Wait for BUSY
*/
int cs48l10_busy(struct cs48l10 *cs48l10)
{
	int retry = BUSY_RETRY;
	int dsp_busy;

	if (!cs48l10->gpio_busy) {
		udelay(CS48L10_SPI_BUSY_WAIT_USEC * 10);
		return 0;
	}

	while (retry--) {
		dsp_busy = gpio_get_value(cs48l10->gpio_busy);
		if (dsp_busy != 0)
			return 0;

		udelay(CS48L10_SPI_BUSY_WAIT_USEC);
	}
	return 1;
}

/*
    cs48l10_dsp_reset()
	DSP reset
*/
int cs48l10_dsp_reset(struct cs48l10 *cs48l10)
{
	int ret;
	dsp_word_t dsp_result;

	/*
	   INT and BUSY are configured as inputs with PULLUPs
	   on OMAP3/Beagle.

	   [HS0,HS1] = [1,1] => DSP is SPI slave.
	 */
	if (!cs48l10->gpio_reset)
		return -EINVAL;
	gpio_set_value(cs48l10->gpio_reset, 0);
	udelay(100);
	gpio_set_value(cs48l10->gpio_reset, 1);
	udelay(100);

	ret = wait_event_interruptible_timeout(cs48l10->rx_data_wait,
			!gpio_get_value(cs48l10->gpio_int), 5 * HZ);
	if (ret < 0)
		return ret;
	if (!ret) {
		dev_err(&cs48l10->spi->dev,
			"%s(): Timeout while wiating for BOOT_READY (%08x)\n",
			__func__, CS48L10_BOOT_READY);
		return -ETIMEDOUT;
	}

	/* Read DSP response */
	ret = cs48l10_read_dsp_word(cs48l10, &dsp_result);
	if (ret < 0)
		return ret;

	/* BOOT_READY expected  */
	if (dsp_result != CS48L10_BOOT_READY) {
		dev_err(&cs48l10->spi->dev,
			"DSP result = 0x%08x, expected %08x\n",
			dsp_result, CS48L10_BOOT_READY);

		return -EIO;
	}

	return 0;
}


/*
    Cirrus Logic Microcondeser Firmware Loader
*/

/*
    cs48l10_fw_validate_ptr()
	Validate pointer value in firmware
*/
int cs48l10_fw_validate_ptr(struct cs48l10_firmware *fw, firmware_address_t ptr)
{
	return (ptr >= fw->firmware_size);
}

/*
    cs48l10_fw_load_config()
	Loads CFG block
*/
int cs48l10_fw_load_config(struct cs48l10 *cs48l10, int cfg)
{
	struct cs48l10_firmware *fw = cs48l10->fw;
	int ret;
	firmware_address_t cfg_addr;
	firmware_address_t cfg_ptr_addr;
	cfg_ptr_t *pcfg;

	const u8 *buf;
	u32 bufsize;

	// get Snapshot[cfg] ptr and address from flash, only 1 snapshot ptr and
	// address is saved in the MCU RAM.
	cfg_addr = cpu_to_be32(fw->pflash_image->project_master_list_ptr) +
	    sizeof(project_master_list_t) - sizeof(project_t) +
	    (cpu_to_be32(fw->pmaster_list->row_size) * fw->current_project) +
	    sizeof(project_t) - sizeof(cfg_ptr_t) + (sizeof(cfg_ptr_t) * cfg);

	if (cs48l10_fw_validate_ptr(fw, cfg_addr))
		return -EIO;

	pcfg = (cfg_ptr_t *) (fw->firmware_data + cfg_addr);
	cfg_ptr_addr = cpu_to_be32(pcfg->cfg_ptr);
	bufsize = cpu_to_be32(pcfg->cfg_length);

	dev_dbg(&cs48l10->spi->dev, "CFG @ %08x, size %d, ptr %08x\n",
		 cfg_addr, bufsize, cfg_ptr_addr);

	if (cs48l10_fw_validate_ptr(fw, cfg_ptr_addr + bufsize))
		return -EIO;

	if (bufsize && cfg_ptr_addr) {
		buf = fw->firmware_data + cfg_ptr_addr;

		if ((ret = cs48l10_write_buf(cs48l10, buf, bufsize)) < 0)
			return ret;
	}
	return 0;
}

/*
    cs48l10_fw_slave_boot_uld()
	Slaveboots ULD
*/
int cs48l10_fw_slave_boot_uld(struct cs48l10 *cs48l10, u32 addr, u32 size)
{
	struct cs48l10_firmware *fw = cs48l10->fw;

	int ret;
	const u8 *buf;
	dsp_word_t dsp_result;

	/* Send SLAVE_BOOT message */
	ret = cs48l10_write_then_read(cs48l10, &Slave_Boot, 1, &dsp_result, 1);
	if (ret < 0)
		return ret;

	/* BOOT_START expected  */
	if (dsp_result != CS48L10_BOOT_START) {
		dev_err(&cs48l10->spi->dev,
			"DSP result = 0x%08x, expected %08x\n", dsp_result,
			CS48L10_BOOT_START);

		return -EIO;
	}

	dev_dbg(&cs48l10->spi->dev, "\t@ %08x, size %d\n", addr, size);

	if (cs48l10_fw_validate_ptr(fw, addr + size))
		return -EIO;

	buf = fw->firmware_data + addr;

	cs48l10->rx_data_available = 0;

	ret = cs48l10_write_uld_using_dma(cs48l10, buf, size);
	if (ret < 0)
		return ret;

	ret = wait_event_interruptible_timeout(cs48l10->rx_data_wait,
		cs48l10->rx_data_available, 5 * HZ);
	if (ret < 0)
		return ret;
	if (!ret) {
		dev_err(&cs48l10->spi->dev,
			"%s(): Timeout while waiting for dsp response\n",
			__func__);
		return -ETIMEDOUT;
	}

	/* Read DSP response */
	if ((ret = cs48l10_read_dsp_word(cs48l10, &dsp_result)) < 0)
		return ret;

	/* BOOT_SUCCESS expected  */
	if (dsp_result != CS48L10_BOOT_SUCCESS) {
		dev_err(&cs48l10->spi->dev,
			"DSP result = 0x%08x, expected %08x\n", dsp_result,
			CS48L10_BOOT_SUCCESS);

		return -EIO;
	}

	dev_dbg(&cs48l10->spi->dev, "\t@ %08x, size %d [OK]\n", addr, size);

	return 0;
}

/*
    cs48l10_fw_load_project()
	Loads project from firmware
*/
int cs48l10_fw_load_project(struct cs48l10 *cs48l10, int project_id)
{
	struct cs48l10_firmware *fw = cs48l10->fw;

	int ret, i;
	project_t *pproject;
	project_display_t *pproject_name;
	dsp_word_t dsp_result = 0;
	struct cs48l10_audio_fmt *dsp_fmt = &cs48l10->dsp_fmt;

/*     	Brian */
/*     	pproject = &fw->projects[project_id]; */
	pproject = (project_t *)((u8 *) fw->projects +
		cpu_to_be32(fw->pmaster_list->row_size) * project_id);

	pproject_name = (project_display_t *)((u8 *) fw->project_names +
		cpu_to_be32(fw->pdisplay_list->row_size) * project_id);

	dev_info(&cs48l10->spi->dev, "Project[%d] %s\n", project_id,
		pproject_name->project_name);

	dev_dbg(&cs48l10->spi->dev, "Project Input Src	%02x\n",
		 pproject->input_src);
	dev_dbg(&cs48l10->spi->dev, "Project Input FS	%02x\n",
		 pproject->input_fs);
	dev_dbg(&cs48l10->spi->dev, "Project Output FS	%02x\n",
		 pproject->output_fs);

	// send OS, MPM, VPM, PPM .uld files IF they exist
	for (i = 0; i < MAX_ULDS_PER_SET; i++) {
		// check to see if there is a none zero uld length, indicated there is a valid .uld file
		if (pproject->uld_set.uld_address[i].uld_length == 0)
			continue;

		// slaveboot the .uld file from SPI_FLASH to the DSP
		dev_dbg(&cs48l10->spi->dev, "ULD[%d]\n", i);
		ret = cs48l10_fw_slave_boot_uld(cs48l10,
			cpu_to_be32(pproject->uld_set.uld_address[i].uld_ptr),
			cpu_to_be32(pproject->uld_set.uld_address[i].
				uld_length));
		if (ret < 0)
			return ret;
	}

	/* Send SOFT_RESET message */
	ret = cs48l10_write_then_read(cs48l10, &Soft_Reset, 1, &dsp_result, 1);
	if (ret < 0)
		return ret;

	/* APP_START expected  */
	if (dsp_result != CS48L10_APP_START) {
		dev_err(&cs48l10->spi->dev,
			"DSP result = 0x%08x, expected %08x\n", dsp_result,
			CS48L10_APP_START);

		return -EIO;
	}

	fw->current_project = project_id;

	ret = cs48l10_fw_load_config(cs48l10, PREKICKSTART_CFG);
	if (ret < 0)
		return ret;

	ret = cs48l10_fw_load_config(cs48l10, INITIAL_CFG);
	if (ret < 0)
		return ret;
	/* handle commands needed for mono configuration */
	if ((dsp_fmt->channels == 1) &&
			(dsp_fmt->format != CS48L10_FORMAT_HE_AAC_V1) &&
			(dsp_fmt->format != CS48L10_FORMAT_HE_AAC_V2)) {
		dev_info(&cs48l10->spi->dev,
			"CS49L10 configured for mono playback\n");

		dsp_configure_for_mono(cs48l10);
	}
	dsp_configure_for_underrun_glitch(cs48l10);

	ret = cs48l10_fw_load_config(cs48l10, KICKSTART_CFG);
	if (ret < 0)
		return ret;

	/* 5ms delay */
	mdelay(5);

	ret = cs48l10_spi_speed(cs48l10->spi, CS48L10_SPI_AUDIOIF_SPEED);
	if (ret < 0)
		return ret;

	fw->active_project = project_id;

	return 0;
}

/*
    cs48l10_fw_softboot_project()
	Softboot project from firmware
*/
int cs48l10_fw_softboot_project(struct cs48l10 *cs48l10, int project_id)
{
	struct cs48l10_firmware *fw = cs48l10->fw;
	int ret;
	dsp_word_t dsp_result;

	if (!fw)
		return -EINVAL;

	if (project_id >= cpu_to_be32(fw->pmaster_list->num_projects)) {
		dev_err(&cs48l10->spi->dev,
			 "Project %d is not in projects master list\n",
			 project_id);
		return -EINVAL;
	}

	ret = cs48l10_write_then_read(cs48l10, Soft_Boot, ARRAY_SIZE(Soft_Boot),
		&dsp_result, 1);
	if (ret < 0)
			return ret;

	if (dsp_result != CS48L10_BOOT_READY) {
		dev_err(&cs48l10->spi->dev,
			"DSP result = 0x%08x, expected %08x\n", dsp_result,
			CS48L10_BOOT_READY);
		return -EIO;
	}
	return cs48l10_fw_load_project(cs48l10, project_id);
}

/*
    cs48l10_fw_boot_project()
	Boot project from firmware (initial boot)
*/
int cs48l10_fw_boot_project(struct cs48l10 *cs48l10, int project_id)
{
	struct cs48l10_firmware *fw = cs48l10->fw;
	int ret;

	if (!fw)
		return -EINVAL;

	if (project_id >= cpu_to_be32(fw->pmaster_list->num_projects)) {
		dev_err(&cs48l10->spi->dev,
			 "Project %d is not in projects master list\n",
			 project_id);
		return -EINVAL;
	}

	ret = cs48l10_spi_speed(cs48l10->spi, CS48L10_SPI_BOOT_SPEED);
	if (ret < 0)
		return ret;

	if ((ret = cs48l10_dsp_reset(cs48l10)) < 0) {
		dev_err(&cs48l10->spi->dev,
			"Error loading project [%d]. DSP reset failed\n",
			project_id);
		return ret;
	}

	return cs48l10_fw_load_project(cs48l10, project_id);
}

/*
    cs48l10_fw_parse_master_list()
	Parse firware project master list
*/
int cs48l10_fw_parse_master_list(struct cs48l10 *cs48l10)
{
	struct cs48l10_firmware *fw = cs48l10->fw;
	firmware_address_t projects_addr, project_addr_last;
	firmware_address_t project_names_addr, project_names_addr_last;

	fw->current_project = 0;
	fw->active_project = 0;

	projects_addr = cpu_to_be32(fw->pflash_image->project_master_list_ptr) +
	    sizeof(project_master_list_t) - sizeof(project_t);

	if (cs48l10_fw_validate_ptr(fw, projects_addr))
		return -EIO;

	project_addr_last = projects_addr +
		(cpu_to_be32(fw->pmaster_list->row_size) *
		(cpu_to_be32(fw->pmaster_list->num_projects) - 1));

	if (cs48l10_fw_validate_ptr(fw, project_addr_last))
		return -EIO;

	fw->projects = (project_t *) (fw->firmware_data + projects_addr);

	project_names_addr =
		cpu_to_be32(fw->pflash_image->project_text_display_ptr) +
		sizeof(project_display_list_t) - sizeof(project_display_t);

	if (cs48l10_fw_validate_ptr(fw, project_names_addr))
		return -EIO;

	project_names_addr_last = project_names_addr +
		(cpu_to_be32(fw->pdisplay_list->row_size) *
		(cpu_to_be32(fw->pdisplay_list->num_projects) - 1));

	if (cs48l10_fw_validate_ptr(fw, project_names_addr_last))
		return -EIO;

	fw->project_names = (project_display_t *) (
		fw->firmware_data + project_names_addr);


	return 0;
}

/*
    cs48l10_fw_parse_firmware()
	Parse firmware
*/
int cs48l10_fw_parse_firmware(struct cs48l10 *cs48l10)
{
	struct cs48l10_firmware *fw = cs48l10->fw;
	dsp_word_t image_marker;

	fw->firmware_data = cs48l10->osfw->data;
	fw->firmware_size = cs48l10->osfw->size;

	if (cs48l10_fw_validate_ptr(fw, sizeof(updateable_flash_image_index_t)))
		return -EINVAL;

	fw->pflash_image = (updateable_flash_image_index_t *) fw->firmware_data;
	dev_dbg(&cs48l10->spi->dev,
		 "Start Marker:			%08x\n",
		 cpu_to_be32(fw->pflash_image->start_marker));

	if (FLASH_IMAGE_MARKER != cpu_to_be32(fw->pflash_image->start_marker)) {
		dev_info(&cs48l10->spi->dev, "Bad start Marker\n");
		return -EIO;
	}

	if (cs48l10_fw_validate_ptr
	    (fw, cpu_to_be32(fw->pflash_image->image_trailer_marker_ptr)))
		return -EIO;

	image_marker = *((dsp_word_t *) (fw->firmware_data +
					 cpu_to_be32(fw->pflash_image->
						     image_trailer_marker_ptr)));

	if (cpu_to_be32(image_marker) != FLASH_IMAGE_MARKER) {
		dev_err(&cs48l10->spi->dev, "Bad trailer Marker %08x\n",
			 cpu_to_be32(image_marker));
		return -EIO;
	}

	dev_dbg(&cs48l10->spi->dev, "Trailer Marker Ptr:		%08x\n",
		 cpu_to_be32(fw->pflash_image->image_trailer_marker_ptr));
	dev_dbg(&cs48l10->spi->dev,
		 "Image Chekcsum:			%08x\n",
		 cpu_to_be32(fw->pflash_image->image_checksum));
	dev_dbg(&cs48l10->spi->dev, "Project Master List Size:	%08x\n",
		 cpu_to_be32(fw->pflash_image->project_master_list_size));
	dev_info(&cs48l10->spi->dev, "Project Master Ptr:		%08x\n",
		 cpu_to_be32(fw->pflash_image->project_master_list_ptr));
	dev_info(&cs48l10->spi->dev, "Project Text Display Ptr:	%08x\n",
		 cpu_to_be32(fw->pflash_image->project_text_display_ptr));
	dev_dbg(&cs48l10->spi->dev,
		 "HCMB[0,1]:			%08x, %08x\n",
		 cpu_to_be32(fw->
			     pflash_image->hcmb_message
			     [0]),
		 cpu_to_be32(fw->pflash_image->hcmb_message[1]));

	if (cs48l10_fw_validate_ptr
	    (fw, cpu_to_be32(fw->pflash_image->project_master_list_ptr)))
		return -EINVAL;

	fw->pmaster_list = (project_master_list_t *)
	    (fw->firmware_data +
	     cpu_to_be32(fw->pflash_image->project_master_list_ptr));

	dev_info(&cs48l10->spi->dev, "Master List\n");
	dev_info(&cs48l10->spi->dev, "Number of Projects:	%d\n",
		 cpu_to_be32(fw->pmaster_list->num_projects));
	dev_info(&cs48l10->spi->dev, "Max Cfgs per Project:	%d\n",
		 cpu_to_be32(fw->pmaster_list->max_cfgs_per_project));
	dev_info(&cs48l10->spi->dev, "Row size:		%d\n",
		 cpu_to_be32(fw->pmaster_list->row_size));

	fw->pdisplay_list = (project_display_list_t *)
	    (fw->firmware_data +
	     cpu_to_be32(fw->pflash_image->project_text_display_ptr));

	dev_info(&cs48l10->spi->dev, "Display List\n");
	dev_info(&cs48l10->spi->dev, "Number of Projects(display):	%d\n",
		cpu_to_be32(fw->pdisplay_list->num_projects));
	dev_info(&cs48l10->spi->dev, "Max snapshots per Project:	%d\n",
		cpu_to_be32(
			fw->pdisplay_list->max_snapshot_names_per_project));
	dev_info(&cs48l10->spi->dev, "Row size:		%d\n",
		cpu_to_be32(fw->pdisplay_list->row_size));


	cs48l10_fw_parse_master_list(cs48l10);

	return 0;
}

/* Microcondenser Firmware Loader */
#ifdef CS48L10_ASYNC_FWLOADER
/*
    cs48l10_fw_load_async_...()
	Async firmware load from a microcondenser image from /lib/firmware
	Use when compiled as built-in module
*/
static void cs48l10_fw_load_async_cont(const struct firmware *osfw,
				       void *context)
{
	struct cs48l10 *cs48l10 = (struct cs48l10 *)context;
	int ret;

	dev_err(&cs48l10->spi->dev, "cs48l10_fw_load_async_cont() ...\n");
	if (!cs48l10)
		return;

	if (!osfw) {
		dev_err(&cs48l10->spi->dev, "Firmware not available\n");
		return;
	}

	mutex_lock(&cs48l10->lock);

	if (cs48l10->fw)
		kfree(cs48l10->fw);

	if (cs48l10->osfw)
		release_firmware(cs48l10->osfw);

	cs48l10->osfw = osfw;

	cs48l10->fw = (struct cs48l10_firmware *)
	    kzalloc(sizeof(struct cs48l10_firmware), GFP_KERNEL);

	if (!cs48l10->fw) {
		release_firmware(cs48l10->osfw);
		cs48l10->osfw = NULL;

		mutex_unlock(&cs48l10->lock);

		return;
	}

	ret = cs48l10_fw_parse_firmware(cs48l10);

	if (ret < 0) {
		dev_err(&cs48l10->spi->dev,
			"firmware (%u bytes) failed to load, ret = %d\n",
			cs48l10->osfw->size, ret);

		kfree(cs48l10->fw);
		release_firmware(cs48l10->osfw);

		cs48l10->osfw = NULL;
		cs48l10->fw = NULL;

		mutex_unlock(&cs48l10->lock);

		return;
	}

	dev_info(&cs48l10->spi->dev,
		 "firmware (%u bytes) parsed successfully\n",
		 cs48l10->osfw->size);

	/* Boot the default project */
	ret = cs48l10_fw_boot_project(cs48l10, CS48L10_DEFAULT_PROJECT_ID);
	if (ret < 0)
		dev_err(&cs48l10->spi->dev,
			"Project %d failed to start, ret = %d\n",
			CS48L10_DEFAULT_PROJECT_ID, ret);

	mutex_unlock(&cs48l10->lock);
}

static int cs48l10_fw_load_async(struct cs48l10 *cs48l10, const char *filename)
{
	int ret;

	cs48l10->fw = NULL;
	cs48l10->osfw = NULL;

	ret =
	    request_firmware_nowait(THIS_MODULE, FW_ACTION_NOHOTPLUG, filename,
				    &cs48l10->spi->dev, cs48l10,
				    cs48l10_fw_load_async_cont);

	if (ret != 0) {
		dev_err(&cs48l10->spi->dev,
			"request async load of %s failed. err = %d\n", filename,
			ret);

		return ret;
	}

	return 0;
}
#else

/*
    cs48l10_fw_load()
	Sync firmware load from a microcondenser image from /lib/firmware
*/
static int cs48l10_fw_load_sync(struct cs48l10 *cs48l10, const char *filename)
{
	int ret;
	ret = request_firmware(&cs48l10->osfw, filename, &cs48l10->spi->dev);

	if (ret != 0) {
		dev_err(&cs48l10->spi->dev,
			"firmware: %s not found. err = %d\n", filename, ret);

		return ret;
	}
	cs48l10->fw = (struct cs48l10_firmware *)
	    kzalloc(sizeof(struct cs48l10_firmware), GFP_KERNEL);

	if (!cs48l10->fw)
		return -ENOMEM;

	mutex_lock(&cs48l10->lock);

	ret = cs48l10_fw_parse_firmware(cs48l10);

	mutex_unlock(&cs48l10->lock);

	if (ret < 0) {
		dev_err(&cs48l10->spi->dev,
			"firmware %s (%u bytes) was not parsed, ret = %d\n",
			filename, cs48l10->osfw->size, ret);

		kfree(cs48l10->fw);
		release_firmware(cs48l10->osfw);

		cs48l10->osfw = NULL;
		cs48l10->fw = NULL;

		return ret;
	}

	dev_info(&cs48l10->spi->dev,
		 "firmware %s (%u bytes) parsed successfully\n",
		 filename, cs48l10->osfw->size);

	return 0;
}

#endif

/* END firmware */


/* CS48L10_nINT irq handler */
static irqreturn_t cs48l10_int_handler(int irq, void *data)
{
	struct cs48l10 *cs48l10 = (struct cs48l10 *)data;
	if (cs48l10->dsp_state == DSP_STATE_PLAY)
		wake_lock(&cs48l10->gpio_int_wake_wakelock);
	cs48l10->rx_data_available = !gpio_get_value(cs48l10->gpio_int);
	wake_up_interruptible(&cs48l10->rx_data_wait);

	if (!list_empty(&cs48l10->cmd_list)) /* may be, or may not be true ..*/
		schedule_work(&cs48l10->int_work);

	return IRQ_HANDLED;
}

/* GPIO */
static int __devinit cs48l10_setup_int_gpio(struct spi_device *spi,
	struct cs48l10 *cs48l10)
{
	struct cs48l10_platform_data *pdata = spi->dev.platform_data;
	int ret;

	if (!pdata)
		return -EINVAL;

	if (!gpio_is_valid(pdata->gpio_int)) {
		dev_err(&spi->dev, "Invalid CS48L10_nINT gpio %d\n",
			pdata->gpio_int);
		return -EINVAL;
	}

/*	ret = gpio_request(pdata->gpio_int, "CS48L10_nINT");
	if (ret) {
		dev_err(&spi->dev,
			"gpio_request() CS48L10_nINT gpio %d failed\n",
			pdata->gpio_int);
		return -EINVAL;
	}*/

	ret = gpio_direction_input(pdata->gpio_int);
	if (ret) {
		gpio_free(pdata->gpio_int);
		return ret;
	}

	ret = request_irq(gpio_to_irq(pdata->gpio_int),
		cs48l10_int_handler, IRQF_TRIGGER_FALLING,
		"cs48l10_int", cs48l10);
	if (ret) {
		dev_err(&spi->dev,
			"request_irq CS48L10_nINT gpio %d -> irq %d failed\n",
			pdata->gpio_int, gpio_to_irq(pdata->gpio_int));
		gpio_free(pdata->gpio_int);
		return ret;
	}
	enable_irq_wake(gpio_to_irq(pdata->gpio_int));

#ifdef CONFIG_GPIO_SYSFS
	/* Expose GPIO value over sysfs for diagnostic purposes */
	gpio_export(pdata->gpio_int, false);
#endif
	cs48l10->gpio_int = pdata->gpio_int;
	wake_lock_init(&cs48l10->gpio_int_wake_wakelock, WAKE_LOCK_SUSPEND,
		"cirrus_gpio_int_wake");
	dev_info(&spi->dev, "CS48L10_nINT gpio %d -> irq %d\n",
	cs48l10->gpio_int, gpio_to_irq(cs48l10->gpio_int));

	return 0;
}


static int __devinit cs48l10_setup_reset_gpio(struct spi_device *spi,
	struct cs48l10 *cs48l10)
{
	struct cs48l10_platform_data *pdata = spi->dev.platform_data;
	int ret;

	if (!pdata)
		return -EINVAL;

	if (!gpio_is_valid(pdata->gpio_reset)) {
		dev_err(&spi->dev, "Invalid CS48L10_nRESET gpio %d\n",
			pdata->gpio_reset);
		return -EINVAL;
	}

/*	ret = gpio_request(pdata->gpio_reset, "CS48L10_nRESET");
	if (ret) {
		dev_err(&spi->dev,
			"gpio_request() CS48L10_nRESET gpio %d failed\n",
			pdata->gpio_reset);
		return -EINVAL;
	}*/
   /* Keep RESET' low until regulator is powered(VD on) */
	ret = gpio_direction_output(pdata->gpio_reset, 0);
	if (ret) {
		gpio_free(pdata->gpio_reset);
		return ret;
	}

#ifdef CONFIG_GPIO_SYSFS
	/* Expose GPIO value over sysfs for diagnostic purposes */
	gpio_export(pdata->gpio_reset, false);
#endif
	cs48l10->gpio_reset = pdata->gpio_reset;
	dev_info(&spi->dev, "CS48L10_nRESET gpio %d\n", cs48l10->gpio_reset);

	return 0;
}

static int __devinit cs48l10_setup_busy_gpio(struct spi_device *spi,
	struct cs48l10 *cs48l10)
{
	struct cs48l10_platform_data *pdata = spi->dev.platform_data;
	int ret;

	if (!pdata)
		return -EINVAL;

	if (!gpio_is_valid(pdata->gpio_busy)) {
		dev_err(&spi->dev, "Invalid CS48L10_nBUSY gpio %d\n",
			pdata->gpio_busy);
		return -EINVAL;
	}

/*	ret = gpio_request(pdata->gpio_busy, "CS48L10_nBUSY");
	if (ret) {
		dev_err(&spi->dev,
			"gpio_request() CS48L10_nBUSY gpio %d failed\n",
			pdata->gpio_busy);
		return -EINVAL;
	}*/

	ret = gpio_direction_input(pdata->gpio_busy);
	if (ret) {
		gpio_free(pdata->gpio_busy);
		return ret;
	}

#ifdef CONFIG_GPIO_SYSFS
	/* Expose GPIO value over sysfs for diagnostic purposes */
	gpio_export(pdata->gpio_busy, false);
#endif
	cs48l10->gpio_busy = pdata->gpio_busy;
	dev_info(&spi->dev, "CS48L10_nBUSY gpio %d\n", cs48l10->gpio_busy);

	return 0;
}


static int cs48l10_free_gpios(struct spi_device *spi, struct cs48l10 *cs48l10)
{
	if (cs48l10->gpio_int) {
		wake_lock_destroy(&cs48l10->gpio_int_wake_wakelock);
		free_irq(gpio_to_irq(cs48l10->gpio_int), cs48l10);
		gpio_free(cs48l10->gpio_int);
	}

	if (cs48l10->gpio_reset)
		gpio_free(cs48l10->gpio_reset);

	if (cs48l10->gpio_busy)
		gpio_free(cs48l10->gpio_busy);

	return 0;
}

/*
    cs48l10_setup_gpios()
	Use the data provided in the dev.platform_data
	Use the gpio_XXX numbers. RESET and INT are essential.
*/
static int __devinit cs48l10_setup_gpios(struct spi_device *spi,
					 struct cs48l10 *cs48l10)
{
	struct cs48l10_platform_data *pdata = spi->dev.platform_data;
	int ret;

	if (!pdata)
		return -EINVAL;

	ret = cs48l10_setup_reset_gpio(spi, cs48l10);
	if (ret != 0)
		return ret;
	ret = cs48l10_setup_int_gpio(spi, cs48l10);
	if (ret != 0)
		return ret;

	cs48l10_setup_busy_gpio(spi, cs48l10);

	return 0;
}

/* END GPIO */

/*
    DSP os/app level read/write functions.
*/
#define DSP_BUFFER_SIZE		0x6000 	/* dsp words */
#define DSP_MIN_PUS		16 	/* min packet unit size (dsp words) */
#define DSP_MAX_PUS		256 	/* max packet unit size (dsp words) */
#define CS48L10_BUFFER_SIZE	(96*1024) /* 96 KB ...*/

/* DSP Modules */
#define DSP_OS		0x01	/* OS module */
#define DSP_AM		0x03	/* Audio Manager */
#define DSP_MP3		0x11	/* MP3 */
#define DSP_AAC		0x18	/* AAC */
#define DSP_HE_AAC_V1	0x18    /* AAC+ module ID */
#define DSP_HE_AAC_V2	0x18    /* AAC+ module ID */
#define DSP_WMA		0x20	/* TODO: WMA module ID */

#define RW_WR		0	/* overwite current value */
#define RW_WROR		1	/* new value or'd with current value */
#define RW_WRAND	2	/* new value and'd with current value */
#define RW_RD		3	/* read value */

/* mod := dsp module id, rw := RW, wc := wordcount - 1*/
#define MK_OP(mod, rw, wc) \
	((((mod & 0x7F) | 0x80) << 8) | ((rw & 3) << 6) | (wc & 0x1F))
#define MK_OPWR(mod)	MK_OP(mod, RW_WR, 0)
#define MK_OPRD(mod)	MK_OP(mod, RW_RD, 0)

#define MK_CMD(op, idx)	((op << 16) | (idx & 0xFFFF)) /* command - op, index*/
#define MK_RES(op)	(op & 0x7FFF)	/* response*/

/* OS Indexes */
#define KICKSTART	0x0000 /* KICKSTART */
#define IO_CONFIG	0x0001 /* IO_CONFIG*/
#define SOFTBOOT	0x0009 /* SOFTBOOT */
#define	SPI_DPUS	0x004A /* SPI_RX_DATA_PACKET_UNIT_SIZE */
#define	SPI_DBS		0x004B /* SPI_RX_DATA_BUF_SIZE */
#define SPI_DBWL	0x004C /* SPI_RX_DATA_BUF_WATERMARK_LEVEL */
#define SPI_DBSA	0x004D /* SPI_RX_DATA_BUF_SPACE_AVAILABLE */

/* Stream renderer /MP3,AAC,AAC+,WMA/ indexes */
#define MP3_CONTROL		0x0000 /* mp3 control */
#define MP3_DECODED_FRAMES	0x0004 /* decoded frames */

#define AAC_CONTROL		0x0000 /* aac control */
#define AAC_FS_CODE		0x0017 /* sample rate (code) */
#define AAC_CH_CONFIG		0x0018 /* channel config */
#define AAC_DECODED_FRAMES	0x0019 /* decoded frames */

/* OP codes */
#define OP_OS_URES	0x8100 /* Unsol Response */

/* Audio Manager */
/* indexes */
#define AM_GAIN		0x0000
#define AM_MUTE		0x0001
/*
    DATA mode. We have 4 buffers (id = 0-3)
    Only buffer id 0 is supported
*/
#define OP_DREQ_ID(id)	(0x8200 | (id & 0x03)) /* id = buffer id (0-3)*/
#define OP_DRES_ID(id)	(0x0200 | (id & 0x03)) /* id = buffer id (0-3)*/
#define OP_DREQ		OP_DREQ_ID(0) 	/* Default buf id = 0 */
#define OP_DRES		OP_DRES_ID(0) 	/* Default buf id = 0 */

/* Unsolicited responses */
#define EVENT_DC	0x10 /* Unsol delivery complete event */
#define EVENT_WM	0x11 /* Unsol watermark event */

/**
 * dsp_write_cmd - Write a DSP command at a given index
 *
 * @cs48l10: the dsp device
 * @opcode: command opcode
 * @index: command index (register)
 * @value: value as 32bit dsp word
 *
 * Returns zero if successful, or a negative error code on failure.
 */
int dsp_write_cmd(struct cs48l10 *cs48l10, u16 opcode, u16 index,
	dsp_word_t value)
{
	int i, ret;
	dsp_word_t request[2];

	if (cs48l10->dsp_mode == DSP_MODE_DATA)
		return -EIO;

	request[0] = MK_CMD(opcode, index);
	request[1] = value;

	for (i = 0; i < ARRAY_SIZE(request); i++) {
		if (cs48l10_busy(cs48l10)) {
			dev_err(&cs48l10->spi->dev,
				": %s() failed: BUSY not going high\n",
				__func__);

			return -EBUSY;
		}

		ret = cs48l10_write_dsp_word(cs48l10, request[i]);
		if (ret < 0) {
			dev_err(&cs48l10->spi->dev, "write %08x, %08x failed\n",
				request[0], request[1]);
			return ret;
		}
	}
	return 0;
}

/**
 * dsp_cmd_dispatch() - Command dispatcher scheduled on interrupt.
 *
 * @work: the scheduled work.
 * Note:
 *	This routine will traverse the request wait list and process
 *	data only for the commands on the list. Unrequested data is
 *	dropped. Caches the last unsol response, if no unsol handler
 * 	is available
 */

#define DSP_CMD_MATCH(res, mask, req) ((res & mask) == (req & mask))
static void dsp_cmd_dispatch(struct work_struct *work)
{
	int ret;
	struct dsp_cmd *c, *cmd = NULL;
	dsp_word_t response = 0;
	struct cs48l10 *cs48l10 = container_of(work, struct cs48l10, int_work);
	int state = 0; /* cmd state 0 - command, 1 - data */

	dev_dbg(&cs48l10->spi->dev, "%s() enter\n", __func__);

	/* Read DSP response. Drop unknown codes */

	/*
	    IF we resume after stop the new fw will generate unsol
	    response, this still can produce race condition, because
	    the unsol readers are not locked togther with the cmd readers
	    e.g we issue read command before unsol reader is started.
	*/

	mutex_lock(&cs48l10->cmd_lock);
	/*
	if (list_empty(&cs48l10->cmd_list)) {
		mutex_unlock(&cs48l10->cmd_lock);
		return;
	}
	*/
	while (!gpio_get_value(cs48l10->gpio_int)) {
		ret = cs48l10_read_dsp_word(cs48l10, &response);
		dev_dbg(&cs48l10->spi->dev, "%s() response = %x\n", __func__,
			response);
		if (ret < 0) {
			mutex_unlock(&cs48l10->cmd_lock);
			return;
		}
		/* unsol prefix or response code */
		if (state == 0) {
			/* Look for a pending command or unsol request */
			list_for_each_entry(c, &cs48l10->cmd_list, link) {
				if (DSP_CMD_MATCH(response, c->mask,
					c->command)) {
					/* found a waiting command or unsol
					   reader */
					c->command = response;
					cmd = c;
					state = 1;
					break;
				}
			} /* list */
			if (state == 1)
				continue;
		} /* state */
		/* value */
		if (state == 1 && cmd) {
			cmd->value = response;
			cmd->state = 1; /* command complete */
			state = 0;
			list_del(&cmd->link);
			complete(&cmd->cmd_wait);
			cmd = NULL;
			continue;
		}

		/*  Unsol response caching ...
		    We don't have pending unsol requests and we don't want to
		    lose unsol response. This is changed in latest firmwares,
		    but still may create problems when sending cmds and writing
		    date. Cache the last unsol response .. if we ever enter in
		    this condition.
		 */
		if (state == 0 &&
			(DSP_CMD_MATCH(response, 0xff000000, 0x81000000) ||
			DSP_CMD_MATCH(response, 0xff000000, 0x02000000))) {
			cs48l10->cached_unsol.command = response;
			state = 1;
			continue;
		}

		if (state == 1) {
			cs48l10->cached_unsol.value = response;
			cs48l10->has_cached_unsol = 1;
			state = 0;
			dev_info(&cs48l10->spi->dev,
				"%s() cached %08x, value %d\n",
				__func__, cs48l10->cached_unsol.command,
				cs48l10->cached_unsol.value);
		}

	}
	mutex_unlock(&cs48l10->cmd_lock);
	dev_dbg(&cs48l10->spi->dev, "%s() exit\n", __func__);
}


/**
 * dsp_clear_queue() - Complete all pending request by leaving
 *			the status 0 (not completed)
 *
 * @cs48l10: the dsp device
 *
 */
void dsp_clear_queue(struct cs48l10 *cs48l10)
{
	struct dsp_cmd *c;
	mutex_lock(&cs48l10->cmd_lock);
	cs48l10->has_cached_unsol = 0;
	list_for_each_entry(c, &cs48l10->cmd_list, link) {
		c->command = 0;
		c->value = 0;
		c->state = 0;
		complete(&c->cmd_wait);
	} /* list */
	mutex_unlock(&cs48l10->cmd_lock);
}


/**
 * dsp_queue_cmd - Add command to responce wait list
 *
 * @cs48l10: the dsp device
 * @dsp_cmd: command
 *
 */
void dsp_queue_cmd(struct cs48l10 *cs48l10, struct dsp_cmd *cmd)
{
	mutex_lock(&cs48l10->cmd_lock);
	list_add_tail(&cmd->link, &cs48l10->cmd_list);
	if (!gpio_get_value(cs48l10->gpio_int))
		schedule_work(&cs48l10->int_work);
	mutex_unlock(&cs48l10->cmd_lock);
}

/**
 * dsp_read_cmd_async - Post a request for read of a DSP word at
			a given index. Waits for dispatch()
 *
 * @cs48l10: the dsp device
 * @opcode: command opcode
 * @index: command index (register)
 * @mask: command prefix match mask for dispatch()
 * @value: the returned value
 *
 * Returns positive if successful, zero on timeout or a negative
 * error code on failure.
 */
int dsp_read_cmd_async(struct cs48l10 *cs48l10, u16 opcode,
	u16 index, dsp_word_t mask, dsp_word_t *value)
{
	int ret;
	struct dsp_cmd *cmd = (struct dsp_cmd *)
		kzalloc(sizeof(struct dsp_cmd), GFP_KERNEL);

	dsp_word_t request = MK_CMD(opcode, index);

	if (!cmd)
		return -ENOMEM;

	cmd->command = request;
	cmd->mask = mask;
	cmd->value = 0;
	init_completion(&cmd->cmd_wait);
	dsp_queue_cmd(cs48l10, cmd);
	/* wait for busy */
	if (cs48l10_busy(cs48l10)) {
		dev_err(&cs48l10->spi->dev,
			": %s()failed: BUSY not going high\n",
			__func__);

		return -EBUSY;
	}

	/* spi write */
	ret = cs48l10_write_dsp_word(cs48l10, request);
	if (ret < 0) {
		dev_err(&cs48l10->spi->dev, "read request %08x failed\n",
			request);
		return ret;
	}

	/* wait for for dispatch */
	ret = wait_for_completion_interruptible_timeout(&cmd->cmd_wait, 1 * HZ);
	mutex_lock(&cs48l10->cmd_lock);
	if (ret > 0 && cmd->state == 1) {
		/* dispatched command are removed from the list */
		*value = cmd->value;
		dev_dbg(&cs48l10->spi->dev,
			"%s(): rq:%08x mask:%08x, resp: %08x, %08x\n",
			__func__, request, mask, cmd->command, cmd->value);
		kfree(cmd);
		mutex_unlock(&cs48l10->cmd_lock);
		return ret;
	}
	*value = 0;

	if (ret == 0)
		dev_err(&cs48l10->spi->dev, "%s() timeout\n", __func__);
	if (ret < 0)
		dev_err(&cs48l10->spi->dev, "%s() error %d\n", __func__, ret);
	if (ret > 0 && cmd->state)
		dev_err(&cs48l10->spi->dev, "%s() cmd dropped\n", __func__);

	list_del(&cmd->link);
	kfree(cmd);

	mutex_unlock(&cs48l10->cmd_lock);

	return ret;
}

/* read/write by module */
#define dsp_write(cs48l10, mod, index, value) \
	dsp_write_cmd(cs48l10, MK_OPWR(mod), index,  value);
#define dsp_read(cs48l10, mod, index, value) \
	dsp_read_cmd_async(cs48l10, MK_OPRD(mod), index, 0x7FFFFFFF, value);

/**
 * dsp_wait_unsol_async - Unsolicited wait from the DSP
 *
 * @cs48l10: the dsp device
 * @unsol_prefix: unsol prefix to match
 * @mask: match mask
 * @unsol: unsol response
 * @value: the value in the unsol response
 *
 * Returns nonzero if successful, or a negative error code on failure.
 * Note: The deffered work will match each response from the DSP with
 * the queued prefix and mask. On success it will dequeue the waiting
 * command and wake the waiting call. Make sure we don't create a filter
 * to match a pending command // (response & mask) = command.
 */
int dsp_wait_unsol_async(struct cs48l10 *cs48l10,
	    dsp_word_t unsol_prefix, dsp_word_t mask,
	    dsp_word_t *unsol_response, dsp_word_t *unsol_value, int timeout)
{
	int ret;
	struct dsp_cmd *cmd;

	dev_dbg(&cs48l10->spi->dev, "%s() enter\n", __func__);

	/* check if we read something ... */
	mutex_lock(&cs48l10->cmd_lock);
	if (cs48l10->has_cached_unsol) {
		if ((cs48l10->cached_unsol.command & mask) ==
			(unsol_prefix & mask)) {
			*unsol_response = cs48l10->cached_unsol.command;
			*unsol_value = cs48l10->cached_unsol.value;
			cs48l10->has_cached_unsol = 0;
			mutex_unlock(&cs48l10->cmd_lock);
			return 1;
		}
	}

	mutex_unlock(&cs48l10->cmd_lock);

	cmd = kzalloc(sizeof(struct dsp_cmd), GFP_KERNEL);
	if (!cmd)
		return -ENOMEM;

	cmd->command = unsol_prefix;
	cmd->mask = mask;
	cmd->value = 0;
	init_completion(&cmd->cmd_wait);
	dsp_queue_cmd(cs48l10, cmd);

	/* wait for for dispatch */
	ret = wait_for_completion_interruptible_timeout(&cmd->cmd_wait,
		timeout);
	mutex_lock(&cs48l10->cmd_lock);
	if (ret > 0 && cmd->state == 1) {
		/* dispatched command are removed from the list */
		*unsol_response = cmd->command;
		*unsol_value = cmd->value;

		dev_dbg(&cs48l10->spi->dev, "%s() unsol result %08x, %08x\n",
			__func__, cmd->command, cmd->value);
		kfree(cmd);
		mutex_unlock(&cs48l10->cmd_lock);
		return ret;
	}
	*unsol_response = 0;
	*unsol_value = 0;

	if (ret == 0)
		dev_err(&cs48l10->spi->dev, "%s() timeout\n", __func__);
	if (ret > 0 && cmd->state == 0) {
		dev_err(&cs48l10->spi->dev, "%s() cmd dropped\n", __func__);
		ret = -EINTR;
	}

	/* timeout or error */
	list_del(&cmd->link);
	kfree(cmd);
	mutex_unlock(&cs48l10->cmd_lock);

	dev_dbg(&cs48l10->spi->dev, "%s() exit\n", __func__);

	return ret;
}


/**
 * dsp_write_audio_buf - Write audio chunk to the DSP
 *
 * @cs48l10: the dsp device
 * @buf: Audio chunk. The first byte must be CS48L10_SPI_WRITE_CMD.
 * @len: The size of the chunk + 1
 *
 * Returns nonzero if successful, or a negative error code on failure.
 * Note: OMAP3 McSPI will use DMA for all buffers with size > 8
 */
static int dsp_write_audio_buf(struct cs48l10 *cs48l10, u8 *buf, int len)
{
	int ret;

	if (cs48l10_busy(cs48l10)) {
		dev_err(&cs48l10->spi->dev,
			": %s()  failed: BUSY not going high\n",
			__func__);

		return -EBUSY;
	}

	ret = spi_write_dma(cs48l10->spi, buf, len);
	if (ret < 0)
		dev_err(&cs48l10->spi->dev,
			": %s() write %d bytes failed: ret = %d\n",
			__func__, len, ret);

	return ret;
}

/**
 * dsp_event_watermark - action upon watermark event signaled from the DSP
 *
 * @cs48l10: the dsp device
 * @value: the value with the unsol message (free space in the DSP buffer)
 * @count: the size of the chunk to be transferred (bytes)
 *
 * Returns nonzero if successful, or a negative error code on failure.
 * Note: count must be aligned to the current packet unit size
 */
int dsp_event_watermark(struct cs48l10 *cs48l10, dsp_word_t value, size_t count)
{
	int ret;
	size_t dsp_word_count = count >> 2;

	dev_dbg(&cs48l10->spi->dev, "%s() enter\n", __func__);

	if (value > DSP_BUFFER_SIZE) {
		dev_err(&cs48l10->spi->dev,
		    "Bad low watermark value - (%08x)\n", value);
		return -EIO;
	}

	if (value < dsp_word_count) {
		dev_err(&cs48l10->spi->dev,
			"Requested %08x words, but only %08x available\n",
			dsp_word_count, value);
		return 0;
	}

	/* Switch to DATA mode - OP_DREQ @ index 1 */
	ret = dsp_write_cmd(cs48l10, OP_DREQ, 1, dsp_word_count);
	if (ret < 0)
		dev_err(&cs48l10->spi->dev, "Switch to DATA mode failed\n");

	dev_dbg(&cs48l10->spi->dev, "%s() exit\n", __func__);

	return ret;
}

/**
 * dsp_event_data - acton upon switched to data mode signaled from the DSP
 *
 * @cs48l10: the dsp device
 * @value: the value with the unsol message (free space in the DSP buffer)
 * @buf - data from user space
 * @count - size (in bytes) of the data in the buffer
 *
 * Returns nonzero if successful, or a negative error code on failure.
 * Note:
 */
int dsp_event_data(struct cs48l10 *cs48l10, dsp_word_t value,
	    const char __user *buf, size_t count)
{
	int ret;
	size_t dsp_word_count = count >> 2;
	/*
	    Check the DSP free space again with the requested
	    audio data size and the DSP buffer size
	*/
	if (value >= dsp_word_count && value <= DSP_BUFFER_SIZE) {
		/* Get Audio data from user */
		if (copy_from_user(&cs48l10->buffer[1], buf, count))
			return -EFAULT;

		cs48l10->buffer[0] = CS48L10_SPI_WRITE_CMD;

		/* Write audio data to DSP */
		ret = dsp_write_audio_buf(cs48l10, cs48l10->buffer, count + 1);
		if (ret < 0)
			return ret;
	}
	return count;
}

static void dsp_configure_for_mono(struct cs48l10 *cs48l10)
{
	dsp_write(cs48l10, DSP_AM, 0x12, 0x1);
	dsp_write(cs48l10, DSP_AM, 0x13, 0x1);
	dsp_write(cs48l10, DSP_AM, 0x14, 0x1);
	dsp_write(cs48l10, DSP_AM, 0x15, 0x1);
}

static void dsp_configure_for_underrun_glitch(struct cs48l10 *cs48l10)
{
	dsp_write(cs48l10, DSP_OS, 0xD, 0x50);
}

void dsp_configure_watermark_level(struct cs48l10 *cs48l10, u32 bitrate)
{
	int wm_level = 0x800;

#if 0 /* Hard code watermark level to 0x800 for now */
	int data_buf_size;
	int ret;

	ret = dsp_read(cs48l10, DSP_OS, SPI_DBS, &data_buf_size);
	if (ret < 0)
		return;
	/* setup watermark level to half of buffer size */
	data_buf_size = ((data_buf_size >> 16) +   /* XMEM dsp words */
			    (data_buf_size & 0xFFFF))  ; /* YMEM dsp words */

	wm_level = data_buf_size >> 1; /* divide by 2 */
#endif

	dsp_write(cs48l10, DSP_OS, SPI_DBWL, wm_level);
}

/* END DSP */

/* AUDIO DECODER*/
/**
 * audio_decoder_init - Initialize a dsp audio decoder
 *
 * @ad: the dsp audio decoder
 * @fmt: the stream format
 *
 * Returns nonnegative number if successful, or a negative error code on
 * failure.
 * Note: each audio decoder has a different init sequence.
 */
int audio_decoder_init(struct dsp_audio_decoder *ad,
	struct cs48l10_audio_fmt *fmt)
{
	if (ad && ad->ops && ad->ops->init)
		return (ad->ops->init)(ad, fmt);

	return -ENODEV;
}

/**
 * audio_decoder_resume - Start/enable a dsp audio decoder
 *
 * @ad: the dsp audio decoder
 *
 * Returns nonnegative number if successful, or a negative error code on
 * failure.
 * Note: IOCTL_RESUME
 */
int audio_decoder_resume(struct dsp_audio_decoder *ad)
{
	int ret;
	struct cs48l10 *cs48l10 =
		container_of(ad, struct cs48l10, decoder);

	if (ad && ad->ops && ad->ops->resume) {
		ret = (ad->ops->resume)(ad);
		if (ret >= 0)
			cs48l10->dsp_state = DSP_STATE_PLAY;
		return ret;
	}
	return -ENODEV;
}


/**
 * audio_decoder_pause - Pause a dsp audio decoder
 *
 * @ad: the dsp audio decoder
 *
 * Returns nonnegative number if successful, or a negative error code on
 * failure.
 * Note: IOCTL_PAUSE
 */
int audio_decoder_pause(struct dsp_audio_decoder *ad)
{
	int ret;
	struct cs48l10 *cs48l10 =
		container_of(ad, struct cs48l10, decoder);

	if (ad && ad->ops && ad->ops->pause) {
		ret = (ad->ops->pause)(ad);
		if (ret >= 0)
			cs48l10->dsp_state = DSP_STATE_PAUSE;
		return ret;
	}
	return -ENODEV;
}

/**
 * audio_decoder_stop - Stop/disable a dsp audio decoder
 *
 * @ad: the dsp audio decoder
 *
 * Returns nonnegative number if successful, or a negative error code on
 * failure.
 * Note: IOCTL_STOP
 */
int audio_decoder_stop(struct dsp_audio_decoder *ad)
{
	int ret;
	struct cs48l10 *cs48l10 =
		container_of(ad, struct cs48l10, decoder);

	if (ad && ad->ops && ad->ops->stop) {
		ret =  (ad->ops->stop)(ad);
		if (ret >= 0)
			cs48l10->dsp_state = DSP_STATE_STOP;
		return ret;
	}
	return -ENODEV;
}

/**
 * audio_decoder_get_decoded_frames - Return the decoded uncompressed frames
 *	by a dsp audio decoder
 *
 * @ad: the dsp audio decoder
 * @frames: the decoded frames
 *
 * Returns nonnegative number if successful, or a negative error code on
 * failure.
 * Note: IOCTL_RESUME
 */
int audio_decoder_get_decoded_frames(struct dsp_audio_decoder *ad, u32 *frames)
{
	if (ad && ad->ops && ad->ops->get_decoded_frames)
		return (ad->ops->get_decoded_frames)(ad, frames);
	return -ENODEV;
}

/* MPEG audio decoder ops */
int ad_mpeg_init(struct dsp_audio_decoder *ad, struct cs48l10_audio_fmt *fmt)
{
	memcpy(&ad->fmt, fmt, sizeof(struct cs48l10_audio_fmt));
	return 0;
}

int ad_mpeg_resume(struct dsp_audio_decoder *ad)
{
	struct cs48l10 *cs48l10 =
		container_of(ad, struct cs48l10, decoder);

	return dsp_write(cs48l10, ad->dsp_module_id, MP3_CONTROL, 0x21);
}

int ad_mpeg_pause(struct dsp_audio_decoder *ad)
{
	struct cs48l10 *cs48l10 =
		container_of(ad, struct cs48l10, decoder);

	return dsp_write(cs48l10, ad->dsp_module_id, MP3_CONTROL, 0x23);
}

int ad_mpeg_stop(struct dsp_audio_decoder *ad)
{
	struct cs48l10 *cs48l10 =
		container_of(ad, struct cs48l10, decoder);

	return dsp_write(cs48l10, ad->dsp_module_id, MP3_CONTROL, 0);
}

int ad_mpeg_get_decoded_frames(struct dsp_audio_decoder *ad, u32 *frames)
{
	struct cs48l10 *cs48l10 =
		container_of(ad, struct cs48l10, decoder);

	return dsp_read(cs48l10, ad->dsp_module_id, MP3_DECODED_FRAMES, frames);
}

static struct dsp_audio_decoder_ops mpeg_decoder_ops = {
	.init = ad_mpeg_init,
	.resume = ad_mpeg_resume,
	.pause = ad_mpeg_pause,
	.stop = ad_mpeg_stop,
	.get_decoded_frames = ad_mpeg_get_decoded_frames,
};

static struct dsp_audio_decoder mpeg_audio_decoder = {
	.dsp_module_id = DSP_MP3,
	.ops = &mpeg_decoder_ops,
};

/* AAC audio decoder ops */
#define AAC_CONTROL_AAC_ENABLE	(1 << 0)
#define AAC_CONTROL_IEC_ENABLE	(1 << 1)
#define AAC_CONTROL_PAUSE	(1 << 3)
#define AAC_CONTROL_RAW_FMT	(1 << 5)
#define AAC_CONTROL_TNS_ENABLE	(1 << 8)
#define AAC_CONTROL_AAC_DOWNMIX	(1 << 12)

int ad_aac_init(struct dsp_audio_decoder *ad, struct cs48l10_audio_fmt *fmt)
{
	struct cs48l10 *cs48l10 =
		container_of(ad, struct cs48l10, decoder);
	int fs_code = 4; /* 44100 */

	if (fmt->sample_rate == 48000)
		fs_code = 3;
	else if (fmt->sample_rate == 32000)
		fs_code = 5;

	dsp_write(cs48l10, ad->dsp_module_id, AAC_FS_CODE, fs_code);
	dsp_write(cs48l10, ad->dsp_module_id, AAC_CH_CONFIG, fmt->channels);

	/*
	    AAC_CONTROL
		:12 	- Downmix Select (1-AAC, 0-Standard)
		:8	- TNS Enable (0-enable, 1-disable)
		:5	- ADTS/RAW format (0-ADTS, 1-RAW)
		:3	- PAUSE (1-enable, 0-disable)
		:1	- IEC Enable (1-enable, 0-disable)
		:0	- AAC Enable (1-enable, 0-disable)
	*/
	memcpy(&ad->fmt, fmt, sizeof(struct cs48l10_audio_fmt));

	return 0;
}

int ad_aac_resume(struct dsp_audio_decoder *ad)
{
	struct cs48l10 *cs48l10 =
		container_of(ad, struct cs48l10, decoder);

	/* TNS disabled, RAW format, AAC Enabled, PAUSE unset */
	return dsp_write(cs48l10, ad->dsp_module_id, AAC_CONTROL,
		AAC_CONTROL_AAC_ENABLE | AAC_CONTROL_RAW_FMT);
}

int ad_aac_pause(struct dsp_audio_decoder *ad)
{
	struct cs48l10 *cs48l10 =
		container_of(ad, struct cs48l10, decoder);

	/* cache AAC_CONTOL bits in the priv and update only the PAUSE bit */
	/* TNS disabled, RAW format, AAC Enabled, PAUSE set */
	return dsp_write(cs48l10, ad->dsp_module_id, AAC_CONTROL,
		AAC_CONTROL_AAC_ENABLE | AAC_CONTROL_RAW_FMT |
		AAC_CONTROL_PAUSE);
}

int ad_aac_stop(struct dsp_audio_decoder *ad)
{
	struct cs48l10 *cs48l10 =
		container_of(ad, struct cs48l10, decoder);

	return dsp_write(cs48l10, ad->dsp_module_id, AAC_CONTROL, 0);
}

int ad_aac_get_decoded_frames(struct dsp_audio_decoder *ad, u32 *frames)
{
	struct cs48l10 *cs48l10 =
		container_of(ad, struct cs48l10, decoder);

	return dsp_read(cs48l10, ad->dsp_module_id, AAC_DECODED_FRAMES, frames);
}

static struct dsp_audio_decoder_ops aac_decoder_ops = {
	.init = ad_aac_init,
	.resume = ad_aac_resume,
	.pause = ad_aac_pause,
	.stop = ad_aac_stop,
	.get_decoded_frames = ad_aac_get_decoded_frames,
};

static struct dsp_audio_decoder aac_audio_decoder = {
	.dsp_module_id = DSP_AAC,
	.ops = &aac_decoder_ops,
};

/* AAC+ audio decoder ops */
/* TODO: reuse the aac ops */
#define HEAAC_CONTROL_HEAAC_ENABLE	(1 << 0)
#define HEAAC_CONTROL_IEC_ENABLE	(1 << 1)
#define HEAAC_CONTROL_SBR_ENABLE	(1 << 2) /* SBR - HE_AAC_V1 */
#define HEAAC_CONTROL_PAUSE		(1 << 3)
#define HEAAC_CONTROL_PS_ENABLE		(1 << 4) /* PS - HE_AAC_V2 */
#define HEAAC_CONTROL_RAW_FMT		(1 << 5)
#define HEAAC_CONTROL_DEFAULT		\
	(HEAAC_CONTROL_HEAAC_ENABLE | HEAAC_CONTROL_SBR_ENABLE |\
	HEAAC_CONTROL_PS_ENABLE | HEAAC_CONTROL_RAW_FMT)

int ad_aacp_init(struct dsp_audio_decoder *ad, struct cs48l10_audio_fmt *fmt)
{
	struct cs48l10 *cs48l10 =
		container_of(ad, struct cs48l10, decoder);
	int fs_code = 7; /* 44100 */
	int half_sampling_freq = fmt->sample_rate / 2;

	switch (half_sampling_freq) {
	case 24000:
		fs_code = 6;
		break;
	case 22050:
		fs_code = 7;
		break;
	case 16000:
		fs_code = 8;
		break;
	case 12000:
		fs_code = 9;
		break;
	case 11025:
		fs_code = 10;
		break;
	case 8000:
		fs_code = 11;
		break;
	default:
		fs_code = 7;
		break;
	}

	dsp_write(cs48l10, ad->dsp_module_id, AAC_FS_CODE, fs_code);
	dsp_write(cs48l10, ad->dsp_module_id, AAC_CH_CONFIG, fmt->channels);

	/*
	    HE_AAC_CONTROL
		:5	- ADTS/RAW format (0-ADTS, 1-RAW)
		:4	- PS_Enable = 0/1 =
			    Disable/Enable Parametric Stereo processing
		:3	- PAUSE (1-enable, 0-disable)
		:2	- SBR_Processing_Enable = 0/1
			    Disable/Enable SBR Processing
			    (or upsampling if SBR is not present)
		:1	- IEC Enable (1-enable, 0-disable)
		:0	- HE_AAC Enable (1-enable, 0-disable)
	*/
	memcpy(&ad->fmt, fmt, sizeof(struct cs48l10_audio_fmt));

	return 0;
}

int ad_aacp_resume(struct dsp_audio_decoder *ad)
{
	struct cs48l10 *cs48l10 =
		container_of(ad, struct cs48l10, decoder);

	/* RAW format, HE_AAC Enabled, PAUSE unset */
	return dsp_write(cs48l10, ad->dsp_module_id, AAC_CONTROL,
		HEAAC_CONTROL_DEFAULT);
}

int ad_aacp_pause(struct dsp_audio_decoder *ad)
{
	struct cs48l10 *cs48l10 =
		container_of(ad, struct cs48l10, decoder);

	/* cache AAC_CONTOL bits in the priv and update only the PAUSE bit */
	/* RAW format, HE_AAC Enabled, PAUSE set */
	return dsp_write(cs48l10, ad->dsp_module_id, AAC_CONTROL,
		HEAAC_CONTROL_DEFAULT | HEAAC_CONTROL_PAUSE);
}

int ad_aacp_stop(struct dsp_audio_decoder *ad)
{
	struct cs48l10 *cs48l10 =
		container_of(ad, struct cs48l10, decoder);

	return dsp_write(cs48l10, ad->dsp_module_id, AAC_CONTROL, 0);
}

int ad_aacp_get_decoded_frames(struct dsp_audio_decoder *ad, u32 *frames)
{
	struct cs48l10 *cs48l10 =
		container_of(ad, struct cs48l10, decoder);

	return dsp_read(cs48l10, ad->dsp_module_id, AAC_DECODED_FRAMES, frames);
}

static struct dsp_audio_decoder_ops aacp_decoder_ops = {
	.init = ad_aacp_init,
	.resume = ad_aacp_resume,
	.pause = ad_aacp_pause,
	.stop = ad_aacp_stop,
	.get_decoded_frames = ad_aacp_get_decoded_frames,
};


static struct dsp_audio_decoder aacp_audio_decoder = {
	.dsp_module_id = DSP_HE_AAC_V2,
	.ops = &aacp_decoder_ops,
};


/* HE_AAC audio decoder */

/* END AUDIO DECODER */

#ifdef CS48L10_CDEV
/*
    Audio DSP CDEV interface.
    This version of the firmware has only a basic IO capablities.
    This allows us to export a very simple cdev interface.

	open() - enable & power codec, dsp init
	write() - send a chuck of compressed data to the DSP
	close() - disable & power down codec,

    TODO: Make me ASoC compressed audio driver
*/

static struct class *cs48l10_class; /* register for hotpug events  */
static atomic_t cs48l10_available = ATOMIC_INIT(1); /* single-open */

/* CDEV */
/* Supported in the firmware formats */
struct dsp_format {
	u32 format;			/* format id */
	const char *format_short_name; /* format name - mp3, aac, aac+, wma  */
	u32 stream_module_id;	/* DSP render module */
	struct dsp_audio_decoder *decoder;
};

static struct dsp_format dsp_formats2[] =
{
	{CS48L10_FORMAT_MP3, "mp3", DSP_MP3, &mpeg_audio_decoder},
	{CS48L10_FORMAT_AAC, "aac", DSP_AAC, &aac_audio_decoder},
	/* HE-AAC-V1/2 are both decoded by the "aac+" decoder */
	{CS48L10_FORMAT_HE_AAC_V1, "aac+", DSP_HE_AAC_V1, &aacp_audio_decoder},
	{CS48L10_FORMAT_HE_AAC_V2, "aac+", DSP_HE_AAC_V2, &aacp_audio_decoder},
	{CS48L10_FORMAT_WMA, "wma", DSP_WMA, NULL},
};

static unsigned int  curr_time_ms, prev_time_ms;
static unsigned int curr_tick, prev_tick;

/* IOCTL */
/**
 * cs48l10_ioctl_change_project - boot thew project from the master list
 *
 * @cs48l10: the dsp device
 * @project_id: new project
 *
 * Returns zero if successful, or a negative error code on failure.
 * Note:
 */
int cs48l10_ioctl_change_project(struct cs48l10 *cs48l10,
					unsigned long project_id, int force)

{
	struct cs48l10_firmware *fw = cs48l10->fw;
	int ret;

	if (!fw || !fw->pmaster_list)
		return -EINVAL;

	if (project_id == fw->active_project && !force) {
		dev_info(&cs48l10->spi->dev,
			"Project %lu is already running\n",
			project_id);
		return project_id;
	}

	if (project_id > 0xFF) {
		dev_err(&cs48l10->spi->dev,
			"Invalid project_id %lu\n", project_id);
		return -EINVAL;
	}

	if (project_id >= cpu_to_be32(fw->pmaster_list->num_projects)) {
		dev_err(&cs48l10->spi->dev,
			 "Project %lu is not in projects master list\n",
			 project_id);
		return -EINVAL;
	}

	/*
	   Boot the new project. Softboot maybe ?
	 */
	ret = cs48l10_fw_boot_project(cs48l10, project_id);

	if (ret < 0) {
		dev_err(&cs48l10->spi->dev,
			"Project %lu failed to start, ret = %d\n",
			project_id, ret);
		return ret;
	}

	return project_id;
}

/**
 * cs48l10_update_packet_timeout - Update the time for waiting unsol
 * 	responses.
 *
 * @cs48l10: the dsp device
 * @bitrate: average bitrate as reported by the user level
 *
 * Returns zero if successful, or a negative error code on failure.
 * Note: This is approx. It's valid for CBR streams, but we have to
 *    take care for VBR streams too.
 */
int cs48l10_update_packet_timeout(struct cs48l10 *cs48l10, u32 bitrate)
{
	int timeout;

	if (bitrate == 0)
		return -EINVAL;

	timeout = (CS48L10_BUFFER_SIZE * HZ) / (bitrate / 8);
	timeout += (timeout / 3); /* give it a third more */

	cs48l10->unsol_timeout = timeout;

	dev_dbg(&cs48l10->spi->dev, "Unsol timeout %u ~ %u sec\n",
		timeout, timeout / HZ);

	return 0;
}

/* cs48l10_select_project_id_by_name - select project id from the display table
 *
 * @cs48l10: the dsp device
 * @format: format id
 * @sample_rate: sample rate
 *
 * Returns project_id if successful, or a negative error code if the display
 * list does not contain a project with a DECODER_FS name, matching the
 * requested format and sample rate.
 * Note:
 */
int cs48l10_select_project_id_by_name(struct cs48l10 *cs48l10,
	    u32 format, u32 sample_rate)
{
	struct cs48l10_firmware *fw = cs48l10->fw;
	int i, fmt_idx = -1;
	char project_name[MAX_DISPLAY_LENGTH + 1]; /* 16 chars */
	u32 project_id = -EINVAL, num_projects;
	project_display_t *pproject_name;
	struct dsp_audio_decoder *decoder;

	if (!fw || !fw->pmaster_list || !fw->pdisplay_list)
		return -EINVAL;

	/* check if format is supported */
	for (i = 0; i < ARRAY_SIZE(dsp_formats2); i++) {
		if (dsp_formats2[i].format == format) {
			fmt_idx = i;
			break;
		}
	}

	if (fmt_idx == -1)
		return -EINVAL; /* unsupported format */

	/* Format project name - DECODER_FS,
	   e.g. 'mp3_44k1'
	*/
	snprintf(project_name, sizeof(project_name), "%s_%uk%u",
		dsp_formats2[fmt_idx].format_short_name,
		sample_rate / 1000, (sample_rate % 1000) / 100);

	/* check if we have a decoder for this sample rate */

	/* Project num in master nad display list must be equal !
	    If not equal, try to use the smaller number, but
	    fw image is broken or not parsed correctly
	*/
	num_projects = cpu_to_be32(fw->pmaster_list->num_projects);
	if (num_projects > cpu_to_be32(fw->pdisplay_list->num_projects)) {
		num_projects = cpu_to_be32(fw->pdisplay_list->num_projects);
		dev_err(&cs48l10->spi->dev,
			"project number in master and display list "\
			"is different!\n");
	}

	if (num_projects == 0)
		return -EINVAL;

	for (i = 0; i < num_projects; i++) {
		pproject_name = (project_display_t *)((u8 *) fw->project_names +
			cpu_to_be32(fw->pdisplay_list->row_size) * i);
		/* validate ? */
		if ((strncmp(project_name, pproject_name->project_name,
			sizeof(project_name))) == 0) {
			project_id = i;
			break;
		}
	}
	/* Setup decoder ops */
	decoder = dsp_formats2[fmt_idx].decoder;
	if (decoder == NULL) {
		cs48l10->decoder.ops = NULL; /* can't control the stream */
	} else {
		cs48l10->decoder.ops = decoder->ops;
		cs48l10->decoder.dsp_module_id = decoder->dsp_module_id;
	}

	return project_id;
}

/**
 * cs48l10_ioctl_set_fmt - Set new audio format
 *
 * @cs48l10: the dsp device
 * @fmt: new audio format
 * @force: force the changes
 *
 * Returns zero if successful, or a negative error code on failure.
 * Note:
 */
int cs48l10_ioctl_set_fmt(struct cs48l10 *cs48l10,
	struct cs48l10_audio_fmt *fmt, int force)
{
	struct cs48l10_audio_fmt *dsp_fmt = &cs48l10->dsp_fmt;
	int ret, project_id = -1;
	u32 format = fmt->format & CS48L10_FORMAT_MAIN_MASK;

	if (dsp_fmt->format == fmt->format &&
		dsp_fmt->sample_rate == fmt->sample_rate &&
		dsp_fmt->channels == fmt->channels && !force) {
		/* same format, rate and channels  only bitrate has changed */
		if (dsp_fmt->bitrate != fmt->bitrate) {
			ret = cs48l10_update_packet_timeout(cs48l10,
					fmt->bitrate);
			dsp_fmt->bitrate = fmt->bitrate;
			return ret;
		}
	}

	/*
	    Sample rate or format has changed.
	    Check if the new one is suppoted and switch
	    the DSP projects
	*/
	project_id =
		cs48l10_select_project_id_by_name(cs48l10,
			format, fmt->sample_rate);

	if (project_id < 0)
		return -EINVAL;

	dev_info(&cs48l10->spi->dev,
		"Set new DSP format %08x, sample rate %u Hz, channels %u, " \
			"bitrate %u bps\n",
		fmt->format, fmt->sample_rate, fmt->channels, fmt->bitrate);

	ret = cs48l10_update_packet_timeout(cs48l10, fmt->bitrate);
	dsp_fmt->bitrate = fmt->bitrate;
	if (ret < 0) {
		dev_info(&cs48l10->spi->dev,
			"Bitrate was not specified. Using default timeouts\n");
		cs48l10_update_packet_timeout(cs48l10, 32000);
		dsp_fmt->bitrate = 32000;
	}

	dsp_fmt->format = fmt->format;
	dsp_fmt->sample_rate = fmt->sample_rate;
	dsp_fmt->channels = fmt->channels;

	ret = cs48l10_ioctl_change_project(cs48l10, project_id, force);
	if (ret < 0)
		return ret;
	dsp_configure_watermark_level(cs48l10, fmt->bitrate);
	/* decoder will be started by ioctl() RESUME ! */
	return audio_decoder_init(&cs48l10->decoder, dsp_fmt);
}

/**
 * cs48l10_ioctl_get_hostif_config - Get the host interface config
 *
 * @cs48l10: the dsp device
 * @config: host interface config
 *
 * Returns zero if successful, or a negative error code on failure.
 * Note: Valid after KICKSTART
 */
int cs48l10_ioctl_get_hostif_config(struct cs48l10 *cs48l10,
	struct cs48l10_hostif_config *config)
{
	int ret;

	ret = dsp_read(cs48l10, DSP_OS, SPI_DPUS, &config->packet_unit_size);
	if (ret < 0)
		return ret;

	ret = dsp_read(cs48l10, DSP_OS, SPI_DBS, &config->data_buf_size);
	if (ret < 0)
		return ret;

	ret = dsp_read(cs48l10, DSP_OS, SPI_DBWL, &config->watermark_level);
	if (ret < 0)
		return ret;

	return 0;
}

/**
 * cs48l10_ioctl_set_hostif_config - Set new host interface config
 *
 * @cs48l10: the dsp device
 * @config: host interface config
 *
 * Returns zero if successful, or a negative error code on failure.
 * Note: we are using this after project is loaded (KICKSTART),
 * so we cant set the data buffer size here
 */
int cs48l10_ioctl_set_hostif_config(struct cs48l10 *cs48l10,
	struct cs48l10_hostif_config *config)
{
	int ret;

	ret = dsp_write(cs48l10, DSP_OS, SPI_DPUS, config->packet_unit_size);
	if (ret < 0)
		return ret;

	/* data buffer size must be set before KICKSTART */
	ret = dsp_write(cs48l10, DSP_OS, SPI_DBWL, config->watermark_level);
	if (ret < 0)
		return ret;

	return 0;
}


/*
	cs48l10_cdev_ioctl()
*/
ssize_t cs48l10_cdev_ioctl(struct inode *inode, struct file *filp,
	unsigned int ioctl, unsigned long arg)
{
	int ret = 0;
	struct cs48l10 *cs48l10 = (struct cs48l10 *) filp->private_data;
	struct dsp_audio_decoder *decoder = &cs48l10->decoder;
	void __user *argp = (void __user *)arg;
	struct cs48l10_audio_fmt fmt;
	struct cs48l10_hostif_config hostif_config;
	dsp_word_t value;
	int cmd_delay;

	mutex_lock(&cs48l10->lock);

	switch (ioctl) {
	case CS48L10_PAUSE: /* DSP PAUSE */
		wake_lock(&cs48l10->gpio_int_wake_wakelock);
		dev_info(&cs48l10->spi->dev, "ioctl() CS48L10_PAUSE\n");
		ret = audio_decoder_pause(decoder);
		/* SEND MUTE COMMAND */
		ret = dsp_write(cs48l10, DSP_AM, AM_MUTE, 1);
		break;

	case CS48L10_STOP: /* DSP STOP */
		if (arg == 0) {
			dev_info(&cs48l10->spi->dev, "ioctl() CS48L10_STOP\n");
			/* SEND MUTE COMMAND*/
			ret = dsp_write(cs48l10, DSP_AM, AM_MUTE, 1);
			mdelay(25);
			ret = audio_decoder_stop(decoder);
			/* wait for stop to complete */
			switch (cs48l10->dsp_fmt.format) {
			case CS48L10_FORMAT_MP3:
				cmd_delay = 24;
				break;
			case CS48L10_FORMAT_HE_AAC_V1:
			case CS48L10_FORMAT_HE_AAC_V2:
			case CS48L10_FORMAT_WMA:
				cmd_delay = 2048000 / 48000;
				break;
			case CS48L10_FORMAT_AAC:
			default:
				cmd_delay = 1024000 / 48000;
			}
			mdelay(cmd_delay+5);
			dsp_clear_queue(cs48l10);
		} else if (arg == 1) {
			dev_info(&cs48l10->spi->dev,
				"ioctl() CS48L10_STOP DEAD\n");
			/* SEND MUTE COMMAND*/
			ret = dsp_write(cs48l10, DSP_AM, AM_MUTE, 1);
			mdelay(20);
			dsp_clear_queue(cs48l10);
		}
		break;

	case CS48L10_RESUME: /* DSP RESUME */
		dev_info(&cs48l10->spi->dev, "ioctl() CS48L10_RESUME\n");
		ret = audio_decoder_resume(decoder);
		/* SEND UNMUTE COMMAND*/
		ret = dsp_write(cs48l10, DSP_AM, AM_MUTE, 0);
		curr_tick = 0;
		prev_tick = 0;
		prev_time_ms = 0;
		break;

	case CS48L10_SET_STREAM_FMT: /* Set new stream format */
		wake_lock(&cs48l10->gpio_int_wake_wakelock);
		dev_info(&cs48l10->spi->dev,
			"ioctl() CS48L10_SET_STREAM_FMT\n");

		dsp_clear_queue(cs48l10);
		ret = -EFAULT;
		if (copy_from_user(&fmt, argp,
				sizeof(struct cs48l10_audio_fmt)))
			break;

		/* force project load. We are in low-power mode after close */
		ret = cs48l10_ioctl_set_fmt(cs48l10, &fmt, 1);
		break;

	case CS48L10_GET_STREAM_FMT: /* get the current stream format */
		dev_info(&cs48l10->spi->dev,
			"ioctl() CS48L10_GET_STREAM_FMT\n");
		ret = 0;
		if (copy_to_user(argp, &cs48l10->dsp_fmt,
				sizeof(struct cs48l10_audio_fmt)))
			ret = -EFAULT;
		break;

	case CS48L10_GET_PROJECT: /* get the current dsp project */
		dev_info(&cs48l10->spi->dev, "ioctl() CS48L10_GET_PROJECT\n");

		ret = -EFAULT;
		if (!cs48l10->fw)
			break;

		ret = cs48l10->fw->active_project;
		break;

	case CS48L10_SET_PROJECT: /* set new dsp project */
		dev_info(&cs48l10->spi->dev, "ioctl() CS48L10_SET_PROJECT\n");
		ret = cs48l10_ioctl_change_project(cs48l10, arg, 0);
		break;

	case CS48L10_SET_MUTE: /* mute: arg = 1, unmute: arg = 0 */
		dev_info(&cs48l10->spi->dev, "ioctl() CS48L10_SET_MUTE\n");
		ret  = -EINVAL;
		if (arg > 1)
			break;
		ret = dsp_write(cs48l10, DSP_AM, AM_MUTE, arg);
		break;

	case CS48L10_SET_VOLUME:
		/* Volume: arg is fixed point value, check AN344 */
		dev_info(&cs48l10->spi->dev, "ioctl() CS48L10_SET_MUTE\n");
		ret = dsp_write(cs48l10, DSP_AM, AM_GAIN, arg);
		break;

	case CS48L10_GET_DECODED_FRAMES:
		dev_info(&cs48l10->spi->dev,
			"ioctl() CS48L10_GET_DECODED_FRAMES\n");
		ret = audio_decoder_get_decoded_frames(decoder, &value);
		if (ret < 0)
			break;

		if (put_user(value,  (int __user *)argp))
			ret = -EFAULT;
		dev_info(&cs48l10->spi->dev, "decoded frames %u\n", value);
		break;

	case CS48L10_GET_HOSTIF_CONFIG:
		dev_info(&cs48l10->spi->dev,
			"ioctl() CS48L10_GET_HOSTIF_CONFIG\n");
		ret = cs48l10_ioctl_get_hostif_config(cs48l10, &hostif_config);
		if (ret < 0)
			break;
		if (copy_to_user(argp, &hostif_config,
				sizeof(struct cs48l10_hostif_config)))
			ret = -EFAULT;
		break;

	case CS48L10_SET_HOSTIF_CONFIG:
		dev_info(&cs48l10->spi->dev,
			"ioctl() CS48L10_SET_HOSTIF_CONFIG\n");
		if (copy_from_user(&hostif_config, argp,
				sizeof(struct cs48l10_hostif_config))) {
			ret = -EFAULT;
			break;
		}
		ret = cs48l10_ioctl_set_hostif_config(cs48l10, &hostif_config);
		break;

	case CS48L10_GET_MUTE: /* requres dsp_read() */
	case CS48L10_GET_VOLUME: /* requres dsp_read() */
		ret = -EINVAL;
		break;

	default:
		break;

	}

	mutex_unlock(&cs48l10->lock);
	return ret;
}

static struct regulator *dsp_regulator;
static int dsp_regulator_state ;/*regulator is off by default*/
static int is_dsp_in_use;
#define DSP_CLK_REQ_DISABLE_TIMEOUT (3*1000) /* 3 sec time out*/

static void disable_dsp_clk_req_work(struct work_struct *work)
{
	audio_clock_request(0);
}
/*
	cs48l10_cdev_open()

		Open CDEV for compressed stream.
		Return -EBUSY if already opened.
		Start compressed stream, power on the codec.

*/
int cs48l10_cdev_open(struct inode *inode, struct file *filp)
{
	struct cs48l10 *cs48l10 =
		container_of(inode->i_cdev, struct cs48l10, cdev);
	int mode = filp->f_flags & O_ACCMODE;
    int ret = 0;

	if (mode == O_RDONLY) {
		if (atomic_read(&cs48l10_available))
			return 0;
		else
			return -EBUSY; /* already opened */
	}

	/* Allow only one instance */
	if (!atomic_dec_and_test(&cs48l10_available)) {
		atomic_inc(&cs48l10_available);
		return -EBUSY; /* already opened */
	}

	mutex_lock(&cs48l10->lock);
	cancel_delayed_work_sync(&cs48l10->dsp_clk_req_disable_work);
	is_dsp_in_use = 1;
	audio_clock_request(1);
	if (dsp_regulator && dsp_regulator_state == 0) {
		ret = regulator_enable(dsp_regulator);
		if (ret < 0) {
			dev_err(&cs48l10->spi->dev,
			"DSP regulator(SW4) enable failed,"
			"ret val = %d\n", ret);
			audio_clock_request(0);
		} else {
		    dsp_regulator_state = 1;
			/* As per spec > 1 us needed after supply
			 * stabilizes and RESET' is high.*/
			udelay(20);
		    dev_info(&cs48l10->spi->dev,
				"DSP regulator(SW4) enable success\n");
			gpio_set_value(cs48l10->gpio_reset, 1);
		}
	}
	filp->private_data = cs48l10;
	cs48l10->rx_data_available = 0;
	/* defaults, query the firmware ! */
	cs48l10->dsp_packet_unit_size = DSP_MAX_PUS * 4;
	/* Max chunk size. One byte is used for spi command prefix */
	cs48l10->dsp_chunk_size =
		(CS48L10_BUFFER_SIZE - cs48l10->dsp_packet_unit_size);

	/* default DSP format */
	cs48l10->dsp_fmt.format = CS48L10_FORMAT_MP3;
	cs48l10->dsp_fmt.sample_rate = 44100;
	cs48l10->dsp_fmt.bitrate = 128000;
	cs48l10->dsp_fmt.channels = 2;
	cs48l10_update_packet_timeout(cs48l10, cs48l10->dsp_fmt.bitrate);
	cs48l10->dsp_mode = DSP_MODE_CMD; /* start in command mode */
	cs48l10->dsp_state = DSP_STATE_LOW_POWER; /* low power */
	cs48l10->dsp_stream_mod_id = DSP_MP3; /* default - MP3 module */
	/* default MP3 audio decoder */
	cs48l10->decoder.ops = mpeg_audio_decoder.ops;
	/* default MP3 audio decoder */
	cs48l10->decoder.dsp_module_id = DSP_MP3;

	mutex_unlock(&cs48l10->lock);

	return ret;
}

static int deferred_wake_unlock;

/*
	cs48l10_cdev_write()
		Write compressed stream in chunks
*/
ssize_t cs48l10_cdev_write(struct file *filp, const char __user *buf,
	size_t count, loff_t *f_pos)
{
	struct cs48l10 *cs48l10 = (struct cs48l10 *) filp->private_data;
	int spi_byte_count, ret = 0;
	dsp_word_t cmd, value, unsol_match;
	struct timeval tv;
	struct timespec now;

	if (count < cs48l10->dsp_packet_unit_size) {
		/* we cannot transfer less than the current packet unit size */
		return 0;
	}

	if (count > cs48l10->dsp_chunk_size)
		spi_byte_count =  cs48l10->dsp_chunk_size;
	else {
		/* dsp_packet_unit_size align */
		spi_byte_count = count;
		spi_byte_count -= (count % cs48l10->dsp_packet_unit_size);
	}
	count = spi_byte_count;
	if (deferred_wake_unlock == 1) {
		wake_unlock(&cs48l10->gpio_int_wake_wakelock);
		deferred_wake_unlock = 0;
	}

	while (1) {
		/*  All unsol responses in CMD mode start with 0x81.
		    Data transfer request confirms delivery with 0x0200xxxx, so
		    we have to switch the fiter for transitional mode :(
		*/
		unsol_match = (cs48l10->dsp_mode == DSP_MODE_CMD_DATA) ?
			  MK_CMD(OP_DRES, 0) : MK_CMD(OP_OS_URES, 0);

		ret = dsp_wait_unsol_async(cs48l10, unsol_match, 0xFF000000,
			&cmd, &value, cs48l10->unsol_timeout);

		/* error or timeout (0) */
		if (ret < 0)
			break;
		if (ret == 0) {
			ret = -ETIMEDOUT;
			break;
		}

		switch (cmd) {
		/* water mark event */
		case MK_CMD(OP_OS_URES, EVENT_WM):
			mutex_lock(&cs48l10->lock);
			ret = dsp_event_watermark(cs48l10, value, count);
			cs48l10->dsp_mode = DSP_MODE_CMD_DATA;
			/* mutex_unlock(&cs48l10->lock);*/
			if (ret < 0)
				goto out;
			break;

		/* switched to data mode event */
		case MK_CMD(OP_DRES, 0):
			/*mutex_lock(&cs48l10->lock); */
			cs48l10->dsp_mode = DSP_MODE_DATA;
			do_gettimeofday(&tv);
			dev_info(&cs48l10->spi->dev,
				"write DMA %d bytes at %d s %d us \n",
				count, (int) tv.tv_sec, (int) tv.tv_usec);
			ret = dsp_event_data(cs48l10, value, buf, count);
			if (ret < 0)
				goto out;
			break;

		/* delivery complete event */
		case MK_CMD(OP_OS_URES, EVENT_DC):
			cs48l10->dsp_mode = DSP_MODE_CMD;
			dev_info(&cs48l10->spi->dev,
				"Delivery complete %x\n", value);
			mutex_unlock(&cs48l10->lock);
			now = CURRENT_TIME;
			curr_time_ms = (now.tv_sec * 1000) +
				(now.tv_nsec / 1000000);
			if ((prev_time_ms < curr_time_ms) &&
					(prev_time_ms != 0))
				curr_tick = curr_time_ms - prev_time_ms;
			if ((curr_tick > WAKELOCK_HOLD_THRESHOLD_MS) &&
					(prev_tick > WAKELOCK_HOLD_THRESHOLD_MS)
					&& (curr_tick != 0) && (prev_tick != 0))
				deferred_wake_unlock = 1;
			else if ((curr_tick == 0) || (prev_tick == 0))
				deferred_wake_unlock = 1;
			prev_tick = curr_tick;
			prev_time_ms = curr_time_ms;

			return count;


		default:
			/* Unexpected command. We shouldn't be here */
			dev_err(&cs48l10->spi->dev,
				"Unexpected DSP command %08x\n", cmd);
			ret = 0;
			goto out;
		}

		if (filp->f_flags & O_NONBLOCK) {
			ret =  -EAGAIN;
			break;
		}

		if (signal_pending(current)) {
			ret = -ERESTARTSYS;
			break;
		}
	}
out:
	/* if still in data mode - unlock */
	if (cs48l10->dsp_mode != DSP_MODE_CMD)
		mutex_unlock(&cs48l10->lock);

	return ret;
}

/*
	cs48l10_cdev_release()

		Close CDEV.
		Stop the compressed stream.
		Power down the codec.

*/
int cs48l10_cdev_release(struct inode *inode, struct file *filp)
{
	struct cs48l10 *cs48l10 = (struct cs48l10 *) filp->private_data;
	int mode = filp->f_flags & O_ACCMODE;

	/* If opened only to check if available, then don't do anything
	   on close */
	if (mode == O_RDONLY)
		return 0;

	mutex_lock(&cs48l10->lock);

	if (cs48l10->dsp_state != DSP_STATE_LOW_POWER) {
		/* decoder stop */
		dsp_write(cs48l10, cs48l10->dsp_stream_mod_id, 0, 0);
		/* set low power mode (hibernate) */
		dsp_write(cs48l10, DSP_OS, SOFTBOOT, 0x10);
		mdelay(5);
		cs48l10->dsp_state = DSP_STATE_LOW_POWER;
	is_dsp_in_use = 0;
	}
	schedule_delayed_work(&cs48l10->dsp_clk_req_disable_work,
		msecs_to_jiffies(DSP_CLK_REQ_DISABLE_TIMEOUT));
	mutex_unlock(&cs48l10->lock);

	wake_up_interruptible(&cs48l10->wait);

	atomic_inc(&cs48l10_available);
	wake_lock_timeout(&cs48l10->gpio_int_wake_wakelock, 5*HZ);
	return 0;
}

/*
    cs48l10_cdev_fsync()
	Wait for the DSP queue to drain.

	Se are caching big chunks of music, so we must wait for the
	DSP queue to drain before signalling EOS (end-of-stream)
*/
int cs48l10_cdev_fsync(struct file *filp, struct dentry *dentry, int datasync)
{
	struct cs48l10 *cs48l10 = (struct cs48l10 *) filp->private_data;
	int ret, retry = 1;
	dsp_word_t cmd, value;
	int data_buf_avail = 0, prev_data_buf_avail = 0;

	mutex_lock(&cs48l10->lock);
	if (cs48l10->dsp_state != DSP_STATE_PLAY) {
		/* We are not in PLAY state. The queue is stopped ! */
		mutex_unlock(&cs48l10->lock);
		return 0;
	}
	mutex_unlock(&cs48l10->lock);

	/* set watermark to very low value */
	dsp_write(cs48l10, DSP_OS, SPI_DBWL, 0x200);

	/* retry once, in case we read another unsol response */
	while (retry--) {
		/* wait for low watermark level unsol*/
		ret = dsp_wait_unsol_async(cs48l10,
			MK_CMD(OP_OS_URES, 0),
			0xFF000000, &cmd, &value, cs48l10->unsol_timeout*2);

		/* error or timeout (0) */
		if (ret < 0)
			return ret;
		if (ret == 0)
			return -ETIMEDOUT;

		/* water mark event */
		if (cmd == MK_CMD(OP_OS_URES, EVENT_WM)) {
			/* Got Low WM from the DSP. Queues are flushed. */
			/* Wait for the data below watermark to be consumed */
			do {
				prev_data_buf_avail = data_buf_avail;
				ret = dsp_read(cs48l10, DSP_OS, SPI_DBSA,
					&data_buf_avail);
				if (ret < 0)
					return 0;
				mdelay(75);
			} while (prev_data_buf_avail != data_buf_avail);


			return 0;
		} else {
		    /* Unexpected unsol response.
			Since we sleep while waiting for unsol, we cannot lock.
			These responses race with each other if the waiters have
			same filter (prefix, mask)
		     */
		    dev_err(&cs48l10->spi->dev,
			"Unexpected DSP unsol response %08x, %08x\n",
			cmd, value);
		}

	}
	return 0; /* retried once, got another unsol, assume success */
}

struct file_operations cs48l10_cdev_fops = {
	.owner =    THIS_MODULE,
	.write =    cs48l10_cdev_write,
	.open =     cs48l10_cdev_open,
	.release =  cs48l10_cdev_release,
	.ioctl =    cs48l10_cdev_ioctl,
	.fsync =    cs48l10_cdev_fsync,
};

static int cs48l10_major = 0;
static int cs48l10_minor = 0;
static int cs48l10_nr_devs = 1;

/*
	cs48l10_cdev_init()
		Initialize CDEV
*/
static int cs48l10_cdev_init(struct cs48l10* cs48l10)
{
	dev_t dev = 0;
        int ret;

	if (cs48l10_major) {
		dev = MKDEV(cs48l10_major, cs48l10_minor);
		ret = register_chrdev_region(dev, cs48l10_nr_devs, "csdsp");
	} else {
		ret = alloc_chrdev_region(&dev, cs48l10_minor, cs48l10_nr_devs,
			"csdsp");
		cs48l10_major = MAJOR(dev);
	}

	if (ret < 0) {
		dev_err(&cs48l10->spi->dev, "Can't allocate major %d\n",
			cs48l10_major);
		return ret;
	}

	cdev_init(&cs48l10->cdev, &cs48l10_cdev_fops);
	cs48l10->cdev.owner = THIS_MODULE;
	cs48l10->cdev.ops = &cs48l10_cdev_fops;
	ret = cdev_add(&cs48l10->cdev, dev, 1);

	if (ret) {
		dev_err(&cs48l10->spi->dev,
			"Failed to initialize cs48l10 cdev\n");
		return -ENODEV;
	}

	cs48l10_class = class_create(THIS_MODULE, "cs48l10");
	if (IS_ERR(cs48l10_class)) {
		cdev_del(&cs48l10->cdev);

		return -ENODEV;
	}
	device_create(cs48l10_class, NULL, MKDEV(cs48l10_major, cs48l10_minor),
		NULL,  "csdsp");

	init_waitqueue_head(&cs48l10->wait);

	dev_info(&cs48l10->spi->dev,
		"Successufully initialized /dev/csdsp major = %d\n",
		cs48l10_major);

	return 0;
}
#endif
/* END CDEV */

/* sysfs entries */
static ssize_t cs48l10_project_id_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct cs48l10 *cs48l10 = dev_get_drvdata(dev);
	struct cs48l10_firmware *fw = cs48l10->fw;
	unsigned int project_id = 0;

	mutex_lock(&cs48l10->lock);

	if (fw)
		project_id = fw->current_project;

	mutex_unlock(&cs48l10->lock);

	return sprintf(buf, "%u\n", project_id);
}

static ssize_t cs48l10_project_id_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct cs48l10 *cs48l10 = dev_get_drvdata(dev);
	struct cs48l10_firmware *fw = cs48l10->fw;
	unsigned long project_id = 0;
	int ret;

	if (!fw || !fw->pmaster_list)
		return -EINVAL;

	ret = strict_strtoul(buf, 10, &project_id);
	if (ret)
		return ret;

	mutex_lock(&cs48l10->lock);

	ret = cs48l10_ioctl_change_project(cs48l10, project_id, 0);

	mutex_unlock(&cs48l10->lock);

	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(project_id, 0666,
		   cs48l10_project_id_show, cs48l10_project_id_store);

static struct attribute *cs48l10_attributes[] = {
	&dev_attr_project_id.attr,
	NULL
};

static const struct attribute_group cs48l10_attr_group = {
	.attrs = cs48l10_attributes,
};

/* END sysfs */

/*
    cs48l10_probe()
*/
static int __devinit cs48l10_probe(struct spi_device *spi)
{
	struct cs48l10 *cs48l10;
	int ret = 0;

	dev_info(&spi->dev, "CS48L10 driver %s loaded\n", DRV_VERSION);

	cs48l10 = kzalloc(sizeof(struct cs48l10), GFP_KERNEL);
	if (cs48l10 == NULL)
		return -ENOMEM;

	cs48l10->spi = spi;	/* cs48l10 to spi reference */

	mutex_init(&cs48l10->lock);
	mutex_init(&cs48l10->cmd_lock);
	init_waitqueue_head(&cs48l10->rx_data_wait);
	INIT_LIST_HEAD(&cs48l10->cmd_list);
	INIT_WORK(&cs48l10->int_work, dsp_cmd_dispatch);
	INIT_DELAYED_WORK(&cs48l10->dsp_clk_req_disable_work,
					  disable_dsp_clk_req_work);

	dev_set_drvdata(&spi->dev, cs48l10);	/* spi to cs48l10 reference */

	/* setup platform RESET/INT/BUSY functions */
	ret = cs48l10_setup_gpios(spi, cs48l10);
	if (ret < 0) {
		kfree(cs48l10);
		return ret;
    }

	dsp_regulator = regulator_get(NULL, "sw4");
	if (IS_ERR(dsp_regulator)) {
		dev_err(&spi->dev, "%s: Cannot get sw4 regulator\n",
				__func__);
		ret = PTR_ERR(dsp_regulator);
		dsp_regulator = NULL;
		kfree(cs48l10);
		return ret;
	} else {
		/* REFCLK should be available before RESET' goes HIGH.
		 * request clock here so there is some delay before
		 * RESET goes HIGH */
		audio_clock_request(1);
		ret = regulator_enable(dsp_regulator);
		if (ret >= 0) {
			dsp_regulator_state = 1;
			/* As per spec >= 1 us needed after supply
			 * stabilizes and RESET' is high.*/
			udelay(20);
			gpio_set_value(cs48l10->gpio_reset, 1);
			dev_info(&cs48l10->spi->dev,
				"DSP regulator(SW4) enable"
				"success in probe()\n");
		} else {
			dev_err(&cs48l10->spi->dev,
					"DSP regulator(SW4) enable"
					"failed in probe()"
					" ret val = %d\n", ret);
			goto err_out;
		}
	}
	if ((ret = sysfs_create_group(&spi->dev.kobj, &cs48l10_attr_group)) < 0) {
		dev_err(&spi->dev, "failed to register sysfs\n");
		goto err_out;
	}

	/* The buffer is mapped by dma_map_single() in omap2_mcspi */
	cs48l10->buffer = kmalloc(CS48L10_BUFFER_SIZE, GFP_KERNEL | GFP_DMA);

	if (cs48l10->buffer == NULL) {
		dev_err(&spi->dev, "no memory for cs48l10->buffer \n");
		sysfs_remove_group(&spi->dev.kobj, &cs48l10_attr_group);
		ret = -ENOMEM;
		goto err_out;
	}

#ifdef CS48L10_ASYNC_FWLOADER
	if ((ret = cs48l10_fw_load_async(cs48l10, CS48L10_FIRMWARE_NAME)) < 0) {

		sysfs_remove_group(&spi->dev.kobj, &cs48l10_attr_group);


		goto err_buffer;
	}
#else

	if ((ret = cs48l10_fw_load_sync(cs48l10, CS48L10_FIRMWARE_NAME)) < 0) {
		dev_err(&spi->dev,
			"Failed to load %s\n", CS48L10_FIRMWARE_NAME);

		sysfs_remove_group(&spi->dev.kobj, &cs48l10_attr_group);


		goto err_buffer;
	}

	mutex_lock(&cs48l10->lock);

	/* Boot the default project and go in hibernate mode */

	ret = cs48l10_fw_boot_project(cs48l10, CS48L10_DEFAULT_PROJECT_ID);
	if (ret >= 0) {
		mdelay(10);
		/* set low power mode (hibernate) */
		dsp_write(cs48l10, DSP_OS, SOFTBOOT, 0x10);
		mdelay(5);
		audio_clock_request(0);
	} else
		dev_err(&cs48l10->spi->dev,
			"Project %d failed to start, ret = %d\n",
			CS48L10_DEFAULT_PROJECT_ID, ret);


	mutex_unlock(&cs48l10->lock);

	if (ret < 0)
		goto err_buffer;
#endif

#ifdef CS48L10_CDEV
        ret = cs48l10_cdev_init(cs48l10);
	if (ret < 0) {
		sysfs_remove_group(&spi->dev.kobj, &cs48l10_attr_group);
		goto err_buffer;
	}

#endif
	return 0;

err_buffer:
	kfree(cs48l10->buffer);
err_out:
	audio_clock_request(0);
	cs48l10_free_gpios(spi, cs48l10);
	kfree(cs48l10);
	if (dsp_regulator) {
		regulator_put(dsp_regulator);
		dsp_regulator = NULL;
		dsp_regulator_state = 0;
	}
	return ret;
}

/*
    cs48l10_remove()
*/
static int cs48l10_remove(struct spi_device *spi)
{
	struct cs48l10 *cs48l10 = dev_get_drvdata(&spi->dev);
#ifdef CS48L10_CDEV
	dev_t devno =  MKDEV(cs48l10_major, cs48l10_minor);
	device_destroy(cs48l10_class, devno);
	class_destroy(cs48l10_class);

	cdev_del(&cs48l10->cdev);
	unregister_chrdev_region(devno, cs48l10_nr_devs);
#endif
	cancel_delayed_work_sync(&cs48l10->dsp_clk_req_disable_work);
	audio_clock_request(0);
	kfree(cs48l10->buffer);

	cs48l10_free_gpios(spi, cs48l10);

	sysfs_remove_group(&spi->dev.kobj, &cs48l10_attr_group);

	mutex_lock(&cs48l10->lock);

	if (cs48l10->fw)
		kfree(cs48l10->fw);

	if (cs48l10->osfw)
		release_firmware(cs48l10->osfw);

	mutex_unlock(&cs48l10->lock);

	if (cs48l10)
		kfree(cs48l10);

	if (dsp_regulator) {
		regulator_put(dsp_regulator);
		dsp_regulator_state = 0;
		dsp_regulator = NULL;
	}

	return 0;
}
/*
    Power management
*/
#ifdef CONFIG_PM
/* The regilator that supplies DSP VD is turned off in suspend if DSP
 * is not in use and turned on in cdev_open. The turn off/on sequence is
 * RESET'=LOW, VD=LOW --- VD=HIGH, wait >= 1uS, RESET'=HIGH */
static int cs48l10_suspend(struct spi_device *spi, pm_message_t msg)
{
	struct cs48l10 *cs48l10 = dev_get_drvdata(&spi->dev);
	if (is_dsp_in_use)
		return 0;
   /* keep RESET' low during regulator being off */
	gpio_set_value(cs48l10->gpio_reset, 0);

	/* Turn off the DSP regulator for additional power saving */
	if (dsp_regulator && dsp_regulator_state == 1) {
		if (regulator_disable(dsp_regulator) < 0)
			dev_err(&spi->dev,
				"DSP regulator(SW4) disable failed\n");
	    else {
			dsp_regulator_state = 0;
			dev_info(&spi->dev,
				"DSP regulator(SW4) disable success\n");
	    }
	}
	cancel_delayed_work_sync(&cs48l10->dsp_clk_req_disable_work);
	audio_clock_request(0);
	return 0;
}

static int cs48l10_resume(struct spi_device *spi)
{
	return 0;
}

#else
#define cs48l10_suspend NULL
#define cs48l10_resume  NULL
#endif

/*
    SPI driver
*/
static struct spi_driver cs48l10_driver = {
	.driver = {
		   .name = "cs48l10",
		   .bus = &spi_bus_type,
		   .owner = THIS_MODULE,
		   },
	.suspend = cs48l10_suspend,
	.resume = cs48l10_resume,
	.probe = cs48l10_probe,
	.remove = __devexit_p(cs48l10_remove),
};

static int __init cs48l10_init(void)
{
	return spi_register_driver(&cs48l10_driver);
}

static void __exit cs48l10_exit(void)
{
	spi_unregister_driver(&cs48l10_driver);
}

module_init(cs48l10_init);
module_exit(cs48l10_exit);

MODULE_AUTHOR("Georgi Vlaev, Nucleus Systems, Ltd, <office@nucleusys.com>");
MODULE_DESCRIPTION("CS48L10 CDEV DSP Driver");
MODULE_LICENSE("GPL");

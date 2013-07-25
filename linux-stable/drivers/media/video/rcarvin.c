/*
 * drivers/media/video/rcarvin.c
 *     V4L2 Driver for R-Car VIN Unit interface
 *
 * Copyright (C) 2011-2012 Renesas Electronics Corporation
 *
 * This file is based on the drivers/media/video/sh_mobile_ceu_camera.c
 *
 * Copyright (C) 2008 Magnus Damm
 *
 * Based on V4L2 Driver for PXA camera host - "pxa_camera.c",
 *
 * Copyright (C) 2006, Sascha Hauer, Pengutronix
 * Copyright (C) 2008, Guennadi Liakhovetski <kernel@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/time.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/videodev2.h>
#include <linux/pm_runtime.h>
#include <linux/sched.h>

#include <media/v4l2-common.h>
#include <media/v4l2-dev.h>
#include <media/soc_camera.h>
#include <media/rcarvin.h>
#include <media/videobuf2-dma-contig.h>
#include <media/v4l2-mediabus.h>
#include <media/soc_mediabus.h>

/* support to check initial value of VIN reg (0:Not support, 1:support)*/
#define RCARVIN_SUPPORT_TO_CHECK_REGS 0
/* support to detect error interrupt (0:Disable, 1:Enable)*/
#define RCARVIN_SUPPORT_ERR_INT 0


/* register offsets for r-car vin */
#define V0MC 0x0000
#define V0MS 0x0004
#define V0FC 0x0008
#define V0SLPrC 0x000C
#define V0ELPrC 0x0010
#define V0SPPrC 0x0014
#define V0EPPrC 0x0018
#define V0SLPoC 0x001C
#define V0ELPoC 0x0020
#define V0SPPoC 0x0024
#define V0EPPoC 0x0028
#define V0IS 0x002C
#define V0MB1 0x0030
#define V0MB2 0x0034
#define V0MB3 0x0038
#define V0LC 0x003C
#define V0IE 0x0040
#define V0INTS 0x0044
#define V0SI 0x0048
#define V0MTC 0x004C
#define V0YS 0x0050
#define V0XS 0x0054
#define V0DMR 0x0058
#define V0DMR2 0x005C
#define V0UVAOF 0x0060
#define V0CSCC1 0x0064
#define V0CSCC2 0x0068
#define V0CSCC3 0x006C
#define V0C1A 0x0080
#define V0C1B 0x0084
#define V0C1C 0x0088
#define V0C2A 0x0090
#define V0C2B 0x0094
#define V0C2C 0x0098
#define V0C3A 0x00A0
#define V0C3B 0x00A4
#define V0C3C 0x00A8
#define V0C4A 0x00B0
#define V0C4B 0x00B4
#define V0C4C 0x00B8
#define V0C5A 0x00C0
#define V0C5B 0x00C4
#define V0C5C 0x00C8
#define V0C6A 0x00D0
#define V0C6B 0x00D4
#define V0C6C 0x00D8
#define V0C7A 0x00E0
#define V0C7B 0x00E4
#define V0C7C 0x00E8
#define V0C8A 0x00F0
#define V0C8B 0x00F4
#define V0C8C 0x00F8

#define BUF_OFFSET	0x4
#define MB_NUM		3
#define MB_MASK		0x18
#define CONT_TRANS	(MB_NUM+1)

#undef DEBUG_GEOMETRY
/*#define DEBUG_GEOMETRY*/
#ifdef DEBUG_GEOMETRY
#define dev_geo	dev_info
#else
#define dev_geo	dev_dbg
#endif

enum rcar_vin_capture_status {
	STOPPED,
	RUNNING,
	STOPPING,
};

/* per video frame buffer */
struct rcar_vin_buffer {
	struct vb2_buffer vb; /* v4l buffer must be first */
	struct list_head queue;
};

struct rcar_vin_dev {
	struct soc_camera_host ici;
	struct soc_camera_device *icd;

	unsigned int irq;
	void __iomem *base;

	spinlock_t lock;		/* Protects video buffer lists */
	struct list_head capture;
	struct vb2_alloc_ctx *alloc_ctx;

	struct rcar_vin_info *pdata;

	enum v4l2_field field;
	int sequence;
	bool use_bt656;

	struct vb2_buffer *queue_buf[MB_NUM];

	bool data_through;
	unsigned int vb_count;
	unsigned int nr_hw_slots;
	enum rcar_vin_capture_status capture_status;
	unsigned int request_to_stop;
	struct completion capture_stop;
};

struct rcar_vin_cam {
	/* VIN offsets within the camera output, before the VIN scaler */
	unsigned int vin_left;
	unsigned int vin_top;
	/* Client output, as seen by the VIN */
	unsigned int width;
	unsigned int height;
	/*
	 * User window from S_CROP / G_CROP, produced by client cropping and
	 * scaling, VIN scaling and VIN cropping, mapped back onto the client
	 * input window
	 */
	struct v4l2_rect subrect;
	/* Camera cropping rectangle */
	struct v4l2_rect rect;
	const struct soc_mbus_pixelfmt *extra_fmt;
	enum v4l2_mbus_pixelcode code;
};

static struct rcar_vin_buffer *to_vin_vb(struct vb2_buffer *vb)
{
	return container_of(vb, struct rcar_vin_buffer, vb);
}

static struct soc_camera_device *q_to_icd(struct vb2_queue *q)
{
	return container_of(q, struct soc_camera_device, vb2_vidq);
}

static void vin_write(struct rcar_vin_dev *priv,
		      unsigned long reg_offs, u32 data)
{
	iowrite32(data, priv->base + reg_offs);
}

static u32 vin_read(struct rcar_vin_dev *priv, unsigned long reg_offs)
{
	return ioread32(priv->base + reg_offs);
}

static void rcar_vin_soft_reset(struct rcar_vin_dev *pcdev)
{
	/* clear V0INTS  */
	vin_write(pcdev, V0INTS, 0);
}

static u32 vin_get_status(struct rcar_vin_dev *priv)
{
	return vin_read(priv, V0MS);
}

static u32 vin_is_field_active(struct rcar_vin_dev *priv)
{
	return (vin_read(priv, V0MS) & 0x2) >> 1;
}

/*
 * .queue_setup() is called to check, whether the driver can accept the
 *		  requested number of buffers and to fill in plane sizes
 *		  for the current frame format if required
 */
static int rcar_vin_videobuf_setup(struct vb2_queue *vq,
			const struct v4l2_format *fmt,
			unsigned int *count, unsigned int *num_planes,
			unsigned int sizes[], void *alloc_ctxs[])
{
	struct soc_camera_device *icd = q_to_icd(vq);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct rcar_vin_dev *pcdev = ici->priv;
	int bytes_per_line;
	unsigned int height;

	if (fmt) {
		const struct soc_camera_format_xlate *xlate;
		xlate =	soc_camera_xlate_by_fourcc(icd,
				fmt->fmt.pix.pixelformat);
		if (!xlate)
			return -EINVAL;
		bytes_per_line = soc_mbus_bytes_per_line(fmt->fmt.pix.width,
							 xlate->host_fmt);
		height = fmt->fmt.pix.height;
	} else {
		/* Called from VIDIOC_REQBUFS or in compatibility mode */
		bytes_per_line = soc_mbus_bytes_per_line(icd->user_width,
						icd->current_fmt->host_fmt);
		height = icd->user_height;
	}
	if (bytes_per_line < 0)
		return bytes_per_line;

	sizes[0] = bytes_per_line * height;

	alloc_ctxs[0] = pcdev->alloc_ctx;

	if (!vq->num_buffers)
		pcdev->sequence = 0;

	if (!*count)
		*count = 2;

	*num_planes = 1;

	pcdev->vb_count = *count;

	/* Number of hardware slots */
	if (pcdev->vb_count >= CONT_TRANS)
		pcdev->nr_hw_slots = MB_NUM;
	else
		pcdev->nr_hw_slots = 1;

	dev_dbg(icd->parent, "count=%d, size=%u\n", *count, sizes[0]);

	return 0;
}

#define VIN_VNIE_FIE2  (1 << 31) /* Field Interrupt Enable 2 */
#define VIN_VNIE_VFE   (1 << 17)
				/* Vsync Falling edge detect interrupt Enable */
#define VIN_VNIE_VRE   (1 << 16)
				/* Vsync Rising edge detect interrupt Enable */
#define VIN_VNIE_FIE   (1 << 4)  /* Field Interrupt Enable */
#define VIN_VNIE_CEE   (1 << 3)  /* Correct Error interrupt Enable */
#define VIN_VNIE_SIE   (1 << 2)  /* Scanline Interrupt Enable */
#define VIN_VNIE_EFE   (1 << 1)  /* End of Frame interrupt Enable */
#define VIN_VNIE_FOE   (1 << 0)  /* Fifo Over flow interrupt Enable */

#define VIN_VNIE_MASK_PROGRESSIVE	(VIN_VNIE_FIE | VIN_VNIE_EFE)
#define VIN_VNIE_MASK_INTERLACED	(VIN_VNIE_EFE)
#define VIN_VNIE_ERROR_MASK  (VIN_VNIE_CEE | VIN_VNIE_FOE)

/* VnMC */
#define VIN_VNMC_FOC		0x00200000 /* Field Order Control */
#define VIN_VNMC_YCAL		0x00080000 /* YCbCr-422 input data ALignment */
#define VIN_VNMC_VUP		0x00000400 /* Vin register UPdate control */
#define VIN_VNMC_INF_YUV8_BT656	0x00000000 /* BT.656/8-bit/YCbCr-422 */
#define VIN_VNMC_INF_YUV8_BT601	0x00010000 /* BT.601/8-bit/YCbCr-422 */
#define VIN_VNMC_INF_YUV10_BT656 0x00020000 /* BT.656/10/12-bit/YCbCr-422 */
#define VIN_VNMC_INF_YUV10_BT601 0x00030000 /* BT.601/10/12-bit/YCbCr-422 */
#define VIN_VNMC_INF_YUV16	0x00050000 /* BT.1358/16bit/YUV-422 */
#define VIN_VNMC_INF_RGB888_24	0x00060000 /* BT.601/12 or 24bit/RGB-888 */
#define VIN_VNMC_INF_RGB666	0x00070000 /* BT.601/18bit/RGB-666 or
						BT.601/2x12bit/RGB-888 */

#define VIN_VNMC_IM_MASK	0x00000018 /* Interlace Mode */
#define VIN_VNMC_IM_ODD		0x00000000
#define VIN_VNMC_IM_ODD_EVEN	0x00000008
#define VIN_VNMC_IM_EVEN	0x00000010
#define VIN_VNMC_IM_FULL	0x00000018

#define VIN_VNMC_FIELD_MASK  (VIN_VNMC_FOC | VIN_VNMC_IM_MASK)

#define VIN_VNMC_BPS		0x00000002
			/* ycbcr-422 -> ycbcr-422 convert ByPaSs mode */
#define VIN_VNMC_ME		0x00000001  /* Module Enable */

/* VnMS */
#define VIN_VNMS_FBS		0x00000018 /* Frame Buffer Status */
#define VIN_VNMS_FS		0x00000004 /* Field Status */
#define VIN_VNMS_AV		0x00000002 /* Active Video status */
#define VIN_VNMS_CA		0x00000001 /* video Capture Active Status */

/* VnFC */
#define VIN_VNFC_C_FRAME	0x00000002 /* Continuous frame Capture mode */
#define VIN_VNFC_S_FRAME	0x00000001 /* Single frame Capture mode */

/* VnDMR */
#define VIN_VNDMR_EVA		0x00010000 /* EVen field Address offset */
#define VIN_VNDMR_EXRGB		0x00000100 /* Extension RGB Conversion Mode */
#define VIN_VNDMR_BPSM		0x00000010 /* Byte Position Swap Mode */
#define VIN_VNDMR_DTMD_YCSEP	0x00000002 /* transfer: YC separate */
#define VIN_VNDMR_DTMD_ARGB1555	0x00000001 /* transfer: ARGB1555 */

/* VnDMR2 */
#define VIN_VNDMR2_FPS		0x80000000 /* Field Polarity Select */
#define VIN_VNDMR2_VPS		0x40000000 /* Vsync Polarity Select */
#define VIN_VNDMR2_VPS_ACTIVE_LOW	0x00000000
#define VIN_VNDMR2_VPS_ACTIVE_HIGH	VIN_VNDMR2_VPS
#define VIN_VNDMR2_HPS		0x20000000 /* Hsync Polarity Select */
#define VIN_VNDMR2_HPS_ACTIVE_LOW	0x00000000
#define VIN_VNDMR2_HPS_ACTIVE_HIGH	VIN_VNDMR2_HPS
#define VIN_VNDMR2_CES		0x10000000 /* Clock Enable polarity Select */
#define VIN_VNDMR2_CHS		0x00800000 /* Clock Enable Hsync Select */
#define VIN_VNDMR2_FTEV		0x00020000 /* Field Toggle Enable of Vsync */

static bool is_continuous_transfer(struct rcar_vin_dev *pcdev)
{
	return (pcdev->vb_count >= CONT_TRANS);
}

static void rcar_vin_setup(struct rcar_vin_dev *pcdev)
{
	struct soc_camera_device *icd = pcdev->icd;
	struct rcar_vin_cam *cam = icd->host_priv;
	u32 mc, dmr;
	int progressive = 0;
	int input_is_yuv = 0;
	int output_is_yuv = 0;

	/* field */
	switch (pcdev->field) {
	case V4L2_FIELD_TOP:
		mc = VIN_VNMC_IM_ODD;
		break;
	case V4L2_FIELD_BOTTOM:
		mc = VIN_VNMC_IM_EVEN;
		break;
	case V4L2_FIELD_INTERLACED:
	case V4L2_FIELD_INTERLACED_TB:
		mc = VIN_VNMC_IM_FULL;
		break;
	case V4L2_FIELD_INTERLACED_BT:
		mc = VIN_VNMC_IM_FULL | VIN_VNMC_FOC;
		break;
	case V4L2_FIELD_NONE:
		if (is_continuous_transfer(pcdev)) {
			mc = VIN_VNMC_IM_ODD_EVEN;
			progressive = 1;
		} else
			mc = VIN_VNMC_IM_ODD;
		break;
	default:
		mc = VIN_VNMC_IM_ODD;
		break;
	}

	/* input interface */
	switch (icd->current_fmt->code) {
	case V4L2_MBUS_FMT_RGB888_1X24_LE:
		/* BT.601 24bit RGB-888 */
		/* FIXME: BT.709 RGB-888 must be also supported. */
		mc |= VIN_VNMC_INF_RGB888_24;
		break;
	case V4L2_MBUS_FMT_RGB888_2X12_LE:
	case V4L2_MBUS_FMT_RGB666_1X18_LE:
		/* BT.601 18bit RGB-666 or 2x12bit RGB-888 */
		/* FIXME: BT.709 RGB-666 must be also supported. */
		mc |= VIN_VNMC_INF_RGB666;
		break;
	case V4L2_MBUS_FMT_YUYV8_1X16:
		/* BT.1358 16bit YCbCr-422 */
		mc |= VIN_VNMC_INF_YUV16;
		input_is_yuv = 1;
		break;
	case V4L2_MBUS_FMT_YUYV8_2X8:
		input_is_yuv = 1;
		/* BT.656 8bit YCbCr-422 */
		if (pcdev->use_bt656)
			mc |= VIN_VNMC_INF_YUV8_BT656;
		else
			mc |= VIN_VNMC_INF_YUV8_BT601;
		break;
	case V4L2_MBUS_FMT_YUYV10_2X10:
		input_is_yuv = 1;
		/* BT.656 10bit YCbCr-422 */
		if (pcdev->use_bt656)
			mc |= VIN_VNMC_INF_YUV10_BT656;
		else
			mc |= VIN_VNMC_INF_YUV10_BT601;
	default:
		break;
	}

	/* output format */
	switch (icd->current_fmt->host_fmt->fourcc) {
	case V4L2_PIX_FMT_NV16:
		vin_write(pcdev, V0UVAOF,
			  ((cam->width * cam->height) + 0x7f) & ~0x7f);
		dmr = VIN_VNDMR_DTMD_YCSEP;
		output_is_yuv = 1;
		break;
	case V4L2_PIX_FMT_YUYV:
		dmr = VIN_VNDMR_BPSM;
		output_is_yuv = 1;
		break;
	case V4L2_PIX_FMT_UYVY:
		dmr = 0;
		output_is_yuv = 1;
		break;
	case V4L2_PIX_FMT_RGB555X:
		dmr = VIN_VNDMR_DTMD_ARGB1555;
		break;
	case V4L2_PIX_FMT_RGB565:
		dmr = 0;
		break;
	case V4L2_PIX_FMT_RGB32:
		dmr = VIN_VNDMR_EXRGB;
		break;
	default:
		pr_alert("%s: Invalid fourcc format (0x%x)\n", __func__,
		icd->current_fmt->host_fmt->fourcc);
		dmr = vin_read(pcdev, V0DMR);
		mc = vin_read(pcdev, V0MC);
		break;
	}

	/* Always update on field change */
	mc |= VIN_VNMC_VUP;

	/* If input and output use the same colorspace, use bypass mode */
	if (input_is_yuv == output_is_yuv)
		mc |= VIN_VNMC_BPS;

	/* enable interrupt */
	if (progressive)
		vin_write(pcdev, V0IE, VIN_VNIE_MASK_PROGRESSIVE);
	else
		vin_write(pcdev, V0IE, VIN_VNIE_MASK_INTERLACED);

	/* start capturing */
	vin_write(pcdev, V0DMR, dmr);
	vin_write(pcdev, V0MC, mc | VIN_VNMC_ME);
}

static void rcar_vin_capture(struct rcar_vin_dev *pcdev)
{
	if (is_continuous_transfer(pcdev)) {
		/* continuous transfer ON */
		vin_write(pcdev, V0FC, VIN_VNFC_C_FRAME);
	} else {
		/* single transfer ON */
		vin_write(pcdev, V0FC, VIN_VNFC_S_FRAME);
	}
}

static void rcar_vin_request_capture_stop(struct rcar_vin_dev *pcdev)
{
	pcdev->capture_status = STOPPING;

	/* continuous & single transfer OFF */
	vin_write(pcdev, V0FC, 0);

	/* disable capture (release DMA buffer), reset */
	vin_write(pcdev, V0MC, vin_read(pcdev, V0MC) & ~VIN_VNMC_ME);

	/* update the status if stopped already */
	if ((vin_read(pcdev, V0MS) & VIN_VNMS_CA) == 0)
		pcdev->capture_status = STOPPED;
}

static int get_available_hw_slot(struct rcar_vin_dev *pcdev)
{
	int slot;

	for (slot = 0; slot < pcdev->nr_hw_slots; slot++) {
		if (pcdev->queue_buf[slot] == NULL)
			return slot;
	}
	return -1;
}

static int hw_ready(struct rcar_vin_dev *pcdev)
{
	int slot;

	/* Ensure all HW slots are filled */
	slot = get_available_hw_slot(pcdev);
	if (slot < 0)
		return 1;
	return 0;
}

/* Moves a buffer from the queue to the HW slots */
static int move_buf_to_hw_slot(struct rcar_vin_dev *pcdev)
{
	struct vb2_buffer *vb;
	dma_addr_t phys_addr_top;
	int slot;

	if (list_empty(&pcdev->capture))
		return 0;

	/* Find a free HW slot */
	slot = get_available_hw_slot(pcdev);
	if (slot < 0)
		return 0;

	vb = &list_entry(pcdev->capture.next,
		struct rcar_vin_buffer, queue)->vb;
	list_del_init(&to_vin_vb(vb)->queue);
	pcdev->queue_buf[slot] = vb;
	phys_addr_top =	vb2_dma_contig_plane_dma_addr(vb, 0);
	vin_write(pcdev, V0MB1 + (BUF_OFFSET * slot), phys_addr_top);

	return 1;
}

static int rcar_vin_videobuf_prepare(struct vb2_buffer *vb)
{
	struct rcar_vin_buffer *buf = to_vin_vb(vb);

	/* Added list head initialization on alloc */
	WARN(!list_empty(&buf->queue), "Buffer %p on queue!\n", vb);

	return 0;
}

static void rcar_vin_videobuf_queue(struct vb2_buffer *vb)
{
	struct soc_camera_device *icd = q_to_icd(vb->vb2_queue);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct rcar_vin_dev *pcdev = ici->priv;
	struct rcar_vin_buffer *buf = to_vin_vb(vb);
	unsigned long size;
	unsigned long flags;

	int bytes_per_line = soc_mbus_bytes_per_line(icd->user_width,
						icd->current_fmt->host_fmt);

	if (bytes_per_line < 0)
		goto error;

	size = icd->user_height * bytes_per_line;

	if (vb2_plane_size(vb, 0) < size) {
		dev_err(icd->parent, "Buffer #%d too small (%lu < %lu)\n",
			vb->v4l2_buf.index, vb2_plane_size(vb, 0), size);
		goto error;
	}

	vb2_set_plane_payload(vb, 0, size);

	dev_dbg(icd->parent, "%s (vb=0x%p) 0x%p %lu\n", __func__,
		vb, vb2_plane_vaddr(vb, 0), vb2_get_plane_payload(vb, 0));

#ifdef DEBUG
	/*
	 * This can be useful if you want to see if we actually fill
	 * the buffer with something
	 */
	if (vb2_plane_vaddr(vb, 0))
		memset(vb2_plane_vaddr(vb, 0), 0xaa,
			vb2_get_plane_payload(vb, 0));
#endif

	spin_lock_irqsave(&pcdev->lock, flags);

	list_add_tail(&buf->queue, &pcdev->capture);
	move_buf_to_hw_slot(pcdev);

	/* If we weren't running, and have enough buffers, start capturing! */
	if ((pcdev->capture_status != RUNNING) && hw_ready(pcdev)) {
		pcdev->request_to_stop = 0;
		init_completion(&pcdev->capture_stop);
		pcdev->capture_status = RUNNING;
		rcar_vin_setup(pcdev);
		rcar_vin_capture(pcdev);
	}

	spin_unlock_irqrestore(&pcdev->lock, flags);

	return;

error:
	vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
}

static void rcar_vin_videobuf_release(struct vb2_buffer *vb)
{
	struct soc_camera_device *icd = q_to_icd(vb->vb2_queue);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct rcar_vin_dev *pcdev = ici->priv;
	struct rcar_vin_buffer *buf = to_vin_vb(vb);
	unsigned int i;
	unsigned long flags;
	int buf_in_use = 0;

	spin_lock_irqsave(&pcdev->lock, flags);

	/* Is the buffer is use by the VIN hardware? */
	for (i = 0; i < MB_NUM; i++) {
		if (pcdev->queue_buf[i] == vb) {
			buf_in_use = 1;
			break;
		}
	}

	if (buf_in_use) {
		while (pcdev->capture_status != STOPPED) {

			/* issue stop if running */
			if (pcdev->capture_status == RUNNING)
				rcar_vin_request_capture_stop(pcdev);

			/* wait until capturing has been stopped */
			if (pcdev->capture_status == STOPPING) {
				pcdev->request_to_stop = 1;
				spin_unlock_irqrestore(&pcdev->lock, flags);
				wait_for_completion(&pcdev->capture_stop);
				spin_lock_irqsave(&pcdev->lock, flags);
			}
		}

		/* Capturing has now stopped. The buffer we have been asked
		   to release could be any of the current buffers in use, so
		   release all buffers that are in use by HW */
		for (i = 0; i < MB_NUM; i++) {
			if (pcdev->queue_buf[i]) {
				vb2_buffer_done(pcdev->queue_buf[i],
					VB2_BUF_STATE_ERROR);
				pcdev->queue_buf[i] = NULL;
			}
		}
	} else {
		if (buf->queue.next)
			list_del_init(&buf->queue);

	}

	spin_unlock_irqrestore(&pcdev->lock, flags);
}

static int rcar_vin_videobuf_init(struct vb2_buffer *vb)
{
	/* This is for locking debugging only */
	INIT_LIST_HEAD(&to_vin_vb(vb)->queue);
	return 0;
}

static int rcar_vin_stop_streaming(struct vb2_queue *q)
{
	struct soc_camera_device *icd = q_to_icd(q);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct rcar_vin_dev *pcdev = ici->priv;
	struct list_head *buf_head, *tmp;

	spin_lock_irq(&pcdev->lock);

	list_for_each_safe(buf_head, tmp, &pcdev->capture)
		list_del_init(buf_head);

	spin_unlock_irq(&pcdev->lock);

	rcar_vin_soft_reset(pcdev);
	return 0;
}

static struct vb2_ops rcar_vin_videobuf_ops = {
	.queue_setup	= rcar_vin_videobuf_setup,
	.buf_prepare	= rcar_vin_videobuf_prepare,
	.buf_queue	= rcar_vin_videobuf_queue,
	.buf_cleanup	= rcar_vin_videobuf_release,
	.buf_init	= rcar_vin_videobuf_init,
	.wait_prepare	= soc_camera_unlock,
	.wait_finish	= soc_camera_lock,
	.stop_streaming	= rcar_vin_stop_streaming,
};

static irqreturn_t rcar_vin_irq(int irq, void *data)
{
	struct rcar_vin_dev *pcdev = data;
	u32 status_of_int;
	bool can_run = 0, hw_stopped;
	int slot;

	spin_lock(&pcdev->lock);

	/* clear interrupt */
	status_of_int = vin_read(pcdev, V0INTS);
	vin_write(pcdev, V0INTS, status_of_int);

#if RCARVIN_SUPPORT_ERR_INT
	/*
	 * When a CEE or FOE interrupt occurs, a capture end interrupt does not
	 * occur and the image of that frame is not captured correctly. So, soft
	 * reset is needed here.
	 */
	if (status_of_int & VIN_VNIE_ERROR_MASK) {
		dev_dbg(icd->parent, "capture error\n");
		rcar_vin_soft_reset(pcdev);
		goto done;
	}
#endif /* RCARVIN_SUPPORT_ERR_INT */

	/* nothing to do if capture status is 'STOPPED'. */
	if (pcdev->capture_status == STOPPED)
		goto done;

	hw_stopped = ((vin_read(pcdev, V0MS) & VIN_VNMS_CA) == 0);

	if (pcdev->request_to_stop == 0) {
		if (is_continuous_transfer(pcdev))
			slot = (vin_get_status(pcdev) & MB_MASK) >> 3;
		else
			slot = 0;

		pcdev->queue_buf[slot]->v4l2_buf.field = pcdev->field;
		pcdev->queue_buf[slot]->v4l2_buf.sequence = pcdev->sequence++;
		do_gettimeofday(&pcdev->queue_buf[slot]->v4l2_buf.timestamp);
		vb2_buffer_done(pcdev->queue_buf[slot], VB2_BUF_STATE_DONE);
		pcdev->queue_buf[slot] = NULL;

		if (pcdev->capture_status != STOPPING)
			can_run = move_buf_to_hw_slot(pcdev);

		if (hw_stopped || !can_run)
			pcdev->capture_status = STOPPED;
		else
			rcar_vin_capture(pcdev);

	} else if (hw_stopped) {
		pcdev->capture_status = STOPPED;
		pcdev->request_to_stop = 0;
		complete(&pcdev->capture_stop);
	}

done:
	spin_unlock(&pcdev->lock);

	return IRQ_HANDLED;
}

/* Called with .video_lock held */
static int rcar_vin_add_device(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct rcar_vin_dev *pcdev = ici->priv;
	int i;

	if (pcdev->icd)
		return -EBUSY;

#ifdef CONFIG_PM
	pm_runtime_get_sync(ici->v4l2_dev.dev);
#endif /* CONFIG_PM */

	for (i = 0 ; i < MB_NUM; i++)
		pcdev->queue_buf[i] = NULL;

	dev_dbg(icd->parent,
		"R-Car VIN Unit driver attached to camera %d\n",
		icd->devnum);

	rcar_vin_soft_reset(pcdev);
	pcdev->icd = icd;

	return 0;
}

/* Called with .video_lock held */
static void rcar_vin_remove_device(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct rcar_vin_dev *pcdev = ici->priv;
	unsigned int i;

	BUG_ON(icd != pcdev->icd);

	/* disable capture, disable interrupts */
	vin_write(pcdev, V0MC, vin_read(pcdev, V0MC) & ~VIN_VNMC_ME);
	vin_write(pcdev, V0IE, 0x00000000);

	rcar_vin_soft_reset(pcdev);

	pcdev->capture_status = STOPPED;
	pcdev->request_to_stop = 0;

	/* make sure active buffer is canceled */
	spin_lock_irq(&pcdev->lock);

	for (i = 0; i < MB_NUM; i++) {
		if (pcdev->queue_buf[i]) {
			list_del_init(&to_vin_vb(pcdev->queue_buf[i])->queue);
			vb2_buffer_done(pcdev->queue_buf[i],
				VB2_BUF_STATE_ERROR);
			pcdev->queue_buf[i] = NULL;
		}
	}

	spin_unlock_irq(&pcdev->lock);

#ifdef CONFIG_PM
	pm_runtime_put_sync(ici->v4l2_dev.dev);
#endif /* CONFIG_PM */

	dev_dbg(icd->parent,
		"R-Car VIN Unit driver detached from camera %d\n",
		icd->devnum);

	pcdev->icd = NULL;
}

static unsigned int size_dst(unsigned int src, unsigned int scale)
{
	unsigned int mant_pre = scale >> 12;
	if (!src || !scale)
		return src;
	return ((mant_pre + 2 * (src - 1)) / (2 * mant_pre) - 1) *
		mant_pre * 4096 / scale + 1;
}

static u16 calc_scale(unsigned int src, unsigned int *dst)
{
	u16 scale;

	if (src == *dst)
		return 0;

	scale = (src * 4096 / *dst) & ~7;

	while (scale > 4096 && size_dst(src, scale) < *dst)
		scale -= 8;

	*dst = size_dst(src, scale);

	return scale;
}

/* rect is guaranteed to not exceed the scaled camera rectangle */
static int rcar_vin_set_rect(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct rcar_vin_cam *cam = icd->host_priv;
	struct rcar_vin_dev *pcdev = ici->priv;
	unsigned int left_offset, top_offset;
	unsigned char dsize;
	struct v4l2_rect *cam_subrect = &cam->subrect;

	dev_geo(icd->parent, "Crop %ux%u@%u:%u\n",
		icd->user_width, icd->user_height, cam->vin_left, cam->vin_top);

	left_offset	= cam->vin_left;
	top_offset	= cam->vin_top;

	dsize = pcdev->data_through ? 1 : 0;

	dev_geo(icd->parent, "Cam %ux%u@%u:%u\n",
		cam->width, cam->height, cam->vin_left, cam->vin_top);
	dev_geo(icd->parent, "Cam subrect %ux%u@%u:%u\n",
		cam_subrect->width, cam_subrect->height,
		cam_subrect->left, cam_subrect->top);

	/* Set Pre-Clip and Post-Clip areas */
	/* Set Pre-Clip */
	vin_write(pcdev, V0SPPrC, left_offset << dsize);
	vin_write(pcdev, V0EPPrC, (left_offset + cam->width - 1) << dsize);
	if ((pcdev->field == V4L2_FIELD_INTERLACED) ||
		(pcdev->field == V4L2_FIELD_INTERLACED_TB) ||
		(pcdev->field == V4L2_FIELD_INTERLACED_BT)) {
		vin_write(pcdev, V0SLPrC, top_offset / 2);
		vin_write(pcdev, V0ELPrC, (top_offset + cam->height) / 2 - 1);
	} else {
		vin_write(pcdev, V0SLPrC, top_offset);
		vin_write(pcdev, V0ELPrC, top_offset + cam->height - 1);
	}

	/* Set Post-Clip */
	vin_write(pcdev, V0SPPoC, 0);
	vin_write(pcdev, V0SLPoC, 0);
	vin_write(pcdev, V0EPPoC, (cam_subrect->width - 1) << dsize);
	if ((pcdev->field == V4L2_FIELD_INTERLACED) ||
		(pcdev->field == V4L2_FIELD_INTERLACED_TB) ||
		(pcdev->field == V4L2_FIELD_INTERLACED_BT))
		vin_write(pcdev, V0ELPoC, cam_subrect->height / 2 - 1);
	else
		vin_write(pcdev, V0ELPoC, cam_subrect->height - 1);

	vin_write(pcdev, V0IS, (cam->width + 15) & ~0xf);

	return 0;
}

static u32 capture_save_reset(struct rcar_vin_dev *pcdev)
{
	u32 vnmc = vin_read(pcdev, V0MC);
	vin_write(pcdev, V0MC, vnmc & ~VIN_VNMC_ME); /* stop capture */
	return vnmc;
}

static void capture_restore(struct rcar_vin_dev *pcdev, u32 vnmc)
{
	unsigned long timeout = jiffies + 10 * HZ;

	/*
	 * Wait until the end of the current frame. It can take a long time,
	 * but if it has been aborted by a MRST1 reset, it should exit sooner.
	 */
	while (vin_is_field_active(pcdev) && time_before(jiffies, timeout))
		msleep(1);

	if (time_after(jiffies, timeout)) {
		dev_err(pcdev->ici.v4l2_dev.dev,
			"Timeout waiting for frame end! Interface problem?\n");
		return;
	}

	/* Anything to restore? */
	if (vnmc & ~VIN_VNMC_ME)
		vin_write(pcdev, V0MC, vnmc);
}

#define VIN_BUS_FLAGS (V4L2_MBUS_MASTER |	\
		V4L2_MBUS_PCLK_SAMPLE_RISING |	\
		V4L2_MBUS_HSYNC_ACTIVE_HIGH |	\
		V4L2_MBUS_HSYNC_ACTIVE_LOW |	\
		V4L2_MBUS_VSYNC_ACTIVE_HIGH |	\
		V4L2_MBUS_VSYNC_ACTIVE_LOW |	\
		V4L2_MBUS_DATA_ACTIVE_HIGH)

static int rcar_vin_set_bus_param(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct rcar_vin_dev *pcdev = ici->priv;
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct v4l2_mbus_config cfg = {.type = V4L2_MBUS_PARALLEL,};
	unsigned long value, common_flags = VIN_BUS_FLAGS;
	u32 capsr = capture_save_reset(pcdev);
	int ret;

	/*
	 * If the client doesn't implement g_mbus_config, we just use our
	 * platform data
	 */
	ret = v4l2_subdev_call(sd, video, g_mbus_config, &cfg);
	if (!ret) {
		common_flags = soc_mbus_config_compatible(&cfg, common_flags);
		if (!common_flags)
			return -EINVAL;
	} else if (ret != -ENOIOCTLCMD) {
		return ret;
	}

	if (cfg.type == V4L2_MBUS_BT656)
		pcdev->use_bt656 = true;
	else
		pcdev->use_bt656 = false;

	/* Make choises, based on platform preferences */
	if ((common_flags & V4L2_MBUS_HSYNC_ACTIVE_HIGH) &&
	    (common_flags & V4L2_MBUS_HSYNC_ACTIVE_LOW)) {
		if (pcdev->pdata->flags & RCAR_VIN_FLAG_HSYNC_LOW)
			common_flags &= ~V4L2_MBUS_HSYNC_ACTIVE_HIGH;
		else
			common_flags &= ~V4L2_MBUS_HSYNC_ACTIVE_LOW;
	}

	if ((common_flags & V4L2_MBUS_VSYNC_ACTIVE_HIGH) &&
	    (common_flags & V4L2_MBUS_VSYNC_ACTIVE_LOW)) {
		if (pcdev->pdata->flags & RCAR_VIN_FLAG_VSYNC_LOW)
			common_flags &= ~V4L2_MBUS_VSYNC_ACTIVE_HIGH;
		else
			common_flags &= ~V4L2_MBUS_VSYNC_ACTIVE_LOW;
	}

	cfg.flags = common_flags;
	ret = v4l2_subdev_call(sd, video, s_mbus_config, &cfg);
	if (ret < 0 && ret != -ENOIOCTLCMD)
		return ret;

	value = (pcdev->field == V4L2_FIELD_NONE) ? VIN_VNDMR2_FTEV : 0;

	value |= common_flags & V4L2_MBUS_VSYNC_ACTIVE_LOW ?
		VIN_VNDMR2_VPS_ACTIVE_LOW : VIN_VNDMR2_VPS_ACTIVE_HIGH;
	value |= common_flags & V4L2_MBUS_HSYNC_ACTIVE_LOW ?
		VIN_VNDMR2_HPS_ACTIVE_LOW : VIN_VNDMR2_HPS_ACTIVE_HIGH;

	/* TODO Generate Clock Enable signal from Hsync */
	/* Required for cameras that don't output a clock enable signal, e.g.
	   the ov10635 in BT.601 mode. This is not needed when connecting
	   cameras with BT.656 output. */
#ifdef GENERATE_CLKENB_FROM_HSYNC
	value |= VIN_VNDMR2_CHS;
#endif

	/* set Data Mode Register2 */
	vin_write(pcdev, V0DMR2, value);

	ret = rcar_vin_set_rect(icd);
	if (ret < 0)
		return ret;

	mdelay(1);

	capture_restore(pcdev, capsr);

	return 0;
}

static int rcar_vin_try_bus_param(struct soc_camera_device *icd,
				  unsigned char buswidth)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	unsigned long common_flags = VIN_BUS_FLAGS;
	struct v4l2_mbus_config cfg = {.type = V4L2_MBUS_PARALLEL,};
	int ret;

	ret = v4l2_subdev_call(sd, video, g_mbus_config, &cfg);
	if (!ret)
		common_flags = soc_mbus_config_compatible(&cfg, common_flags);
	else if (ret != -ENOIOCTLCMD)
		return ret;

	if (!common_flags)
		return -EINVAL;

	return 0;
}

static const struct soc_mbus_pixelfmt rcar_vin_formats[] = {
	{
		.fourcc			= V4L2_PIX_FMT_NV16,
		.name			= "NV16",
		.bits_per_sample	= 16,
		.packing		= SOC_MBUS_PACKING_NONE,
		.order			= SOC_MBUS_ORDER_LE,
	},
	{
		.fourcc			= V4L2_PIX_FMT_YUYV,
		.name			= "YUYV",
		.bits_per_sample	= 16,
		.packing		= SOC_MBUS_PACKING_NONE,
		.order			= SOC_MBUS_ORDER_LE,
	},
	{
		.fourcc			= V4L2_PIX_FMT_UYVY,
		.name			= "UYVY",
		.bits_per_sample	= 16,
		.packing		= SOC_MBUS_PACKING_NONE,
		.order			= SOC_MBUS_ORDER_LE,
	},
	{
		.fourcc			= V4L2_PIX_FMT_RGB565,
		.name			= "RGB565",
		.bits_per_sample	= 16,
		.packing		= SOC_MBUS_PACKING_NONE,
		.order			= SOC_MBUS_ORDER_LE,
	},
	{
		.fourcc			= V4L2_PIX_FMT_RGB555X,
		.name			= "ARGB1555",
		.bits_per_sample	= 16,
		.packing		= SOC_MBUS_PACKING_NONE,
		.order			= SOC_MBUS_ORDER_LE,
	},
	{
		.fourcc			= V4L2_PIX_FMT_RGB32,
		.name			= "RGB888",
		.bits_per_sample	= 32,
		.packing		= SOC_MBUS_PACKING_NONE,
		.order			= SOC_MBUS_ORDER_LE,
	},
};

static int client_g_rect(struct v4l2_subdev *sd, struct v4l2_rect *rect);

static int rcar_vin_get_formats(struct soc_camera_device *icd, unsigned int idx,
				struct soc_camera_format_xlate *xlate)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct device *dev = icd->parent;
	struct soc_camera_host *ici = to_soc_camera_host(dev);
	struct rcar_vin_dev *pcdev = ici->priv;
	int ret, k, n;
	int formats = 0;
	struct rcar_vin_cam *cam;
	enum v4l2_mbus_pixelcode code;
	const struct soc_mbus_pixelfmt *fmt;

	ret = v4l2_subdev_call(sd, video, enum_mbus_fmt, idx, &code);
	if (ret < 0)
		/* No more formats */
		return 0;

	fmt = soc_mbus_get_fmtdesc(code);
	if (!fmt) {
		dev_err(icd->parent,
			"Invalid format code #%u: %d\n", idx, code);
		return -EINVAL;
	}

	ret = rcar_vin_try_bus_param(icd, fmt->bits_per_sample);
	if (ret < 0)
		return 0;

	if (!icd->host_priv) {
		struct v4l2_mbus_framefmt mf;
		struct v4l2_rect rect;
		struct device *dev = icd->parent;
		int shift = 0;

		/* FIXME: subwindow is lost between close / open */

		/* Cache current client geometry */
		ret = client_g_rect(sd, &rect);
		if (ret < 0)
			return ret;

		/* First time */
		ret = v4l2_subdev_call(sd, video, g_mbus_fmt, &mf);
		if (ret < 0) {
			dev_err(icd->parent,
				"SoC camera must implement g_mbus_fmt!\n");
			return ret;
		}

		while ((mf.width > 2560 || mf.height > 1920) && shift < 4) {
			/* Try 2560x1920, 1280x960, 640x480, 320x240 */
			mf.width	= 2560 >> shift;
			mf.height	= 1920 >> shift;
			ret = v4l2_subdev_call(sd, video, s_mbus_fmt, &mf);
			if (ret < 0)
				return ret;
			shift++;
		}

		if (shift == 4) {
			dev_err(dev,
				"Failed to configure the client below %ux%x\n",
				mf.width, mf.height);
			return -EIO;
		}

		dev_geo(dev, "camera fmt %ux%u\n", mf.width, mf.height);

		cam = kzalloc(sizeof(*cam), GFP_KERNEL);
		if (!cam)
			return -ENOMEM;

		/* We are called with current camera crop,
		   initialise subrect with it */
		cam->rect	= rect;
		cam->subrect	= rect;

		cam->width	= mf.width;
		cam->height	= mf.height;

		cam->width	= mf.width;
		cam->height	= mf.height;

		icd->host_priv = cam;
	} else {
		cam = icd->host_priv;
	}

	/* Beginning of a pass */
	if (!idx)
		cam->extra_fmt = NULL;

	switch (code) {
	case V4L2_MBUS_FMT_RGB888_1X24_LE:
	case V4L2_MBUS_FMT_RGB888_2X12_LE:
	case V4L2_MBUS_FMT_RGB666_1X18_LE:
	case V4L2_MBUS_FMT_YUYV8_1X16:
	case V4L2_MBUS_FMT_YUYV8_2X8:
	case V4L2_MBUS_FMT_YUYV10_2X10:
		if (cam->extra_fmt)
			break;

		/*
		 * Since the SoC camera framework doesn't allow the board to
		 * specify the MBUS format used, we have to limit the formats
		 * reported back.
		 */
		if (pcdev->pdata->format) {
			if (pcdev->pdata->format != code)
				break;
		}

		/* Add all our formats can be generated by VIN */
		cam->extra_fmt = rcar_vin_formats;

		n = ARRAY_SIZE(rcar_vin_formats);
		formats += n;
		for (k = 0; xlate && k < n; k++) {
			xlate->host_fmt	= &rcar_vin_formats[k];
			xlate->code	= code;
			xlate++;
			dev_dbg(dev, "Providing format %s using code %d\n",
				rcar_vin_formats[k].name, code);
		}
		break;
	default:
		return 0;
	}

	return formats;
}

static void rcar_vin_put_formats(struct soc_camera_device *icd)
{
	kfree(icd->host_priv);
	icd->host_priv = NULL;
}

/* Check if any dimension of r1 is smaller than respective one of r2 */
static bool is_smaller(struct v4l2_rect *r1, struct v4l2_rect *r2)
{
	return r1->width < r2->width || r1->height < r2->height;
}

/* Check if r1 fails to cover r2 */
static bool is_inside(struct v4l2_rect *r1, struct v4l2_rect *r2)
{
	return r1->left > r2->left || r1->top > r2->top ||
		r1->left + r1->width < r2->left + r2->width ||
		r1->top + r1->height < r2->top + r2->height;
}

static unsigned int scale_down(unsigned int size, unsigned int scale)
{
	return (size * 4096 + scale / 2) / scale;
}

static unsigned int calc_generic_scale(unsigned int input, unsigned int output)
{
	return (input * 4096 + output / 2) / output;
}

/* Get and store current client crop */
static int client_g_rect(struct v4l2_subdev *sd, struct v4l2_rect *rect)
{
	struct v4l2_crop crop;
	struct v4l2_cropcap cap;
	int ret;

	crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	ret = v4l2_subdev_call(sd, video, g_crop, &crop);
	if (!ret) {
		*rect = crop.c;
		return ret;
	}

	/* Camera driver doesn't support .g_crop(), assume default rectangle */
	cap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	ret = v4l2_subdev_call(sd, video, cropcap, &cap);
	if (!ret)
		*rect = cap.defrect;

	return ret;
}

/* Client crop has changed, update our sub-rectangle to remain
   within the area */
static void update_subrect(struct rcar_vin_cam *cam)
{
	struct v4l2_rect *rect = &cam->rect, *subrect = &cam->subrect;

	if (rect->width < subrect->width)
		subrect->width = rect->width;

	if (rect->height < subrect->height)
		subrect->height = rect->height;

	if (rect->left > subrect->left)
		subrect->left = rect->left;
	else if (rect->left + rect->width >
		 subrect->left + subrect->width)
		subrect->left = rect->left + rect->width -
			subrect->width;

	if (rect->top > subrect->top)
		subrect->top = rect->top;
	else if (rect->top + rect->height >
		 subrect->top + subrect->height)
		subrect->top = rect->top + rect->height -
			subrect->height;
}

/*
 * The common for both scaling and cropping iterative approach is:
 * 1. try if the client can produce exactly what requested by the user
 * 2. if (1) failed, try to double the client image until we get one big enough
 * 3. if (2) failed, try to request the maximum image
 */
static int client_s_crop(struct soc_camera_device *icd, struct v4l2_crop *crop,
			 struct v4l2_crop *cam_crop)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct v4l2_rect *rect = &crop->c, *cam_rect = &cam_crop->c;
	struct device *dev = sd->v4l2_dev->dev;
	struct rcar_vin_cam *cam = icd->host_priv;
	struct v4l2_cropcap cap;
	int ret;
	unsigned int width, height;

	v4l2_subdev_call(sd, video, s_crop, crop);
	ret = client_g_rect(sd, cam_rect);
	if (ret < 0)
		return ret;

	/*
	 * Now cam_crop contains the current camera input rectangle, and it must
	 * be within camera cropcap bounds
	 */
	if (!memcmp(rect, cam_rect, sizeof(*rect))) {
		/* Even if camera S_CROP failed, but camera rectangle matches */
		dev_dbg(dev, "Camera S_CROP successful for %dx%d@%d:%d\n",
			rect->width, rect->height, rect->left, rect->top);
		cam->rect = *cam_rect;
		return 0;
	}

	/* Try to fix cropping, that camera hasn't managed to set */
	dev_geo(dev, "Fix camera S_CROP for %dx%d@%d:%d to %dx%d@%d:%d\n",
		cam_rect->width, cam_rect->height,
		cam_rect->left, cam_rect->top,
		rect->width, rect->height, rect->left, rect->top);

	/* We need sensor maximum rectangle */
	ret = v4l2_subdev_call(sd, video, cropcap, &cap);
	if (ret < 0)
		return ret;

	/*
	 * Popular special case - some cameras can only handle fixed sizes like
	 * QVGA, VGA,... Take care to avoid infinite loop.
	 */
	width = max(cam_rect->width, 2);
	height = max(cam_rect->height, 2);

	/*
	 * Loop as long as sensor is not covering the requested rectangle and
	 * is still within its bounds
	 */
	while (!ret && (is_smaller(cam_rect, rect) ||
			is_inside(cam_rect, rect)) &&
	       (cap.bounds.width > width || cap.bounds.height > height)) {

		width *= 2;
		height *= 2;

		cam_rect->width = width;
		cam_rect->height = height;

		/*
		 * We do not know what capabilities the camera has to set up
		 * left and top borders. We could try to be smarter in iterating
		 * them, e.g., if camera current left is to the right of the
		 * target left, set it to the middle point between the current
		 * left and minimum left. But that would add too much
		 * complexity: we would have to iterate each border separately.
		 * Instead we just drop to the left and top bounds.
		 */
		if (cam_rect->left > rect->left)
			cam_rect->left = cap.bounds.left;

		if (cam_rect->left + cam_rect->width < rect->left + rect->width)
			cam_rect->width = rect->left + rect->width -
				cam_rect->left;

		if (cam_rect->top > rect->top)
			cam_rect->top = cap.bounds.top;

		if (cam_rect->top + cam_rect->height < rect->top + rect->height)
			cam_rect->height = rect->top + rect->height -
				cam_rect->top;

		v4l2_subdev_call(sd, video, s_crop, cam_crop);
		ret = client_g_rect(sd, cam_rect);
		dev_geo(dev, "Camera S_CROP %d for %dx%d@%d:%d\n", ret,
			cam_rect->width, cam_rect->height,
			cam_rect->left, cam_rect->top);
	}

	/* S_CROP must not modify the rectangle */
	if (is_smaller(cam_rect, rect) || is_inside(cam_rect, rect)) {
		/*
		 * The camera failed to configure a suitable cropping,
		 * we cannot use the current rectangle, set to max
		 */
		*cam_rect = cap.bounds;
		v4l2_subdev_call(sd, video, s_crop, cam_crop);
		ret = client_g_rect(sd, cam_rect);
		dev_geo(dev, "Camera S_CROP %d for max %dx%d@%d:%d\n", ret,
			cam_rect->width, cam_rect->height,
			cam_rect->left, cam_rect->top);
	}

	if (!ret) {
		cam->rect = *cam_rect;

		dev_geo(dev, "Update subrect for %dx%d@%d:%d to %dx%d@%d:%d\n",
			cam->subrect.width, cam->subrect.height,
			cam->subrect.left, cam->subrect.top,
			cam->rect.width, cam->rect.height,
			cam->rect.left, cam->rect.top);

		update_subrect(cam);
	}

	return ret;
}

/* Iterative s_mbus_fmt, also updates cached client crop on success */
static int client_s_fmt(struct soc_camera_device *icd,
			struct v4l2_mbus_framefmt *mf, bool vin_can_scale)
{
	struct rcar_vin_cam *cam = icd->host_priv;
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct device *dev = icd->parent;
	unsigned int width = mf->width, height = mf->height, tmp_w, tmp_h;
	unsigned int max_width, max_height;
	struct v4l2_cropcap cap;
	int ret;

	ret = v4l2_subdev_call(sd, video, s_mbus_fmt, mf);
	if (ret < 0)
		return ret;

	dev_geo(dev, "camera scaled to %ux%u\n", mf->width, mf->height);

	if ((width == mf->width && height == mf->height) || !vin_can_scale)
		goto update_cache;

	cap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	ret = v4l2_subdev_call(sd, video, cropcap, &cap);
	if (ret < 0)
		return ret;

	max_width = min(cap.bounds.width, 2560);
	max_height = min(cap.bounds.height, 1920);

	/* Camera set a format, but geometry is not precise, try to improve */
	tmp_w = mf->width;
	tmp_h = mf->height;

	/* width <= max_width && height <= max_height - guaranteed by try_fmt */
	while ((width > tmp_w || height > tmp_h) &&
	       tmp_w < max_width && tmp_h < max_height) {
		tmp_w = min(2 * tmp_w, max_width);
		tmp_h = min(2 * tmp_h, max_height);
		mf->width = tmp_w;
		mf->height = tmp_h;
		ret = v4l2_subdev_call(sd, video, s_mbus_fmt, mf);
		dev_geo(dev, "Camera scaled to %ux%u\n",
			mf->width, mf->height);
		if (ret < 0) {
			/* This shouldn't happen */
			dev_err(dev, "Client failed to set format: %d\n", ret);
			return ret;
		}
	}

update_cache:
	/* Update cache */
	ret = client_g_rect(sd, &cam->rect);
	if (ret < 0)
		return ret;

	update_subrect(cam);

	return 0;
}

/**
 * @width	- on output: user width, mapped back to input
 * @height	- on output: user height, mapped back to input
 * @mf		- in- / output camera output window
 */
static int client_scale(struct soc_camera_device *icd,
			struct v4l2_mbus_framefmt *mf,
			unsigned int *width, unsigned int *height,
			bool vin_can_scale)
{
	struct rcar_vin_cam *cam = icd->host_priv;
	struct device *dev = icd->parent;
	struct v4l2_mbus_framefmt mf_tmp = *mf;
	unsigned int scale_h, scale_v;
	int ret;

	/*
	 * 5. Apply iterative camera S_FMT for camera user window (also updates
	 *    client crop cache and the imaginary sub-rectangle).
	 */
	ret = client_s_fmt(icd, &mf_tmp, vin_can_scale);
	if (ret < 0)
		return ret;

	dev_geo(dev, "5: camera scaled to %ux%u\n",
		mf_tmp.width, mf_tmp.height);

	/* 6. Retrieve camera output window (g_fmt) */

	/* unneeded - it is already in "mf_tmp" */

	/* 7. Calculate new client scales. */
	scale_h = calc_generic_scale(cam->rect.width, mf_tmp.width);
	scale_v = calc_generic_scale(cam->rect.height, mf_tmp.height);

	mf->width	= mf_tmp.width;
	mf->height	= mf_tmp.height;
	mf->colorspace	= mf_tmp.colorspace;

	/*
	 * 8. Calculate new VIN crop - apply camera scales to previously
	 *    updated "effective" crop.
	 */
	*width = scale_down(cam->subrect.width, scale_h);
	*height = scale_down(cam->subrect.height, scale_v);

	dev_geo(dev, "8: new client sub-window %ux%u\n", *width, *height);

	return 0;
}

/*
 * VIN can crop.
 */
static int rcar_vin_set_crop(struct soc_camera_device *icd,
			     struct v4l2_crop *a)
{
	struct v4l2_rect *rect = &a->c;
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct rcar_vin_dev *pcdev = ici->priv;
	struct v4l2_crop cam_crop;
	struct rcar_vin_cam *cam = icd->host_priv;
	struct v4l2_rect *cam_rect = &cam_crop.c;
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct device *dev = icd->parent;
	struct v4l2_mbus_framefmt mf;
	u32 vnmc;
	int ret, i;

	dev_geo(dev, "S_CROP(%ux%u@%u:%u)\n", rect->width, rect->height,
		rect->left, rect->top);

	/* During camera cropping its output window can change too, stop VIN */
	vnmc = capture_save_reset(pcdev);
	dev_dbg(dev, "V0MC 0x%x\n", vnmc);

	/* 1. - 2. Apply iterative camera S_CROP for new input window. */
	ret = client_s_crop(icd, a, &cam_crop);
	if (ret < 0)
		return ret;

	dev_geo(dev, "1-2: camera cropped to %ux%u@%u:%u\n",
		cam_rect->width, cam_rect->height,
		cam_rect->left, cam_rect->top);

	/* On success cam_crop contains current camera crop */

	/* 3. Retrieve camera output window */
	ret = v4l2_subdev_call(sd, video, g_mbus_fmt, &mf);
	if (ret < 0)
		return ret;

	if (mf.width > 2560 || mf.height > 1920)
		return -EINVAL;

	/* Cache camera output window */
	cam->width	= mf.width;
	cam->height	= mf.height;

	icd->user_width  = cam->width;
	icd->user_height = cam->height;

	if (rect->left < 0)
		rect->left = 0;
	if (rect->top < 0)
		rect->top = 0;

	cam->vin_left	= rect->left & ~1;
	cam->vin_top	= rect->top & ~1;

	/* 6. Use VIN cropping to crop to the new window. */
	ret = rcar_vin_set_rect(icd);
	if (ret < 0)
		return ret;

	cam->subrect = *rect;

	dev_geo(dev, "6: VIN cropped to %ux%u@%u:%u\n",
		icd->user_width, icd->user_height,
		cam->vin_left, cam->vin_top);

	/* Restore capture */
	for (i = 0; i < MB_NUM; i++) {
		if ((pcdev->queue_buf[i]) &&
		    (pcdev->capture_status == STOPPED)) {
			vnmc |= VIN_VNMC_ME;
			break;
		}
	}
	capture_restore(pcdev, vnmc);

	/* Even if only camera cropping succeeded */
	return ret;
}

static int rcar_vin_get_crop(struct soc_camera_device *icd,
			     struct v4l2_crop *a)
{
	struct rcar_vin_cam *cam = icd->host_priv;

	a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	a->c = cam->subrect;

	return 0;
}

/*
 * Calculate real client output window by applying new scales to the current
 * client crop. New scales are calculated from the requested output format and
 * VIN crop, mapped backed onto the client input (subrect).
 */
static void calculate_client_output(struct soc_camera_device *icd,
		struct v4l2_pix_format *pix, struct v4l2_mbus_framefmt *mf)
{
	struct rcar_vin_cam *cam = icd->host_priv;
	struct device *dev = icd->parent;
	struct v4l2_rect *cam_subrect = &cam->subrect;
	unsigned int scale_v, scale_h;

	if (cam_subrect->width == cam->rect.width &&
	    cam_subrect->height == cam->rect.height) {
		/* No sub-cropping */
		mf->width	= pix->width;
		mf->height	= pix->height;
		return;
	}

	/* 1.-2. Current camera scales and subwin - cached. */

	dev_geo(dev, "2: subwin %ux%u@%u:%u\n",
		cam_subrect->width, cam_subrect->height,
		cam_subrect->left, cam_subrect->top);

	/*
	 * 3. Calculate new combined scales from input sub-window to requested
	 *    user window.
	 */

	scale_h = calc_generic_scale(cam_subrect->width, pix->width);
	scale_v = calc_generic_scale(cam_subrect->height, pix->height);

	dev_geo(dev, "3: scales %u:%u\n", scale_h, scale_v);

	/*
	 * 4. Calculate client output window by applying combined scales to real
	 *    input window.
	 */
	mf->width	= scale_down(cam->rect.width, scale_h);
	mf->height	= scale_down(cam->rect.height, scale_v);
}

/* Similar to set_crop multistage iterative algorithm */
static int rcar_vin_set_fmt(struct soc_camera_device *icd,
			    struct v4l2_format *f)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct rcar_vin_dev *pcdev = ici->priv;
	struct rcar_vin_cam *cam = icd->host_priv;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct v4l2_mbus_framefmt mf;
	struct device *dev = icd->parent;
	__u32 pixfmt = pix->pixelformat;
	const struct soc_camera_format_xlate *xlate;
	unsigned int vin_sub_width = 0, vin_sub_height = 0;
	u16 scale_v, scale_h;
	int ret;
	bool can_scale;
	bool data_through;
	enum v4l2_field field;

	dev_geo(dev, "S_FMT(pix=0x%x, %ux%u)\n",
		pixfmt, pix->width, pix->height);

	switch (pix->field) {
	default:
		pix->field = V4L2_FIELD_NONE;
		/* fall-through */
	case V4L2_FIELD_NONE:
	case V4L2_FIELD_TOP:
	case V4L2_FIELD_BOTTOM:
	case V4L2_FIELD_INTERLACED_TB:
	case V4L2_FIELD_INTERLACED_BT:
		field = pix->field;
		break;
	case V4L2_FIELD_INTERLACED:
		field = V4L2_FIELD_INTERLACED_TB;
		break;
	}

	xlate = soc_camera_xlate_by_fourcc(icd, pixfmt);
	if (!xlate) {
		dev_warn(dev, "Format %x not found\n", pixfmt);
		return -EINVAL;
	}
	/* 1.-4. Calculate client output geometry */
	calculate_client_output(icd, &f->fmt.pix, &mf);
	mf.field	= pix->field;
	mf.colorspace	= pix->colorspace;
	mf.code		= xlate->code;

	data_through = (pixfmt == V4L2_PIX_FMT_RGB32);
	can_scale = (!data_through && (pixfmt != V4L2_PIX_FMT_NV16));

	dev_geo(dev, "4: request camera output %ux%u\n", mf.width, mf.height);

	/* 5. - 9. */
	ret = client_scale(icd, &mf, &vin_sub_width, &vin_sub_height,
			can_scale);

	dev_geo(dev, "5-9: client scale return %d\n", ret);

	/* Done with the camera. Now see if we can improve the result */

	dev_geo(dev, "Camera %d fmt %ux%u, requested %ux%u\n",
		ret, mf.width, mf.height, pix->width, pix->height);
	if (ret < 0)
		return ret;

	if (mf.code != xlate->code)
		return -EINVAL;

	/* 9. Prepare VIN crop */
	cam->width = mf.width;
	cam->height = mf.height;

	/* 10. Use VIN scaling to scale to the requested user window. */

	/* We cannot scale up */
	if (pix->width > vin_sub_width)
		vin_sub_width = pix->width;

	if (pix->height > vin_sub_height)
		vin_sub_height = pix->height;

	pix->colorspace = mf.colorspace;

	if (can_scale) {
		/* Scale pix->{width x height} down to width x height */
		scale_h		= calc_scale(vin_sub_width, &pix->width);
		scale_v		= calc_scale(vin_sub_height, &pix->height);
	} else {
		pix->width	= vin_sub_width;
		pix->height	= vin_sub_height;
		scale_h		= 0;
		scale_v		= 0;
	}

	/*
	 * We have calculated CFLCR, the actual configuration will be performed
	 * in rcar_vin_set_bus_param()
	 */

	dev_geo(dev, "10: W: %u : 0x%x = %u, H: %u : 0x%x = %u\n",
		vin_sub_width, scale_h, pix->width,
		vin_sub_height, scale_v, pix->height);

	cam->code		= xlate->code;
	icd->current_fmt	= xlate;

	pcdev->field = field;
	pcdev->data_through = data_through;

	return 0;
}

static int rcar_vin_try_fmt(struct soc_camera_device *icd,
			    struct v4l2_format *f)
{
	const struct soc_camera_format_xlate *xlate;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct v4l2_mbus_framefmt mf;
	__u32 pixfmt = pix->pixelformat;
	int width, height;
	int ret;

	xlate = soc_camera_xlate_by_fourcc(icd, pixfmt);
	if (!xlate) {
		dev_warn(icd->parent, "Format %x not found\n", pixfmt);
		return -EINVAL;
	}

	/* FIXME: calculate using depth and bus width */

	v4l_bound_align_image(&pix->width, 2, 2560, 1,
			      &pix->height, 4, 1920, 2, 0);

	width = pix->width;
	height = pix->height;

	pix->bytesperline = soc_mbus_bytes_per_line(width, xlate->host_fmt);
	if ((int)pix->bytesperline < 0)
		return pix->bytesperline;
	pix->sizeimage = height * pix->bytesperline;

	/* limit to sensor capabilities */
	mf.width	= pix->width;
	mf.height	= pix->height;
	mf.field	= pix->field;
	mf.code		= xlate->code;
	mf.colorspace	= pix->colorspace;

	ret = v4l2_subdev_call(sd, video, try_mbus_fmt, &mf);
	if (ret < 0)
		return ret;

	pix->width	= mf.width;
	pix->height	= mf.height;
	pix->field	= mf.field;
	pix->colorspace	= mf.colorspace;

	switch (pixfmt) {
	case V4L2_PIX_FMT_NV16:
		/* FIXME: check against rect_max after converting soc-camera */
		/* We can scale precisely, need a bigger image from camera */
		if (pix->width < width || pix->height < height) {
			/*
			 * We presume, the sensor behaves sanely, i.e., if
			 * requested a bigger rectangle, it will not return a
			 * smaller one.
			 */
			mf.width = 2560;
			mf.height = 1920;
			ret = v4l2_subdev_call(sd, video, try_mbus_fmt, &mf);
			if (ret < 0) {
				/* Shouldn't actually happen... */
				dev_err(icd->parent,
					"FIXME: client try_fmt() = %d\n", ret);
				return ret;
			}
		}
		/* We will scale exactly */
		if (mf.width > width)
			pix->width = width;
		if (mf.height > height)
			pix->height = height;
	}

	return ret;
}

static unsigned int rcar_vin_poll(struct file *file, poll_table *pt)
{
	struct soc_camera_device *icd = file->private_data;

	return vb2_poll(&icd->vb2_vidq, file, pt);
}

static int rcar_vin_querycap(struct soc_camera_host *ici,
			     struct v4l2_capability *cap)
{
	strlcpy(cap->card, "R_Car_VIN", sizeof(cap->card));
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	return 0;
}

static int rcar_vin_init_videobuf(struct vb2_queue *q,
				  struct soc_camera_device *icd)
{
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_USERPTR;
	q->drv_priv = icd;
	q->ops = &rcar_vin_videobuf_ops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->buf_struct_size = sizeof(struct rcar_vin_buffer);

	return vb2_queue_init(q);
}

static struct soc_camera_host_ops rcar_vin_host_ops = {
	.owner		= THIS_MODULE,
	.add		= rcar_vin_add_device,
	.remove		= rcar_vin_remove_device,
	.get_formats	= rcar_vin_get_formats,
	.put_formats	= rcar_vin_put_formats,
	.get_crop	= rcar_vin_get_crop,
	.set_crop	= rcar_vin_set_crop,
	.set_fmt	= rcar_vin_set_fmt,
	.try_fmt	= rcar_vin_try_fmt,
	.poll		= rcar_vin_poll,
	.querycap	= rcar_vin_querycap,
	.set_bus_param	= rcar_vin_set_bus_param,
	.init_videobuf2	= rcar_vin_init_videobuf,
};

#if RCARVIN_SUPPORT_TO_CHECK_REGS
struct rcar_vin_test_of_reg {
	char *name;
	unsigned int attr;
	unsigned int offset;
	unsigned int mask;
	unsigned int value;
};

static int rcar_vin_test_of_reg(struct rcar_vin_dev *pcdev, int mode)
{
	int index = 0;
	unsigned int real;
	int result = 1;
	void __iomem *base = pcdev->base;
	struct rcar_vin_test_of_reg checktable1[] = {
		/* name, attr, offset, mask, value */
		{"V0MC", 0, V0MC, 0, 0x00000000},
		{"V0FC", 0, V0FC, 0, 0x00000000},
		{"V0SLPrC", 0, V0SLPrC, 0, 0x00000000},
		{"V0ELPrC", 0, V0ELPrC, 0, 0x00000000},
		{"V0SPPrC", 0, V0SPPrC, 0, 0x00000000},
		{"V0EPPrC", 0, V0EPPrC, 0, 0x00000000},
		{"V0SLPoC", 0, V0SLPoC, 0, 0x00000000},
		{"V0ELPoC", 0, V0ELPoC, 0, 0x00000000},
		{"V0SPPoC", 0, V0SPPoC, 0, 0x00000000},
		{"V0EPPoC", 0, V0EPPoC, 0, 0x00000000},
		{"V0IS", 0, V0IS, 0, 0x00000000},
		{"V0MB1", 0, V0MB1, 0, 0x00000000},
		{"V0MB2", 0, V0MB2, 0, 0x00000000},
		{"V0MB3", 0, V0MB3, 0, 0x00000000},
		{"V0LC", 0, V0LC, 0, 0x00000000},
		{"V0IE", 0, V0IE, 0, 0x00000000},
		{"V0SI", 0, V0SI, 0, 0x00000000},
		{"V0YS", 0, V0YS, 0, 0x00000000},
		{"V0XS", 0, V0XS, 0, 0x00000000},
		{"V0DMR", 0, V0DMR, 0, 0x00000000},
		{"V0DMR2", 0, V0DMR2, 0, 0x00000000},
		{"V0CSCC1", 0, V0CSCC1, 0, 0x01291080},
		{"V0CSCC2", 0, V0CSCC2, 0, 0x019800D0},
		{"V0CSCC3", 0, V0CSCC3, 0, 0x00640204},
		{"V0C1A", 0, V0C1A, 0, 0x00000000},
		{"V0C1B", 0, V0C1B, 0, 0x00000000},
		{"V0C1C", 0, V0C1C, 0, 0x00000000},
		{"V0C2A", 0, V0C2A, 0, 0x00000000},
		{"V0C2B", 0, V0C2B, 0, 0x00000000},
		{"V0C2C", 0, V0C2C, 0, 0x00000000},
		{"V0C3A", 0, V0C3A, 0, 0x00000000},
		{"V0C3B", 0, V0C3B, 0, 0x00000000},
		{"V0C3C", 0, V0C3C, 0, 0x00000000},
		{"V0C4A", 0, V0C4A, 0, 0x00000000},
		{"V0C4B", 0, V0C4B, 0, 0x00000000},
		{"V0C4C", 0, V0C4C, 0, 0x00000000},
		{"V0C5A", 0, V0C5A, 0, 0x00000000},
		{"V0C5B", 0, V0C5B, 0, 0x00000000},
		{"V0C5C", 0, V0C5C, 0, 0x00000000},
		{"V0C6A", 0, V0C6A, 0, 0x00000000},
		{"V0C6B", 0, V0C6B, 0, 0x00000000},
		{"V0C6C", 0, V0C6C, 0, 0x00000000},
		{"V0C7A", 0, V0C7A, 0, 0x00000000},
		{"V0C7B", 0, V0C7B, 0, 0x00000000},
		{"V0C7C", 0, V0C7C, 0, 0x00000000},
		{"V0C8A", 0, V0C8A, 0, 0x00000000},
		{"V0C8B", 0, V0C8B, 0, 0x00000000},
		{"V0C8C", 0, V0C8C, 0, 0x00000000},
		{NULL, 0xFFFFFFFF, 0, 0, 0}
	};

	pr_alert("<LOG msg=\"Start rcar_vin_test_of_reg\" base=\"0x%x\">\n",
		(unsigned long)base);

	while (checktable1[index].name != NULL) {
		real = vin_read(pcdev, checktable1[index].offset);
		if (real != checktable1[index].value) {
			pr_alert("<check type=\"initial value\" name=\"%s\" real=\"0x%x\" expected=\"0x%x\"/>\n",
		checktable1[index].name, real, checktable1[index].value);

			result = 0;
		}
		index++;
	}
	if (result)
		pr_alert("<Result summary=\"Pass\"/>\n");
	else
		pr_alert("<Result summary=\"Fail\"/>\n");

	pr_alert("</LOG>\n");

	return 0;
}
#endif /* RCARVIN_SUPPORT_TO_CHECK_REGS */

static int __devinit rcar_vin_probe(struct platform_device *pdev)
{
	struct rcar_vin_dev *pcdev;
	struct resource *res;
	void __iomem *base;
	unsigned int irq, i;
	int err = 0;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);
	if (!res || (int)irq <= 0) {
		dev_err(&pdev->dev, "Not enough VIN platform resources.\n");
		err = -ENODEV;
		goto exit;
	}

	pcdev = kzalloc(sizeof(*pcdev), GFP_KERNEL);
	if (!pcdev) {
		dev_err(&pdev->dev, "Could not allocate pcdev\n");
		err = -ENOMEM;
		goto exit;
	}

	INIT_LIST_HEAD(&pcdev->capture);
	spin_lock_init(&pcdev->lock);

	pcdev->pdata = pdev->dev.platform_data;
	if (!pcdev->pdata) {
		err = -EINVAL;
		dev_err(&pdev->dev, "VIN platform data not set.\n");
		goto exit_kfree;
	}

	base = ioremap_nocache(res->start, resource_size(res));
	if (!base) {
		err = -ENXIO;
		dev_err(&pdev->dev, "Unable to ioremap VIN registers.\n");
		goto exit_kfree;
	}

	pcdev->irq = irq;
	pcdev->base = base;
	pcdev->capture_status = STOPPED;
	for (i = 0; i < MB_NUM; i++)
		pcdev->queue_buf[i] = NULL;

#if RCARVIN_SUPPORT_TO_CHECK_REGS
	if (rcar_vin_test_of_reg(pcdev, 0) < 0)
		goto exit_iounmap;
#endif /* RCARVIN_SUPPORT_TO_CHECK_REGS */

	/* request irq */
	err = request_irq(pcdev->irq, rcar_vin_irq, IRQF_DISABLED,
			  dev_name(&pdev->dev), pcdev);
	if (err) {
		dev_err(&pdev->dev, "Unable to register VIN interrupt.\n");
		goto exit_iounmap;
	}

#ifdef CONFIG_PM
	pm_suspend_ignore_children(&pdev->dev, true);
	pm_runtime_enable(&pdev->dev);
	pm_runtime_resume(&pdev->dev);
#endif /* CONFIG_PM */

	pcdev->ici.priv = pcdev;
	pcdev->ici.v4l2_dev.dev = &pdev->dev;
	pcdev->ici.nr = pdev->id;
	pcdev->ici.drv_name = dev_name(&pdev->dev);
	pcdev->ici.ops = &rcar_vin_host_ops;

	pcdev->alloc_ctx = vb2_dma_contig_init_ctx(&pdev->dev);
	if (IS_ERR(pcdev->alloc_ctx)) {
		err = PTR_ERR(pcdev->alloc_ctx);
		goto exit_free_clk;
	}
	err = soc_camera_host_register(&pcdev->ici);
	if (err)
		goto exit_free_ctx;

	return 0;

exit_free_ctx:
	vb2_dma_contig_cleanup_ctx(pcdev->alloc_ctx);
exit_free_clk:
#ifdef CONFIG_PM
	pm_runtime_disable(&pdev->dev);
#endif /* CONFIG_PM */
	free_irq(pcdev->irq, pcdev);
exit_iounmap:
	iounmap(base);
exit_kfree:
	kfree(pcdev);
exit:
	return err;
}

static int __devexit rcar_vin_remove(struct platform_device *pdev)
{
	struct soc_camera_host *soc_host = to_soc_camera_host(&pdev->dev);
	struct rcar_vin_dev *pcdev = container_of(soc_host,
					struct rcar_vin_dev, ici);

	soc_camera_host_unregister(soc_host);
#ifdef CONFIG_PM
	pm_runtime_disable(&pdev->dev);
#endif /* CONFIG_PM */
	free_irq(pcdev->irq, pcdev);
	if (platform_get_resource(pdev, IORESOURCE_MEM, 1))
		dma_release_declared_memory(&pdev->dev);

	iounmap(pcdev->base);
	vb2_dma_contig_cleanup_ctx(pcdev->alloc_ctx);
	kfree(pcdev);
	return 0;
}

#ifdef CONFIG_PM
static int rcar_vin_runtime_nop(struct device *dev)
{
	/* Runtime PM callback shared between ->runtime_suspend()
	 * and ->runtime_resume(). Simply returns success.
	 *
	 * This driver re-initializes all registers after
	 * pm_runtime_get_sync() anyway so there is no need
	 * to save and restore registers here.
	 */
	return 0;
}

static const struct dev_pm_ops rcar_vin_dev_pm_ops = {
	.runtime_suspend = rcar_vin_runtime_nop,
	.runtime_resume = rcar_vin_runtime_nop,
};
#endif /* CONFIG_PM */

static struct platform_driver rcar_vin_driver = {
	.driver		= {
		.name	= "rcar_vin",
#ifdef CONFIG_PM
		.pm	= &rcar_vin_dev_pm_ops,
#endif /* CONFIG_PM */
	},
	.probe		= rcar_vin_probe,
	.remove		= __devexit_p(rcar_vin_remove),
};

module_platform_driver(rcar_vin_driver);

MODULE_DESCRIPTION("R-Car VIN Unit driver");
MODULE_AUTHOR("Magnus Damm");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.0.6");
MODULE_ALIAS("platform:rcar_vin");

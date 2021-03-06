/*
 * arch/arm/mach-rcar/hpb-dmae.c
 *
 * Copyright (C) 2011-2012 Renesas Electronics Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/rcar-hpbdma.h>
#include <mach/hardware.h>
#include <mach/hpb-dmae.h>
#include <mach/irqs.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>

/* Transmit sizes and respective register values */
enum {
	XMIT_SZ_8BIT		= 0,
	XMIT_SZ_16BIT		= 1,
	XMIT_SZ_32BIT		= 2,
};

/* log2(size / 8) - used to calculate number of transfers */
#define TS_SHIFT {			\
	[XMIT_SZ_8BIT]		= 0,	\
	[XMIT_SZ_16BIT]		= 1,	\
	[XMIT_SZ_32BIT]		= 2,	\
}

#define SSI_CHAN_OFFSET	0x40
#define SSI_TX_OFFSET	0x8
#define SSI_RX_OFFSET	0xC

static const struct hpb_dmae_slave_config rcar_dmae_slaves[] = {
	{
		.id	= HPBDMA_SLAVE_HSPI2_TX,
		.addr	= 0xfffc6000 + 0x0C,
		.dcr	= SPDS_8BIT | DMDL | DPDS_8BIT,
		.port	= 0x1313,
		.flags	= 0,
		.dma_ch	= 4,
	}, {
		.id	= HPBDMA_SLAVE_HSPI2_RX,
		.addr	= 0xfffc6000 + 0x13,
		.dcr	= SMDL | SPDS_8BIT | DPDS_8BIT,
		.port	= 0x1313,
		.flags	= 0,
		.dma_ch	= 0,
	}, {
		.id	= HPBDMA_SLAVE_HSPI0_TX,
		.addr	= 0xfffc7000 + 0x0C,
		.dcr	= SPDS_8BIT | DMDL | DPDS_8BIT,
		.port	= 0x0A0A,
		.flags	= 0,
		.dma_ch	= 5,
	}, {
		.id	= HPBDMA_SLAVE_HSPI0_RX,
		.addr	= 0xfffc7000 + 0x13,
		.dcr	= SMDL | SPDS_8BIT | DPDS_8BIT,
		.port	= 0x0A0A,
		.flags	= 0,
		.dma_ch	= 1,
	}, {
		.id	= HPBDMA_SLAVE_SDHI0_TX,
		.addr	= 0xffe4c000 + 0x30,
		.dcr	= SPDS_16BIT | DMDL | DPDS_16BIT,
		.mdr	= ASYNC_MD_SINGLE | ASYNC_BTMD_NBURST,
		.port	= 0x0D0C,
		.flags	= HPB_DMAE_SET_ASYNC_RESET | HPB_DMAE_SET_ASYNC_MODE,
		.dma_ch	= 21,
	}, {
		.id	= HPBDMA_SLAVE_SDHI0_RX,
		.addr	= 0xffe4c000 + 0x30,
		.dcr	= SMDL | SPDS_16BIT | DPDS_16BIT,
		.mdr	= ASYNC_MD_SINGLE | ASYNC_BTMD_NBURST,
		.port	= 0x0D0C,
		.flags	= HPB_DMAE_SET_ASYNC_RESET | HPB_DMAE_SET_ASYNC_MODE |
				HPB_DMAE_SET_SHPT1,
		.dma_ch	= 22,
	}, {
		.id	= HPBDMA_SLAVE_MMC0_TX,
		.addr	= 0xffe5a000 + 0x34,
		.dcr	= SPDS_32BIT | DMDL | DPDS_32BIT,
		.mdr	= ASYNC_MD_MULTI | ASYNC_BTMD_NBURST,
		.port	= 0x0100,
		.flags  = HPB_DMAE_SET_ASYNC_RESET | HPB_DMAE_SET_ASYNC_MODE,
		.dma_ch	= 24,
	}, {
		.id	= HPBDMA_SLAVE_MMC0_RX,
		.addr	= 0xffe5a000 + 0x34,
		.dcr	= SPDS_32BIT | SMDL | DPDS_32BIT,
		.mdr	= ASYNC_MD_MULTI | ASYNC_BTMD_NBURST,
		.port	= 0x0100,
		.flags  = HPB_DMAE_SET_ASYNC_RESET | HPB_DMAE_SET_ASYNC_MODE |
				HPB_DMAE_SET_SHPT1,
		.dma_ch	= 24,
	}, {
		.id	= HPBDMA_SLAVE_SSI0_TX_ST,
		.addr	= 0xffd91000 + (0*SSI_CHAN_OFFSET) + SSI_TX_OFFSET,
		.dcr	= CT | DIP | SPDS_32BIT | DMDL | DPDS_32BIT,
		.port	= 0x0000,
		.flags	= 0,
		.dma_ch	= 28,
	}, {
		.id	= HPBDMA_SLAVE_SSI1_RX_ST,
		.addr	= 0xffd91000 + (1*SSI_CHAN_OFFSET) + SSI_RX_OFFSET,
		.dcr	= CT | DIP | SMDL | SPDS_32BIT | DPDAM | DPDS_32BIT,
		.port	= 0x0101,
		.flags	= 0,
		.dma_ch	= 29,
	}, {
		.id	= HPBDMA_SLAVE_SSI7_TX_ST,
		.addr	= 0xffd91000 + (7*SSI_CHAN_OFFSET) + SSI_TX_OFFSET,
		.dcr	= CT | DIP | SPDS_32BIT | DMDL | DPDS_32BIT,
		.port	= 0x0707,
		.flags	= 0,
		.dma_ch	= 35,
	}, {
		.id	= HPBDMA_SLAVE_SSI8_RX_ST,
		.addr	= 0xffd91000 + (8*SSI_CHAN_OFFSET) + SSI_RX_OFFSET,
		.dcr	= CT | DIP | SMDL | SPDS_32BIT | DPDAM | DPDS_32BIT,
		.port	= 0x0808,
		.flags	= 0,
		.dma_ch	= 36,
	}, {
		.id	= HPBDMA_SLAVE_MMC1_TX,
		.addr	= 0xffe5b000 + 0x34,
		.dcr	= SPDS_32BIT | DMDL | DPDS_32BIT,
		.mdr	= ASYNC_MD_MULTI | ASYNC_BTMD_NBURST,
		.port	= 0x0100,
		.flags  = HPB_DMAE_SET_ASYNC_RESET | HPB_DMAE_SET_ASYNC_MODE,
		.dma_ch	= 43,
	}, {
		.id	= HPBDMA_SLAVE_MMC1_RX,
		.addr	= 0xffe5b000 + 0x34,
		.dcr	= SPDS_32BIT | SMDL | DPDS_32BIT,
		.mdr	= ASYNC_MD_MULTI | ASYNC_BTMD_NBURST,
		.port	= 0x0100,
		.flags  = HPB_DMAE_SET_ASYNC_RESET | HPB_DMAE_SET_ASYNC_MODE |
				HPB_DMAE_SET_SHPT1,
		.dma_ch	= 43,
	},
};

#define DMAE_CHANNEL(_irq, _s_id)	\
	{						\
		.ch_irq		= _irq,			\
		.s_id		= _s_id,		\
	}

/* comment out for not using Ch */
static const struct hpb_dmae_channel rcar_dmae_channels[] = {
	/* ch.0 HSPI2 */
	DMAE_CHANNEL(IRQ_DMAC_H(0), HPBDMA_SLAVE_HSPI2_RX),
	/* ch.4 HSPI2 */
	DMAE_CHANNEL(IRQ_DMAC_H(4), HPBDMA_SLAVE_HSPI2_TX),
	/* ch.1 HSPI0 */
	DMAE_CHANNEL(IRQ_DMAC_H(1), HPBDMA_SLAVE_HSPI0_RX),
	/* ch.5 HSPI0 */
	DMAE_CHANNEL(IRQ_DMAC_H(5), HPBDMA_SLAVE_HSPI0_TX),
	/* ch.21 SD0 */
	DMAE_CHANNEL(IRQ_DMAC_H(21), HPBDMA_SLAVE_SDHI0_TX),
	/* ch.22 SD0 */
	DMAE_CHANNEL(IRQ_DMAC_H(22), HPBDMA_SLAVE_SDHI0_RX),
	/* ch.24 MMC0 */
	DMAE_CHANNEL(IRQ_DMAC_H(24), HPBDMA_SLAVE_MMC0_TX),
	DMAE_CHANNEL(IRQ_DMAC_H(24), HPBDMA_SLAVE_MMC0_RX),
	/* ch.28 SSI0 */
	DMAE_CHANNEL(IRQ_DMAC_H(28), HPBDMA_SLAVE_SSI0_TX_ST),
	/* ch.29 SSI1 */
	DMAE_CHANNEL(IRQ_DMAC_H(29), HPBDMA_SLAVE_SSI1_RX_ST),
	/* ch.35 SSI7 */
	DMAE_CHANNEL(IRQ_DMAC_H(35), HPBDMA_SLAVE_SSI7_TX_ST),
	/* ch.36 SSI8 */
	DMAE_CHANNEL(IRQ_DMAC_H(36), HPBDMA_SLAVE_SSI8_RX_ST),
	/* ch.43 MMC1 */
	/*DMAE_CHANNEL(IRQ_DMAC_H(43), HPBDMA_SLAVE_MMC1_TX),*/
	/*DMAE_CHANNEL(IRQ_DMAC_H(43), HPBDMA_SLAVE_MMC1_RX),*/
};

static const unsigned int ts_shift[] = TS_SHIFT;

static struct hpb_dmae_pdata dma_platform_data = {
	.slave		= rcar_dmae_slaves,
	.slave_num	= ARRAY_SIZE(rcar_dmae_slaves),
	.channel	= rcar_dmae_channels,
	.channel_num	= ARRAY_SIZE(rcar_dmae_channels),
	.ts_shift	= ts_shift,
	.ts_shift_num	= ARRAY_SIZE(ts_shift),
};

/* Resource order important! */
static struct resource rcar_dmae_resources[] = {
	{
		/* Channel registers */
		.start	= 0xffc08000,
		.end	= 0xffc08000 + (0x40 * HPB_DMA_MAXCH) - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		/* Common registers */
		.start	= 0xffc09000,
		.end	= 0xffc09170 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		/* Asynchronous reset registers */
		.start	= 0xffc00300,
		.end	= 0xffc00304 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		/* Asynchronous mode registers */
		.start	= 0xffc00400,
		.end	= 0xffc00404 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		/* IRQ for channels DMA ch.20 - ch.43 */
		.start	= IRQ_DMAC_H(HPB_DMA_USE_START_CH),
		.end	= IRQ_DMAC_H(HPB_DMA_USE_END_CH),
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device rcar_dma_device = {
	.name		= "hpb-dma-engine",
	.id		= 0,
	.resource	= rcar_dmae_resources,
	.num_resources	= ARRAY_SIZE(rcar_dmae_resources),
	.dev		= {
		.platform_data	= &dma_platform_data,
	},
};

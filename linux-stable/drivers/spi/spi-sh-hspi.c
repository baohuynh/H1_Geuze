/*
 * SuperH HSPI bus driver
 *
 * Copyright (C) 2011  Kuninori Morimoto
 *
 * Based on spi-sh.c:
 * Based on pxa2xx_spi.c:
 * Copyright (C) 2011 Renesas Solutions Corp.
 * Copyright (C) 2005 Stephen Street / StreetFire Sound Labs
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/list.h>
#include <linux/workqueue.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/io.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/rcar-hpbdma.h>
#include <linux/spi/spi.h>
#include <linux/spi/sh_hspi.h>

#define SPCR	0x00
#define SPSR	0x04
#define SPSCR	0x08
#define SPTBR	0x0C
#define SPRBR	0x10
#define SPCR2	0x14

#define SPI_DMA_MSG_SIZE_THRESHOLD 8
#define HSPI_DEBUG

/* SPSR */
#define RXFL	(1 << 2)

#define hspi2info(h)	(h->dev->platform_data)

struct hspi_priv {
	void __iomem *addr;
	void *platform_data;
	u32 max_speed_hz;
	struct spi_master *master;
	struct list_head queue;
	struct work_struct ws;
	wait_queue_head_t wait;
	spinlock_t lock;
	struct device *dev;
	struct clk *clk;
	unsigned long spsr;

	/* for dmaengine */
	unsigned int dma_is_used;
	struct hpb_dmae_slave dma_tx;
	struct hpb_dmae_slave dma_rx;
	struct dma_chan *chan_tx;
	struct dma_chan *chan_rx;
	int irq;

	unsigned dma_tx_callbacked:1;
	unsigned dma_rx_callbacked:1;
};

/*
 *		basic function
 */
static void hspi_write(struct hspi_priv *hspi, int reg, u32 val)
{
	iowrite32(val, hspi->addr + reg);
}

static u32 hspi_read(struct hspi_priv *hspi, int reg)
{
	return ioread32(hspi->addr + reg);
}

static void hspi_bit_set(struct hspi_priv *hspi, int reg, u32 mask, u32 set)
{
	u32 val = hspi_read(hspi, reg);

	val &= ~mask;
	val |= set;

	hspi_write(hspi, reg, val);
}

/*
 *		spi master function
 */
static int hspi_prepare_transfer(struct spi_master *master)
{
	struct hspi_priv *hspi = spi_master_get_devdata(master);

	pm_runtime_get_sync(hspi->dev);
	return 0;
}

static int hspi_unprepare_transfer(struct spi_master *master)
{
	struct hspi_priv *hspi = spi_master_get_devdata(master);

	pm_runtime_put_sync(hspi->dev);
	return 0;
}

#define hspi_hw_cs_enable(hspi)		hspi_hw_cs_ctrl(hspi, 0)
#define hspi_hw_cs_disable(hspi)	hspi_hw_cs_ctrl(hspi, 1)
static void hspi_hw_cs_ctrl(struct hspi_priv *hspi, int hi)
{
	hspi_bit_set(hspi, SPSCR, (1 << 6), (hi) << 6);
}

static void hspi_enable_irq(struct hspi_priv *hspi, u32 enable)
{
	hspi_bit_set(hspi, SPSCR, 0, enable);
}

static void hspi_disable_irq(struct hspi_priv *hspi, u32 disable)
{
	hspi_bit_set(hspi, SPSCR, disable, 0);
}

static int hspi_wait_for_interrupt(struct hspi_priv *hspi, u32 wait_mask,
				   u32 enable_bit)
{
	int ret;

	hspi->spsr = hspi_read(hspi, SPSR);
	hspi_enable_irq(hspi, enable_bit);
	ret = wait_event_timeout(hspi->wait, hspi->spsr & wait_mask, HZ);
	if (ret == 0 && !(hspi->spsr & wait_mask))
		return -ETIMEDOUT;

	return 0;
}

/* hspi_reset: reset HSPI to a reset state
 * One of the following conditions makes a HSPI reset:
 * (1) If any of the FBS, CLKP, IDIV or CLKC (in SPCR register) bit values are changed
 * (2) If any of the FFEN, LMSB, CSA or MASL (in SPSCR register) bit values are changed
 *
 * This function simply changes FBS bit value from 1 to 0 to creat a HSPI reset
*/
static void hspi_reset(struct hspi_priv *hspi)
{
	hspi_bit_set(hspi, SPCR, 0, (1 << 7)); /* FBS = 1 */
	hspi_bit_set(hspi, SPCR, (1 << 7), 0); /* FBS = 0 */

}

static void hspi_hw_setup(struct hspi_priv *hspi,
			  struct spi_message *msg,
			  struct spi_transfer *t)
{
	struct spi_device *spi = msg->spi;
	u32 target_rate;
	u32 spcr, spcr2, idiv_clk;
	u32 rate, best_rate, min, tmp;
	u32 ratio;

	/* Before of all, reset the system.
	 * If not, rx data in a DMA transfer after a PIO tranfer won't be correct.
	*/
	hspi_reset(hspi);
	
	target_rate = t ? t->speed_hz : 0;
	if (!target_rate)
		target_rate = spi->max_speed_hz;

	ratio = clk_get_rate(hspi->clk) / target_rate;
	
	/*
	* find best IDIV/CLKCx settings
	*/
	min = ~0;
	best_rate = 0;
	spcr2 = 0;
	spcr = 0;
	if(ratio < 32){
		for (idiv_clk = 9; idiv_clk <= 0x3F; idiv_clk++) {
			rate = clk_get_rate(hspi->clk);
			rate = rate / (idiv_clk  + 1) ;
	
			/* save best settings */
			tmp = abs(target_rate - rate);
			if (tmp < min) {
				min = tmp;
				spcr2 = idiv_clk;
				spcr2 |= (((idiv_clk + 1)/2) << 6);
				spcr2 |= (1 << 15); /* High speed transfer enable */
				best_rate = rate;
			}
		}
	}else{
		for (idiv_clk = 0x00; idiv_clk <= 0x3F; idiv_clk++) {
			rate = clk_get_rate(hspi->clk);
	
			/* IDIV calculation */
			if (idiv_clk & (1 << 5))
				rate /= 128;
			else
				rate /= 16;
	
			/* CLKCx calculation */
			rate /= (((idiv_clk & 0x1F) + 1) * 2) ;
	
			/* save best settings */
			tmp = abs(target_rate - rate);
			if (tmp < min) {
				min = tmp;
				spcr = idiv_clk;
				best_rate = rate;
			}
		}
	}

	if (spi->mode & SPI_CPHA)
		spcr |= 1 << 7;
	if (spi->mode & SPI_CPOL)
		spcr |= 1 << 6;

#ifdef HSPI_DEBUG
	dev_info(&hspi->master->dev, "target_rate/best_rate = %d/%d\n", target_rate, best_rate);
#endif

	hspi_write(hspi, SPCR, spcr);
	hspi_write(hspi, SPCR2, spcr2);
	hspi_write(hspi, SPSR, 0x0);
	hspi_write(hspi, SPSCR, 0x21); /* master mode / CS control */
}

static int hspi_transfer_pio(struct hspi_priv *hspi, struct spi_message *msg,
			 struct spi_transfer *t)
{
	u32 tx;
	u32 rx;
	int i, ret = 0;

#ifdef HSPI_DEBUG
	dev_info(&hspi->master->dev, "Use PIO mode when transfer.\n");
#endif

	for(i = 0; i < t->len; i++) {
		tx = 0;
		if (t->tx_buf)
			tx = (u32)((u8 *)t->tx_buf)[i];

		hspi_write(hspi, SPTBR, tx);
		if (hspi_wait_for_interrupt(hspi,(1 << 1), (1 << 4)) < 0) {
			dev_err(&hspi->master->dev,
				"%s: tx empty timeout\n", __func__);
			return -ETIMEDOUT;
		}

		rx = hspi_read(hspi, SPRBR);
		if (t->rx_buf)
			((u8 *)t->rx_buf)[i] = (u8)rx;
	}

	return ret;
}


static void hspi_dma_tx_complete(void *arg)
{
	struct hspi_priv *hspi = arg;

	hspi->dma_tx_callbacked = 1;
	wake_up_interruptible(&hspi->wait);
}

static void hspi_dma_rx_complete(void *arg)
{
	struct hspi_priv *hspi = arg;

	hspi->dma_rx_callbacked = 1;
	wake_up_interruptible(&hspi->wait);
}

static int hspi_dma_map_sg(struct scatterlist *sg, void *buf, unsigned len,
			   struct dma_chan *chan,
			   enum dma_data_direction dir)
{
	sg_init_table(sg, 1);
	sg_set_buf(sg, buf, len);
	sg_dma_len(sg) = len;
	return dma_map_sg(chan->device->dev, sg, 1, dir);
}

static void hspi_dma_unmap_sg(struct scatterlist *sg, struct dma_chan *chan,
			      enum dma_data_direction dir)
{
	dma_unmap_sg(chan->device->dev, sg, 1, dir);
}

static bool hspi_filter(struct dma_chan *chan, void *filter_param)
{
	chan->private = filter_param;
	return true;
}

static void hspi_release_dma(struct hspi_priv *hspi)
{
	if (hspi->chan_tx)
		dma_release_channel(hspi->chan_tx);
	if (hspi->chan_rx)
		dma_release_channel(hspi->chan_rx);
}

static int hspi_request_dma(struct hspi_priv *hspi)
{
	struct sh_hspi_info *hspi_pd = hspi->platform_data;
	dma_cap_mask_t mask;
	
	if (!hspi_pd){
		return 0;
	}
	
	if(hspi_pd->dma_rx_id && hspi_pd->dma_tx_id){
	/* If the module receives data by DMAC, it also needs TX DMAC */
		dma_cap_zero(mask);
		dma_cap_set(DMA_SLAVE, mask);
		hspi->dma_rx.slave_id = hspi_pd->dma_rx_id;
		hspi->chan_rx = dma_request_channel(mask, hspi_filter,
						    &hspi->dma_rx);
		if (!hspi->chan_rx)
			goto error;
		
		dma_cap_zero(mask);
		dma_cap_set(DMA_SLAVE, mask);
		hspi->dma_tx.slave_id = hspi_pd->dma_tx_id;
		hspi->chan_tx = dma_request_channel(mask, hspi_filter,
						    &hspi->dma_tx);
		if (!hspi->chan_tx)
			goto error;
	}

	return 1;

error:
	hspi_release_dma(hspi);
	return 0;
}

static int hspi_transfer_dma(struct hspi_priv *hspi, struct spi_message *mesg, struct spi_transfer *t)
{
	struct scatterlist tx_sg, rx_sg;
	void *buf = NULL;
	struct dma_async_tx_descriptor *tx_desc = NULL;
	struct dma_async_tx_descriptor *rx_desc = NULL;
	int ret = 0;

#ifdef HSPI_DEBUG
	dev_info(&hspi->master->dev, "Use DMA engine when transfer.\n");
#endif

	disable_irq(hspi->irq);
	if(t->rx_buf){
		/* Process tx buffer in case of rx :
		 * This must be done before hspi_dma_map_sg(&rx_sg, ...).
		 * Otherwise, the phenomenon is that if transfer length >=32 bytes
		 * then the correctness of transmit and receive data is not guaranteed!
		 */
		buf = kmalloc(t->len, GFP_KERNEL);
		if (!buf){
			ret = -ENOMEM;
			goto end_rx_nomem;
		}
		
		(t->tx_buf) ? memcpy(buf, t->tx_buf, t->len) : 
					  memset(buf, 0, sizeof buf);
		t->tx_buf = buf;
		/* End of processing tx buffer*/
		
		if (!hspi_dma_map_sg(&rx_sg, (void *)t->rx_buf, t->len, hspi->chan_rx, DMA_FROM_DEVICE)) {
			ret = -EFAULT;
			goto end_rx_nomap;
		}
		
		rx_desc = dmaengine_prep_slave_sg(hspi->chan_rx, &rx_sg, 1, DMA_FROM_DEVICE,
						DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
		if (!rx_desc) {
			ret = -EIO;
			goto end_rx;
		}
		
		hspi->dma_rx_callbacked = 0;
	
		rx_desc->callback = hspi_dma_rx_complete;
		rx_desc->callback_param = hspi;
	} /* end if(rx_buf) */
	
	if(t->tx_buf)
	{
		if (!hspi_dma_map_sg(&tx_sg, (void *)t->tx_buf, t->len, hspi->chan_tx, DMA_TO_DEVICE)) {
			ret = -EFAULT;
			goto end_tx_nomap;
		}
		
		tx_desc = dmaengine_prep_slave_sg(hspi->chan_tx, &tx_sg, 1, DMA_TO_DEVICE,
						DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
		if (!tx_desc) {
			ret = -EIO;
			goto end_tx;
		}
	
		hspi->dma_tx_callbacked = 0;
		tx_desc->callback = hspi_dma_tx_complete;
		tx_desc->callback_param = hspi;
	} /* end if(tx_buf) */
	
	if(t->rx_buf){
		ret = hspi_read(hspi, SPRBR); /* dummy read to erase RXFL which is set in previous transmit */
		hspi_bit_set(hspi, SPSCR, 0, (1 << 1)); /* TXDE = 1: trasmit dma enable */
		hspi_bit_set(hspi, SPSCR, 0, (1 << 2)); /* RXDE = 1: receive dma enable */
		
		rx_desc->tx_submit(rx_desc);
		dma_async_issue_pending(hspi->chan_rx);
		
		tx_desc->tx_submit(tx_desc);
		dma_async_issue_pending(hspi->chan_tx);
		
		ret = wait_event_interruptible_timeout(hspi->wait,
					        hspi->dma_tx_callbacked && hspi->dma_rx_callbacked, HZ);
	
		if (ret > 0 && (hspi->dma_tx_callbacked && hspi->dma_rx_callbacked))
			ret = 0;
		else if (!ret)
			ret = -ETIMEDOUT;
	}
	else{
		hspi_bit_set(hspi, SPSCR, 0, (1 << 1)); /* TXDE = 1: trasmit dma enable */
		
		tx_desc->tx_submit(tx_desc);
		dma_async_issue_pending(hspi->chan_tx);
		
		ret = wait_event_interruptible_timeout(hspi->wait,
					       hspi->dma_tx_callbacked, HZ);
		if (ret > 0 && (hspi->dma_tx_callbacked))
			ret = 0;
		else if (!ret)
			ret = -ETIMEDOUT;
	}
	
	hspi_bit_set(hspi, SPSCR, (1 << 1) | (1 << 2), 0); /* TXDE = 0, RXDE = 0: trasmit & receive dma disable */
	
end_tx:
	hspi_dma_unmap_sg(&tx_sg, hspi->chan_tx, DMA_TO_DEVICE);
end_tx_nomap:
end_rx:
	if(t->rx_buf)
		hspi_dma_unmap_sg(&rx_sg, hspi->chan_rx, DMA_FROM_DEVICE);
end_rx_nomap:
	if(buf)
		kfree(buf);
end_rx_nomem:
	enable_irq(hspi->irq);
	return ret;
}

static int hspi_transfer_one_message(struct spi_master *master,
				     struct spi_message *msg)
{
	struct hspi_priv *hspi = spi_master_get_devdata(master);
	struct spi_transfer *t;
	int ret;
	unsigned int cs_change;
	const int nsecs = 50;

	dev_dbg(hspi->dev, "%s\n", __func__);

	cs_change = 1;
	ret = 0;
	list_for_each_entry(t, &msg->transfers, transfer_list) {
		hspi_hw_setup(hspi, msg, t);
		if (cs_change) {
			hspi_hw_cs_enable(hspi);
			ndelay(nsecs);
		}
		cs_change = t->cs_change;

		/* If transfer length is bigger than SPI_DMA_MSG_SIZE_THRESHOLD bytes,then
		 * use DMA engine. If DMA channel tx or rx requests fail,
		 * switch back to PIO mode.
		 *
		 * If transfer length is less than SPI_DMA_MSG_SIZE_THRESHOLD bytes, use PIO 
		 * mode
		*/
		if((t->len > SPI_DMA_MSG_SIZE_THRESHOLD) && hspi->dma_is_used){
			if(hspi_request_dma(hspi)){
				ret = hspi_transfer_dma(hspi, msg, t);
				hspi_release_dma(hspi);
			}else{ /* DMA request failed -> switch to PIO mode */
				ret = hspi_transfer_pio(hspi, msg, t);
			}
		}
		else{
			ret = hspi_transfer_pio(hspi, msg, t);
		}
		
		if (ret < 0)
			break;

		msg->actual_length += t->len;

		if (t->delay_usecs)
			udelay(t->delay_usecs);

		if (cs_change) {
			ndelay(nsecs);
			hspi_hw_cs_disable(hspi);
			ndelay(nsecs);
		}
	}

	msg->status = ret;
	if (!cs_change) {
		ndelay(nsecs);
		hspi_hw_cs_disable(hspi);
	}
	spi_finalize_current_message(master);

	return ret;
}

static int hspi_setup(struct spi_device *spi)
{
	struct hspi_priv *hspi = spi_master_get_devdata(spi->master);
	struct device *dev = hspi->dev;

	if (8 != spi->bits_per_word) {
		dev_err(dev, "bits_per_word should be 8\n");
		return -EIO;
	}

	dev_dbg(dev, "%s setup\n", spi->modalias);

	return 0;
}

static void hspi_cleanup(struct spi_device *spi)
{
	struct hspi_priv *hspi = spi_master_get_devdata(spi->master);
	struct device *dev = hspi->dev;

	dev_dbg(dev, "%s cleanup\n", spi->modalias);
}

static irqreturn_t hspi_irq(int irq, void *_sr)
{
	struct hspi_priv *hspi = (struct hspi_priv *)_sr;
	unsigned long spsr;
	irqreturn_t ret = IRQ_NONE;
	int disable_irq = 0;

	hspi->spsr = spsr = hspi_read(hspi, SPSR);
	
	if (spsr & (1 << 1)){ /* TXFL = 1: transmit complete */
		disable_irq |= (1 << 4);
		
	}

	if (disable_irq) {
		ret = IRQ_HANDLED;
		hspi_disable_irq(hspi, disable_irq);
		wake_up(&hspi->wait);
	}

	return ret;
}

static int __devinit hspi_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct spi_master *master;
	struct hspi_priv *hspi;
	struct clk *clk;
	int ret, irq;
	struct sh_hspi_info *hspi_pd = pdev->dev.platform_data;
	
	/* get base addr */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "invalid resource\n");
		return -EINVAL;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "platform_get_irq error\n");
		return -ENODEV;
	}

	master = spi_alloc_master(&pdev->dev, sizeof(*hspi));
	if (!master) {
		dev_err(&pdev->dev, "spi_alloc_master error.\n");
		return -ENOMEM;
	}

	clk = clk_get(NULL, "shyway_clk");
	if (!clk) {
		dev_err(&pdev->dev, "shyway_clk is required\n");
		ret = -EINVAL;
		goto error0;
	}

	hspi = spi_master_get_devdata(master);
	dev_set_drvdata(&pdev->dev, hspi);
	hspi->platform_data = hspi_pd;
	
	/* init hspi */
	hspi->master	= master;
	hspi->dev	= &pdev->dev;
	hspi->clk	= clk;
	hspi->addr	= devm_ioremap(hspi->dev,
				       res->start, resource_size(res));
	if (!hspi->addr) {
		dev_err(&pdev->dev, "ioremap error.\n");
		ret = -ENOMEM;
		goto error1;
	}

	init_waitqueue_head(&hspi->wait);
	
	master->num_chipselect	= 1;
	master->bus_num		= pdev->id;
	master->setup		= hspi_setup;
	master->cleanup		= hspi_cleanup;
	master->mode_bits	= SPI_CPOL | SPI_CPHA;
	master->prepare_transfer_hardware	= hspi_prepare_transfer;
	master->transfer_one_message		= hspi_transfer_one_message;
	master->unprepare_transfer_hardware	= hspi_unprepare_transfer;
	
	ret = request_irq(irq, hspi_irq, IRQF_DISABLED, dev_name(&pdev->dev), hspi);
	if (ret < 0) {
		dev_err(&pdev->dev, "request_irq error\n");
		goto error2;
	}

	hspi->irq = irq;
	hspi->dma_is_used = (hspi_pd->dma_rx_id && hspi_pd->dma_tx_id) ? 1 : 0;

	ret = spi_register_master(master);
	if (ret < 0) {
		dev_err(&pdev->dev, "spi_register_master error.\n");
		goto error3;
	}

	pm_runtime_enable(&pdev->dev);

	dev_info(&pdev->dev, "probed\n");

	return 0;
 error3:
	free_irq(irq, hspi);
 error2:
	devm_iounmap(hspi->dev, hspi->addr);
 error1:
	clk_put(clk);
 error0:
	spi_master_put(master);
	
	return ret;
}

static int __devexit hspi_remove(struct platform_device *pdev)
{
	struct hspi_priv *hspi = dev_get_drvdata(&pdev->dev);

	pm_runtime_disable(&pdev->dev);

	clk_put(hspi->clk);
	spi_unregister_master(hspi->master);
	devm_iounmap(hspi->dev, hspi->addr);

	return 0;
}

static struct platform_driver hspi_driver = {
	.probe = hspi_probe,
	.remove = __devexit_p(hspi_remove),
	.driver = {
		.name = "sh-hspi",
		.owner = THIS_MODULE,
	},
};
module_platform_driver(hspi_driver);

MODULE_DESCRIPTION("SuperH HSPI bus driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kuninori Morimoto <kuninori.morimoto.gx@renesas.com>");
MODULE_ALIAS("platform:sh_spi");

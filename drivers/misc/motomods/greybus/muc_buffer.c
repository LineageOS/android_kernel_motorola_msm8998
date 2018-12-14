/*
 * Copyright (C) 2016 Motorola Mobility LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#define pr_fmt(fmt) "MODS_BUFF: " fmt

#include <linux/slab.h>
#include "muc.h"

#define MAX(a, b) ((a) >= (b) ? (a) : (b))

static struct muc_buffers *_buffers;

struct muc_buffers *muc_get_buffers(void)
{
	return _buffers;
}

int muc_buffer_init(void)
{
	size_t i2c_pkt_sz = muc_i2c_get_pkt_sz(MAX_DATAGRAM_SZ);
	size_t spi_pkt_sz = muc_spi_get_pkt_sz(MAX_DATAGRAM_SZ);
	size_t pkt_sz = MAX(i2c_pkt_sz, spi_pkt_sz);

	_buffers = kzalloc(sizeof(*_buffers), GFP_KERNEL);
	if (!_buffers)
		goto done;

	_buffers->tx_pkt = kzalloc(pkt_sz, GFP_KERNEL | GFP_DMA);
	if (!_buffers->tx_pkt)
		goto free_buf;

	_buffers->rx_pkt = kzalloc(pkt_sz, GFP_KERNEL | GFP_DMA);
	if (!_buffers->rx_pkt)
		goto free_tx_pkt;

	_buffers->tx_datagram = kzalloc(MAX_DATAGRAM_SZ, GFP_KERNEL | GFP_DMA);
	if (!_buffers->tx_datagram)
		goto free_rx_pkt;

	_buffers->rx_datagram = kzalloc(MAX_DATAGRAM_SZ, GFP_KERNEL | GFP_DMA);
	if (!_buffers->rx_datagram)
		goto free_tx_dg;

	return 0;

free_tx_dg:
	kfree(_buffers->tx_datagram);
free_rx_pkt:
	kfree(_buffers->rx_pkt);
free_tx_pkt:
	kfree(_buffers->tx_pkt);
free_buf:
	kfree(_buffers);
done:
	return -ENOMEM;
}

void muc_buffer_exit(void)
{
	kfree(_buffers->tx_pkt);
	kfree(_buffers->rx_pkt);
	kfree(_buffers->rx_datagram);
	kfree(_buffers->tx_datagram);
	kfree(_buffers);
	_buffers = NULL;
}

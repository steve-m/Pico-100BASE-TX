/*
 * Pico-100BASE-TX, a bit-banged 100Base-TX Ethernet MAC+PHY and UDP transmitter
 *
 * PIO counter value example
 *
 * Copyright (c) 2024-2025 by Steve Markgraf <steve@steve-m.de>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the author nor the names of its contributors may
 *    be used to endorse or promote products derived from this software
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hardware/vreg.h"
#include "hardware/dma.h"
#include "hardware/pio.h"

#include "pico100basetx.h"
#include "counter.pio.h"

#define SYS_CLK		125000

#define DMACH_PIO_PING	0
#define DMACH_PIO_PONG	1

static bool pio_dma_pong = false;
struct udp_frame ringbuffer[RBUF_DEFAULT_NUM];
int ringbuf_head = 2;

void __scratch_y("") pio_dma_irq_handler()
{
	uint ch_num = pio_dma_pong ? DMACH_PIO_PONG : DMACH_PIO_PING;
	dma_channel_hw_t *ch = &dma_hw->ch[ch_num];
	dma_hw->intr = 1u << ch_num;
	pio_dma_pong = !pio_dma_pong;

	ringbuf_head = (ringbuf_head + 1) % RBUF_DEFAULT_NUM;

	ch->write_addr = (uintptr_t)&ringbuffer[ringbuf_head].data;
	ch->transfer_count = RBUF_MAX_DATA_LEN / sizeof(uint16_t);

	pico100basetx_update_head(0, ringbuf_head);
}

void init_pio_input(void)
{
	PIO pio = pio0;
	uint offset = pio_add_program(pio, &counter_program);
	uint sm_data = pio_claim_unused_sm(pio, true);
	counter_program_init(pio, sm_data, offset);

	dma_channel_config c;
	c = dma_channel_get_default_config(DMACH_PIO_PING);
	channel_config_set_chain_to(&c, DMACH_PIO_PONG);
	channel_config_set_dreq(&c, pio_get_dreq(pio, sm_data, false));
	channel_config_set_read_increment(&c, false);
	channel_config_set_write_increment(&c, true);
	channel_config_set_transfer_data_size(&c, DMA_SIZE_16);

	dma_channel_configure(
		DMACH_PIO_PING,
		&c,
		&ringbuffer[0].data,
		&pio->rxf[sm_data],
		RBUF_MAX_DATA_LEN / sizeof(uint16_t),
		false
	);
	c = dma_channel_get_default_config(DMACH_PIO_PONG);
	channel_config_set_chain_to(&c, DMACH_PIO_PING);
	channel_config_set_dreq(&c, pio_get_dreq(pio, sm_data, false));
	channel_config_set_read_increment(&c, false);
	channel_config_set_write_increment(&c, true);
	channel_config_set_transfer_data_size(&c, DMA_SIZE_16);

	dma_channel_configure(
		DMACH_PIO_PONG,
		&c,
		&ringbuffer[1].data,
		&pio->rxf[sm_data],
		RBUF_MAX_DATA_LEN / sizeof(uint16_t),
		false
	);

	dma_hw->ints0 |= (1u << DMACH_PIO_PING) | (1u << DMACH_PIO_PONG);
	dma_hw->inte0 |= (1u << DMACH_PIO_PING) | (1u << DMACH_PIO_PONG);
	irq_set_exclusive_handler(DMA_IRQ_0, pio_dma_irq_handler);
	irq_set_enabled(DMA_IRQ_0, true);

	dma_channel_start(DMACH_PIO_PING);
}

int main()
{
	set_sys_clock_khz(SYS_CLK, true);
	stdio_init_all();

	pico100basetx_init(0);
	pico100basetx_add_stream(0, 1, (SYS_CLK/8) * 1000, RBUF_MAX_DATA_LEN, RBUF_DEFAULT_NUM, ringbuffer);
	pico100basetx_start();
	init_pio_input();

	while (1)
		__wfi();
}

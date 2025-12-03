/*
 * Pico-100BASE-TX, a bit-banged 100Base-TX Ethernet MAC+PHY and UDP transmitter
 *
 * PCM1802 audio ADC example
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
#include "hardware/pll.h"
#include "hardware/dma.h"
#include "hardware/pio.h"

#include "pico100basetx.h"
#include "pcm1802_fmt00.pio.h"

#define SYS_CLK			125000

#define AUDIO_DATA_LEN		((RBUF_MAX_DATA_LEN / sizeof(uint32_t)) - 1)
#define AUDIO_RBUF_SLICES	RBUF_DEFAULT_NUM

#define PCM1802_DATA_PIN	22

#define DMACH_AUDIO_PIO_PING	0
#define DMACH_AUDIO_PIO_PONG	1

static bool audio_pio_dma_pong = false;
struct udp_frame audio_ringbuffer[AUDIO_RBUF_SLICES];
int audio_ringbuf_head = 2;

void __scratch_y("") audio_pio_dma_irq_handler()
{
	uint ch_num = audio_pio_dma_pong ? DMACH_AUDIO_PIO_PONG : DMACH_AUDIO_PIO_PING;
	dma_channel_hw_t *ch = &dma_hw->ch[ch_num];
	dma_hw->intr = 1u << ch_num;
	audio_pio_dma_pong = !audio_pio_dma_pong;

	audio_ringbuf_head = (audio_ringbuf_head + 1) % AUDIO_RBUF_SLICES;

	ch->write_addr = (uintptr_t)&audio_ringbuffer[audio_ringbuf_head].data;
	ch->transfer_count = AUDIO_DATA_LEN;

	pico100basetx_update_head(0, audio_ringbuf_head);
}

void init_audio_pio_input(void)
{
	PIO pio = pio0;
	uint offset = pio_add_program(pio, &pcm1802_fmt00_program);
	uint sm_data = pio_claim_unused_sm(pio, true);
	pcm1802_fmt00_program_init(pio, sm_data, offset, PCM1802_DATA_PIN);

	dma_channel_config c;
	c = dma_channel_get_default_config(DMACH_AUDIO_PIO_PING);
	channel_config_set_chain_to(&c, DMACH_AUDIO_PIO_PONG);
	channel_config_set_dreq(&c, pio_get_dreq(pio, sm_data, false));
	channel_config_set_read_increment(&c, false);
	channel_config_set_write_increment(&c, true);
	channel_config_set_transfer_data_size(&c, DMA_SIZE_32);

	dma_channel_configure(
		DMACH_AUDIO_PIO_PING,
		&c,
		&audio_ringbuffer[0].data,
		&pio->rxf[sm_data],
		AUDIO_DATA_LEN,
		false
	);
	c = dma_channel_get_default_config(DMACH_AUDIO_PIO_PONG);
	channel_config_set_chain_to(&c, DMACH_AUDIO_PIO_PING);
	channel_config_set_dreq(&c, pio_get_dreq(pio, sm_data, false));
	channel_config_set_read_increment(&c, false);
	channel_config_set_write_increment(&c, true);
	channel_config_set_transfer_data_size(&c, DMA_SIZE_32);

	dma_channel_configure(
		DMACH_AUDIO_PIO_PONG,
		&c,
		&audio_ringbuffer[1].data,
		&pio->rxf[sm_data],
		AUDIO_DATA_LEN,
		false
	);

	dma_hw->ints1 |= (1u << DMACH_AUDIO_PIO_PING) | (1u << DMACH_AUDIO_PIO_PONG);
	dma_hw->inte1 |= (1u << DMACH_AUDIO_PIO_PING) | (1u << DMACH_AUDIO_PIO_PONG);
	irq_set_exclusive_handler(DMA_IRQ_1, audio_pio_dma_irq_handler);
	irq_set_enabled(DMA_IRQ_1, true);

	dma_channel_start(DMACH_AUDIO_PIO_PING);
}

int main()
{
	set_sys_clock_khz(SYS_CLK, true);

	pll_init(pll_usb, 1, 1536 * MHZ, 2, 2);

	/* set USB clock to clk_usb/8 */
	hw_write_masked(&clocks_hw->clk[clk_usb].div, 8 << CLOCKS_CLK_USB_DIV_INT_LSB, CLOCKS_CLK_USB_DIV_INT_BITS);

	/* set GPOUT0 clock to USB PLL/10 -> 38.4 MHz, resulting in 75 kHz ADC sample rate (19.2M/512) */
	clock_gpio_init(21, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB, 10);

	stdio_init_all();

	pico100basetx_init(0);
	pico100basetx_add_stream(0, 0, 78000, AUDIO_DATA_LEN * sizeof(uint32_t), AUDIO_RBUF_SLICES, audio_ringbuffer);
	pico100basetx_start();
	init_audio_pio_input();

	while (1)
		__wfi();
}

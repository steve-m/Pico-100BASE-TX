/*
 * Pico-100BASE-TX, a bit-banged 100Base-TX Ethernet MAC+PHY and UDP transmitter
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

#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"
#include "hardware/structs/bus_ctrl.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "pico/unique_id.h"
#include "pico100basetx.h"
#include "mlt3_out.pio.h"
#include "pin_out.pio.h"

/* we want to use the top DMA channels, so the application
 * can use the bottom ones */
#ifdef PICO_RP2040
	#define DMACH_CHECKSUM	9
	#define DMACH_MLT_PING	10
	#define DMACH_MLT_PONG	11
	#define MLT_DMA_IRQ	(DMA_IRQ_1)
	#define MLT_OUT_PIO	(pio1)
#else
	#define DMACH_CHECKSUM	13
	#define DMACH_MLT_PING	14
	#define DMACH_MLT_PONG	15
	#define MLT_DMA_IRQ	(DMA_IRQ_3)
	#define MLT_OUT_PIO	(pio2)
#endif

/* for PIO inter-SM communication - can be any unused GPIO */
#define DUMMY_OUT_PIN	3

typedef struct
{
	bool active;
	struct udp_frame *rbuf;
	uint tail;
	uint head;
	uint rbuf_slices;
	uint16_t format;
	uint32_t srate;
	uint16_t len;
	uint64_t data_cnt;
	uint32_t frame_cnt;
	bool overflow;
	uint8_t dst_mac[MAC_ADDR_LEN];
	uint32_t dst_ip;
	uint16_t dst_port;
} stream_t;

stream_t streams[MAX_STREAMS];

/*
 * Calculation: 6 4b5b symbols per uint32, so 3 actual bytes
 *
 *    8 octets	JK + preamble
 * 1518 octets	Ethernet II frame with MAC header and checksum
 *   12 octets	Interframe gap (IFG), already includes TR symbol (see 24.2.2.2 of 802.3-2005)
 * -----------
 * 1538 octets = 3076 4b5b symbols = at least 513 words containing 6 4b5b symbols
 * So we just add two additional idle symbols (1 octet) and it lines up nicely with 513 words.
 */

#define TX_RBUF_WORDS	513
#define TX_RBUF_CNT	2

static uint32_t tx_rbuf[TX_RBUF_WORDS * TX_RBUF_CNT];
static uint32_t lfsr_index = 0;

static bool use_udp_checksum = true;
static bool mlt_dma_pong = false;
static uint8_t src_mac[MAC_ADDR_LEN];
static uint32_t src_ip = IP_ADDR(192,168,23,50);
static uint32_t ip_frame_ctr = 0;

static uint32_t scramble(uint8_t unscrambled_bit)
{
	static uint16_t lfsr = 0x7ff;

	uint16_t lfsr_next = !!(lfsr & (1 << 10)) ^ !!(lfsr & (1 << 8));
	uint16_t scrambled = unscrambled_bit ^ lfsr_next;

	lfsr <<= 1;
	lfsr |= scrambled;
	lfsr &= 0x7ff;

	return (uint32_t)scrambled;
}

#define LFSR_LEN	2047

/* We duplicate parts of the LUT at the end so when there is no frame to transmit
 * we directly can use this buffer to transmit the idle symbols */
static uint32_t lfsr_lut[LFSR_LEN + TX_RBUF_WORDS];

void init_scrambler_lut(void)
{
	/* The idea is to pack six 4b5b symbols into a word, resulting
	 * in 30 used bits. As we simply want to XOR those words with the
	 * scramler LUT, it needs to use 30 bits as well, and repeat again at
	 * some point. For this we initialize a LUT with 30 copies of the scrambling
	 * sequence, at the end of which it wraps around and aligns again. */

	for (int i = 0; i < (LFSR_LEN + TX_RBUF_WORDS); i++) {
		lfsr_lut[i] = 0;

		for (int j = 0; j < 30; j++)
			lfsr_lut[i] |= scramble(0) << (31-j);
	}
}

static uint16_t lut_dual_4b5b[256];

/* generate inverted LUT for two 4b5b symbols that takes a byte index */
static void init_4b5b_lut(void)
{
	/* 4B5B lookup table */
	const uint8_t symbols_4b5b[] =
	{
		0b11110, 0b01001, 0b10100, 0b10101, 0b01010, 0b01011, 0b01110, 0b01111,
		0b10010, 0b10011, 0b10110, 0b10111, 0b11010, 0b11011, 0b11100, 0b11101
	};

	for (int i = 0; i < 256 ; i++) {
		uint16_t symb = symbols_4b5b[(i & 0xf0) >> 4] | (symbols_4b5b[i & 0xf] << 5);
		lut_dual_4b5b[i] = (~symb & 0x3ff) << 2;
	}
}

/* 4B5B command characters (inverted) */
#define CMD_JK	(0x0ee << 2)	/* Sync, Start delimiter, replaces the first 8 bits of the preamble */
#define CMD_I	(0x000 << 2)	/* 100BASE-X idle marker */
#define CMD_II	(0x000 << 2)
#define CMD_TR	(0x258 << 2)	/* 100BASE-X end delimiter */

static void inline encode_eth_frame(uint32_t *fptr, uint8_t *data, int len)
{
	int i, out_len = 0;

	/* as we have a copy of the LFSR data (with the size of a max MTU ethernet frame)
	 * that extends beyond the end of the PN sequence, we can simply increment the
	 * index of the LFSR LUT and do the modulo operation once at the end */
	int scramb = lfsr_index;

	dma_channel_config dma_conf;
	dma_conf = dma_channel_get_default_config(DMACH_CHECKSUM);

	channel_config_set_transfer_data_size(&dma_conf, DMA_SIZE_8);
	channel_config_set_read_increment(&dma_conf, true);
	channel_config_set_write_increment(&dma_conf, false);
	dma_channel_configure (
		DMACH_CHECKSUM,
		&dma_conf,
		NULL,
		data,
		len,
		false
	);

	dma_sniffer_enable(DMACH_CHECKSUM, DMA_SNIFF_CTRL_CALC_VALUE_CRC32R, true);
	dma_sniffer_set_output_invert_enabled(true);
	dma_sniffer_set_output_reverse_enabled(true);
	dma_sniffer_set_data_accumulator(0xffffffff);
	dma_channel_set_read_addr(DMACH_CHECKSUM, data, true);

	/* encode preamble and SFD */
	const uint8_t preamb[] = { /* JK symbol */ 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0xd5 };
	fptr[out_len++] = lfsr_lut[scramb++] ^ ((CMD_II << 20) | (CMD_JK << 10) | lut_dual_4b5b[preamb[0]]);
	fptr[out_len++] = lfsr_lut[scramb++] ^ ((lut_dual_4b5b[preamb[1]] << 20) | (lut_dual_4b5b[preamb[2]] << 10) | lut_dual_4b5b[preamb[3]]);
	fptr[out_len++] = lfsr_lut[scramb++] ^ ((lut_dual_4b5b[preamb[4]] << 20) | (lut_dual_4b5b[preamb[5]] << 10) | lut_dual_4b5b[preamb[6]]);

	/* encode and scramble actual frame bytes */
	int rem_bytes = len % 3;
	for (i = 0; i < (len - rem_bytes); i += 3) {
		fptr[out_len++] = lfsr_lut[scramb++] ^ ((lut_dual_4b5b[data[i+0]] << 20) |
							(lut_dual_4b5b[data[i+1]] << 10) |
							 lut_dual_4b5b[data[i+2]]);
	}

	/* Now the last bytes, the CRC and the end of frame marker follow, and
	 * we need to take care of alignment. We have 2 remaining payload bytes
	 * at max, 4 CRC bytes, and the TR symbol, a total of 7, which we need to
	 * round up to 9 because we iterate in 3 symbol steps */
	const int tail_len = 9;
	uint16_t tail_syms[tail_len];
	int nsyms = 0;

	for (int j = 0; j < rem_bytes; j++)
		tail_syms[nsyms++] = lut_dual_4b5b[data[i+j]];

	dma_channel_wait_for_finish_blocking(DMACH_CHECKSUM);
	uint32_t crc = dma_sniffer_get_data_accumulator();

	for (i = 0; i < sizeof(crc); i++)
		tail_syms[nsyms++] = lut_dual_4b5b[(crc >> (i * 8)) & 0xff];

	tail_syms[nsyms++] = CMD_TR;

	while (nsyms < tail_len)
		tail_syms[nsyms++] = CMD_II;

	/* now scramble the tail */
	for (i = 0; i < tail_len; i += 3) {
		fptr[out_len++] = lfsr_lut[scramb++] ^ ((tail_syms[i+0] << 20) |
							(tail_syms[i+1] << 10) |
							 tail_syms[i+2]);
	}

	/* copy the remaining idle symbols */
	while (out_len < TX_RBUF_WORDS)
		fptr[out_len++] = lfsr_lut[scramb++];

	/* update LFSR index */
	lfsr_index = scramb % LFSR_LEN;
}

static inline void start_udp_checksum(uint8_t *datagram, uint16_t len)
{
	dma_channel_config dma_conf;

	dma_conf = dma_channel_get_default_config(DMACH_CHECKSUM);
	channel_config_set_transfer_data_size(&dma_conf, DMA_SIZE_16);

	channel_config_set_read_increment(&dma_conf, true);
	channel_config_set_write_increment(&dma_conf, false);
	dma_channel_configure (
		DMACH_CHECKSUM,
		&dma_conf,
		NULL,
		datagram,
		len / sizeof(uint16_t),
		false
	);

	dma_sniffer_enable(DMACH_CHECKSUM, DMA_SNIFF_CTRL_CALC_VALUE_SUM, true);
	dma_sniffer_set_data_accumulator(0);
	dma_sniffer_set_output_invert_enabled(false);
	dma_sniffer_set_output_reverse_enabled(false);

	/* start transfer */
	dma_channel_set_read_addr(DMACH_CHECKSUM, datagram, true);
}

static inline uint16_t finish_udp_checksum(uint8_t *datagram, uint16_t len, uint32_t saddr, uint32_t daddr)
{
	uint32_t sum = 0;

	/* add padding for odd packet lengths */
	if (len & 1)
		sum += datagram[len-1];

	/* pseudo-header */
	sum += saddr & 0xffff;
	sum += saddr >> 16;
	sum += daddr & 0xffff;
	sum += daddr >> 16;
	sum += htons(IPPROTO_UDP);
	sum += htons(len);

	dma_channel_wait_for_finish_blocking(DMACH_CHECKSUM);
	sum += dma_sniffer_get_data_accumulator();

	/* take care of carries */
	while (sum >> 16)
		sum = (sum & 0xffff) + (sum >> 16);

	sum = ~sum & 0xffff;

	return sum ? sum : 0xffff;
}

static inline uint16_t in_cksum(struct ip_hdr *hdr, int len)
{
	uint16_t *word = (void *)hdr;
	uint32_t sum = 0;

	for (int i = 0; i < len/sizeof(uint16_t); i++)
		sum += word[i];

	while (sum >> 16)
		sum = (sum & 0xffff) + (sum >> 16);

	return ~sum & 0xffff;
}

static inline void init_udp_frame(struct udp_frame *f, stream_t *s, uint16_t payload_len)
{
	for (int i = 0; i < MAC_ADDR_LEN; i++)
		f->eth.saddr[i] = src_mac[i];

	for (int i = 0; i < MAC_ADDR_LEN; i++)
		f->eth.daddr[i] = s->dst_mac[i];

	f->eth.type = htons(IPV4_ETHERTYPE);

	f->ip.ihl = 5;
	f->ip.version = 4;
	f->ip.tos = 0;
	f->ip.tot_len = htons(IP_HEADER_LEN + UDP_HEADER_LEN + payload_len);
	f->ip.id = htons(ip_frame_ctr++ & 0xffff);
	f->ip.frag_off = 0;
	f->ip.ttl = 128;
	f->ip.protocol = IPPROTO_UDP;
	f->ip.chksum = 0;
	f->ip.saddr = htonl(src_ip);
	f->ip.daddr = htonl(s->dst_ip);

	f->udp.src_port = htons(UDP_PORT_BASE);
	f->udp.dst_port = htons(s->dst_port);
	f->udp.len = htons(UDP_HEADER_LEN + payload_len);
	f->udp.chksum = 0;
}

void __scratch_x("") mlt_dma_irq_handler()
{
	uint ch_num = mlt_dma_pong ? DMACH_MLT_PONG : DMACH_MLT_PING;
	dma_channel_hw_t *ch = &dma_hw->ch[ch_num];
	dma_hw->intr = 1u << ch_num;

	uint32_t *txbuf = mlt_dma_pong ? &tx_rbuf[0] : &tx_rbuf[TX_RBUF_WORDS];

	mlt_dma_pong = !mlt_dma_pong;
	struct udp_frame *tx_frame = NULL;
	stream_t *stream = NULL;
	uint32_t fcnt = 0;

	for (uint i = 0; i < MAX_STREAMS; i++) {
		stream = &streams[i];

		if (!stream->active)
			continue;

		int next_tail = (stream->tail + 1) % stream->rbuf_slices;
		if (stream->head != next_tail) {
			fcnt = stream->frame_cnt++;

			tx_frame = &stream->rbuf[stream->tail];
			stream->tail = next_tail;
			stream->data_cnt += stream->len;
			break;
		}
	}

	if (tx_frame) {
		/* we have a frame to transmit */
		int length = INFO_HEADER_LEN + stream->len;
		init_udp_frame(tx_frame, stream, length);

		/* insert 20-bit frame counter */
		tx_frame->info_hdr[0] = fcnt & 0xff;
		tx_frame->info_hdr[1] = (fcnt >> 8) & 0xff;
		tx_frame->info_hdr[2] = (fcnt >> 16) & 0x0f;
		tx_frame->info_hdr[3] = 0x00;

		if (stream->overflow)
			tx_frame->info_hdr[2] |= STATUS_OVERFLOW;

		if (use_udp_checksum)
			start_udp_checksum((uint8_t *)&tx_frame->udp, ntohs(tx_frame->udp.len));

		tx_frame->ip.chksum = in_cksum(&tx_frame->ip, IP_HEADER_LEN);

		if (use_udp_checksum)
			tx_frame->udp.chksum = finish_udp_checksum((uint8_t *)&tx_frame->udp,
								   ntohs(tx_frame->udp.len),
								   tx_frame->ip.saddr, tx_frame->ip.daddr);

		int eth_len = ETH_HEADER_LEN + IP_HEADER_LEN + UDP_HEADER_LEN + length;
		encode_eth_frame(txbuf, (uint8_t *)&tx_frame->eth, eth_len);

		ch->read_addr = (uintptr_t)txbuf;
		ch->transfer_count = TX_RBUF_WORDS;
	} else {
		/* no frame to transmit, just send the scrambled idle symbols */
		ch->read_addr = (uintptr_t)&lfsr_lut[lfsr_index];
		ch->transfer_count = TX_RBUF_WORDS;

		lfsr_index = (lfsr_index + TX_RBUF_WORDS) % LFSR_LEN;
	}
}

void init_mlt3_output(int base_pin)
{
	PIO pio = MLT_OUT_PIO;

	uint offset = pio_add_program(pio, &mlt3_output_program);
	uint sm_data = pio_claim_unused_sm(pio, true);
	mlt3_output_program_init(pio, sm_data, offset, base_pin, DUMMY_OUT_PIN);

	offset = pio_add_program(pio, &pin_output_program);
	sm_data = pio_claim_unused_sm(pio, true);
	pin_output_program_init(pio, sm_data, offset, DUMMY_OUT_PIN);

	dma_channel_config c;
	c = dma_channel_get_default_config(DMACH_MLT_PING);
	channel_config_set_chain_to(&c, DMACH_MLT_PONG);
	channel_config_set_dreq(&c, pio_get_dreq(pio, sm_data, true));
	channel_config_set_read_increment(&c, true);
	channel_config_set_write_increment(&c, false);
	channel_config_set_transfer_data_size(&c, DMA_SIZE_32);

	dma_channel_configure(
		DMACH_MLT_PING,
		&c,
		&pio->txf[sm_data],
		lfsr_lut,
		LFSR_LEN,
		false
	);
	c = dma_channel_get_default_config(DMACH_MLT_PONG);
	channel_config_set_chain_to(&c, DMACH_MLT_PING);
	channel_config_set_dreq(&c, pio_get_dreq(pio, sm_data, true));
	channel_config_set_read_increment(&c, true);
	channel_config_set_write_increment(&c, false);
	channel_config_set_transfer_data_size(&c, DMA_SIZE_32);

	dma_channel_configure(
		DMACH_MLT_PONG,
		&c,
		&pio->txf[sm_data],
		lfsr_lut,
		LFSR_LEN,
		false
	);

#ifdef PICO_RP2040
	dma_hw->ints1 |= (1u << DMACH_MLT_PING) | (1u << DMACH_MLT_PONG);
	dma_hw->inte1 |= (1u << DMACH_MLT_PING) | (1u << DMACH_MLT_PONG);
#else
	dma_hw->ints3 |= (1u << DMACH_MLT_PING) | (1u << DMACH_MLT_PONG);
	dma_hw->inte3 |= (1u << DMACH_MLT_PING) | (1u << DMACH_MLT_PONG);
#endif

	/* synchronously start both state machines */
	pio_set_sm_mask_enabled(pio, 3, true);
}

void __scratch_y("") pico100basetx_update_head(int stream_id, int head)
{
	if (streams[stream_id].tail == head)
		streams[stream_id].overflow = true;
	else
		streams[stream_id].head = head;
}

static void pico100basetx_core1_entry()
{
	irq_set_exclusive_handler(MLT_DMA_IRQ, mlt_dma_irq_handler);
	irq_set_enabled(MLT_DMA_IRQ, true);

	while (1)
		__wfi();
}

void pico100basetx_start(void)
{
	multicore_launch_core1(pico100basetx_core1_entry);
	dma_channel_start(DMACH_MLT_PING);
}

int pico100basetx_set_destination(uint16_t stream_id, uint64_t dest_mac, uint32_t dest_ip, uint16_t dest_port)
{
	if (stream_id >= MAX_STREAMS)
		return -1;

	stream_t *s = &streams[stream_id];

	for (int i = 0; i < MAC_ADDR_LEN; i++)
		s->dst_mac[i] = (dest_mac >> i*8) & 0xff;

	s->dst_ip = dest_ip;
	s->dst_port = dest_port;

	return 0;
}

void pico100basetx_set_host_address(uint64_t mac, uint32_t ip)
{
	if (mac)
		for (int i = 0; i < MAC_ADDR_LEN; i++)
			src_mac[i] = (mac >> i*8) & 0xff;

	if (ip)
		src_ip = ip;
}

void pico100basetx_enable_udp_checksum(bool on)
{
	use_udp_checksum = on;
}

int pico100basetx_add_stream(uint16_t stream_id, uint16_t format, uint32_t samplerate, uint length, uint slices, struct udp_frame *ringbuf)
{
	if (stream_id >= MAX_STREAMS)
		return -1;

	stream_t *s = &streams[stream_id];
	s->active = false;
	s->rbuf = ringbuf;
	s->format = format;
	s->srate = samplerate;
	s->len = length;
	s->rbuf_slices = slices;
	s->tail = slices-1;
	s->head = 0;
	s->data_cnt = 0;
	s->frame_cnt = 0;
	pico100basetx_set_destination(stream_id, 0xffffffffffff, IP_ADDR(255,255,255,255), UDP_PORT_BASE + stream_id);

	s->active = true;

	return 0;
}

int pico100basetx_remove_stream(uint16_t stream_id)
{
	if (stream_id >= MAX_STREAMS)
		return -1;

	streams[stream_id].active = false;

	return 0;
}

void pico100basetx_init(int base_pin)
{
	pico_unique_board_id_t uid;
	pico_get_unique_board_id(&uid);

	/* initialize our MAC address from UID, set 'locally administered' bit */
	src_mac[0] = (uid.id[0] & ~1) | (1 << 1);
	for (int i = 1; i < MAC_ADDR_LEN; i++)
		src_mac[i] = uid.id[i];

	for (uint i = 0; i < MAX_STREAMS; i++)
		streams[i].active = false;

	init_scrambler_lut();
	init_4b5b_lut();
	init_mlt3_output(base_pin);

	/* give the DMA the priority over the CPU on the bus */
	bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_DMA_W_BITS | BUSCTRL_BUS_PRIORITY_DMA_R_BITS;
}

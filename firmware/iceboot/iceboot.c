/*
 * Configure the ICE40 with new bitstreams:
 *	- repeatedly from usbcdc
*/
/*
Copyright (c) 2019, Alan Wood All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of the copyright holder nor the names of its contributors may
be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "main.h"
#include "usbd_cdc_if.h"
#include "stm32f7xx_hal.h"
#include "errno.h"

#define SPI3_MISO_Pin GPIO_PIN_4
#define SPI3_MISO_GPIO_Port GPIOB
#define SPI3_SCK_Pin GPIO_PIN_3
#define SPI3_SCK_GPIO_Port GPIOB

#define VER "<myStorm 0.503> "

enum { OK, TIMEOUT, ICE_ERROR };
enum { CDC_FIFOSIZE = 1024 };	/* must be power of 2 */

typedef int (*Reader)(uint8_t*, uint16_t*);

/* GPIO function Macros */
#define gpio_low(pin)	HAL_GPIO_WritePin(pin##_GPIO_Port, pin##_Pin, GPIO_PIN_RESET)
#define gpio_high(pin)	HAL_GPIO_WritePin(pin##_GPIO_Port, pin##_Pin, GPIO_PIN_SET)
#define gpio_ishigh(pin)	(HAL_GPIO_ReadPin(pin##_GPIO_Port, pin##_Pin) == GPIO_PIN_SET)
#define gpio_toggle(pin)	HAL_GPIO_TogglePin(pin##_GPIO_Port, pin##_Pin)
#define status_led_high() HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET)
#define status_led_low() HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET)
#define status_led_toggle() HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin)
#define mode_led_high() HAL_GPIO_WritePin(MODE_LED_GPIO_Port, MODE_LED_Pin, GPIO_PIN_SET)
#define mode_led_low() HAL_GPIO_WritePin(MODE_LED_GPIO_Port, MODE_LED_Pin, GPIO_PIN_RESET)
#define mode_led_toggle() HAL_GPIO_TogglePin(MODE_LED_GPIO_Port, MODE_LED_Pin)
#define hold_flash() HAL_GPIO_WritePin(ICE40_HOLD_GPIO_Port, ICE40_HOLD_Pin, GPIO_PIN_RESET)
#define release_flash() HAL_GPIO_WritePin(ICE40_HOLD_GPIO_Port, ICE40_HOLD_Pin, GPIO_PIN_SET)
#define protect_flash() HAL_GPIO_WritePin(ICE40_WP_GPIO_Port, ICE40_WP_Pin, GPIO_PIN_RESET)
#define free_flash() HAL_GPIO_WritePin(ICE40_WP_GPIO_Port, ICE40_WP_Pin, GPIO_PIN_SET)

extern SPI_HandleTypeDef hspi3;
extern USBD_HandleTypeDef hUsbDeviceFS;

static uint16_t crc;
static int cdc_stopped;
static int mode = 0;

static uint8_t icebuf[64];
static int nice;

/*
 * Dummy memory allocator for newlib, so we can call snprintf
 */
caddr_t
_sbrk(int incr)
{
	errno = ENOMEM;
	return (caddr_t) -1;
}

/*
 * Readahead fifo for input
 */
static struct fifo {
	int head, tail, max;
	uint8_t buf[CDC_FIFOSIZE];
} in_fifo;

static int
fifo_put(struct fifo *f, int c)
{
	int tl;
	int count;

	tl = f->tail;
	count = tl - f->head;
	if (count < 0)
		count += CDC_FIFOSIZE;
	if (count > f->max)
		f->max = count;
	if (count == CDC_FIFOSIZE - 1)
		return -1;
	f->buf[tl++] = c;
	f->tail = tl & (CDC_FIFOSIZE-1);
	if (count > (3*CDC_FIFOSIZE)/4)
		cdc_stopped = 1;
	return OK;
}

static int
fifo_get(struct fifo *f, uint8_t *b)
{
	int hd;
	int count;

	hd = f->head;
	count = f->tail - hd;
	if (count < 0)
		count += CDC_FIFOSIZE;
	if (count == 0)
		return -1;
	*b = f->buf[hd++];
	f->head = hd & (CDC_FIFOSIZE-1);
	if (cdc_stopped && count < CDC_FIFOSIZE/4) {
		cdc_stopped = 0;
		USBD_CDC_ReceivePacket(&hUsbDeviceFS);
	}
	return OK;
}

/*
 * Write a string to usbcdc, and to uart1 if not detached,
 * adding an extra CR if it ends with LF
 */
static void
cdc_puts(char *s)
{
	char *p;

	for (p = s; *p; p++)
		;
	CDC_Transmit_FS((unsigned char*)s, p - s);
	if (p > s && p[-1] == '\n')
		cdc_puts("\r");
}

/*
 * Read one byte from input fifo, waiting until one is available
 */
static int
cdc_getc(uint8_t *b)
{
	while (in_fifo.head == in_fifo.tail)
		;
	return fifo_get(&in_fifo, b);
}

/*
 * Interrupt callback when a packet has been read from usbcdc
 */
static int8_t
usbcdc_rxcallback(uint8_t *data, uint32_t *len)
{
	int i;
	int n;

	n = *len;
	for (i = 0; i < n; i++) {
		if (fifo_put(&in_fifo, *data++) < 0) {
			cdc_puts("Fifo overflow!\n");
			return OK;
		}
	}
	if (!cdc_stopped)
		USBD_CDC_ReceivePacket(&hUsbDeviceFS);
	return OK;
}

/*
 * Enable reading from usbcdc in interrupt mode
 */
static void
usbcdc_startread(void)
{
	USBD_Interface_fops_FS.Receive = &usbcdc_rxcallback;
	cdc_stopped = 1;
}

/*
 * ICE Write bytes, bit bang byte Transmit
 */
static int
ice_write(uint8_t *p, uint32_t len)
{
	int ret,b,i;
	uint8_t d;

	ret = HAL_OK;
	for(i = 0; i < len; i++)
	{
		d = *p++;
		for(b = 0; b < 8; b++){
			if(d & 0x80) {
				HAL_GPIO_WritePin(GPIOB,SPI3_SCK_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB,SPI3_MISO_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,SPI3_SCK_Pin,GPIO_PIN_SET);
			} else {
				HAL_GPIO_WritePin(GPIOB,SPI3_MISO_Pin | SPI3_SCK_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB,SPI3_SCK_Pin,GPIO_PIN_SET);
			}
			d <<= 1;
		}
		gpio_high(SPI3_SCK);
	}
	return ret;
}

/*
 * Append one byte to output buffer for ice40.
 * If arg is negative, flush the buffer.
 */
static void
ice_write_buf(int b)
{
	if (b >= 0)
		icebuf[nice++] = b;
	if ((b < 0 && nice > 0) || nice == sizeof icebuf) {
		ice_write(icebuf, sizeof icebuf);
		nice = 0;
	}
}

// /*
//  * Write to spi3 in multiple chunks (HAL_SPI_Transmit length is limited to 16 bits)
//  */
// static int
// spi_write(uint8_t *p, uint32_t len)
// {
// 	int ret;
// 	uint16_t n;
//
// 	ret = HAL_OK;
// 	n = 0x8000;
// 	while (len > 0) {
// 		if (len < n)
// 			n = len;
// 		ret = HAL_SPI_Transmit(&hspi3, p, n, HAL_MAX_DELAY);
// 		if (ret != HAL_OK)
// 			return ret;
// 		len -= n;
// 		p += n;
// 	}
// 	return ret;
// }

/*
 * Tri-state spi3 pins which are shared with ice40 LED1-4 signals
 */
static void
spi_detach(void)
{
	HAL_SPI_MspDeInit(&hspi3);
	HAL_GPIO_DeInit(SPI3_MISO_GPIO_Port, SPI3_MISO_Pin);
	HAL_GPIO_DeInit(SPI3_SCK_GPIO_Port, SPI3_SCK_Pin);
	HAL_GPIO_DeInit(ICE40_SPI_CS_GPIO_Port, ICE40_SPI_CS_Pin);
}

/*
 * Reconnect the spi3 pins
 */
static void
spi_reattach(void)
{
	GPIO_InitTypeDef g;

	HAL_SPI_MspInit(&hspi3);

	g.Pin = SPI3_MISO_Pin;
	g.Mode = GPIO_MODE_OUTPUT_PP;
	g.Pull = GPIO_NOPULL;
	g.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SPI3_MISO_GPIO_Port, &g);

	g.Pin = SPI3_SCK_Pin;
	g.Mode = GPIO_MODE_OUTPUT_PP;
	g.Pull = GPIO_NOPULL;
	g.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SPI3_SCK_GPIO_Port, &g);

	g.Pin = ICE40_SPI_CS_Pin;
	g.Mode = GPIO_MODE_OUTPUT_PP;
	g.Pull = GPIO_NOPULL;
	g.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(ICE40_SPI_CS_GPIO_Port, &g);
}

/*
 * Reset the ICE40 while holding SPI_SS_B low to force a spi-slave configuration
 */
static int
ice40_reset(void)
{
	int timeout;

	gpio_low(ICE40_CRST);
	gpio_low(ICE40_SPI_CS);
	HAL_Delay(1);
	gpio_high(ICE40_CRST);
	timeout = 100;
	while (gpio_ishigh(ICE40_CDONE)) {
		if (--timeout == 0)
			return TIMEOUT;
	}
	HAL_Delay(2);
	return OK;
}

/*
 * Wait for end of ICE40 configuration
 */
static int
ice40_configdone(void)
{
	uint8_t b = 0;

	cdc_puts(VER);
	for (int timeout = 100; !gpio_ishigh(ICE40_CDONE); timeout--) {
		if (timeout == 0) {
			cdc_puts("CDONE not set\n");
			return ICE_ERROR;
		}
		ice_write(&b, 1);
	}
	for (int i = 0; i < 7; i++)
		ice_write(&b, 1);
	cdc_puts("Config done\n");
	return OK;
}

/*
 * Update bitstream checksum
 */
static void
crc_update(uint8_t b)
{
	int c, v;

	v = b << 8;
	c = crc;
	for (int i = 0; i < 8; i++) {
		int x = 0;
		if ((c ^ v) & 0x8000)
			x = 0x1021;
		c = (c << 1) ^ x;
		v <<= 1;
	}
	crc = c;
}

/*
 * Restart bitstream checksum
 */
static void
crc_reset(void)
{
	crc = 0xFFFF;
}

/*
 * Read a byte from uart, update checksum, and send it to spi3
 */
static int
rbyte_uart_send(uint8_t *b, uint16_t *crc)
{
	if (b == NULL) {
		ice_write_buf(-1);
		return OK;
	}
	if (cdc_getc(b) != OK)
		return -1;
	ice_write_buf(*b);
	crc_update(*b);
	return OK;
}

/*
 * Read n bytes using the given reader
 */
static int
rbytes(Reader rbyte, int n, uint8_t *b, uint16_t *crc)
{
	while (n-- > 0) {
		if (rbyte(b, crc) < 0)
			return -1;
	}
	return 0;
}

/*
 * Read and parse a bitstream using the given reader
 */
static int
rbits(Reader rbyte, int firstb)
{
	int preamble;
	int crc_checked = 0;
	uint8_t b;
	int cmd, len, arg;
	int width = 0, height = 0;

	/* find the synchronising marker */
	preamble = firstb;
	while (preamble != 0x7EAA997E) {
		if (rbyte(&b, &crc) < 0)
			return -1;
		preamble <<= 8;
		preamble |= b;
	}

	/* parse the bitstream to find crc reset+check commands */
	while (rbyte(&b, &crc) == 0) {
		cmd = b >> 4;
		len = b & 0xF;
		arg = 0;
		while (len-- > 0) {
			if (rbyte(&b, &crc) < 0)
				return -1;
			arg <<= 8;
			arg |= b;
		}
		switch (cmd) {
		default:	/* unknown */
			return -1;
		case 1:		/* current bank */
		case 5:		/* frequency range */
		case 8:		/* offset of section */
		case 9:		/* warm boot */
			break;
		case 2:		/* check crc */
			if (crc != 0) {
				cdc_puts("CRC error..");
				return -1;
			}
			break;
		case 6:		/* width of section */
			width = arg + 1;
			break;
		case 7:		/* height of section */
			height = arg;
			break;
		case 0:
			switch (arg) {
			default:	/* unknown */
				return -1;
			case 1:		/* CRAM data */
			case 3:		/* BRAM data */
				if (rbytes(rbyte, height*width/8, &b, &crc) < 0)
					return -1;
				if (rbyte(&b, &crc) < 0 || b != 0)
					return -1;
				if (rbyte(&b, &crc) < 0 || b != 0)
					return -1;
				break;
			case 5:		/* crc reset */
				crc_reset();
				break;
			case 6:		/* wakeup */
				if (!crc_checked)
					return -1;
				/* discard the final padding byte added by icepack */
				rbyte(&b, &crc);
				rbyte(NULL, NULL);
				return OK;
			}
		}
		crc_checked = (cmd == 2);
	}
	return -1;
}

/*
 * Setup function (called once at powerup)
 *	- flush any input in uart buffer
 *	- if there's a bitstream in flash, send it to the ice40
 */
void
setup(void)
{
	mode_led_high();
	status_led_low();
	cdc_puts(VER);
	cdc_puts("\n");
	crc_reset();
	cdc_puts("Setup done\n");
	usbcdc_startread();
}

/*
 * Loop function (called repeatedly)
 *	- wait for the start of a bitstream to be received on uart1 or usbcdc
 *	- receive the bitstream and pass it to the ice40
 */
void
loop(void)
{
	uint8_t b = 0;
	static int err = 0;

	if (err) {
		status_led_toggle();
		HAL_Delay(100);
		if(gpio_ishigh(MODE_BOOT)) {
			err = 0;
			status_led_low();
		}
		return;
	}
	cdc_puts("Waiting for USB serial\n");
	do {
		if(gpio_ishigh(MODE_BOOT)) {
			mode_led_toggle();
			mode = ~ mode;
			HAL_Delay(1000);
		}
		if (cdc_stopped && USBD_CDC_ReceivePacket(&hUsbDeviceFS) == OK)
			cdc_stopped = 0;
		fifo_get(&in_fifo, &b);
	} while (b != 0x7E);
	status_led_high();
	protect_flash();
	hold_flash();
	spi_reattach();
	err = ice40_reset();
	if (err)
		return;
	crc_reset();
	ice_write(&b, 1);
	crc_update(b);
	if ((err = rbits(rbyte_uart_send, b)) != OK) {
		cdc_puts("rbits failed\n");
		return;
	}
	err = ice40_configdone();
	spi_detach();
	release_flash();
	free_flash();
	status_led_low();
}

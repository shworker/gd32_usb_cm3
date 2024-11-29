/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2014 Daniel Thompson <daniel@redfelineninja.org.uk>
 * Copyright (C) 2015 Piotr Esden-Tempski <piotr@esden.net>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/audio.h>
#include <libopencm3/usb/midi.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/desig.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/pwr.h>

#include "buttons.h"
#include "commands.h"
#include "usart.h"
#include "microrl.h"
#include "microrl_callbacks.h"
#include "uptime.h"
#include "utils.h"
#include "midiqueue.h"
#include "encoders.h"
#include "rpot.h"

usbd_device *usbd_dev;

volatile uint32_t Timer;
volatile uint8_t SysFlag;
volatile uint8_t LedFlag;

#define ENCODER_MAX				255
#define ENCODER_MIN_IDLE		10

#define PITCH_MAX		100
#define PITCH_MIN		10
#define PITCH_STEP		5

#define ENCODER_POLL_PERIOD		5
#define BUTTON_POLL_PERIOD		20
#define MIDI_SEND_PERIOD		100
#define ONE_SECOND_PERIOD		1000

#define STATUS_GPIO				GPIOB
#define STATUS_PIN				GPIO3

#define VERSION_MAJOR           1
#define VERSION_MINOR           3

#define COPYRIGHT				"(C) Dmitry Saychenko <sadmitry@gmail.com>"

#define USB_DPLUS_GPIO			GPIOA
#define USB_DPLUS_PIN			GPIO12

Encoder_t Encoders[2];
uint8_t Encoder_MIDI_Values[2];

// create microrl object and pointer on it
microrl_t rl;
microrl_t *prl = &rl;

void CmdSendSPI(int argc, const char * const * argv);
void CmdSendMIDI(int argc, const char * const * argv);
void CmdUpTime(int argc, const char * const * argv);
void CmdStatus(int argc, const char * const * argv);
void CmdReset(int argc, const char * const * argv);
void CmdHelp(int argc, const char * const * argv);
void CmdVersion(int argc, const char * const * argv);

Command_t Commands[] = {
		{"uptime","Print uptime","uptime",0,CmdUpTime},
		{"spi","Send SPI data","spi byte1 byte2 byte3",3,CmdSendSPI},
		{"midi","Send MIDI command","midi command byte1 byte2 byte3 byte4",4,CmdSendMIDI},
		{"status","Print current buttons status","status",0,CmdStatus},
		{"version","Print Armon version","version",0,CmdVersion},
		{"reset","Module reset","reset",0,CmdReset},
		{"help","Print help","help",0,CmdHelp},
		{NULL,NULL,NULL,0,NULL}
};

#define NUM_BUTTONS				4

#define BUTTON_TYPE_BUTTON		1
#define BUTTON_TYPE_ENCODER		2

typedef struct {
	uint32_t gpioport;
	uint16_t gpiopin;
	uint8_t button_type;
	uint8_t enc_index;
} ButtonPins_t;

ButtonPins_t GpioButtons[] = {
		{GPIOB, GPIO2, BUTTON_TYPE_BUTTON, 0},
		{GPIOB, GPIO0, BUTTON_TYPE_BUTTON, 0},
		{GPIOB, GPIO8, BUTTON_TYPE_ENCODER, 1},
		{GPIOB, GPIO9, BUTTON_TYPE_ENCODER, 0}
};

ButtonState_t button_State[NUM_BUTTONS];

uint8_t ButtonRead(int button_indexn);
void ButtonCallBack(int button_index, uint8_t state);
void DoSendMIDI(char *buf);
void EncoderCallback(int enc, uint32_t value);

/*
 * All references in this file come from Universal Serial Bus Device Class
 * Definition for MIDI Devices, release 1.0.
 */

/*
 * Table B-1: MIDI Adapter Device Descriptor
 */
static const struct usb_device_descriptor dev = {
		.bLength = USB_DT_DEVICE_SIZE,
		.bDescriptorType = USB_DT_DEVICE,
		.bcdUSB = 0x0200,    /* was 0x0110 in Table B-1 example descriptor */
		.bDeviceClass = 0,   /* device defined at interface level */
		.bDeviceSubClass = 0,
		.bDeviceProtocol = 0,
		.bMaxPacketSize0 = 64,
		.idVendor = 0x6666,  /* Prototype product vendor ID */
		.idProduct = 0x5119, /* dd if=/dev/random bs=2 count=1 | hexdump */
		.bcdDevice = 0x0100,
		.iManufacturer = 1,  /* index to string desc */
		.iProduct = 2,       /* index to string desc */
		.iSerialNumber = 3,  /* index to string desc */
		.bNumConfigurations = 1,
};

/*
 * Midi specific endpoint descriptors.
 */
static const struct usb_midi_endpoint_descriptor midi_bulk_endp[] = {{
		/* Table B-12: MIDI Adapter Class-specific Bulk OUT Endpoint
		 * Descriptor
		 */
		.head = {
				.bLength = sizeof(struct usb_midi_endpoint_descriptor),
				.bDescriptorType = USB_AUDIO_DT_CS_ENDPOINT,
				.bDescriptorSubType = USB_MIDI_SUBTYPE_MS_GENERAL,
				.bNumEmbMIDIJack = 1,
		},
		.jack[0] = {
				.baAssocJackID = 0x01,
		},
}, {
		/* Table B-14: MIDI Adapter Class-specific Bulk IN Endpoint
		 * Descriptor
		 */
		.head = {
				.bLength = sizeof(struct usb_midi_endpoint_descriptor),
				.bDescriptorType = USB_AUDIO_DT_CS_ENDPOINT,
				.bDescriptorSubType = USB_MIDI_SUBTYPE_MS_GENERAL,
				.bNumEmbMIDIJack = 1,
		},
		.jack[0] = {
				.baAssocJackID = 0x03,
		},
} };

/*
 * Standard endpoint descriptors
 */
static const struct usb_endpoint_descriptor bulk_endp[] = {{
		/* Table B-11: MIDI Adapter Standard Bulk OUT Endpoint Descriptor */
		.bLength = USB_DT_ENDPOINT_SIZE,
		.bDescriptorType = USB_DT_ENDPOINT,
		.bEndpointAddress = 0x01,
		.bmAttributes = USB_ENDPOINT_ATTR_BULK,
		.wMaxPacketSize = 0x40,
		.bInterval = 0x00,

		.extra = &midi_bulk_endp[0],
		.extralen = sizeof(midi_bulk_endp[0])
}, {
		.bLength = USB_DT_ENDPOINT_SIZE,
		.bDescriptorType = USB_DT_ENDPOINT,
		.bEndpointAddress = 0x81,
		.bmAttributes = USB_ENDPOINT_ATTR_BULK,
		.wMaxPacketSize = 0x40,
		.bInterval = 0x00,

		.extra = &midi_bulk_endp[1],
		.extralen = sizeof(midi_bulk_endp[1])
} };

/*
 * Table B-4: MIDI Adapter Class-specific AC Interface Descriptor
 */
static const struct {
	struct usb_audio_header_descriptor_head header_head;
	struct usb_audio_header_descriptor_body header_body;
} __attribute__((packed)) audio_control_functional_descriptors = {
		.header_head = {
				.bLength = sizeof(struct usb_audio_header_descriptor_head) +
				1 * sizeof(struct usb_audio_header_descriptor_body),
				.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
				.bDescriptorSubtype = USB_AUDIO_TYPE_HEADER,
				.bcdADC = 0x0100,
				.wTotalLength =
						sizeof(struct usb_audio_header_descriptor_head) +
						1 * sizeof(struct usb_audio_header_descriptor_body),
						.binCollection = 1,
		},
		.header_body = {
				.baInterfaceNr = 0x01,
		},
};

/*
 * Table B-3: MIDI Adapter Standard AC Interface Descriptor
 */
static const struct usb_interface_descriptor audio_control_iface[] = {{
		.bLength = USB_DT_INTERFACE_SIZE,
		.bDescriptorType = USB_DT_INTERFACE,
		.bInterfaceNumber = 0,
		.bAlternateSetting = 0,
		.bNumEndpoints = 0,
		.bInterfaceClass = USB_CLASS_AUDIO,
		.bInterfaceSubClass = USB_AUDIO_SUBCLASS_CONTROL,
		.bInterfaceProtocol = 0,
		.iInterface = 0,

		.extra = &audio_control_functional_descriptors,
		.extralen = sizeof(audio_control_functional_descriptors)
} };

/*
 * Class-specific MIDI streaming interface descriptor
 */
static const struct {
	struct usb_midi_header_descriptor header;
	struct usb_midi_in_jack_descriptor in_embedded;
	struct usb_midi_in_jack_descriptor in_external;
	struct usb_midi_out_jack_descriptor out_embedded;
	struct usb_midi_out_jack_descriptor out_external;
} __attribute__((packed)) midi_streaming_functional_descriptors = {
		/* Table B-6: Midi Adapter Class-specific MS Interface Descriptor */
		.header = {
				.bLength = sizeof(struct usb_midi_header_descriptor),
				.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
				.bDescriptorSubtype = USB_MIDI_SUBTYPE_MS_HEADER,
				.bcdMSC = 0x0100,
				.wTotalLength = sizeof(midi_streaming_functional_descriptors),
		},
		/* Table B-7: MIDI Adapter MIDI IN Jack Descriptor (Embedded) */
		.in_embedded = {
				.bLength = sizeof(struct usb_midi_in_jack_descriptor),
				.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
				.bDescriptorSubtype = USB_MIDI_SUBTYPE_MIDI_IN_JACK,
				.bJackType = USB_MIDI_JACK_TYPE_EMBEDDED,
				.bJackID = 0x01,
				.iJack = 0x00,
		},
		/* Table B-8: MIDI Adapter MIDI IN Jack Descriptor (External) */
		.in_external = {
				.bLength = sizeof(struct usb_midi_in_jack_descriptor),
				.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
				.bDescriptorSubtype = USB_MIDI_SUBTYPE_MIDI_IN_JACK,
				.bJackType = USB_MIDI_JACK_TYPE_EXTERNAL,
				.bJackID = 0x02,
				.iJack = 0x00,
		},
		/* Table B-9: MIDI Adapter MIDI OUT Jack Descriptor (Embedded) */
		.out_embedded = {
				.head = {
						.bLength = sizeof(struct usb_midi_out_jack_descriptor),
						.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
						.bDescriptorSubtype = USB_MIDI_SUBTYPE_MIDI_OUT_JACK,
						.bJackType = USB_MIDI_JACK_TYPE_EMBEDDED,
						.bJackID = 0x03,
						.bNrInputPins = 1,
				},
				.source[0] = {
						.baSourceID = 0x02,
						.baSourcePin = 0x01,
				},
				.tail = {
						.iJack = 0x00,
				}
		},
		/* Table B-10: MIDI Adapter MIDI OUT Jack Descriptor (External) */
		.out_external = {
				.head = {
						.bLength = sizeof(struct usb_midi_out_jack_descriptor),
						.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
						.bDescriptorSubtype = USB_MIDI_SUBTYPE_MIDI_OUT_JACK,
						.bJackType = USB_MIDI_JACK_TYPE_EXTERNAL,
						.bJackID = 0x04,
						.bNrInputPins = 1,
				},
				.source[0] = {
						.baSourceID = 0x01,
						.baSourcePin = 0x01,
				},
				.tail = {
						.iJack = 0x00,
				},
		},
};

/*
 * Table B-5: MIDI Adapter Standard MS Interface Descriptor
 */
static const struct usb_interface_descriptor midi_streaming_iface[] = {{
		.bLength = USB_DT_INTERFACE_SIZE,
		.bDescriptorType = USB_DT_INTERFACE,
		.bInterfaceNumber = 1,
		.bAlternateSetting = 0,
		.bNumEndpoints = 2,
		.bInterfaceClass = USB_CLASS_AUDIO,
		.bInterfaceSubClass = USB_AUDIO_SUBCLASS_MIDISTREAMING,
		.bInterfaceProtocol = 0,
		.iInterface = 0,

		.endpoint = bulk_endp,

		.extra = &midi_streaming_functional_descriptors,
		.extralen = sizeof(midi_streaming_functional_descriptors)
} };

static const struct usb_interface ifaces[] = {{
		.num_altsetting = 1,
		.altsetting = audio_control_iface,
}, {
		.num_altsetting = 1,
		.altsetting = midi_streaming_iface,
} };

/*
 * Table B-2: MIDI Adapter Configuration Descriptor
 */
static const struct usb_config_descriptor config = {
		.bLength = USB_DT_CONFIGURATION_SIZE,
		.bDescriptorType = USB_DT_CONFIGURATION,
		.wTotalLength = 0, /* can be anything, it is updated automatically
			      when the usb code prepares the descriptor */
		.bNumInterfaces = 2, /* control and data */
		.bConfigurationValue = 1,
		.iConfiguration = 0,
		.bmAttributes = 0x80, /* bus powered */
		.bMaxPower = 0x32,

		.interface = ifaces,
};

static char usb_serial_number[25]; /* 12 bytes of desig and a \0 */

static const char *usb_strings[] = {
		"SilverAudio",
		"MIDI controller",
		usb_serial_number
};

void ClockSetup(void){
	rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);

	rcc_periph_clock_enable(RCC_AFIO);
	rcc_periph_clock_enable(RCC_USART2);
	rcc_periph_clock_enable(RCC_TIM3);
	rcc_periph_clock_enable(RCC_TIM4);
	rcc_periph_clock_enable(RCC_SPI1);
}

void PrintMIDIBuffer(char *buffer){
	Printf("xcxcxcx",buffer[0], ' ', buffer[1], ' ', buffer[2], ' ', buffer[3]);
}

void PrintVersion(void){
	CLI_Print("Armon version: ");
	CLI_Put_Int(VERSION_MAJOR);
	CLI_Print(".");
	CLI_Put_Int(VERSION_MINOR);
}

void PrintUpTime(void){

	//	uint8_t end1;
	uint32_t d;

	d = UpTimeGetDays(); CLI_Put_Int(d);
	if(1 == d){ CLI_Print(" Day "); } else{ CLI_Print(" Days "); }

	d = UpTimeGetHours(); CLI_Put_Int(d);
	if(1== d){ CLI_Print(" hour "); } else { CLI_Print(" hours "); }

	d = UpTimeGetMinutes(); CLI_Put_Int(d);
	if(1== d) { CLI_Print(" minute "); } else { CLI_Print(" minutes "); }

	d = UpTimeGetSeconds(); CLI_Put_Int(d);
	if(1== d) { CLI_Print(" second"); } else { CLI_Print(" seconds"); }
}

void CmdSendSPI(int argc, const char * const * argv){
	uint8_t buf[3];
	uint8_t i;

	buf[0] = hex2int(argv[1]);
	buf[1] = hex2int(argv[2]);
	buf[2] = hex2int(argv[3]);

	Printf("sxcxcx","Sending: ", buf[0], ' ', buf[1], ' ', buf[2]);

	for(i = 0; i < 3; i++){
		spi_send(SPI1, (uint8_t) buf[i]);
	}
}

void CmdSendMIDI(int argc, const char * const * argv){
	char buf[4];

	buf[0] = hex2int(argv[1]);
	buf[1] = hex2int(argv[2]);
	buf[2] = hex2int(argv[3]);
	buf[3] = hex2int(argv[4]);

	PrintMIDIBuffer(buf);
	//	while (usbd_ep_write_packet(usbd_dev, 0x81, buf, sizeof(buf)) == 0);
	DoSendMIDI(buf);
}

void CmdUpTime(int argc, const char * const * argv){ PrintUpTime(); }
void CmdStatus(int argc, const char * const * argv){ CLI_Print("Status\r\n"); }
void CmdReset(int argc, const char * const * argv){ scb_reset_system(); }
void CmdHelp(int argc, const char * const * argv){ Print_Help(); }
void CmdVersion(int argc, const char * const * argv){ PrintVersion(); }

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];

/* SysEx identity message, preformatted with correct USB framing information */
const uint8_t sysex_identity[] = {
		0x04,	/* USB Framing (3 byte SysEx) */
		0xf0,	/* SysEx start */
		0x7e,	/* non-realtime */
		0x00,	/* Channel 0 */
		0x04,	/* USB Framing (3 byte SysEx) */
		0x7d,	/* Educational/prototype manufacturer ID */
		0x66,	/* Family code (byte 1) */
		0x66,	/* Family code (byte 2) */
		0x04,	/* USB Framing (3 byte SysEx) */
		0x51,	/* Model number (byte 1) */
		0x19,	/* Model number (byte 2) */
		0x00,	/* Version number (byte 1) */
		0x04,	/* USB Framing (3 byte SysEx) */
		0x00,	/* Version number (byte 2) */
		0x01,	/* Version number (byte 3) */
		0x00,	/* Version number (byte 4) */
		0x05,	/* USB Framing (1 byte SysEx) */
		0xf7,	/* SysEx end */
		0x00,	/* Padding */
		0x00,	/* Padding */
};

static void usbmidi_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
	(void)ep;

	char buf[64];
	int len = usbd_ep_read_packet(usbd_dev, 0x01, buf, 64);

	/* This implementation treats any message from the host as a SysEx
	 * identity request. This works well enough providing the host
	 * packs the identify request in a single 8 byte USB message.
	 */
	if (len) {
		while (usbd_ep_write_packet(usbd_dev, 0x81, sysex_identity,
				sizeof(sysex_identity)) == 0);
	}
}

static void usbmidi_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
	(void)wValue;

	usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64,
			usbmidi_data_rx_cb);
	usbd_ep_setup(usbd_dev, 0x81, USB_ENDPOINT_ATTR_BULK, 64, NULL);
}

void UsbSetupSerialNumber(void){
	char uid[16];
	memset(uid, 0, 16);
	desig_get_unique_id_as_dfu(uid);
	strcpy(usb_serial_number, uid);
}

void DoSendMIDI(char *buf){
	MIDIQueue_Put(buf);
	CLI_Print(" qs=");
	CLI_Put_Int(MIDIQueue_Get_Size());
	CLI_Print(" ");
}

static void button_send_event(usbd_device *usbd_dev, int button, int pressed)
{
	char buf[4] = { 0x0B,
			0xB5,
			60 + button,
			0 };

	DoSendMIDI(buf);
}

void Buttons_Init(void){
	uint8_t i;

	for(i = 0; i < NUM_BUTTONS; i++){
		gpio_set_mode(GpioButtons[i].gpioport, GPIO_MODE_INPUT,
				GPIO_CNF_INPUT_PULL_UPDOWN, GpioButtons[i].gpiopin);
		// Internal pull-up for button
		gpio_set(GpioButtons[i].gpioport, GpioButtons[i].gpiopin);
	}
}

uint8_t ButtonRead(int button_indexn){
	uint8_t state = gpio_get(GpioButtons[button_indexn].gpioport, GpioButtons[button_indexn].gpiopin);;
	if(0 == state){
		gpio_toggle(STATUS_GPIO, STATUS_PIN);
	}
	return state;
}

void ButtonCallBack(int button_index, uint8_t state){
	if(BUTTON_STATE_PRESSED == state){
		if(BUTTON_TYPE_BUTTON == GpioButtons[button_index].button_type){
			Printf("ds", button_index, ": PB ");
			button_send_event(usbd_dev, button_index, 1);
		}
		if(BUTTON_TYPE_ENCODER == GpioButtons[button_index].button_type){
			Encoder_MIDI_Values[GpioButtons[button_index].enc_index] = PITCH_MAX / 2;
			Printf("dsds", button_index, ": PE ", Encoder_MIDI_Values[GpioButtons[button_index].enc_index], " ");
		}
	}

	if(BUTTON_STATE_RELEASED == state){
		Printf("ds", button_index, ": R ");
		button_send_event(usbd_dev, button_index, 0);
	}
}

static void DoSendEncMIDI(uint8_t enc, uint32_t value){
	if(Encoders[enc].idle_counter >= ENCODER_MIN_IDLE){
		char buf[4] = { 0x0E,
				0xE5 + enc,
				value,
				0 };

		DoSendMIDI(buf);
	}
}

void EncoderCallback(int enc, uint32_t value){

	if(ENC_DIR_UP == Encoders[enc].direction){
		if(Encoder_MIDI_Values[enc] < PITCH_MAX){
			Encoder_MIDI_Values[enc] += PITCH_STEP;
			Printf("dsdsds", enc, ": ", Encoder_MIDI_Values[enc], " U ", Encoders[enc].idle_counter, "\r\n");
			DoSendEncMIDI(enc, Encoder_MIDI_Values[enc]);
		}else{
			Encoder_MIDI_Values[enc] = PITCH_MAX;
		}
	}

	if(ENC_DIR_DOWN == Encoders[enc].direction){
		if(Encoder_MIDI_Values[enc] > PITCH_MIN){
			Encoder_MIDI_Values[enc] -= PITCH_STEP;
			Printf("dsdsds", enc, ": ", Encoder_MIDI_Values[enc], " D ", Encoders[enc].idle_counter, "\r\n");
			DoSendEncMIDI(enc, Encoder_MIDI_Values[enc]);
		}else{
			Encoder_MIDI_Values[enc] = PITCH_MIN;
		}
	}
}

void sys_tick_handler(void)
{
	SysFlag = 1;
	Timer++;
}

void systick_setup(void){

	/* 72MHz / 8 => 9000000 counts per second */
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
	/* 9000000/9000 = 1000 overflows per second - every 1ms one interrupt */
	/* SysTick interrupt every N clock pulses: set reload to N-1 */
	systick_set_reload(8999);

	systick_interrupt_enable();

	/* Start counting. */
	systick_counter_enable();
}

void SPISetup(void){
	/* Configure GPIOs: SCK=PA5 and MOSI=PA7 */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
			GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO5 | GPIO7 );
	/* Reset SPI, SPI_CR1 register cleared, SPI is disabled */
	rcc_periph_reset_pulse(RST_SPI2);
	spi_init_master(SPI2, SPI_CR1_BAUDRATE_FPCLK_DIV_64, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
			SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
	spi_enable_software_slave_management(SPI2);
	spi_set_nss_high(SPI2);

	spi_enable(SPI2);
}

void EncoderSetup(void){
	// Timer pins
	gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO4 | GPIO5 | GPIO6 | GPIO7);
	gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON, AFIO_MAPR_TIM3_REMAP_PARTIAL_REMAP);

	memset(Encoders, 0, sizeof(Encoder_t) * 2);

	Encoders[0].timer = TIM3;
	Encoders[1].timer = TIM4;

	Encoders_Init(ENCODER_MAX, EncoderCallback, Encoders, 2);
}

// MicroRL callbacks
void clitask(void){
	// put received char from USART to microrl lib
	if(CLI_Get_Received()){
		// if received char then
		// put received char from USART to microrl lib
		// without waiting
		microrl_insert_char(prl, CLI_Get_Char());
	}
}

int execute (int argc, const char * const * argv){

	Command_t *cmd;

	CLI_Print("\r\n");

	if(argc > 0){
		cmd = Find_Command(argc, argv);
		if(NULL == cmd){
			CLI_Print("Invalid command !\r\n");
		}else{
			CMDFUNC cmdfunc = cmd->handler;
			(cmdfunc)(argc, argv);
		}
	}

	CLI_Print("\r\n");

	return 0;
}

int main(void)
{
	int i;
	unsigned long CurTimer;

	SysFlag = 0;
	LedFlag = 1;
	Timer = 0;

	ClockSetup();

	AFIO_MAPR |= AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON;

	gpio_set_mode(STATUS_GPIO, GPIO_MODE_OUTPUT_50_MHZ,
			GPIO_CNF_OUTPUT_PUSHPULL, STATUS_PIN);

	SPISetup();

	EncoderSetup();

	Buttons_Init();
	SetButtons(button_State, NUM_BUTTONS);
	SetButtonCallbacks(ButtonCallBack, ButtonRead);

	USART2Init(115200);

	microrl_init(prl, CLI_Print);
	microrl_set_execute_callback(prl, execute);
	InitCommands(CLI_Print, Commands);

	UpTimeInit();

	MIDIQueue_Init();

	CLI_Print("\r\n");
	PrintVersion();
	CLI_Print("\r\n");
	CLI_Print(COPYRIGHT);
	CLI_Print("\r\nStart\r\n");

	Printf("xs", GPIO_CRH(GPIOB), "\r\n");

	systick_setup();

	CLI_Print("USB Start\r\n");
	/* USB pin D+ */
	gpio_set_mode(USB_DPLUS_GPIO, GPIO_MODE_OUTPUT_2_MHZ,
			GPIO_CNF_OUTPUT_PUSHPULL, USB_DPLUS_PIN);
	gpio_clear(USB_DPLUS_GPIO, USB_DPLUS_PIN);

	for (i = 0; i < 800000; i++)    /* Wait a bit. */
		__asm__("nop");

	UsbSetupSerialNumber();

	usbd_dev = usbd_init(&st_usbfs_v1_usb_driver,
			&dev,
			&config,
			usb_strings,
			3,
			usbd_control_buffer,
			sizeof(usbd_control_buffer));

	usbd_register_set_config_callback(usbd_dev, usbmidi_set_config);

	CLI_Print("Press RETURN to get started!\r\n");

	while (1) {
		usbd_poll(usbd_dev);

		if(1 == SysFlag){
			SysFlag = 0;

			CurTimer = Timer;

			if(0 == (CurTimer % ENCODER_POLL_PERIOD)){
				EncoderPoll(0);
				EncoderPoll(1);
			}
			if(0 == (CurTimer % BUTTON_POLL_PERIOD)){
				PollButtons();
			}
			if(0 == (CurTimer % MIDI_SEND_PERIOD)){
				MIDIQueue_Poll();
			}
			if(0 == (CurTimer % ONE_SECOND_PERIOD)){
				UpTimeUpdate();
			}
			clitask();
		}
	}
}

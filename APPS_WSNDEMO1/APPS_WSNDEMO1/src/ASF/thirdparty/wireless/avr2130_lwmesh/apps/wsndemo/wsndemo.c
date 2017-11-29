/**
 * \file WSNDemo.c
 *
 * \brief WSNDemo application implementation
 *
 * Copyright (C) 2014 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 *
 */

/*
 * Copyright (c) 2014, Atmel Corporation All rights reserved.
 *
 * Licensed under Atmel's Limited License Agreement --> EULA.txt
 */

/**
 * \mainpage
 * \section preface Preface
 * This is the reference manual for the WSN Demo Application Application
 * The WSNDemo application implements a typical wireless sensor network
 *scenario,
 * in which one central node collects the data from a network of sensors and
 *passes this data over a serial connection for further processing.
 * In the case of the WSNDemo this processing is performed by the WSNMonitor PC
 *application. The BitCloud庐 Quick Start Guide  provides a detailed description
 *of the WSNDemo application scenario, and instructions on how to use
 *WSNMonitor.
 *  However since BitCloud is a ZigBee庐 PRO stack, there are a few differences
 *in the protocol:
 * 鈥Device types (Coordinator, Router and End Device) are simulated on the
 *application level; there is no such separation in Lightweight Mesh on the
 *stack level
 * 鈥The value of the extended address field is set equal to the value of the
 *short address field
 * 鈥For all frames, the LQI and RSSI fields are filled in by the coordinator
 *with the values of LQI and RSSI from the received frame. This means that nodes
 *that are not connected to the coordinator directly will have the same values
 *as the last node on the route to the coordinator
 * 鈥Sensor data values are generated randomly on all platforms
 * 鈥Sending data to the nodes on the network is not implemented and not
 *supported in this demo application
 */
 /**
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "config.h"
#include "sys.h"
#include "phy.h"
#include "sys.h"
#include "nwk.h"
#include "sysTimer.h"
#if APP_ENDDEVICE
#include "sleep_mgr.h"
#endif
#include "commands.h"
#if APP_COORDINATOR
#include "sio2host.h"
#endif
#if SAMD || SAMR21
#include "system.h"
#else
#include "sysclk.h"
#if (LED_COUNT > 0)
#include "led.h"
#endif
#endif
#include "asf.h"
#include "board.h"
#include "wsndemo.h"

//#include "adc.h"  //added this
/*****************************************************************************
*****************************************************************************/

#define APP_CAPTION_SIZE  (sizeof(APP_CAPTION) - 1)

/*- Types ------------------------------------------------------------------*/
COMPILER_PACK_SET(1)
typedef struct  AppMessage_t {
	uint8_t commandId;
	uint8_t nodeType;
	uint64_t extAddr;
	uint16_t shortAddr;
	uint32_t softVersion;
	uint32_t channelMask;
	uint16_t panId;
	uint8_t workingChannel;
	uint16_t parentShortAddr;
	uint8_t lqi;
	int8_t rssi;

	struct {
		uint8_t type;
		uint8_t size;
		int32_t battery;
		int32_t temperature;
		int32_t light;
	} sensors;

	struct {
		uint8_t type;
		uint8_t size;
		char text[APP_CAPTION_SIZE];
	} caption;
} AppMessage_t;

typedef enum AppState_t {
	APP_STATE_INITIAL,
	APP_STATE_SEND,
	APP_STATE_WAIT_CONF,
	APP_STATE_SENDING_DONE,
	APP_STATE_WAIT_SEND_TIMER,
	APP_STATE_WAIT_COMMAND_TIMER,
	APP_STATE_PREPARE_TO_SLEEP,
	APP_STATE_SLEEP,
	APP_STATE_WAKEUP,
} AppState_t;
COMPILER_PACK_RESET()
/*- Variables --------------------------------------------------------------*/
static AppState_t appState = APP_STATE_INITIAL;

#if APP_ROUTER || APP_ENDDEVICE
static NWK_DataReq_t appNwkDataReq;
static SYS_Timer_t appNetworkStatusTimer;
static SYS_Timer_t appCommandWaitTimer;
static bool appNetworkStatus;
#endif

#if APP_COORDINATOR
static uint8_t rx_data[APP_RX_BUF_SIZE];
#endif

static AppMessage_t appMsg;
static SYS_Timer_t appDataSendingTimer;
#define APP_COMMAND_PENDING 0x01
void UartBytesReceived(uint16_t bytes, uint8_t *byte );

// $$ MY DEFINES... $$ //
//#define MY_LED    IOPORT_CREATE_PIN(PORTA, 5)
//#define MY_BUTTON IOPORT_CREATE_PIN(PORTA, 6)

/*- Implementations --------------------------------------------------------*/

//Federico's added stuff for ADC read
void Simple_Clk_Init(void);
// Ports
void enable_port(void);
//EIC
void enable_EIC_clocks(void);
void config_EIC(void);
Eic *portEIC = EIC;
void battery_handler(void);
int BatteryLife = 0x7A;
// ADC
void enable_adc_clocks(void);
void init_adc(void);
int read_adc(void);
Adc *portADC = ADC;
volatile int z = 0;				// variable to transfer the reading function over to another variable (resultADC)
//volatile unsigned int resultADC = 81;
volatile uint8_t resultADC = 81;
// ADC From module
#if APP_COORDINATOR

/*****************************************************************************
*****************************************************************************/
void UartBytesReceived(uint16_t bytes, uint8_t *byte )
{
	for (uint16_t i = 0; i < bytes; i++) {
		APP_CommandsByteReceived(byte[i]);
	}
}

static void appUartSendMessage(uint8_t *data, uint8_t size)
{
	uint8_t cs = 0;   //some kind of counter for error detection on the other end?  

	sio2host_putchar(0x10);
	sio2host_putchar(0x02);

	for (uint8_t i = 0; i < size; i++) {
		if (data[i] == 0x10) {
			sio2host_putchar(0x10);
			cs += 0x10;
		}

		sio2host_putchar(data[i]);
		cs += data[i];
	}

	sio2host_putchar(0x10);
	sio2host_putchar(0x03);
	cs += 0x10 + 0x02 + 0x10 + 0x03;  //escape character + 

	sio2host_putchar(cs); //we append this big number to the end, possible checksum value for error detection on the other side of UART
}

#endif

/*****************************************************************************
*****************************************************************************/
static bool appDataInd(NWK_DataInd_t *ind)
{
	AppMessage_t *msg = (AppMessage_t *)ind->data;
#if (LED_COUNT > 0)
	LED_Toggle(LED_DATA);
#endif
	msg->lqi = ind->lqi;
	msg->rssi = ind->rssi;
#if APP_COORDINATOR
	appUartSendMessage(ind->data, ind->size);

	if (APP_CommandsPending(ind->srcAddr)) {
		NWK_SetAckControl(APP_COMMAND_PENDING);
	}
#endif
	return true;
}

/*****************************************************************************
*****************************************************************************/
static void appDataSendingTimerHandler(SYS_Timer_t *timer)
{
	if (APP_STATE_WAIT_SEND_TIMER == appState) {
		appState = APP_STATE_SEND;
	} else {
		SYS_TimerStart(&appDataSendingTimer);
	}

	(void)timer;
}

#if APP_ROUTER || APP_ENDDEVICE

/*****************************************************************************
*****************************************************************************/
static void appNetworkStatusTimerHandler(SYS_Timer_t *timer)
{
#if (LED_COUNT > 0)
	LED_Toggle(LED_NETWORK);
#endif
	(void)timer;
}

/*************************************************************************//**
*****************************************************************************/
static void appCommandWaitTimerHandler(SYS_Timer_t *timer)
{
	appState = APP_STATE_SENDING_DONE;
	(void)timer;
}

#endif

/*****************************************************************************
*****************************************************************************/
#if APP_ROUTER || APP_ENDDEVICE
static void appDataConf(NWK_DataReq_t *req)
{
#if (LED_COUNT > 0)
	LED_Off(LED_DATA);
#endif

	if (NWK_SUCCESS_STATUS == req->status) {
		if (!appNetworkStatus) {
#if (LED_COUNT > 0)
			LED_On(LED_NETWORK);
#endif
			SYS_TimerStop(&appNetworkStatusTimer);
			appNetworkStatus = true;
		}
	} else {
		if (appNetworkStatus) {
#if (LED_COUNT > 0)
			LED_Off(LED_NETWORK);
#endif
			SYS_TimerStart(&appNetworkStatusTimer);
			appNetworkStatus = false;
		}
	}

	if (APP_COMMAND_PENDING == req->control) {
		SYS_TimerStart(&appCommandWaitTimer);
#if (LED_COUNT > 0)
		LED_Toggle(LED_NETWORK);
#endif
		appState = APP_STATE_WAIT_COMMAND_TIMER;
	} else {
		appState = APP_STATE_SENDING_DONE;
	}
}

#endif

/*****************************************************************************
*****************************************************************************/
static void appSendData(void)
{
#ifdef NWK_ENABLE_ROUTING
	appMsg.parentShortAddr = NWK_RouteNextHop(0, 0);
#else
	appMsg.parentShortAddr = 0;
#endif
//$$$ change this to alter data sent to network $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ //
  
  //Get state of the on board switch SW0
  uint8_t switchData = 0x46;
  bool pinRead;
  pinRead = port_pin_get_input_level(BUTTON_0_PIN);
  if(pinRead==false){
	  switchData = 0x54;
  }
  //Read and store the ADC for the temperature sensor
  //uint8_t temperatureData;
  
  resultADC = read_adc();
  
  appMsg.sensors.battery     =	0x46;//switchData; //thisData;		//0x42;//B for battery //rand() & 0xffff;
  appMsg.sensors.temperature =	0x42;//T for temp //rand() & 0x7f;
  appMsg.sensors.light       =	0x53;//S to indicate position of sensor data in array//read_adc();//L for light //rand() & 0xff;
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ //

#if APP_COORDINATOR
	appUartSendMessage((uint8_t *)&appMsg, sizeof(appMsg));
	SYS_TimerStart(&appDataSendingTimer);
	appState = APP_STATE_WAIT_SEND_TIMER;
#else
	appNwkDataReq.dstAddr = 0;
	appNwkDataReq.dstEndpoint = APP_ENDPOINT;
	appNwkDataReq.srcEndpoint = APP_ENDPOINT;
	appNwkDataReq.options = NWK_OPT_ACK_REQUEST | NWK_OPT_ENABLE_SECURITY;
	appNwkDataReq.data = (uint8_t *)&appMsg;
	appNwkDataReq.size = sizeof(appMsg);
	appNwkDataReq.confirm = appDataConf;
#if (LED_COUNT > 0)
	LED_On(LED_DATA);
#endif
	NWK_DataReq(&appNwkDataReq);

	appState = APP_STATE_WAIT_CONF;
#endif
}

/*************************************************************************//**
*****************************************************************************/
static void appInit(void)
{
	appMsg.commandId            = APP_COMMAND_ID_NETWORK_INFO;
	appMsg.nodeType             = APP_NODE_TYPE;
	appMsg.extAddr              = APP_ADDR;
	appMsg.shortAddr            = APP_ADDR;
	appMsg.softVersion          = 0x01010100;
	appMsg.channelMask          = (1L << APP_CHANNEL);
	appMsg.panId                = APP_PANID;
	appMsg.workingChannel       = APP_CHANNEL;
	appMsg.parentShortAddr      = 0;
	appMsg.lqi                  = 0;
	appMsg.rssi                 = 0;

	appMsg.sensors.type        = 1;
	appMsg.sensors.size        = sizeof(int32_t) * 3;
	appMsg.sensors.battery     = 0;
	appMsg.sensors.temperature = 0;
	appMsg.sensors.light       = 0;

	appMsg.caption.type         = 32;
	appMsg.caption.size         = APP_CAPTION_SIZE;
	memcpy(appMsg.caption.text, APP_CAPTION, APP_CAPTION_SIZE);

	NWK_SetAddr(APP_ADDR);
	NWK_SetPanId(APP_PANID);
	PHY_SetChannel(APP_CHANNEL);
#if (defined(PHY_AT86RF212B) || defined(PHY_AT86RF212))
	PHY_SetBand(APP_BAND);
	PHY_SetModulation(APP_MODULATION);
#endif
	PHY_SetRxState(true);

#ifdef NWK_ENABLE_SECURITY
	NWK_SetSecurityKey((uint8_t *)APP_SECURITY_KEY);
#endif

	NWK_OpenEndpoint(APP_ENDPOINT, appDataInd);

	appDataSendingTimer.interval = APP_SENDING_INTERVAL;
	appDataSendingTimer.mode = SYS_TIMER_INTERVAL_MODE;
	appDataSendingTimer.handler = appDataSendingTimerHandler;

#if APP_ROUTER || APP_ENDDEVICE
	appNetworkStatus = false;
	appNetworkStatusTimer.interval = 500;
	appNetworkStatusTimer.mode = SYS_TIMER_PERIODIC_MODE;
	appNetworkStatusTimer.handler = appNetworkStatusTimerHandler;
	SYS_TimerStart(&appNetworkStatusTimer);

	appCommandWaitTimer.interval = NWK_ACK_WAIT_TIME;
	appCommandWaitTimer.mode = SYS_TIMER_INTERVAL_MODE;
	appCommandWaitTimer.handler = appCommandWaitTimerHandler;
#else
#if (LED_COUNT > 0)
	LED_On(LED_NETWORK);
#endif
#endif

#ifdef PHY_ENABLE_RANDOM_NUMBER_GENERATOR
	srand(PHY_RandomReq());
#endif

	APP_CommandsInit();

	appState = APP_STATE_SEND;
}

/*************************************************************************//**
*****************************************************************************/
static void APP_TaskHandler(void)
{
	switch (appState) {
	case APP_STATE_INITIAL:
	{
		appInit();
	}
	break;

	case APP_STATE_SEND:
	{
		appSendData();
	}
	break;

	case APP_STATE_SENDING_DONE:
	{
#if APP_ENDDEVICE
		appState = APP_STATE_PREPARE_TO_SLEEP;
#else
		SYS_TimerStart(&appDataSendingTimer);
		appState = APP_STATE_WAIT_SEND_TIMER;
#endif
	}
	break;

#if APP_ENDDEVICE
	case APP_STATE_PREPARE_TO_SLEEP:
	{
		if (!NWK_Busy()) {
			NWK_SleepReq();
			appState = APP_STATE_SLEEP;
		}
	}
	break;

	case APP_STATE_SLEEP:
	{
		sm_sleep(APP_SENDING_INTERVAL / 1000);
		appState = APP_STATE_WAKEUP;
	}
	break;

	case APP_STATE_WAKEUP:
	{
		NWK_WakeupReq();

		/*
		 * #if (LED_COUNT > 0)
		 *    LED_On(LED_NETWORK);
		 #endif*/

		appState = APP_STATE_SEND;
	}
	break;
#endif
	default:
		break;
	}

#if (APP_COORDINATOR)
	uint16_t bytes;
	if ((bytes = sio2host_rx(rx_data, APP_RX_BUF_SIZE)) > 0) {
		UartBytesReceived(bytes, (uint8_t *)&rx_data);
	}
#endif
}

/*****************************************************************************
*****************************************************************************/

/**
 * Init function of the WSNDemo application
 */
void wsndemo_init(void) 
{
	SYS_Init();
	
	//my init stuff for pins 
	struct port_config config_port_pin;
	port_get_config_defaults(&config_port_pin);
	config_port_pin.direction  = PORT_PIN_DIR_INPUT;
	config_port_pin.input_pull = PORT_PIN_PULL_UP;
	port_pin_set_config(BUTTON_0_PIN, &config_port_pin);
	config_port_pin.direction = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(LED_0_PIN, &config_port_pin);
	////////////////////////////////////////////////
	// enable the pins we use
	enable_port();
	
	// ADC initialization
	//struct adc_config config;
	//adc_get_config_defaults(&config);
	enable_adc_clocks();		// sets the general clock
	init_adc();
	
	//EIC initialization
	
#if APP_ENDDEVICE
	sm_init();
#endif
#if APP_COORDINATOR
	sio2host_init();
#endif
	
}


/**
 * Task of the WSNDemo application
 * This task should be called in a while(1)
 */
void wsndemo_task(void)
{
	//resultADC = read_adc();
	SYS_TaskHandler();
	APP_TaskHandler();
}
//FEDERICO'S ADDED CODE $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

//added code, think about moving to main.c
// setting up pin values to test 
void enable_port(void){
	
	Port *ports = PORT;
	// not necessary but can come in handy 
	PortGroup *portA = &(ports->Group[0]);
	
	// clears pin PA06 as an input
	// PA06 has no I2C, PA09 does but is set in the ALTERNATE on the board (may not matter)
	portA->DIRCLR.reg = 1<<6;
	// make PA06 owned by the adc
	portA->PMUX[3].bit.PMUXE = 0x1;
	portA->PINCFG[6].bit.PMUXEN = 1;
	
	// setting pins PA18 and PA05 as outputs
	portA->DIRCLR.reg = 1<<5|1<<18;
	// having PA18 and PA05 owned by the EIC
	portA->PMUX[2].bit.PMUXO = 0x00;
	portA->PINCFG[5].bit.PMUXEN = 1;
	portA->PMUX[9].bit.PMUXE = 0x00;
	portA->PINCFG[18].bit.PMUXEN = 1;
	
	// setting pin PA18 as an output, could connect to the pi, think about it later 
	// need to check if its configurable with the adc (its not)
	// PA18 EIC - EXTINT[2], SERCOM - SERCOM1/PAD[2], NO I2C
	// PA13		- EXTINT[13],		- SERCOM2/PAD[1], HAS I2C	(may not matter which pin)
	// sets pins as output to send the data
	//portA->DIRSET.reg = 1<<18|1<<13;
	
	// this is only necessary is the input is by a button
	// portA->PINCFG[6].reg = PORT_PINCFG_INEN | PORT_PINCFG_PULLEN;
}
/*
void enable_EIC_clocks(void){
	PM->APBAMASK.reg |= 0x1 << 6; // PM_APBAMASK for EIC is in 6th bit
	uint32_t temp= 0x03;		// ID for EIC
	temp |= 0<<8;         			//  Selection Generic clock generator 0
	GCLK->CLKCTRL.reg=temp;   		//  Setup in the CLKCTRL register
	GCLK->CLKCTRL.reg |= 0x1u << 14;
}
*/
void config_EIC(void){
	// disable EIC
	portEIC->CTRL.reg = 0x00;
	
	portEIC->EVCTRL.reg = 1<<5|1<<2;	// PA05 and PA18
	portEIC->WAKEUP.reg = 1<<5|1<<2;
	portEIC->CONFIG[0].bit.SENSE5 = 0x4;
	portEIC->INTENSET.reg = 1<<5|1<<2;
	
	portEIC->CTRL.reg = 0x02;
}

void battery_handler(void){
	
	Port *ports = PORT;
	PortGroup *portA = &(ports->Group[0]);
	
	if(portA->IN.reg & 1<<18){
		BatteryLife = BatteryLife + 0x01;
	}
	else{
		BatteryLife = BatteryLife - 0x01;
	}
	portEIC->INTFLAG.reg = 1<<5;
}


void enable_adc_clocks(void){
	
	PM->APBCMASK.reg |= 01u<<16; // PM_APBCMASK for ADC is located at the 16th bit
	uint32_t temp = 0x1e;
	temp |= 0<<8;
	GCLK->CLKCTRL.reg = temp;
	GCLK->CLKCTRL.reg |= 0x1u<<14;
}

void init_adc(void){
	
	portADC->CTRLA.reg = 0<<1;
	
	// unsure which values to choose here, so i had some values from EE138Lab5 to act as dummy values
	portADC->REFCTRL.reg = 0x02;
	portADC->AVGCTRL.reg = 0x44;	// adjusting the ADJRES and the SAMPLENUM
	portADC->SAMPCTRL.reg = 0;		// need to look more into this value, but will be set to 0 for now
	portADC->CTRLB.bit.PRESCALER = 0x7;	// peripheral clock divided by 512 for now
	portADC->CTRLB.bit.RESSEL = 0x1;	// resolution is set to 16bits
	portADC->INPUTCTRL.bit.GAIN = 0xF;	// gain is 1/2 
	portADC->INPUTCTRL.bit.MUXNEG = 0x19;	// 0x19 is the I/O ground
	portADC->INPUTCTRL.bit.MUXPOS = 0x06;	// 0x06 is PA06 which is set to be owned by the ADC
	
	portADC->CTRLA.reg = 1<<1;
}
int read_adc(void)  //program gets stuck in here as of 2:16pm 11.28.17
{

	// start the conversion
	portADC->SWTRIG.bit.START = 1;
	portADC->INTFLAG.reg = 0x01;
	while((portADC->INTFLAG.bit.RESRDY));  //see if this while loop is what's causing us to get stuck
	//wait for conversion to be available
	z = portADC->RESULT.reg;
	return(z); 					//insert register where ADC store value
}
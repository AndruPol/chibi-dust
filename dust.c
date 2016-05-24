/*
 * dust.c - Sharp GP2Y1010AU0F dust and smoke sensor ChibiOS 2.6.10+ driver
 *
 *  Created on: 22.05.2016
 *      Author: Andrey Polyakov
 */

/*
 * BOARD STM32F103C_MINI	MCU	STM32F103C8T6
 *
 * PERIPHERALS	MODES	REMAP	FUNCTIONS	PINS
 * ADC1			IN1		0		ADC1_IN1	PA1
 * TIM3	Internal Clock	TIM3_VS_ClockSourceINT	VP_TIM3_VS_ClockSourceINT
 *
 * PA1	ADC1_IN1
 * PB13	GPIO_Output	ILED
 */

#include "dust.h"

#include "ch.h"
#include "hal.h"

/* ILED interval width:
 * ILED_START_US = 280
 * ILED_LAST_US = 2
 * -O0 : ~322uS ChibiOS debug ALL, ~307uS ChibiOS debug NO
 * -O2 : ~308uS ChibiOS debug ALL, ~295uS ChibiOS debug NO
 * ILED_LAST in 2-25
 */
#define ILED_START_US			280		// datasheet: 280us
#define ILED_LAST_US			2		// datasheet: 40us, 2-25 RTOS & compiler configuration
#define ILED_SLEEP_US			9680	// datasheet: 10mS = ILED_START_US + ILED_LAST_US + ILED_SLEEP_US

#define DUST_TIMEOUT_MS			2
#define ADC_TIMEOUT_US			10

#define ILED_GPIO				GPIOB			// LED on/off
#define ILED_PIN				GPIOB_PIN13		// LED on/off

#define VO_GPIO					GPIOA			// ADC pin
#define VO_PIN					GPIOA_PIN1		// ADC pin
#define VO_CHANNEL				ADC_CHANNEL_IN1	// ADC channel

#define DUSTGPT					GPTD3		// Timer, precision interval

#define NUM_CHANNELS			1
#define DUST_CH					0

BinarySemaphore adcsem;						// ADC share semaphore
static BinarySemaphore dust_cbsem;			// ADC read semaphore
static adcsample_t samples[NUM_CHANNELS];	// ADC samples buffer
static volatile dust_error_t dust_error;	// dust sensor error flag

static void adccallback(ADCDriver *adcp, adcsample_t *buffer, size_t n);
static void adcerrcallback(ADCDriver *adcp, adcerror_t err);

#define DUST_PRIO				(NORMALPRIO+1)
static WORKING_AREA(waDUSTThread, 128);

/*
 * ADC conversion group.
 * Mode:        Linear buffer, 1 samples of 1 channel, SW triggered.
 * Channels:    IN1 (1.5 cycles sampling time)
 */
static const ADCConversionGroup adcgrpcfg = {
  FALSE,			//circular
  NUM_CHANNELS,		//number of channels
  adccallback,		//adc callback function
  adcerrcallback,	//error callback function
  /* HW dependent part.*/
  0,	//cr1
  0,	//cr2
  //SMPR1 register
//  ADC_SMPR1_SMP_VREF(ADC_SAMPLE_239P5) |
//  ADC_SMPR1_SMP_SENSOR(ADC_SAMPLE_239P5) |
//  ADC_SMPR1_SMP_AN15(ADC_SAMPLE_239P5) |
//  ADC_SMPR1_SMP_AN14(ADC_SAMPLE_239P5) |
//  ADC_SMPR1_SMP_AN13(ADC_SAMPLE_239P5) |
//  ADC_SMPR1_SMP_AN12(ADC_SAMPLE_239P5) |
//  ADC_SMPR1_SMP_AN11(ADC_SAMPLE_239P5) |
//  ADC_SMPR1_SMP_AN10(ADC_SAMPLE_239P5) |
  0,
  //SMPR2 register
//  ADC_SMPR2_SMP_AN9(ADC_SAMPLE_239P5) |
//  ADC_SMPR2_SMP_AN8(ADC_SAMPLE_239P5) |
//  ADC_SMPR2_SMP_AN7(ADC_SAMPLE_239P5) |
//  ADC_SMPR2_SMP_AN6(ADC_SAMPLE_239P5) |
//  ADC_SMPR2_SMP_AN5(ADC_SAMPLE_239P5) |
//  ADC_SMPR2_SMP_AN4(ADC_SAMPLE_239P5) |
//  ADC_SMPR2_SMP_AN3(ADC_SAMPLE_239P5) |
//  ADC_SMPR2_SMP_AN2(ADC_SAMPLE_239P5) |
  ADC_SMPR2_SMP_AN1(ADC_SAMPLE_1P5) |
//  ADC_SMPR2_SMP_AN0(ADC_SAMPLE_239P5) |
  0,
  //SQR1 register
//  ADC_SQR1_SQ16_N(ADC_CHANNEL_SENSOR) |
//  ADC_SQR1_SQ15_N(ADC_CHANNEL_IN15) |
//  ADC_SQR1_SQ14_N(ADC_CHANNEL_IN14) |
//  ADC_SQR1_SQ13_N(ADC_CHANNEL_IN13) |
  ADC_SQR1_NUM_CH(NUM_CHANNELS),
  //SQR2 register
//  ADC_SQR2_SQ12_N(ADC_CHANNEL_IN12) |
//  ADC_SQR2_SQ11_N(ADC_CHANNEL_IN11) |
//  ADC_SQR2_SQ10_N(ADC_CHANNEL_IN10) |
//  ADC_SQR2_SQ9_N(ADC_CHANNEL_IN9) |
//  ADC_SQR2_SQ8_N(ADC_CHANNEL_IN8) |
//  ADC_SQR2_SQ7_N(ADC_CHANNEL_IN7) |
  0,
  //SQR3 register
//  ADC_SQR3_SQ6_N(ADC_CHANNEL_IN9) |
//  ADC_SQR3_SQ5_N(ADC_CHANNEL_IN8) |
//  ADC_SQR3_SQ4_N(ADC_CHANNEL_IN8) |
//  ADC_SQR3_SQ3_N(ADC_CHANNEL_IN7) |
//  ADC_SQR3_SQ2_N(ADC_CHANNEL_VREFINT) |
  ADC_SQR3_SQ1_N(VO_CHANNEL) |
  0
};

/*
 * ADC end conversion callback
 */
static void adccallback(ADCDriver *adcp, adcsample_t *buffer, size_t n) {
	(void) buffer;
	(void) n;
	if (adcp->state == ADC_COMPLETE){
		chSysLockFromIsr();
		chBSemSignalI(&dust_cbsem);
		chSysUnlockFromIsr();
	}
}

static void adcerrcallback(ADCDriver *adcp, adcerror_t err) {
  (void)adcp;
  (void)err;
  if (adcp->state == ADC_ERROR){
	  dust_error = DUST_CONV_ERROR;
	  chSysLockFromIsr();
	  chBSemSignalI(&dust_cbsem);
	  chSysUnlockFromIsr();
  }
}

static const GPTConfig GPTCfg = {
  1000000,	 // 1 MHz timer clock.
  NULL,		 // No callback
  0,
};

/*
 *  dust sensor read process
 */
static Thread *pDUSTThread;
__attribute__((noreturn))
msg_t DUSTThread(void *arg) {
	(void)arg;
	chRegSetThreadName("DUSTThd");
	chBSemInit(&dust_cbsem,TRUE);

	while (TRUE) {
		msg_t req;
		Thread *tp;

		tp = chMsgWait();
		req = chMsgGet(tp);
		chMsgRelease(tp, (msg_t) req);

		dust_error = DUST_NO_ERROR;
		chBSemReset(&dust_cbsem,TRUE);

		// start read cycle
		palClearPad(ILED_GPIO, ILED_PIN);
		gptPolledDelay(&DUSTGPT, ILED_START_US);

		adcStartConversion(&ADCD1, &adcgrpcfg, samples, 1);
		if (chBSemWaitTimeout(&dust_cbsem, US2ST(ADC_TIMEOUT_US)) == RDY_TIMEOUT){
			dust_error = DUST_TIMEOUT;
		}
		gptPolledDelay(&DUSTGPT, ILED_LAST_US);
		palSetPad(ILED_GPIO, ILED_PIN);
		chBSemSignal(&adcsem);
		chThdSleepMicroseconds(ILED_SLEEP_US);
	}
}

/*
 * Initializes the ADC driver 1.
 */
void dust_init(void){

	palSetPadMode(ILED_GPIO, ILED_PIN, PAL_MODE_OUTPUT_PUSHPULL);	//
	palSetPad(ILED_GPIO, ILED_PIN);	//  not active

	adcStart(&ADCD1, NULL);
	palSetPadMode(VO_GPIO, VO_PIN, PAL_MODE_INPUT_ANALOG);	// 01: PA1 -> ADC1 channel 1

	gptStart(&DUSTGPT, &GPTCfg);

	chBSemInit(&adcsem,FALSE);
	pDUSTThread = chThdCreateStatic(waDUSTThread, sizeof(waDUSTThread), DUST_PRIO, DUSTThread, NULL);
}

dust_error_t dust_read(dust_read_t *value) {
	msg_t msg;
	chBSemWait(&adcsem); /* to be sure */

	chMsgSend(pDUSTThread, (msg_t) &msg);

	/* wait for reply */
	if(chBSemWaitTimeout(&adcsem, MS2ST(DUST_TIMEOUT_MS)) == RDY_TIMEOUT) {
		return DUST_TIMEOUT;
	}
	chBSemReset(&adcsem, FALSE);
	if (dust_error == DUST_NO_ERROR){
		value->raw = samples[DUST_CH];
		value->voltage = samples[DUST_CH] * 3.25 / 4096.0;	// VREF = 3.25V
		// linear eqaution taken from http://www.howmuchsnow.com/arduino/airquality/
		// Chris Nafis (c) 2012
		// Dust Density, mg/m3
		//dustDensity = 0.17 * value->voltage - 0.1;
		return DUST_NO_ERROR;
	}
	return dust_error;
}

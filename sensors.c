/*
 * sensors.c
 *
 *  Created on: 01.04.2013
 *      Author: andru
 */

/*
 * BOARD STM32F103C_MINI	MCU	STM32F103C8T6
 *
 * PERIPHERALS	MODES	REMAP	FUNCTIONS	PINS
 * ADC1			IN4		0		ADC1_IN4	PA4
 * ADC1			IN5		0		ADC1_IN5	PA5
 * ADC1			IN7		0		ADC1_IN7	PA7
 * ADC1			IN8		0		ADC1_IN8	PB0
 */

#include "ch.h"
#include "hal.h"

#include "sensors.h"

adc_read_t sensors;							// описание датчиков
static BinarySemaphore adcsem, adc_cbsem;	// семафор управления процессом adc

static adcsample_t samples[SENSORSALL];	// буфер чтения АЦП

static void adccallback(ADCDriver *adcp, adcsample_t *buffer, size_t n);
static void adcerrcallback(ADCDriver *adcp, adcerror_t err);

/*
 * ADC conversion group.
 * Mode:        Linear buffer, 1 samples of 17 channels, SW triggered.
 * Channels:    IN0-17 (239.5 cycles sampling time)
 */
static const ADCConversionGroup adcgrpcfg = {
  FALSE,			//circular
  SENSORSALL,		//number of channels
  adccallback,		//adc callback function
  adcerrcallback,	//error callback function
  /* HW dependent part.*/
  0,	//cr1
  0,	//cr2
  //SMPR1 register
  ADC_SMPR1_SMP_VREF(ADC_SAMPLE_239P5) |
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
  ADC_SMPR2_SMP_AN8(ADC_SAMPLE_239P5) |
  ADC_SMPR2_SMP_AN7(ADC_SAMPLE_239P5) |
//  ADC_SMPR2_SMP_AN6(ADC_SAMPLE_239P5) |
  ADC_SMPR2_SMP_AN5(ADC_SAMPLE_239P5) |
  ADC_SMPR2_SMP_AN4(ADC_SAMPLE_239P5) |
//  ADC_SMPR2_SMP_AN3(ADC_SAMPLE_239P5) |
//  ADC_SMPR2_SMP_AN2(ADC_SAMPLE_239P5) |
//  ADC_SMPR2_SMP_AN1(ADC_SAMPLE_239P5) |
//  ADC_SMPR2_SMP_AN0(ADC_SAMPLE_239P5) |
  0,
  //SQR1 register
//  ADC_SQR1_SQ16_N(ADC_CHANNEL_SENSOR) |
//  ADC_SQR1_SQ15_N(ADC_CHANNEL_IN15) |
//  ADC_SQR1_SQ14_N(ADC_CHANNEL_IN14) |
//  ADC_SQR1_SQ13_N(ADC_CHANNEL_IN13) |
  ADC_SQR1_NUM_CH(SENSORSALL),
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
  ADC_SQR3_SQ4_N(ADC_CHANNEL_IN8) |
  ADC_SQR3_SQ3_N(ADC_CHANNEL_IN7) |
  ADC_SQR3_SQ2_N(ADC_CHANNEL_IN5) |
  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN4) |
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
		chBSemSignalI(&adc_cbsem);
		chSysUnlockFromIsr();
	}
}

static void adcerrcallback(ADCDriver *adcp, adcerror_t err) {
  (void)adcp;
  (void)err;
  if (adcp->state == ADC_ERROR){
	  sensors.error = ADC_CONV_ERROR;
	  chSysLockFromIsr();
	  chBSemSignalI(&adc_cbsem);
	  chSysUnlockFromIsr();
  }
}

/*
 *  процесс опроса датчиков АЦП
 */
static Thread *ADCThread_p;
WORKING_AREA(waADCThread, 128);
__attribute__((noreturn))
msg_t ADCThread(void *arg) {
	chRegSetThreadName("ADCThd");
	chBSemInit(&adcsem,FALSE);
	chBSemInit(&adc_cbsem,TRUE);

	(void)arg;
	while (TRUE) {
		adc_read_t *req;
		Thread *tp;

		tp = chMsgWait();
		req = (adc_read_t *) chMsgGet(tp);
		chMsgRelease(tp, (msg_t) req);

		req->error = ADC_NO_ERROR;
		chBSemReset(&adc_cbsem,TRUE);
		adcStartConversion(&ADCD1, &adcgrpcfg, samples, 1);
		if (chBSemWaitTimeout(&adc_cbsem, MS2ST(ADC_TIMEOUT_MS)) == RDY_TIMEOUT){
			req->error = ADC_TIMEOUT;
		}
		if (req->error == ADC_NO_ERROR) {
			// датчик напряжения аккумулятора
			req->sensor[BATTSENSOR].value = samples[BATTSENSOR];
			// остальные датчики
			for (uint8_t i = 0; i < SENSORSNUM; i++){
				req->sensor[i].value = samples[i];
			}
		}
		chBSemSignal(&adcsem);
	}
}

/*
 * Initializes the ADC driver 1.
 */
void sensors_init(void){

	adcStart(&ADCD1, NULL);
	palSetPadMode(GPIOA, GPIOA_PIN4, PAL_MODE_INPUT_ANALOG);	// 00: PA.04 -> ADC1 channel 4
	palSetPadMode(GPIOA, GPIOA_PIN5, PAL_MODE_INPUT_ANALOG);	// 01: PA.05 -> ADC1 channel 5
	palSetPadMode(GPIOA, GPIOA_PIN7, PAL_MODE_INPUT_ANALOG);	// 02: PA.07 -> ADC1 channel 7
	palSetPadMode(GPIOB, GPIOB_PIN0, PAL_MODE_INPUT_ANALOG);	// 03: PB.00 -> ADC1 channel 8

	sensors.sensor[BATTSENSOR].type = ADC_BATTERY;
	ADCThread_p = chThdCreateStatic(waADCThread, sizeof(waADCThread), ADC_PRIO, ADCThread, NULL);
}

adc_error_t sensors_read(void) {
	adc_read_t *adc_read_p = &sensors;

	chBSemWait(&adcsem); /* to be sure */

	chMsgSend(ADCThread_p, (msg_t) adc_read_p);

	/* wait for reply */
	if(chBSemWaitTimeout(&adcsem, MS2ST(ADC_TIMEOUT_MS)) == RDY_TIMEOUT) {
		adc_read_p->error = ADC_TIMEOUT;
		return ADC_TIMEOUT;
	}
	chBSemReset(&adcsem, FALSE);
	if (adc_read_p->error == ADC_CONV_ERROR){
		return ADC_CONV_ERROR;
	}
	adc_read_p->error = ADC_NO_ERROR;
	return ADC_NO_ERROR;
}

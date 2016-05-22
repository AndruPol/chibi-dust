/*
 * sensors.h
 */
#ifndef SENSORS_H_
#define SENSORS_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "ch.h"
#include "hal.h"

#define SENSORSNUM		3 		// число внешних датчиков
#define SENSORSALL		4		// общее число датчиков
#define BATTSENSOR		3		// номер датчика контроля напряжения батареи
#define ADC_TIMEOUT_MS	200		// задержка опроса датчиков, мсек

#define ADC_ERROR		5		// 5 - 20% error
#define SENSOR_OK(v, n) ((v) <= ((n) + (n) / ADC_ERROR) && (v) >= ((n) - (n) / ADC_ERROR))

#define MOVENORM		770		// adc value for normal move sensor ~0.65V
#define MOVEPWRUPINT	60		// power up interval for move sensor
#define MOVEPWROFFINT	0		// power off interval for move sensor
#define MOVEFIREINT		5		// fired move sensor check delay
#define MOVEMAXERR		5		// max errors move sensor init

#define SMOKENORM		2280	// adc value for normal smoke sensor ~1.46V
#define SMOKEPWRUPINT	60		// power up interval for smoke sensor
#define SMOKEPWROFFINT	3		// power off interval for smoke sensor
#define SMOKEFIREINT	20		// fired smoke sensor check delay
#define SMOKEMAXERR		5		// max errors smoke sensor init
#define SMOKEFIRECNT	10		// continues fire count for smoke sensor

typedef enum {
	ADC_NO_ERROR,
	ADC_CONV_ERROR,
	ADC_TIMEOUT,
} adc_error_t;

typedef enum {
	ADC_NOTUSED = 0,	// not used
	ADC_MOVE,			// move sensor
	ADC_SMOKE,			// smoke sensor
	ADC_BATTERY			// battery voltage
} adc_alarm_t;

typedef struct _adc_delay_t adc_delay_t;
struct _adc_delay_t {
	bool_t active;		/* delay active */
	uint8_t delay;		/* delay value, s */
};

// описание датчика
typedef struct _adc_sensor_t adc_sensor_t;
struct _adc_sensor_t {
	adc_alarm_t type;		/* тип датчика: 0 - не исп, 1 - охранный, 2 - пожарный, 3 - батарея */
	uint16_t value;			/* значение датчика */
	uint16_t norma;			/* значение датчика принятое за норму */
	uint8_t errcnt;			/* ошибки датчика */
	uint8_t firecnt;		/* количество сработок, ошибок датчика после постановки */
	adc_delay_t fireint;	/* задержка проверки после сработки */
	adc_delay_t pwrupint;	/* задержка включения, сек */
	adc_delay_t pwroffint;	/* задержка выключения питания датчика */
};

typedef struct _adc_read_t adc_read_t;
struct _adc_read_t {
	adc_error_t error; 					/* out */
	adc_sensor_t sensor[SENSORSALL]; 	/* out */
};

#define ADC_PRIO	NORMALPRIO+1
extern adc_read_t sensors;			// описание датчиков

// инициализация АЦП
void sensors_init(void);
// чтение значений ADC
adc_error_t sensors_read(void);

#ifdef __cplusplus
}
#endif
#endif /* SENSORS_H_ */

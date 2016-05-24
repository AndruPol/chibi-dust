/*
 * dust.h
 */
#ifndef DUST_H_
#define DUST_H_

#include "ch.h"

typedef enum {
	DUST_NO_ERROR,
	DUST_CONV_ERROR,
	DUST_TIMEOUT,
} dust_error_t;

typedef struct _dust_read_t dust_read_t;
struct _dust_read_t {
	uint16_t raw;
	float voltage;
};

extern BinarySemaphore adcsem;

#ifdef __cplusplus
extern "C" {
#endif

// инициализация датчика пыли
void dust_init(void);
// чтение значения датчика пыли
dust_error_t dust_read(dust_read_t *value);

#ifdef __cplusplus
}
#endif

#endif /* DUST_H_ */

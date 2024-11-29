/*
 * encoders.h
 *
 *  Created on: Nov 14, 2024
 *      Author: dmitry
 */

#ifndef LIB_ENCODERS_H_
#define LIB_ENCODERS_H_

#include <inttypes.h>

#define ENC_DIR_UP		0
#define ENC_DIR_DOWN	1

typedef struct {
	uint32_t OldValue;
	uint32_t CurValue;
	uint8_t direction;
	uint32_t idle_counter;
	uint32_t timer;
} Encoder_t;

typedef void (*ENCFUNC)(int enc, uint32_t value);

#define CHK_BIT32(number, bit)  ((number & (1LL << bit)) >> bit)

void Encoders_Init(uint32_t period, ENCFUNC func, Encoder_t *encs, uint8_t encs_count);
void Encoder_Set_Counter(uint8_t enc, uint32_t value);
void EncoderPoll(uint8_t enc);

#endif /* LIB_ENCODERS_H_ */

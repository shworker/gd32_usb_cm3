/*
 * encoders.c
 *
 *  Created on: Nov 14, 2024
 *      Author: dmitry
 */


#include <libopencm3/stm32/timer.h>

#include "encoders.h"

static Encoder_t *Encs;
static uint32_t max_value;
static uint8_t nEncs;
static ENCFUNC encfunc;

static uint32_t rotary_encoder_get_value(uint32_t timer);
static void EncoderInit(uint32_t timer);

static void EncoderInit(uint32_t timer){
	timer_set_period(timer, max_value*4 );
	timer_slave_set_mode(timer, TIM_SMCR_SMS_EM3);
	timer_ic_set_filter(timer, TIM_IC1,  TIM_IC_DTF_DIV_32_N_8 );
	timer_ic_set_filter(timer, TIM_IC2,  TIM_IC_DTF_DIV_32_N_8 );
	timer_ic_set_input(timer, TIM_IC1, TIM_IC_IN_TI1);
	timer_ic_set_input(timer, TIM_IC2, TIM_IC_IN_TI2);
	timer_set_counter(timer, 0);
//	timer_enable_update_event(timer);
	timer_enable_counter(timer);
}

void Encoders_Init(uint32_t period, ENCFUNC func, Encoder_t *encs, uint8_t encs_count){

	uint8_t i;

	max_value = period;
	encfunc = func;
	Encs = encs;
	nEncs = encs_count;

	for(i = 0; i < nEncs; i++){
		EncoderInit(Encs[i].timer);
	}
}

static uint8_t IsTimCountingDown(uint32_t timer){
        return CHK_BIT32(TIM_CR1(timer), 4);
}

static uint32_t rotary_encoder_get_value(uint32_t timer){
	return (timer_get_counter(timer) >> 2);
}

//************************************************************************************
//************************************************************************************

void Encoder_Set_Counter(uint8_t enc, uint32_t value){
	timer_disable_counter(Encs[enc].timer);
	timer_set_counter(Encs[enc].timer, value);
	timer_enable_counter(Encs[enc].timer);
}

void EncoderPoll(uint8_t enc){
	Encs[enc].CurValue = rotary_encoder_get_value(Encs[enc].timer);
	if(Encs[enc].OldValue != Encs[enc].CurValue){
		Encs[enc].direction = IsTimCountingDown(Encs[enc].timer);
		encfunc(enc, Encs[enc].CurValue);
		Encs[enc].idle_counter = 0;
		Encs[enc].OldValue = Encs[enc].CurValue;
	}else{ Encs[enc].idle_counter++; }
}


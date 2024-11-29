/*
 * buttons.c
 *
 *  Created on: Jul 20, 2024
 *      Author: dmitry
 */

#include <stdint.h>
#include "buttons.h"

static ButtonState_t *Buttons;
static uint8_t NumButtons;
BUTTONREAD buttonread;
BUTTONFUNC buttonsfunc;

void SetButtons(ButtonState_t *btns, uint8_t nb){
	Buttons = btns;
	NumButtons = nb;
}

void SetButtonCallbacks(BUTTONFUNC func, BUTTONREAD readfunc){
	buttonread = readfunc;
	buttonsfunc = func;
}

void PollButtons(void){
	int i;

	for(i = 0; i < NumButtons; i++){

		if(0 == buttonread(i)){
			// Button pressed
			if(0 == Buttons[i].pressed_flag){
				Buttons[i].pressed_flag = 1;
			}else{
				if(BUTTON_STATE_IDLE == Buttons[i].prev_state){
					Buttons[i].cur_state = BUTTON_STATE_PRESSED;
					Buttons[i].prev_state = Buttons[i].cur_state;

					buttonsfunc(i, Buttons[i].cur_state);
				}
			}
		}else{
			// Button released
			if(Buttons[i].pressed_flag){
				if(BUTTON_STATE_PRESSED == Buttons[i].prev_state){
					Buttons[i].cur_state = BUTTON_STATE_RELEASED;
					Buttons[i].prev_state = Buttons[i].cur_state;
					Buttons[i].pressed_flag = 0;

					buttonsfunc(i, Buttons[i].cur_state);
				}
			}else{
				Buttons[i].cur_state = BUTTON_STATE_IDLE;
				Buttons[i].prev_state = BUTTON_STATE_IDLE;
				Buttons[i].pressed_flag = 0;
			}
		}
	} // for()
}

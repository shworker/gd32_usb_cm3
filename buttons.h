/*
 * buttons.h
 *
 *  Created on: Jul 20, 2024
 *      Author: dmitry
 */

#ifndef INC_BUTTONS_H_
#define INC_BUTTONS_H_

#define MIDI_BUTTONS_COUNT	11	// 9 buttons with led + 2 encoder

#define BUTTON_STATE_IDLE		0x00
#define BUTTON_STATE_PRESSED	0x01
#define BUTTON_STATE_RELEASED	0x02

#define BUTTON_UNKNOWN			0xFF

typedef struct {
	uint8_t cur_state;
	uint8_t prev_state;
	uint8_t pressed_flag;
} ButtonState_t;

typedef uint8_t(*BUTTONREAD)(int button_index);
typedef void (*BUTTONFUNC)(int button_index, uint8_t state);

void SetButtons(ButtonState_t *btns, uint8_t nb);
void SetButtonCallbacks(BUTTONFUNC func, BUTTONREAD readfunc);
void PollButtons(void);

#endif /* INC_BUTTONS_H_ */

//*****************************************************************************
//
// Copyright (c) 2017 Tony Mach.  Released under GNU GPL V3.
//
//	This file is part of project DORAK.
//
//	Project DORAK is free software: you can redistribute it and/or modify
//	it under the terms of the GNU General Public License as published by
//	the Free Software Foundation, either version 3 of the License, or
//	(at your option) any later version.
//
//	This program is distributed in the hope that it will be useful,
//	but WITHOUT ANY WARRANTY; without even the implied warranty of
//	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//	GNU General Public License for more details.
//
//	You should have received a copy of the GNU General Public License
//	along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//*****************************************************************************

#ifndef __D_HW_H__
#define __D_HW_H__

#include "d_def.h"

void d_hw_task(void);

#define D_MIN(a,b)	( (a)<(b) ? (a) : (b) )
#define D_MAX(a,b)	( (a)>(b) ? (a) : (b) )


#define D_LED_OFF	0
#define D_LED_ON	1



// LED matriX
#define D_LEDx_OUT	0x0001	// Middle bar
#define D_LEDx_M5	0x0002	// Low right
#define D_LEDx_M4	0x0004	// dot
#define D_LEDx_nil0	0x0008	// low bar
#define D_LEDx_CV	0x0010	// low left
#define D_LEDx_nil1	0x0020	// top bar
#define D_LEDx_nil2	0x0040	// top left
#define D_LEDx_M2	0x0080	// top right


#define D_LEDx_CC	0x0100	// Middle bar
#define D_LEDx_M3	0x0200	// Low right
#define D_LEDx_nil3	0x0400	// dot
#define D_LEDx_OCP	0x0800	// low bar
#define D_LEDx_nil4	0x1000	// low left
#define D_LEDx_OVP	0x2000	// top right !!!
#define D_LEDx_nil5	0x4000	// top left
#define D_LEDx_M1	0x8000	// top !!!


// bottom
#define D_LEDx_row1_mC	0x0001	// Middle center
#define D_LEDx_row1_bR	0x0002	// bottom right
#define D_LEDx_row1_DT	0x0004	// dot
#define D_LEDx_row1_bC	0x0008	// bottom center
#define D_LEDx_row1_bL	0x0010	// bottom left
#define D_LEDx_row1_tC	0x0020	// top center
#define D_LEDx_row1_tL	0x0040	// top left
#define D_LEDx_row1_tR	0x0080	// top right

// top
#define D_LEDx_row0_mC	0x0100	// Middle center
#define D_LEDx_row0_bR	0x0200	// bottom right
#define D_LEDx_row0_DT	0x0400	// dot
#define D_LEDx_row0_bC	0x0800	// bottom center
#define D_LEDx_row0_bL	0x1000	// bottom left
#define D_LEDx_row0_tR	0x2000	// top right !!!
#define D_LEDx_row0_tL	0x4000	// top left
#define D_LEDx_row0_tC	0x8000	// top center !!!

// bottom
#define D_LEDx_mC(row) ((row)?(D_LEDx_row1_mC):(D_LEDx_row0_mC))	// Middle center
#define D_LEDx_bR(row) ((row)?(D_LEDx_row1_bR):(D_LEDx_row0_bR))	// bottom right
#define D_LEDx_DT(row) ((row)?(D_LEDx_row1_DT):(D_LEDx_row0_DT))	// dot
#define D_LEDx_bC(row) ((row)?(D_LEDx_row1_bC):(D_LEDx_row0_bC))	// bottom center
#define D_LEDx_bL(row) ((row)?(D_LEDx_row1_bL):(D_LEDx_row0_bL))	// bottom left
#define D_LEDx_tR(row) ((row)?(D_LEDx_row1_tR):(D_LEDx_row0_tR))	// top center
#define D_LEDx_tL(row) ((row)?(D_LEDx_row1_tL):(D_LEDx_row0_tL))	// top left
#define D_LEDx_tC(row) ((row)?(D_LEDx_row1_tC):(D_LEDx_row0_tC))	// top right


void d_LEDs_write_digit(U8 row, U8 pos, U8 val, U8 dot_state);
void d_LEDx_write_dot(U8 row, U8 pos, U32 on);
void d_LEDx_write_LEDs(U16 led_mask, U32 on);



#define D_LEDx_VOLT		0	// row 0, upper row
#define D_LEDx_AMPR		1	// row 1, lower row

#define D_HIGHLIGHT_NONE	0
#define D_HIGHLIGHT_VOLT	0x10
#define D_HIGHLIGHT_AMPR	0x11

void d_set_g_HighlightRow(U8 row);


void d_write_number_b10(U8 row, U32 val);	// Base 10, aka decimal
void d_write_number_b16(U8 row, U32 val);	// Base 16, aka hexadecimal


#define D_ADC_VOLT		0
#define D_ADC_AMPR		1
#define D_ADC_TEMP_PSU	2
#define D_ADC_TEMP_MCU	3

U32 d_get_ADC(U8 uADC);


//void d_DAC_write(U32 val);
void d_DAC_74595(U32 volt, U32 amp);

void d_set_volt_ampr(void);

U32 d_SI_to_DAC(U8 uDACidx, U32 uSI_val);
U32 d_ADC_to_SI(U8 uADCidx, U32 uADC_val);


#define D_OUT_REL3	(1<<6)	// P3.6
#define D_OUT_REL2	(1<<7)	// P3.7

#define D_RELAY_MASK	(D_OUT_REL3 | D_OUT_REL2)

//void d_set_out(U32 mask, U32 on);

void d_set_relays(U32 on);
U32 d_get_relays(void);


#define D_IN_OCP	0x01

U32 d_get_in(U32 input); // D_GPIO(D_ANL_OCP)

													//	0	1		2
#define D_KEY_M1		0x001	//1	D_PIN_KEY_READ_1	//	M1	LOCK	NC
#define D_KEY_M4		0x002	//4	D_PIN_KEY_READ_4	//	M4	ON		RIGHT
#define D_KEY_M2		0x004	//2	D_PIN_KEY_READ_2	//	M2	OCP		LEFT
#define D_KEY_M3		0x008	//3	D_PIN_KEY_READ_3	//	M3	OVP		VOLT_AMP
#define D_KEY_LCK		0x010	//1	D_PIN_KEY_READ_1	//	M1	LOCK	NC
#define D_KEY_OUT		0x020	//4	D_PIN_KEY_READ_4	//	M4	ON		RIGHT
#define D_KEY_OCP		0x040	//2	D_PIN_KEY_READ_2	//	M2	OCP		LEFT
#define D_KEY_OVP		0x080	//3	D_PIN_KEY_READ_3	//	M3	OVP		VOLT_AMP
#define D_KEY_NC		0x100	//1	D_PIN_KEY_READ_1	//	M1	LOCK	NC
#define D_KEY_LEFT		0x200	//4	D_PIN_KEY_READ_4	//	M4	ON		LEFT
#define D_KEY_RIGHT		0x400	//2	D_PIN_KEY_READ_2	//	M2	OCP		RIGHT
#define D_KEY_VOLT_AMP	0x800	//3	D_PIN_KEY_READ_3	//	M3	OVP		VOLT_AMP

U32 d_get_keys(void);

S32 d_get_wheel(void);

#define D_OUTPUT_TOGGLE		2
U8 d_set_output(U8 opt);




#endif

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
//
//	You should have received a copy of the GNU General Public License
//	along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//*****************************************************************************

// {} | []

#include "M051Series.h"

#include "d_hw.h"


U16 g_eeprom_MEM_ADDR = 0;
U8 g_eeprom_DATA = 0;

static U32 g_uLastKeys = 0;

U32 d_was_key_pressed(U32 uCur, U32 uLast, U32 uKeyMask)
{
	U32 uPressed = (~uLast) & uCur & uKeyMask;

	return uPressed;
}

U32 uOutputEnabled = 0;

//U32 volt = 0;
//U32 ampr = 0;

U32 g_ADC_volt = 0;
U32 g_ADC_ampr = 0;

extern U32 g_OutputValues[2];


S32 g_max[2]  = { 31000, 51000 };	// 31.00 Volt, 5.100 Ampere

#define D_MAX_WHEEL_PRECI	2
#define D_VAR_WHEEL_PRECI	1

#define D_ARR_WHEEL_PRECI	(D_MAX_WHEEL_PRECI+1)

S32 g_step[2][D_ARR_WHEEL_PRECI] = {
	1000, 	// 01.00 Volt
	 100, 	// 00.10 Volt
	  10, 	// 00.01 Volt
	1000,	// 0.100 Ampere
	 100,	// 0.010 Ampere
	  10	// 0.001 Ampere
};


U8 g_wheel_precision = D_VAR_WHEEL_PRECI;	// 0-3
S32 g_last_wheel = 0;

S32 d_get_new_value( U8 uIDX, S32 sVal, S32 inc )
{
	S32 sNewVal = sVal;

	S32 i = inc;
	S32 k = (inc < 0) ? -1 : +1;

	while( i != 0 )
	{
		sNewVal += k * g_step[uIDX][g_wheel_precision];

		i = (i - k);
	}

	sNewVal = (sNewVal < 0) ? 0 : sNewVal;
	sNewVal = (sNewVal > g_max[uIDX]) ? g_max[uIDX] : sNewVal;

	return sNewVal;
}

void d_i2c_read(void)
{
	g_eeprom_MEM_ADDR &= 0x1FFF;	// 64k EEPROM

	U8 eeprom_ADDR = 0x50 << 1;	// 0xA0 ???

	//send i2c start
	I2C_Trigger(I2C0, 1, 0, 1, 0);	//set start
	//I2C_Trig(I2C0);
	I2C_WAIT_READY(I2C0);
	//I2C_ClearIntFlag(I2C0);

	//send writer command
	I2C_SetData(I2C0, eeprom_ADDR);	// EEPROM addr
	I2C_Trigger(I2C0, 0, 0, 1, 1);
		//I2C_Trig(I2C0);
	I2C_WAIT_READY(I2C0);
	//I2C_ClearIntFlag(I2C0);

	//send address high
	I2C_SetData(I2C0, (g_eeprom_MEM_ADDR>>8)&0xFF);
	I2C_Trigger(I2C0, 0, 0, 1, 1);
	//I2C_Trig(I2C0);
	I2C_WAIT_READY(I2C0);
	//I2C_ClearIntFlag(I2C0);

	//send address low
	I2C_SetData(I2C0, g_eeprom_MEM_ADDR&0xFF);
	I2C_Trigger(I2C0, 0, 0, 1, 1);
	//I2C_Trig(I2C0);
	I2C_WAIT_READY(I2C0);
	//I2C_ClearIntFlag(I2C0);


	//send start flag
	I2C_Trigger(I2C0, 1, 0, 1, 0);
	//I2C_Trig(I2C0);
	I2C_WAIT_READY(I2C0);
	//I2C_ClearIntFlag(I2C0);

	//send read command
	I2C_SetData(I2C0, eeprom_ADDR | 0x01);
	I2C_Trigger(I2C0, 0, 0, 1, 1);
	//I2C_Trig(I2C0);
	I2C_WAIT_READY(I2C0);
	//I2C_ClearIntFlag(I2C0);

	//receive data
	I2C_SetData(I2C0, 0xFF);
	I2C_Trigger(I2C0, 0, 0, 1, 1);
	//I2C_Trig(I2C0);
	I2C_WAIT_READY(I2C0);
	g_eeprom_DATA = I2C_GetData(I2C0);

	//send i2c stop
	I2C_Trigger(I2C0, 0, 1, 1, 0);
	//I2C_Trig(I2C0);

	return;


	//I2C_WAIT_READY(I2C0);
	I2C_SetData(I2C0, 0x50);
	I2C_START(I2C0);
	I2C_WAIT_READY(I2C0);

	I2C_SetData(I2C0, 0x00);
	I2C_START(I2C0);
	I2C_WAIT_READY(I2C0);

	I2C_SetData(I2C0, g_eeprom_MEM_ADDR);
	I2C_START(I2C0);
	I2C_WAIT_READY(I2C0);

	I2C_START(I2C0);
	I2C_WAIT_READY(I2C0);
	g_eeprom_DATA = I2C_GetData(I2C0);
}

U32 g_bOCP_CC_CV = 0;

void d_ui_task(void)
{
	static U32 q = 0;
	U8 bUpdateVoltAmprADC = 0;

	U32 uCurKeys = d_get_keys();
	U32 uPressed = d_was_key_pressed(uCurKeys, g_uLastKeys, ~0);


#if 0
	d_LEDx_write_dot(D_LEDx_VOLT, 0, uCurKeys&D_KEY_LCK);
	d_LEDx_write_dot(D_LEDx_VOLT, 1, uCurKeys&D_KEY_OCP);
	d_LEDx_write_dot(D_LEDx_VOLT, 2, uCurKeys&D_KEY_OVP);
	d_LEDx_write_dot(D_LEDx_VOLT, 3, uCurKeys&D_KEY_OUT);

	d_LEDx_write_dot(D_LEDx_AMPR, 0, uCurKeys&D_KEY_NC);
	d_LEDx_write_dot(D_LEDx_AMPR, 1, uCurKeys&D_KEY_LEFT);
	d_LEDx_write_dot(D_LEDx_AMPR, 2, uCurKeys&D_KEY_VOLT_AMP);
	d_LEDx_write_dot(D_LEDx_AMPR, 3, uCurKeys&D_KEY_RIGHT);

	d_LEDx_write_LEDs(D_LEDx_M1, uCurKeys&D_KEY_M1);
	d_LEDx_write_LEDs(D_LEDx_M2, uCurKeys&D_KEY_M2);
	d_LEDx_write_LEDs(D_LEDx_M3, uCurKeys&D_KEY_M3);
	d_LEDx_write_LEDs(D_LEDx_M4, uCurKeys&D_KEY_M4);
#endif

	if( uPressed & D_KEY_OUT )
	{
		uOutputEnabled = d_set_output(D_OUTPUT_TOGGLE);

		d_LEDx_write_LEDs(D_LEDx_OUT, uOutputEnabled);

		if(uOutputEnabled)
		{
			g_bOCP_CC_CV = -1;
		}
		else
		{
			d_LEDx_write_LEDs(D_LEDx_CC, 0);
			d_LEDx_write_LEDs(D_LEDx_CV, 0);
		}

	}

	U32 bOCP_CC_CV = d_get_in(D_IN_OCP);

	if(bOCP_CC_CV != g_bOCP_CC_CV)
	{
		//bUpdateDisplay = 1;
		//d_LEDx_write_LEDs(D_LEDx_OCP, bOCP_CC_CV);

		if(uOutputEnabled)
		{
			d_LEDx_write_LEDs(D_LEDx_CC,  bOCP_CC_CV);
			d_LEDx_write_LEDs(D_LEDx_CV, !bOCP_CC_CV);
		}

		g_bOCP_CC_CV = bOCP_CC_CV;
	}

	static U8 editVoltAmpr = D_HIGHLIGHT_NONE;

	if( uPressed & D_KEY_VOLT_AMP )
	{
		bUpdateVoltAmprADC = 1;

		g_last_wheel = d_get_wheel();

		switch(editVoltAmpr)
		{
		case D_HIGHLIGHT_NONE:	editVoltAmpr = D_HIGHLIGHT_VOLT;	g_wheel_precision = D_VAR_WHEEL_PRECI;	break;
		case D_HIGHLIGHT_VOLT:	editVoltAmpr = D_HIGHLIGHT_AMPR;	g_wheel_precision = 0;	break;
		default:
		case D_HIGHLIGHT_AMPR:	editVoltAmpr = D_HIGHLIGHT_NONE;	break;
		}

#if 1
		d_LEDx_write_LEDs(D_LEDx_M5, editVoltAmpr);
#endif

		d_set_g_HighlightRow(editVoltAmpr);
	}

	//static uHi = D_HIGHLIGHT_NONE;

	if( uPressed & D_KEY_LEFT || uPressed & D_KEY_RIGHT )
	{
		//U32 uRelay = d_get_relays();

		if( uPressed & D_KEY_LEFT )
		{
			//uRelay = (uRelay ^ D_OUT_REL2);

			//uHi = (uHi == D_HIGHLIGHT_VOLT) ? D_HIGHLIGHT_NONE : D_HIGHLIGHT_VOLT;

			g_eeprom_MEM_ADDR --;

			g_wheel_precision = (g_wheel_precision > 0) ? (g_wheel_precision-1) : 0;
		}

		if( uPressed & D_KEY_RIGHT )
		{
			//uRelay = (uRelay ^ D_OUT_REL3);

			//uHi = (uHi == D_HIGHLIGHT_AMPR) ? D_HIGHLIGHT_NONE : D_HIGHLIGHT_AMPR;

			g_eeprom_MEM_ADDR ++;

			g_wheel_precision = (g_wheel_precision < D_MAX_WHEEL_PRECI) ? (g_wheel_precision + 1) : (D_MAX_WHEEL_PRECI);
		}

		#if 0
			d_set_relays(uRelay);

			d_LEDx_write_LEDs(D_LEDx_CC, uRelay & D_OUT_REL2);
			d_LEDx_write_LEDs(D_LEDx_CV, uRelay & D_OUT_REL3);
		#endif

		//d_set_g_HighlightRow(uHi);
			//g_eeprom_MEM_ADDR &= 0x1FFF;	// 64k EEPROM
			d_i2c_read();
	}

	// switch relays at
	//  7V	 8V
	// 14V	15V
	// 20V	21V

	S32 new_wheel = d_get_wheel();

	if( g_last_wheel !=  new_wheel)
	{
		const S32 inc = (new_wheel - g_last_wheel) / 4;

		if( editVoltAmpr != D_HIGHLIGHT_NONE )
		{
			bUpdateVoltAmprADC = 1;

			U32 uIDX = (editVoltAmpr == D_HIGHLIGHT_VOLT) ? 0 : 1;

			S32 sVal = g_OutputValues[uIDX];

			//uVal = (inc < dec) ? (uVal + inc*step) : (uVal - dec*step);
			sVal = d_get_new_value(uIDX, sVal, inc);

			g_OutputValues[uIDX] = sVal;

			//g_OutputValues[0] = volt;
			//g_OutputValues[1] = ampr;

		}
		else
		{
			g_eeprom_MEM_ADDR += inc;	// 64k EEPROM
			d_i2c_read();
		}

		g_last_wheel = new_wheel;
	}

	if( (q & (0x200-1)) == 0)
	{
		g_ADC_volt = d_get_ADC_SI(D_ADC_VOLT);
		g_ADC_ampr = d_get_ADC_SI(D_ADC_AMPR);

		bUpdateVoltAmprADC = 1;
	}

	if(bUpdateVoltAmprADC)
	{
		const U32 uDisplayVolt = (editVoltAmpr == D_HIGHLIGHT_VOLT || uOutputEnabled == FALSE) ? g_OutputValues[0] : g_ADC_volt;
		const U32 uDisplayAmpr = (editVoltAmpr == D_HIGHLIGHT_AMPR || uOutputEnabled == FALSE) ? g_OutputValues[1] : g_ADC_ampr;

		d_write_number_b10(D_LEDx_VOLT, uDisplayVolt );
		d_LEDx_write_dot(D_LEDx_VOLT, 2, 1);

		d_write_number_b10(D_LEDx_AMPR, uDisplayAmpr );
		d_LEDx_write_dot(D_LEDx_AMPR, 3, 1);

		if(uOutputEnabled)
			d_set_volt_ampr();
	}



#if 1
	extern U32 g_debug_relay;

	d_LEDx_write_LEDs(D_LEDx_M1, g_debug_relay & 0x01);
	d_LEDx_write_LEDs(D_LEDx_M2, g_debug_relay & 0x02);

#if 0
	if(uOutputEnabled && bUpdateVoltAmprADC && editVoltAmpr != D_HIGHLIGHT_AMPR)
    {
		d_write_number_b16(D_LEDx_AMPR, g_debug_relay );
		d_LEDx_write_dot(D_LEDx_AMPR, 3, 0);
    }
#endif
#endif

#if 0
	if(uOutputEnabled && bUpdateVoltAmprADC && editVoltAmpr != D_HIGHLIGHT_AMPR)
    {
		const U32 uADC_volt = d_get_ADC(D_ADC_VOLT);
		//d_write_number_b16(D_LEDx_VOLT, uADC_volt & 0xFFFF );
		//d_LEDx_write_dot(D_LEDx_VOLT, 2, 0);

		d_write_number_b16(D_LEDx_AMPR, uADC_volt & 0xFFFF );
		d_LEDx_write_dot(D_LEDx_AMPR, 3, 0);

		//const U32 uADC_ampr = d_get_ADC(D_ADC_AMPR);
		//d_write_number_b16(D_LEDx_AMPR, uADC_ampr & 0xFFFF );
    }
#endif


#if 0
	d_write_number_b16(D_LEDx_VOLT, g_eeprom_MEM_ADDR);
    d_write_number_b16(D_LEDx_AMPR, g_eeprom_DATA);
#endif

#if 0
	if(editVoltAmpr != D_HIGHLIGHT_NONE)
	{
		volt = g_OutputValues[0];
		ampr = g_OutputValues[1];

		U32 bIsVolt = (editVoltAmpr == D_HIGHLIGHT_VOLT);
		U32 val = bIsVolt ? volt : ampr;
		U32 LSB = bIsVolt ? 0x4 : 0x8;

		U32 min = bIsVolt ? 0x027c : 0x02c0;
		U32 max = bIsVolt ? 0xd690 : 0xc1e0;

		U32 pval = 0;
		U32 nval = 0;

		pval += ( uPressed & D_KEY_M1 ) ? 0x1000 : 0;
		pval += ( uPressed & D_KEY_M2 ) ? 0x0100 : 0;
		pval += ( uPressed & D_KEY_M3 ) ? 0x0010 : 0;
		pval += ( uPressed & D_KEY_M4 ) ? LSB : 0;

		nval += ( uPressed & D_KEY_LCK ) ? 0x1000 : 0;
		nval += ( uPressed & D_KEY_OCP ) ? 0x0100 : 0;
		nval += ( uPressed & D_KEY_OVP ) ? 0x0010 : 0;
		nval += ( uPressed & D_KEY_OUT ) ? LSB : 0;

		val -= nval;

		val = (val > 0xFFFF || val < min) ? min : val;

		val += pval;

		val = (val > max) ? max : val;

		val &= ~(LSB-1);

		volt = bIsVolt ? val : volt;
		ampr = bIsVolt ? ampr : val;

		g_OutputValues[0] = volt;
		g_OutputValues[1] = ampr;
	}



	d_DAC_74595(volt, ampr);


	d_DAC_74595(volt, ampr);

#if 0
	d_write_number_b16(D_LEDx_VOLT, volt);
    d_write_number_b16(D_LEDx_AMPR, ampr);
#elif 0
    const U32 wheel = d_get_wheel();
	d_write_number_b16(D_LEDx_VOLT, (wheel>>16) );
    d_write_number_b16(D_LEDx_AMPR, wheel & 0xFFFF );
#elif 1
	const U32 uADC_volt = d_get_ADC_SI(D_ADC_VOLT);
	const U32 uADC_ampr = d_get_ADC_SI(D_ADC_AMPR);
	d_write_number_b10(D_LEDx_VOLT, uADC_volt );
	d_write_number_b10(D_LEDx_AMPR, uADC_ampr );

	d_LEDx_write_dot(D_LEDx_VOLT, 2, 1);
	d_LEDx_write_dot(D_LEDx_AMPR, 3, 1);
#elif 1
    if( (u & 0x3) == 0 )
    {
		const U32 uADC_volt = d_get_ADC(D_ADC_VOLT);
		const U32 uADC_ampr = d_get_ADC(D_ADC_AMPR);
		d_write_number_b16(D_LEDx_VOLT, uADC_volt & 0xFFFF );
		d_write_number_b16(D_LEDx_AMPR, uADC_ampr & 0xFFFF );
    }
#elif 1
    // "Normal" 0x0244-0x0260-0x290	Without cable: 0x0085
    const U32 uADC_psu = d_get_ADC(D_ADC_TEMP_PSU);

    // "Normal" 0x0013
    const U32 uADC_mcu = d_get_ADC(D_ADC_TEMP_MCU);

    d_write_number_b16(D_LEDx_AMPR, uADC_mcu & 0xFFFF );
    d_write_number_b16(D_LEDx_VOLT, uADC_psu & 0xFFFF );
#endif

#endif

    g_uLastKeys = uCurKeys;

	q++;

}

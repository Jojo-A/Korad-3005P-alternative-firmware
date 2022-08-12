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

//#include <stdio.h>

#include "d_def.h"
#include "d_hw.h"

// {}
// []

extern void d_i2c_read(void);

int main(void)
{
	d_hw_init();

	//d_DAC_74595(0, 0);

	d_i2c_read();

    //d_write_number_b16(D_LEDx_VOLT, 0xCAFE);
    //d_write_number_b16(D_LEDx_AMPR, 0xFEFE);

	//U32 u = 0;
    //U32 c = 0x8000 ;
    //int on = 0;
    //U8 q = 0;

    while(1)
    {
    	//u++;

    	//if( (u & (0x8000-1)) == 0)
    	{
    		//uLED = d_blinkenlights(uLED);
    		//uLED_buf[q] = uLED;
    	}

    	//if( (u & (0x80-1)) == 0)
    	{
    		d_hw_task();
    		d_ui_task();
    	}

		//if( (u & (0x200000-1)) == 0)
		{
			// low speed
    	}


    	//if( (u & (c-1)) == 0)
    	{
    		//on = !on;
    		//P43 = on;
    	}

    }


}
//! @}
//! @}

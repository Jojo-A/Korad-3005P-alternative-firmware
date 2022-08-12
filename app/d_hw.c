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

#include "M051Series.h"

#include "d_def.h"

#include "d_hw.h"

#define D_CLK_50MHZ           		FREQ_50MHZ


#define D_DEFINE_GPIO(port,pin)	(((port)<<8) | ((pin)&0xFF))
#define D_GET_PIN(gpio)			((gpio)&0xFF)
#define D_GET_PORT(gpio)		(((gpio)>>8)&0xFF)

#define D_NUV_PORT(port)		((GPIO_T *) (GPIO_BASE + 0x0040*(port)) )
#define D_GPIO(gpio)			GPIO_PIN_ADDR( D_GET_PORT(gpio), D_GET_PIN(gpio) )


///// IO defines //////
// outputs //
#define D_PIN_FAN		D_DEFINE_GPIO(4,3)

#define D_PIN_DISPLAY_COL0	D_DEFINE_GPIO(1,6)	// P1.6
#define D_PIN_DISPLAY_COL1	D_DEFINE_GPIO(1,5)	// P1.5
#define D_PIN_DISPLAY_COL2	D_DEFINE_GPIO(1,3)	// P1.3
#define D_PIN_DISPLAY_COL3	D_DEFINE_GPIO(1,4)	// P1.4

#define D_PIN_DISPLAY_COLx	D_DEFINE_GPIO(4,2)	// Look at all those points of light! Oh they are so pretty!

#define D_PIN_DISPLAY_STORE	D_DEFINE_GPIO(0,1)	// STCP			// latch
#define D_PIN_DISPLAY_SHIFT	D_DEFINE_GPIO(0,2)	// SHCP			// clock
#define D_PIN_DISPLAY_DATA	D_DEFINE_GPIO(0,0)	// SD and DS	// data

#define D_PIN_DAC_SHIFT	D_DEFINE_GPIO(4,1)	// STCP
#define D_PIN_DAC_STORE	D_DEFINE_GPIO(0,4)	// SHCP

#define D_PIN_DAC_DATA	D_DEFINE_GPIO(0,5)	// SD and DS

#define D_DAC_BITS	24


#define D_PIN_DAC_VRE1	D_DEFINE_GPIO(4,5)	// pin29
#define D_PIN_DAC_VRE2	D_DEFINE_GPIO(4,4)	// pin28
#define D_PIN_DAC_VRE3	D_DEFINE_GPIO(4,0)	// pin24

#define D_PIN_REL1		D_DEFINE_GPIO(2,0)	//	missing "output relay"
#define D_PIN_REL2		D_DEFINE_GPIO(3,7)	//	windings-relay
#define D_PIN_REL3		D_DEFINE_GPIO(3,6)	//	windings-relay


// REL1	REL2	Volt
//

#define D_PIN_BUZZER	D_DEFINE_GPIO(0,3)

////////////////
// Analog INs //




////////////
// inputs //
#define D_PIN_IN_OCP	D_DEFINE_GPIO(0,6)	// Overcurrent protection


#define D_PIN_KEY_L		D_DEFINE_GPIO(3,2)	// Rotary wheel
#define D_PIN_KEY_R		D_DEFINE_GPIO(3,3)	// Rotary wheel

//#define D_	D_DEFINE_GPIO(,)	//



////////////////
// Key matrix //

#define D_PIN_GPIO_25	D_DEFINE_GPIO(2,5)	//KEY_OUT_2
#define D_PIN_GPIO_26	D_DEFINE_GPIO(2,6)	//KEY_OUT_3
#define D_PIN_GPIO_27	D_DEFINE_GPIO(2,7)	//KEY_OUT_4
#define D_PIN_GPIO_22	D_DEFINE_GPIO(2,2)	//KEY_IN_1
#define D_PIN_GPIO_23	D_DEFINE_GPIO(2,3)	//KEY_IN_2
#define D_PIN_GPIO_24	D_DEFINE_GPIO(2,4)	//KEY_IN_3
#define D_PIN_GPIO_21	D_DEFINE_GPIO(2,1)	//

										//	read1	read2	read3	read4
#define D_PIN_KEY_SCAN_0	D_PIN_GPIO_21	//	M1		M2		M3		M4
#define D_PIN_KEY_SCAN_1	D_PIN_GPIO_22	//	LOCK	OCP 	OVP 	ON
#define D_PIN_KEY_SCAN_2	D_PIN_GPIO_23	//	-		RIGHT	VOLT	LEFT

#define D_PIN_KEY_READ_1	D_PIN_GPIO_24	//	M1	LOCK	NC
#define D_PIN_KEY_READ_2	D_PIN_GPIO_26	//	M2	OCP		LEFT
#define D_PIN_KEY_READ_3	D_PIN_GPIO_27	//	M3	OVP		VOLT_AMP
#define D_PIN_KEY_READ_4	D_PIN_GPIO_25	//	M4	ON		RIGHT


U16 g_uRelayState = 0;

U32 g_set_relay = 0;

void d_set_relays(U32 on);

#define DAC_CALIB_VOLT_00	0
#define DAC_CALIB_AMPR_0	1
#define DAC_CALIB_VOLT_30	2
#define DAC_CALIB_AMPR_5	3
#define ADC_CALIB_VOLT_00	4
#define ADC_CALIB_AMPR_0	5
#define ADC_CALIB_VOLT_31	6
#define ADC_CALIB_AMPR_5	7
#define ADC_CALIB_NUM		8

U16	g_Calibration[ADC_CALIB_NUM];

U32 g_OutputValues[2] = { 3300, 1000 };	// 3.3 Volt, 0.100 Ampere

U8 g_OutputEnable = 0;

U8 g_HighlightRow = 0;


static U16 d_LEDx_output_col(U8 col, U8 highlight);


void _d_hw_init(void)
{
    //return;
	/*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
#if 1
    CLK_EnableXtalRC(CLK_PWRCON_OSC10K_EN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_OSC10K_STB_Msk);
#endif

#if 1
	/* Enable Internal RC 22.1184MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_OSC22M_EN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_OSC22M_STB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HIRC, CLK_CLKDIV_HCLK(1));

    /* Enable external XTAL 12MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_XTL12M_EN_Msk);

    /* Waiting for external XTAL clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL12M_STB_Msk);

    /* Set core clock as 50 MHz from PLL */
    CLK_SetCoreClock(D_CLK_50MHZ);
#endif

    /*---------------------------------------------------------------------------------------------------------*/
    /* Usart                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable UART module clock */
    //CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source */
    //CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_PLL, CLK_CLKDIV_UART(1));

    /* Set P3 multi-function pins for UART0 RXD and TXD */
    //SYS->P3_MFP &= ~(SYS_MFP_P30_Msk | SYS_MFP_P31_Msk);
    //SYS->P3_MFP |= (SYS_MFP_P30_RXD0 | SYS_MFP_P31_TXD0);



    /*---------------------------------------------------------------------------------------------------------*/
    /* ADC                                                                                      */
    /*---------------------------------------------------------------------------------------------------------*/

    CLK_EnableModuleClock(ADC_MODULE);

     // 22MHz / 75 = 293kHz clock for ADC module
     // -> approx 137µs conversion time.
    //CLK_SetModuleClock(ADC_MODULE, CLK_CLKSEL1_ADC_S_IRC22M, CLK_CLKDIV_ADC(50));


    // 50MHz / 500,000 = 100Hz clock for ADC module
    // -> approx 400µs conversion time.
    //CLK_SetModuleClock(ADC_MODULE, CLK_CLKSEL1_ADC_S_HCLK, CLK_CLKDIV_ADC(500 000));
    CLK_SetModuleClock(ADC_MODULE, CLK_CLKSEL1_WDT_S_LIRC, CLK_CLKDIV_ADC(0xFF));


    // Enable channel 0, 1, 2 and 7
    // P1.0, P1.1, P1.2
    ADC_Open(ADC, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_CONTINUOUS, (1<<0 | 1<<1 | 1<<2 | 1<<7) );
    //ADC_Open(ADC, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_CONTINUOUS, (1<<2) );

    ADC_POWER_ON(ADC);
    /* Set ADC divisor */
    //_ADC_SET_CLK_DIV(7);

    ADC_SET_INPUT_CHANNEL(ADC, (1<<0 | 1<<1 | 1<<2 | 1<<7) );
    //ADC_SET_INPUT_CHANNEL(ADC, (1<<2) );

    ADC_CONFIG_CH7( ADC, ADC_ADCHER_PRESEL_INT_TEMPERATURE_SENSOR );

    // Enable ADC ADC_IF interrupt
    ADC_EnableInt(ADC, ADC_ADF_INT);
    NVIC_EnableIRQ(ADC_IRQn);

	/* Disable digital input path to avoid leakage current. */
	GPIO_DISABLE_DIGITAL_PATH(P1, (1<<0) | (1<<1) | (1<<2) );	// P1.0 // P1.1 // P1.2

	/* Configure the analog input pins */
	SYS->P1_MFP = SYS_MFP_P10_AIN0 | SYS_MFP_P11_AIN1 | SYS_MFP_P12_AIN2 ;

    ADC_START_CONV( ADC );

//	SYS->ALT_MFP1 = 0;
//	SYS->P1_ALT

    /*---------------------------------------------------------------------------------------------------------*/
    /* I2C                                                                                      */
    /*---------------------------------------------------------------------------------------------------------*/

    CLK_SetModuleClock(I2C0_MODULE, 0, 0);
    CLK_EnableModuleClock(I2C0_MODULE);

    /* Set multi function pin for I2C0 */
	SYS->P3_MFP = SYS_MFP_P34_SDA0 | SYS_MFP_P35_SCL0 ;

	//I2C_SetSlaveAddr(I2C0, 0, 0x50, I2C_GCMODE_DISABLE);
    //I2C_SetSlaveAddrMask(I2C0, 0, 0x01);

    //I2C_EnableInt(I2C0);
    //NVIC_EnableIRQ(I2C0_IRQn);
	//I2C_Init
    I2C_Open(I2C0, 100000);
}

void d_GPIO_configure(void)
{
	// for EINTx_IRQHandler()
#if 0
	GPIO_EnableInt( D_NUV_PORT(D_GET_PORT(D_PIN_KEY_L)), D_GET_PIN(D_PIN_KEY_L), GPIO_INT_BOTH_EDGE);
	GPIO_EnableInt( D_NUV_PORT(D_GET_PORT(D_PIN_KEY_R)), D_GET_PIN(D_PIN_KEY_R), GPIO_INT_BOTH_EDGE);
#else
	GPIO_EnableInt( P3, 2, GPIO_INT_BOTH_EDGE);
	GPIO_EnableInt( P3, 3, GPIO_INT_BOTH_EDGE);
#endif

	NVIC_EnableIRQ(EINT0_IRQn);
	NVIC_EnableIRQ(EINT1_IRQn);

#if 0
	// outputs //
	d_GPIO_enable_out(D_PIN_DISPLAY_COL0, 0);
    d_GPIO_enable_out(D_PIN_DISPLAY_COL1, 0);
    d_GPIO_enable_out(D_PIN_DISPLAY_COL2, 0);
    d_GPIO_enable_out(D_PIN_DISPLAY_COL3, 0);

    d_GPIO_enable_out(D_PIN_DISPLAY_COLx, 0);

    d_GPIO_enable_out(D_PIN_DISPLAY_STORE, 0);
    d_GPIO_enable_out(D_PIN_DISPLAY_SHIFT, 0);
    d_GPIO_enable_out(D_PIN_DISPLAY_DATA, 0);
#endif

#if 1
    d_GPIO_enable_out(D_PIN_DAC_STORE, 0);
    d_GPIO_enable_out(D_PIN_DAC_SHIFT, 0);
    d_GPIO_enable_out(D_PIN_DAC_DATA, 0);
#endif


    //return;
#if 0
    d_GPIO_enable_out(D_BUZZER, 0);
#endif

    d_GPIO_enable_out(D_PIN_FAN, 0);

#if 1
	// TODO: Done: Read entire port.
	g_uRelayState = 0; // GPIO_GET_IN_DATA( D_NUV_PORT(D_GET_PORT(D_PIN_REL2)) );
	g_uRelayState &= D_RELAY_MASK;

    //d_GPIO_enable_out(D_PIN_REL1, 1);	// P2.0
    //d_GPIO_enable_out(D_PIN_REL2, g_uRelayState & D_OUT_REL2);	//P3.7
    //d_GPIO_enable_out(D_PIN_REL3, g_uRelayState & D_OUT_REL2);	//P3.6

    GPIO_DISABLE_DOUT_MASK( D_NUV_PORT(D_GET_PORT(D_PIN_REL2)), ~0 /*D_RELAY_MASK*/ );
    //GPIO_ENABLE_DOUT_MASK ( D_NUV_PORT(D_GET_PORT(D_PIN_REL2)),  D_RELAY_MASK );
#elif 0
    D_GPIO_SETMODE( D_PIN_REL1, GPIO_PMD_QUASI);
    D_GPIO_SETMODE( D_PIN_REL2, GPIO_PMD_QUASI);
    D_GPIO_SETMODE( D_PIN_REL3, GPIO_PMD_QUASI);

    D_GPIO(D_PIN_REL1) = 0;
    D_GPIO(D_PIN_REL2) = 0;
    D_GPIO(D_PIN_REL3) = 0;

#endif

#if 0
	// inputs //
	D_GPIO_SETMODE( D_PIN_KEY_L, GPIO_PMD_INPUT);
	D_GPIO_SETMODE( D_PIN_KEY_R, GPIO_PMD_INPUT);
#endif

#if 0
	//GPIO_SetMode( P0_BASE, 6, GPIO_PMD_INPUT);
	D_GPIO_SETMODE( D_PIN_IN_OCP, GPIO_PMD_INPUT);
#endif


}

#if 0
 void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART0->LCR |=0x07;
    UART0->BAUD = 0x30000066;   /* 12MHz reference clock input, for 115200 */
}
#endif

void d_set_HW_calib(void)
{
	g_Calibration[DAC_CALIB_VOLT_00	] = 0x027c;
	g_Calibration[DAC_CALIB_VOLT_30	] = 0xd690;

	g_Calibration[DAC_CALIB_AMPR_0	] = 0x02c0;
	g_Calibration[DAC_CALIB_AMPR_5	] = 0xc1e0;

	g_Calibration[ADC_CALIB_VOLT_00	] = 0x0000;	// TBD
	g_Calibration[ADC_CALIB_VOLT_31	] = 0x0DB0;	// 31V == 3504,3 == 0x0DB0

	g_Calibration[ADC_CALIB_AMPR_0	] = 0x0026;	// == 38
	g_Calibration[ADC_CALIB_AMPR_5	] = 0x0BD5;	// 4150 mA == 0x09f8 == 2552
												// 5000 mA == 3028,9 == 0x0BD5

}

void d_hw_init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    _d_hw_init();

    /* Lock protected registers */
    SYS_LockReg();

    d_GPIO_configure();

    d_set_HW_calib();

    g_OutputEnable = 0;
    d_set_volt_ampr();

    g_set_relay = 0;
    d_set_relays(g_set_relay << 6);

    return;
}

#define D_DISPLAY_COLS	5


extern U8 g_wheel_precision;

void d_set_g_HighlightRow(U8 row)
{
	g_HighlightRow = row;
}

static U8 highlight = D_HIGHLIGHT_NONE;
static U8 highlight_digit = 0;
static U16 g_LEDcounter = 0;

U8 q = (D_DISPLAY_COLS-1);


void d_display_task(void)
{
	if(g_LEDcounter == 0)
	{
		g_LEDcounter = d_LEDx_output_col(q, highlight);

		if(highlight != D_HIGHLIGHT_NONE && 2-q == g_wheel_precision)
		{
			g_LEDcounter <<= 2;	// highlight current digit
		}

		q--;

		if(q >= D_DISPLAY_COLS)
		{
			q = (D_DISPLAY_COLS-1);
			highlight = (highlight == D_HIGHLIGHT_NONE) ? g_HighlightRow : D_HIGHLIGHT_NONE;
		}
	}
	else
	{
		g_LEDcounter--;
	}

}

void d_hw_task(void)
{
	d_read_keys();

	d_display_task();
}


/*
void d_set_out(U32 mask, U32 on)
{
	if(mask & D_OUT_REL1)	D_GPIO(D_PIN_REL1) = (on & D_OUT_REL1) ? 0 : 1;
	if(mask & D_OUT_REL2)	D_GPIO(D_PIN_REL2) = (on & D_OUT_REL2) ? 0 : 1;
	if(mask & D_OUT_REL3)	D_GPIO(D_PIN_REL3) = (on & D_OUT_REL3) ? 0 : 1;
}
*/
#if 0
// ADC	// Volt
0x0090, // 01
0x0100, // 02
0x0170, // 03
0x01E1, // 04
0x024E, // 05
0x02BE, // 06
0x032E, // 07
0x039E, // 08
0x0411, // 09
0x0481, // 10
0x04F1, // 11
0x0562, // 12
0x05D2, // 13
0x063F, // 14
0x06AF, // 15
0x071F, // 16
0x078F, // 17
0x0808, // 18
0x0878, // 19
0x08E8, // 20
0x0959, // 21
0x09C9, // 22
0x0A36, // 23
0x0AA6, // 24
0x0B16, // 25
0x0B87, // 26
0x9BF9, // 27
0x0C6A, // 28
0x0CDA, // 29
0x0D46, // 30V
0x0DBB, // 31
0xFFFF, // FS
#endif

			// 00b	     01b      10b       11b
// up/max	//        09       17        21
// dn/min	//     07       14	      18

#define D_MAX_RELAY_STATES	0x03

// 00	= max ~11-12V
// 01	= max ~24-25V
// 10	= max ~29-30V


U16 g_relay_threshold_dn[D_MAX_RELAY_STATES] =
{
		0x032E, // 07
		0x063F, // 14
		0x0808, // 18
};


U16 g_relay_threshold_up[D_MAX_RELAY_STATES] =
{
		0x0411, // 09
		0x078F, // 17
		0x0959, // 21
};
U32 g_debug_relay = 0;

U32 d_get_allowed_relay(U32 uADC_volt, U32 current_relay)
{
	current_relay = g_debug_relay & 0x0F;

	U32 i = D_MAX_RELAY_STATES;

	U32 min = D_MAX_RELAY_STATES;
	U32 max = D_MAX_RELAY_STATES;

	do
	{
		i--;

		if(uADC_volt < g_relay_threshold_dn[i])	min = i;
		if(uADC_volt < g_relay_threshold_up[i])	max = i;
	}
	while(i);

	U32 uRelay = current_relay;

	if(current_relay <= max)
	{
		uRelay = max;
	}
	else if(current_relay > min)
	{
		uRelay = min;
	}

	g_debug_relay = uRelay | (min << 8) | (max << 12);

	//return (uRelay <= 0x01) ? uRelay : 0x01;
	return uRelay;
}



void d_set_relays(U32 on)
{
	if(on & ~D_RELAY_MASK)
	{
		__BKPT(23);
		//return;
	}

	g_uRelayState = (on & D_RELAY_MASK);

    GPIO_ENABLE_DOUT_MASK ( D_NUV_PORT(D_GET_PORT(D_PIN_REL2)),  D_RELAY_MASK );

	GPIO_SET_OUT_DATA( D_NUV_PORT(D_GET_PORT(D_PIN_REL2)), g_uRelayState );

	GPIO_DISABLE_DOUT_MASK( D_NUV_PORT(D_GET_PORT(D_PIN_REL2)), ~D_RELAY_MASK );

	if( g_uRelayState == 0)
	{
		GPIO_SetMode( D_NUV_PORT(D_GET_PORT(D_PIN_REL2)), (1 << D_GET_PIN(D_PIN_REL2) ) | (1 << D_GET_PIN(D_PIN_REL3) ), GPIO_PMD_QUASI);
	}
	else
	{
		GPIO_SetMode( D_NUV_PORT(D_GET_PORT(D_PIN_REL2)), (1 << D_GET_PIN(D_PIN_REL2) ) | (1 << D_GET_PIN(D_PIN_REL3) ), GPIO_PMD_OUTPUT);
	}


}

U32 d_get_relays(void)
{
	return g_uRelayState;
}

U32 d_get_in(U32 input)
{
	U32 uRet = 0;

	uRet |= ( (input & D_IN_OCP) && D_GPIO(D_PIN_IN_OCP) ) ? D_IN_OCP : 0;

	return uRet;
}

static U32 uKeyBuffer = 0;

U32 d_get_keys(void)
{
	return uKeyBuffer;
}





#define D_GPIO_SETMODE(gpio,mode)	GPIO_SetMode( D_NUV_PORT(D_GET_PORT(gpio)), 0x01 << D_GET_PIN(gpio), mode)

#if 0
void GPIO_SetMode(GPIO_T *port, uint32_t u32PinMask, uint32_t u32Mode)
{
    uint32_t i;

    for(i = 0; i < GPIO_PIN_MAX; i++)
    {
        if(u32PinMask & (1 << i))
        {
            port->PMD = (port->PMD & ~(0x3 << (i << 1))) | (u32Mode << (i << 1));
        }
    }
}
#else
void d_GPIO_SetMode(U16 gpio, uint32_t u32Mode)
{
	U8 pin = D_GET_PIN(gpio);
	GPIO_T *port = D_NUV_PORT(D_GET_PORT(gpio));
	uint32_t u32PinMask = 0x01 << pin;
	uint32_t i;

     port->PMD = (port->PMD & ~(0x3 << (pin << 1))) | (u32Mode << (pin << 1));
}
#endif

void d_GPIO_enable_out(U16 dGPIO, U8 val )
{
	//D_GPIO_SETMODE( dGPIO, GPIO_PMD_OUTPUT);
	GPIO_SetMode( D_NUV_PORT(D_GET_PORT(dGPIO)), 0x01 << D_GET_PIN(dGPIO), GPIO_PMD_OUTPUT);

    D_GPIO(dGPIO) = val ? 1 : 0;
}




static inline
void d_74595_store(U16 gpio_store)
{
	D_GPIO(gpio_store) = 1;
	D_GPIO(gpio_store) = 0;
}


static inline
void d_74595_shift_LSB_first(U16 gpio_shift, U16 gpio_data, U32 data, U8 bits)
{
	for(U8 i = 0; i < bits;)
	{
		D_GPIO(gpio_data) = data & 0x0001;

		i++;

		D_GPIO(gpio_shift) = 1;

		data = data >> 1;

		D_GPIO(gpio_shift) = 0;
	}
}

//U32 waitstate = 0;

static inline
void d_74595_shift_MSB_first(U16 gpio_shift, U16 gpio_data, U32 data, U8 bits)
{
	for(U32 mask = 1<<(bits-1); mask > 0;)
	{
		D_GPIO(gpio_data) = (data & mask) ? 1 : 0;

		D_GPIO(gpio_shift) = 1;

		mask = mask >> 1;

		D_GPIO(gpio_shift) = 0;
	}
}

#if 0
#define d_74595_shift d_74595_shift_LSB_first
#else
#define d_74595_shift d_74595_shift_MSB_first
#endif



void d_DAC_74595(U32 volt, U32 ampr)
{
#if 1
	for(U32 mask = 1 << (((D_DAC_BITS/2)-1+4) ); mask > (1<<3);)
	{
		D_GPIO(D_PIN_DAC_DATA) = (ampr & mask) ? 1 : 0;

		D_GPIO(D_PIN_DAC_SHIFT) = 1;

		D_GPIO(D_PIN_DAC_SHIFT) = 0;

		D_GPIO(D_PIN_DAC_DATA) = (volt & mask) ? 1 : 0;

		D_GPIO(D_PIN_DAC_SHIFT) = 1;

		mask = mask >> 1;

		D_GPIO(D_PIN_DAC_SHIFT) = 0;
	}

	//d_74595_store(D_PIN_DAC_STORE);
	D_GPIO(D_PIN_DAC_STORE) = 1;

	D_GPIO(D_PIN_DAC_VRE2) = (volt & 0x008) ? 1 : 0; // pin24
	D_GPIO(D_PIN_DAC_VRE3) = (volt & 0x004) ? 1 : 0; // pin28

	D_GPIO(D_PIN_DAC_VRE1) = (ampr & 0x008) ? 1 : 0; // pin29

	D_GPIO(D_PIN_DAC_STORE) = 0;

	//d_LEDx_write_LEDs(D_LEDx_CC, (volt & 0x008));
	//d_LEDx_write_LEDs(D_LEDx_CV, (volt & 0x004));

	//d_LEDx_write_LEDs(D_LEDx_OUT,(ampr & 0x008));
#endif
}

void d_set_volt_ampr(void)
{
	U16 volt = d_SI_to_DAC( 0, g_OutputEnable ? g_OutputValues[0] : 0 );
	U16 ampr = d_SI_to_DAC( 1, g_OutputEnable ? g_OutputValues[1] : 0 );

	d_DAC_74595(volt, ampr);
}



#define D_LED_74595(data)	do{d_74595_shift(D_PIN_DISPLAY_SHIFT, D_PIN_DISPLAY_DATA, data, 16); d_74595_store(D_PIN_DISPLAY_STORE); }while(0)
#define D_LED_74595_nolatch(data)	do{d_74595_shift(D_PIN_DISPLAY_SHIFT, D_PIN_DISPLAY_DATA, data, 16); }while(0)
#define D_DAC_74595(data)	do{d_74595_shift(D_PIN_DAC_SHIFT, D_PIN_DAC_DATA, data, D_DAC_BITS); d_74595_store(D_PIN_DAC_STORE); }while(0)
#define D_DAC_74595_nolatch(data)	do{d_74595_shift(D_PIN_DAC_SHIFT, D_PIN_DAC_DATA, data, D_DAC_BITS); }while(0)

void d_DAC_write(U32 val)
{
	D_DAC_74595(val);
}



//		next	prev
// 00	01		10		0
// 01	11		00		1
// 11	10		01		3
// 10	00		11		2

//		next	prev
// 00	01		10		0
// 01	11		00		1
// 10	00		11		2
// 11	10		01		3


const U8 enc_table[4] =
{
	0x1,
	0x3,
	0x0,
	0x2
};

U8 OldEnc = 0x3;

volatile S32 wheel_counter = 0;

//U32 int_counter = 0;
U32 missed = 0;

S32 d_get_wheel(void)
{
	//return int_counter;
	//return missed;
	return wheel_counter & ~0x03;
}

void  EINTx_IRQHandler(void)
{
	//int_counter++;

	// TODO: directly read port
	const U8 NewEnc = (GPIO_GET_IN_DATA( D_NUV_PORT(D_GET_PORT(D_PIN_KEY_L)) ) >> 2) & 0x3;

	if (NewEnc ^ OldEnc)
	{
		U8 enc_next = enc_table[OldEnc];
		//U8 enc_prev = ~enc_next & 0x03;
		U8 enc_prev = enc_table[3-OldEnc];

		if(enc_next == NewEnc)
			wheel_counter++;
		else if(enc_prev == NewEnc)
			wheel_counter--;
		else
			missed++;
	}

	//wheel_counter = int_counter;
	//wheel_counter = missed;

	OldEnc = NewEnc;
}


void  EINT0_IRQHandler(void)
{
	EINTx_IRQHandler();

	GPIO_CLR_INT_FLAG(P3, 1<<2);	// P3.2
}


void EINT1_IRQHandler(void)
{
	EINTx_IRQHandler();

	GPIO_CLR_INT_FLAG(P3, 1<<3);	// P3.3

}




U16 d_key_read(void)
{
	return 0;
}


#define D_LEDx_0(row)	(D_LEDx_tC(row)|D_LEDx_tR(row)|D_LEDx_bR(row)|D_LEDx_bC(row)|D_LEDx_bL(row)|D_LEDx_tL(row))
#define D_LEDx_1(row)	(D_LEDx_tR(row)|D_LEDx_bR(row))
#define D_LEDx_2(row)	(D_LEDx_tC(row)|D_LEDx_tR(row)|D_LEDx_bC(row)|D_LEDx_bL(row)|D_LEDx_mC(row))
#define D_LEDx_3(row)	(D_LEDx_tC(row)|D_LEDx_tR(row)|D_LEDx_bR(row)|D_LEDx_bC(row)|D_LEDx_mC(row))
#define D_LEDx_4(row)	(D_LEDx_tR(row)|D_LEDx_bR(row)|D_LEDx_tL(row)|D_LEDx_mC(row))
#define D_LEDx_5(row)	(D_LEDx_tC(row)|D_LEDx_bR(row)|D_LEDx_bC(row)|D_LEDx_tL(row)|D_LEDx_mC(row))
#define D_LEDx_6(row)	(D_LEDx_tC(row)|D_LEDx_bR(row)|D_LEDx_bC(row)|D_LEDx_bL(row)|D_LEDx_tL(row)|D_LEDx_mC(row))
#define D_LEDx_7(row)	(D_LEDx_tC(row)|D_LEDx_tR(row)|D_LEDx_bR(row))
#define D_LEDx_8(row)	(D_LEDx_tC(row)|D_LEDx_tR(row)|D_LEDx_bR(row)|D_LEDx_bC(row)|D_LEDx_bL(row)|D_LEDx_tL(row)|D_LEDx_mC(row))
#define D_LEDx_9(row)	(D_LEDx_tC(row)|D_LEDx_tR(row)|D_LEDx_bR(row)|D_LEDx_bC(row)|D_LEDx_tL(row)|D_LEDx_mC(row))
#define D_LEDx_A(row)	(D_LEDx_tC(row)|D_LEDx_tR(row)|D_LEDx_bR(row)|D_LEDx_bL(row)|D_LEDx_tL(row)|D_LEDx_mC(row))
#define D_LEDx_b(row)	(D_LEDx_bR(row)|D_LEDx_bC(row)|D_LEDx_bL(row)|D_LEDx_tL(row)|D_LEDx_mC(row))
#define D_LEDx_C(row)	(D_LEDx_tC(row)|D_LEDx_bC(row)|D_LEDx_bL(row)|D_LEDx_tL(row))
#define D_LEDx_d(row)	(D_LEDx_bR(row)|D_LEDx_bC(row)|D_LEDx_bL(row)|D_LEDx_tR(row)|D_LEDx_mC(row))
#define D_LEDx_E(row)	(D_LEDx_tC(row)|D_LEDx_bC(row)|D_LEDx_bL(row)|D_LEDx_tL(row)|D_LEDx_mC(row))
#define D_LEDx_F(row)	(D_LEDx_tC(row)|D_LEDx_bL(row)|D_LEDx_tL(row)|D_LEDx_mC(row))




U16 u7seg[16] = {
D_LEDx_0(0)|D_LEDx_0(1),
D_LEDx_1(0)|D_LEDx_1(1),
D_LEDx_2(0)|D_LEDx_2(1),
D_LEDx_3(0)|D_LEDx_3(1),
D_LEDx_4(0)|D_LEDx_4(1),
D_LEDx_5(0)|D_LEDx_5(1),
D_LEDx_6(0)|D_LEDx_6(1),
D_LEDx_7(0)|D_LEDx_7(1),
D_LEDx_8(0)|D_LEDx_8(1),
D_LEDx_9(0)|D_LEDx_9(1),
D_LEDx_A(0)|D_LEDx_A(1),
D_LEDx_b(0)|D_LEDx_b(1),
D_LEDx_C(0)|D_LEDx_C(1),
D_LEDx_d(0)|D_LEDx_d(1),
D_LEDx_E(0)|D_LEDx_E(1),
D_LEDx_F(0)|D_LEDx_F(1),
0,
};



// Keys

// nur 1 out auf low, andere auf high (ergebnis input geht auf low wenn gedrueckt)
// left		o2 -> i2
// right	o3 -> i2
// vol_cur	o4 -> i2

// ONoff	o2 -> i1
// OVP		o4 -> i1


// nur 1 out auf high, andere auf low (ergebnis input geht auf     wenn gedrueckt)

U32 uKeyBuf = 0;

void d_read_keys(void)
{
	const a = 0;
	const b = 1;

	U32 uKeys = 0;

	for(U8 i = 0; i < 3; i++)
	{
		//TODO: write and read entire port

		D_GPIO(D_PIN_KEY_SCAN_0) = (i==0) ? a : b;
		D_GPIO(D_PIN_KEY_SCAN_1) = (i==1) ? a : b;
		D_GPIO(D_PIN_KEY_SCAN_2) = (i==2) ? a : b;

		uKeys |= D_GPIO(D_PIN_KEY_READ_1) ? 0x00 : (0x1<<(i*4));
		uKeys |= D_GPIO(D_PIN_KEY_READ_4) ? 0x00 : (0x2<<(i*4));
		uKeys |= D_GPIO(D_PIN_KEY_READ_2) ? 0x00 : (0x4<<(i*4));
		uKeys |= D_GPIO(D_PIN_KEY_READ_3) ? 0x00 : (0x8<<(i*4));
	}

	uKeyBuffer = uKeys;

#if 0
	d_LEDx_write_LEDs(D_LEDx_CC, !D_GPIO(D_PIN_KEY_L));
	d_LEDx_write_LEDs(D_LEDx_CV, !D_GPIO(D_PIN_KEY_R));
#endif
}

#define D_DOT_OFF		0
#define D_DOT_ON		1
#define D_DOT_IGNORE	2

U16 uLED_buf[D_DISPLAY_COLS];


void d_LEDs_write_digit(U8 row, U8 pos, U8 val, U8 dot_state)
{
	if(row >= 2 || pos >= 4 || val >= 16)
	{
		__BKPT(0);
		return;
	}

	U16 mask = (row==0) ? 0xFF00 : 0x00FF;

	U16 dot_mask = D_LEDx_DT(row);

	mask &= (dot_state == D_DOT_IGNORE) ? ~dot_mask : ~0;	// Do not overwrite the DOT

	uLED_buf[pos] &= ~(mask);

    uLED_buf[pos] |= (u7seg[val] & mask) | ( (dot_state == D_DOT_ON) ? dot_mask : 0);
}

#define D_LEDx_COL	4 // the column with the LEDs


void d_LEDx_write_dot(U8 row, U8 pos, U32 on)
{
	if(row >= 2 || pos >= 4 )
	{
		__BKPT(0);
		return;
	}

	const U16 dot_mask = D_LEDx_DT(row);

	if(on)
	    uLED_buf[pos] |= dot_mask;
	else
		uLED_buf[pos] &= ~(dot_mask);
}

void d_LEDx_write_LEDs(U16 led_mask, U32 on)
{
	if(on)
	    uLED_buf[D_LEDx_COL] |= led_mask;
	else
		uLED_buf[D_LEDx_COL] &= ~(led_mask);
}


// Base 10, aka decimal
void d_write_number_b10(U8 row, U32 val)
{
	val += 5;

	if(val > 99999)
	{
		__BKPT(24);
		val = 99999;
	}

	U32 uDigits[5] = {
			val / 10,
			val / 100,
			val / 1000,
			val / 10000,
			0
	};

	for(U32 i = 0; i < 4; i++)
	{
		uDigits[i] -= (uDigits[i+1] * 10);
		d_LEDs_write_digit(row, i, uDigits[i], D_DOT_IGNORE);
	}
}
// Base 16, aka hexadecimal
void d_write_number_b16(U8 row, U32 val)
{
	d_LEDs_write_digit(row, 0, (val>> 0) & 0xF, D_DOT_IGNORE);
	d_LEDs_write_digit(row, 1, (val>> 4) & 0xF, D_DOT_IGNORE);
	d_LEDs_write_digit(row, 2, (val>> 8) & 0xF, D_DOT_IGNORE);
	d_LEDs_write_digit(row, 3, (val>>12) & 0xF, D_DOT_IGNORE);
}



U16 uQ_buf[D_DISPLAY_COLS] = {D_PIN_DISPLAY_COL0, D_PIN_DISPLAY_COL1, D_PIN_DISPLAY_COL2, D_PIN_DISPLAY_COL3, D_PIN_DISPLAY_COLx};
U8 uLastCol = -1;

U8 d_count_bits(U32 uLED)
{
	return __builtin_popcount(uLED);
}

U16 d_LEDx_output_col(U8 col, U8 highlight)
{
	if(col >= D_DISPLAY_COLS)
	{
		__BKPT(0);
		return;
	}

	U32 uOutputVal = uLED_buf[col];

	if(highlight && col != D_PIN_DISPLAY_COLx)
	{
		uOutputVal &= (highlight & 1) ? 0x00FF : 0xFF00; //
	}

	if(uOutputVal != 0x0000)
	{
		d_74595_shift(D_PIN_DISPLAY_SHIFT, D_PIN_DISPLAY_DATA, uOutputVal, 16);
	}

	if(col != uLastCol && uLastCol < D_DISPLAY_COLS)
	{
		D_GPIO(uQ_buf[uLastCol]) = 0;	// disable output transistor for last column
	}

	if(uOutputVal != 0x0000)
	{
		d_74595_store(D_PIN_DISPLAY_STORE);
		D_GPIO(uQ_buf[col]) = 1;	// enable output transistor for current column
	}

	uLastCol = col;

	// count LEDs
	U16 uLEDcount = 5;
	U8 bits = 0;

	if(col == D_PIN_DISPLAY_COLx)
	{
		bits = d_count_bits(uOutputVal);
		// 10 LEDs
		uLEDcount = bits;
	}
	else
	{
		U32 u7Seg = uOutputVal & ~( D_LEDx_DT(0) | D_LEDx_DT(1) );
		bits = d_count_bits(u7Seg);

		// 14 LEDs
		uLEDcount = bits;
	}

	uLEDcount <<= 1;

	return uLEDcount;
}





#define D_ADC_HISTORY_BTS	8
#define D_ADC_HISTORY_SIZE	(1<<D_ADC_HISTORY_BTS)

volatile U32 uADC_last[4] = {0, 0, 0, 0};

volatile U16 uADC_hist[2][D_ADC_HISTORY_SIZE];

U32 ADC_irq_idx = 0;

/*

#define DAC_CALIB_VOLT_00	0
#define DAC_CALIB_AMPR_0	1
#define DAC_CALIB_VOLT_30	2
#define DAC_CALIB_AMPR_5	3
#define ADC_CALIB_VOLT_00	4
#define ADC_CALIB_AMPR_0	5
#define ADC_CALIB_VOLT_30	6
#define ADC_CALIB_AMPR_5	7
#define ADC_CALIB_NUM		8

U16	g_Calibration[ADC_CALIB_NUM];

 */

U32 uMaxSI_DAC[2] = {
	30000,	// 30 Volt
	50000	// 5 Amp
};

U32 uMaxSI_ADC[2] = {
	31000,	// 30 Volt
	50000	// 5 Amp
};

U32 d_SI_to_DAC(U8 uDACidx, U32 uSI_val)
{
	if(uDACidx > D_ADC_AMPR)
	{
		__BKPT(25);
		return 0;
	}

	const U32 uDAC_min = g_Calibration[DAC_CALIB_VOLT_00 + uDACidx];
	const U32 uDAC_max = g_Calibration[DAC_CALIB_VOLT_30 + uDACidx] - uDAC_min;

	U32 uDAC_val = uSI_val * uDAC_max;
	uDAC_val /= uMaxSI_DAC[uDACidx];

	uDAC_val += uDAC_min;	// _00

	return uDAC_val;
}

U32 d_ADC_to_SI(U8 uADCidx, U32 uADC_val)
{
	if(uADCidx > D_ADC_AMPR)
	{
		__BKPT(27);
		return 0;
	}

	const U32 uADC_min = g_Calibration[ADC_CALIB_VOLT_00 + uADCidx];
	const U32 uADC_max = g_Calibration[ADC_CALIB_VOLT_31 + uADCidx] - uADC_min;

	uADC_val = (uADC_val < uADC_min) ? 0 : (uADC_val - uADC_min);	// _00

	U32 uSI_val = uADC_val * uMaxSI_ADC[uADCidx];
	uSI_val /= uADC_max;

	return uSI_val;
}

U32 d_get_ADC_SI(U8 uADCidx)
{
	if(uADCidx > D_ADC_AMPR)
	{
		__BKPT(25);
		return 0;
	}

	U32 uADC_val = d_get_ADC(uADCidx);

	U32 uADC_SI = d_ADC_to_SI( uADCidx, uADC_val );

	return uADC_SI;
}

U32 d_get_ADC(U8 uADCidx)
{
	if(uADCidx > D_ADC_TEMP_MCU)
	{
		__BKPT(26);
		return 0;
	}
	else if(uADCidx > D_ADC_AMPR)
		return uADC_last[uADCidx];

	U32 uSum = 0;

	for(U32 i = 0; i < D_ADC_HISTORY_SIZE; i++)
	{
		uSum += uADC_hist[uADCidx][i];
	}

	return (uSum >> D_ADC_HISTORY_BTS);
}


void ADC_IRQHandler(void)
{
    U32 uFlags = ADC_GET_INT_FLAG(ADC, ADC_ADF_INT);

    if(uFlags & ADC_ADF_INT)
    {
    	uADC_hist[D_ADC_VOLT][ADC_irq_idx] =
    	uADC_last[D_ADC_VOLT] = (ADC_GET_CONVERSION_DATA(ADC, 0));

    	U32 uNewRelay = d_get_allowed_relay(uADC_last[D_ADC_VOLT], g_set_relay);

    	uADC_hist[D_ADC_AMPR][ADC_irq_idx] =
    	uADC_last[D_ADC_AMPR] = (ADC_GET_CONVERSION_DATA(ADC, 1));

    	//uADC_hist[D_ADC_TEMP_PSU][ADC_irq_idx] =
    	uADC_last[D_ADC_TEMP_PSU] = (ADC_GET_CONVERSION_DATA(ADC, 2));

    	//uADC_hist[D_ADC_TEMP_MCU][ADC_irq_idx] =
    	uADC_last[D_ADC_TEMP_MCU] = (ADC_GET_CONVERSION_DATA(ADC, 7));

    	ADC_irq_idx = (ADC_irq_idx+1) & (D_ADC_HISTORY_SIZE - 1);

    	if( uNewRelay != g_set_relay )
    	{
    		g_set_relay = uNewRelay;
    		d_set_relays( g_set_relay << 6 );
    	}

    }



    ADC_CLR_INT_FLAG(ADC, uFlags);

}

U8 d_set_output(U8 opt)
{
	g_OutputEnable = (opt == D_OUTPUT_TOGGLE) ? !g_OutputEnable : opt;

	d_set_volt_ampr();

	D_GPIO(D_PIN_FAN) = g_OutputEnable;

	return g_OutputEnable;
}


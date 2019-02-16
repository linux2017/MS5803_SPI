
#include <stdio.h>
#include <math.h>
#include "utils.h"
#include "debug.h"
//#include "i2c.h"
#include "driver_interface.h"
#include "ms5803_spi.h"
static  uint32 C[8];   // calibration coefficients

static double P;   // compensated pressure value
static double T;   // compensated temperature value
uint8 crc4 ( uint32 n_prom[] );
uint8 MS5803_Reset ( void );
uint32 MS5803_ReadPROM ( char coef_num );
#define IMITATE_SPI
#ifndef IMITATE_SPI

#define     MS5803_SPI_ENABLE()         SSP_SSELToggle(1, 3, 0)
#define     MS5803_SPI_DISABLE()        SSP_SSELToggle(1, 3, 1)
#else
#warning "imitate spi with GPIO0 pins"

#define     MS5803_SPI_ENABLE()         (LPC_GPIO0->CLR |= 1UL << 15)
#define     MS5803_SPI_DISABLE()        (LPC_GPIO0->SET |= 1UL << 15)

#define     MS5803_SPI_CLK_0()         	(LPC_GPIO2->CLR |= 1UL << 8)
#define     MS5803_SPI_CLK_1()        	(LPC_GPIO2->SET |= 1UL << 8)

#define     MS5803_SPI_MO_0()         	(LPC_GPIO2->CLR |= 1UL << 3)
#define     MS5803_SPI_MO_1()        	(LPC_GPIO2->SET |= 1UL << 3)

#define     MS5803_SPI_MI()         	 (LPC_GPIO2->PIN & ( 1UL << 4 ) )
/*
#define     MS5803_SPI_ENABLE()         (LPC_GPIO0->CLR |= 1UL << 6)
#define     MS5803_SPI_DISABLE()        (LPC_GPIO0->SET |= 1UL << 6)

#define     MS5803_SPI_CLK_0()         	(LPC_GPIO0->CLR |= 1UL << 7)
#define     MS5803_SPI_CLK_1()        	(LPC_GPIO0->SET |= 1UL << 7)

#define     MS5803_SPI_MO_0()         	(LPC_GPIO0->CLR |= 1UL << 9)
#define     MS5803_SPI_MO_1()        	(LPC_GPIO0->SET |= 1UL << 9)

#define     MS5803_SPI_MI()         	 (LPC_GPIO0->PIN & ( 1UL << 8 ) )
*/
void IMITATE_SPI_init ( void )
{

	set_pin_dir(LPC_GPIO0,15,PIN_OUTPUT);
	set_pin_dir(LPC_GPIO2,8,PIN_OUTPUT);
	set_pin_dir(LPC_GPIO2,3,PIN_OUTPUT);
	set_pin_dir(LPC_GPIO2,4,PIN_INPUT);
    	
/*
    set_pin_dir ( LPC_GPIO0, 9, PIN_OUTPUT );
    set_pin_dir ( LPC_GPIO0, 6, PIN_OUTPUT );
    set_pin_dir ( LPC_GPIO0, 7, PIN_OUTPUT );
    set_pin_dir ( LPC_GPIO0, 8, PIN_INPUT );
 */
    MS5803_SPI_CLK_0();
    MS5803_SPI_DISABLE();
}

uint8 IMITATE_SPI_send_byte ( uint8 byte )
{
    uint8 i;
    MS5803_SPI_CLK_0();
    OSIntEnter();

    for ( i = 0;i < 8;i++ )
    {
        if ( ( byte& ( 0x80 >> i ) ) != 0 )
        {
            MS5803_SPI_MO_1();

        }
        else
        {
            MS5803_SPI_MO_0();
        }

        delay ( 1 );

        MS5803_SPI_CLK_1();
        delay ( 5 );
        MS5803_SPI_CLK_0();

    }

    OSIntExit();

    return 1;
}

uint8 IMITATE_SPI_recv_byte ( void )
{
    uint8 byte = 0;
    uint8 i;
    MS5803_SPI_CLK_0();
    OSIntEnter();

    for ( i = 0;i < 8;i++ )
    {
        delay ( 3 );
        MS5803_SPI_CLK_1();
        byte <<= 1;

        if ( MS5803_SPI_MI() )
        {
            byte += 1;

        }

        delay ( 2 );

        MS5803_SPI_CLK_0();

    }

    OSIntExit();

    return byte;
}

#endif
uint8 MS5803_init ( void )
{
	uint8 i;
	uint8 n_crc; // crc value of the prom
#ifndef IMITATE_SPI
    SSP1Init ( 3 );
#else

    IMITATE_SPI_init();
#endif

	MS5803_Reset();

    for ( i = 0;i < 8;i++ )
    {
        C[i] = MS5803_ReadPROM ( i );
        DEBUG_PRINT ( DEBUG_ERROR, DEBUG_MS5803, "MS5803 PROM[%d] data=%d\r\n", i, C[i] );
    } // read coefficients

    n_crc = crc4 ( C );

    if ( n_crc != ( C[7]&0x0000000f ) )
    {
        DEBUG_PRINT ( DEBUG_ERROR, DEBUG_MS5803, "MS5803 PROM crc=%x\r\n", n_crc );
		return 0;
    }
    return 1;
}

uint8 MS5803_write_cmd ( uint8 cmd )
{
#ifndef IMITATE_SPI/*  发送写使能信号               */
    SSPSend ( 1, &cmd, 1 );
#else
    IMITATE_SPI_send_byte ( cmd );
#endif
    return 1;

}

uint8 MS5803_read_data ( uint8 *recv, uint8 num )
{
    uint8 i;
#ifndef IMITATE_SPI
    SSPReceive ( 1, recv, num );
#else

    for ( i = 0;i < num;i++ )
    {
        recv[i] = IMITATE_SPI_recv_byte();
    }

#endif
    return 1;
}

//********************************************************
//! @brief send reset sequence //!
//! @return none
//********************************************************
uint8 MS5803_Reset ( void )
{
    uint8 ret = 0;
    uint8 i;
    uint8 send_cmd = MS5803_CMD_RESET;
    MS5803_SPI_ENABLE();
    ret = MS5803_write_cmd ( send_cmd );
    MS5803_SPI_DISABLE();
    delayms ( 50 );  // wait for the reset sequence timing
    DEBUG_PRINT ( DEBUG_ERROR, DEBUG_MS5803, "reset MS5803 %s\r\n", ret ? "OK" : "ERROR" );

    for ( i = 0;i < 8;i++ )
    {
        C[i] = 0;
    }

    return ret;
}

//********************************************************
//! @brief preform adc conversion
//! //! @return 24bit result
//********************************************************
uint32 MS5803_ReadAdcData ( uint8 cmd )
{
    uint32 ret = 0;
    ulong temp = 0;
    uint8 send_cmd = MS5803_CMD_ADC_CONV + cmd;
    uint8 recv_bytes[4] = {0, 0, 0, 0};
    MS5803_SPI_ENABLE();
    ret = MS5803_write_cmd ( send_cmd );
    MS5803_SPI_DISABLE();

    if ( !ret )
    {
        DEBUG_PRINT ( DEBUG_ERROR, DEBUG_MS5803, "send %02X CONV ADC CMD fail\r\n", send_cmd );
    }

    // send conversion command
    switch ( cmd & 0x0f )
        // wait necessary conversion time
    {

    case MS5803_CMD_ADC_256 :
        delay ( 3000 );
        break;

    case MS5803_CMD_ADC_512 :
        delayms ( 15 );
        break;

    case MS5803_CMD_ADC_1024:
        delayms ( 20 );
        break;

    case MS5803_CMD_ADC_2048:
        delayms ( 30 );
        break;

    case MS5803_CMD_ADC_4096:
        delayms ( 50 );
        break;
    }

    send_cmd = MS5803_CMD_ADC_READ;

    MS5803_SPI_ENABLE();
    ret = MS5803_write_cmd ( send_cmd );

    if ( !ret )
    {
        MS5803_SPI_DISABLE();
        DEBUG_PRINT ( DEBUG_ERROR, DEBUG_MS5803, "Send %02X READ ADC CMD fail\r\n", send_cmd );
    }
    else
    {//issuing start condition ok, device accessible
        ret = MS5803_read_data ( recv_bytes, 3 );
        MS5803_SPI_DISABLE();

        if ( !ret )
        {
            DEBUG_PRINT ( DEBUG_ERROR, DEBUG_MS5803, "READ ADC data fail data=%d\r\n", temp );
        }
        else
        {
            temp = recv_bytes[0];
            temp <<= 8;
            temp += recv_bytes[1];
            temp <<= 8;
            temp += recv_bytes[2];
            DEBUG_PRINT ( DEBUG_ERROR, DEBUG_MS5803, "MS5803 ADC data=%d\r\n", temp );
        }
    }



    return temp;
}

//********************************************************
//! @brief calculate the CRC code
//!
//! @return crc code
//********************************************************
uint8 crc4 ( uint32 n_prom[] )
{
    int32 cnt;     // simple counter
    uint32 n_rem;    // crc reminder
    uint32 crc_read;   // original value of the crc
    uint8  n_bit;
    n_rem = 0x00;
    crc_read = n_prom[7];  //save read CRC
    n_prom[7] = ( 0xFF00 & ( n_prom[7] ) );  //CRC byte is replaced by 0

    for ( cnt = 0; cnt < 16; cnt++ )    // operation is performed on bytes
    {// choose LSB or MSB
        if ( cnt % 2 == 1 )
            n_rem ^= ( uint16 ) ( ( n_prom[cnt>>1] ) & 0x00FF );
        else
            n_rem ^= ( uint16 ) ( n_prom[cnt>>1] >> 8 );

        for ( n_bit = 8; n_bit > 0; n_bit-- )
        {
            if ( n_rem & ( 0x8000 ) )
            {
                n_rem = ( n_rem << 1 ) ^ 0x3000;
            }
            else
            {
                n_rem = ( n_rem << 1 );
            }
        }
    }

    n_rem =  ( 0x000F & ( n_rem >> 12 ) );  // final 4-bit reminder is CRC code

    n_prom[7] = crc_read; // restore the crc_read to its original place
    return ( n_rem ^ 0x0 );
}

//********************************************************
//! @brief Read calibration coefficients
//!
//! @return coefficient
//********************************************************
uint32 MS5803_ReadPROM ( char coef_num )
{
    uint32 ret;
    uint32 rC = 0;
    uint8 recv_bytes[2] = {0, 0};
    uint8 send_cmd = MS5803_CMD_PROM_RD + ( coef_num << 1 );
    MS5803_SPI_ENABLE();
    ret = MS5803_write_cmd ( send_cmd );

    if ( !ret )
    {
        DEBUG_PRINT ( DEBUG_ERROR, DEBUG_MS5803, "Send %02X READ PROM CMD fail\r\n", send_cmd );
    }

    //issuing start condition ok, device accessible
    ret = MS5803_read_data ( recv_bytes, 2 );

    MS5803_SPI_DISABLE();

    if ( ret )
    {
        rC  = recv_bytes[0];
        rC  <<= 8;
        rC  += recv_bytes[1];
    }

    return rC;
}

//********************************************************
//! @brief main program
//!
//! @return 0
//********************************************************
uint32 deal_MS5803_SPI ( void )
{
    uint32 D1;    // ADC value of the pressure conversion
    uint32 D2;    // ADC value of the temperature conversion
    double dT;   // difference between actual and measured temperature
    double OFFSET;   // offset at actual temperature
    double SENS;   // sensitivity at actual temperature
    uint8 i;
    double T2, OFFSET2, SENS2;
	double _P=0,_T=0;
    D1 = 0;
    D2 = 0;

	for(i=0;i<4;i++){
	    // calculate the CRC
	    D2 = MS5803_ReadAdcData ( MS5803_CMD_ADC_D2 + MS5803_CMD_ADC_4096 );   // read D2

	    D1 = MS5803_ReadAdcData ( MS5803_CMD_ADC_D1 + MS5803_CMD_ADC_4096 );   // read D1

	    DEBUG_PRINT ( DEBUG_ERROR, DEBUG_MS5803, "MS5803 D1=%d D2=%d\r\n", D1, D2 );

	    // calcualte 1st order pressure and temperature (MS5607 1st order algorithm)
	    dT = D2 - C[5] * pow ( 2, 8 );

	    OFFSET = C[2] * pow ( 2, 16 ) + dT * C[4] / pow ( 2, 7 );

	    SENS = C[1] * pow ( 2, 15 ) + dT * C[3] / pow ( 2, 8 );

	    T = ( 2000 + ( dT * C[6] ) / pow ( 2, 23 ) );

	    P = ( ( ( D1 * SENS ) / pow ( 2, 21 ) - OFFSET ) / pow ( 2, 15 ) ) / 100;

	    //DEBUG_PRINT ( DEBUG_ERROR, DEBUG_MS5803, "MS5803 T=%f,RealTemprature=%f\r\n", T, RealTemprature );

	    //DEBUG_PRINT ( DEBUG_ERROR, DEBUG_MS5803, "MS5803 SENS=%f\r\n", SENS );

	    //DEBUG_PRINT ( DEBUG_ERROR, DEBUG_MS5803, "MS5803 OFF=%f\r\n", OFFSET );

	    //DEBUG_PRINT ( DEBUG_ERROR, DEBUG_MS5803, "MS5803 DT=%f\r\n", dT );

	    DEBUG_PRINT ( DEBUG_ERROR, DEBUG_MS5803, "MS5803 P=%f\r\n", P );

	    if ( T < 2000 )
	    {
	        T2 = pow ( dT, 2 ) / pow ( 2, 31 );

	        OFFSET2 = 3 * ( T - 2000 ) * ( T - 2000 );

	        SENS2 = 7 * ( T - 2000 ) * ( T - 2000 ) / pow ( 2, 3 );

	        if ( T < -1500 )
	        {

	            SENS2 = SENS2 + 2 * ( T + 1500 ) * ( T + 1500 );
	        }
	    }
	    else
	    {

	        T2 = 0;
	        OFFSET2 = 0;
	        SENS2 = 0;

	        if ( T >= 4500 )
	        {
	            SENS2 = SENS2 * ( T - 4500 ) * ( T - 4500 ) / pow ( 2, 3 );

	        }
	    }

	    //DEBUG_PRINT ( DEBUG_ERROR, DEBUG_MS5803, "MS5803 T2=%f, OFFSET2=%f,SENS2=%f\r\n", T2, OFFSET2, SENS2 );

	    T = T - T2;

	    OFFSET = OFFSET - OFFSET2;

	    SENS = SENS - SENS2;

	    //DEBUG_PRINT ( DEBUG_ERROR, DEBUG_MS5803, "MS5803 T=%f, OFFSET=%f,SENS=%f\r\n", T, OFFSET, SENS );
	    P = ( ( ( D1 * SENS ) / pow ( 2, 21 ) - OFFSET ) / pow ( 2, 15 ) ) / 100;
		OSTimeDly(10);
		_P +=P;
		_T +=T;
	    DEBUG_PRINT ( DEBUG_ERROR, DEBUG_MS5803, "MS5803 cal P=%f\r\n", P );
	}
	P=_P/4;
	T=_T/4;
	if(T<-4000||T>8000){
		DEBUG_PRINT ( DEBUG_ERROR, DEBUG_MS5803, "MS5803 avr T=%f Out of Range\r\n", T/100 );
	}

	if(P<300||P>1300){//hPa
		DEBUG_PRINT ( DEBUG_ERROR, DEBUG_MS5803, "MS5803 avr P=%f Out of Range\r\n", P);
	}

	DEBUG_PRINT ( DEBUG_ERROR, DEBUG_MS5803, "MS5803 avr T=%f P=%f\r\n", T/100,P );
    return 1;
}

double MS5803_get_T(void)
{
	return T/100;
}
double MS5803_get_P(void)
{
	return P*100;
}

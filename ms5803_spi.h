//! 
//! @file an520_I2C.c,v 
//! 
//! Copyright (c) 2009 MEAS Switzerland  
//!  
//! 
//! 
//! @brief This C code is for starter reference only. It is written for the 
//! MEAS Switzerland  MS56xx pressure sensor modules and Atmel Atmega644p 
//! microcontroller. //! //! @version 1.0 $Id: an520_I2C.c,v 1.0  
//! 
//! @todo

#ifndef __MS5803_SPI__H__
#define __MS5803_SPI__H__
#ifndef TRUE
#define TRUE 1 
#endif
#ifndef FALSE
#define FALSE 0 
 #endif
 #define ANALOG_I2C 1
#define MS5803_SPIADDR  0xEE    // Module address write mode 
 
#define MS5803_CMD_RESET   		0x1E  // ADC reset command 
#define MS5803_CMD_ADC_READ 	0x00  // ADC read command 
#define MS5803_CMD_ADC_CONV 	0x40  // ADC conversion command 
#define MS5803_CMD_ADC_D1   	0x00    // ADC D1 conversion 
#define MS5803_CMD_ADC_D2   	0x10    // ADC D2 conversion 
#define MS5803_CMD_ADC_256  	0x00    // ADC OSR=256 
#define MS5803_CMD_ADC_512  	0x02    // ADC OSR=512 
#define MS5803_CMD_ADC_1024 	0x04    // ADC OSR=1024 
#define MS5803_CMD_ADC_2048 	0x06    // ADC OSR=2048 
#define MS5803_CMD_ADC_4096 	0x08    // ADC OSR=4096 
#define MS5803_CMD_PROM_RD  	0xA0  // Prom read command 
uint8 MS5803_init ( void );
uint32 deal_MS5803_SPI (void) ;


double MS5803_get_T(void);
double MS5803_get_P(void);
#endif
 


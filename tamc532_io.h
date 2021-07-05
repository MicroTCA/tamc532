/**
*Copyright 2016-  DESY (Deutsches Elektronen-Synchrotron, www.desy.de)
*
*This file is part of TAMC532 driver.
*
*TAMC532 is free software: you can redistribute it and/or modify
*it under the terms of the GNU General Public License as published by
*the Free Software Foundation, either version 3 of the License, or
*(at your option) any later version.
*
*TAMC532 is distributed in the hope that it will be useful,
*but WITHOUT ANY WARRANTY; without even the implied warranty of
*MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*GNU General Public License for more details.
*
*You should have received a copy of the GNU General Public License
*along with TAMC532.  If not, see <http://www.gnu.org/licenses/>.
**/


#ifndef TAMC532_IO_H
	#define TAMC532_IO_H

	#include "tamc532_reg.h"

	typedef uint64_t pointer_type;

	#define ADCCHANNUM      32
	#define DMACNUM            4
	#define CSPTNUM             4

	#define I2C_BCC_BUS	           0x0 
	#define I2C_RTM_BUS	           0x1 
	#define I2C_PL_BUS	               0x2 
	#define I2C_SFP0_BUS	           0x3 
	#define I2C_SFP1_BUS	           0x4 

	#define TAMC532_CSPTA      0x1
	#define TAMC532_CSPTB      0x2
	#define TAMC532_CSPTC      0x4
	#define TAMC532_CSPTD      0x8
	#define TAMC532_CSPTALL   0xF

	#define TAMC532_TRG_RTMD5      0x1
	#define TAMC532_TRG_RTMD6      0x2
	#define TAMC532_TRG_RTMD7      0x3
	#define TAMC532_TRG_RTMD8      0x4
	#define TAMC532_AMC_RX17        0x5
	#define TAMC532_AMC_TX17        0x6
	#define TAMC532_AMC_RX18        0x7
	#define TAMC532_AMC_TX18        0x8
	#define TAMC532_AMC_RX19        0x9
	#define TAMC532_AMC_TX19        0xA
	#define TAMC532_AMC_RX20        0xB
	#define TAMC532_AMC_TX20        0xC


	struct device_ioctrl_stream_dma  {
		unsigned int   dma_offset;
		unsigned int   dma_size;
		unsigned int   dma_cmd;          // value to DMA Control register
		unsigned int   dma_pattern;     // DMA BAR num
		unsigned int   dma_reserved1; // DMA Control register offset (31:16) DMA Length register offset (15:0)
		unsigned int   dma_reserved2; // DMA Read/Write Source register offset (31:16) Destination register offset (15:0)
		unsigned int   pre_dma_status_reg[4];
		unsigned int   dma_control_reg[4]; 
		unsigned int   cspt_control[4]; 
		pointer_type  dataBuf[4];	// user side DMA Buf 0
	};
	typedef struct device_ioctrl_stream_dma device_ioctrl_stream_dma;
	
	struct device_staus_registers  {
		unsigned int   module_control_reg;
		unsigned int   module_status_reg;
		unsigned int   dma_control_reg[4];         
		unsigned int   dma_status_reg[4];   
		unsigned int   cspt_control[4]; 
		unsigned int   application_control_reg; 
		unsigned int   application_status_reg; 
		unsigned int   adc_channel_data[16]; 
	};
	typedef struct device_staus_registers device_staus_registers;

	struct device_adc_data_registers  {
		unsigned int   adc_channel_data[16]; 
	};
	typedef struct device_adc_data_registers device_adc_data_registers;

	/* Use 'o' as magic number */

	#define TAMC532_IOC           			'0'
	#define TAMC532_GET_DMA_TIME 	    _IOWR(TAMC532_IOC, 20, int)
	#define TAMC532_READ_DMA                   _IOWR(TAMC532_IOC, 21, int)
	#define TAMC532_WRITE_DMA                 _IOWR(TAMC532_IOC, 22, int)
	#define TAMC532_STOP_DMA                   _IOWR(TAMC532_IOC, 23, int)
	#define TAMC532_BLINK_LED                   _IOWR(TAMC532_IOC, 24, int)
	#define TAMC532_I2C_RW                         _IOWR(TAMC532_IOC, 25, int)
	#define TAMC532_ENDIAN                        _IOWR(TAMC532_IOC, 26, int)
	#define TAMC532_TRG_POLARITY            _IOWR(TAMC532_IOC, 27, int)
	#define TAMC532_TRG_SOURCE               _IOWR(TAMC532_IOC, 28, int)
	#define TAMC532_ARM_CSPT                    _IOWR(TAMC532_IOC, 29, int)

	#define TAMC532_RAISE_SW_TRG              _IOWR(TAMC532_IOC, 30, int)
	#define TAMC532_CSPT_RESET                   _IOWR(TAMC532_IOC, 31, int)
	#define TAMC532_SET_DMA                         _IOWR(TAMC532_IOC, 32, int)
	#define TAMC532_REM_DMA                       _IOWR(TAMC532_IOC, 33, int)
	#define TAMC532_IRQ_STS                           _IOWR(TAMC532_IOC, 34, int)

	#define TAMC532_SET_STREAM_DMA			_IOWR(TAMC532_IOC, 35, int)
	#define TAMC532_READ_STREAM_DMA		_IOWR(TAMC532_IOC, 36, int)
	#define TAMC532_REMOVE_STREAM_DMA 	_IOWR(TAMC532_IOC, 37, int)

	#define TAMC532_READ_STATUS_REG		_IOWR(TAMC532_IOC, 38, int)
	#define TAMC532_GET_STATUS_REG			_IOWR(TAMC532_IOC, 39, int)
	#define TAMC532_GET_ADC_DATA			_IOWR(TAMC532_IOC, 40, int)

	#define TAMC532_SET_CONT_DMA			_IOWR(TAMC532_IOC, 41, int)
	#define TAMC532_READ_CONT_DMA			_IOWR(TAMC532_IOC, 42, int)
	#define TAMC532_REMOVE_CONT_DMA		_IOWR(TAMC532_IOC, 43, int)

	#define TAMC532_IOC_MAXNR           44
	#define TAMC532_IOC_MINNR            20

#endif	/* TAMC532_IO_H */


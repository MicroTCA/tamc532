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
    #define TAMC532_CSPT_RESET                 _IOWR(TAMC532_IOC, 31, int)
    #define TAMC532_SET_DMA                     _IOWR(TAMC532_IOC, 32, int)
    #define TAMC532_REM_DMA                    _IOWR(TAMC532_IOC, 33, int)
    #define TAMC532_IRQ_STS                    _IOWR(TAMC532_IOC, 34, int)

    #define TAMC532_IOC_MAXNR           35
    #define TAMC532_IOC_MINNR            20

#endif	/* TAMC532_IO_H */


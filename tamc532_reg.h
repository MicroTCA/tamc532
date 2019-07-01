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

#ifndef TAMC532_REG_H_
#define TAMC532_REG_H_

#define MODULE_CONTROL_REG  		   0x00
#define MODULE_STATUS_REG                  0x04
#define MODULE_INTERRUPT_ENABLE_REG        0x08
#define MODULE_INTERRUPT_STAUS_REG	   0x0C
#define ADC_CHANNEL01_DATA_REG		   0x10
#define ADC_CHANNEL23_DATA_REG		   0x14
#define ADC_CHANNEL45_DATA_REG	           0x18
#define ADC_CHANNEL67_DATA_REG	           0x1C
#define ADC_CHANNEL89_DATA_REG	  	   0x20
#define ADC_CHANNEL1011_DATA_REG	   0x24
#define ADC_CHANNEL1213_DATA_REG	   0x28
#define ADC_CHANNEL1415_DATA_REG	   0x2C
#define ADC_CHANNEL1617_DATA_REG	   0x30
#define ADC_CHANNEL1819_DATA_REG	   0x34
#define ADC_CHANNEL2021_DATA_REG	   0x38
#define ADC_CHANNEL2223_DATA_REG	   0x3C
#define ADC_CHANNEL2425_DATA_REG	   0x40
#define ADC_CHANNEL2627_DATA_REG	   0x44
#define ADC_CHANNEL2829_DATA_REG	   0x48
#define ADC_CHANNEL3031_DATA_REG	   0x4C
#define I2C_BRIDGE_CONTROL_REG	           0x50
#define I2C_BRIDGE_CLOCK_DIVIDER_REG       0x54
#define I2C_BRIDGE_STATUS_REG	           0x58
#define I2C_BRIDGE_COMMAND_REG	           0x5C
#define I2C_BRIDGE_W_DATA_FIFO_REG     	   0x60
#define I2C_BRIDGE_R_DATA_FIFO_REG	   0x64
#define DMAC0_CONTROL_REG	           0x80
#define DMAC0_STATUS_REG	           0x84
#define DMAC0_BASE_DESCR_ADDRESS_REG	   0x88
#define DMAC0_CURRENT_MEM_WR_ADDRESS_REG	   0x8C
#define DMAC1_CONTROL_REG	           0x90
#define DMAC1_STATUS_REG	           0x94
#define DMAC1_BASE_DESCR_ADDRESS_REG	   0x98
#define DMAC1_CURRENT_MEM_WR_ADDRESS_REG	   0x9C
#define DMAC2_CONTROL_REG	           0xA0
#define DMAC2_STATUS_REG	           0xA4
#define DMAC2_BASE_DESCR_ADDRESS_REG	   0xA8
#define DMAC2_CURRENT_MEM_WR_ADDRESS_REG	   0xAC
#define DMAC3_CONTROL_REG	           0xB0
#define DMAC3_STATUS_REG	           0xB4
#define DMAC3_BASE_DESCR_ADDRESS_REG	   0xB8
#define DMAC3_CURRENT_MEM_WR_ADDRESS_REG	   0xBC
#define APPLICATION_CONTROL_REG 	   0x100
#define CSPTA_CONTROL_REG	           0x104
#define CSPTA_DATA0_REG	                   0x108
#define CSPTA_DATA1_REG	                   0x10C
#define CSPTA_RESERVED	                   0x110
#define CSPTB_CONTROL_REG	           0x114
#define CSPTB_DATA0_REG	                   0x118
#define CSPTB_DATA1_REG	                   0x11C
#define CSPTB_RESERVED	                   0x120
#define CSPTC_CONTROL_REG	           0x124
#define CSPTC_DATA0_REG	                   0x128
#define CSPTC_DATA1_REG	                   0x12C
#define CSPTC_RESERVED	                   0x130
#define CSPTD_CONTROL_REG	           0x134
#define CSPTD_DATA0_REG	                   0x138
#define CSPTD_DATA1_REG	                   0x13C
#define CSPTD_RESERVED	                   0x140
#define APPLICATION_STATUS_REG	           0x160
#define APPLICATION_COMMAND_REG	           0x164
#define FIRMWARE_ID_REG                    0x1FC

/* TAMC532_REG_I2C_CONTROL */
#define TAMC532_I2CCONTROL_MASTERENA        (1 << 0)

/* TAMC532_REG_I2C_STATUS */
#define TAMC532_I2CSTATUS_DEV_OP_ERR        (1 << 13)
#define TAMC532_I2CSTATUS_BUS_ARBL          (1 << 12)
#define TAMC532_I2CSTATUS_DEV_DET           (1 << 8)
#define TAMC532_I2CSTATUS_IWRS_OP_IP        (1 << 5)
#define TAMC532_I2CSTATUS_MST_IDLE          (1 << 4)
#define TAMC532_I2CSTATUS_I2C_BUS_LS        (1 << 0)

/* TAMC532_REG_I2C_COMMAND */
#define TAMC532_I2CCOMMAND_WRS_OP_CMD       (1 << 8)
#define TAMC532_I2CCOMMAND_RD_DFIFO_RST     (1 << 5)
#define TAMC532_I2CCOMMAND_WR_DFIFO_RST     (1 << 4)
#define TAMC532_I2CCOMMAND_I2C_MST_RST      (1 << 0)

/* TAMC532_REG_DMA_CONTROL */
#define TAMC532_DMACONTROL_LDBDA            (1 << 17)
#define TAMC532_DMACONTROL_RST              (1 << 16)
#define TAMC532_DMACONTROL_EN               (1 << 0)

/* TAMC532_REG_CSPT_CONTROL */
#define TAMC532_CSPTCONTROL_DEM             (1 << 16)
#define TAMC532_CSPTCONTROL_MASK_TP         (0xF000)
#define TAMC532_CSPTCONTROL_MASK_ET         (0x0F00)

/*
** Register Bit Definitions
*/
/* TAMC532_REG_MODULECONTROL */
#define TAMC532_MODULECONTROL_MODINTGE      (1 << 0)

/* TAMC532_REG_MODULESTATUS */
#define TAMC532_MODULESTATUS_Z3EN           (1 << 8)
#define TAMC532_MODULESTATUS_ADCCALOK_D     (1 << 7)
#define TAMC532_MODULESTATUS_ADCCALOK_C     (1 << 6)
#define TAMC532_MODULESTATUS_ADCCALOK_B     (1 << 5)
#define TAMC532_MODULESTATUS_ADCCALOK_A     (1 << 4)
#define TAMC532_MODULESTATUS_INTMODE_MSI    (1 << 3)
#define TAMC532_MODULESTATUS_INTMODE_INTA   (1 << 2)
#define TAMC532_MODULESTATUS_INTMODE_ERROR  (1 << 0)

/* TAMC532_REG_MODULEINTENA + TAMC532_REG_MODULEINTSTAT */
#define TAMC532_MODULEINT_DMAC3_EOL         (1 << 22)
#define TAMC532_MODULEINT_DMAC3_DMF         (1 << 21)
#define TAMC532_MODULEINT_DMAC3_DDL         (1 << 20)
#define TAMC532_MODULEINT_DMAC2_EOL         (1 << 18)
#define TAMC532_MODULEINT_DMAC2_DMF         (1 << 17)
#define TAMC532_MODULEINT_DMAC2_DDL         (1 << 16)
#define TAMC532_MODULEINT_DMAC1_EOL         (1 << 14)
#define TAMC532_MODULEINT_DMAC1_DMF         (1 << 13)
#define TAMC532_MODULEINT_DMAC1_DDL         (1 << 12)
#define TAMC532_MODULEINT_DMAC0_EOL         (1 << 10)
#define TAMC532_MODULEINT_DMAC0_DMF         (1 <<  9)
#define TAMC532_MODULEINT_DMAC0_DDL         (1 <<  8)
#define TAMC532_MODULEINT_I2C               (1 <<  4)
#define TAMC532_MODULEINT_BCC               (1 <<  2)

#define TAMC532_MODULEINT_DMACx_EOL(unit)   (TAMC532_MODULEINT_DMAC0_EOL << (unit * 4))
#define TAMC532_MODULEINT_DMACx_DMF(unit)   (TAMC532_MODULEINT_DMAC0_DMF << (unit * 4))
#define TAMC532_MODULEINT_DMACx_DDL(unit)   (TAMC532_MODULEINT_DMAC0_DDL << (unit * 4))
#define TAMC532_MODULEINT_DMACx_EOL_ALL    0x444400
#define TAMC532_MODULEINT_DMACx_DMF_ALL    0x222200

#define TAMC532_DESCR_HI_DDL        (1 << 4)
#define TAMC532_DESCR_HI_DMF        (1 << 1)
#define TAMC532_DESCR_HI_EOL        (1 << 0)

#define I2C_ACCESS_TIMEOUT	           8000   //usec

struct tamc532_dma_desc {
    unsigned int control;
    unsigned int next_desc;     // holds the physical address of the following descriptor
    unsigned int dataptr;       // holds the physical address of the linked data buffer
    unsigned int reserved;
};
typedef struct tamc532_dma_desc tamc532_dma_desc;

#endif // TAMC532_REG_

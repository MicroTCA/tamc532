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

#include <linux/version.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/pagemap.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/vmalloc.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/sched.h>
#include <asm/delay.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/dmapool.h>
#include <linux/time.h>

#include <linux/timer.h>
#include <linux/sched.h>

#include <asm/msr.h>    // used for timing measurement (debug only)

#include "tamc532_io.h"
#include "tamc532_fnc.h"
#include "tamc532_reg.h"

#if 0
    struct device_i2c_rw  {
       u_int            busNum; /* I2C Bus num*/
       u_int            devAddr;   /* I2C device address on the current bus */
       u_int            regAddr;   /* I2C register address on the current device */
       u_int            size;   /* number of bytes to  read/write*/
       u_int            done;  /* read done*/
       u_int            timeout;   /* transfer timeout */             
       u_int            status;  /* status */
       u_int            data[256];  /* data */
};
    typedef struct device_i2c_rw device_i2c_rw;
#endif


int tamc532_i2c_read(pciedev_dev * p_dev, device_i2c_rw * i2cbuf)
{
    int                   i                   = 0;
    int                   rd_status      = 0;
    int                   rd_count       = 0;
    int                   tm_count       = 0;
    int                   retval            = 0;
    int                   i2c_timeout  = 0;
    u32                 tmp_data_32 = 0;
    u8                   tmp_data_8   = 0;
    void*                address;
    
    if(!p_dev->dev_sts){
        printk("TAMC532_IOCTL_DMA: NO DEVICE %d\n", p_dev->dev_num);
        retval = -EFAULT;
        return retval;
    }
         
    address = pciedev_get_baraddress(BAR0, p_dev);
    if(!address){
        printk("TAMC532_IOCTL_DMA: NO BAR0 ON THE DEVICE %d\n", p_dev->dev_num);
        retval = -EFAULT;
        return retval;
    }
        
    rd_count     = i2cbuf->size;
    i2c_timeout = i2cbuf->timeout;
    if(!i2c_timeout) i2c_timeout = I2C_ACCESS_TIMEOUT;
    
#if 0   
    /* Enable I2C Interrupt we don't use it for now*/
            tmp_data_32 = READ_REGISTER_U32_BE( (uint32_t*)(pDCB->regbase + TAMC532_REG_MODULEINTENA));
            tmp_data_32 |= TAMC532_MODULEINT_I2C;
            iowrite32(TAMC532_SWAPL(tmp_data_32), (address + TAMC532_REG_MODULEINTENA));
# endif
            
    for(i = 0; i < rd_count; i++){
        
         /* disable I2C Bridge for reset */
        iowrite32(0, (address + I2C_BRIDGE_CONTROL_REG));
        smp_wmb();
        udelay(2);
        /*enable I2C Bridge Master*/
        iowrite32(TAMC532_SWAPL(TAMC532_I2CCONTROL_MASTERENA), (address + I2C_BRIDGE_CONTROL_REG));
        smp_wmb();
        /*read I2C Bridge Status*/
        tmp_data_32       = ioread32(address + I2C_BRIDGE_STATUS_REG);
        smp_rmb();
        /* Reset I2C Bridge, just to be sure. */
        tmp_data_32  = TAMC532_I2CCOMMAND_RD_DFIFO_RST | TAMC532_I2CCOMMAND_WR_DFIFO_RST |  TAMC532_I2CCOMMAND_I2C_MST_RST;
        iowrite32(TAMC532_SWAPL(tmp_data_32), (address + I2C_BRIDGE_COMMAND_REG));
        smp_wmb();
        /* Setup Clock Divider Register => 100kHz */
        iowrite32(TAMC532_SWAPL(0x04E2), (address + I2C_BRIDGE_CLOCK_DIVIDER_REG));
        smp_wmb();
        udelay(2);
        /* Setup Clock Divider Register => 50kHz */
        iowrite32(TAMC532_SWAPL(0x09C4), (address + I2C_BRIDGE_CLOCK_DIVIDER_REG));
        smp_wmb();
    
        /* setup bus-number, I2C address and length of Write or Read transaction */
        tmp_data_32  = ((i2cbuf->busNum << 28) | (i2cbuf->devAddr<< 20) | (0x1 << 12) | (0x1 << 4) | TAMC532_I2CCONTROL_MASTERENA);
        iowrite32(TAMC532_SWAPL(tmp_data_32), (address + I2C_BRIDGE_CONTROL_REG));
        smp_wmb();
        
        
        /*Setup the the register address, write it to the I2C Bridge Write FIFO Register */
        iowrite32(TAMC532_SWAPL(i2cbuf->regAddr + i), (address + I2C_BRIDGE_W_DATA_FIFO_REG));
        smp_wmb();
        /* start the transfer */
        iowrite32(TAMC532_SWAPL(TAMC532_I2CCOMMAND_WRS_OP_CMD), (address + I2C_BRIDGE_COMMAND_REG));
        smp_wmb();
        udelay(i2c_timeout);

        tmp_data_32       = ioread32(address + I2C_BRIDGE_STATUS_REG);
        smp_rmb();
        tmp_data_32       = TAMC532_SWAPL(tmp_data_32);
        rd_status             = (tmp_data_32 >> 5) & 0x1;
        tm_count             = 0;
        while(rd_status){
            udelay(I2C_ACCESS_TIMEOUT);
            tmp_data_32       = ioread32(address + I2C_BRIDGE_STATUS_REG);
            smp_rmb();
            tmp_data_32       = TAMC532_SWAPL(tmp_data_32);
            rd_status             = (tmp_data_32 >> 5) & 0x1;
            tm_count++;
            if(tm_count >10){
                rd_status  = 0;
	       tmp_data_32  = TAMC532_I2CCOMMAND_RD_DFIFO_RST | TAMC532_I2CCOMMAND_WR_DFIFO_RST |  TAMC532_I2CCOMMAND_I2C_MST_RST;
	       iowrite32(TAMC532_SWAPL(tmp_data_32), (address + I2C_BRIDGE_COMMAND_REG));
	       smp_wmb();
                printk(KERN_ALERT "TAMC532-PCIEDEV_I2C_READ: ERROR\n");
                retval = -EFAULT;
	       break;
            }
        }
        tmp_data_8       = ioread8(address + I2C_BRIDGE_R_DATA_FIFO_REG);
        smp_rmb();
        i2cbuf->data[i] = tmp_data_8;
    }

    return retval;
}

int tamc532_i2c_write(pciedev_dev * p_dev, device_i2c_rw * i2cbuf)
{
    int                   i                   = 0;
    int                   rd_status      = 0;
    int                   rd_count       = 0;
    int                   tm_count       = 0;
    int                   retval            = 0;
    int                   i2c_timeout  = 0;
    u32                 tmp_data_32 = 0;
    u8                   tmp_data_8   = 0;
    void*                address;
    
    if(!p_dev->dev_sts){
        printk("TAMC532_IOCTL_DMA: NO DEVICE %d\n", p_dev->dev_num);
        retval = -EFAULT;
        return retval;
    }
         
    address = pciedev_get_baraddress(BAR0, p_dev);
    if(!address){
        printk("TAMC532_IOCTL_DMA: NO BAR0 ON THE DEVICE %d\n", p_dev->dev_num);
        retval = -EFAULT;
        return retval;
    }
        
    rd_count     = i2cbuf->size;
    i2c_timeout = i2cbuf->timeout;
    if(!i2c_timeout) i2c_timeout = I2C_ACCESS_TIMEOUT;
    
     for(i = 0; i < rd_count; i++){
		/**FIRST WRITE REG ADDRESS**/
        /* disable I2C Bridge for reset */
        iowrite32(0, (address + I2C_BRIDGE_CONTROL_REG));
        smp_wmb();
        udelay(2);
        /*enable I2C Bridge Master*/
        iowrite32(TAMC532_SWAPL(TAMC532_I2CCONTROL_MASTERENA), (address + I2C_BRIDGE_CONTROL_REG));
        smp_wmb();
        /*read I2C Bridge Status*/
        tmp_data_32       = ioread32(address + I2C_BRIDGE_STATUS_REG);
        smp_rmb();
        /* Reset I2C Bridge, just to be sure. */
        tmp_data_32  = TAMC532_I2CCOMMAND_RD_DFIFO_RST | TAMC532_I2CCOMMAND_WR_DFIFO_RST |  TAMC532_I2CCOMMAND_I2C_MST_RST;
        iowrite32(TAMC532_SWAPL(tmp_data_32), (address + I2C_BRIDGE_COMMAND_REG));
        smp_wmb();
        /* Setup Clock Divider Register => 100kHz */
        iowrite32(TAMC532_SWAPL(0x04E2), (address + I2C_BRIDGE_CLOCK_DIVIDER_REG));
        smp_wmb();
        udelay(2);
        /* Setup Clock Divider Register => 50kHz */
        iowrite32(TAMC532_SWAPL(0x09C4), (address + I2C_BRIDGE_CLOCK_DIVIDER_REG));
        smp_wmb();
        
        /* setup bus-number, I2C address and length of Write or Read transaction */
        tmp_data_32  = ((i2cbuf->busNum << 28) | (i2cbuf->devAddr<< 20) | (0x2 << 12) | (0x0 << 4) | (0x1 << 0)| TAMC532_I2CCONTROL_MASTERENA);
        //printk(KERN_ALERT "TAMC532-PCIEDEV_I2C_WRITE: WRITE DATA %X TO I2C_BRIDGE_CONTROL_REG\n", tmp_data_32);
        iowrite32(TAMC532_SWAPL(tmp_data_32), (address + I2C_BRIDGE_CONTROL_REG));
        smp_wmb();
        /*Setup the the register address, write it to the I2C Bridge Write FIFO Register */
        tmp_data_32 = ( i2cbuf->regAddr + i) + ((i2cbuf->data[i] & 0xFF) << 8);
        //printk(KERN_ALERT "TAMC532-PCIEDEV_I2C_WRITE: WRITE DATA %X TO W_DATA_FIFO\n", tmp_data_32);
        iowrite32(TAMC532_SWAPL(tmp_data_32), (address + I2C_BRIDGE_W_DATA_FIFO_REG));
        smp_wmb();
        /* start the transfer */
        iowrite32(TAMC532_SWAPL(TAMC532_I2CCOMMAND_WRS_OP_CMD), (address + I2C_BRIDGE_COMMAND_REG));
        smp_wmb();
        udelay(i2c_timeout);
        tmp_data_32       = ioread32(address + I2C_BRIDGE_STATUS_REG);
        smp_rmb();
        tmp_data_32       = TAMC532_SWAPL(tmp_data_32);
        rd_status             = (tmp_data_32 >> 5) & 0x1;
        rd_count = 0;
        while(rd_status){
            udelay(I2C_ACCESS_TIMEOUT);
            tmp_data_32       = ioread32(address + I2C_BRIDGE_STATUS_REG);
            smp_rmb();
            tmp_data_32       = TAMC532_SWAPL(tmp_data_32);
            rd_status             = (tmp_data_32 >> 5) & 0x1;
            rd_count++;
            if(rd_count >10){
                printk(KERN_ALERT "TAMC532-PCIEDEV_I2C_WRITE: ERROR\n");
                rd_status  = 0;
	       tmp_data_32  = TAMC532_I2CCOMMAND_RD_DFIFO_RST | TAMC532_I2CCOMMAND_WR_DFIFO_RST |  TAMC532_I2CCOMMAND_I2C_MST_RST;
	       iowrite32(TAMC532_SWAPL(tmp_data_32), (address + I2C_BRIDGE_COMMAND_REG));
	       smp_wmb();
                retval = -EFAULT;
	       break;
            }
        }
    }
    return retval;
}

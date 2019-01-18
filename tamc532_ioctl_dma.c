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
#include <linux/swap.h>

#include <asm/msr.h>    // used for timing measurement (debug only)

#include "tamc532_io.h"
#include "tamc532_fnc.h"
#include "tamc532_reg.h"

long ms_to_ticks( long ms )
{
    return ( (ms * HZ / 1000) + 1);
}

long     tamc532_ioctl_dma(struct file *filp, unsigned int *cmd_p, unsigned long *arg_p)
{
    int                           i             = 0;
    int                           tmp_dma_num      = 0;
    int                           tmp_dma_count    = 0;
     int                          tmp_dma_count_offset    = 0;
    int                           retval      = 0;
    int                           tamcsts   = 0;
    int                           dma_wait_count   = 0;
    unsigned int            cmd;
    unsigned long           arg;
    pid_t                        cur_proc = 0;
    int                            minor    = 0;
    int                            d_num    = 0;
    void*                         address;
    struct pci_dev*          pdev;
    struct pciedev_dev*   dev ;
    struct tamc532_dev* tamc532dev ;
    tamc532_dma_desc   tmp_dma_desc;
    
    device_i2c_rw            i2c_data;
    int                             i2c_buf_size;
    device_ioctrl_data     io_data;
    int                             io_data_size;
    device_ioctrl_time     time_data;
    device_ioctrl_dma     dma_data;
    int                            size_time;
    int                            io_dma_size;

    u_int                  tmp_offset;
    u32                    tmp_data_32;
    u32                    tmp_data;
    
    ulong           value;
    u_int	    tmp_dma_size;
    u_int	    tmp_dma_count_size;
    u_int	    tmp_dma_size_rest;
    u_int	    tmp_dma_buf_size;
    u_int	    tmp_dma_trns_size;
    u_int	    tmp_dma_desc_size;
    u_int	    tmp_dma_offset;
    
    //void*           pWriteBuf           = 0;
    //void*           pDescBuf            = 0;
    //dma_addr_t      pTmpDmaHandle;
    //dma_addr_t      pTmpDescHandle;
    
    unsigned long   length              = 0;
    int              tmp_order            = 0;
    //u32             dma_sys_addr ;
    //int               tmp_source_address  = 0;
    
    long            timeDMAwait;
    int              tmp_free_pages = 0;
    int              dma_trans_cnt            = 0;
    int              dma_trans_rest           = 0; 
    int              dma_cnt_done            = 0;

   

    cmd                            = *cmd_p;
    arg                              = *arg_p;
    i2c_buf_size                = sizeof(device_i2c_rw);
    io_data_size                 = sizeof(device_ioctrl_data);
    size_time                     = sizeof (device_ioctrl_time);
    io_dma_size                 = sizeof(device_ioctrl_dma);

    dev                 = filp->private_data;
    tamc532dev    = (tamc532_dev   *)dev->dev_str;
    pdev               = (dev->pciedev_pci_dev);
    minor             = dev->dev_minor;
    d_num           = dev->dev_num;	
    cur_proc       = current->group_leader->pid;

    if(!dev->dev_sts){
        printk(KERN_ALERT "TAMC532_IOCTL_DMA: NO DEVICE %d\n", dev->dev_num);
        retval = -EFAULT;
        return retval;
    }

    address = pciedev_get_baraddress(BAR0, dev);
    if(!address){
        printk(KERN_ALERT "TAMC532_IOCTL_DMA: NO MEMORY\n");
        retval = -EFAULT;
        return retval;
    }


    if (mutex_lock_interruptible(&dev->dev_mut))
    return -ERESTARTSYS;
    
    switch (cmd) {
        case PCIEDEV_I2C_READ:
            //printk(KERN_ALERT "TAMC532_IOCTL_DMA: TAMC532_I2C_RW\n");
            retval = 0;
            if (copy_from_user(&i2c_data, (device_i2c_rw*)arg, (size_t)i2c_buf_size)) {
                retval = -EFAULT;
                mutex_unlock(&dev->dev_mut);
                return retval;
            }
            tamcsts         =  tamc532_i2c_read(dev, &i2c_data);
            smp_rmb();
            if(tamcsts){
                retval = -EFAULT;
                mutex_unlock(&dev->dev_mut);
                return retval;
            }
            if (copy_to_user((device_i2c_rw*)arg, &i2c_data, (size_t)i2c_buf_size)) {
                retval = -EFAULT;
                mutex_unlock(&dev->dev_mut);
                return retval;
            }
            break;
        case PCIEDEV_I2C_WRITE:
            //printk(KERN_ALERT "TAMC532_IOCTL_DMA: TAMC532_I2C_RW\n");
             retval = 0;
            if (copy_from_user(&i2c_data, (device_i2c_rw*)arg, (size_t)i2c_buf_size)) {
                retval = -EFAULT;
                mutex_unlock(&dev->dev_mut);
                return retval;
            }
                tamcsts         =  tamc532_i2c_write(dev, &i2c_data);
                if(tamcsts){
                    retval = -EFAULT;
                    mutex_unlock(&dev->dev_mut);
                    return retval;
                }
            if (copy_to_user((device_i2c_rw*)arg, &i2c_data, (size_t)i2c_buf_size)) {
                retval = -EFAULT;
                mutex_unlock(&dev->dev_mut);
                return retval;
            }
            break;
        case TAMC532_ARM_CSPT:
            //printk(KERN_ALERT "TAMC532_IOCTL_DMA: TAMC532_ARM_CSPT\n");
            retval = 0;
            if (copy_from_user(&io_data, (device_ioctrl_data*)arg, (size_t)io_data_size)) {
                retval = -EFAULT;
                mutex_unlock(&dev->dev_mut);
                return retval;
            }
            tmp_data_32       = ioread32(address + APPLICATION_CONTROL_REG);
            smp_rmb();
            tmp_data_32 = TAMC532_SWAPL(tmp_data_32);
            switch(io_data.cmd){
                case 0:
                    switch (io_data.offset){
                        case TAMC532_CSPTA:
                            if(io_data.data){
                                tmp_data_32       |= TAMC532_CSPTA;
                                iowrite32(TAMC532_SWAPL(tmp_data_32), (address + APPLICATION_CONTROL_REG));
                                smp_wmb();
                            }else{
                                tmp_data_32       &= ~TAMC532_CSPTA;
                                iowrite32(TAMC532_SWAPL(tmp_data_32), (address + APPLICATION_CONTROL_REG));
                                smp_wmb();
                            }
                            break;
                        case TAMC532_CSPTB:
                            if(io_data.data){
                                tmp_data_32       |= TAMC532_CSPTB;
                                iowrite32(TAMC532_SWAPL(tmp_data_32), (address + APPLICATION_CONTROL_REG));
                                smp_wmb();
                            }else{
                                tmp_data_32       &= ~TAMC532_CSPTB;
                                iowrite32(TAMC532_SWAPL(tmp_data_32), (address + APPLICATION_CONTROL_REG));
                                smp_wmb();
                            }
                            break;
                        case TAMC532_CSPTC:
                            if(io_data.data){
                                tmp_data_32       |= TAMC532_CSPTC;
                                iowrite32(TAMC532_SWAPL(tmp_data_32), (address + APPLICATION_CONTROL_REG));
                                smp_wmb();
                            }else{
                                tmp_data_32       &= ~TAMC532_CSPTC;
                                iowrite32(TAMC532_SWAPL(tmp_data_32), (address + APPLICATION_CONTROL_REG));
                                smp_wmb();
                            }
                            break;
                        case TAMC532_CSPTD:
                            if(io_data.data){
                                tmp_data_32       |= TAMC532_CSPTD;
                                iowrite32(TAMC532_SWAPL(tmp_data_32), (address + APPLICATION_CONTROL_REG));
                                smp_wmb();
                            }else{
                                tmp_data_32       &= ~TAMC532_CSPTD;
                                iowrite32(TAMC532_SWAPL(tmp_data_32), (address + APPLICATION_CONTROL_REG));
                                smp_wmb();
                            }
                            break;
                        case TAMC532_CSPTALL:
                            if(io_data.data){
                                iowrite32(TAMC532_SWAPL(TAMC532_CSPTALL), (address + APPLICATION_CONTROL_REG));
                                smp_wmb();
                            }else{
                                iowrite32(0, (address + APPLICATION_CONTROL_REG));
                                smp_wmb();
                            }
                            break;
                        default:
                            retval = -EFAULT;
                            mutex_unlock(&dev->dev_mut);
                            return retval;
                    }
                    break;
                case 1:
                    tmp_data_32  &= io_data.offset;
                    if(io_data.offset == TAMC532_CSPTALL){
                        io_data.data   = tmp_data_32;
                    }else{
                        io_data.data   = 0;
                        if(tmp_data_32) io_data.data   = 1;
                    }
                    break;
                default:
                    retval = -EFAULT;
                    mutex_unlock(&dev->dev_mut);
                    return retval;
            }
             if (copy_to_user((device_ioctrl_data*)arg, &io_data, (size_t)io_data_size)) {
                retval = -EFAULT;
                mutex_unlock(&dev->dev_mut);
                return retval;
            }
            break;
         case TAMC532_ENDIAN:
            //printk(KERN_ALERT "TAMC532_IOCTL_DMA: TAMC532_ENDIAN\n");
            retval = 0;
            if (copy_from_user(&io_data, (device_ioctrl_data*)arg, (size_t)io_data_size)) {
                retval = -EFAULT;
                mutex_unlock(&dev->dev_mut);
                return retval;
            }
            switch (io_data.offset){
                case TAMC532_CSPTA:
                    tmp_offset = CSPTA_CONTROL_REG;
                    break;
                case TAMC532_CSPTB:
                    tmp_offset = CSPTB_CONTROL_REG;
                    break;
                case TAMC532_CSPTC:
                    tmp_offset = CSPTC_CONTROL_REG;
                    break;
                case TAMC532_CSPTD:
                    tmp_offset = CSPTD_CONTROL_REG;
                    break;
                 case TAMC532_CSPTALL:
                    tmp_data = 0;
                    for(i = 0; i < 4; i++){
                        tmp_offset = CSPTA_CONTROL_REG + i * 0x10;
                        tmp_data_32       = ioread32(address + tmp_offset);
                        smp_rmb();
                        tmp_data_32 = TAMC532_SWAPL(tmp_data_32);
                        if(io_data.cmd){
                            tmp_data_32  = tmp_data_32 >> 16 & 0x1;
                            tmp_data        += (tmp_data_32 << i);
                        }else{
                            if(io_data.data){
                                tmp_data_32  |= (0x1 << 16);
                                iowrite32(TAMC532_SWAPL(tmp_data_32), (address + tmp_offset));
                                smp_wmb();
                            }else{
                                tmp_data_32  &=~ (0x1 << 16);
                                iowrite32(TAMC532_SWAPL(tmp_data_32), (address + tmp_offset));
                                smp_wmb();
                                tmp_data        += (0x1 << i);
                            }
                        }
                    }
                    io_data.data = tmp_data;
                    if (copy_to_user((device_ioctrl_data*)arg, &io_data, (size_t)io_data_size)) {
                        retval = -EFAULT;
                        mutex_unlock(&dev->dev_mut);
                        return retval;
                    }
                    mutex_unlock(&dev->dev_mut);
                    return retval;
                default:
                    retval = -EFAULT;
                    mutex_unlock(&dev->dev_mut);
                    return retval;
            }
            
            tmp_data_32       = ioread32(address + tmp_offset);
            smp_rmb();
            tmp_data_32 = TAMC532_SWAPL(tmp_data_32);
            if(io_data.cmd){
                io_data.data  = tmp_data_32 >> 16 & 0x1;
            }else{
                if(io_data.data){
                    tmp_data_32  |= (0x1 << 16);
                    iowrite32(TAMC532_SWAPL(tmp_data_32), (address + tmp_offset));
                    smp_wmb();
                }else{
                    tmp_data_32  &=~ (0x1 << 16);
                    iowrite32(TAMC532_SWAPL(tmp_data_32), (address + tmp_offset));
                    smp_wmb();
                }
            }

            if ( copy_to_user((device_ioctrl_data*)arg, &io_data, (size_t)io_data_size) ) {
                retval = -EFAULT;
                mutex_unlock(&dev->dev_mut);
                return retval;
            }
            break;
        case TAMC532_TRG_POLARITY:
            //printk(KERN_ALERT "TAMC532_IOCTL_DMA: TAMC532_TRG_POLARITY\n");
            retval = 0;
            if (copy_from_user(&io_data, (device_ioctrl_data*)arg, (size_t)io_data_size)) {
                retval = -EFAULT;
                mutex_unlock(&dev->dev_mut);
                return retval;
            }
            switch (io_data.offset){
                case TAMC532_CSPTA:
                    tmp_offset = CSPTA_CONTROL_REG;
                    break;
                case TAMC532_CSPTB:
                    tmp_offset = CSPTB_CONTROL_REG;
                    break;
                case TAMC532_CSPTC:
                    tmp_offset = CSPTC_CONTROL_REG;
                    break;
                case TAMC532_CSPTD:
                    tmp_offset = CSPTD_CONTROL_REG;
                    break;
                 case TAMC532_CSPTALL:
                    tmp_data = 0;
                    for(i = 0; i < 4; i++){
                        tmp_offset = CSPTA_CONTROL_REG + i * 0x10;
                        tmp_data_32       = ioread32(address + tmp_offset);
                        smp_rmb();
                        tmp_data_32 = TAMC532_SWAPL(tmp_data_32);
                        if(io_data.cmd){
                            tmp_data_32  = tmp_data_32 >> 12 & 0x1;
                            tmp_data        += (tmp_data_32 << i);
                        }else{
                            if(io_data.data){
                                tmp_data_32  |= (0x1 << 12);
                                iowrite32(TAMC532_SWAPL(tmp_data_32), (address + tmp_offset));
                                smp_wmb();
                            }else{
                                tmp_data_32  &=~ (0x1 << 12);
                                iowrite32(TAMC532_SWAPL(tmp_data_32), (address + tmp_offset));
                                smp_wmb();
                                tmp_data        += (0x1 << i);
                            }
                        }
                    }
                    io_data.data = tmp_data;
                    if (copy_to_user((device_ioctrl_data*)arg, &io_data, (size_t)io_data_size)) {
                        retval = -EFAULT;
                        mutex_unlock(&dev->dev_mut);
                        return retval;
                    }
                    mutex_unlock(&dev->dev_mut);
                    return retval;
                default:
                    retval = -EFAULT;
                    mutex_unlock(&dev->dev_mut);
                    return retval;
            }
            
            tmp_data_32       = ioread32(address + tmp_offset);
            smp_rmb();
            tmp_data_32 = TAMC532_SWAPL(tmp_data_32);
            if(io_data.cmd){
                io_data.data  = tmp_data_32 >> 12 & 0x1;
            }else{
                if(io_data.data){
                    tmp_data_32  |= (0x1 << 12);
                    iowrite32(TAMC532_SWAPL(tmp_data_32), (address + tmp_offset));
                    smp_wmb();
                }else{
                    tmp_data_32  &=~ (0x1 << 12);
                    iowrite32(TAMC532_SWAPL(tmp_data_32), (address + tmp_offset));
                    smp_wmb();
                }
            }

            if ( copy_to_user((device_ioctrl_data*)arg, &io_data, (size_t)io_data_size) ) {
                retval = -EFAULT;
                mutex_unlock(&dev->dev_mut);
                return retval;
            }
            break;
        case TAMC532_TRG_SOURCE:
            //printk(KERN_ALERT "TAMC532_IOCTL_DMA: TAMC532_TRG_SOURCE\n");
            retval = 0;
            if (copy_from_user(&io_data, (device_ioctrl_data*)arg, (size_t)io_data_size)) {
                retval = -EFAULT;
                mutex_unlock(&dev->dev_mut);
                return retval;
            }
            switch (io_data.offset){
                case TAMC532_CSPTA:
                    tmp_offset = CSPTA_CONTROL_REG;
                    break;
                case TAMC532_CSPTB:
                    tmp_offset = CSPTB_CONTROL_REG;
                    break;
                case TAMC532_CSPTC:
                    tmp_offset = CSPTC_CONTROL_REG;
                    break;
                case TAMC532_CSPTD:
                    tmp_offset = CSPTD_CONTROL_REG;
                    break;
                 case TAMC532_CSPTALL:
                    tmp_data = 0;
                    for(i = 0; i < 4; i++){
                        tmp_offset = CSPTA_CONTROL_REG + i * 0x10;
                        tmp_data_32       = ioread32(address + tmp_offset);
                        smp_rmb();
                        tmp_data_32 = TAMC532_SWAPL(tmp_data_32);
                        if(io_data.cmd){
                            tmp_data_32  = tmp_data_32 >> 8 & 0xF;
                            tmp_data        += (tmp_data_32 << i*4);
                        }else{
                            tmp_data_32  &= 0xFFFFF0FF;
                            tmp_data_32  |= ((io_data.data & 0xF) << 8);
                            iowrite32(TAMC532_SWAPL(tmp_data_32), (address + tmp_offset));
                            smp_wmb();
                            mutex_unlock(&dev->dev_mut);
                            return retval;
                        }
                    }
                    io_data.data = tmp_data;
                    if (copy_to_user((device_ioctrl_data*)arg, &io_data, (size_t)io_data_size)) {
                        retval = -EFAULT;
                        mutex_unlock(&dev->dev_mut);
                        return retval;
                    }
                    mutex_unlock(&dev->dev_mut);
                    return retval;
                default:
                    retval = -EFAULT;
                    mutex_unlock(&dev->dev_mut);
                    return retval;
            }
            
            tmp_data_32       = ioread32(address + tmp_offset);
            smp_rmb();
            tmp_data_32 = TAMC532_SWAPL(tmp_data_32);
            if(io_data.cmd){
                io_data.data  = tmp_data_32 >> 8 & 0xF;
            }else{
                tmp_data_32  &=0xFFFFF0FF;
                tmp_data_32  |= ((io_data.data & 0xF) << 8);
                iowrite32(TAMC532_SWAPL(tmp_data_32), (address + tmp_offset));
                smp_wmb();
            }

            if ( copy_to_user((device_ioctrl_data*)arg, &io_data, (size_t)io_data_size) ) {
                retval = -EFAULT;
                mutex_unlock(&dev->dev_mut);
                return retval;
            }
            break;
        case TAMC532_RAISE_SW_TRG:
            printk(KERN_ALERT "TAMC532_IOCTL_DMA: TAMC532_RAISE_SW_TRG\n");
            retval = 0;
            if (copy_from_user(&io_data, (device_ioctrl_data*)arg, (size_t)io_data_size)) {
                retval = -EFAULT;
                mutex_unlock(&dev->dev_mut);
                return retval;
            }
            tmp_data_32       = ioread32(address + APPLICATION_COMMAND_REG);
            smp_rmb();
            tmp_data_32 = TAMC532_SWAPL(tmp_data_32);
            switch (io_data.offset){
                case TAMC532_CSPTA:
                    tmp_data_32       |= 0x1;
                    break;
                case TAMC532_CSPTB:
                    tmp_data_32       |= 0x2;
                    break;
                case TAMC532_CSPTC:
                    tmp_data_32       |= 0x4;
                    break;
                case TAMC532_CSPTD:
                    tmp_data_32       |= 0x8;
                    break;
                case TAMC532_CSPTALL:
                    tmp_data_32       |= 0xF;
                    break;
                default:
                    retval = -EFAULT;
                    mutex_unlock(&dev->dev_mut);
                    return retval;
            }
            iowrite32(TAMC532_SWAPL(tmp_data_32), (address + APPLICATION_COMMAND_REG));
            smp_wmb();
            break;
        case TAMC532_CSPT_RESET:
            printk(KERN_ALERT "TAMC532_IOCTL_DMA: TAMC532_CSPT_RESET\n");
            retval = 0;
            if (copy_from_user(&io_data, (device_ioctrl_data*)arg, (size_t)io_data_size)) {
                retval = -EFAULT;
                mutex_unlock(&dev->dev_mut);
                return retval;
            }
            tmp_data_32       = ioread32(address + APPLICATION_COMMAND_REG);
            smp_rmb();
            tmp_data_32 = TAMC532_SWAPL(tmp_data_32);
            switch (io_data.offset){
                case TAMC532_CSPTA:
                    tmp_data_32       |= 0x10;
                    break;
                case TAMC532_CSPTB:
                    tmp_data_32       |= 0x20;
                    break;
                case TAMC532_CSPTC:
                    tmp_data_32       |= 0x40;
                    break;
                case TAMC532_CSPTD:
                    tmp_data_32       |= 0x80;
                    break;
                case TAMC532_CSPTALL:
                    tmp_data_32       |= 0xF0;
                    break;
                default:
                    retval = -EFAULT;
                    mutex_unlock(&dev->dev_mut);
                    return retval;
            }
            iowrite32(TAMC532_SWAPL(tmp_data_32), (address + APPLICATION_COMMAND_REG));
            smp_wmb();
            break;
        case PCIEDEV_READ_DMA:
        case TAMC532_SET_DMA:
            retval = 0;
            if (copy_from_user(&dma_data, (device_ioctrl_dma*)arg, (size_t)io_dma_size)) {
                retval = -EFAULT;
                mutex_unlock(&dev->dev_mut);
                printk (KERN_ALERT "PCIEDEV_SSET_DMA: ERROR COPY FROM USER\n");
                return retval;
            }
            tmp_dma_offset        = dma_data.dma_offset;
            tmp_dma_size           = dma_data.dma_size;
            length                       = tmp_dma_size;
            //printk (KERN_ALERT "\n\n\nPCIEDEV_SSET_DMA:###### SIZE %i OFFSET %i \n",tmp_dma_size, tmp_dma_offset);
            if(tmp_dma_size <= 0){
                mutex_unlock(&dev->dev_mut);
                printk (KERN_ALERT "PCIEDEV_SSET_DMA: ERROR SIZE %i DMAC %i \n",tmp_dma_size, tmp_dma_offset);
                return -EFAULT;
            }
            if(tmp_dma_offset < 0){
                mutex_unlock(&dev->dev_mut);
                printk (KERN_ALERT "PCIEDEV_SSET_DMA: ERROR OFFSET DMAC %i \n",tmp_dma_offset);
                return -EFAULT;
            }
            if(tmp_dma_offset > 3){
                mutex_unlock(&dev->dev_mut);
                printk (KERN_ALERT "PCIEDEV_SSET_DMA: ERROR OFFSET DMAC %i \n",tmp_dma_offset);
                return -EFAULT;
            }
            tmp_dma_num      = tamc532dev->dma_page_num[tmp_dma_offset];
            //printk (KERN_ALERT "PCIEDEV_SSET_DMA: SIZE %i DMAC %i \n",tmp_dma_size, tmp_dma_offset);
            /**COPY DATA TO USER**/
            if((tamc532dev->dmac_enable[tmp_dma_offset])){
                /* disable the DMA controller */
                iowrite32( 0, ( address + (DMAC0_CONTROL_REG + tmp_dma_offset*0x10) ) );
                smp_wmb();
                iowrite32( 0, ( address + DMAC0_BASE_DESCR_ADDRESS_REG + tmp_dma_offset*0x10));
                smp_wmb();
                udelay(2);
                if(!(tamc532dev->dmac_done[tmp_dma_offset] )){
                    for(dma_wait_count = 0; dma_wait_count < 20; dma_wait_count++){
                        if(tamc532dev->dmac_done[tmp_dma_offset] ) break;
                         if(dma_wait_count == 19){ 
			 //printk (KERN_ALERT "PCIEDEV_SET_DMA: DMA WAIT ERROR\n");
                            retval = -EAGAIN;
                        }
                    }
                }
                tmp_dma_count_offset    = 0;
                for(tmp_dma_count    = (tmp_dma_num -1) ; tmp_dma_count >= 0; tmp_dma_count--){
                    tmp_dma_size = tamc532dev->dev_dma_size[tmp_dma_offset][tmp_dma_count] ;
                    //printk (KERN_ALERT "@@@@@@@@@@@@@@@PCIEDEV_SET_DMA: DMA_COUNT %i SIZE %i DMAC %i OFFSET %i\n",tmp_dma_count, tmp_dma_size, tmp_dma_offset, tmp_dma_count_offset);
                    pci_unmap_single(pdev, tamc532dev->pTmpDescHandle[tmp_dma_offset][tmp_dma_count], tamc532dev->dev_dma_desc_size[tmp_dma_offset][tmp_dma_count], PCI_DMA_TODEVICE);
                    free_pages((ulong)tamc532dev->pDescBuf[tmp_dma_offset][tmp_dma_count], (ulong)tamc532dev->dma_desc_order[tmp_dma_offset][tmp_dma_count]);
                    tamc532dev->pTmpDescHandle[tmp_dma_offset][tmp_dma_count] = 0;
                    tamc532dev->pDescBuf[tmp_dma_offset][tmp_dma_count]  = 0;
                    pci_unmap_single(pdev, tamc532dev->pTmpDmaHandle[tmp_dma_offset][tmp_dma_count], tamc532dev->dev_dma_trans_size[tmp_dma_offset][tmp_dma_count], PCI_DMA_FROMDEVICE);
                    if (copy_to_user ( ((void *)arg + tmp_dma_count_offset) , tamc532dev->pWriteBuf[tmp_dma_offset][tmp_dma_count] , tmp_dma_size) ) {
                        printk (KERN_ALERT "PCIEDEV_SSET_DMA: ERROR COPY TO USER DMAC %i \n",tmp_dma_offset);
                        retval = -EFAULT;
                    }
                    free_pages((ulong)tamc532dev->pWriteBuf[tmp_dma_offset][tmp_dma_count], (ulong)tamc532dev->dma_order[tmp_dma_offset][tmp_dma_count]);
                    tamc532dev->pTmpDmaHandle[tmp_dma_offset][tmp_dma_count] = 0;
                    tamc532dev->pWriteBuf[tmp_dma_offset][tmp_dma_count] = 0;
                    tamc532dev->dev_dma_size[tmp_dma_offset][tmp_dma_count]           = 0;
                    tamc532dev->dma_order[tmp_dma_offset][tmp_dma_count]               = 0;
                    tamc532dev->dev_dma_trans_size[tmp_dma_offset][tmp_dma_count] =  0;
                    tamc532dev->dev_dma_desc_size[tmp_dma_offset][tmp_dma_count]  = 0;
                    tmp_dma_count_offset    += tmp_dma_size;
                }
                tamc532dev->dmac_enable[tmp_dma_offset] = 0;
            }
            
            /**SET NEW DMA**/
			
	   tmp_free_pages        = nr_free_pages();
	   tmp_free_pages        = tmp_free_pages << (PAGE_SHIFT-10);
	   tmp_free_pages        = tmp_free_pages/2;
	   /*
                if(tmp_dma_size > tmp_free_pages){
                    dma_trans_cnt          = tmp_dma_size/tmp_free_pages;
                    dma_trans_rest         = tmp_dma_size - dma_trans_cnt*tmp_free_pages;
                    tmp_dma_size           = tmp_free_pages;
                    tmp_dma_trns_size   = tmp_free_pages;
                }
          */
	   
	   
	               
	   tmp_dma_size         = dma_data.dma_size;
	   //printk(KERN_INFO "Memory: %luk  Requst : %luk\n",  nr_free_pages() << (PAGE_SHIFT-10), tmp_dma_size/1000);
	   //printk (KERN_ALERT "PCIEDEV_SSET_DMA:  FREE PAGES SIZE %i: PAGES %i \n",  tmp_dma_size, tmp_free_pages);
	   
	   if(tmp_dma_size > tmp_free_pages*1000){
		 mutex_unlock(&dev->dev_mut);
                   printk (KERN_ALERT "PCIEDEV_SSET_DMA: ERROR NO FREE PAGES SIZE %i: PAGES %i \n",  tmp_dma_size, tmp_free_pages*1000);
                   return -ENOMEM;
	  }	   
	   
            tmp_dma_num        = tmp_dma_size/(TAMC532_DMAC_MAX_SIZE * 4);
            tmp_dma_size_rest  = tmp_dma_size%(TAMC532_DMAC_MAX_SIZE * 4);
            //printk (KERN_ALERT "*************PCIEDEV_SSET_DMA: DMA_NUM %i  DMA_SIZE /REST %i/%i \n",tmp_dma_num, tmp_dma_size, tmp_dma_size_rest);
            if(!tmp_dma_num){
                tmp_dma_num = 1;
                tmp_dma_size  = tmp_dma_size_rest;
            }else{
                tmp_dma_size  = TAMC532_DMAC_MAX_SIZE*4;
                if(tmp_dma_size_rest){
                    tmp_dma_num += 1;
                }else{
                    tmp_dma_size_rest = tmp_dma_size;
                }
            }
            //printk (KERN_ALERT "PCIEDEV_SSET_DMA: DMA_NUM %i  DMA_SIZE /REST %i/%i \n",tmp_dma_num, tmp_dma_size, tmp_dma_size_rest);
			
            tamc532dev->dma_page_num[tmp_dma_offset] = tmp_dma_num;
            
            for(tmp_dma_count    = 0; tmp_dma_count < tmp_dma_num; tmp_dma_count++){
	       tmp_dma_count_size = tmp_dma_size;
                if(tmp_dma_count == 0 ) tmp_dma_size = tmp_dma_size_rest;
                tamc532dev->dev_dma_size[tmp_dma_offset][tmp_dma_count] = tmp_dma_size;
                if(tmp_dma_size <= 0){
                    mutex_unlock(&dev->dev_mut);
                    printk (KERN_ALERT "PCIEDEV_SSET_DMA: ERROR DMA SIZE DMAC %i \n",tmp_dma_offset);
                    return -EFAULT;
                }
				
	       tmp_dma_buf_size    = tmp_dma_size;
                tmp_dma_trns_size   = tmp_dma_buf_size;
                
                if((tmp_dma_buf_size%PCIEDEV_DMA_SYZE)){
                    tmp_dma_buf_size    = tmp_dma_size + (tmp_dma_size%PCIEDEV_DMA_SYZE);
                }
                tmp_dma_trns_size = tmp_dma_buf_size;
                dma_trans_cnt   = 0;
                dma_trans_rest  = tmp_dma_size; 
                tmp_order = get_order(tmp_dma_trns_size);
                tamc532dev->dma_order[tmp_dma_offset][tmp_dma_count]               = tmp_order;
                tamc532dev->dev_dma_trans_size[tmp_dma_offset][tmp_dma_count]  =  tmp_dma_trns_size;
				
	       // printk (KERN_ALERT "PCIEDEV_SSET_DMA: GET ORDER DMA_NUM %i  ORDER_NUM %i DMA_SIZE /REST %i/%i \n",
		//	 tmp_dma_num, tmp_order, tmp_dma_size, tmp_dma_size_rest);
	        //printk (KERN_ALERT "PCIEDEV_SSET_DMA: FREE_PAGES %i \n",tmp_free_pages);
	        
				
                if (!tamc532dev->pWriteBuf[tmp_dma_offset][tmp_dma_count]){
                    tamc532dev->pWriteBuf[tmp_dma_offset][tmp_dma_count] = (void *)__get_free_pages(GFP_KERNEL | __GFP_DMA, tmp_order);
                }
                if (!tamc532dev->pWriteBuf[tmp_dma_offset][tmp_dma_count]){
                    printk (KERN_ALERT "PCIEDEV_SET_DMA: NO MEMORY FOR SIZE,  %X\n",tmp_dma_size);
                    tamc532dev->dev_dma_size[tmp_dma_offset][tmp_dma_count]           = 0;
                    tamc532dev->dma_order[tmp_dma_offset][tmp_dma_count]               = 0;
                    tamc532dev->dev_dma_trans_size[tmp_dma_offset][tmp_dma_count] =  0;
                    mutex_unlock(&dev->dev_mut);
                    return -EFAULT;
                }

				
            //***************set DMA DESCRIPTORS*******************				
            tmp_dma_desc_size = sizeof(tamc532_dma_desc);
            tmp_order = get_order(tmp_dma_desc_size);
            tamc532dev->dma_desc_order[tmp_dma_offset][tmp_dma_count] = tmp_order;
            tamc532dev->dev_dma_desc_size[tmp_dma_offset][tmp_dma_count] = tmp_dma_desc_size;
            if (!tamc532dev->pDescBuf[tmp_dma_offset][tmp_dma_count]){
                tamc532dev->pDescBuf[tmp_dma_offset][tmp_dma_count] = (void *)__get_free_pages(GFP_KERNEL | __GFP_DMA, tmp_order);
            }    
            if (!tamc532dev->pDescBuf[tmp_dma_offset][tmp_dma_count]){
                printk (KERN_ALERT "PCIEDEV_SET_DMA: NO MEMORY FOR DMA DESC SIZE,  %X\n",tmp_dma_size);
                free_pages((ulong)tamc532dev->pWriteBuf[tmp_dma_offset][tmp_dma_count], (ulong)tamc532dev->dma_order[tmp_dma_offset][tmp_dma_count]);
                tamc532dev->dev_dma_size[tmp_dma_offset][tmp_dma_count]           = 0;
                tamc532dev->dma_order[tmp_dma_offset][tmp_dma_count]               = 0;
                tamc532dev->dev_dma_trans_size[tmp_dma_offset][tmp_dma_count] =  0;
                tamc532dev->dev_dma_desc_size[tmp_dma_offset][tmp_dma_count]  = 0;
                tamc532dev->pWriteBuf[tmp_dma_offset][tmp_dma_count] = 0;
                mutex_unlock(&dev->dev_mut);
                return -EFAULT;
            }
            //printk (KERN_ALERT "PCIEDEV_SSET_DMA: CURRENT DMAC %i DMA_NUM %i  DMA_SIZE /REST %i/%i DMA TRANS SIZE %i \n",tmp_dma_count, tmp_dma_num, tmp_dma_size, tmp_dma_size_rest, tmp_dma_trns_size);
            if(!(tamc532dev->pTmpDmaHandle[tmp_dma_offset][tmp_dma_count] ))
                tamc532dev->pTmpDmaHandle[tmp_dma_offset][tmp_dma_count]      = pci_map_single(pdev, tamc532dev->pWriteBuf[tmp_dma_offset][tmp_dma_count], tmp_dma_trns_size, PCI_DMA_FROMDEVICE);
            tmp_data_32         = ((tmp_dma_size/4) << 16);
            if(tmp_dma_count == 0)
                tmp_data_32         |= TAMC532_DESCR_HI_EOL;
            tmp_dma_desc.control     = cpu_to_be32((unsigned int)(tmp_data_32));                
            tmp_dma_desc.dataptr     = cpu_to_be32( (unsigned int)(tamc532dev->pTmpDmaHandle[tmp_dma_offset][tmp_dma_count]) );
            if(tmp_dma_count > 0){
                tmp_dma_desc.next_desc = cpu_to_be32((unsigned int)(tamc532dev->pTmpDescHandle[tmp_dma_offset][tmp_dma_count -1]));
            }
            if(tmp_dma_count == 0){
                tmp_dma_desc.next_desc = 0x00000000;
            }
            memcpy(tamc532dev->pDescBuf[tmp_dma_offset][tmp_dma_count], &tmp_dma_desc, tmp_dma_desc_size);
            if(!(tamc532dev->pTmpDescHandle[tmp_dma_offset][tmp_dma_count] ))
                tamc532dev->pTmpDescHandle[tmp_dma_offset][tmp_dma_count]      = pci_map_single(pdev, tamc532dev->pDescBuf[tmp_dma_offset][tmp_dma_count], tmp_dma_desc_size, PCI_DMA_TODEVICE);
            tmp_dma_size = tmp_dma_count_size;
        }
        
        /************ disable the DMA controller ***************/
        iowrite32( 0, ( address + (DMAC0_CONTROL_REG + tmp_dma_offset*0x10) ) );
        smp_wmb();
        iowrite32( 0, ( address + DMAC0_BASE_DESCR_ADDRESS_REG + tmp_dma_offset*0x10));
        smp_wmb();
        udelay(2);

        /* initialize registers */
        tmp_data_32       = ioread32(address + MODULE_INTERRUPT_ENABLE_REG);
        smp_rmb();
        tmp_data_32       = TAMC532_SWAPL(tmp_data_32);
        tmp_data_32       |=  TAMC532_MODULEINT_DMACx_EOL(tmp_dma_offset) ;
        iowrite32( TAMC532_SWAPL(tmp_data_32), ( address + MODULE_INTERRUPT_ENABLE_REG ));
        smp_wmb();
        udelay(2);

        tmp_data_32       = tamc532dev->pTmpDescHandle[tmp_dma_offset][tmp_dma_num -1];
        iowrite32( cpu_to_be32((unsigned int)(tmp_data_32)), ( address + DMAC0_BASE_DESCR_ADDRESS_REG + tmp_dma_offset*0x10));
        smp_wmb();
        udelay(20);
        /* Enable DMA controller. This will cause a prefetch of the descriptor list (or at least part of it) */
        do_gettimeofday(&(tamc532dev->dma_start_time));
        tamc532dev->dmac_done[tmp_dma_offset] = 0; 
        iowrite32( TAMC532_SWAPL(0x1), ( address + (DMAC0_CONTROL_REG + tmp_dma_offset*0x10)));
        smp_wmb();
        tamc532dev->dmac_enable[tmp_dma_offset] =1;
        udelay(2);
            break;
        
        case TAMC532_REM_DMA:
            retval = 0;
            
            if (copy_from_user(&dma_data, (device_ioctrl_dma*)arg, (size_t)io_dma_size)) {
                retval = -EFAULT;
                mutex_unlock(&dev->dev_mut);
                return retval;
            }
            tmp_dma_size           = dma_data.dma_size;
            tmp_dma_offset        = dma_data.dma_offset;
            length                       = tmp_dma_size;
            if(tmp_dma_offset < 0){
                mutex_unlock(&dev->dev_mut);
                return -EFAULT;
            }
            if(tmp_dma_offset > 3){
                mutex_unlock(&dev->dev_mut);
                return -EFAULT;
            }
            if(tamc532dev->dmac_enable[tmp_dma_offset]){
                tmp_dma_num      = tamc532dev->dma_page_num[tmp_dma_offset];
                //printk (KERN_ALERT "TAMC532_REM_DMA: DMAC CONTROLLER %i DMA_NUM %i\n", tmp_dma_offset, tmp_dma_num);
                iowrite32( 0, ( address + (DMAC0_CONTROL_REG + tmp_dma_offset*0x10)));
                smp_wmb();
                udelay(20);
                iowrite32( 0, ( address + DMAC0_BASE_DESCR_ADDRESS_REG + tmp_dma_offset*0x10));
                smp_wmb();
                for(tmp_dma_count    = 0; tmp_dma_count < tmp_dma_num; tmp_dma_count++){
                    if(tamc532dev->pTmpDmaHandle[tmp_dma_offset][tmp_dma_count]){
                        pci_unmap_single(pdev, tamc532dev->pTmpDmaHandle[tmp_dma_offset][tmp_dma_count], tamc532dev->dev_dma_trans_size[tmp_dma_offset][tmp_dma_count], PCI_DMA_FROMDEVICE);
                        free_pages((ulong)tamc532dev->pWriteBuf[tmp_dma_offset][tmp_dma_count], (ulong)tamc532dev->dma_order[tmp_dma_offset][tmp_dma_count]);
                        tamc532dev->pTmpDmaHandle[tmp_dma_offset][tmp_dma_count] = 0;
                        tamc532dev->pWriteBuf[tmp_dma_offset][tmp_dma_count] = 0;
                    }
                    if(tamc532dev->pTmpDescHandle[tmp_dma_offset][tmp_dma_count]){		
                        pci_unmap_single(pdev, tamc532dev->pTmpDescHandle[tmp_dma_offset][tmp_dma_count], tamc532dev->dev_dma_desc_size[tmp_dma_offset][tmp_dma_count], PCI_DMA_TODEVICE);
                        free_pages((ulong)tamc532dev->pDescBuf[tmp_dma_offset][tmp_dma_count], (ulong)tamc532dev->dma_desc_order[tmp_dma_offset][tmp_dma_count]);
                        tamc532dev->pTmpDescHandle[tmp_dma_offset][tmp_dma_count] = 0;
                        tamc532dev->pDescBuf[tmp_dma_offset][tmp_dma_count]  = 0;
                    }
                    tamc532dev->dev_dma_size[tmp_dma_offset][tmp_dma_count]           = 0;
                    tamc532dev->dev_dma_size[tmp_dma_offset][tmp_dma_count]           = 0;
                    tamc532dev->dma_order[tmp_dma_offset][tmp_dma_count]               = 0;
                    tamc532dev->dev_dma_trans_size[tmp_dma_offset][tmp_dma_count] =  0;
                    tamc532dev->dev_dma_desc_size[tmp_dma_offset][tmp_dma_count]  = 0;
                }
                tamc532dev->dmac_enable[tmp_dma_offset]            = 0;
            }
            break;
        case PCIEDEV_GET_DMA_TIME:
            retval = 0;
            tamc532dev->dma_start_time.tv_sec+= (long)dev->slot_num;
            tamc532dev->dma_stop_time.tv_sec  += (long)dev->slot_num;
            tamc532dev->dma_start_time.tv_usec+= (long)dev->brd_num;
            tamc532dev->dma_stop_time.tv_usec  += (long)dev->brd_num;
            time_data.start_time = tamc532dev->dma_start_time;
            time_data.stop_time  = tamc532dev->dma_stop_time;
            if (copy_to_user((device_ioctrl_time*)arg, &time_data, (size_t)size_time)) {
                retval = -EIO;
                mutex_unlock(&dev->dev_mut);
                return retval;
            }
            break;
         case TAMC532_IRQ_STS:
            retval = 0;
            io_data.data = tamc532dev->irqsts;
            if ( copy_to_user((device_ioctrl_data*)arg, &io_data, (size_t)io_data_size) ) {
                retval = -EFAULT;
                mutex_unlock(&dev->dev_mut);
                return retval;
            }
            break;  
        default:
            mutex_unlock(&dev->dev_mut);
            return -ENOTTY;
            break;
    }
    
    mutex_unlock(&dev->dev_mut);
    return retval;
}

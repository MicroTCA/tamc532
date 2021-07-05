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

extern struct workqueue_struct *tamc532_workqueue;

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

	device_i2c_rw			 i2c_data;
	int					i2c_buf_size;
	device_ioctrl_data		 io_data;
	int					io_data_size;
	device_ioctrl_time		time_data;
	device_ioctrl_dma		dma_data;
	device_ioctrl_stream_dma     stream_dma_data;
	int					io_stream_dma_size;
	int					size_time;
	int					io_dma_size;
	
	device_staus_registers		dev_status_reg_data;
	device_adc_data_registers	dev_adc_channel_data;
	int					size_status_reg;
	int					size_adc_data;

	u_int                  tmp_offset;
	u32                    tmp_data_32;
	u32                    tmp_data;

	ulong           value;
	u_int	    tmp_dma_size;
	//u_int	    tmp_dma_buf_size;
	u_int	    tmp_new_dma_size;
	u_int	    tmp_last_dma_size;
	u_int	    tmp_dma_count_size;
	u_int	    tmp_dma_size_rest;
	u_int	    tmp_dma_buf_size;
	u_int	    tmp_dma_trns_size;
	u_int	    tmp_dma_desc_size;
	u_int	    tmp_dma_offset;
	u_int	    tmp_dma_cmd;
	u_int	    tmp_stream_trg_num;
	 
	int tmp_pre_sample = 0;
	int tmp_post_sample = 0;
	int tmp_sample_num = 0;

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

	int              dmac_dqh_idle =0;
	int              dmac_aif_idle   = 0;


	cmd                            = *cmd_p;
	arg                              = *arg_p;
	i2c_buf_size                = sizeof(device_i2c_rw);
	io_data_size                 = sizeof(device_ioctrl_data);
	size_time                     = sizeof (device_ioctrl_time);
	io_dma_size                 = sizeof(device_ioctrl_dma);
	io_stream_dma_size                 = sizeof(device_ioctrl_stream_dma);
	
	size_status_reg = sizeof(device_staus_registers);
	size_adc_data    = sizeof(device_adc_data_registers);

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


/*
	if (mutex_lock_interruptible(&dev->dev_mut))
		return -ERESTARTSYS;
*/
	mutex_lock(&dev->dev_mut);
    
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
    
    case TAMC532_READ_DMA:   
		retval = 0;
		if (copy_from_user(&dma_data, (device_ioctrl_dma*)arg, (size_t)io_dma_size)) {
			retval = -EFAULT;
			mutex_unlock(&dev->dev_mut);
			printk (KERN_ALERT "TAMC532_READ_DMA: ERROR COPY FROM USER\n");
			return retval;
		}
		tamc532dev->dev_dma_size_change = 0;
		tmp_dma_offset           = dma_data.dma_offset;
		tmp_dma_size              = dma_data.dma_size;
		tmp_dma_cmd             = dma_data.dma_cmd;
		length                            = tmp_dma_size;
		tmp_new_dma_size      = tmp_dma_size;
		
		
		//printk (KERN_ALERT "******TAMC532_READ_DMA: CMD_COMMAN %i\n", tmp_dma_cmd);
		
		if(tmp_dma_offset < 0){
			mutex_unlock(&dev->dev_mut);
			printk (KERN_ALERT "TAMC532_READ_DMA: ERROR OFFSET DMAC %i \n",tmp_dma_offset);
			return -EFAULT;
		}
		if(tmp_dma_offset > 3){
			mutex_unlock(&dev->dev_mut);
			printk (KERN_ALERT "TAMC532_READ_DMA: ERROR OFFSET DMAC %i \n",tmp_dma_offset);
			return -EFAULT;
		}
		
		tamc532dev->is_strm_dma = 0;
		if(tmp_dma_cmd){
			tamc532dev->is_strm_dma = 1;
		}
		
		tmp_last_dma_size       =  tamc532dev->dev_dma_last_size;
		tamc532dev->dev_dma_last_size = tmp_dma_size;
		if(tmp_new_dma_size != tmp_last_dma_size){
			 tamc532dev->dev_dma_size_change = 1;
		}
		
		tmp_dma_num      = tamc532dev->dma_page_num[tmp_dma_offset];
		
		//**COPY DATA TO USER**/
		if((tamc532dev->dmac_enable[tmp_dma_offset])){
			tmp_data_32         = ioread32(address + (DMAC0_STATUS_REG + tmp_dma_offset*0x10));
			smp_rmb();
			tmp_data_32       =TAMC532_SWAPL(tmp_data_32);
			dmac_dqh_idle     = (tmp_data_32>>16) & 0xFFFF;
			dmac_aif_idle       =  (tmp_data_32>>8) & 0xFF;
			
			//printk (KERN_ALERT "TAMC532_READ_DMA: DMAC_STATUS %X OFFSET %i\n", tmp_data_32, tmp_dma_offset);

			tmp_data = 0;
			tmp_data = dmac_dqh_idle + dmac_aif_idle;
			if(!tmp_data) tamc532dev->dmac_done[tmp_dma_offset] = 1;
			
			if(!(tamc532dev->dmac_done[tmp_dma_offset] )){
				//printk (KERN_ALERT "TAMC532_READ_DMA: DMA NOT DONE OFFSET %i\n", tmp_dma_offset);
				for(dma_wait_count = 0; dma_wait_count < 5; dma_wait_count++){
					//printk (KERN_ALERT "******TAMC532_READ_DMA: DMA NOT DONE OFFSET %i POLLING NUM %i\n", tmp_dma_offset, dma_wait_count);
					if(tamc532dev->dmac_done[tmp_dma_offset] ) break;
					
					tmp_data_32       = ioread32(address + (DMAC0_STATUS_REG + tmp_dma_offset*0x10));
					smp_rmb();
					tmp_data_32       =TAMC532_SWAPL(tmp_data_32);
					dmac_dqh_idle = (tmp_data_32>>16) & 0xFFFF;
					dmac_aif_idle   =  (tmp_data_32>>8) & 0xFF;
					tmp_data = 0;
					tmp_data = dmac_dqh_idle + dmac_aif_idle;
					
					//printk (KERN_ALERT "******TAMC532_READ_DMA: DMA NOT DONE OFFSET %i POLLING NUM %i STAUS %X DONE %X/%x/%x\n", 
					//		tmp_dma_offset, dma_wait_count, tmp_data_32, tmp_data, dmac_dqh_idle, dmac_aif_idle);
					
					if(!tmp_data) tamc532dev->dmac_done[tmp_dma_offset] = 1;
					udelay(1000);
				}
			}
			if(!(tamc532dev->dmac_done[tmp_dma_offset] )){
				//printk (KERN_ALERT "******TAMC532_READ_DMA: DMA NOT DONE OFFSET %i \n", tmp_dma_offset);
				retval = -EAGAIN;
			}
			
			//* disable the DMA controller */
			iowrite32( 0, ( address + (DMAC0_CONTROL_REG + tmp_dma_offset*0x10) ) );
			smp_wmb();
			iowrite32( 0, ( address + DMAC0_BASE_DESCR_ADDRESS_REG + tmp_dma_offset*0x10));
			smp_wmb();
			udelay(2);
			tmp_dma_count_offset    = 0;
						
			for(tmp_dma_count    = (tmp_dma_num -1) ; tmp_dma_count >= 0; tmp_dma_count--){
				tmp_dma_size = tamc532dev->dev_dma_size[tmp_dma_offset][tmp_dma_count] ;
				pci_unmap_single(pdev, tamc532dev->pTmpDescHandle[tmp_dma_offset][tmp_dma_count], TAMC532_DESC_SIZE, PCI_DMA_TODEVICE);
				tamc532dev->pTmpDescHandle[tmp_dma_offset][tmp_dma_count] = 0;
								
				pci_unmap_single(pdev, tamc532dev->pTmpDmaHandle[tmp_dma_offset][tmp_dma_count], tamc532dev->dev_dma_trans_size[tmp_dma_offset][tmp_dma_count], PCI_DMA_FROMDEVICE);
				if (copy_to_user ( ((void *)arg + tmp_dma_count_offset) , tamc532dev->pWriteBuf[tmp_dma_offset][tmp_dma_count] , tmp_dma_size) ) {
					printk (KERN_ALERT "TAMC532_READ_DMA: ERROR COPY TO USER DMAC %i \n",tmp_dma_offset);
					retval = -EFAULT;
				}
				
				if( tamc532dev->dev_dma_size_change){
					if (tamc532dev->pWriteBuf[tmp_dma_offset][tmp_dma_count]){
						free_pages((ulong)tamc532dev->pWriteBuf[tmp_dma_offset][tmp_dma_count], (ulong)tamc532dev->dma_order[tmp_dma_offset][tmp_dma_count]);
					}
					tamc532dev->pWriteBuf[tmp_dma_offset][tmp_dma_count]                  = 0;
				}	
			    
				tamc532dev->pTmpDmaHandle[tmp_dma_offset][tmp_dma_count]      = 0;
				tamc532dev->dev_dma_size[tmp_dma_offset][tmp_dma_count]            = 0;
				tamc532dev->dma_order[tmp_dma_offset][tmp_dma_count]                 = 0;
				tamc532dev->dev_dma_trans_size[tmp_dma_offset][tmp_dma_count] =  0;
				tamc532dev->dev_dma_desc_size[tmp_dma_offset][tmp_dma_count]  = 0;
				
				tmp_dma_count_offset    += tmp_dma_size;
			}
			tamc532dev->dmac_enable[tmp_dma_offset] = 0;
			tamc532dev->dmac_dma_int_enabled[tmp_dma_offset]  = 0;
		}
		tamc532dev->irq_num[tmp_dma_offset] = 0;
		   
		//**SET NEW DMA**/
		tmp_free_pages        = nr_free_pages();
		tmp_free_pages        = tmp_free_pages << (PAGE_SHIFT-10);
		tmp_free_pages        = tmp_free_pages/2;
		//printk (KERN_ALERT "TAMC532_READ_DMA: FREE PAGES %i \n",tmp_free_pages);
		
		tmp_dma_size         = tmp_new_dma_size;
		if(tmp_dma_size <= 0){
			mutex_unlock(&dev->dev_mut);
			printk (KERN_ALERT "TAMC532_READ_DMA: ERROR SIZE %i DMAC %i \n",tmp_dma_size, tmp_dma_offset);
			return -EFAULT;
		}

		if(tmp_dma_size > tmp_free_pages*1000){
			mutex_unlock(&dev->dev_mut);
			printk (KERN_ALERT "TAMC532_READ_DMA: ERROR NO FREE PAGES SIZE %i: PAGES %i \n",  tmp_dma_size, tmp_free_pages*1000);
			return -ENOMEM;
		}	   
	   
		//printk (KERN_ALERT "TAMC532_READ_DMA: DMA_NUM %i: REST_DMA_SIZE %i   DMA_SIZE %i \n",  tmp_dma_num, tmp_dma_size_rest, tmp_dma_size);
		tmp_dma_num         = tmp_dma_size/(TAMC532_DMAC_MAX_SIZE * 4);
		tmp_dma_size_rest  = tmp_dma_size%(TAMC532_DMAC_MAX_SIZE * 4);
		//printk (KERN_ALERT "TAMC532_READ_DMA: DMA_NUM %i: REST_DMA_SIZE %i   DMA_SIZE %i \n",  tmp_dma_num, tmp_dma_size_rest, tmp_dma_size);
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
		tamc532dev->dma_page_num[tmp_dma_offset] = tmp_dma_num;
		//printk (KERN_ALERT "TAMC532_READ_DMA: DMA_NUM %i: REST_DMA_SIZE %i   DMA_SIZE %i \n",  tmp_dma_num, tmp_dma_size_rest, tmp_dma_size);
		            
		for(tmp_dma_count    = 0; tmp_dma_count < tmp_dma_num; ++tmp_dma_count){
			tmp_dma_count_size = tmp_dma_size;
			if(tmp_dma_count == 0 ) tmp_dma_size = tmp_dma_size_rest;
			tamc532dev->dev_dma_size[tmp_dma_offset][tmp_dma_count] = tmp_dma_size;
			tmp_dma_buf_size    = tmp_dma_size;
			tmp_dma_trns_size   = tmp_dma_buf_size;

			if((tmp_dma_buf_size%PCIEDEV_DMA_SYZE)){
				tmp_dma_buf_size    = tmp_dma_size + (tmp_dma_size%PCIEDEV_DMA_SYZE);
			}
			
			
			tmp_dma_trns_size = tmp_dma_buf_size;
			dma_trans_cnt   = 0;
			dma_trans_rest  = tmp_dma_size; 
			tmp_order = get_order(tmp_dma_trns_size);
			tamc532dev->dma_order[tmp_dma_offset][tmp_dma_count]                  = tmp_order;
			tamc532dev->dev_dma_trans_size[tmp_dma_offset][tmp_dma_count]  =  tmp_dma_trns_size;
			
			if (!tamc532dev->pWriteBuf[tmp_dma_offset][tmp_dma_count]){
				tamc532dev->pWriteBuf[tmp_dma_offset][tmp_dma_count] = (void *)__get_free_pages(GFP_KERNEL | __GFP_DMA, tmp_order);
			}
			if (!tamc532dev->pWriteBuf[tmp_dma_offset][tmp_dma_count]){
				printk (KERN_ALERT "------TAMC532_READ_DMA: NO MEMORY FOR SIZE %i DMAC %i COUNT %i\n",tmp_dma_size, tmp_dma_offset, tmp_dma_count);
				for(i = 0; i < tmp_dma_num; ++i){
					if(tamc532dev->pWriteBuf[tmp_dma_offset][i])
						free_pages((ulong)tamc532dev->pWriteBuf[tmp_dma_offset][i], (ulong)tamc532dev->dma_order[tmp_dma_offset][i]);
						tamc532dev->dev_dma_size[tmp_dma_offset][i]           = 0;
						tamc532dev->dma_order[tmp_dma_offset][i]               = 0;
						tamc532dev->dev_dma_trans_size[tmp_dma_offset][i] =  0;
						tamc532dev->dev_dma_desc_size[tmp_dma_offset][i]  = 0;
						tamc532dev->pWriteBuf[tmp_dma_offset][i] = 0;
					}
				mutex_unlock(&dev->dev_mut);
				return -EFAULT;
			}
	
			//***************set DMA DESCRIPTORS*******************	
			tmp_dma_desc_size = sizeof(tamc532_dma_desc);
			
			if(!(tamc532dev->pTmpDmaHandle[tmp_dma_offset][tmp_dma_count] ))
				tamc532dev->pTmpDmaHandle[tmp_dma_offset][tmp_dma_count]      = pci_map_single(pdev, tamc532dev->pWriteBuf[tmp_dma_offset][tmp_dma_count], tmp_dma_trns_size, PCI_DMA_FROMDEVICE);
			tmp_data_32         = ((tmp_dma_size/4) << 16);
			
			if(tmp_dma_count == 0){
				tmp_data_32         |= TAMC532_DESCR_HI_EOL;
			}
			
			tmp_dma_desc.control      = cpu_to_be32((unsigned int)(tmp_data_32));                
			tmp_dma_desc.dataptr     = cpu_to_be32( (unsigned int)(tamc532dev->pTmpDmaHandle[tmp_dma_offset][tmp_dma_count]) );
			if(tmp_dma_count > 0){
				tmp_dma_desc.next_desc = cpu_to_be32((unsigned int)(tamc532dev->pTmpDescHandle[tmp_dma_offset][tmp_dma_count -1]));
			}
			if(tmp_dma_count == 0){
				tmp_dma_desc.next_desc = 0x00000000;
			}
			memcpy(tamc532dev->pDescBuf[tmp_dma_offset][tmp_dma_count], &tmp_dma_desc, TAMC532_DESC_SIZE);
			if(!(tamc532dev->pTmpDescHandle[tmp_dma_offset][tmp_dma_count] ))
				tamc532dev->pTmpDescHandle[tmp_dma_offset][tmp_dma_count]      = pci_map_single(pdev, tamc532dev->pDescBuf[tmp_dma_offset][tmp_dma_count], TAMC532_DESC_SIZE, PCI_DMA_TODEVICE);
			tmp_dma_size = tmp_dma_count_size;
			++tamc532dev->irq_num[tmp_dma_offset] ;
		}
        
		//************ disable the DMA controller ***************/
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
		//tmp_data_32       |=  TAMC532_MODULEINT_DMACx_DMF(tmp_dma_offset) ;
		
/*
		if(tamc532dev->is_strm_dma){
			//printk (KERN_ALERT "******TAMC532_READ_DMA: SET INTERRUPTS\n");
			iowrite32( TAMC532_SWAPL(tmp_data_32), ( address + MODULE_INTERRUPT_ENABLE_REG ));
		}else{
			iowrite32( 0, ( address + MODULE_INTERRUPT_ENABLE_REG ));
		}
*/
		iowrite32( 0, ( address + MODULE_INTERRUPT_ENABLE_REG ));
		//iowrite32( TAMC532_SWAPL(tmp_data_32), ( address + MODULE_INTERRUPT_ENABLE_REG ));
		
		
		smp_wmb();
		udelay(2);

		tmp_data_32       = tamc532dev->pTmpDescHandle[tmp_dma_offset][tmp_dma_num -1];
		iowrite32( cpu_to_be32((unsigned int)(tmp_data_32)), ( address + DMAC0_BASE_DESCR_ADDRESS_REG + tmp_dma_offset*0x10));
		smp_wmb();
		udelay(20);
		
		//* Enable DMA controller. This will cause a prefetch of the descriptor list (or at least part of it) */
		do_gettimeofday(&(tamc532dev->dma_start_time));
		tamc532dev->dmac_done[tmp_dma_offset] = 0; 
		iowrite32( TAMC532_SWAPL(0x1), ( address + (DMAC0_CONTROL_REG + tmp_dma_offset*0x10)));
		smp_wmb();
		tamc532dev->dmac_enable[tmp_dma_offset] =1;
		udelay(2);
		break;
    case PCIEDEV_READ_DMA:   
		retval = 0;
		if (copy_from_user(&dma_data, (device_ioctrl_dma*)arg, (size_t)io_dma_size)) {
			retval = -EFAULT;
			mutex_unlock(&dev->dev_mut);
			printk (KERN_ALERT "PCIEDEV_SSET_DMA: ERROR COPY FROM USER\n");
			return retval;
		}
		tamc532dev->dev_dma_size_change = 0;
		tmp_dma_offset           = dma_data.dma_offset;
		tmp_dma_size              = dma_data.dma_size;
		length                          = tmp_dma_size;
		tmp_new_dma_size       = tmp_dma_size;
		tmp_last_dma_size        =  tamc532dev->dev_dma_last_size;
		
		tamc532dev->dev_dma_last_size = tmp_dma_size;
		
		if(tmp_new_dma_size != tmp_last_dma_size){
			 tamc532dev->dev_dma_size_change = 1;
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
				
		
		tmp_data_32         = ioread32(address + (DMAC0_STATUS_REG + tmp_dma_offset*0x10));
		smp_rmb();
		tmp_data_32       =TAMC532_SWAPL(tmp_data_32);
		printk (KERN_ALERT "PCIEDEV_SET_DMA: DMAC_STATUS %x  OFFSET %i\n", tmp_data_32, tmp_dma_offset);
		dmac_dqh_idle     = (tmp_data_32>>16) & 0xFFFF;
		dmac_aif_idle       =  (tmp_data_32>>8) & 0xFF;
		
		tmp_data = 0;
		tmp_data = dmac_dqh_idle + dmac_aif_idle;
		if(!tmp_data) tamc532dev->dmac_done[tmp_dma_offset] = 1;
		
		//**COPY DATA TO USER**/
		if((tamc532dev->dmac_enable[tmp_dma_offset])){
			
			if(!(tamc532dev->dmac_done[tmp_dma_offset] )){
				printk (KERN_ALERT "PCIEDEV_SET_DMA: DMA NOT DONE OFFSET %i\n", tmp_dma_offset);
				for(dma_wait_count = 0; dma_wait_count < 20; dma_wait_count++){
					printk (KERN_ALERT "******PCIEDEV_SET_DMA: DMA NOT DONE OFFSET %i POLLING NUM %I\n", tmp_dma_offset, dma_wait_count);
					if(tamc532dev->dmac_done[tmp_dma_offset] ) break;
					 if(dma_wait_count == 19){ 
						 printk (KERN_ALERT "******PCIEDEV_SET_DMA: DMA NOT DONE OFFSET %i \n", tmp_dma_offset);
						retval = -EAGAIN;
					}
					
					tmp_data_32       = ioread32(address + (DMAC0_STATUS_REG + tmp_dma_offset*0x10));
					smp_rmb();
					tmp_data_32       =TAMC532_SWAPL(tmp_data_32);
					
					dmac_dqh_idle = (tmp_data>>16) & 0xFFFF;
					dmac_aif_idle   =  (tmp_data>>8) & 0xFF;
					
					tmp_data = 0;
					tmp_data = dmac_dqh_idle + dmac_aif_idle;
					if(!tmp_data) tamc532dev->dmac_done[tmp_dma_offset] = 1;
					udelay(1000);
				}
			}
			//* disable the DMA controller */
			iowrite32( 0, ( address + (DMAC0_CONTROL_REG + tmp_dma_offset*0x10) ) );
			smp_wmb();
			iowrite32( 0, ( address + DMAC0_BASE_DESCR_ADDRESS_REG + tmp_dma_offset*0x10));
			smp_wmb();
			udelay(2);
			tmp_dma_count_offset    = 0;
						
			for(tmp_dma_count    = (tmp_dma_num -1) ; tmp_dma_count >= 0; tmp_dma_count--){
				tmp_dma_size = tamc532dev->dev_dma_size[tmp_dma_offset][tmp_dma_count] ;
				pci_unmap_single(pdev, tamc532dev->pTmpDescHandle[tmp_dma_offset][tmp_dma_count], TAMC532_DESC_SIZE, PCI_DMA_TODEVICE);
				tamc532dev->pTmpDescHandle[tmp_dma_offset][tmp_dma_count] = 0;
								
				pci_unmap_single(pdev, tamc532dev->pTmpDmaHandle[tmp_dma_offset][tmp_dma_count], tamc532dev->dev_dma_trans_size[tmp_dma_offset][tmp_dma_count], PCI_DMA_FROMDEVICE);
				if (copy_to_user ( ((void *)arg + tmp_dma_count_offset) , tamc532dev->pWriteBuf[tmp_dma_offset][tmp_dma_count] , tmp_dma_size) ) {
					printk (KERN_ALERT "PCIEDEV_SSET_DMA: ERROR COPY TO USER DMAC %i \n",tmp_dma_offset);
					retval = -EFAULT;
				}
				//>>>>>>>>
				if( tamc532dev->dev_dma_size_change){
					if (tamc532dev->pWriteBuf[tmp_dma_offset][tmp_dma_count]){
						free_pages((ulong)tamc532dev->pWriteBuf[tmp_dma_offset][tmp_dma_count], (ulong)tamc532dev->dma_order[tmp_dma_offset][tmp_dma_count]);
					}
					tamc532dev->pWriteBuf[tmp_dma_offset][tmp_dma_count]                  = 0;
				}	
				//<<<<<<<<<
			    
				tamc532dev->pTmpDmaHandle[tmp_dma_offset][tmp_dma_count]      = 0;
				tamc532dev->dev_dma_size[tmp_dma_offset][tmp_dma_count]            = 0;
				tamc532dev->dma_order[tmp_dma_offset][tmp_dma_count]                 = 0;
				tamc532dev->dev_dma_trans_size[tmp_dma_offset][tmp_dma_count] =  0;
				tamc532dev->dev_dma_desc_size[tmp_dma_offset][tmp_dma_count]  = 0;
				
				tmp_dma_count_offset    += tmp_dma_size;
			}
			tamc532dev->dmac_enable[tmp_dma_offset] = 0;
			tamc532dev->dmac_dma_int_enabled[tmp_dma_offset]  = 0;
		}
		tamc532dev->irq_num[tmp_dma_offset] = 0;
		   
		//**SET NEW DMA**/
		tmp_free_pages        = nr_free_pages();
		tmp_free_pages        = tmp_free_pages << (PAGE_SHIFT-10);
		tmp_free_pages        = tmp_free_pages/2;
		
		tmp_dma_size         = tmp_new_dma_size;
		if(tmp_dma_size <= 0){
			mutex_unlock(&dev->dev_mut);
			printk (KERN_ALERT "PCIEDEV_SSET_DMA: ERROR SIZE %i DMAC %i \n",tmp_dma_size, tmp_dma_offset);
			return -EFAULT;
		}

		if(tmp_dma_size > tmp_free_pages*1000){
			mutex_unlock(&dev->dev_mut);
			printk (KERN_ALERT "PCIEDEV_SSET_DMA: ERROR NO FREE PAGES SIZE %i: PAGES %i \n",  tmp_dma_size, tmp_free_pages*1000);
			return -ENOMEM;
		}	   
	   
		tmp_dma_num         = tmp_dma_size/(TAMC532_DMAC_MAX_SIZE * 4);
		tmp_dma_size_rest  = tmp_dma_size%(TAMC532_DMAC_MAX_SIZE * 4);
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
		tamc532dev->dma_page_num[tmp_dma_offset] = tmp_dma_num;
		            
		for(tmp_dma_count    = 0; tmp_dma_count < tmp_dma_num; ++tmp_dma_count){
			tmp_dma_count_size = tmp_dma_size;
			if(tmp_dma_count == 0 ) tmp_dma_size = tmp_dma_size_rest;
			tamc532dev->dev_dma_size[tmp_dma_offset][tmp_dma_count] = tmp_dma_size;
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
			
			if (!tamc532dev->pWriteBuf[tmp_dma_offset][tmp_dma_count]){
				tamc532dev->pWriteBuf[tmp_dma_offset][tmp_dma_count] = (void *)__get_free_pages(GFP_KERNEL | __GFP_DMA, tmp_order);
			}
			if (!tamc532dev->pWriteBuf[tmp_dma_offset][tmp_dma_count]){
				printk (KERN_ALERT "------PCIEDEV_SET_DMA: NO MEMORY FOR SIZE %i DMAC %i COUNT %i\n",tmp_dma_size, tmp_dma_offset, tmp_dma_count);
				for(i = 0; i < tmp_dma_num; ++i){
					if(tamc532dev->pWriteBuf[tmp_dma_offset][i])
						free_pages((ulong)tamc532dev->pWriteBuf[tmp_dma_offset][i], (ulong)tamc532dev->dma_order[tmp_dma_offset][i]);
						tamc532dev->dev_dma_size[tmp_dma_offset][i]           = 0;
						tamc532dev->dma_order[tmp_dma_offset][i]               = 0;
						tamc532dev->dev_dma_trans_size[tmp_dma_offset][i] =  0;
						tamc532dev->dev_dma_desc_size[tmp_dma_offset][i]  = 0;
						tamc532dev->pWriteBuf[tmp_dma_offset][i] = 0;
					}
				mutex_unlock(&dev->dev_mut);
				return -EFAULT;
			}
	
			//***************set DMA DESCRIPTORS*******************	
			tmp_dma_desc_size = sizeof(tamc532_dma_desc);
			
			if(!(tamc532dev->pTmpDmaHandle[tmp_dma_offset][tmp_dma_count] ))
				tamc532dev->pTmpDmaHandle[tmp_dma_offset][tmp_dma_count]      = pci_map_single(pdev, tamc532dev->pWriteBuf[tmp_dma_offset][tmp_dma_count], tmp_dma_trns_size, PCI_DMA_FROMDEVICE);
			tmp_data_32         = ((tmp_dma_size/4) << 16);
			
			if(tmp_dma_count == 0){
				tmp_data_32         |= TAMC532_DESCR_HI_EOL;
			}
			
			tmp_dma_desc.control      = cpu_to_be32((unsigned int)(tmp_data_32));                
			tmp_dma_desc.dataptr     = cpu_to_be32( (unsigned int)(tamc532dev->pTmpDmaHandle[tmp_dma_offset][tmp_dma_count]) );
			if(tmp_dma_count > 0){
				tmp_dma_desc.next_desc = cpu_to_be32((unsigned int)(tamc532dev->pTmpDescHandle[tmp_dma_offset][tmp_dma_count -1]));
			}
			if(tmp_dma_count == 0){
				tmp_dma_desc.next_desc = 0x00000000;
			}
			memcpy(tamc532dev->pDescBuf[tmp_dma_offset][tmp_dma_count], &tmp_dma_desc, TAMC532_DESC_SIZE);
			if(!(tamc532dev->pTmpDescHandle[tmp_dma_offset][tmp_dma_count] ))
				tamc532dev->pTmpDescHandle[tmp_dma_offset][tmp_dma_count]      = pci_map_single(pdev, tamc532dev->pDescBuf[tmp_dma_offset][tmp_dma_count], TAMC532_DESC_SIZE, PCI_DMA_TODEVICE);
			tmp_dma_size = tmp_dma_count_size;
			++tamc532dev->irq_num[tmp_dma_offset] ;
		}
        
		//************ disable the DMA controller ***************/
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
		//tmp_data_32       |=  TAMC532_MODULEINT_DMACx_DMF(tmp_dma_offset) ;
		//iowrite32( TAMC532_SWAPL(tmp_data_32), ( address + MODULE_INTERRUPT_ENABLE_REG ));
		iowrite32( 0, ( address + MODULE_INTERRUPT_ENABLE_REG ));
		smp_wmb();
		udelay(2);

		tmp_data_32       = tamc532dev->pTmpDescHandle[tmp_dma_offset][tmp_dma_num -1];
		iowrite32( cpu_to_be32((unsigned int)(tmp_data_32)), ( address + DMAC0_BASE_DESCR_ADDRESS_REG + tmp_dma_offset*0x10));
		smp_wmb();
		udelay(20);
		//* Enable DMA controller. This will cause a prefetch of the descriptor list (or at least part of it) */
		do_gettimeofday(&(tamc532dev->dma_start_time));
		tamc532dev->dmac_done[tmp_dma_offset] = 0; 
		iowrite32( TAMC532_SWAPL(0x1), ( address + (DMAC0_CONTROL_REG + tmp_dma_offset*0x10)));
		smp_wmb();
		tamc532dev->dmac_enable[tmp_dma_offset] =1;
		udelay(2);
		break;
	case TAMC532_SET_DMA:
/*
		#define TAMC532_DMAC_MAX_SIZE             32768
		//#define TAMC532_DMAC_MAX_SIZE             262140 // Byte 2097120
		//#define TAMC532_DMAC_MAX_SIZE             65536      //262140/4

		#define TAMC532_DMABUF_MAX_SIZE       262140 // 65535*4
		#define TAMC532_DESC_SIZE                       16

		//#define TAMC532_DMAC_MAX_NUM             64
		//#define TAMC532_DESC_OFFSET                 1024 //0x400
		#define TAMC532_DMAC_MAX_NUM             256
		#define TAMC532_DESC_OFFSET                 16 //0x400

		#define TAMC532_STRM_SAMPLE_NUM                 24
		#define TAMC532_STRM_TRQ_NUM                        500
*/

		retval = 0;
		if (copy_from_user(&dma_data, (device_ioctrl_dma*)arg, (size_t)io_dma_size)) {
			retval = -EFAULT;
			mutex_unlock(&dev->dev_mut);
			printk (KERN_ALERT "PCIEDEV_SSET_DMA: ERROR COPY FROM USER\n");
			return retval;
		}
		tmp_dma_offset        = dma_data.dma_offset;
		tmp_dma_size           = dma_data.dma_size;
		length                         = tmp_dma_size;
		tmp_new_dma_size  = tmp_dma_size;
		tmp_dma_cmd             = dma_data.dma_cmd;
		
		if(tmp_dma_offset < 0){
			mutex_unlock(&dev->dev_mut);
			printk (KERN_ALERT "TAMC532_READ_DMA: ERROR OFFSET DMAC %i \n",tmp_dma_offset);
			return -EFAULT;
		}
		if(tmp_dma_offset > 3){
			mutex_unlock(&dev->dev_mut);
			printk (KERN_ALERT "TAMC532_READ_DMA: ERROR OFFSET DMAC %i \n",tmp_dma_offset);
			return -EFAULT;
		}
		
		tamc532dev->is_strm_dma = 0;
		if(tmp_dma_cmd){
			tamc532dev->is_strm_dma = 1;
		}
		

		
		tmp_dma_num      = tamc532dev->dma_page_num[tmp_dma_offset];
				
		tmp_data_32         = ioread32(address + (DMAC0_STATUS_REG + tmp_dma_offset*0x10));
		smp_rmb();
		tmp_data_32       =TAMC532_SWAPL(tmp_data_32);
		dmac_dqh_idle = (tmp_data_32>>16) & 0xFFFF;
		dmac_aif_idle   =  (tmp_data_32>>8) & 0xFF;
		
		tmp_data = 0;
		tmp_data = dmac_dqh_idle + dmac_aif_idle;
		if(!tmp_data) tamc532dev->dmac_done[tmp_dma_offset] = 1;
		
		//**COPY DATA TO USER**/
		if((tamc532dev->dmac_enable[tmp_dma_offset])){
			
			if(!(tamc532dev->dmac_done[tmp_dma_offset] )){
				printk (KERN_ALERT "PCIEDEV_SET_DMA: DMA NOT DONE OFFSET %i\n", tmp_dma_offset);
				for(dma_wait_count = 0; dma_wait_count < 20; dma_wait_count++){
					printk (KERN_ALERT "******PCIEDEV_SET_DMA: DMA NOT DONE OFFSET %i POLLING NUM %I\n", tmp_dma_offset, dma_wait_count);
					if(tamc532dev->dmac_done[tmp_dma_offset] ) break;
					 if(dma_wait_count == 19){ 
						 printk (KERN_ALERT "******PCIEDEV_SET_DMA: DMA NOT DONE OFFSET %i \n", tmp_dma_offset);
						retval = -EAGAIN;
					}
					
					tmp_data_32       = ioread32(address + (DMAC0_STATUS_REG + tmp_dma_offset*0x10));
					smp_rmb();
					tmp_data_32       =TAMC532_SWAPL(tmp_data_32);
					
					dmac_dqh_idle = (tmp_data>>16) & 0xFFFF;
					dmac_aif_idle   =  (tmp_data>>8) & 0xFF;
					
					tmp_data = 0;
					tmp_data = dmac_dqh_idle + dmac_aif_idle;
					if(!tmp_data) tamc532dev->dmac_done[tmp_dma_offset] = 1;
					udelay(1000);
				}
			}
			//* disable the DMA controller */
			iowrite32( 0, ( address + (DMAC0_CONTROL_REG + tmp_dma_offset*0x10) ) );
			smp_wmb();
			iowrite32( 0, ( address + DMAC0_BASE_DESCR_ADDRESS_REG + tmp_dma_offset*0x10));
			smp_wmb();
			udelay(2);
			tmp_dma_count_offset    = 0;
						
			for(tmp_dma_count    = (tmp_dma_num -1) ; tmp_dma_count >= 0; tmp_dma_count--){
				tmp_dma_size = tamc532dev->dev_dma_size[tmp_dma_offset][tmp_dma_count] ;
				pci_unmap_single(pdev, tamc532dev->pTmpDescHandle[tmp_dma_offset][tmp_dma_count], TAMC532_DESC_SIZE, PCI_DMA_TODEVICE);
				tamc532dev->pTmpDescHandle[tmp_dma_offset][tmp_dma_count] = 0;
								
				pci_unmap_single(pdev, tamc532dev->pTmpDmaHandle[tmp_dma_offset][tmp_dma_count], tamc532dev->dev_dma_trans_size[tmp_dma_offset][tmp_dma_count], PCI_DMA_FROMDEVICE);
				if (copy_to_user ( ((void *)arg + tmp_dma_count_offset) , tamc532dev->pWriteBuf[tmp_dma_offset][tmp_dma_count] , tmp_dma_size) ) {
					printk (KERN_ALERT "PCIEDEV_SSET_DMA: ERROR COPY TO USER DMAC %i \n",tmp_dma_offset);
					retval = -EFAULT;
				}
				
				if (tamc532dev->pWriteBuf[tmp_dma_offset][tmp_dma_count]){
					free_pages((ulong)tamc532dev->pWriteBuf[tmp_dma_offset][tmp_dma_count], (ulong)tamc532dev->dma_order[tmp_dma_offset][tmp_dma_count]);
			    }
				tamc532dev->pTmpDmaHandle[tmp_dma_offset][tmp_dma_count]      = 0;
				tamc532dev->pWriteBuf[tmp_dma_offset][tmp_dma_count]                  = 0;
				tamc532dev->dev_dma_size[tmp_dma_offset][tmp_dma_count]            = 0;
				tamc532dev->dma_order[tmp_dma_offset][tmp_dma_count]                 = 0;
				tamc532dev->dev_dma_trans_size[tmp_dma_offset][tmp_dma_count] =  0;
				tamc532dev->dev_dma_desc_size[tmp_dma_offset][tmp_dma_count]  = 0;
				tmp_dma_count_offset    += tmp_dma_size;
			}
			tamc532dev->dmac_enable[tmp_dma_offset] = 0;
			tamc532dev->dmac_dma_int_enabled[tmp_dma_offset]  = 0;
		}
		tamc532dev->irq_num[tmp_dma_offset] = 0;
		   
		//**SET NEW DMA**/
		tmp_free_pages        = nr_free_pages();
		tmp_free_pages        = tmp_free_pages << (PAGE_SHIFT-10);
		tmp_free_pages        = tmp_free_pages/2;
		
		tmp_dma_size         = tmp_new_dma_size;
		if(tmp_dma_size <= 0){
			mutex_unlock(&dev->dev_mut);
			printk (KERN_ALERT "PCIEDEV_SSET_DMA: ERROR SIZE %i DMAC %i \n",tmp_dma_size, tmp_dma_offset);
			return -EFAULT;
		}

		if(tmp_dma_size > tmp_free_pages*1000){
			mutex_unlock(&dev->dev_mut);
			printk (KERN_ALERT "PCIEDEV_SSET_DMA: ERROR NO FREE PAGES SIZE %i: PAGES %i \n",  tmp_dma_size, tmp_free_pages*1000);
			return -ENOMEM;
		}	   
	   
		tmp_dma_num         = tmp_dma_size/(TAMC532_DMAC_MAX_SIZE * 4);
		tmp_dma_size_rest  = tmp_dma_size%(TAMC532_DMAC_MAX_SIZE * 4);
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
		tamc532dev->dma_page_num[tmp_dma_offset] = tmp_dma_num;
		            
		for(tmp_dma_count    = 0; tmp_dma_count < tmp_dma_num; ++tmp_dma_count){
			tmp_dma_count_size = tmp_dma_size;
			if(tmp_dma_count == 0 ) tmp_dma_size = tmp_dma_size_rest;
			tamc532dev->dev_dma_size[tmp_dma_offset][tmp_dma_count] = tmp_dma_size;
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
			
			if (!tamc532dev->pWriteBuf[tmp_dma_offset][tmp_dma_count]){
				tamc532dev->pWriteBuf[tmp_dma_offset][tmp_dma_count] = (void *)__get_free_pages(GFP_KERNEL | __GFP_DMA, tmp_order);
			}
			if (!tamc532dev->pWriteBuf[tmp_dma_offset][tmp_dma_count]){
				printk (KERN_ALERT "------PCIEDEV_SET_DMA: NO MEMORY FOR SIZE %i DMAC %i COUNT %i\n",tmp_dma_size, tmp_dma_offset, tmp_dma_count);
				for(i = 0; i < tmp_dma_num; ++i){
					if(tamc532dev->pWriteBuf[tmp_dma_offset][i])
						free_pages((ulong)tamc532dev->pWriteBuf[tmp_dma_offset][i], (ulong)tamc532dev->dma_order[tmp_dma_offset][i]);
						tamc532dev->dev_dma_size[tmp_dma_offset][i]           = 0;
						tamc532dev->dma_order[tmp_dma_offset][i]               = 0;
						tamc532dev->dev_dma_trans_size[tmp_dma_offset][i] =  0;
						tamc532dev->dev_dma_desc_size[tmp_dma_offset][i]  = 0;
						tamc532dev->pWriteBuf[tmp_dma_offset][i] = 0;
					}
				mutex_unlock(&dev->dev_mut);
				return -EFAULT;
			}
	
			//***************set DMA DESCRIPTORS*******************	
			tmp_dma_desc_size = sizeof(tamc532_dma_desc);
			
			if(!(tamc532dev->pTmpDmaHandle[tmp_dma_offset][tmp_dma_count] ))
				tamc532dev->pTmpDmaHandle[tmp_dma_offset][tmp_dma_count]      = pci_map_single(pdev, tamc532dev->pWriteBuf[tmp_dma_offset][tmp_dma_count], tmp_dma_trns_size, PCI_DMA_FROMDEVICE);
			tmp_data_32         = ((tmp_dma_size/4) << 16);
			
			if(tmp_dma_count == 0){
				tmp_data_32         |= TAMC532_DESCR_HI_EOL;
			}
			
			tmp_dma_desc.control      = cpu_to_be32((unsigned int)(tmp_data_32));                
			tmp_dma_desc.dataptr     = cpu_to_be32( (unsigned int)(tamc532dev->pTmpDmaHandle[tmp_dma_offset][tmp_dma_count]) );
			if(tmp_dma_count > 0){
				tmp_dma_desc.next_desc = cpu_to_be32((unsigned int)(tamc532dev->pTmpDescHandle[tmp_dma_offset][tmp_dma_count -1]));
			}
			if(tmp_dma_count == 0){
				tmp_dma_desc.next_desc = 0x00000000;
			}
			memcpy(tamc532dev->pDescBuf[tmp_dma_offset][tmp_dma_count], &tmp_dma_desc, TAMC532_DESC_SIZE);
			if(!(tamc532dev->pTmpDescHandle[tmp_dma_offset][tmp_dma_count] ))
				tamc532dev->pTmpDescHandle[tmp_dma_offset][tmp_dma_count]      = pci_map_single(pdev, tamc532dev->pDescBuf[tmp_dma_offset][tmp_dma_count], TAMC532_DESC_SIZE, PCI_DMA_TODEVICE);
			tmp_dma_size = tmp_dma_count_size;
			++tamc532dev->irq_num[tmp_dma_offset] ;
		}
        
		//************ disable the DMA controller ***************/
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
		//tmp_data_32       |=  TAMC532_MODULEINT_DMACx_DMF(tmp_dma_offset) ;
		//iowrite32( TAMC532_SWAPL(tmp_data_32), ( address + MODULE_INTERRUPT_ENABLE_REG ));
		iowrite32( 0, ( address + MODULE_INTERRUPT_ENABLE_REG ));
		smp_wmb();
		udelay(2);

		tmp_data_32       = tamc532dev->pTmpDescHandle[tmp_dma_offset][tmp_dma_num -1];
		iowrite32( cpu_to_be32((unsigned int)(tmp_data_32)), ( address + DMAC0_BASE_DESCR_ADDRESS_REG + tmp_dma_offset*0x10));
		smp_wmb();
		udelay(20);
		//* Enable DMA controller. This will cause a prefetch of the descriptor list (or at least part of it) */
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
						pci_unmap_single(pdev, tamc532dev->pTmpDescHandle[tmp_dma_offset][tmp_dma_count], TAMC532_DESC_SIZE, PCI_DMA_TODEVICE);
						//free_pages((ulong)tamc532dev->pDescBuf[tmp_dma_offset][tmp_dma_count], (ulong)tamc532dev->dma_desc_order[tmp_dma_offset][tmp_dma_count]);
						tamc532dev->pTmpDescHandle[tmp_dma_offset][tmp_dma_count] = 0;
						//tamc532dev->pDescBuf[tmp_dma_offset][tmp_dma_count]  = 0;
					}
					tamc532dev->dev_dma_size[tmp_dma_offset][tmp_dma_count]           = 0;
					tamc532dev->dev_dma_size[tmp_dma_offset][tmp_dma_count]           = 0;
					tamc532dev->dma_order[tmp_dma_offset][tmp_dma_count]               = 0;
					tamc532dev->dev_dma_trans_size[tmp_dma_offset][tmp_dma_count] =  0;
					tamc532dev->dev_dma_desc_size[tmp_dma_offset][tmp_dma_count]  = 0;
				}
				
			}
			tamc532dev->dmac_enable[tmp_dma_offset]            = 0;
			tamc532dev->dmac_dma_int_enabled[tmp_dma_offset]  = 0;
			tamc532dev->irq_num[tmp_dma_offset] = 0;
			tamc532dev->irq_dmac_rcv[tmp_dma_offset] = 0;

			tmp_data_32       = ioread32(address + MODULE_INTERRUPT_ENABLE_REG);
			smp_rmb();
			tmp_data_32       = TAMC532_SWAPL(tmp_data_32);
			tmp_data_32       = ( tmp_data_32 & ~TAMC532_MODULEINT_DMACx_DMF(tmp_dma_offset) );
			tmp_data_32       = ( tmp_data_32 & ~TAMC532_MODULEINT_DMACx_EOL(tmp_dma_offset) );
			iowrite32( TAMC532_SWAPL(tmp_data_32), ( address + MODULE_INTERRUPT_ENABLE_REG ));
			//iowrite32( 0, ( address + MODULE_INTERRUPT_ENABLE_REG ));
			smp_wmb();
			udelay(2);
		
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
			
	case TAMC532_READ_STREAM_DMA:
		retval = 0;

		if (copy_from_user(&stream_dma_data, (device_ioctrl_stream_dma*)arg, (size_t)io_stream_dma_size)) {
			retval = -EFAULT;
			mutex_unlock(&dev->dev_mut);
			return retval;
		}
		tmp_dma_offset  = stream_dma_data.dma_offset & 0xF;
		tmp_dma_num      = tamc532dev->strm_dma_page_num;
		
		//printk (KERN_ALERT "TAMC532_READ_STREAM_DMA:  DMAC %i DMA_NUM%i CUR_BUF %i\n", tmp_dma_offset, tmp_dma_num, tamc532dev->strm_dma_cur_buf);
		
		tmp_dma_count_offset    = 0;
		for(tmp_dma_count    = (tmp_dma_num -1) ; tmp_dma_count >= 0; tmp_dma_count--){
			tmp_dma_size = tamc532dev->dev_stream_dma_size[tmp_dma_offset][tmp_dma_count] ;
			//printk (KERN_ALERT "TAMC532_READ_STREAM_DMA:  COPY BUF0 COUNT %i:%i SIZE %i\n", tmp_dma_count, tmp_dma_num, tmp_dma_size);
			if(tamc532dev->strm_dma_cur_buf){
				dma_sync_single_for_cpu(&(pdev->dev), tamc532dev->pStrmDmaHandle[tmp_dma_offset][tmp_dma_count], tamc532dev->dev_stream_dma_trans_size[tmp_dma_offset][tmp_dma_count], PCI_DMA_FROMDEVICE);

				
				//printk (KERN_ALERT "TAMC532_READ_STREAM_DMA:  CUR_BUF %i : SIZE %i,   %X:%X\n", tamc532dev->strm_dma_cur_buf, tamc532dev->dev_stream_dma_trans_size[tmp_dma_offset][tmp_dma_count],
				//		 tamc532dev->pStrmWriteBuf[tmp_dma_offset][tmp_dma_count], tamc532dev->pStrmDmaHandle[tmp_dma_offset][tmp_dma_count]);
				
				
				if (copy_to_user ( ((void *)arg + tmp_dma_count_offset) , tamc532dev->pStrmWriteBuf[tmp_dma_offset][tmp_dma_count] , tmp_dma_size) ) {
					printk (KERN_ALERT "TAMC532_DO_WORK: ERROR COPY TO USER DMAC %i \n",tmp_dma_offset);
					dma_sync_single_for_device(&(pdev->dev), tamc532dev->pStrmDmaHandle[tmp_dma_offset][tmp_dma_count], tamc532dev->dev_stream_dma_trans_size[tmp_dma_offset][tmp_dma_count], PCI_DMA_FROMDEVICE);
					retval = -EFAULT;
					mutex_unlock(&dev->dev_mut);
					return retval;
				}


				dma_sync_single_for_device(&(pdev->dev), tamc532dev->pStrmDmaHandle[tmp_dma_offset][tmp_dma_count], tamc532dev->dev_stream_dma_trans_size[tmp_dma_offset][tmp_dma_count], PCI_DMA_FROMDEVICE);
			}else{
				dma_sync_single_for_cpu(&(pdev->dev), tamc532dev->pStrmDmaHandle1[tmp_dma_offset][tmp_dma_count], tamc532dev->dev_dma_trans_size[tmp_dma_offset][tmp_dma_count], PCI_DMA_FROMDEVICE);

				
				//printk (KERN_ALERT "TAMC532_READ_STREAM_DMA:  CUR_BUF %i : SIZE %i,   %X:%X\n", tamc532dev->strm_dma_cur_buf, tamc532dev->dev_stream_dma_trans_size[tmp_dma_offset][tmp_dma_count],
				//		 tamc532dev->pStrmWriteBuf1[tmp_dma_offset][tmp_dma_count], tamc532dev->pStrmDmaHandle1[tmp_dma_offset][tmp_dma_count]);
				
				
				//printk (KERN_ALERT "TAMC532_READ_STREAM_DMA:  COPY BUF1 COUNT %i:%i\n", tmp_dma_count, tmp_dma_num);
				if (copy_to_user ( ((void *)arg + tmp_dma_count_offset) , tamc532dev->pStrmWriteBuf1[tmp_dma_offset][tmp_dma_count] , tmp_dma_size) ) {
					printk (KERN_ALERT "TAMC532_DO_WORK: ERROR COPY TO USER DMAC %i \n",tmp_dma_offset);
					dma_sync_single_for_device(&(pdev->dev), tamc532dev->pStrmDmaHandle1[tmp_dma_offset][tmp_dma_count], tamc532dev->dev_dma_trans_size[tmp_dma_offset][tmp_dma_count], PCI_DMA_FROMDEVICE);
					retval = -EFAULT;
					mutex_unlock(&dev->dev_mut);
					return retval;
				}


				dma_sync_single_for_device(&(pdev->dev), tamc532dev->pStrmDmaHandle1[tmp_dma_offset][tmp_dma_count], tamc532dev->dev_dma_trans_size[tmp_dma_offset][tmp_dma_count], PCI_DMA_FROMDEVICE);
			}

			tmp_dma_count_offset    += tmp_dma_size;
		}
	
		break;
			
	case TAMC532_SET_STREAM_DMA:
		retval = 0;

		if (copy_from_user(&stream_dma_data, (device_ioctrl_stream_dma*)arg, (size_t)io_stream_dma_size)) {
			retval = -EFAULT;
			mutex_unlock(&dev->dev_mut);
			return retval;
		}
		
		tmp_dma_cmd            = stream_dma_data.dma_cmd;
		tmp_stream_trg_num = stream_dma_data.dma_reserved1;
		tmp_dma_offset         = stream_dma_data.dma_offset & 0xF;
		tmp_dma_size             = stream_dma_data.dma_size; //size in B for one DMAC
		length                           = tmp_dma_size;
		tmp_new_dma_size    = tmp_dma_size;
		tmp_dma_buf_size       = stream_dma_data.dma_size * tmp_stream_trg_num;  //500 TRGs with 62usec will done ~30msec to fill the buffer
		
		tamc532dev->strm_dmac_num = 0;
		for(i = 0; i < DMACNUM; i++){
			tamc532dev->strm_dmac_enbl[i] = 0;
			tamc532dev->strm_trg_num[i] = 0;
			if( ((tmp_dma_offset>>i)&0x1 )){
				tamc532dev->strm_dmac_num += 1;
				tamc532dev->strm_dmac_enbl[i] = 1;
			}
		}
		
		printk (KERN_ALERT "TAMC532_SET_STREAM_DMA: SAMPLE SIZE %i DMACs %X  DMACS_NUM %i TRG_NUM%i DMA_SIZE %i \n",
				tmp_dma_size, tmp_dma_offset, tamc532dev->strm_dmac_num, tmp_stream_trg_num, tmp_dma_buf_size);
		
		if(tmp_dma_size <= 0){
			mutex_unlock(&dev->dev_mut);
			printk (KERN_ALERT "TAMC532_SET_STREAM_DMA: ERROR SIZE %i DMAC %X \n",tmp_dma_size, tmp_dma_offset);
			return -EFAULT;
		}
		
		if(tmp_dma_size  > TAMC532_DMAC_MAX_SIZE*4){
			mutex_unlock(&dev->dev_mut);
			printk (KERN_ALERT "TAMC532_SET_STREAM_DMA: ERROR SIZE %i DMAC %X \n",tmp_dma_size, tmp_dma_offset);
			return -EFAULT;
		}
		
		if(tmp_dma_offset < 0){
			mutex_unlock(&dev->dev_mut);
			printk (KERN_ALERT "TAMC532_READ_DMA: ERROR OFFSET DMAC %X \n",tmp_dma_offset);
			return -EFAULT;
		}
		if(tmp_dma_offset > 0xF){
			mutex_unlock(&dev->dev_mut);
			printk (KERN_ALERT "TAMC532_READ_DMA: ERROR OFFSET DMAC %X \n",tmp_dma_offset);
			return -EFAULT;
		}
		
		tmp_data_32       = ioread32(address + CSPTA_DATA0_REG );
		smp_rmb();
		tmp_pre_sample = TAMC532_SWAPL(tmp_data_32);
		
		tmp_data_32       = ioread32(address + CSPTA_DATA1_REG);
		smp_rmb();
		tmp_post_sample = TAMC532_SWAPL(tmp_data_32);
		
		tmp_sample_num = tmp_pre_sample + tmp_post_sample;
		
		printk (KERN_ALERT "TAMC532_SET_STREAM_DMA: PRE_SAMPLE %i POST_SAMPLE %i  SAMPLE_NUM %i \n",
				tmp_pre_sample, tmp_post_sample, tmp_sample_num);
		
		
		tamc532dev->strm_buf_dma_size =tmp_dma_buf_size;
		tamc532dev->strm_dma_size        = tmp_dma_size ; 
		
		//**SET NEW DMA**/
		
		
		tmp_free_pages        = nr_free_pages();
		tmp_free_pages        = tmp_free_pages << (PAGE_SHIFT-10);
		tmp_free_pages        = tmp_free_pages;
		if( (tmp_dma_buf_size*2*tamc532dev->strm_dmac_num) > tmp_free_pages*1000){
			for(i = 0; i < DMACNUM; i++){
				tamc532dev->strm_dmac_enbl[i] = 0;
				tamc532dev->strm_trg_num[i] = 0;
			}
			tamc532dev->strm_dmac_num = 0;
			tamc532dev->is_strm_dma = 0;
			tamc532dev->is_strm_dma_run = 0;
			tamc532dev->strm_dma_size = 0;
			tamc532dev->strm_buf_dma_size =0;
			mutex_unlock(&dev->dev_mut);
			printk (KERN_ALERT "TAMC532_SET_STREAM_DMA: ERROR NO FREE PAGES SIZE %i: PAGES %i \n",  
					                                                                      tamc532dev->strm_dma_size, tmp_free_pages*1000);
			return -ENOMEM;
		}
		
		
		tmp_dma_num         = tmp_dma_buf_size/(TAMC532_DMAC_MAX_SIZE * 4);
		tmp_dma_size_rest  = tmp_dma_buf_size%(TAMC532_DMAC_MAX_SIZE * 4);
		if(!tmp_dma_num){
			tmp_dma_num = 1;
			tmp_dma_buf_size  = tmp_dma_size_rest;
		}else{
			tmp_dma_buf_size  = TAMC532_DMAC_MAX_SIZE*4;
			if(tmp_dma_size_rest){
				tmp_dma_num += 1;
			}else{
				tmp_dma_size_rest = tmp_dma_buf_size;
			}
		}
		tamc532dev->strm_dma_page_num = tmp_dma_num;
		
		
/*
		int	 	         dev_stream_dma_size[TAMC532_DMAC_MAX_NUM];
		int                        dev_stream_dma_trans_size[TAMC532_DMAC_MAX_NUM];
		int                        stream_dma_order[TAMC532_DMAC_MAX_NUM];
		void*                    pStrmWriteBuf[2][TAMC532_DMAC_MAX_NUM] ;
		dma_addr_t        pStrmDmaHandle[2][TAMC532_DMAC_MAX_NUM];
*/
		
		for(tmp_dma_offset = 0; tmp_dma_offset < DMACNUM; tmp_dma_offset++){
			if(tamc532dev->strm_dmac_enbl[tmp_dma_offset]){
				printk (KERN_ALERT "TAMC532_SET_STREAM_DMA: SET BUFFERS FOR DMAC %i \n", tmp_dma_offset);
				for(tmp_dma_count    = 0; tmp_dma_count < tmp_dma_num; ++tmp_dma_count){
					tmp_dma_count_size = tmp_dma_buf_size;
					if(tmp_dma_count == 0 ) tmp_dma_buf_size = tmp_dma_size_rest;
					tamc532dev->dev_stream_dma_size[tmp_dma_offset][tmp_dma_count] = tmp_dma_buf_size;
					tmp_dma_trns_size   = tmp_dma_buf_size;

					if((tmp_dma_buf_size%PCIEDEV_DMA_SYZE)){
						tmp_dma_buf_size    = tmp_dma_buf_size + (tmp_dma_buf_size%PCIEDEV_DMA_SYZE);
					}

					tmp_dma_trns_size = tmp_dma_buf_size;
					tmp_order = get_order(tmp_dma_trns_size);
					tamc532dev->stream_dma_order[tmp_dma_offset][tmp_dma_count]                  = tmp_order;
					tamc532dev->dev_stream_dma_trans_size[tmp_dma_offset][tmp_dma_count]  =  tmp_dma_trns_size;

					if (!tamc532dev->pStrmWriteBuf[tmp_dma_offset][tmp_dma_count]){
						tamc532dev->pStrmWriteBuf[tmp_dma_offset][tmp_dma_count] = (void *)__get_free_pages(GFP_KERNEL | __GFP_DMA, tmp_order);
					}
					if (!tamc532dev->pStrmWriteBuf[tmp_dma_offset][tmp_dma_count]){
						printk (KERN_ALERT "------TAMC532_SET_STREAM_DMA: NO MEMORY FOR BUF0 SIZE %i DMAC %i COUNT %i\n",tmp_dma_buf_size, tmp_dma_offset, tmp_dma_count);
						for(i = 0; i < tmp_dma_num; ++i){
							if(tamc532dev->pStrmWriteBuf[tmp_dma_offset][i])
								free_pages((ulong)tamc532dev->pStrmWriteBuf[tmp_dma_offset][i], (ulong)tamc532dev->stream_dma_order[tmp_dma_offset][i]);
							tamc532dev->dev_stream_dma_size[tmp_dma_offset][i]           = 0;
							tamc532dev->stream_dma_order[tmp_dma_offset][i]               = 0;
							tamc532dev->dev_stream_dma_trans_size[tmp_dma_offset][i] =  0;
							tamc532dev->pStrmWriteBuf[tmp_dma_offset][i] = 0;
						}
						mutex_unlock(&dev->dev_mut);
						return -EFAULT;
					}
					if (!tamc532dev->pStrmWriteBuf1[tmp_dma_offset][tmp_dma_count]){
						tamc532dev->pStrmWriteBuf1[tmp_dma_offset][tmp_dma_count] = (void *)__get_free_pages(GFP_KERNEL | __GFP_DMA, tmp_order);
					}
					if (!tamc532dev->pStrmWriteBuf1[tmp_dma_offset][tmp_dma_count]){
						printk (KERN_ALERT "------TAMC532_READ_DMA: NO MEMORY FOR BUF1 SIZE %i DMAC %i COUNT %i\n",tmp_dma_buf_size, tmp_dma_offset, tmp_dma_count);
						for(i = 0; i < tmp_dma_num; ++i){
							if(tamc532dev->pStrmWriteBuf1[tmp_dma_offset][i])
								free_pages((ulong)tamc532dev->pStrmWriteBuf1[tmp_dma_offset][i], (ulong)tamc532dev->stream_dma_order[tmp_dma_offset][i]);
							tamc532dev->dev_stream_dma_size[tmp_dma_offset][i]           = 0;
							tamc532dev->stream_dma_order[tmp_dma_offset][i]               = 0;
							tamc532dev->dev_stream_dma_trans_size[tmp_dma_offset][i] =  0;
							tamc532dev->pStrmWriteBuf1[tmp_dma_offset][i] = 0;
							if(tamc532dev->pStrmWriteBuf[tmp_dma_offset][i])
								free_pages((ulong)tamc532dev->pStrmWriteBuf[tmp_dma_offset][i], (ulong)tamc532dev->stream_dma_order[tmp_dma_offset][i]);
							tamc532dev->pStrmWriteBuf[tmp_dma_offset][i] = 0;
						}
						mutex_unlock(&dev->dev_mut);
						return -EFAULT;
					}
					
					
					

					//***************set DMA DESCRIPTORS*******************	
					tmp_dma_desc_size = sizeof(tamc532_dma_desc);

					if(!(tamc532dev->pStrmDmaHandle[tmp_dma_offset][tmp_dma_count] ))
						tamc532dev->pStrmDmaHandle[tmp_dma_offset][tmp_dma_count]      = pci_map_single(pdev, tamc532dev->pStrmWriteBuf[tmp_dma_offset][tmp_dma_count], tmp_dma_trns_size, PCI_DMA_FROMDEVICE);
					tmp_data_32         = ((tmp_dma_buf_size/4) << 16);

					if(tmp_dma_count == 0){
						//tmp_data_32         |= TAMC532_DESCR_HI_EOL;
						tmp_data_32         |= TAMC532_DESCR_HI_DMF;
					}

					tmp_dma_desc.control      = cpu_to_be32((unsigned int)(tmp_data_32));                
					tmp_dma_desc.dataptr     = cpu_to_be32( (unsigned int)(tamc532dev->pStrmDmaHandle[tmp_dma_offset][tmp_dma_count]) );
					if(tmp_dma_count > 0){
						tmp_dma_desc.next_desc = cpu_to_be32((unsigned int)(tamc532dev->pStrmDmaHandle[tmp_dma_offset][tmp_dma_count -1]));
					}
					if(tmp_dma_count == 0){
						tmp_dma_desc.next_desc = 0x00000000;
					}
					printk (KERN_ALERT "------TAMC532_READ_DMA:MEMORY FOR DMA BOARD %i DESC0 %i:%i %X  \n", 
							tamc532dev->brd_num, tmp_dma_offset, tmp_dma_count, tamc532dev->pDescBuf[tmp_dma_offset][tmp_dma_count]);
					memcpy(tamc532dev->pDescBuf[tmp_dma_offset][tmp_dma_count], &tmp_dma_desc, TAMC532_DESC_SIZE);
					if(!(tamc532dev->pTmpDescHandle[tmp_dma_offset][tmp_dma_count] ))
						tamc532dev->pTmpDescHandle[tmp_dma_offset][tmp_dma_count]      = pci_map_single(pdev, tamc532dev->pDescBuf[tmp_dma_offset][tmp_dma_count], TAMC532_DESC_SIZE, PCI_DMA_TODEVICE);
					
					
					if(!(tamc532dev->pStrmDmaHandle1[tmp_dma_offset][tmp_dma_count] ))
						tamc532dev->pStrmDmaHandle1[tmp_dma_offset][tmp_dma_count]      = pci_map_single(pdev, tamc532dev->pStrmWriteBuf1[tmp_dma_offset][tmp_dma_count], tmp_dma_trns_size, PCI_DMA_FROMDEVICE);
					tmp_data_32         = ((tmp_dma_buf_size/4) << 16);

					if(tmp_dma_count == 0){
						//tmp_data_32         |= TAMC532_DESCR_HI_EOL;
						tmp_data_32         |= TAMC532_DESCR_HI_DMF;
					}

					tmp_dma_desc.control      = cpu_to_be32((unsigned int)(tmp_data_32));                
					tmp_dma_desc.dataptr     = cpu_to_be32( (unsigned int)(tamc532dev->pStrmDmaHandle1[tmp_dma_offset][tmp_dma_count]) );
					if(tmp_dma_count > 0){
						tmp_dma_desc.next_desc = cpu_to_be32((unsigned int)(tamc532dev->pStrmDmaHandle1[tmp_dma_offset][tmp_dma_count -1]));
					}
					if(tmp_dma_count == 0){
						tmp_dma_desc.next_desc = 0x00000000;
					}
					printk (KERN_ALERT "------TAMC532_READ_DMA:MEMORY FOR DMA BOARD %i DESC1 %i:%i %X  \n", 
							tamc532dev->brd_num, tmp_dma_offset, tmp_dma_count, tamc532dev->pDescBuf1[tmp_dma_offset][tmp_dma_count]);
					memcpy(tamc532dev->pDescBuf1[tmp_dma_offset][tmp_dma_count], &tmp_dma_desc, TAMC532_DESC_SIZE);
					if(!(tamc532dev->pTmpDescHandle1[tmp_dma_offset][tmp_dma_count] ))
						tamc532dev->pTmpDescHandle1[tmp_dma_offset][tmp_dma_count]      = pci_map_single(pdev, tamc532dev->pDescBuf1[tmp_dma_offset][tmp_dma_count], TAMC532_DESC_SIZE, PCI_DMA_TODEVICE);
					
					
					tmp_dma_buf_size = tmp_dma_count_size;
				}
			}
		}
		
		
		
		//************ disable CSPTs ***************/
		iowrite32( 0, ( address + APPLICATION_CONTROL_REG ) );
		smp_wmb();
		udelay(2);
		
		//************ FLASH DMA data and terminate current DMA ***************/
		tmp_data_32 = 0xF00;
		iowrite32(TAMC532_SWAPL(tmp_data_32), (address + APPLICATION_COMMAND_REG));
		smp_wmb();
		udelay(2);
		
		//************ reset CSPTs ***************/
		tmp_data_32 = 0xF0;
		iowrite32(TAMC532_SWAPL(tmp_data_32), (address + APPLICATION_COMMAND_REG));
		smp_wmb();
		udelay(2);
		
		//************ disable the DMA controller ***************/
		for(i = 0; i < DMACNUM; i++){
			// disable the DMA controller 
			iowrite32( 0, ( address + (DMAC0_CONTROL_REG + i*0x10) ) );
			smp_wmb();
			iowrite32( 0, ( address + DMAC0_BASE_DESCR_ADDRESS_REG + i*0x10));
			smp_wmb();
			udelay(2);
		}
		
		tamc532dev->server_signal_stack [0].f_ServerPref = cur_proc;
		tamc532dev->is_strm_dma         = 1;
		tamc532dev->is_strm_dma_run = 1;
		tamc532dev->strm_dmac_num  = 0;
		tamc532dev->strm_dma_cur_buf = 0;
		tamc532dev->dmac_strm_trg_num      = tmp_stream_trg_num;
		
		
		//************ set ABT DISSABLE ***************/
		for(i = 0; i < DMACNUM; i++){
			if( tamc532dev->strm_dmac_enbl[i]){
				tmp_data_32       = ioread32(address + CSPTA_CONTROL_REG + i*0x10);
				smp_rmb();
				tmp_data_32 = TAMC532_SWAPL(tmp_data_32);
				tmp_data_32 |= 0x10;
				iowrite32(TAMC532_SWAPL(tmp_data_32), (address + CSPTA_CONTROL_REG + i*0x10));
				smp_wmb();
			}
		}
		
		//************ set GlobalInterrupt to allow EOL/DMF interrupt ***************/
		tmp_data_32       = ioread32(address + MODULE_INTERRUPT_ENABLE_REG);
		smp_rmb();
		tmp_data_32       = TAMC532_SWAPL(tmp_data_32);
		for(i = 0; i < DMACNUM; i++){
			if( tamc532dev->strm_dmac_enbl[i]){
				//tmp_data_32       |=  TAMC532_MODULEINT_DMACx_EOL(i) ;		
				tmp_data_32       |=  TAMC532_MODULEINT_DMACx_DMF(i) ;
		
			}
		}
		//iowrite32( 0, ( address + MODULE_INTERRUPT_ENABLE_REG ));
		iowrite32( TAMC532_SWAPL(tmp_data_32), ( address + MODULE_INTERRUPT_ENABLE_REG ));
		smp_wmb();
		udelay(2);
		
		//************ set Descriptor Base Address Register to Buffer0 and enable DMACs and CSPTs ***************/
		tmp_dma_offset = 0;
		for(i = 0; i < DMACNUM; i++){
			if( tamc532dev->strm_dmac_enbl[i]){
				tmp_dma_offset += 1<<i;
				tmp_data_32       = tamc532dev->pTmpDescHandle[i][tmp_dma_num -1];
				iowrite32( cpu_to_be32((unsigned int)(tmp_data_32)), ( address + DMAC0_BASE_DESCR_ADDRESS_REG + i*0x10));
				smp_wmb();
				udelay(20);
				printk (KERN_ALERT "------TAMC532_SET_STREAM_DMA: DESC BUF ADDRESS %X :: %X\n",tmp_data_32, cpu_to_be32((unsigned int)(tmp_data_32)));
				
				tamc532dev->strm_dmac_done[i] = 0; 
				tamc532dev->strm_dmac_enbl[i] =1;
				
				iowrite32( TAMC532_SWAPL(0x1), ( address + (DMAC0_CONTROL_REG + i*0x10)));
				smp_wmb();
			}
		}
		iowrite32( TAMC532_SWAPL(tmp_dma_offset), ( address + APPLICATION_CONTROL_REG ) );
		smp_wmb();
		udelay(2);
		
		
		for(i = 0; i < 4; i++){
			tmp_data_32       = ioread32(address + DMAC0_STATUS_REG + i*0x10);
			smp_rmb();
			tmp_data_32 = TAMC532_SWAPL(tmp_data_32);
			stream_dma_data.pre_dma_status_reg[i] = tmp_data_32;
			
			tmp_data_32       = ioread32(address + DMAC0_CONTROL_REG + i*0x10);
			smp_rmb();
			tmp_data_32 = TAMC532_SWAPL(tmp_data_32);
			stream_dma_data.dma_control_reg[i] = tmp_data_32;
			
			tmp_data_32       = ioread32(address + CSPTA_CONTROL_REG + i*0x10);
			smp_rmb();
			tmp_data_32 = TAMC532_SWAPL(tmp_data_32);
			stream_dma_data.cspt_control[i] = tmp_data_32;
		}
		if ( copy_to_user((device_ioctrl_stream_dma*)arg, &stream_dma_data, (size_t)io_stream_dma_size) ) {
			retval = -EFAULT;
			mutex_unlock(&dev->dev_mut);
			return retval;
		}
		
		break;
	case TAMC532_REMOVE_STREAM_DMA:
		retval = 0;

		if (copy_from_user(&stream_dma_data, (device_ioctrl_stream_dma*)arg, (size_t)io_stream_dma_size)) {
			retval = -EFAULT;
			mutex_unlock(&dev->dev_mut);
			return retval;
		}
		
		tmp_dma_offset        = stream_dma_data.dma_offset & 0xF;
		tmp_dma_size            = stream_dma_data.dma_size;
		tmp_dma_buf_size     = stream_dma_data.dma_size * TAMC532_STRM_TRQ_NUM;  //500 TRGs with 62usec will done ~30msec to fill the buffer
		length                          = tmp_dma_size;
		tmp_new_dma_size   = tmp_dma_size;
		tmp_dma_cmd           = stream_dma_data.dma_cmd;
		
		printk (KERN_ALERT "+++++++++++++++TAMC532_REMOVE_STREAM_DMA\n");
		tamc532dev->is_strm_dma         = 0;
		tamc532dev->is_strm_dma_run = 0;
		
		iowrite32( 0, ( address + MODULE_INTERRUPT_ENABLE_REG ));
		smp_wmb();
		udelay(2);
		
		for(i = 0; i < 4; i++){
			tmp_data_32       = ioread32(address + DMAC0_STATUS_REG + i*0x10);
			smp_rmb();
			tmp_data_32 = TAMC532_SWAPL(tmp_data_32);
			stream_dma_data.pre_dma_status_reg[i] = tmp_data_32;
			
			tmp_data_32       = ioread32(address + DMAC0_CONTROL_REG + i*0x10);
			smp_rmb();
			tmp_data_32 = TAMC532_SWAPL(tmp_data_32);
			stream_dma_data.dma_control_reg[i] = tmp_data_32;
			
			tmp_data_32       = ioread32(address + CSPTA_CONTROL_REG + i*0x10);
			smp_rmb();
			tmp_data_32 = TAMC532_SWAPL(tmp_data_32);
			stream_dma_data.cspt_control[i] = tmp_data_32;
		}
		
		//************ disable CSPTs ***************/
		iowrite32( 0, ( address + APPLICATION_CONTROL_REG ) );
		smp_wmb();
		udelay(2);
		
		//************ FLAS DMA dat and terminate current DMA ***************/
		tmp_data_32 = 0xF00;
		iowrite32(TAMC532_SWAPL(tmp_data_32), (address + APPLICATION_COMMAND_REG));
		smp_wmb();
		udelay(2);
		
		//************ reset CSPTs ***************/
		tmp_data_32 = 0xF0;
		iowrite32(TAMC532_SWAPL(tmp_data_32), (address + APPLICATION_COMMAND_REG));
		smp_wmb();
		udelay(2);
		
		//************ disable the DMA controller ***************/
		for(i = 0; i < DMACNUM; i++){
			// disable the DMA controller 
			iowrite32( 0, ( address + (DMAC0_CONTROL_REG + i*0x10) ) );
			smp_wmb();
			iowrite32( 0, ( address + DMAC0_BASE_DESCR_ADDRESS_REG + i*0x10));
			smp_wmb();
			udelay(2);
		}
		
		//************ clear ABT DISSABLE ***************/
		for(i = 0; i < DMACNUM; i++){
			tmp_data_32       = ioread32(address + CSPTA_CONTROL_REG + i*0x10);
			smp_rmb();
			tmp_data_32 = TAMC532_SWAPL(tmp_data_32);
			tmp_data_32 &= 0xFFFFFFEF;
			iowrite32(TAMC532_SWAPL(tmp_data_32), (address + CSPTA_CONTROL_REG + i*0x10));
			smp_wmb();
		}
		
		//************ clear all buffers ***************/
		for(tmp_dma_offset  = 0; tmp_dma_offset < DMACNUM ; tmp_dma_offset++){
			for(i = 0; i < TAMC532_DMAC_MAX_NUM; i++){
				if(tamc532dev->pStrmDmaHandle[tmp_dma_offset][i]){
					pci_unmap_single(pdev, tamc532dev->pStrmDmaHandle[tmp_dma_offset][i], tamc532dev->dev_stream_dma_trans_size[tmp_dma_offset][i], PCI_DMA_FROMDEVICE);
					tamc532dev->pStrmDmaHandle[tmp_dma_offset][i] = 0;
				}
				if(tamc532dev->pStrmWriteBuf[tmp_dma_offset][i]){
					free_pages((ulong)tamc532dev->pStrmWriteBuf[tmp_dma_offset][i], (ulong)tamc532dev->stream_dma_order[tmp_dma_offset][i]);
					tamc532dev->pStrmWriteBuf[tmp_dma_offset][i] = 0;
				}

				if(tamc532dev->pStrmDmaHandle1[tmp_dma_offset][i]){
					pci_unmap_single(pdev, tamc532dev->pStrmDmaHandle1[tmp_dma_offset][i], tamc532dev->dev_stream_dma_trans_size[tmp_dma_offset][i], PCI_DMA_FROMDEVICE);
					tamc532dev->pStrmDmaHandle1[tmp_dma_offset][i] = 0;
				}
				if(tamc532dev->pStrmWriteBuf1[tmp_dma_offset][i]){
					free_pages((ulong)tamc532dev->pStrmWriteBuf1[tmp_dma_offset][i], (ulong)tamc532dev->stream_dma_order[tmp_dma_offset][i]);
					tamc532dev->pStrmWriteBuf1[tmp_dma_offset][i] = 0;
				}
				
				tamc532dev->dev_stream_dma_size[tmp_dma_offset][i]           = 0;
				tamc532dev->stream_dma_order[tmp_dma_offset][i]               = 0;
				tamc532dev->dev_stream_dma_trans_size[tmp_dma_offset][i] =  0;
			}
			tamc532dev->strm_dmac_done[tmp_dma_offset]          = 0;
			tamc532dev->strm_dmac_enbl[tmp_dma_offset]           = 0;		
			tamc532dev->irq_dmac_rcv[tmp_dma_offset]                = 0 ;
		}
		
		tamc532dev->dmac_strm_trg_num  = 0;
		tamc532dev->strm_dma_size  = 0;
		tamc532dev->strm_dma_cur_buf  = 0;
		tamc532dev->strm_buf_dma_size  = 0;
		tamc532dev->is_strm_dma  = 0;
		tamc532dev->is_strm_dma_run  = 0;
		tamc532dev->strm_dmac_num  = 0;
		tamc532dev->strm_dma_page_num  = 0;
		
		
		
		if ( copy_to_user((device_ioctrl_stream_dma*)arg, &stream_dma_data, (size_t)io_stream_dma_size) ) {
			retval = -EFAULT;
			mutex_unlock(&dev->dev_mut);
			return retval;
		}
		
		break;
		
	case TAMC532_READ_STATUS_REG:
		
/*
		struct device_staus_registers  {
			unsigned int   module_control_reg;
			unsigned int   module_status_reg;
			unsigned int   dma_control_reg[8];         
			unsigned int   dma_status_reg[8];     
			unsigned int   application_control_reg; 
			unsigned int   application_status_reg; 
			unsigned int   adc_channel_data[16]; 
		};
*/
		
		retval = 0;

		tmp_data_32       = ioread32(address + MODULE_CONTROL_REG);
		smp_rmb();
		tmp_data_32 = TAMC532_SWAPL(tmp_data_32);
		dev_status_reg_data.module_control_reg = tmp_data_32;
		
		tmp_data_32       = ioread32(address + MODULE_STATUS_REG);
		smp_rmb();
		tmp_data_32 = TAMC532_SWAPL(tmp_data_32);
		dev_status_reg_data.module_status_reg = tmp_data_32;
		
		tmp_data_32       = ioread32(address + APPLICATION_CONTROL_REG);
		smp_rmb();
		tmp_data_32 = TAMC532_SWAPL(tmp_data_32);
		dev_status_reg_data.application_control_reg = tmp_data_32;
		
		tmp_data_32       = ioread32(address + APPLICATION_STATUS_REG);
		smp_rmb();
		tmp_data_32 = TAMC532_SWAPL(tmp_data_32);
		dev_status_reg_data.application_status_reg = tmp_data_32;
		
		for(i = 0; i < 16; i++){
			if(i < 4){
				tmp_data_32       = ioread32(address + DMAC0_CONTROL_REG + i*0x10);
				smp_rmb();
				tmp_data_32 = TAMC532_SWAPL(tmp_data_32);
				dev_status_reg_data.dma_control_reg[i] = tmp_data_32;
				
				tmp_data_32       = ioread32(address + DMAC0_STATUS_REG + i*0x10);
				smp_rmb();
				tmp_data_32 = TAMC532_SWAPL(tmp_data_32);
				dev_status_reg_data.dma_status_reg[i] = tmp_data_32;
				
				tmp_data_32       = ioread32(address + CSPTA_CONTROL_REG + i*0x10);
				smp_rmb();
				tmp_data_32 = TAMC532_SWAPL(tmp_data_32);
				dev_status_reg_data.cspt_control[i] = tmp_data_32;
			}
			tmp_data_32       = ioread32(address + ADC_CHANNEL01_DATA_REG + i*4);
			smp_rmb();
			tmp_data_32 = TAMC532_SWAPL(tmp_data_32);
			dev_status_reg_data.adc_channel_data[i] = tmp_data_32;
		}

		if ( copy_to_user((device_staus_registers*)arg, &dev_status_reg_data, (size_t)size_status_reg) ) {
			retval = -EFAULT;
			mutex_unlock(&dev->dev_mut);
			return retval;
		}
		break;
		
	case TAMC532_GET_STATUS_REG:
		retval = 0;
		
		dev_status_reg_data.module_control_reg = tamc532dev->dev_sts_regs.module_control_reg;
		dev_status_reg_data.module_status_reg =tamc532dev->dev_sts_regs.module_status_reg;
		dev_status_reg_data.application_control_reg = tamc532dev->dev_sts_regs.application_control_reg;
		dev_status_reg_data.application_status_reg = tamc532dev->dev_sts_regs.application_status_reg;
		
		for(i = 0; i < 16; i++){
			if(i < 4){
				dev_status_reg_data.dma_control_reg[i] = tamc532dev->dev_sts_regs.dma_control_reg[i];
				dev_status_reg_data.dma_status_reg[i] = tamc532dev->dev_sts_regs.dma_status_reg[i];
			}
			dev_status_reg_data.adc_channel_data[i] = tamc532dev->dev_sts_regs.adc_channel_data[i];
		}

		if ( copy_to_user((device_staus_registers*)arg, &dev_status_reg_data, (size_t)size_status_reg) ) {
			retval = -EFAULT;
			mutex_unlock(&dev->dev_mut);
			return retval;
		}
		break;
		
	case TAMC532_GET_ADC_DATA:
		
/*
		struct device_adc_data_registers  {
			unsigned int   adc_channel_data[16]; 
		};
*/
		
		for(i = 0; i < 16; i++){
			tmp_data_32       = ioread32(address + ADC_CHANNEL01_DATA_REG + i*4);
			smp_rmb();
			tmp_data_32 = TAMC532_SWAPL(tmp_data_32);
			dev_adc_channel_data.adc_channel_data[i] = tmp_data_32;
		}
		
		if ( copy_to_user((device_adc_data_registers*)arg, &dev_adc_channel_data, (size_t)size_adc_data) ) {
			retval = -EFAULT;
			mutex_unlock(&dev->dev_mut);
			return retval;
		}
		break;
		
		
		
		
	case TAMC532_SET_CONT_DMA:
		retval = 0;

		if (copy_from_user(&stream_dma_data, (device_ioctrl_stream_dma*)arg, (size_t)io_stream_dma_size)) {
			retval = -EFAULT;
			mutex_unlock(&dev->dev_mut);
			return retval;
		}
		
		tmp_dma_cmd            = stream_dma_data.dma_cmd;
		tmp_stream_trg_num = stream_dma_data.dma_reserved1;
		tmp_dma_offset         = stream_dma_data.dma_offset & 0xF;
		tmp_dma_size             = stream_dma_data.dma_size; //size in B for one DMAC
		length                           = tmp_dma_size;
		tmp_new_dma_size    = tmp_dma_size;
		tmp_dma_buf_size       = stream_dma_data.dma_size * tmp_stream_trg_num;  //500 TRGs with 62usec will done ~30msec to fill the buffer
		
		tamc532dev->strm_dmac_num = 0;
		for(i = 0; i < DMACNUM; i++){
			tamc532dev->strm_dmac_enbl[i] = 0;
			tamc532dev->strm_trg_num[i] = 0;
			if( ((tmp_dma_offset>>i)&0x1 )){
				tamc532dev->strm_dmac_num += 1;
				tamc532dev->strm_dmac_enbl[i] = 1;
			}
		}
		
		printk (KERN_ALERT "TAMC532_SET_CONT_DMA: SAMPLE SIZE %i DMACs %X  DMACS_NUM %i TRG_NUM %i DMA_SIZE %i \n",
				tmp_dma_size, tmp_dma_offset, tamc532dev->strm_dmac_num, tmp_stream_trg_num, tmp_dma_buf_size);
		
		if(tmp_dma_size <= 0){
			mutex_unlock(&dev->dev_mut);
			printk (KERN_ALERT "TAMC532_SET_CONT_DMA: ERROR SIZE %i DMAC %X \n",tmp_dma_size, tmp_dma_offset);
			return -EFAULT;
		}
		
		if(tmp_dma_size  > TAMC532_DMAC_MAX_SIZE*4){
			mutex_unlock(&dev->dev_mut);
			printk (KERN_ALERT "TAMC532_SET_CONT_DMA: ERROR SIZE %i DMAC %X \n",tmp_dma_size, tmp_dma_offset);
			return -EFAULT;
		}
		
		if(tmp_dma_offset < 0){
			mutex_unlock(&dev->dev_mut);
			printk (KERN_ALERT "TAMC532_SET_CONT_DMA: ERROR OFFSET DMAC %X \n",tmp_dma_offset);
			return -EFAULT;
		}
		if(tmp_dma_offset > 0xF){
			mutex_unlock(&dev->dev_mut);
			printk (KERN_ALERT "TAMC532_SET_CONT: ERROR OFFSET DMAC %X \n",tmp_dma_offset);
			return -EFAULT;
		}
		
		tmp_data_32       = ioread32(address + CSPTA_DATA0_REG );
		smp_rmb();
		tmp_pre_sample = TAMC532_SWAPL(tmp_data_32);
		
		tmp_data_32       = ioread32(address + CSPTA_DATA1_REG);
		smp_rmb();
		tmp_post_sample = TAMC532_SWAPL(tmp_data_32);
		
		tmp_sample_num = tmp_pre_sample + tmp_post_sample;
		
		printk (KERN_ALERT "TAMC532_SET_CONT_DMA: PRE_SAMPLE %i POST_SAMPLE %i  SAMPLE_NUM %i \n",
				tmp_pre_sample, tmp_post_sample, tmp_sample_num);
		
		
		tamc532dev->strm_buf_dma_size =tmp_dma_buf_size;
		tamc532dev->strm_dma_size        = tmp_dma_size ; 
		
		//**SET NEW DMA**/
		
		
		tmp_free_pages        = nr_free_pages();
		tmp_free_pages        = tmp_free_pages << (PAGE_SHIFT-10);
		tmp_free_pages        = tmp_free_pages;
		if( (tmp_dma_buf_size*2*tamc532dev->strm_dmac_num) > tmp_free_pages*1000){
			for(i = 0; i < DMACNUM; i++){
				tamc532dev->strm_dmac_enbl[i] = 0;
				tamc532dev->strm_trg_num[i] = 0;
			}
			tamc532dev->strm_dmac_num = 0;
			tamc532dev->is_strm_dma = 0;
			tamc532dev->is_strm_dma_run = 0;
			tamc532dev->strm_dma_size = 0;
			tamc532dev->strm_buf_dma_size =0;
			mutex_unlock(&dev->dev_mut);
			printk (KERN_ALERT "TAMC532_SET_CONT_DMA: ERROR NO FREE PAGES SIZE %i: PAGES %i \n",  
					                                                                      tamc532dev->strm_dma_size, tmp_free_pages*1000);
			return -ENOMEM;
		}
		
		tamc532dev->strm_dma_page_num = 1;
		tmp_dma_buf_size =tmp_dma_buf_size;
		
/*
		int	 	         dev_stream_dma_size[TAMC532_DMAC_MAX_NUM];
		int                        dev_stream_dma_trans_size[TAMC532_DMAC_MAX_NUM];
		int                        stream_dma_order[TAMC532_DMAC_MAX_NUM];
		void*                    pStrmWriteBuf[2][TAMC532_DMAC_MAX_NUM] ;
		dma_addr_t        pStrmDmaHandle[2][TAMC532_DMAC_MAX_NUM];
*/
		tmp_dma_num = 1;
		
		for(tmp_dma_offset = 0; tmp_dma_offset < DMACNUM; tmp_dma_offset++){
			if(tamc532dev->strm_dmac_enbl[tmp_dma_offset]){
				printk (KERN_ALERT "TAMC532_SET_CONT_DMA: SET BUFFERS FOR DMAC %i BUF SIZE %i\n", tmp_dma_offset, tmp_dma_buf_size);
				tmp_dma_count    = 0;
				tmp_dma_count_size = tmp_dma_buf_size;
				
				//if(tmp_dma_count == 0 ) tmp_dma_buf_size = tmp_dma_size_rest;
				
				tamc532dev->dev_stream_dma_size[tmp_dma_offset][tmp_dma_count] = tmp_dma_buf_size;
				tmp_dma_trns_size   = tmp_dma_buf_size;

/*
				if((tmp_dma_buf_size%PCIEDEV_DMA_SYZE)){
					tmp_dma_buf_size    = tmp_dma_buf_size + (tmp_dma_buf_size%PCIEDEV_DMA_SYZE);
				}
*/

				tmp_dma_trns_size = tmp_dma_buf_size;
				tmp_order = get_order(tmp_dma_trns_size);
				tamc532dev->stream_dma_order[tmp_dma_offset][tmp_dma_count]                  = tmp_order;
				tamc532dev->dev_stream_dma_trans_size[tmp_dma_offset][tmp_dma_count]   =  tmp_dma_trns_size;

				if (!tamc532dev->pStrmWriteBuf[tmp_dma_offset][tmp_dma_count]){
					tamc532dev->pStrmWriteBuf[tmp_dma_offset][tmp_dma_count] = (void *)__get_free_pages(GFP_KERNEL | __GFP_DMA, tmp_order);
				}
				if (!tamc532dev->pStrmWriteBuf[tmp_dma_offset][tmp_dma_count]){
					printk (KERN_ALERT "------TAMC532_SET_CONT_DMA: NO MEMORY FOR BUF0 SIZE %i DMAC %i COUNT %i\n",tmp_dma_buf_size, tmp_dma_offset, tmp_dma_count);
					for(i = 0; i < tmp_dma_num; ++i){
						if(tamc532dev->pStrmWriteBuf[tmp_dma_offset][i])
							free_pages((ulong)tamc532dev->pStrmWriteBuf[tmp_dma_offset][i], (ulong)tamc532dev->stream_dma_order[tmp_dma_offset][i]);
						tamc532dev->dev_stream_dma_size[tmp_dma_offset][i]           = 0;
						tamc532dev->stream_dma_order[tmp_dma_offset][i]               = 0;
						tamc532dev->dev_stream_dma_trans_size[tmp_dma_offset][i] =  0;
						tamc532dev->pStrmWriteBuf[tmp_dma_offset][i] = 0;
					}
					mutex_unlock(&dev->dev_mut);
					return -EFAULT;
				}
				if (!tamc532dev->pStrmWriteBuf1[tmp_dma_offset][tmp_dma_count]){
					tamc532dev->pStrmWriteBuf1[tmp_dma_offset][tmp_dma_count] = (void *)__get_free_pages(GFP_KERNEL | __GFP_DMA, tmp_order);
				}
				if (!tamc532dev->pStrmWriteBuf1[tmp_dma_offset][tmp_dma_count]){
					printk (KERN_ALERT "------TAMC532_SET_CONT_DMA: NO MEMORY FOR BUF1 SIZE %i DMAC %i COUNT %i\n",tmp_dma_buf_size, tmp_dma_offset, tmp_dma_count);
					for(i = 0; i < tmp_dma_num; ++i){
						if(tamc532dev->pStrmWriteBuf1[tmp_dma_offset][i])
							free_pages((ulong)tamc532dev->pStrmWriteBuf1[tmp_dma_offset][i], (ulong)tamc532dev->stream_dma_order[tmp_dma_offset][i]);
						tamc532dev->dev_stream_dma_size[tmp_dma_offset][i]           = 0;
						tamc532dev->stream_dma_order[tmp_dma_offset][i]               = 0;
						tamc532dev->dev_stream_dma_trans_size[tmp_dma_offset][i] =  0;
						tamc532dev->pStrmWriteBuf1[tmp_dma_offset][i] = 0;
						if(tamc532dev->pStrmWriteBuf[tmp_dma_offset][i])
							free_pages((ulong)tamc532dev->pStrmWriteBuf[tmp_dma_offset][i], (ulong)tamc532dev->stream_dma_order[tmp_dma_offset][i]);
						tamc532dev->pStrmWriteBuf[tmp_dma_offset][i] = 0;
					}
					mutex_unlock(&dev->dev_mut);
					return -EFAULT;
				}
					
				//***************set DMA DESCRIPTORS*******************	
				tmp_dma_desc_size = sizeof(tamc532_dma_desc);

				if(!(tamc532dev->pStrmDmaHandle[tmp_dma_offset][tmp_dma_count] ))
					tamc532dev->pStrmDmaHandle[tmp_dma_offset][tmp_dma_count]      = pci_map_single(pdev, tamc532dev->pStrmWriteBuf[tmp_dma_offset][tmp_dma_count], tmp_dma_trns_size, PCI_DMA_FROMDEVICE);
				tmp_data_32         = ((tmp_dma_size/4) << 16);

				if(tmp_dma_count == 0){
					//tmp_data_32         |= TAMC532_DESCR_HI_EOL;
					tmp_data_32         |= TAMC532_DESCR_HI_DMF;
				}

				tmp_dma_desc.control      = cpu_to_be32((unsigned int)(tmp_data_32));                
				tmp_dma_desc.dataptr     = cpu_to_be32( (unsigned int)(tamc532dev->pStrmDmaHandle[tmp_dma_offset][tmp_dma_count]) );
				
				printk (KERN_ALERT "------TAMC532_SET_CONT_DMA: DATAPOINTER DMAC %i, CONTROL %i POINTER %X:%X\n",tmp_dma_offset, tmp_data_32, 
						tamc532dev->pStrmWriteBuf[tmp_dma_offset][tmp_dma_count] , tamc532dev->pStrmDmaHandle[tmp_dma_offset][tmp_dma_count]);
				
				if(tmp_dma_count > 0){
					//tmp_dma_desc.next_desc = cpu_to_be32((unsigned int)(tamc532dev->pStrmDmaHandle[tmp_dma_offset][tmp_dma_count -1]));
					tmp_dma_desc.next_desc = cpu_to_be32((unsigned int)(tamc532dev->pTmpDescHandle[tmp_dma_offset][tmp_dma_count -1]));
				}
				if(tmp_dma_count == 0){
					tmp_dma_desc.next_desc = 0x00000000;
				}
				printk (KERN_ALERT "------TAMC532_SET_CONT_DMA:MEMORY FOR DMA BOARD %i DESC0 %i:%i %X  \n", 
						tamc532dev->brd_num, tmp_dma_offset, tmp_dma_count, tamc532dev->pDescBuf[tmp_dma_offset][tmp_dma_count]);
				memcpy(tamc532dev->pDescBuf[tmp_dma_offset][tmp_dma_count], &tmp_dma_desc, TAMC532_DESC_SIZE);
				if(!(tamc532dev->pTmpDescHandle[tmp_dma_offset][tmp_dma_count] ))
					tamc532dev->pTmpDescHandle[tmp_dma_offset][tmp_dma_count]      = pci_map_single(pdev, tamc532dev->pDescBuf[tmp_dma_offset][tmp_dma_count], TAMC532_DESC_SIZE, PCI_DMA_TODEVICE);


				if(!(tamc532dev->pStrmDmaHandle1[tmp_dma_offset][tmp_dma_count] ))
					tamc532dev->pStrmDmaHandle1[tmp_dma_offset][tmp_dma_count]      = pci_map_single(pdev, tamc532dev->pStrmWriteBuf1[tmp_dma_offset][tmp_dma_count], tmp_dma_trns_size, PCI_DMA_FROMDEVICE);
				tmp_data_32         = ((tmp_dma_size/4) << 16);

				if(tmp_dma_count == 0){
					//tmp_data_32         |= TAMC532_DESCR_HI_EOL;
					tmp_data_32         |= TAMC532_DESCR_HI_DMF;
				}

				tmp_dma_desc.control      = cpu_to_be32((unsigned int)(tmp_data_32));                
				tmp_dma_desc.dataptr     = cpu_to_be32( (unsigned int)(tamc532dev->pStrmDmaHandle1[tmp_dma_offset][tmp_dma_count]) );
				
				printk (KERN_ALERT "------TAMC532_SET_CONT_DMA: DATAPOINTER DMAC %i, CONTROL %i POINTER %X:%X\n",tmp_dma_offset, tmp_data_32, 
						tamc532dev->pStrmWriteBuf1[tmp_dma_offset][tmp_dma_count] , tamc532dev->pStrmDmaHandle1[tmp_dma_offset][tmp_dma_count]);
				
				
				if(tmp_dma_count > 0){
					//tmp_dma_desc.next_desc = cpu_to_be32((unsigned int)(tamc532dev->pStrmDmaHandle1[tmp_dma_offset][tmp_dma_count -1]));
					tmp_dma_desc.next_desc = cpu_to_be32((unsigned int)(tamc532dev->pTmpDescHandle1[tmp_dma_offset][tmp_dma_count -1]));
				}
				if(tmp_dma_count == 0){
					tmp_dma_desc.next_desc = 0x00000000;
				}
				printk (KERN_ALERT "------TAMC532_SET_CONT_DMA:MEMORY FOR DMA BOARD %i DESC1 %i:%i %X  \n", 
						tamc532dev->brd_num, tmp_dma_offset, tmp_dma_count, tamc532dev->pDescBuf1[tmp_dma_offset][tmp_dma_count]);
				memcpy(tamc532dev->pDescBuf1[tmp_dma_offset][tmp_dma_count], &tmp_dma_desc, TAMC532_DESC_SIZE);
				if(!(tamc532dev->pTmpDescHandle1[tmp_dma_offset][tmp_dma_count] ))
					tamc532dev->pTmpDescHandle1[tmp_dma_offset][tmp_dma_count]      = pci_map_single(pdev, tamc532dev->pDescBuf1[tmp_dma_offset][tmp_dma_count], TAMC532_DESC_SIZE, PCI_DMA_TODEVICE);
			}
		}
		
		
		
		//************ disable CSPTs ***************/
		iowrite32( 0, ( address + APPLICATION_CONTROL_REG ) );
		smp_wmb();
		udelay(2);
		
		//************ FLASH DMA data and terminate current DMA ***************/
		tmp_data_32 = 0xF00;
		iowrite32(TAMC532_SWAPL(tmp_data_32), (address + APPLICATION_COMMAND_REG));
		smp_wmb();
		udelay(2);
		
		//************ reset CSPTs ***************/
		tmp_data_32 = 0xF0;
		iowrite32(TAMC532_SWAPL(tmp_data_32), (address + APPLICATION_COMMAND_REG));
		smp_wmb();
		udelay(2);
		
		//************ disable the DMA controller ***************/
		for(i = 0; i < DMACNUM; i++){
			// disable the DMA controller 
			iowrite32( 0, ( address + (DMAC0_CONTROL_REG + i*0x10) ) );
			smp_wmb();
			iowrite32( 0, ( address + DMAC0_BASE_DESCR_ADDRESS_REG + i*0x10));
			smp_wmb();
			udelay(2);
		}
		
		tamc532dev->server_signal_stack [0].f_ServerPref = cur_proc;
		tamc532dev->is_strm_dma         = 2;
		tamc532dev->is_strm_dma_run = 1;
		tamc532dev->strm_dmac_num  = 0;
		tamc532dev->strm_dma_cur_buf = 0;
		tamc532dev->dmac_strm_trg_num      = tmp_stream_trg_num;
		tamc532dev->strm_trg_num[0] = 0;
		
		printk(KERN_ALERT "TAMC532_SET_CONT_DMA TRG_NUM %i TRG_DONE %i\n", tamc532dev->dmac_strm_trg_num, tamc532dev->strm_trg_num[0]);
		
		
		//************ set GlobalInterrupt to allow EOL/DMF interrupt ***************/
		tmp_data_32       = ioread32(address + MODULE_INTERRUPT_ENABLE_REG);
		smp_rmb();
		tmp_data_32       = TAMC532_SWAPL(tmp_data_32);
		for(i = 0; i < DMACNUM; i++){
			if( tamc532dev->strm_dmac_enbl[i]){
				//tmp_data_32       |=  TAMC532_MODULEINT_DMACx_EOL(i) ;		
				tmp_data_32       |=  TAMC532_MODULEINT_DMACx_DMF(i) ;
		
			}
		}
		//iowrite32( 0, ( address + MODULE_INTERRUPT_ENABLE_REG ));
		iowrite32( TAMC532_SWAPL(tmp_data_32), ( address + MODULE_INTERRUPT_ENABLE_REG ));
		smp_wmb();
		udelay(2);
		
		//************ set Descriptor Base Address Register to Buffer0 and enable DMACs and CSPTs ***************/
		tmp_dma_offset = 0;
		for(i = 0; i < DMACNUM; i++){
			if( tamc532dev->strm_dmac_enbl[i]){
				tmp_dma_offset += 1<<i;
				tmp_data_32       = tamc532dev->pTmpDescHandle[i][tmp_dma_num -1];
				iowrite32( cpu_to_be32((unsigned int)(tmp_data_32)), ( address + DMAC0_BASE_DESCR_ADDRESS_REG + i*0x10));
				smp_wmb();
				udelay(20);
				printk (KERN_ALERT "------TAMC532_SET_STREAM_DMA: DESC BUF ADDRESS %X :: %X\n",tmp_data_32, cpu_to_be32((unsigned int)(tmp_data_32)));
				
				tamc532dev->strm_dmac_done[i] = 0; 
				tamc532dev->strm_dmac_enbl[i] =1;
				
				iowrite32( TAMC532_SWAPL(0x1), ( address + (DMAC0_CONTROL_REG + i*0x10)));
				smp_wmb();
			}
		}
		iowrite32( TAMC532_SWAPL(tmp_dma_offset), ( address + APPLICATION_CONTROL_REG ) );
		smp_wmb();
		udelay(2);
		
		
		for(i = 0; i < 4; i++){
			tmp_data_32       = ioread32(address + DMAC0_STATUS_REG + i*0x10);
			smp_rmb();
			tmp_data_32 = TAMC532_SWAPL(tmp_data_32);
			stream_dma_data.pre_dma_status_reg[i] = tmp_data_32;
			
			tmp_data_32       = ioread32(address + DMAC0_CONTROL_REG + i*0x10);
			smp_rmb();
			tmp_data_32 = TAMC532_SWAPL(tmp_data_32);
			stream_dma_data.dma_control_reg[i] = tmp_data_32;
			
			tmp_data_32       = ioread32(address + CSPTA_CONTROL_REG + i*0x10);
			smp_rmb();
			tmp_data_32 = TAMC532_SWAPL(tmp_data_32);
			stream_dma_data.cspt_control[i] = tmp_data_32;
		}
		if ( copy_to_user((device_ioctrl_stream_dma*)arg, &stream_dma_data, (size_t)io_stream_dma_size) ) {
			retval = -EFAULT;
			mutex_unlock(&dev->dev_mut);
			return retval;
		}
		
		break;
	case TAMC532_REMOVE_CONT_DMA:
		retval = 0;

		if (copy_from_user(&stream_dma_data, (device_ioctrl_stream_dma*)arg, (size_t)io_stream_dma_size)) {
			retval = -EFAULT;
			mutex_unlock(&dev->dev_mut);
			return retval;
		}
		
		tmp_dma_offset        = stream_dma_data.dma_offset & 0xF;
		tmp_dma_size            = stream_dma_data.dma_size;
		tmp_dma_buf_size     = stream_dma_data.dma_size * TAMC532_STRM_TRQ_NUM;  //500 TRGs with 62usec will done ~30msec to fill the buffer
		length                          = tmp_dma_size;
		tmp_new_dma_size   = tmp_dma_size;
		tmp_dma_cmd           = stream_dma_data.dma_cmd;
		
		printk (KERN_ALERT "+++++++++++++++TAMC532_REMOVE_CONT_DMA\n");
		tamc532dev->is_strm_dma         = 0;
		tamc532dev->is_strm_dma_run = 0;
		
		iowrite32( 0, ( address + MODULE_INTERRUPT_ENABLE_REG ));
		smp_wmb();
		udelay(2);
		
		for(i = 0; i < 4; i++){
			tmp_data_32       = ioread32(address + DMAC0_STATUS_REG + i*0x10);
			smp_rmb();
			tmp_data_32 = TAMC532_SWAPL(tmp_data_32);
			stream_dma_data.pre_dma_status_reg[i] = tmp_data_32;
			
			tmp_data_32       = ioread32(address + DMAC0_CONTROL_REG + i*0x10);
			smp_rmb();
			tmp_data_32 = TAMC532_SWAPL(tmp_data_32);
			stream_dma_data.dma_control_reg[i] = tmp_data_32;
			
			tmp_data_32       = ioread32(address + CSPTA_CONTROL_REG + i*0x10);
			smp_rmb();
			tmp_data_32 = TAMC532_SWAPL(tmp_data_32);
			stream_dma_data.cspt_control[i] = tmp_data_32;
		}
		
		//************ disable CSPTs ***************/
		iowrite32( 0, ( address + APPLICATION_CONTROL_REG ) );
		smp_wmb();
		udelay(2);
		
		//************ FLASH DMA dat and terminate current DMA ***************/
		tmp_data_32 = 0xF00;
		iowrite32(TAMC532_SWAPL(tmp_data_32), (address + APPLICATION_COMMAND_REG));
		smp_wmb();
		udelay(2);
		
		//************ reset CSPTs ***************/
		tmp_data_32 = 0xF0;
		iowrite32(TAMC532_SWAPL(tmp_data_32), (address + APPLICATION_COMMAND_REG));
		smp_wmb();
		udelay(2);
		
		//************ disable the DMA controller ***************/
		for(i = 0; i < DMACNUM; i++){
			// disable the DMA controller 
			iowrite32( 0, ( address + (DMAC0_CONTROL_REG + i*0x10) ) );
			smp_wmb();
			iowrite32( 0, ( address + DMAC0_BASE_DESCR_ADDRESS_REG + i*0x10));
			smp_wmb();
			udelay(2);
		}
				
		//************ clear all buffers ***************/
		for(tmp_dma_offset  = 0; tmp_dma_offset < DMACNUM ; tmp_dma_offset++){
			for(i = 0; i < TAMC532_DMAC_MAX_NUM; i++){
				if(tamc532dev->pStrmDmaHandle[tmp_dma_offset][i]){
					pci_unmap_single(pdev, tamc532dev->pStrmDmaHandle[tmp_dma_offset][i], tamc532dev->dev_stream_dma_trans_size[tmp_dma_offset][i], PCI_DMA_FROMDEVICE);
					tamc532dev->pStrmDmaHandle[tmp_dma_offset][i] = 0;
				}
				if(tamc532dev->pStrmWriteBuf[tmp_dma_offset][i]){
					free_pages((ulong)tamc532dev->pStrmWriteBuf[tmp_dma_offset][i], (ulong)tamc532dev->stream_dma_order[tmp_dma_offset][i]);
					tamc532dev->pStrmWriteBuf[tmp_dma_offset][i] = 0;
				}

				if(tamc532dev->pStrmDmaHandle1[tmp_dma_offset][i]){
					pci_unmap_single(pdev, tamc532dev->pStrmDmaHandle1[tmp_dma_offset][i], tamc532dev->dev_stream_dma_trans_size[tmp_dma_offset][i], PCI_DMA_FROMDEVICE);
					tamc532dev->pStrmDmaHandle1[tmp_dma_offset][i] = 0;
				}
				if(tamc532dev->pStrmWriteBuf1[tmp_dma_offset][i]){
					free_pages((ulong)tamc532dev->pStrmWriteBuf1[tmp_dma_offset][i], (ulong)tamc532dev->stream_dma_order[tmp_dma_offset][i]);
					tamc532dev->pStrmWriteBuf1[tmp_dma_offset][i] = 0;
				}
				
				tamc532dev->dev_stream_dma_size[tmp_dma_offset][i]           = 0;
				tamc532dev->stream_dma_order[tmp_dma_offset][i]               = 0;
				tamc532dev->dev_stream_dma_trans_size[tmp_dma_offset][i] =  0;
			}
			tamc532dev->strm_dmac_done[tmp_dma_offset]          = 0;
			tamc532dev->strm_dmac_enbl[tmp_dma_offset]           = 0;		
			tamc532dev->irq_dmac_rcv[tmp_dma_offset]                = 0 ;
			tamc532dev->strm_trg_num[i]				       = 0;
		}
		
		tamc532dev->dmac_strm_trg_num  = 0;
		tamc532dev->strm_dma_size  = 0;
		tamc532dev->strm_dma_cur_buf  = 0;
		tamc532dev->strm_buf_dma_size  = 0;
		tamc532dev->is_strm_dma  = 0;
		tamc532dev->is_strm_dma_run  = 0;
		tamc532dev->strm_dmac_num  = 0;
		tamc532dev->strm_dma_page_num  = 0;
		
		
		
		if ( copy_to_user((device_ioctrl_stream_dma*)arg, &stream_dma_data, (size_t)io_stream_dma_size) ) {
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

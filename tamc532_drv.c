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

#include <linux/module.h>
#include <linux/fs.h>	
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/timer.h>

#include "pciedev_io.h"
#include "pciedev_ufn.h"
#include "tamc532_io.h"
#include "tamc532_fnc.h"
#include "tamc532_reg.h"

MODULE_AUTHOR("Ludwig Petrosyan (ludwig.petrosyan@desy.de)");
MODULE_DESCRIPTION("TAMC532 board driver");
MODULE_VERSION("3.0.0");
MODULE_LICENSE("Dual BSD/GPL");

pciedev_cdev     *tamc532_cdev_m = 0;
tamc532_dev       *tamc532_dev_p[PCIEDEV_NR_DEVS];
tamc532_dev       *tamc532_dev_pp;

static int        tamc532_open( struct inode *inode, struct file *filp );
static int        tamc532_release(struct inode *inode, struct file *filp);
static ssize_t tamc532_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos);
static ssize_t tamc532_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos);
static long     tamc532_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
static int        tamc532_remap_mmap(struct file *filp, struct vm_area_struct *vma);

struct file_operations tamc532_fops = {
    .owner                   =  THIS_MODULE,
    .read                     =  tamc532_read,
    .write                    =  tamc532_write,
    .unlocked_ioctl    =  tamc532_ioctl,
    .open                    =  tamc532_open,
    .release                =  tamc532_release,
    .mmap                 = tamc532_remap_mmap,
};

static struct pci_device_id tamc532_ids[] = {
    { PCI_DEVICE(TAMC532_VENDOR_ID, TAMC532_DEVICE_ID), },
    { 0, }
};
MODULE_DEVICE_TABLE(pci, tamc532_ids);

/*
 * The top-half interrupt handler.
 */
#if LINUX_VERSION_CODE < 0x20613 // irq_handler_t has changed in 2.6.19
static irqreturn_t tamc532_interrupt(int irq, void *dev_id, struct pt_regs *regs)
#else
static irqreturn_t tamc532_interrupt(int irq, void *dev_id)
#endif
{
    uint32_t                   intreg     = 0;
    unsigned long          flags       = 0;
    int                            intcount = 0;
    int                            unit = 0;
    void*                        address;
    int                             i = 0;

    struct pciedev_dev *pdev   = (pciedev_dev*)dev_id;
    struct tamc532_dev *dev     = (tamc532_dev *)(pdev->dev_str);
    
    spin_lock_irqsave(&(dev->irq_lock), flags);
        address    = pciedev_get_baraddress(BAR0, pdev);
        intreg       = ioread32(address + MODULE_INTERRUPT_STAUS_REG);
        smp_rmb();
        intreg       =TAMC532_SWAPL(intreg);
/*
        printk(KERN_ALERT "TAMC532-INTERRUPT STATUS %X \n", intreg);
        for(i = 0; i < DMACNUM; i++){
                printk(KERN_ALERT"###############TAMC532: dma_done %i dmac %i.\n", dev->dmac_done[i], i);
        }
*/
        dev->irqsts = intreg;
        if(intreg == 0){
            spin_unlock_irqrestore(&(dev->irq_lock), flags);
            return IRQ_NONE;
        }
    
        /* acknowledge interrupts */
        iowrite32(TAMC532_SWAPL(intreg), (address + MODULE_INTERRUPT_STAUS_REG));
        smp_wmb();
        if (intreg & TAMC532_MODULEINT_DMACx_EOL_ALL){
            dev->waitFlag = 1;
            //wake_up_interruptible(&(dev->waitDMA));
            //wake_up(&(dev->waitDMA));
            for(i = 0; i < DMACNUM; i++){
                unit = (intreg >> (10 + i*4)) & 0x1;
                if(unit){
                    iowrite32( 0, ( address + (DMAC0_CONTROL_REG + i*0x10)));
                    smp_wmb();
                    dev->dmac_done[i] = 1;                    
                    //printk(KERN_ALERT"###############TAMC532: Interrupt for DMAC Num %i.\n", i);
                }
            }
        }
   
#if 0    
    for (;;)
    {

        //address    = pciedev_get_baraddress(BAR0, pdev);
        intreg       = ioread32(address + MODULE_INTERRUPT_STAUS_REG);
        printk(KERN_ALERT "TAMC532-INTERRUPT STATUS %X \n", intreg);
        smp_rmb();
        intreg       =TAMC532_SWAPL(intreg);
        if (intreg == 0)
        {
            break;
        }

        /* acknowledge interrupts */
        intreg       =TAMC532_SWAPL(intreg);
        iowrite32(intreg, (address + MODULE_INTERRUPT_STAUS_REG));
        smp_wmb();
        intcount++;
        if (intcount > 20)
        {
            /* this might be a real problem, so disable all interrupts and exit */
            iowrite32(0, (address + MODULE_INTERRUPT_ENABLE_REG));
            smp_wmb();
            iowrite32(0xFFFFFFFF, (address + MODULE_INTERRUPT_STAUS_REG));
            smp_wmb();
            iowrite32(TAMC532_SWAPL(TAMC532_MODULECONTROL_MODINTGE), (address + MODULE_CONTROL_REG));
            smp_wmb();
            printk(KERN_ALERT"!!!!!!!!!!!!!!!!!TAMC532: Interrupt cannot be cleared after 20 cycles! Interrupts Disabled.\n");
        }


        /* handle the detected interrupts */
        if (intreg & TAMC532_MODULEINT_I2C){
            /* handle I2C interrupt */
            dev->i2cWaitFlag = 1;
            wake_up_interruptible(&(dev->waitI2C));
        }
        
         if (intreg & TAMC532_MODULEINT_BCC)
        {
            printk("[BCC]\n");
        }
        

        
        /* handle DMAC interrupts */
        
        if (intreg & TAMC532_MODULEINT_DMACx_EOL_ALL){
            dev->waitFlag = 1;
            for(i = 0; i < DMACNUM; i++){
				unit = (intreg >> (10 + i*4)) & 0x1;
				if(unit){
					iowrite32( 0, ( address + (DMAC0_CONTROL_REG + i*0x10)));
                    smp_wmb();
					printk(KERN_ALERT"###############TAMC532: Interrupt cannot for DMAC Num %i.\n", i);
				}
			  }
            //wake_up_interruptible(&(dev->waitDMA));
            //wake_up(&(dev->waitDMA));
        }
 
        for (unit=0; unit<4; unit++)
        {
            if (intreg & TAMC532_MODULEINT_DMACx_DMF(unit))
            {
                //printk("[DMF #%d]\n", unit);
            }
            if (intreg & TAMC532_MODULEINT_DMACx_DDL(unit))
            {
                //printk("[DDL #%d]\n", unit);
            }
            if (intreg & TAMC532_MODULEINT_DMACx_EOL(unit))
            {
                dev->waitFlag = 1;
                //wake_up_interruptible(&(dev->waitDMA));
                wake_up(&(dev->waitDMA));
            }
        }
       

    }
#endif
    spin_unlock_irqrestore(&(dev->irq_lock), flags);
    return IRQ_HANDLED;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0)
    static int tamc532_probe(struct pci_dev *dev, const struct pci_device_id *id)
#else 
static int __devinit tamc532_probe(struct pci_dev *dev, const struct pci_device_id *id)
#endif
{
    int i                       = 0;
    int result                = 0;
    int unit                   = 0;
    int tmp_brd_num  = -1;
    u32 tmp_info          = 0;
    u32                    tmp_data_32;
    u8                      tmp_data_8;
    device_i2c_rw     i2c_rw_buf;
    pciedev_dev       *tamc532_pcie_dev;
    void*                   address;
    
    u_int  ptrn  ;
    u_char *lptr ;
    
    ptrn  = 0x01020304;
    lptr = (unsigned char*)&ptrn;
    
    printk(KERN_ALERT "TAMC532-PCIEDEV_PROBE CALLED \n");
    result = pciedev_probe_exp(dev, id, &tamc532_fops, tamc532_cdev_m, TAMC532DEVNAME, &tmp_brd_num);
    printk(KERN_ALERT "TAMC532-PCIEDEV_PROBE_EXP CALLED  FOR BOARD %i result %i\n", tmp_brd_num, result);
    if(!(tamc532_cdev_m->pciedev_dev_m[tmp_brd_num]->pciedev_all_mems)){
        printk(KERN_ALERT "TAMC532-PCIEDEV_PROBE CALLED; NO BARs \n");
        result = pciedev_remove_exp(dev,  tamc532_cdev_m, TAMC532DEVNAME, &tmp_brd_num);
        printk(KERN_ALERT "TAMC532-PCIEDEV_REMOVE_EXP CALLED  FOR SLOT %i\n", tmp_brd_num);  
        return -ENOMEM;
    }
    /*if board has created we will create our structure and pass it to pcedev_dev*/
    if(!result){
        printk(KERN_ALERT "TAMC532-PCIEDEV_PROBE_EXP CREATING CURRENT STRUCTURE FOR BOARD %i\n", tmp_brd_num);
        tamc532_pcie_dev = tamc532_cdev_m->pciedev_dev_m[tmp_brd_num];
        tamc532_dev_pp = kzalloc(sizeof(tamc532_dev), GFP_KERNEL);
        if(!tamc532_dev_pp){
                return -ENOMEM;
        }
        printk(KERN_ALERT "TAMC532-PCIEDEV_PROBE CALLED; CURRENT STRUCTURE CREATED \n");
        
       
        printk(KERN_ALERT "TAMC532-PCIEDEV_PROBE CALLED; CHECK FOR L_ENDIAN \n");
       
        tamc532_dev_p[tmp_brd_num] = tamc532_dev_pp;
        tamc532_dev_pp->brd_num      = tmp_brd_num;
        tamc532_dev_pp->parent_dev  = tamc532_cdev_m->pciedev_dev_m[tmp_brd_num];
        init_waitqueue_head(&tamc532_dev_pp->waitDMA);
        init_waitqueue_head(&tamc532_dev_pp->waitI2C);
        pciedev_set_drvdata(tamc532_cdev_m->pciedev_dev_m[tmp_brd_num], tamc532_dev_p[tmp_brd_num]);
        //pciedev_setup_interrupt(tamc532_interrupt, tamc532_cdev_m->pciedev_dev_m[tmp_brd_num], TAMC532DEVNAME); 
        pciedev_setup_interrupt(tamc532_interrupt, tamc532_cdev_m->pciedev_dev_m[tmp_brd_num], TAMC532DEVNAME, 0); 
        
        /*****Switch ON USER_LED*****/
        address = pciedev_get_baraddress(BAR0, tamc532_pcie_dev);
        /* disable interrupts */
         iowrite32(0, (address + MODULE_INTERRUPT_ENABLE_REG));
         smp_wmb();
         /**check to swap data**/
        if(cpu_to_le32(0x12345678) == 0x12345678 ) {
            tamc532_cdev_m->pciedev_dev_m[tmp_brd_num]->swap = 1;
            tamc532_dev_pp->lendian = 1;
            printk(KERN_ALERT "TAMC532-PCIEDEV_PROBE CALLED; THS CPU IS  L_ENDIAN \n");
        }
        else{
            tamc532_cdev_m->pciedev_dev_m[tmp_brd_num]->swap = 0;
            tamc532_dev_pp->lendian = 0;
            printk(KERN_ALERT "TAMC532-PCIEDEV_PROBE CALLED; THE CPU IS B_ENDIAN \n");
        }
        spin_lock_init(&tamc532_dev_pp->irq_lock);
        /*Collect INFO*/
        tmp_info = ioread32(address + FIRMWARE_ID_REG);
        smp_rmb();
        tmp_info = TAMC532_SWAPL(tmp_info);
        tamc532_cdev_m->pciedev_dev_m[tmp_brd_num]->brd_info_list.PCIEDEV_BOARD_ID = (tmp_info >>16) & 0xFFFF;
        printk(KERN_ALERT "TAMC532-PCIEDEV_PROBE CALLED: FW VERSION %X \n", (tmp_info >>16) & 0xFFFF);
        tamc532_cdev_m->pciedev_dev_m[tmp_brd_num]->brd_info_list.PCIEDEV_BOARD_VERSION = (tmp_info >> 8)  & 0xFF;
        printk(KERN_ALERT "TAMC532-PCIEDEV_PROBE CALLED: FW REVISION %X \n",  (tmp_info >> 8)  & 0xFF);
        tamc532_cdev_m->pciedev_dev_m[tmp_brd_num]->brd_info_list.PCIEDEV_HW_VERSION = tmp_info & 0xFF;
        printk(KERN_ALERT "TAMC532-PCIEDEV_PROBE CALLED: FW BUID CPUNT %X \n", tmp_info & 0xFF);
        
        /****initialize hardware****/
        
        iowrite32(TAMC532_SWAPL(TAMC532_MODULECONTROL_MODINTGE), (address + MODULE_CONTROL_REG));
        smp_wmb();
        
        tmp_data_32 = 0;
        tmp_data_32 = TAMC532_SWAPL(TAMC532_I2CCOMMAND_RD_DFIFO_RST | TAMC532_I2CCOMMAND_WR_DFIFO_RST 
                                | TAMC532_I2CCOMMAND_I2C_MST_RST);   /* reset I2C bridge */
        iowrite32(tmp_data_32, (address + I2C_BRIDGE_CONTROL_REG));
        smp_wmb();
        
        /*** Determine the machine's endianess, and setup the DMA controllers accordingly** */
        for (unit=0; unit<4; unit++){
            tmp_data_32 = ioread32(address + (CSPTA_CONTROL_REG + 16*unit));
            tmp_data_32 = TAMC532_SWAPL(tmp_data_32);
            smp_rmb();
            tmp_data_32 &= (unsigned int)~(TAMC532_CSPTCONTROL_DEM);
            if (cpu_to_le32(0x12345678) == 0x12345678){
                /* this is a little-endian machine, so the DMA controller must swap (the TAMC532 is big-endian). */
                tmp_data_32 |= TAMC532_CSPTCONTROL_DEM;
            }
            tmp_data_32 = TAMC532_SWAPL(tmp_data_32);
            iowrite32(tmp_data_32, (address + (CSPTA_CONTROL_REG + 16*unit)));
            smp_wmb();
        }
        
        printk(KERN_ALERT "TAMC532-PCIEDEV_PROBE CALLED: START I2C READ \n");
                
        i2c_rw_buf.busNum = 0x0;
        i2c_rw_buf.devAddr = 0x68;
        i2c_rw_buf.regAddr = 0xFC;
        i2c_rw_buf.size        = 4;
        tamc532_i2c_read(tamc532_cdev_m->pciedev_dev_m[tmp_brd_num], &i2c_rw_buf);
        for(i = 0; i < 4; i++){
            printk(KERN_ALERT "TAMC532-PCIEDEV_PROBE CALLED: I2C %i READ DATA %X\n", i, i2c_rw_buf.data[i]);
        }
        
        iowrite32( TAMC532_SWAPL(0xF0), ( address +APPLICATION_COMMAND_REG));
        smp_wmb();
        iowrite32( 0, ( address + DMAC0_CONTROL_REG  ) );
        smp_wmb();
        iowrite32( 0, ( address + DMAC1_CONTROL_REG  ) );
        smp_wmb();
        iowrite32( 0, ( address + DMAC2_CONTROL_REG  ) );
        smp_wmb();
        iowrite32( 0, ( address + DMAC3_CONTROL_REG  ) );
        smp_wmb();
        iowrite32( 0, ( address + DMAC0_BASE_DESCR_ADDRESS_REG ));
        smp_wmb();
        iowrite32( 0, ( address + DMAC1_BASE_DESCR_ADDRESS_REG ));
        smp_wmb();
        iowrite32(0, ( address + DMAC2_BASE_DESCR_ADDRESS_REG));
        smp_wmb();
        iowrite32( 0, ( address + DMAC3_BASE_DESCR_ADDRESS_REG));
        smp_wmb();
        iowrite32( TAMC532_SWAPL(0xF0), ( address +APPLICATION_COMMAND_REG));
        smp_wmb();
        for(unit = 0; unit < DMACNUM; unit++){
            for(i = 0; i < TAMC532_DMAC_MAX_NUM; i++){
                tamc532_dev_pp->pWriteBuf[unit][i]           = 0;
                tamc532_dev_pp->pDescBuf[unit][i]             = 0;
                tamc532_dev_pp->pTmpDmaHandle[unit][i] = 0;
                tamc532_dev_pp->pTmpDescHandle[unit][i] = 0;
            }
        }
     }
    return result;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0)
static void tamc532_remove(struct pci_dev *dev)
#else
static void __devexit tamc532_remove(struct pci_dev *dev)
#endif
{
    int dmacnum        = 0;
    int descnum         = 0;
    int result               = 0;
    int tmp_slot_num = -1;
    int tmp_brd_num = -1;
    printk(KERN_ALERT "TAMC532-REMOVE CALLED\n");
    tmp_brd_num =pciedev_get_brdnum(dev);
    printk(KERN_ALERT "TAMC532-REMOVE CALLED FOR BOARD %i\n", tmp_brd_num);
    
    
    for(dmacnum  = 0; dmacnum < DMACNUM ; dmacnum++){
        for(descnum = 0; descnum < TAMC532_DMAC_MAX_NUM; descnum++)
            if(tamc532_dev_p[tmp_brd_num]->pTmpDescHandle[dmacnum][descnum]){
                pci_unmap_single(dev, tamc532_dev_p[tmp_brd_num]->pTmpDescHandle[dmacnum][descnum], tamc532_dev_p[tmp_brd_num]->dev_dma_desc_size[dmacnum][descnum], PCI_DMA_TODEVICE);
                free_pages((ulong)tamc532_dev_p[tmp_brd_num]->pDescBuf[dmacnum][descnum], (ulong)tamc532_dev_p[tmp_brd_num]->dma_desc_order[dmacnum][descnum]);
                tamc532_dev_p[tmp_brd_num]->pTmpDescHandle[dmacnum][descnum] = 0;
                tamc532_dev_p[tmp_brd_num]->pDescBuf[dmacnum][descnum]  = 0;
            }
            if(tamc532_dev_p[tmp_brd_num]->pTmpDmaHandle[dmacnum][descnum]){
                pci_unmap_single(dev, tamc532_dev_p[tmp_brd_num]->pTmpDmaHandle[dmacnum][descnum], tamc532_dev_p[tmp_brd_num]->dev_dma_trans_size[dmacnum][descnum], PCI_DMA_FROMDEVICE);
                free_pages((ulong)tamc532_dev_p[tmp_brd_num]->pWriteBuf[dmacnum][descnum], (ulong)tamc532_dev_p[tmp_brd_num]->dma_order[dmacnum][descnum]);
                tamc532_dev_p[tmp_brd_num]->pTmpDmaHandle[dmacnum][descnum] = 0;
                tamc532_dev_p[tmp_brd_num]->pWriteBuf[dmacnum][descnum] = 0;
                tamc532_dev_p[tmp_brd_num]->dev_dma_size[dmacnum][descnum]           = 0;
                tamc532_dev_p[tmp_brd_num]->dma_order[dmacnum][descnum]               = 0;
                tamc532_dev_p[tmp_brd_num]->dev_dma_trans_size[dmacnum][descnum] =  0;
                tamc532_dev_p[tmp_brd_num]->dev_dma_desc_size[dmacnum][descnum]  = 0;
            }
        }
    
    /* clean up any allocated resources and stuff here */
    kfree(tamc532_dev_p[tmp_brd_num]);
    /*now we can call pciedev_remove_exp to clean all standard allocated resources
    will clean all interrupts if it seted 
    */
    result = pciedev_remove_exp(dev,  tamc532_cdev_m, TAMC532DEVNAME, &tmp_slot_num);
    printk(KERN_ALERT "TAMC532-PCIEDEV_REMOVE_EXP CALLED  FOR SLOT %i\n", tmp_slot_num);  
}

/****************************************************************************************/
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0)
static struct pci_driver pci_tamc532_driver = {
    .name       = TAMC532DEVNAME,
    .id_table   = tamc532_ids,
    .probe      = tamc532_probe,
    .remove   = tamc532_remove,
};
#else
static struct pci_driver pci_tamc532_driver = {
    .name       = TAMC532DEVNAME,
    .id_table   = tamc532_ids,
    .probe      = tamc532_probe,
    .remove   = __devexit_p(tamc532_remove),
};
#endif

static int tamc532_open( struct inode *inode, struct file *filp )
{
    int    result = 0;
    result = pciedev_open_exp( inode, filp );
    return result;
}

static int tamc532_release(struct inode *inode, struct file *filp)
{
     int result            = 0;
     result = pciedev_release_exp(inode, filp);
     return result;
} 

static ssize_t tamc532_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    ssize_t    retval         = 0;
    retval  = pciedev_read_exp(filp, buf, count, f_pos);
    return retval;
}

static ssize_t tamc532_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
    ssize_t         retval = 0;
    retval = pciedev_write_exp(filp, buf, count, f_pos);
    return retval;
}

static int tamc532_remap_mmap(struct file *filp, struct vm_area_struct *vma)
{
	ssize_t         retval = 0;
	//printk(KERN_ALERT "PCIEDEV_MMAP CALLED\n");
	retval =pciedev_remap_mmap_exp(filp, vma);
	//printk(KERN_ALERT "PCIEDEV_MMAP_EXP CALLED\n");
	return 0;
}

static long  tamc532_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    long result = 0;
    if (_IOC_TYPE(cmd) == PCIEDOOCS_IOC){
	if (_IOC_NR(cmd) <= PCIEDOOCS_IOC_MAXNR && _IOC_NR(cmd) >= PCIEDOOCS_IOC_MINNR) {
		if (_IOC_NR(cmd) <= PCIEDOOCS_IOC_DMA_MAXNR && _IOC_NR(cmd) >= PCIEDOOCS_IOC_DMA_MINNR) {
			result = tamc532_ioctl_dma(filp, &cmd, &arg);
		}else{
			result = pciedev_ioctl_exp(filp, &cmd, &arg, tamc532_cdev_m);
		}
	}else{
		if (_IOC_NR(cmd) <= TAMC532_IOC_MAXNR && _IOC_NR(cmd) >= TAMC532_IOC_MINNR) {
			result = tamc532_ioctl_dma(filp, &cmd, &arg);
		}else{
			return -ENOTTY;
		}
        }
    }else{
         return -ENOTTY;
    }
    return result;
}

static void __exit tamc532_cleanup_module(void)
{
    printk(KERN_NOTICE "TAMC532_CLEANUP_MODULE: PCI DRIVER UNREGISTERED\n");
    pci_unregister_driver(&pci_tamc532_driver);
    printk(KERN_NOTICE "TAMC532_CLEANUP_MODULE CALLED\n");
    upciedev_cleanup_module_exp(&tamc532_cdev_m);
    
}

static int __init tamc532_init_module(void)
{
    int   result = 0;
    
    printk(KERN_WARNING "TAMC532_INIT_MODULE CALLED\n");
    result = upciedev_init_module_exp(&tamc532_cdev_m, &tamc532_fops, TAMC532DEVNAME);
    result = pci_register_driver(&pci_tamc532_driver);
    printk(KERN_ALERT "TAMC532_INIT_MODULE:REGISTERING PCI DRIVER RESUALT %d\n", result);
    return 0; /* succeed */
}

module_init(tamc532_init_module);
module_exit(tamc532_cleanup_module);


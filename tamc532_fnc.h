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


#ifndef _TAMC532_FNC_H_
#define _TAMC532_FNC_H_

#include <linux/ioctl.h> /* needed for the _IOW etc stuff used later */
#include "pciedev_io.h"
#include "pciedev_ufn.h"
#include "tamc532_io.h"

#define	TAMC532_SWAPL(x) ((((x) >> 24) & 0x000000FF) | (((x) >> 8) & 0x0000FF00) | (((x) << 8) & 0x00FF0000) | (((x) << 24) & 0xFF000000))
#define	TAMC532_SWAPS(x) ((((x) >> 8) & 0x00FF) | (((x) << 8) & 0xFF00))

#ifndef TAMC532_NR_DEVS
#define TAMC532_NR_DEVS 15 
#endif

#define TAMC532DEVNAME     "tamc532"	                    /* name of device */
#define TAMC532_VENDOR_ID 0x1498	                    /* TEWS vendor ID */
#define TAMC532_DEVICE_ID   0x8214	                    /* TAMC532 dev board device ID */

#define TAMC532_KERNEL_DMA_BLOCK_SIZE 131072    // 128kByte
#define TAMC532_MEM_MAX_SIZE                536870912 // 512MByte
#define TAMC532_DMA_MAX_SYZE               32768
#define TAMC532_DMA_MIN_SYZE                128
#define TAMC532_DMA_SYZE                        4096
#define TAMC532_DMAC_MAX_NUM             64
//#define TAMC532_DMAC_MAX_SIZE             61440
#define TAMC532_DMAC_MAX_SIZE             32768
#define TAMC532_DESC_SIZE                       16
#define TAMC532_DESC_OFFSET                 1024 //0x400


struct tamc532_dev {
	int                              brd_num;
	spinlock_t                   irq_lock;
	struct timeval             dma_start_time;
	struct timeval             dma_stop_time;
	
	
	u32                             dma_page_num[DMACNUM];
	int                               dmac_enable[DMACNUM];
	int                               dmac_done[DMACNUM];
	int			      dmac_dma_int_enabled[DMACNUM];
	struct work_struct    tamc532_work;
	int                               waitFlag;
	int                               i2cWaitFlag;
	wait_queue_head_t       waitDMA;
	wait_queue_head_t       waitI2C;
	struct pciedev_dev       *parent_dev;
	int                                lendian;
	int                                irqsts;
	int                                irq_num[DMACNUM];
	int                                irq_dmac_rcv[DMACNUM];
	int                                board_irq_num;
	int                                board_irq_rcv;
	u32                             dmac_dmf[DMACNUM];
	int                               dmac_dq[DMACNUM];

	
	void*                pDescBuf[DMACNUM][TAMC532_DMAC_MAX_NUM];
	void*                pDescBufDMAC;
	dma_addr_t      pTmpDescHandle[DMACNUM][TAMC532_DMAC_MAX_NUM];
	//void*                pDescBuf[DMACNUM];
	//dma_addr_t      pTmpDescHandle[DMACNUM];
	
	
	
	u32                   dev_dma_desc_size[DMACNUM][TAMC532_DMAC_MAX_NUM];
	int                    dma_desc_order[DMACNUM][TAMC532_DMAC_MAX_NUM];
	
	void*                pWriteBuf[DMACNUM][TAMC532_DMAC_MAX_NUM] ;
	dma_addr_t      pTmpDmaHandle[DMACNUM][TAMC532_DMAC_MAX_NUM];
	u32                    dev_dma_size[DMACNUM][TAMC532_DMAC_MAX_NUM];
	u32                    dev_dma_last_size;
	u32                    dev_dma_trans_size[DMACNUM][TAMC532_DMAC_MAX_NUM];
	int                      dma_order[DMACNUM][TAMC532_DMAC_MAX_NUM];
	int                       dev_dma_size_change;
	
};
typedef struct tamc532_dev tamc532_dev;

long tamc532_ioctl_dma(struct file *, unsigned int* , unsigned long* );
int tamc532_i2c_read(pciedev_dev *, device_i2c_rw *);
int tamc532_i2c_write(pciedev_dev *, device_i2c_rw *);
long ms_to_ticks( long ms );

#endif /* _TAMC532_FNC_H_ */

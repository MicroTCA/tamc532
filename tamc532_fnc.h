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

#define TAMC532_MEM_MAX_SIZE                536870912 // 512MByte

#define TAMC532_DMAC_MAX_SIZE             32768
//#define TAMC532_DMAC_MAX_SIZE             262140 // Byte 2097120
//#define TAMC532_DMAC_MAX_SIZE             65536      //262140/4

#define TAMC532_DESC_SIZE                       16

#define TAMC532_DMAC_MAX_NUM             64
#define TAMC532_DESC_OFFSET                 1024 //0x400

#define TAMC532_STRM_SAMPLE_NUM                 24
#define TAMC532_STRM_TRQ_NUM                        500

#define TAMC532_MAX_SERVER	 	0x25

struct tamc532_server_signal{
   pid_t         f_ServerPref;
   u32           f_IrqProc[5];
   u_short     f_IrqFlag;
   u_short     f_CurSig ;
   u_short     f_HPSig ;
};
typedef struct tamc532_server_signal t_tamc532_ServerSignal;


struct tamc532_dev {
	int                              brd_num;
	spinlock_t                   irq_lock;
	struct timeval             dma_start_time;
	struct timeval             dma_stop_time;
	
	device_staus_registers dev_sts_regs;
	
	
	u32                             dma_page_num[DMACNUM];
	int                               dmac_enable[DMACNUM];
	int                               dmac_done[DMACNUM];
	int			      dmac_dma_int_enabled[DMACNUM];
	struct work_struct    tamc532_work;
	struct pciedev_dev   *parent_dev;
	int                                lendian;
	int                                irqsts;
	int                                irq_enable_reg;
	int                                irq_num[DMACNUM];
	int                                irq_dmac_rcv[DMACNUM];
	int                                board_irq_num;
	int                                board_irq_rcv;
	u32                             dmac_dmf[DMACNUM];
	int                               dmac_dq[DMACNUM];

	
	void*                  pDescBuf[DMACNUM][TAMC532_DMAC_MAX_NUM];
	void*                  pDescBufDMAC;
	dma_addr_t      pTmpDescHandle[DMACNUM][TAMC532_DMAC_MAX_NUM];
	
	void*                  pDescBuf1[DMACNUM][TAMC532_DMAC_MAX_NUM];
	void*                  pDescBufDMAC1;
	dma_addr_t      pTmpDescHandle1[DMACNUM][TAMC532_DMAC_MAX_NUM];
	
	//void*                pDescBuf[DMACNUM];
	//dma_addr_t      pTmpDescHandle[DMACNUM];
	
	
	
	u32                   dev_dma_desc_size[DMACNUM][TAMC532_DMAC_MAX_NUM];
	int                     dma_desc_order[DMACNUM][TAMC532_DMAC_MAX_NUM];
	
	void*                  pWriteBuf[DMACNUM][TAMC532_DMAC_MAX_NUM] ;
	dma_addr_t      pTmpDmaHandle[DMACNUM][TAMC532_DMAC_MAX_NUM];
	u32                    dev_dma_size[DMACNUM][TAMC532_DMAC_MAX_NUM];
	u32                    dev_dma_last_size;
	u32                    dev_dma_trans_size[DMACNUM][TAMC532_DMAC_MAX_NUM];
	int                      dma_order[DMACNUM][TAMC532_DMAC_MAX_NUM];
	int                      dev_dma_size_change;
	
	t_tamc532_ServerSignal     server_signal_stack [TAMC532_MAX_SERVER];
	int			strm_trg_num[DMACNUM];
	int			dmac_strm_trg_num;
	int			strm_dma_size;
	int			strm_dma_cur_buf;
	int			strm_buf_dma_size;
	int			is_strm_dma;
	int			is_strm_dma_run;
	int			strm_dmac_done[DMACNUM];
	int			strm_dmac_enbl[DMACNUM];
	int			strm_dmac_num;
	int                        strm_dma_page_num;
	int	 	         dev_stream_dma_size[DMACNUM][TAMC532_DMAC_MAX_NUM];
	int                        dev_stream_dma_trans_size[DMACNUM][TAMC532_DMAC_MAX_NUM];
	int                        stream_dma_order[DMACNUM][TAMC532_DMAC_MAX_NUM];
	void*                    pStrmWriteBuf[DMACNUM][TAMC532_DMAC_MAX_NUM] ;
	dma_addr_t        pStrmDmaHandle[DMACNUM][TAMC532_DMAC_MAX_NUM];
	void*                    pStrmWriteBuf1[DMACNUM][TAMC532_DMAC_MAX_NUM] ;
	dma_addr_t        pStrmDmaHandle1[DMACNUM][TAMC532_DMAC_MAX_NUM];
	
};
typedef struct tamc532_dev tamc532_dev;

long tamc532_ioctl_dma(struct file *, unsigned int* , unsigned long* );
int tamc532_i2c_read(pciedev_dev *, device_i2c_rw *);
int tamc532_i2c_write(pciedev_dev *, device_i2c_rw *);
long ms_to_ticks( long ms );

#endif /* _TAMC532_FNC_H_ */

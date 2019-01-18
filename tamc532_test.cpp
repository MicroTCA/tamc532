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


#include <stdlib.h>
#include <strings.h>
#include <string.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <ctype.h>
#include <signal.h>
#include <setjmp.h>
#include <fcntl.h>
#include <unistd.h>

#include <iostream>
#include <fstream>

#include "pciedev_io.h"
#include "tamc532_io.h"

/* useconds from struct timeval */
#define	MIKRS(tv) (((double)(tv).tv_usec ) + ((double)(tv).tv_sec * 1000000.0)) 
#define	MILLS(tv) (((double)(tv).tv_usec/1000 )  + ((double)(tv).tv_sec * 1000.0)) 


int	         fd;
struct timeval   start_time;
struct timeval   end_time;

int main(int argc, char* argv[])
{
    int	 ch_in        = 0;
    char nod_name[24] = "";
    device_rw	           l_RW;
    device_ioctrl_data	  l_Read;
    device_i2c_rw            i2c_Reg;
    device_ioctrl_dma     DMA_RW;
    device_ioctrl_time    DMA_TIME;
    u_int	          tmp_offset;
    int                    tmp_mode;
    int      	          tmp_barx;
    u_int	          tmp_size;
    u_int	          tmp_sample;
    u_int	          tmp_pattern;
    int      	          tmp_data;
    int      	          tmp_print = 0;
    int      	          tmp_print_start = 0;
    int      	          tmp_print_stop  = 0;
    int                   len = 0;
    int                   code = 0;
    int*                  tmp_dma_buf;
    int*                  tmp_write_buf;
    int                   k = 0;
    double                time_tmp = 0;
    double                time_tmp_loop               = 0;
    double                time_tmp_loop_dlt        = 0;
    double                time_tmp_loop_drv      = 0;
    double                time_tmp_loop_dlt_drv = 0;
    double                time_dlt;
    float                 tmp_fdata;
    int                   itemsize = 0;
    int                   tmp_loop = 0;
    device_i2c_rw     i2c_rw_buf;

    itemsize = sizeof(device_rw);

    if(argc ==1){
        printf("Input \"prog /dev/tamc532s3\" \n");
        return 0;
    }
    strncpy(nod_name,argv[1],sizeof(nod_name));
    fd = open (nod_name, O_RDWR);
    if (fd < 0) {
        printf ("#CAN'T OPEN FILE %s \n", nod_name);
        exit (1);
    }
    while (ch_in != 11){
        printf("\n READ (1) or WRITE (0) ?-");
        printf("\n GET DRIVER VERSION (2) or GET FIRMWARE VERSION (3) or GET SLOT NUM (4) ?-");
        printf("\n GET_ARM_CSPT (5) or SET_ARM_CSPT (6)  ?-");
        printf("\n GET_ENDIAN (7) or  SET_ENDIAN (8) ?-");
        printf("\n GET_TRG_POL (9) or  SET_TRG_POL (10) ?-");
        printf("\n GET_TRG_SOURCE (12) or  SET_TRG_SOURCE (13) ?-");
        printf("\n I2C_READ (14) or  I2C_WRITE(15) ?-");
        printf("\n CTRL_SET_DMA (28) CTRL_REM_DMA (29) ?-");
        printf("\n CTRL_DMA READ (30) END (11) ?-");
        scanf("%d",&ch_in);
        fflush(stdin);
switch (ch_in){
            case 0 :
                printf ("\n INPUT  BARx (0,1,2,3,4,5)  -");
                scanf ("%x",&tmp_barx);
                fflush(stdin);

                printf ("\n INPUT  MODE  (0-D8,1-D16,2-D32)  -");
                scanf ("%x",&tmp_mode);
                fflush(stdin);

                printf ("\n INPUT  ADDRESS (IN HEX)  -");
                scanf ("%x",&tmp_offset);
                fflush(stdin);

                printf ("\n INPUT DATA (IN HEX)  -");
                scanf ("%x",&tmp_data);
                fflush(stdin);

                l_RW.data_rw   = tmp_data;
                l_RW.offset_rw = tmp_offset;
                l_RW.mode_rw   = tmp_mode;
                l_RW.barx_rw   = tmp_barx;
                l_RW.size_rw   = 0;

                printf ("MODE - %X , OFFSET - %X, DATA - %X\n",
                     l_RW.mode_rw, l_RW.offset_rw, l_RW.data_rw);

                len = write (fd, &l_RW, sizeof(device_rw));
                if (len != itemsize ){
                        printf ("#CAN'T READ FILE \n");
                }
                break;
	    case 1 :
                printf ("\n INPUT  BARx (0,1,2,3,4,5)  -");
                scanf ("%x",&tmp_barx);
                fflush(stdin);
                printf ("\n INPUT  MODE  (0-D8,1-D16,2-D32)  -");
                scanf ("%x",&tmp_mode);
                fflush(stdin);
                printf ("\n INPUT OFFSET (IN HEX)  -");
                scanf ("%x",&tmp_offset);
                fflush(stdin);
                printf ("\n INPUT SAMPLE NUM (DEC)  -");
                scanf ("%i",&tmp_sample);
                fflush(stdin);
                                
                l_RW.data_rw    = 0;
                l_RW.offset_rw  = tmp_offset;
                l_RW.mode_rw  = tmp_mode;
                l_RW.barx_rw    = tmp_barx;
                l_RW.size_rw     = tmp_sample;
                switch(tmp_mode){
                    case 0:
                         tmp_size = sizeof(u_char)*tmp_sample;
                        break;
                   case 1:
                         tmp_size = sizeof(u_short)*tmp_sample;
                        break;
                   case 2:
                         tmp_size = sizeof(u_int)*tmp_sample;
                        break;
                  default:
                         tmp_size = sizeof(u_int)*tmp_sample;
                        break;
                }
                 printf ("MODE - %X , OFFSET - %X, SAMPLE %i DATA - %X\n", l_RW.mode_rw, l_RW.offset_rw, l_RW.size_rw , l_RW.data_rw);
                if(tmp_sample < 2){
                        len = read (fd, &l_RW, sizeof(device_rw));
                        if (len != itemsize ){
                           printf ("#CAN'T READ FILE ERROR %i \n", len);
                        }
                        printf ("READED : MODE - %X , OFFSET - %X, DATA - %X\n",  l_RW.mode_rw, l_RW.offset_rw, l_RW.data_rw);
                }else{
                       tmp_dma_buf     = new int[tmp_size + itemsize];
                       memcpy(tmp_dma_buf, &l_RW, itemsize);
                       gettimeofday(&start_time, 0);
                       len = read (fd, tmp_dma_buf, sizeof(device_rw));
                       gettimeofday(&end_time, 0);
                        if (len != itemsize ){
                           printf ("#CAN'T READ FILE ERROR %i \n", len);
                        }
                       time_tmp    =  MIKRS(end_time) - MIKRS(start_time);
                       time_dlt       =  MILLS(end_time) - MILLS(start_time);
                       printf("STOP READING TIME %fms : %fmks  SIZE %lu\n", time_dlt, time_tmp,tmp_size);
                       printf("STOP READING KBytes/Sec %f\n",(tmp_size*1000)/time_tmp);
                       
                        printf ("PRINT (0 NO, 1 YES)  -\n");
                        scanf ("%d",&tmp_print);
                        fflush(stdin);
                        while (tmp_print){
                            printf ("START POS  -\n");
                            scanf ("%d",&tmp_print_start);
                            fflush(stdin);
                            printf ("STOP POS  -\n");
                            scanf ("%d",&tmp_print_stop);
                            fflush(stdin);
                            k = tmp_print_start*4;
                            for(int i = tmp_print_start; i < tmp_print_stop; i++){
                                    printf("NUM %i OFFSET %X : DATA %X\n", i,k, (u_int)(tmp_dma_buf[i] & 0xFFFFFFFF));
                                    k += 4;
                            }
                            printf ("PRINT (0 NO, 1 YES)  -\n");
                            scanf ("%d",&tmp_print);
                            fflush(stdin);
                    }
                     if(tmp_dma_buf) delete tmp_dma_buf;
                }
	       break;
            case 2 :
                ioctl(fd, PCIEDEV_DRIVER_VERSION, &l_Read);
                tmp_fdata = (float)((float)l_Read.offset/10.0);
                tmp_fdata += (float)l_Read.data;
                printf ("DRIVER VERSION IS %f\n", tmp_fdata);
                break;
	    case 3 :
                ioctl(fd, PCIEDEV_FIRMWARE_VERSION, &l_Read);
                printf ("FIRMWARE VERSION IS - %X\n", l_Read.data);
		break;
            case 4 :
                ioctl(fd, PCIEDEV_PHYSICAL_SLOT, &l_Read);
                printf ("SLOT NUM IS - %X\n", l_Read.data);
                break;
            case 5 :
                printf ("\n INPUT CSPT_NUM (1,2,4,8,F)  -");
                scanf ("%x",&tmp_offset);
                fflush(stdin);
                l_Read.offset = tmp_offset;
                l_Read.cmd = 1;        
                ioctl(fd, TAMC532_ARM_CSPT, &l_Read);
                printf ("CSPT ENABLE - %X\n", l_Read.data);
                break;
	    case 6 :
                printf ("\n INPUT CSPT_NUM (1,2,4,8,F)  -");
                scanf ("%x",&tmp_offset);
                fflush(stdin);
                printf ("\n INPUT DATA (0/1)  -");
                scanf ("%x",&tmp_data);
                fflush(stdin);
                l_Read.offset = tmp_offset;
                l_Read.data   = tmp_data;
                l_Read.cmd = 0;   		
                ioctl(fd, TAMC532_ARM_CSPT, &l_Read);
	       break;
           case 7 :
                printf ("\n INPUT CSPT_NUM (1,2,4,8,F)  -");
                scanf ("%x",&tmp_offset);
                fflush(stdin);
                l_Read.offset = tmp_offset;
                l_Read.cmd = 1;        
                ioctl(fd, TAMC532_ENDIAN, &l_Read);
                printf ("ENDIAN ENABLE - %X\n", l_Read.data);
                break;
	    case 8 :
                printf ("\n INPUT CSPT_NUM (1,2,4,8,F)  -");
                scanf ("%x",&tmp_offset);
                fflush(stdin);
                printf ("\n INPUT DATA (0/1)  -");
                scanf ("%x",&tmp_data);
                fflush(stdin);
                l_Read.offset = tmp_offset;
                l_Read.data   = tmp_data;
                l_Read.cmd = 0;   		
                ioctl(fd, TAMC532_ENDIAN, &l_Read);
	       break;
           case 9 :
                printf ("\n INPUT CSPT_NUM (1,2,4,8,F)  -");
                scanf ("%x",&tmp_offset);
                fflush(stdin);
                l_Read.offset = tmp_offset;
                l_Read.cmd = 1;        
                ioctl(fd, TAMC532_TRG_POLARITY, &l_Read);
                printf ("TRG POLARITY - %X\n", l_Read.data);
                break;
	    case 10 :
                printf ("\n INPUT CSPT_NUM (1,2,4,8,F)  -");
                scanf ("%x",&tmp_offset);
                fflush(stdin);
                printf ("\n INPUT DATA (0/1)  -");
                scanf ("%x",&tmp_data);
                fflush(stdin);
                l_Read.offset = tmp_offset;
                l_Read.data   = tmp_data;
                l_Read.cmd = 0;   		
                ioctl(fd, TAMC532_TRG_POLARITY, &l_Read);
	       break;
           case 12 :
                printf ("\n INPUT CSPT_NUM (1,2,4,8,F)  -");
                scanf ("%x",&tmp_offset);
                fflush(stdin);
                l_Read.offset = tmp_offset;
                l_Read.cmd = 1;        
                ioctl(fd, TAMC532_TRG_SOURCE, &l_Read);
                printf ("TRG SOURCE - %X\n", l_Read.data);
                break;
	    case 13 :
                printf ("\n INPUT CSPT_NUM (1,2,4,8,F)  -");
                scanf ("%x",&tmp_offset);
                fflush(stdin);
                printf ("\n INPUT DATA (0/1)  -");
                scanf ("%x",&tmp_data);
                fflush(stdin);
                l_Read.offset = tmp_offset;
                l_Read.data   = tmp_data;
                l_Read.cmd = 0;   		
                ioctl(fd, TAMC532_TRG_SOURCE, &l_Read);
	       break;
	       
	       case 14 :
                printf ("\n INPUT BUS NUM (HEX)");
                scanf ("%x",&tmp_offset);
                fflush(stdin);
                i2c_rw_buf.busNum = tmp_offset;
                printf ("\n INPUT  DEV ADDRESS (HEX) -");
                scanf ("%x",&tmp_size);
                fflush(stdin);
                i2c_rw_buf.devAddr = tmp_size;
                printf ("\n INPUT REG_ADDRESS (HEX)");
                scanf ("%x",&tmp_data);
                fflush(stdin);
				i2c_rw_buf.regAddr = tmp_data;
				i2c_rw_buf.size        = 1;
				ioctl(fd, PCIEDEV_I2C_READ, &i2c_rw_buf);
				tmp_data = i2c_rw_buf.data[0] ;
				printf("DATA %X\n", tmp_data);
                break;
	    case 15 :
                printf ("\n INPUT BUS NUM (HEX)");
                scanf ("%x",&tmp_offset);
                fflush(stdin);
                i2c_rw_buf.busNum = tmp_offset;
                printf ("\n INPUT  DEV ADDRESS (HEX) -");
                scanf ("%x",&tmp_size);
                fflush(stdin);
                i2c_rw_buf.devAddr = tmp_size;
                printf ("\n INPUT REG_ADDRESS (HEX)");
                scanf ("%x",&tmp_data);
                fflush(stdin);
				i2c_rw_buf.regAddr = tmp_data;
				i2c_rw_buf.size        = 1;
				printf ("\n INPUT DATA (HEX)");
                scanf ("%x",&tmp_data);
                fflush(stdin);
				i2c_rw_buf.data[0]   = tmp_data & 0xFF;
	            ioctl(fd, PCIEDEV_I2C_WRITE, &i2c_rw_buf);    
	            break;
           
           case 30 :
                DMA_RW.dma_offset  = 0;
                DMA_RW.dma_size    = 0;
                DMA_RW.dma_cmd     = 0;
                DMA_RW.dma_pattern = 0; 
                printf ("\n INPUT  DMA_SIZE (num of sumples (int))  -");
                scanf ("%d",&tmp_size);
                fflush(stdin);
                DMA_RW.dma_size    = sizeof(int)*tmp_size;
                printf ("\n INPUT OFFSET (int)  -");
                scanf ("%d",&tmp_offset);
                fflush(stdin);
                DMA_RW.dma_offset = tmp_offset;
                
                printf ("DMA_OFFSET - %X, DMA_SIZE - %X\n", DMA_RW.dma_offset, DMA_RW.dma_size);
                printf ("MAX_MEM- %X, DMA_MEM - %X:%X\n", 536870912,  (DMA_RW.dma_offset + DMA_RW.dma_size),
                                                                                              (DMA_RW.dma_offset + DMA_RW.dma_size*4));
                
                tmp_dma_buf     = new int[tmp_size + DMA_DATA_OFFSET];
                memcpy(tmp_dma_buf, &DMA_RW, sizeof (device_ioctrl_dma));
                
                gettimeofday(&start_time, 0);
                code = ioctl (fd, PCIEDEV_READ_DMA, tmp_dma_buf);
                gettimeofday(&end_time, 0);
                printf ("===========READED  CODE %i\n", code);
                time_tmp    =  MIKRS(end_time) - MIKRS(start_time);
                time_dlt       =  MILLS(end_time) - MILLS(start_time);
                printf("STOP READING TIME %fms : %fmks  SIZE %lu\n", time_dlt, time_tmp,(sizeof(int)*tmp_size));
                printf("STOP READING KBytes/Sec %f\n",((sizeof(int)*tmp_size*1000)/time_tmp));
                code = ioctl (fd, PCIEDEV_GET_DMA_TIME, &DMA_TIME);
                if (code) {
                    printf ("######ERROR GET TIME %d\n", code);
                }
                printf ("===========DRIVER TIME \n");
                time_tmp = MIKRS(DMA_TIME.stop_time) - MIKRS(DMA_TIME.start_time);
                time_dlt    = MILLS(DMA_TIME.stop_time) - MILLS(DMA_TIME.start_time);
                printf("STOP DRIVER TIME START %li:%li STOP %li:%li\n",
                                                            DMA_TIME.start_time.tv_sec, DMA_TIME.start_time.tv_usec, 
                                                            DMA_TIME.stop_time.tv_sec, DMA_TIME.stop_time.tv_usec);
                printf("STOP DRIVER READING TIME %fms : %fmks  SIZE %lu\n", time_dlt, time_tmp,(sizeof(int)*tmp_size));
                printf("STOP DRIVER READING KBytes/Sec %f\n",((sizeof(int)*tmp_size*1000)/time_tmp));
                printf ("PRINT (0 NO, 1 YES)  -\n");
                scanf ("%d",&tmp_print);
                fflush(stdin);
                while (tmp_print){
                    printf ("START POS  -\n");
                    scanf ("%d",&tmp_print_start);
                    fflush(stdin);
                    printf ("STOP POS  -\n");
                    scanf ("%d",&tmp_print_stop);
                    fflush(stdin);
                    k = tmp_print_start*4;
                    for(int i = tmp_print_start; i < tmp_print_stop; i++){
                            printf("NUM %i OFFSET %X : DATA %X\n", i,k, (u_int)(tmp_dma_buf[i] & 0xFFFFFFFF));
                            k += 4;
                    }
                    printf ("PRINT (0 NO, 1 YES)  -\n");
		    scanf ("%d",&tmp_print);
		    fflush(stdin);
                }
                if(tmp_dma_buf) delete tmp_dma_buf;
                break;
        
                
         case 28 :
                DMA_RW.dma_offset  = 0;
                DMA_RW.dma_size    = 0;
                DMA_RW.dma_cmd     = 0;
                DMA_RW.dma_pattern = 0; 
                printf ("\n INPUT  DMA_SIZE (num of sumples (int))  -");
                scanf ("%d",&tmp_size);
                fflush(stdin);
                DMA_RW.dma_size    = sizeof(int)*tmp_size;
                printf ("\n INPUT OFFSET (int)  -");
                scanf ("%d",&tmp_offset);
                fflush(stdin);
                DMA_RW.dma_offset = tmp_offset;
                
                printf ("DMA_OFFSET - %X, DMA_SIZE - %X\n", DMA_RW.dma_offset, DMA_RW.dma_size);
                printf ("MAX_MEM- %X, DMA_MEM - %X:%X\n", 536870912,  (DMA_RW.dma_offset + DMA_RW.dma_size),
                                                                                              (DMA_RW.dma_offset + DMA_RW.dma_size*4));
                
                tmp_dma_buf     = new int[tmp_size + DMA_DATA_OFFSET];
                memcpy(tmp_dma_buf, &DMA_RW, sizeof (device_ioctrl_dma));
                code = ioctl (fd, TAMC532_SET_DMA, tmp_dma_buf);
                if(tmp_dma_buf) delete tmp_dma_buf;
                break;       
         case 29 :
                DMA_RW.dma_offset  = 0;
                DMA_RW.dma_size    = 0;
                DMA_RW.dma_cmd     = 0;
                DMA_RW.dma_pattern = 0; 
                printf ("\n INPUT OFFSET (int)  -");
                scanf ("%d",&tmp_offset);
                fflush(stdin);
                DMA_RW.dma_offset = tmp_offset;
                code = ioctl (fd, TAMC532_REM_DMA, DMA_RW, &DMA_RW);
                break;            
                
        default:
              break;
	}
    }
    close(fd);
    return 0;
}


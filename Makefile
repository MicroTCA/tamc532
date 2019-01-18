tamc532-objs := tamc532_drv.o tamc532_ioctl_dma.o tamc532_i2c.o
obj-m := tamc532.o

KVERSION = $(shell uname -r)
all:
	make -C /lib/modules/$(KVERSION)/build V=1 M=$(PWD) modules
clean:
	test ! -d /lib/modules/$(KVERSION) || make -C /lib/modules/$(KVERSION)/build V=1 M=$(PWD) clean

EXTRA_CFLAGS += -I/usr/include
#KBUILD_EXTRA_SYMBOLS += /home/petros/doocs/source/unixdriver/utca/linux/upciedev/Upciedev.symvers
KBUILD_EXTRA_SYMBOLS = /lib/modules/$(KVERSION)/upciedev/Upciedev.symvers

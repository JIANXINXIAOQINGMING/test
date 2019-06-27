
KERNEL_SRC_DIR  ?= /home/lxl/vkbs/vkbs_source/GPL/linux/


ARCH?=arm
CROSS_COMPILE  ?= arm-linux-gnueabihf-


obj-m := globalmem.o

all:
	make -C $(KERNEL_SRC_DIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) M=$(PWD) modules

clean:
	make -C $(KERNEL_SRC_DIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) M=$(PWD) clean
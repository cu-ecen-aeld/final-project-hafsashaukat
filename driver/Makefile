# BME280 Linux I2C Driver Makefile
# Raspberry Pi 4 - Linux 6.1.61

# build bme280_driver.c as a loadable kernel module
obj-m += bme280_driver.o

# buildroot output directory
BUILDROOT_OUTPUT := $(HOME)/Desktop/final-project-buildroot-output

# linux kernel build directory
KERNEL_DIR := $(BUILDROOT_OUTPUT)/build/linux-custom

# cross-compiler provided by Buildroot
CROSS_COMPILE := $(BUILDROOT_OUTPUT)/host/bin/aarch64-buildroot-linux-gnu-

# compile the module for ARM64
all:
	$(MAKE) -C $(KERNEL_DIR) \
		ARCH=arm64 \
		CROSS_COMPILE=$(CROSS_COMPILE) \
		M=$(CURDIR) modules

# remove generated compilation files
clean:
	$(MAKE) -C $(KERNEL_DIR) \
		ARCH=arm64 \
		CROSS_COMPILE=$(CROSS_COMPILE) \
		M=$(CURDIR) clean


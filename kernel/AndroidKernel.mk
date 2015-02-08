#Android makefile to build kernel as a part of Android Build

TARGET_OUT_INTERMEDIATES := ../../out
KERNEL_OUT := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ
KERNEL_CONFIG := $(KERNEL_OUT)/.config
#TARGET_PREBUILT_INT_KERNEL := $(KERNEL_OUT)/arch/arm/boot/zImage
TARGET_PREBUILT_INT_KERNEL := $(TARGET_OUT_INTERMEDIATES)/kernel
ZIMAGE_OUTPUT := $(KERNEL_OUT)/arch/arm//boot/zImage
KERNEL_HEADERS_INSTALL := $(KERNEL_OUT)/usr
KERNEL_MODULES_OUT := $(TARGET_OUT)/lib/modules
KERNEL_DEFCONFIG := msm7627a_sku1_es03-perf_defconfig
CROSS_COMPILE := /usr/arm-eabi-4.4.3/bin/arm-eabi-
PERMISSION_D := sudo



define mv-modules
mdpath=`find $(KERNEL_MODULES_OUT) -type f -name modules.dep`;\
if [ "$$mdpath" != "" ];then\
mpath=`dirname $$mdpath`;\
ko=`find $$mpath/kernel -type f -name *.ko`;\
for i in $$ko; do mv $$i $(KERNEL_MODULES_OUT)/; done;\
fi
endef

define clean-module-folder
mdpath=`find $(KERNEL_MODULES_OUT) -type f -name modules.dep`;\
if [ "$$mdpath" != "" ];then\
mpath=`dirname $$mdpath`; rm -rf $$mpath;\
fi
endef

.PHONY :all
all: $(TARGET_PREBUILT_INT_KERNEL)

$(KERNEL_OUT):
	mkdir -p $(KERNEL_OUT)

$(KERNEL_CONFIG): $(KERNEL_OUT)
	$(MAKE)   O=$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) $(KERNEL_DEFCONFIG)


$(TARGET_PREBUILT_INT_KERNEL): $(KERNEL_OUT) $(KERNEL_CONFIG) $(KERNEL_HEADERS_INSTALL)
	$(MAKE)   O=$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE)
	$(MAKE)   O=$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) modules
	$(PERMISSION_D) $(MAKE)   O=$(KERNEL_OUT) INSTALL_MOD_PATH=../../$(KERNEL_MODULES_INSTALL) ARCH=arm CROSS_COMPILE=arm-eabi- modules_install

$(KERNEL_HEADERS_INSTALL): $(KERNEL_OUT) $(KERNEL_CONFIG)
	$(MAKE)   O=$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) headers_install

kerneltags: $(KERNEL_OUT) $(KERNEL_CONFIG)
	$(MAKE)  O=$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) tags

kernelconfig: $(KERNEL_OUT) $(KERNEL_CONFIG)
	env KCONFIG_NOTIMESTAMP=true \
	     $(MAKE) -C kernel O=$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) menuconfig
	env KCONFIG_NOTIMESTAMP=true \
	     $(MAKE) -C kernel O=$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) savedefconfig
	cp $(KERNEL_OUT)/defconfig kernel/arch/arm/configs/$(KERNEL_DEFCONFIG)


.PHONY : FORCE

FORCE: ;

##########################################
########## Amlogic Drivers ###############
##########################################

obj-$(CONFIG_PWM_MESON)			+= pwm/
obj-$(CONFIG_AMLOGIC_WIFI)		+= wifi/
obj-$(CONFIG_AMLOGIC_BT_DEVICE)		+= bluetooth/


KBUILD_CFLAGS_MODULE += $(GKI_EXT_MODULE_PREDEFINE)

all:
	@$(MAKE) -C $(KERNEL_SRC) M=$(M)  modules $(GKI_EXT_MODULE_CONFIG)

modules_install:
	@echo "$(MAKE) INSTALL_MOD_STRIP=1 M=$(M) -C $(KERNEL_SRC) modules_install"
#	@$(MAKE) INSTALL_MOD_STRIP=1 M=$(M) -C $(KERNEL_SRC) modules_install
#	mkdir -p ${OUT_DIR}/../vendor_lib/modules
#	cd ${OUT_DIR}/$(M)/; find -name "*.ko" -exec cp {} ${OUT_DIR}/../vendor_lib/modules/ \;
	mkdir -p ${OUT_DIR}/../vendor_lib/modules
	(cd ${OUT_DIR}/$(M)/; find -name "dvb_demux.ko" -exec cp {} ${OUT_DIR}/../vendor_lib/modules/ \;)
	(cd ${OUT_DIR}/$(M)/; find -name "aml_aucpu.ko" -exec cp {} ${OUT_DIR}/../vendor_lib/modules/ \;)

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) clean


TARGET=vlsicomm.ko

#PROJECT=PROJECT_W1
#export PROJECT
export PROJ_NAME

all: firmware driver

firmware:
	echo "make w1ss1 firmware ..."
	make -C firmware

driver:
	echo "make w1ss1 driver ..."
	make -C vmac
	cp vmac/$(TARGET) ./
   

clean :
	make clean -C firmware
	make clean -C vmac
	rm -rf $(TARGET)

.PHONY:firmware driver

CC:=g++
CFLAGS:= -Wall -g


lidar_driver: main.cpp
	$(CC) -o $@ $(CFLAGS) $< -lrplidar_sdk -lpthread


ARCH:=$(shell uname -s)
SDK_ROOT:=./rplidar_sdk/sdk
SDK_INSTALL_PATH:=/usr/local

.PHONY: sdk install_sdk uninstall_sdk clean

sdk:
	git submodule update --init rplidar_sdk
	$(MAKE) -C rplidar_sdk/sdk/

install_sdk:
	cp $(SDK_ROOT)/output/$(ARCH)/Release/librplidar_sdk.a $(SDK_INSTALL_PATH)/lib/
	mkdir -p $(SDK_INSTALL_PATH)/include/rplidar
	cp $(SDK_ROOT)/sdk/include/* $(SDK_INSTALL_PATH)/include/rplidar/
	cp -r $(SDK_ROOT)/sdk/src/hal $(SDK_INSTALL_PATH)/include/rplidar/

uninstall_sdk:
	rm $(SDK_INSTALL_PATH)/lib/librplidar_sdk.a
	rm -rf $(SDK_INSTALL_PATH)/include/rplidar/

clean:
	rm lidar_driver

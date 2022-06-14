#!/bin/sh
#
# This script is stored within /usr/arduino/extra .
#
# The purpose of this script is to load the kernel
# modules BEFORE flashing/resetting STM32H7. The purpose
# of following modules is to provide basic spi access to STM
# and read firmware version from it.

insmod /usr/arduino/extra/x8h7_drv.ko
insmod /usr/arduino/extra/x8h7_h7.ko
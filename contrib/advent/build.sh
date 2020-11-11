#!/bin/sh

set -e

# requires the gcc-arm-embedded toolchain
PATH=/usr/local/gcc-arm-embedded/bin:"$PATH"

ctangle advent
arm-none-eabi-gcc -Wall -g -Os -fno-builtin -mcpu=cortex-m0 \
	-Wl,-T cortex-m0.ld -o advent.elf -I. -nostartfiles -nostdlib \
	libc.c advent.c -lgcc
arm-none-eabi-objcopy -O binary advent.elf advent.img

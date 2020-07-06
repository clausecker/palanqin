AS=arm-none-eabi-gcc -mcpu=cortex-m0
LD=arm-none-eabi-ld -T cortex-m0.ld

.SUFFIXES: .img

.s.img:
	$(AS) -c -o $*.o $<
	$(LD) -o $*.elf $*.o
	objcopy -O binary $*.elf $@
	rm -f $*.o $*.elf

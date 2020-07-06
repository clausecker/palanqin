AS=arm-none-eabi-gcc -mcpu=cortex-m0
LD=arm-none-eabi-ld -T cortex-m0.ld

.SUFFIXES: .img .elf

.s.img:
	$(AS) -c -o $*.o $<
	$(LD) -o $*.elf $*.o
	objcopy -O binary $*.elf $@
	rm -f $*.o $*.elf

.s.elf:
	$(AS) -c -o $*.o $<
	$(LD) -o $*.elf $*.o
	rm -f $*.o

AS=arm-none-eabi-gcc -mcpu=cortex-m0
LD=arm-none-eabi-ld -T cortex-m0.ld

.SUFFIXES: .elf .img

.elf.img:
	objcopy -O binary $< $@

.s.elf:
	$(AS) -c -o $*.o $<
	$(LD) -o $*.elf $*.o
	rm -f $*.o

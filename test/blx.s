	.include "macros.s"

	.globl _start
	.thumb_func
_start:	debug
	ldr r0, =bxtest+1
	bx r0
	bye

bxtest:	debug
	ldr r0, =blxtest+1
	blx r0
	bye

blxtest:
	debug
	bye

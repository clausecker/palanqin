	.include "macros.s"

	.globl _start
	.thumb_func
_start:	debug
	movs	r0, #1
	debug
	movs	r1, #2
	debug
	adds	r2, r1, r0
	debug
	key
	movs	r0, #42
	debug
	emit
	bye

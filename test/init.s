	.include "macros.s"

	.globl	_start
	.thumb_func
_start:	debug		@ dumb register
	bye		@ terminate emulation

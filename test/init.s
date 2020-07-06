	.thumb

	.globl	_start
	.thumb_func
_start:	.hword	0xb701		@ dump registers
	.hword	0xb700		@ terminate

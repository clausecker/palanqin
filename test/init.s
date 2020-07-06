	.thumb

	@ interrupt vector table
	.word	0x1000		@ initial SP
	.word	_start		@ entry point

	.globl	_start
	.thumb_func
_start:	.hword	0xb701		@ dump registers
	.hword	0xb700		@ terminate

	.syntax unified
	.thumb

	.macro	bye
	.inst.n	0xb700
	.endm

	.macro	debug
	.inst.n	0xb701
	.endm

	.macro	emit
	.inst.n	0xb702
	.endm

	.macro	key
	.inst.n	0xb703
	.endm

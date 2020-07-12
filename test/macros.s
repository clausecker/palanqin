	.syntax unified
	.thumb

	.macro	bye
	.hword	0xb700
	.endm

	.macro	debug
	.hword	0xb701
	.endm

	.macro	emit
	.hword	0xb702
	.endm

	.macro	key
	.hword	0xb703
	.endm

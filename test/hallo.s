	.include "macros.s"

	.globl _start
	.thumb_func
_start:	ldr	r0, =0x6c6c6548	@ 'hell'
	emit
	lsrs	r0, r0, #8
	emit
	asrs	r0, r0, #8
	emit
	lsrs	r0, r0, #8
	emit

	ldr	r0, =0x6f57206f	@ 'o Wo'
	emit
	rev16	r0, r0
	emit
	rev	r0, r0
	emit
	revsh	r0, r0
	emit

	ldr	r0, =0x21646c72	@ 'rld!'
	emit
	str	r0, [sp, #0]
	mov	r1, sp
	ldrb	r0, [r1, #1]
	emit
	ldrh	r0, [r1, #2]
	emit
	rev16	r0, r0
	emit

	bye

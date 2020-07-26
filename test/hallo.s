	.include "macros.s"

	.globl _start
	.thumb_func

	@ print "Hello World!" using shifts and loads
_start:	bl	hell
	bl	o_wo
	bl	rld
	bl	rest
	bye

hell:	ldr	r0, =0x6c6c6548	@ 'Hell'
	emit
	lsrs	r0, r0, #8
	emit
	asrs	r0, r0, #8
	emit
	lsrs	r0, r0, #8
	emit
	bx	lr

o_wo:	ldr	r0, =0x6f57206f	@ 'o Wo'
	emit
	rev16	r0, r0
	emit
	rev	r0, r0
	emit
	revsh	r0, r0
	emit
	bx	lr

rld:	ldr	r0, =0x21646c72	@ 'rld!'
	str	r0, [sp, #0]
	mov	r1, sp
	movs	r0, '?'
	ldr	r0, [sp, #0]
	emit
	ldrb	r0, [r1, #1]
	emit
	ldrh	r0, [r1, #2]
	emit
	rev16	r0, r0
	emit
	bx	lr

	@ print a string using a loop
rest:	adr	r1, string
0:	ldrb	r0, [r1]
	adds	r1, r1, #1
	cmp	r0, #0
	beq	1f
	emit
	b	0b
1:	bx	lr

	.balign	4
string:	.string	"\r\nJedem Anfang wohnt ein Zauber inne."

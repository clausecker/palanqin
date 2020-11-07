	.include "macros.s"

	.globl _start
	.thumb_func

	@ print "Hello World!" using shifts and loads
_start:	bl	hell
	bl	o_wo
	bl	rld
	adr	r1, string
	bl	puts
	bl	indexd
	b	rest

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
puts:	ldrb	r0, [r1]
	adds	r1, r1, #1
	cmp	r0, #0
	beq	1f
	emit
	b	puts
1:	bx	lr

	@ print a test pattern using a [Rn, Rm] addressing mode
indexd:	movs	r0, #0
	push	{r0, lr}	@ push end of string and LR
	sub	sp, sp, #16	@ make space on the stack
	adr	r4, tapete
	movs	r1, #15
	mov	r2, sp
0:	ldrb	r3, [r4, r1]
	strb	r3, [r2, r0]
	adds	r0, r0, #1
	subs	r1, r1, #1
	bpl	0b		@ iterate until whole string reversed
	mov	r1, sp
	bl	puts
	pop	{r3-r7, pc}	@ pop string off stack and return

	@ remaining code (to exercise B #imm11)
rest:	adr	r1, crlf
	bl	puts
	sub	sp, #20
	mov	r1, sp
	stmia	r1!, {r3-r7}
	subs	r1, #20
	bl	puts
	add	sp, #20
	bye

	.balign	4
string:	.string	"\r\nJedem Anfang wohnt ein Zauber inne.\r\n"
	.balign 4
crlf:	.string "\r\n"
	.balign 4
tapete:	.ascii "0123456789abcdef"

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

	movs	r0, #'*'
	emit

1:	key

	debug
	cmp	r0, #'q'	@ Ende mit "q"
	debug
	beq 	2f


	adds	r0, #1
	emit

	b 1b


3:	movs	r0, '?'
	emit

2:	movs	r0, 'b'
	emit

	movs	r0, 'y'
	emit

	movs	r0, 'e'
	emit

	bye


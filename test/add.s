	.include "macros.s"

	.globl _start
	.thumb_func
_start:	debug
	add	r7, sp, #0xc4
	lsrs	r6, r7, #13
	debug
	subs	r1, r2, #1
	lsls	r2, r1, #31
	debug
	adds	r1, r2, r1
	debug
	bye

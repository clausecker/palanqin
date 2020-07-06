	.syntax unified
	.thumb
	.globl _start
	.thumb_func
_start:	.hword	0xb701
	add	r7, sp, #0xc4
	lsrs	r6, r7, #13
	.hword	0xb701
	subs	r1, r2, #1
	lsls	r2, r1, #31
	.hword	0xb701
	adds	r1, r2, r1
	.hword	0xb701
	.hword	0xb700

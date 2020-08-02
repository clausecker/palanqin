; Palanquin -- Cortex-M0 emulator for 8086
; Copyright (c) 2020 Robert Clausecker <fuz@fuz.su>

	cpu	8086		; restrict nasm to 8086 instructions
	bits	16

	; symbols provided by the linker script
	extern	end, edata, etext, bsswords

	section	.data
ident	db	"Copyright (c) 2020 Robert Clausecker <fuz@fuz.su>"
crlf	db	13, 10, 0

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Parameters                                                                 ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

stack	equ	0x100		; emulator stack size in bytes (multiple of 16)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Macros and Constants                                                       ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

	; low and high registers.
	; no parentheses so we can use segment overrides if desired
%define	rlo(r)	[r*4+bp+regs]
%define	rhi(r)	[r*4+bp+regs+hi]

	; load value into ARM register
%macro	ldrlo	2
	mov	rlo(%1), %2
%endmacro

%macro	ldrhi	2
	mov	rhi(%1), %2
%endmacro

	; store value of ARM register
%macro	strlo	2
	mov	%1, rlo(%2)
%endmacro

%macro	strhi	2
	mov	%1, rhi(%2)
%endmacro

	; functionality not implemented
%macro	todo	0
	int3
	ret
%endmacro

	; 8086 flags (those we find useful)
CF	equ	0x0001
ZF	equ	0x0040
SF	equ	0x0080
OF	equ	0x0800

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Startup and Initialisation                                                 ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

	section	.text
	global	start

	; relocate the stack
start:	mov	sp, end+stack	; beginning of stack

	; set up bp with emulator state
	mov	bp, state

	; print copyright notice
	mov	si, ident
	call	puts

	; initialise .bss
	xor	ax, ax
	mov	di, edata
	mov	cx, bsswords	; .bss section length in words
	rep	stosw		; clear .bss

	; configure emulator base address
	mov	dx, sp		; an address just past the end of the memory
	mov	cl, 4
	shr	dx, cl		; convert to paragraph count
	mov	ax, cs
	add	ax, dx		; emulator image base address

	; initialise memory maps
	lea	di, [bp+mmadj]	; initialise adjusted memory tables
	call	mminit		; to be relative to the image base address
	lea	di, [bp+mmraw]	; initialise raw memory tables
	xor	ax, ax		; to base segment 0
	call	mminit

	; terminate argument vector
	mov	di, 0x80	; argument vector length pointer
	xor	bx, bx		; clear bx
	mov	bl, [di]	; load argument vector length
	inc	di		; beginning of arguments
	mov	[bx+di], bh	; NUL-terminate arguments

	mov	al, 0x20	; AL = ' '
	mov	cx, bx		; max length: argument vector length
	repe	scasb		; find first non-space
	dec	di		; go back to first non-space
	mov	[file], di	; remember file name for later

	cmp	byte [di], bh	; was there any argument at all?
	jne	.0

	; no argument was given: print usage and exit
	mov	si, usage
	call	puts		; print usage
.die:	mov	ax, 0x4c01	; error level 1 (failure)
	int	0x21		; 0x4c: TERMINATE PROGRAM

	; an argument was given: try to open it
.0:	mov	dx, di		; file name
	mov	ax, 0x3d00	; AL=00 (open file for reading)
	int	0x21		; 0x3d: OPEN EXISTING FILE
	jnc	.1		; did an error occur?

	; opening, reading, or closing the file failed
.err:	push	cs
	pop	ds		; set DS = CS
	mov	si, [file]	; load file name for error message
	call	perror		; print error message
	jmp	.die		; and die

	; opening the file was succesful: load program image
.1:	xchg	bx, ax		; the file handle is needed in BX
	mov	ax, sp
	mov	dx, cs		; DX:AX = image base

	; read until EOF
	; TODO: reject image if it is too large
.2:	test	ax, ax		; check if AX > 0x8000 to make sure that
	jns	.3		; 0x8000 bytes remain in DX:AX
	sub	ax, 0x8000	; if not, shift segments to ensure this
	add	dh, 0x08
.3:	mov	ds, dx
	xchg	dx, ax		; buffer address at DS:DX
	mov	cx, 0x8000	; number of bytes to read
	mov	ah, 0x3f
	int	0x21		; 0x3f: READ FROM FILE VIA HANDLE
	jc	.err		; IO error?
	test	ax, ax		; end of file reached?
	jz	.eof

	add	ax, dx		; compute new base address
	mov	dx, ds		; move buffer address to DX:AX
	jmp	.2		; and read some more data

	; close the image file
.eof:	mov	ah, 0x3e	; BX still contains the handle here
	int	0x21		; 0x3e: CLOSE A FILE HANDLE
	jc	.err

	; restore DS = CS
	push	cs
	pop	ds

	; initial register set up: image base address (R0)
	mov	bx, [bp+imgbase] ; load image base
	mov	dx, bx
	xor	ax, ax		; DX:AX contains the image base
	call	seglin		; as a linear address
	ldrlo	0, ax		; write load address to R0
	ldrhi	0, dx

	; initial register set up: memory size (R1)
	mov	dx, [2]		; load first segment past program image from PSP
	sub	dx, bx		; compute number of paragraphs available
	xor	ax, ax		;  to the program
	call	seglin		; and convert to a linear address
	ldrlo	1, ax		; write memory size to R1
	ldrhi	1, dx

	; initial register set up: stack pointer and reset vector
	mov	ds, bx		; load DS with emulated address space
	xor	si, si		; vector table begin
	lea	di, rlo(13)	; DI = &SP
	movsw			; load initial SP, low half
	movsw			; load initial SP, high half
	add	di, 4		; advance past LR
	movsw			; load initial PC (reset vector), low half
	movsw			; load initial PC (reset vector), high half

	call	run		; emulate a Cortex M0

	strlo	al, 0		; load error level from R0
	mov	ah, 0x4c
	int	0x21		; 0x4c: TERMINATE PROGRAM

	section	.data
usage	db	"Usage: PALANQIN CORTEXM0.IMG", 0

	section	.bss
	align	2
file	resw	1		; image file name

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Emulator State                                                             ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

	; the emulator state structure
	struc	st
regs	resd	16		; ARM registers
hi	equ	2		; advances low register half to high half
flags	resw	1		; CPU flags in 8086 format
				; only CF, ZF, SF, and OF are meaningful
zsreg	resw	1		; pointer to the register according to which the
				; zero and sign flags shall be set or 0 if they
				; are already set up correctly.  This is R0--R7.

	; instruction decoding state variables
	; immediate operands are zero/sign-extended to 16 bit
	; register operands are represented by a pointer to the appropriate
	; regs array member
oprC	resw	1		; third operand (towards least significant bit)
oprB	resw	1		; second operand (middle of the instruction)
oprA	resw	1		; first operand (towards most significant bit)

	; Memory maps.  The high word of an ARM address sans the address space
	; nibble is looked up in this table to form a segment.  The low word
	; stays the same, forming an offset.
mmsize	equ	16		; number of entries in a memory map
mmraw	resw	mmsize		; memory maps for unadjusted memory
imgbase	equ	$		; image base address (first seg. of adj. table)
mmadj	resw	mmsize		; memory maps for adjusted memory
	endstruc

	section	.bss
	alignb	4
state	resb	st_size		; BP points here

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Instruction Simulation                                                     ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

	section	.text
	; load one instruction into AX and advance PC past it.
	; trashes BX, CX, and SI.
ifetch:	strhi	cx, 15		; high part of PC for translation
	call	translate	; BX: handler, CX:AX: address
	strlo	si, 15		; CX:SI = translated address
	and	si, ~1		; clear thumb bit
	add	word rlo(15), 2	; PC += 2
	adc	word rhi(15), 0
	jmp	[bx+mem.ldrh]	; AX = instruction, tail call

	; run the emulation until we need to stop for some reason
run:	push	cs		; set up es = ds = cs
	push	cs
	pop	ds
	pop	es
.step:	call	step		; simulate one instruction
	jmp	.step		; do it again and again

	; simulate one instruction.  Assumes ES=DS=CS.
step:	call	ifetch		; fetch instruction
	int3
	push	ax		; push a copy of the current instruction
	mov	bx, ax		; and keep another one in AX
	mov	cl, 5		; mask out the instruction's top 4 bits
	rol	bx, cl		; and form a table offset
	and	bx, 0x1e	; bx = ([insn] & 0xf000) >> (16 - 4) << 1
	mov	dx, 0x1c	; mask for use with the decode handlers
	lea	si, [bp+regs]	; for use with the decode handlers
	lea	di, [bp+oprC]	; for use with the decode handlers
				; which also assume that AX=insn
	call	[dtXXXX+bx]	; decode operands
	pop	ax		; the current instruction
	jmp	[htXXXX+bx]	; execute behaviour

	section	.data
	align	2, db 0

	; decoder jump table: decode the operands according
	; to the top 4 instruction bits
	; the decode handlers decode the various instruction fields and store
	; their contents into the emulator state variables.
dtXXXX:	dw	imm5rr		; 000XX imm5 / Rm / Rd
	dw	d0001		; 000110 Rm / Rn / Rd
				; 000111 imm3 / Rn / Rd
	dw	rimm8		; 001XX Rdn / imm8
	dw	rimm8
	dw	d0100		; 010000 Rm / Rdn
				; 010001 DN / rm / Rdn
				; 01001 Rd / imm8
	dw	rrr		; 0101 Rm / Rn / Rd
	dw	imm5rr		; 011XX imm5 / Rn / Rd
	dw	imm5rr
	dw	imm5rr		; 1000X imm5 / Rn / Rd
	dw	rimm8		; 1001X Rd / imm8
	dw	rimm8		; 1010X Rd / imm8
	dw	dnone		; 1011 misc. instructions
	dw	rimm8		; 1100X Rn / imm8
	dw	dnone		; 1101 cond / imm8
	dw	dnone		; 11100 imm11
				; 11101 32 bit instructions
	dw	dnone		; 1111 32 bit instructions

	section	.text

	; special decode handler for instructions starting with 0001
	; 000XX... where XX != 11 is decoded as imm5 / reg / reg,
	; 000110... as reg / reg / reg and
	; 000111 as imm5 / reg / reg again
d0001:	mov	cx, ax		; make a copy of insn
	and	ch, 0xc		; mask the two bits 00001100
	cmp	ch, 0x08	; are these set to 10?
	je	rrr		; if yes, decode as rrr
				; else fall through and decode as imm5rr

	; decode handler for imm5 / reg / reg
	; instruction layout: XXXX XAAA AABB BCCC
	; for d000, we don't treat the 00011 opcodes specially;
	; the handler for these instructions must manually decode the
	; register from oprA
imm5rr:	mov	cx, ax		; CX = XXXX XAAA AABB BCCC
	shl	ax, 1		; AX = XXXX AAAA ABBB CCC0
	shl	ax, 1		; AX = XXXA AAAA BBBC CC00
	and	ax, dx		; AX = 0000 0000 000C CC00
	add	ax, si		; AX = &regs[C]
	stosw			; oprC = &regs[C]
	shr	cx, 1		; AX = 0XXX XXAA AAAB BBCC
	mov	ax, cx
	and	ax, dx		; AX = 0000 0000 000B BB00
	add	ax, si		; AX == &regs[B]
	stosw			; oprB = &regs[B]
	xchg	ax, cx		; AX = 0XXX XXAA AAAB BBCC
	mov	cl, 5		; prepare shift amount
	shr	ax, cl		; AX = 0000 00XX XXXA AAAA
	and	ax, 0x1f	; AX = 0000 0000 000A AAAA
	stosw			; oprA = A
	ret

	; decode handler for reg / imm8
	; instruction layout: XXXX XBBB CCCC CCCC
	; TODO: oprC isn't actually needed anywere.  Can we skip setting it up?
rimm8:	xor	cx, cx
	xchg	ah, cl		; AX=imm8, CX=reg
	stosw			; oprC=imm8
	xchg	ax, cx		; AX=reg
	shl	ax, 1		; AX = XXXX XXXX XXXX BBB0
	shl	ax, 1		; AX = XXXX XXXX XXXB BB00
	and	ax, dx		; AX = 0000 0000 000B BB00
	add	ax, si		; AX = &regs[B]
	stosw			; oprB = &regs[B]
	ret

	; decode handler for reg / reg / reg
	; instruction layout: XXXXXXXAAABBBCCC
rrr:	mov	cx, ax		; CX = XXXX XXXA AABB BCCC
	shl	ax, 1		; CX = XXXX XXAA ABBB CCC0
	shl	ax, 1		; CX = XXXX XAAA BBBC CC00
	and	ax, dx		; CX = 0000 0000 000C CC00
	add	ax, si		; AX = &regs[C]
	stosw			; oprC = &regs[C]
	mov	ax, cx
	shr	ax, 1		; CX = 0XXX XXXX AAAB BBCC
	mov	cx, ax
	and	ax, dx		; CX = 0000 0000 000B BB00
	add	ax, si		; CX = &regs[B]
	stosw			; oprB = &regs[B]
	xchg	ax, cx
	mov	cl, 3		; prepare shift amount
	shr	ax, cl		; CX = 0000 XXXX 000A AABB
	and	ax, dx		; CX = 0000 0000 000A AA00
	add	ax, si		; CX = &regs[A]
	stosw			; oprA = &regs[A]
	ret

	; special decode handler for instructions starting with 0100
	; 010000... is decoded as imm5 / reg / reg (imm4, really)
	; 010001... is decoded in a special manner
	; 01001... is decoded as reg / imm8
d0100:	test	ah, 0x08	; is this 01001...?
	jnz	rimm8		; if yes, decode as reg / imm8
	test	ah, 0x04	; else, is this 010000...?
	jz	imm5rr		; if yes, decode as imm5 / reg / reg

	; if we get here, we have instruction 0100 01XX CBBB BCCC
	; note how the C operand is split in two!
	mov	cx, ax		; CX = 0100 01XX CBBB BCCC
	shl	ax, 1		; AX = XXXX XXXC BBBB CCC0
	shl	ax, 1		; AX = XXXX XXCB BBBC CC00
	and	ax, dx		; AX = 0000 0000 000C CC00
	shr	cx, 1		; CX = 0010 001X XCBB BBCC
	mov	dx, cx		; make a copy for masking
	shr	dx, 1		; DX = 0001 0001 XXCB BBBC
	and	dx, 0x20	; DX = 0000 0000 00C0 0000
	or	ax, dx		; AX = 0000 0000 00CC CC00
	add	ax, si		; AX = &regs[C]
	stosw			; oprC = &regs[C]
	xchg	cx, ax
	and	ax, 0x3c	; AX = 0000 0000 00BB BB00
	add	ax, si		; AX = &regs[B]
	stosw			; oprB = &regs[B]
	; fallthrough

	; decode handlers that perform no decoding
dnone:	ret

	section	.data
	align	2, db 0

	; first level handler jump table: decode the top 4 instruction bits
htXXXX	dw	h000		; 000XX shift immediate
	dw	h000		; 00011 add/subtract register/immediate
	dw	h001		; 001XX add/subtract/compare/move immediate
	dw	h001
	dw	h0100		; 010000XXXX data-processing register
				; 010001XX special data processing
				; 01001 LDR  (literal pool)
	dw	h0101		; 0101 load/store register offset
	dw	h011 		; 011XX load/store word/byte immediate offset
	dw	h011
	dw	h1000		; 1000X load/store halfword immediate offset
	dw	h1001		; 1001X load from/store to stack
	dw	h1010		; 1010X add to SP or PC
	dw	h1011		; 1011XXXX miscellaneous instructions
	dw	h1100		; 1100X load/store multiple
	dw	h1101		; 1101XXXX conditional branch
				; 11011110 undefined instruction
				; 11011111 service call
	dw	h1110		; 11100 B (unconditional branch)
	dw	h1111		; 11110 branch and misc. control

	; Jump table for instructions 000XXX interleaved with the
	; jump table for add/subtract/compare/move immediate.
ht000XX	dw	h00000		; LSLS Rd, Rm, #imm5
ht001XX	dw	h00100		; MOVS Rd, #imm8
	dw	h00001		; LSR Rd, Rm, #imm5
	dw	h00101		; CMP  Rd, #imm8
	dw	h00010		; ASR Rd, Rm, #imm5
	dw	h00110		; ADDS Rd, #imm8
	dw	h00011		; ADD/SUB register/immediate
	dw	h00111		; SUBS Rd, #imm8

	; jump table for data-processing register instructions
ht010000XXXX:
	dw	h0100000000	; ANDS Rdn, Rm
	dw	h0100000001	; EORS Rdn, Rm
	dw	h0100000010	; LSLS Rdn, Rm
	dw	h0100000011	; LSRS Rdn, Rm
	dw	h0100000100	; ASRS Rdn, Rm
	dw	h0100000101	; ADCS Rdn, Rm
	dw	h0100000110	; SBCS Rdn, Rm
	dw	h0100000111	; RORS Rdn, Rm
	dw	h0100001000	; TST  Rn, Rm
	dw	h0100001001	; RSBS Rdn, Rm, #0
	dw	h0100001010	; CMP  Rn, Rm
	dw	h0100001011	; CMN  Rn, Rm
	dw	h0100001100	; ORRS Rdn, Rm
	dw	h0100001101	; MULS Rdn, Rm
	dw	h0100001110	; BICS Rdn, Rm
	dw	h0100001111	; MVNS Rdn, Rm

	; jump table for special data-processing instructions
ht010001XX:
	dw	h01000100	; ADD Rdn, Rm
	dw	h01000101	; CMP Rd, Rm
	dw	h01000110	; MOV Rd, Rm
	dw	h01000111	; BX Rm, BLX Rm

	; jump table for load/store register offset
ht0101	dw	str		; 0101000 STR   Rt, [Rn, Rm]
	dw	strh		; 0101001 STRH  Rt, [Rn, Rm]
	dw	strb		; 0101010 STRB	Rt, [Rn, Rm]
	dw	ldrsb		; 0101011 LDRSB Rt, [Rn, Rm]
	dw	ldr		; 0101100 LDR   Rt, [Rn, Rm]
	dw	ldrh		; 0101101 LDRH  Rt, [Rn, Rm]
	dw	ldrb		; 0101110 LDRB  Rt, [Rn, Rm]
	dw	ldrsh		; 0101111 LDRSH Rt, [Rn, Rm]

	; jump table for the miscellaneous instructions 1011XXXX
	; instructions in parentheses are not available on Cortex-M0
	; cores and generate an undefined instruction exception.
ht1011XXXX:
	dw	h10110000	; ADD/SUB SP, SP, #imm7
	dw	h10110001	; (CBZ Rn, #imm5)
	dw	h10110010	; SXTB/SXTH/UXTB/UXTH
	dw	h10110011	; (CBZ Rn, #imm5)
	dw	h1011010	; PUSH {...}
	dw	h1011010	; PUSH {..., LR}
	dw	h10110110	; CPS
	dw	h10110111	; escape hatch
	dw	h10111000	; undefined
	dw	h10111001	; (CBNZ Rn, #imm5)
	dw	h10111010	; REV/REV16/REVSH
	dw	h10111011	; (CBNZ Rn, #imm5)
	dw	h1011110	; POP {...}
	dw	h1011110	; POP {..., LR}
	dw	h10111110	; BKPT #imm8
	dw	h10111111	; (IT), hints

	; jump table for (un)signed byte/halfword extend
	; and jump table for reverse bytes
htB2BA	dw	hB200		; SXTH Rd, Rm
	dw	hB201		; SXTB Rd, Rm
	dw	hB210		; UXTH Rd, Rm
	dw	hB211		; UXTB Rd, Rm
	dw	hBA00		; REV Rd, Rm
	dw	hBA01		; undefined
	dw	hBA10		; REV16 Rd, Rm
	dw	hBA11		; REVSH Rd, Rm

	section	.text

	; 000XXAAAAABBBCCC shift immediate
	; 00011XYAAABBBCCC add/subtract register/immediate
h000:	mov	bl, ah		; BL = 000XXAAA
	shr	bl, 1		; BL = 0000XXAA
	and	bx, 0x0c	; BL = 0000XX00
	mov	si, [bp+oprB]	; SI = &regs[Rm]
	mov	di, [bp+oprC]	; DI = &regs[Rd]
	mov	[bp+zsreg], di	; set SF and ZF according to Rd
	mov	cx, [bp+oprA]	; CL = imm5 (or Rm for 000110...)
	test	cx, cx		; if CL == 0 and it's not LSLS, adjust to 32
				; for ADDS/SUBS, CX is a reg and is never 0.
	jz	.adj
	jmp	[ht000XX+bx]	; call instruction specific handler

.adj:	test	bl, bl		; is this LSLS?
	jz	h0000000000
	mov	cl, 32		; otherwise adjust CL to 32
	jmp	[ht000XX+bx]

	; 0000000000BBBCCC MOVS Rd, Rm
	; 01000110CBBBBCCC MOV Rd, Rm
h01000110:
	call	fixRd		; fix flags if needed
h0000000000:
	movsw			; Rd = Rm
	movsw
	ret

	; 00011XXAAABBBCCC add/sub register/immediate
h00011:	test	ah, 4		; is this register or immediate?
	jnz	h000111

	; 0001100AAABBBCCC ADDS Rd, Rn, Rm
	; 0001101AAABBBCCC SUBS Rd, Rn, Rm
	mov	bx, cx		; need BX to form an address
	mov	cx, [si]	; DX:CX = Rn
	mov	dx, [si+hi]
	test	ah, 2		; is this ADDS or SUBS?
	jnz	.subs
	add	cx, [bx]	; DX:CX = Rn + Rm
	adc	dx, [bx+hi]
	jmp	.fi
.subs:	sub	cx, [bx]	; DX:CX = Rn - Rm
	sbb	dx, [bx+hi]
	cmc			; adjust CF to ARM conventions
.fi:	mov	[di], cx	; Rd = DX:CX
	mov	[di+hi], dx
	pushf			; remember all flags
	pop	word [bp+flags]
	ret

	; 0001110AAABBBCCC ADDS Rd, Rn, #imm3
	; 0001111AAABBBCCC SUBS Rd, Rn, #imm3
h000111:and	cx, 7		; CX = #imm3
	test	ah, 2		; is this ADDS or SUBS?
	jnz	.subs
	xor	ax, ax
	add	cx, [si]	; AX:CX = Rn + #imm3
	adc	ax, [si+hi]
	jmp	.fi
.subs:	lodsw			; AX:DX = Rn
	xchg	ax, dx
	lodsw
	sub	dx, cx		; AX:DX = Rn - #imm3
	sbb	ax, 0
	cmc			; adjust CF to ARM conventions
.fi:	xchg	ax, dx		; DX:AX = AX:DX
	stosw			; Rd = DX:AX
	mov	[di], dx
	pushf			; remember all flags
	pop	word [bp+flags]
	ret

	; 001XXBBBCCCCCCCC add/subtract/compare/move immediate
h001:	mov	bl, ah		; BL = 001XXAAA
	shr	bl, 1		; BL = 0001XXAA
	and	bx, 0xc		; BL = 0000XX00
	xor	si, si		; SI = 0
	mov	di, [bp+oprB]	; DI = &regs[Rd]
	mov	ah, 0		; AX = #imm8
	mov	[bp+zsreg], di	; set SF and ZF according to Rd
	jmp	[ht001XX+bx]	; call instruction specific handler

	; 00100BBBCCCCCCCC MOVS Rd, #imm8
h00100:	stosw			; Rd = #imm8
	mov	[di], si
	ret

	; 00101BBBCCCCCCCC CMP Rn, #imm8
h00101:	mov	dx, [di+hi]
	cmp	[di], ax	; Rd(lo) - #imm8 (for ZF and borrow)
	lahf			; remember ZF according to Rd(lo) - #imm8 in AH
	sbb	dx, si		; Rd - #imm8 (for CF, SF, OF, and ZF)
	cmc			; adjust CF to ARM conventions
	pushf			; load flags in DX
	pop	dx
	or	ah, ~ZF		; isolate ZF in AH
	and	dl, ah		; set ZF in DX if Rn == Rm
	mov	[bp+flags], dx	; save flags in flags
	mov	[bp+zsreg], si	; mark flags as fixed
	ret

	; 00110BBBCCCCCCCC ADDS Rd, #imm8
h00110:	add	[di], ax	; Rd += AX
	add	[di+hi], si
	pushf			; remember flags
	pop	word [bp+flags]
	ret

	; 00111BBBCCCCCCCC SUBS Rd, #imm8
h00111:	sub	[di], ax	; Rd -= AX
	sbb	[di+hi], si
	cmc			; adjust CF to ARM conventions
	pushf			; and remember flags
	pop	word [bp+flags]
	ret

	; 010000AAAABBBCCC data-processing register
	; 010001AACBBBBCCC special data processing
	; 01001BBBCCCCCCCC LDR Rd, [PC, #imm8]
h0100:	mov	di, [bp+oprC]	; DI = &Rdn
	mov	si, [bp+oprB]	; SI = &Rm
	test	ah, 0x08	; is this LDR Rd, [PC, #imm8]?
	jnz	.ldr
	test	ah, 0x04	; else, is this special data processing?
	jnz	.sdp		; otherwise, it's data-processing register
	mov	[bp+zsreg], di	; set flags according to Rdn
	mov	bx, [bp+oprA]	; BX = 0000AAAA
	shl	bx, 1		; BX = 000AAAA0
	jmp	[ht010000XXXX+bx]
.sdp:	mov	bl, ah		; BL = 010001AA
	and	bx, 0x03	; BX = 000000AA
	shl	bx, 1		; BX = 00000AA0
	jmp	[ht010001XX+bx]
	; 01001BBBCCCCCCCC LDR Rt, [PC, #imm8]
.ldr:	xchg	di, si		; set up DI = &Rt, SI = #imm8
	call	fixRd		; set flags on Rd if needed
	strlo	ax, 15		; AX = R15(lo)
	shl	si, 1		; SI = #imm8 << 2 + 2
	inc	si
	shl	si, 1
	xor	cx, cx
	add	ax, si		; CX:AX = R15 + #imm8
	adc	cx, rhi(15)
	and	al, ~3		; align to word boundary
	jmp	ldr		; perform the actual load

	; 0100000000BBBCCC ANDS Rdn, Rm
h0100000000:
	lodsw			; AX = Rm(lo)
	and	[di], ax	; Rdn(lo) &= Rm(lo)
	lodsw			; AX = Rm(hi)
	and	[di+hi], ax	; Rdn(hi) &= Rm(hi)
	ret

	; 0100000001BBBCCC EORS Rdn, Rm
h0100000001:
	lodsw			; AX = Rm(lo)
	xor	[di], ax	; Rdn(lo) ^= Rm(lo)
	lodsw			; AX = Rm(hi)
	xor	[di+hi], ax	; Rdn(hi) ^= Rm(hi)
	ret

	; 0100000010BBBCCC LSLS Rdn, Rm
h0100000010:
	mov	cl, [si]	; CL = Rm
	test	cl, cl		; no shift?
	jz	h00000.ret
	cmp	cl, 32		; shift by more than 32?
	ja	h00000.hi
	mov	si, di		; SI = DI = Rdn
	; fallthrough

	; 00000AAAAABBBCCC LSLS Rd, Rm, #imm5
h00000:	cmp	cl, 16		; shift by more than 16?
	jbe	.lo

	; shift by 16 < CL <= 32
	sub	cl, 16		; adjust shift amount to 0 < CL <= 16
	lodsw			; AX = Rm(lo),
	shl	ax, cl		; AX = Rm(lo) << imm5 - 16
	mov	word [di], 0	; Rd(lo) = 0
	mov	[di+hi], ax	; Rd(hi) = Rm(lo) << imm5 - 16
	lahf			; update CF in flags
	mov	[bp+flags], ah
.ret:	ret

	; shift by 0 < CL <= 16
.lo:	lodsw			; AX = Rm(lo)
	mov	bx, ax		; keep a copy
	shl	ax, cl		; AX = Rm(lo) << #imm5
	stosw			; Rd(lo) = Rm(lo) << #imm5
	mov	si, [si]	; SI = Rm(hi)
	shl	si, cl		; SI = Rm(hi) << #imm5
	lahf			; update CF in flags
	mov	[bp+flags], ah
	sub	cl, 16		; CL = 16 - CL
	neg	cl
	shr	bx, cl		; BX = Rm(lo) >> 16 - #imm5
	lea	ax, [bx+si]	; AX = Rm(hi) << #imm5 | Rm(lo) >> 16 - #imm5
	stosw			; Rd(hi) = Rm << #imm5 (hi)
	ret

	; shift by 32 < CL
.hi:	xor	ax, ax		; Rd = 0
	stosw
	stosw
	mov	byte [bp+flags], al ; clear CF in flags
	ret

	; 0100000011BBBCCC LSRS Rdn, Rm
h0100000011:
	mov	cl, [si]	; CL = Rm
	test	cl, cl		; no shift?
	jz	h00001.ret
	cmp	cl, 32		; shift by more than 32?
	ja	h00000.hi	; same as lsl by more than 32
	mov	si, di		; SI = DI = Rdn

	; 00001AAAAABBBCCC LSRS Rd, Rm, #imm5
h00001:	cmp	cl, 16		; shift by more than 16?
	jbe	.lo

	; shift by 16 < CL <= 32
	sub	cl, 16		; CL = imm5 - 16
	mov	ax, [si+hi]	; AX = Rm(hi)
	shr	ax, cl		; AX = Rm(hi) >> imm5 - 16
	stosw			; Rd(lo) = Rm(hi) >> imm5 - 16
	mov	word [di], 0	; Rd(hi) = 0
	lahf			; update CF, SF, and ZF in flags
	mov	[bp+flags], ah
.ret:	ret

	; shift by 0 < CL <= 16
.lo:	lodsw			; DX = Rm(lo)
	xchg	ax, dx
	shr	dx, cl		; DX = Rm(lo) >> #imm5
	lahf			; update CF in flags
	mov	[bp+flags], ah
	lodsw			; AX = Rm(hi)
	mov	si, ax		; keep a copy
	shr	si, cl		; SI = Rm(hi) >> #imm5
	mov	[di+hi], si	; Rd(hi) = Rm(hi) >> #imm5
	sub	cl, 16		; CL = 16 - CL
	neg	cl
	shl	ax, cl		; AX = Rm(hi) << 16 - #imm5
	or	ax, dx		; AX = Rm(hi) << 16 - #imm5 | Rm(lo) >> #imm5
	stosw			; Rd(lo) = Rm >> #imm5 (lo)
	ret

	; 0100000100BBBCCC ASRS Rdn, Rm
h0100000100:
	mov	cl, [si]	; CL = Rm
	test	cl, cl		; shift by 0?
	jz	h00010.ret
	cmp	cl, 32		; shift by 32 or more?
	jae	h00010.hi
	mov	si, di		; SI = DI = Rdn

	; 00010AAAAABBBCCC ASRS Rd, Rm, #imm5
h00010:	cmp	cl, 16		; shift by more than 16?
	jbe	.lo

	; shift by 16 < CL < 32
	sub	cl, 16		; CL = imm5 - 16
	mov	ax, [si+hi]	; AX = Rm(hi)
	sar	ax, cl		; AX = Rm(hi) >> imm5 - 16
	cwd			; DX = Rm(hi) < 0 ? -1 : 0
	stosw			; Rd = DX:AX
	xchg	ax, dx
	stosw
	mov	[bp+flags], dl	; update CF in flags
.ret:	ret

	; shift by 0 < CL <= 16
.lo:	lodsw			; DX = Rm(lo)
	xchg	ax, dx
	shr	dx, cl		; DX = Rm(lo) >> #imm5
	lahf			; update CF in flags
	mov	[bp+flags], ah
	lodsw			; AX = Rm(hi)
	mov	si, ax		; keep a copy
	sar	si, cl		; SI = Rm(hi) >> #imm5
	mov	[di+hi], si	; Rd(hi) = Rm(hi) >> #imm5
	sub	cl, 16		; CL = 16 - CL
	neg	cl
	shl	ax, cl		; AX = Rm(hi) << 16 - #imm5
	or	ax, dx		; AX = Rm(hi) << 16 - #imm5 | Rm(lo) >> #imm5
	stosw			; Rd(lo) = Rm >> #imm5 (lo)
	ret

	; shift by 32 <= CL
.hi:	mov	ah, [si+hi+1]	; AH = Rm(hi) (high byte)
	cwd			; DX = Rm < 0 ? -1 : 0
	mov	[bp+flags], dl	; set CF depending on DX
	xchg	ax, dx		; Rd = DX:DX
	stosw
	stosw
	ret

	; 0100000101BBBCCC ADCS Rdn, Rm
h0100000101:
	mov	ah, [bp+flags]	; restore CF from flags
	sahf
	lodsw			; AX = Rm(lo)
	adc	[di], ax	; Rdn(lo) += Rm(lo) + CF
	lodsw			; AX = Rm(hi)
	adc	[di+hi], ax	; Rdn(hi) += Rm(hi) + CF
	pushf			; remember CF and OF in flags
	pop	word [bp+flags]
	ret

	; 0100000110BBBCCC SBCS Rdn, Rm
h0100000110:
	mov	ah, [bp+flags]	; restore CF from flags
	sahf
	cmc			; adapt CF from ARM conventions
	lodsw
	sbb	[di], ax	; Rdn(lo) -= Rm(lo) - 1 + CF
	lodsw			; AX = Rm(hi)
	sbb	[di+hi], ax	; Rdn(hi) += Rm(hi) + CF
	cmc			; adjust CF to ARM conventions
	pushf			; remember CF and OF in flags
	pop	word [bp+flags]
	ret

	; 0100000111BBBCCC RORS Rdn, Rm
h0100000111:
	mov	cl, [si]	; CL = Rm
	test	cl, cl		; no rotate?
	jz	.ret
	mov	ax, [di]	; DX:AX = Rdn
	mov	dx, [di+hi]
	and	cl, 31		; mask out useless extra rotates
	cmp	cl, 16		; rotating by 16 or more?
	jb	.lo
	sub	cl, 16		; if yes, reduce to rotation to 0 <= cl < 16
	xchg	ax, dx		; with a pre-rotate by 16

	; rotate by 0 <= CL < 16
.lo:	mov	bx, ax		; BX = Rdn(lo)
	shr	ax, cl		; AX = Rdn(lo) >> CL
	mov	si, dx		; SI = Rdn(hi)
	shr	si, cl		; DX = Rdn(hi) >> CL
	sub	cl, 16		; CL = 16 - CL
	neg	cl
	shl	dx, cl		; DX = Rdn(hi) << 16 - CL
	or	ax, dx		; AX = Rdn(lo) >> CL | Rdn(hi) << 16 - CL
	stosw			; Rdn(lo) = Rdn ror CL (lo)
	shl	bx, cl		; BX = Rdn(lo) << 16 - CL
	lea	ax, [bx+si]	; AX = Rdn(hi) >> CL | Rdn(lo) << 16 - CL
	stosw			; Rdn(hi) = Rdn ror CL (hi)
	rol	ah, 1		; shift Rdn sign bit into LSB of AH
	mov	[bp+flags], ah	; and deposit into flags as CF
.ret:	ret

	; 0100001000BBBCCC TST Rn, Rm
h0100001000:
	lodsw			; DX:AX = Rm
	mov	dx, [si]
	test	[di], ax	; set ZF according to Rm(lo) & Rn(lo)
	lahf
	mov	al, ah		; AL = Rm(lo) & Rn(lo) flags
	test	[di+hi], dx	; set ZF and SF according Rm(hi) & Rn(hi)
	lahf
	or	al, ~ZF		; isolate ZF in AL
	and	ah, al		; AH = SF, ZF according to Rm(lo) & Rn(lo)
	mov	al, [bp+flags]
	and	ax, (ZF|SF)<<8|~(ZF|SF)&0xff
				; mask AL to all but ZF and SF,
				; AH to just ZF and SF
	or	al, ah		; merge the two
	mov	[bp+flags], al	; write them back
	mov	word [bp+zsreg], 0 ; mark flags as being fixed
	ret

	; 0100001001BBBCCC RSBS Rd, Rm, #0
	; CF = Rn == 0
h0100001001:
	lodsw			; AX = Rm(lo)
	xor	dx, dx
	neg	ax		; AX = -AX
	stosw
	sbb	ax, ax		; AX = Rm(lo) == 0 ? -1 : 0
	sub	ax, [si]	; AX = -Rm(hi) - carry
	stosw			; Rd = DX:AX
	cmc			; adjust CF to ARM conventions
	lahf			; remember CF in flags
	mov	[bp+flags], ah
	ret

	; 0100001010BBBCCC CMP Rn, Rm
	; 01000101CBBBBCCC CMP Rn, Rm
h01000101:
h0100001010:
	lodsw			; AX = Rm(lo)
	mov	dx, [di+hi]	; DX = Rn(hi)
	cmp	[di], ax	; set flags according to Rn(lo) - Rm(lo)
	lahf			; and remember ZF in AH
	sbb	dx, [si]	; set CF, SF, and OF according to Rn - Rm
	cmc			; adapt CF to ARM conventions
.flags:	pushf			; load flags into DX
	pop	dx
	or	ah, ~ZF		; isolate ZF in AH
	and	dl, ah		; set ZF in DX if Rn == Rm
	mov	[bp+flags], dx	; save flags in flags
	mov	word [bp+zsreg], 0 ; mark flags as fixed
	ret

	; 0100001011BBBCCC CMN Rn, Rm
h0100001011:
	lodsw			; AX = Rm(lo)
	add	ax, [di]	; set flags according to Rn(lo) + Rm(lo)
	lahf			; and remember ZF in AH
	lodsw			; AX = Rm(hi)
	adc	ax, [di+hi]	; set CF, SF, and OF according to Rn - Rm
	jmp	h0100001010.flags ; rest is the same as with CMP Rn, Rm

	; 0100001100BBBCCC ORRS Rd, Rm
h0100001100:
	lodsw			; AX = Rm(lo)
	or	[di], ax	; Rdn(lo) |= Rm(lo)
	lodsw			; AX = Rm(hi)
	or	[di+hi], ax	; Rdn(hi) |= Rm(hi)
	ret

	; 0100001101BBBCCC MULS Rd, Rm
h0100001101:
	lodsw			; AX = Rm(lo)
	mov	bx, ax		; remember a copy
	mul	word [di]	; DX:AX = Rm(lo) * Rd(lo)
	mov	cx, dx		; CX = Rm(lo) * Rd(lo) (hi)
	xchg	bx, ax		; BX = Rm * Rd (lo), AX = Rm(lo)
	mul	word [di+hi]	; AX = Rm(lo) * Rd(hi) (lo), DX = junk
	add	cx, ax		; CX = Rm(lo)*Rd(lo) (hi) + Rm(lo)*Rd(hi) (lo)
	lodsw			; AX = Rm(hi)
	mul	word [di]	; AX = Rm(hi)*Rd(lo), DX = junk
	add	cx, ax		; AX = Rm * Rd (hi)
	xchg	ax, bx		; Rd = CX:BX
	stosw
	mov	[di], cx
	ret

	; 0100001110BBBCCC BICS Rd, Rm
h0100001110:
	lodsw			; AX = Rm(lo)
	not	ax		; AX = ~Rm(lo)
	and	[di], ax	; Rdn(lo) |= Rm(lo)
	lodsw			; AX = Rm(hi)
	not	ax		; AX = ~Rm(hi)
	and	[di+hi], ax	; Rdn(hi) |= Rm(hi)
	ret


	; 0100001111BBBCCC MVNS Rd, Rm
h0100001111:
	lodsw			; AX = Rm(lo)
	not	ax		; AX = ~Rm(lo)
	stosw			; Rd(lo) = ~Rm(lo)
	lodsw			; AX = Rm(hi)
	not	ax		; AX = ~Rm(hi)
	stosw			; Rd(hi) = ~Rm(hi)
	ret

	; ADD Rd, Rm
h01000100:
	call	fixRd		; fix flags if needed
	lodsw			; AX = Rm(lo)
	add	[di], ax	; Rd(lo) += Rm(lo)
	lodsw			; AX = Rm(hi)
	adc	[di+hi], ax	; Rd(hi) += Rm(hi) + C
	ret

	; 010001110BBBBXXX BX Rm
	; 010001111BBBBXXX BLX Rm
h01000111:
	lea	di, rlo(15)	; DI = &PC
	test	al, 0x80	; is this BLX?
	jnz	.blx
	movsw			; PC = Rm
	movsw
	test	byte rlo(15), 1	; is the thumb bit set?
	jz	.arm
.ret:	ret
.blx:	lodsw			; DX:AX = PC
	mov	dx, [di]
	dec	di		; DI = &PC
	dec	di
	movsw			; PC = Rm
	movsw
	mov	rlo(14), ax	; LR = DX:AX
	mov	rhi(14), dx
	test	byte rlo(15), 1	; is the thumb bit set?
	jnz	.ret
.arm:	jmp	undefined

	; 0101XXXAAABBBCCC load/store register offset
h0101:	mov	bl, ah		; BL = 0101XXXA
	and	bx, 0x0e	; BL = 0000XXX0
	mov	di, [bp+oprC]	; DI = &Rt
	call	fixRd		; fix flags for Rt
	mov	si, [bp+oprB]	; SI = &Rn
	lodsw			; CX:AX = Rn
	mov	cx, [si]
	mov	si, [bp+oprA]	; SI = &rm
	add	ax, [si]	; CX:AX = Rn + Rm
	adc	cx, [si+hi]
	jmp	[ht0101+bx]	; perform instruction behaviour

	; 01100AAAAABBBCCC STR	Rt, [Rn, #imm5]
	; 01110AAAAABBBCCC STRB Rt, [Rn, #imm5]
	; 01101AAAAABBBCCC LDR  Rt, [Rn, #imm5]
	; 01111AAAAABBBCCC LDRB Rt, [Rn, #imm5]
h011:	mov	si, [bp+oprB]	; SI = &Rn
	mov	di, [bp+oprC]	; DI = &Rt
	push	ax		; remember the instruction
	call	fixRd		; fix flags on Rt
	pop	dx		; DX = instruction
	mov	bx, [bp+oprA]	; BX = #imm5
	lodsw			; CX:AX = Rn
	mov	cx, [si]
	test	dh, 0x10	; byte instruction?
	jnz	.b		; if not, scale immediate
	shl	bx, 1		; BX = #imm5 << 2
	shl	bx, 1
.b:	add	ax, bx		; CX:AX = Rn + #imm5 << 2
	adc	cx, 0
	test	dh, 0x08	; is this LDR(B)?
	jz	.str		; otherwise it is STR(B)
	test	dh, 0x10	; is this LDR or LDRB?
	jnz	.ldrb
	jmp	ldr
.ldrb:	jmp	ldrb
.str:	test	dh, 0x10	; is this STR or STRB?
	jnz	.strb
	jmp	str
.strb:	jmp	strb

	; 10000AAAAABBBCCC STRH Rt, [Rn, #imm5]
	; 10001AAAAABBBCCC LDRH Rt, [Rn, #imm5]
h1000:	mov	si, [bp+oprB]	; SI = &Rn
	mov	di, [bp+oprC]	; DI = &Rt
	push	ax		; remember the instruction
	call	fixRd		; fix flags on Rt
	pop	dx		; and restore it to DX
	mov	bx, [bp+oprA]	; BX = #imm5
	shl	bx, 1		; BX = #imm5 << 1
	lodsw			; CX:AX = Rn
	mov	cx, [si]
	add	ax, bx		; CX:AX = Rn + #imm5 << 1
	adc	cx, 0
	test	dh, 0x08	; LDRH?
	jz	.strh		; otherwise it is STRH
	jmp	ldrh
.strh:	jmp	strh

	; 10010BBBCCCCCCCC STR Rd, [SP, #imm8]
	; 10011BBBCCCCCCCC LDR Rd, [SP, #imm8]
h1001:	push	ax		; remember the instruction
	mov	di, [bp+oprB]	; DI = &Rt
	call	fixRd
	pop	dx		; DX = instruction
	xor	ax, ax
	mov	al, [bp+oprC]	; AX = #imm8
	shl	ax, 1		; AX = #imm8 << 2
	shl	ax, 1
	xor	cx, cx
	add	ax, rlo(13)	; CX:AX = SP + #imm8
	adc	cx, rhi(13)
	test	dh, 0x08	; is this LDR?
	jz	.str		; otherwise it is STR
	jmp	ldr
.str:	jmp	str

	; 10100BBBCCCCCCCC ADD Rd, PC, #imm8 (ADR Rd, label)
	; 10101BBBCCCCCCCC ADD Rd, SP, #imm8
h1010:	mov	di, [bp+oprB]	; di = &Rd
	test	ah, 0x8		; is this ADD Rd, SP, #imm8?
	jnz	.sp		; if not, this is ADD Rd, PC, #imm8
	call	fixRd		; fix up flags to Rd if needed
	mov	ax, 2		; DX:AX = 2
	cwd
	add	ax, rlo(15)	; DX:AX = R15 + 2
	adc	dx, rhi(15)
	and	al, ~3		; aligned to word boundary
	jmp	.fi
.sp:	call	fixRd		; fix up flags to Rd if needed
	strlo	ax, 13		; load SP into DX:AX
	strhi	dx, 13
.fi:	mov	cx, [bp+oprC]	; CX = #imm8 >> 2
	shl	cx, 1
	shl	cx, 1		; CX = #imm8
	add	ax, cx		; DX:AX == DX:AX + CX (= PC/SP + #imm8)
	adc	dx, 0
	stosw			; Rd = DX:AX
	mov	[di], dx
	ret

	; miscellaneous instructions
	; 4 more instruction bits decode the subinstruction
h1011:	mov	bl, ah		; BL = 1011XXXX
	shl	bl, 1		; BL = 011XXXX0
	and	bx, 0x1e	; BX = 000XXXX0
	jmp	[ht1011XXXX+bx]

	; 101100000AAAAAAA ADD SP, SP, #imm7
	; 101100001AAAAAAA SUB SP, SP, #imm7
h10110000:
	xor	ah, ah		; AX = 00000000XAAAAAAA
	shl	al, 1		; AX = 00000000AAAAAAA0, CF = ADD/SUB
	jc	.sub
	shl	ax, 1		; AX = #imm7 (in words)
	add	rlo(13), ax	; R13 += #imm7
	adc	word rhi(13), 0
	ret
.sub:	shl	ax, 1		; AX = #imm7 (in words)
	sub	rlo(13), ax	; R13 += #imm7
	sbb	word rhi(13), 0
	ret

	; 101100B1BBBBBCCC (CBZ Rd, #imm5)
h10110001 equ	undefined

	; 10111010XXBBBCCC reverse bytes
h10111010:
	inc	ah		; AX = 10111011XXBBBCCC
				; to select the second set of jumps in htB2BA
	; fallthrough

	; 10110010XXBBBCCC (un)signed extend byte/word
	; 10111011XXBBBCCC reverse bytes (adjusted opcode)
h10110010:
	mov	dx, 0x1c	; mask for extracting the instruction fields
	lea	si, [bp+regs]
	mov	di, ax		; DI = #### ###X XXBB BCCC
	shl	di, 1		; DI = #### ##XX XBBB CCC0
	shl	di, 1		; DI = #### #XXX BBBC CC00
	and	di, dx		; DI = 0000 0000 000C CC00
	add	di, si		; DI = &regs[C]

	shr	ax, 1		; AX = 0### #### XXXB BBCC
	mov	bx, ax
	and	ax, dx		; AX = 0000 0000 000B BB00
	add	si, ax		; SI = &regs[B]

	mov	cl, 4
	shr	bx, cl		; BX = 0000 0### #### XXXB
	and	bx, 0xe		; BX = 0000 0000 0000 XXX0
	call	fixRd		; set flags on Rd if needed
	lodsw			; AX = Rm(lo), SI += 2
	jmp	[htB2BA+bx]	; jump to instruction handler

	; 1011001000AAABBB SXTH Rd, Rm
hB200:	cwd			; DX:AX = SXTH(Rm(lo))
	stosw			; Rd = DX:AX
	mov	[di], dx
	ret

	; 1011001001AAABBB SXTB Rd, Rm
hB201:	cbw			; DX:AX = SXTB(Rm(lo))
	cwd
	stosw			; Rd = DX:AX
	mov	[di], dx
	ret

hB210:	; 1011001010AAABBB UXTH Rd, Rm
	stosw			; Rd = UXTH(Rm(lo))
	xor	ax, ax
	stosw
	ret

	; 1011001011AAABBB UXTB Rd, Rm
hB211:	stosb			; Rd = UXTB(Rm(lo))
	xor	ax, ax
	stosb
	stosw
	ret

	; 101100B1BBBBBCCC (CBZ Rd, #imm5)
h10110011 equ	undefined

	; 10110100BBBBBBBB PUSH {...}
	; 10110101BBBBBBBB PUSH {..., LR}
h1011010:
	lea	di, rlo(0)	; SI = &R0
.loop:	shr	al, 1		; advance bit-mask to next register
	ja	.done		; any registers left (CF != 0 or ZF != 0)?
	jnc	.nostr		; store current register?
	mov	si, rlo(13)	; CX:SI = SP
	mov	cx, rhi(13)
	sub	si, 4		; CX:SI -= 4
	sbb	cx, 0
	mov	rlo(13), si	; SP -= 4
	mov	rhi(13), cx
	push	ax		; remember the instruction
	call	str		; deposit register into memory
	pop	ax		; restore the instruction
.nostr:	add	di, 4		; advance to next register
	jmp	.loop		; and try again if any registers are left
.done:	test	ax, 0x100	; store LR?
	jz	.nolr
	mov	si, rlo(13)	; CX:SI = SP
	mov	cx, rhi(13)
	sub	si, 4		; CX:SI -= 4
	sbb	cx, 0
	mov	rlo(13), si	; SP -= 4
	mov	rhi(13), cx
	lea	di, rlo(14)	; DI = &LR
	jmp	str		; deposit LR into memory
.nolr:	ret

	; 10110110011ABBBB CPS
h10110110:
	todo

	; 10110111 escape hatch
	; this instruction allows the program to interact with the host
h10110111:
	cmp	al, B7max	; is this a valid escape hatch opcode?
	ja	.ud		; if not, treat as undefined instruction
	xchg	ax, bx		; BL = operation code
	xor	bh, bh		; BX = operation code
	shl	bl, 1		; form table index
	jmp	[htB7+bx]
.ud:	jmp	undefined	; treat as undefined instruction

	; 10111000XXXXXXXX undefined
h10111000 equ	undefined

	; 101110A1AAAAABBB (CBNZ Rd, #imm5)
h10111001 equ	undefined

	; 10111010XXAAABBB reverse bytes (see h10110010)

	; 1011101000AAABBB REV Rd, Rm
hBA00:	xchg	ax, dx
	lodsw			; AX = Rm(hi)
	xchg	ah, al		; reverse Rm(hi)
	stosw			; Rd = REV(Rm)
	xchg	dh, dl		; reverse Rm(lo)
	mov	[di], dx
	ret

	; 1011101001AAABBB REV16 Rd, Rm
hBA01:	xchg	ah, al		; reverse Rm(lo)
	stosw			; Rd(lo) = REV(Rm(lo))
	lodsw			; AX = Rm(hi)
	xchg	ah, al		; reverse Rm(hi)
	stosw			; Rd(hi) = REV(Rm(hi))
	ret

	; 1011101010XXXXXX undefined
hBA10	equ	undefined

	; 1011101011AAABBB REVSH Rd, Rm
hBA11:	xchg	ah, al		; reverse Rm(lo)
	cwd			; sign extend into DX:AX
	stosw			; Rd = DX:AX
	mov	[di], dx
	ret

	; 101110A1AAAAABBB (CBNZ Rd, #imm5)
h10111011 equ	undefined

	; 10111100AAAAAAAA POP {...}
	; 10111101AAAAAAAA POP {..., PC}
h1011110:
	xchg	ax, cx		; preserve AX
	call	fixflags	; fix flags (easier than checking for each reg)
	xchg	ax, cx		; restore AX
	lea	di, rlo(0)	; SI = &R0
.loop:	shr	al, 1		; advance bit-mask to next register
	ja	.done		; any registers left (CF != 0 or ZF != 0)?
	jnc	.noldr		; store current register?
	mov	si, rlo(13)	; CX:SI = SP
	mov	cx, rhi(13)
	add	word rlo(13), 4	; SP += 4
	adc	word rhi(13), 0
	push	ax		; remember the instruction
	call	ldr		; load register from memory
	pop	ax		; restore the instruction
.noldr:	add	di, 4		; advance to next register
	jmp	.loop		; and try again if any registers are left
.done:	test	ax, 0x100	; load PC?
	jz	.nopc
	mov	si, rlo(13)	; CX:SI = SP
	mov	cx, rhi(13)
	add	word rlo(13), 4	; SP += 4
	adc	word rhi(13), 0
	lea	di, rlo(15)	; DI = &PC
	jmp	ldr		; load PC from memory
.nopc:	ret

	; 10111110AAAAAAAA BKPT #imm8
h10111110:
	todo

	; hint instructions
	; 10111111XXXXYYYY (IT) where YYYY != 0000
	; 1011111100000000 NOP
	; 1011111100010000 YIELD
	; 1011111100100000 WFE
	; 1011111100110000 WFI
	; 1011111101000000 SEV
h10111111:
	test	al, 0xf		; is this IT?
	je	.it		; if yes, generate UNDEFINED
	ret			; else, treat as NOP
.it:	jmp	undefined	; generate an undefined instruction exception

	; 11000AAABBBBBBBB STMIA Rn!, {...}
	; 11001AAABBBBBBBB LDMIA Rn!, {...}
h1100:	todo

	; 1101BBBBCCCCCCCC B<c> <label>
	; 11011110CCCCCCCC UDF #imm8
	; 11011111CCCCCCCC SVC #imm8
h1101:	mov	bl, ah		; BL = 1101AAAA
	xchg	bx, ax		; BX = insn, AL = 1101AAAA
	shl	al, 1		; AL = 101AAAA0
	shl	al, 1		; AL = 01AAAA00
	cbw			; AX = 01AAAA00
	add	ax, h1101xxxx-0x40
	xchg	di, ax		; DI = h1101xxxx[AAAA]
	call	fixflags	; set up true flags in flags
	push	word [bp+flags]
	popf			; set up SF, OF, ZF, and CF according to flags
	jmp	di

	; conditional branch jump table.  Each entry is four bytes long and
	; corresponds to one conditional code.  If the jump is not taken, the
	; table entry returns, otherwise it branches to .taken.  UDF and SVC
	; receive special treatment.
	align	4, int3
h1101xxxx:
	; 0000 BEQ
	je	.taken
	ret
	int3

	; 0001 BNE
	jne	.taken
	ret
	int3

	; 0010 BCS, BHS
	jc	.taken
	ret
	int3

	; 0011 BCC, BLO
	jnc	.taken
	ret
	int3

	; 0100 BMI
	js	.taken
	ret
	int3

	; 0101 BPL
	jns	.taken
	ret
	int3

	; 0110 BVS
	jo	.taken
	ret
	int3

	; 0111 BVC
	jno	.taken
	ret
	int3

	; 1000 BHI
	cmc
	ja	.taken
	ret

	; 1001 BLS
	cmc
	jbe	.taken
	ret

	; 1010	BGE
	jge	.taken
	ret
	int3

	; 1011	BLT
	jl	.taken
	ret
	int3

	; 1100	BGT
	jg	.taken
	ret
	int3

	; 1101	BLE
	jle	.taken
	ret
	int3

	; 1110	UDF (BAL)
	jmp	undefined

	; 1111	SVC (BNV)
	align	4, int3
	jmp	.svc

.taken:	xchg	ax, bx		; AL = #imm8
	cbw			; AX = #imm8
	inc	ax		; AX = #imm8 + 1
	shl	ax, 1		; AX = #imm8:0 + 2
	cwd			; DX:AX = #imm8 + 2
	add	rlo(15), ax	; R15 += #imm8 + 2 + 2
	adc	rhi(15), dx	; note that R15 had already been advanced by two
				; in the initial ifetch call.
	ret

	; 11011111XXXXXXXX SVC #imm8
.svc:	todo			; todo

	; 11100CCCCCCCCCCC B #imm11
	; 11101XXXXXXXXXXX 32-bit instructions
h1110:	test	ah, 0x08	; is this B #imm11?
	jnz	.udf		; or is it a 32 bit instruction?
	mov	cl, 5		; sign extend #imm11 into AX
	shl	ax, cl		; AX=CCCCCCCCCCC00000
	dec	cx		; keep #imm11 as a word offset
	sar	ax, cl		; AX=CCCCCCCCCCCCCCC0
	inc	ax		; AX = #imm11 + 2
	inc	ax
	cwd			; DX:AX = #imm11 + 2
	add	rlo(15), ax	; R15 += #imm11 + 2
	adc	rhi(15), dx
	ret
.udf:	jmp	undefined	; 11101XXXXXXXXXXX is undefined

	; 32 bit instructions
	; 11110011100XAAAA 10X0XXXXBBBBBBBB MSR spec_reg, Rn
	; 1111001111XXXXXX 10X0AAAABBBBBBBB MRS Rn, spec_reg
	; 111100111011XXXX 10X0XXXX0100AAAA DSB #option
	; 111100111011XXXX 10X0XXXX0101AAAA DMB #option
	; 111100111011XXXX 10X0XXXX0110AAAA ISB #option
	; 111101111111AAAA 1010AAAAAAAAAAAA UDF #imm16
h1111:	test	ah, 0x08	; is this 11111XXXXXXXXXXX?
	jnz	h1110.udf	; if yes, execute as undefined.
	push	ax		; remember low instruction word
	call	ifetch		; fetch high instruction word into AX
	pop	dx		; AX:DX holds the instruction
	test	ax, ax		; is AX 0XXXXXXXXXXXXXXX?
	jns	.udf		; if yes, execute as undefined.
	mov	cl, ah
	and	cl, 0x50	; mask high word to 0X0X00000000
	jz	.notbl		; for the case X0X0XXXXXXXX
	jpo	.udf		; if it was not X1X1XXXXXXXX
	; fallthrough to BL #imm24

	; 11110SBBBBBBBBBB 11J1JAAAAAAAAAAA BL #imm24
	lea	si, rlo(15)	; SI = &PC
	lea	di, rlo(14)	; DI = &LR
	movsw			; LR = PC
	movsw
	mov	cl, 4
	shl	dx, cl		; DX = 0SBBBBBBBBBB0000
	mov	bx, ax		; keep a copy of BX for later
	shl	bx, 1		; BX = 1J1JAAAAAAAAAAA0
	and	bh, 0x0f	; BX = 0000AAAAAAAAAAA0
	or	bh, dl		; BX = BBBBAAAAAAAAAAA0
	shl	dx, 1		; DX = SBBBBBBBBBB00000
	mov	cl, 9
	shr	dx, cl		; DX = SSSSSSSSSSBBBBBB
	test	ah, 0x10	; is J1 set?
	jnz	.j1
	xor	dl, 0x40	; DX = SSSSSSSSSyBBBBBB
.j1:	test	ah, 0x20	; is J2 set?
	jnz	.j2
	xor	dl, 0x80	; DX = SSSSSSSSxyBBBBBB, xy = ~SS^JJ
.j2:	add	rlo(15), bx	; PC += #imm24:0
	adc	rhi(15), dx
	ret

.udf:	sub	word rlo(15), 2	; move PC back to current instruction + 2
	sbb	word rhi(15), 0
	jmp	undefined	; and treat as an undefined instruction
.notbl:	todo

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Flag Manipulation                                                          ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

	; set ZF and SF in flags according to zsreg
	; trashes AX and DX.
fixflags:
	push	si		; preserve SI
	mov	si, [bp+zsreg]
	test	si, si		; flags already fixed?
	jz	.nofix
.entry:	xor	dx, dx
	cmp	[si], dx	; set ZF according to R(lo)
	lahf
	mov	al, ah		; AL = R(lo) flags
	cmp	[si+hi], dx	; set ZF according to R(hi)
	lahf
	or	al, ~ZF		; isolate ZF in AL
	and	ah, al		; AH = SF, ZF according to R
	mov	al, [bp+flags]
	and	ax, (ZF|SF)<<8|~(ZF|SF)&0xff
				; mask AL to all but ZF and SF,
				; AH to just ZF and SF
	or	al, ah		; merge the two
	mov	[bp+flags], al	; write them back
	mov	[bp+zsreg], dx	; and mark the flags as being fixed
.nofix:	pop	si		; restore SI
	ret

	; compare DI with [bp+zsreg].  If both are equal, fix the flags.
	; trashes AX, DX.  Preserves DI which may not be zero.
	; the intent is to save the flags if Rd == [bp+zsreg] and flag
	; recovery would otherwise be impossible.
fixRd:	push	si		; preserve SI
	mov	si, [bp+zsreg]
	cmp	si, di		; is Rd == [bp+zsreg]?
	je	fixflags.entry	; if yes, fix it up
	pop	si		; otherwise restore SI and return
	ret

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Memory Access                                                              ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

	; Translate an ARM address and return a pointer to a set of
	; accessor functions.  These functions can then be used to
	; access the memory behind it.  The offset in AX can be adjusted
	; by the caller to perform multiple memory accesses on related
	; addresses, but once it overflows, translate must be called anew.
	; Expects an ARM address high word in CX.  Returns a translated segment
	; in CX and a pointer to a structure of accessor functions in BX.
	; Preserves all other registers.
	section	.text
translate:
	mov	bl, ch		; load address space nibble
	and	bx, 0xf0	; isolate address space nibble
	shr	bx, 1		; form a table index
	shr	bx, 1
	shr	bx, 1
	jmp	[xlttab+bx]	; call nibble-specific translator

	; Address space translators.  One for each part of the address space.
	section	.data
	align	2, db 0
xlttab:	dw	xltadj		; 00000000--000fffff adjusted memory
	dw	xltraw		; 10000000--100fffff unadjusted memory
	dw	xltadj		; 20000000--200fffff adjusted memory (mirror)
	dw	xltraw		; 30000000--300fffff unadjusted memory (mirror)
	dw	xltio		; 40000000--4000ffff I/O ports
	dw	xltnone		; 50000000--5fffffff open bus
	dw	xltnone		; 60000000--6fffffff open bus
	dw	xltnone		; 70000000--7fffffff open bus
	dw	xltnone		; 80000000--8fffffff open bus
	dw	xltnone		; 90000000--9fffffff open bus
	dw	xltnone		; a0000000--afffffff open bus
	dw	xltnone		; b0000000--bfffffff open bus
	dw	xltnone		; c0000000--cfffffff open bus
	dw	xltnone		; d0000000--dfffffff open bus
	dw	xltppb		; e0000000--e00fffff private peripheral bus PPB
	dw	xltvendor	; f0000000--ffffffff vendor use

	; translator for imgbase adjusted memory
	section	.text
xltadj:	xchg	si, cx		; preserve old SI
	shl	si, 1		; form a table index
	and	si, 0x001e	; mask out relevant nibble
	mov	si, [bp+si+mmadj] ; translate address
	xchg	cx, si		; restore SI and move address to CX
	mov	bx, memmem	; load accessor function address
	ret

	; translator for unadjusted memory
xltraw:	xchg	si, cx		; preserve old SI
	shl	si, 1		; form a table index
	and	si, 0x001e	; mask out relevant nibble
	mov	si, [bp+si+mmraw] ; translate address
	xchg	cx, si		; restore SI and move address to CX
	mov	bx, memmem	; load accessor function address
	ret

	; translator for I/O ports
xltio:	mov	bx, memio	; I/O memory access
	ret

	; translator for open bus
xltnone:mov	bx, memnone	; no memory access
	ret

	; translator for the private peripheral bus (PPB)
xltppb:	todo

	; translator for the vendor-use area
xltvendor:
	todo

	; Memory accessor functions.  These functions all follow the same
	; convention: CX:SI holds the translated address, DX:AX holds the datum
	; to be written (in case of a store function).  On return, DX:AX holds
	; the datum loaded (in case of a load function).  In case of a store
	; function, DX:AX is undefined on return.  The function preserves CX:SI
	; as well as all segment registers.  The caller is responsible for
	; checking for alignment problems.
	struc	mem
.ldr	resw	1		; load word
.ldrh	resw	1		; load half word
.ldrb	resw	1		; load byte
.str	resw	1		; store word
.strh	resw	1		; store half word
.strb	resw	1		; store byte
	endstruc

	; memory accessor functions for ordinary memory
	section	.data
memmem	dw	ldrmem
	dw	ldrhmem
	dw	ldrbmem
	dw	strmem
	dw	strhmem
	dw	strbmem

	; memory accessor functions for I/O ports
memio	dw	ldrio
	dw	ldrhio
	dw	ldrbio
	dw	strio
	dw	strhio
	dw	strbio

	; memory accessor functions for open bus
memnone	times	6 dw ldstnone

	; load word from memory
	section	.text
ldrmem:	push	ds		; remember old DS
	mov	ds, cx		; set up DS for DS:SI memory load
	mov	ax, [si]	; DX:AX = [CX:SI]
	mov	dx, [si+2]
	pop	ds		; restore DS
	ret

	; load half word from memory
ldrhmem:mov	dx, ds		; remember old DS
	mov	ds, cx		; set up DS for DS:SI memory load
	mov	ax, [si]	; AX = [CX:SI]
	mov	ds, dx		; restore DS
	ret

	; load byte from memory
ldrbmem:mov	dx, ds		; remember old DS
	mov	ds, cx		; set up DS for DS:SI memory load
	mov	al, [si]	; AL = [CX:SI]
	mov	ds, dx		; restore DS
	ret

	; store word to memory
strmem:	push	ds		; remember old DS
	mov	ds, cx		; set up DS for DS:SI store
	mov	[si], ax	; [CX:SI] = AX:DX
	mov	[si+2], dx
	pop	ds
	ret

	; store half word to memory
strhmem:mov	dx, ds		; remember old DS
	mov	ds, cx		; set up DS for DS:SI store
	mov	[si], ax	; [CX:SI] = AX
	mov	ds, dx		; restore old DS
	ret

	; store byte to memory
strbmem:mov	dx, ds		; remember old DS
	mov	ds, cx		; set up DS for DS:SI store
	mov	[si], al	; [CX:SI] = AL
	mov	ds, dx		; restore old DS
	ret

	; load word from I/O port
ldrio:	mov	dx, si		; set up DX for IN instruction
	in	ax, dx		; load low word
	inc	dx
	inc	dx		; advance to high port
	push	ax		; remember low word
	in	ax, dx		; load high word
	xchg	ax, dx		; move high word to dx
	pop	ax		; restore low word
	ret

	; load half word from I/O port
ldrhio:	mov	dx, si		; set up DX for IN instruction
	in	ax, dx		; load word
	ret

	; load byte from I/O port
ldrbio:	mov	dx, si		; set up DX for IN instruction
	in	al, dx		; load byte
	ret

	; store word to I/O port
strio:	push	dx		; temporarily remember high word
	mov	dx, si		; set up DX for OUT instruction
	out	dx, ax		; store low word
	inc	dx		; advance to high port
	inc	dx
	pop	ax		; load high word
	out	dx, ax		; store high word
	ret

	; store half word to I/O port
strhio:	mov	dx, si		; set up DX for OUT instruction
	out	dx, ax		; store word
	ret

	; store byte to I/O port
strbio:	mov	dx, si		; set up DX for OUT instruction
	out	dx, al		; store byte
	ret

	; store nothing/load 0xffffffff
ldstnone:
	mov	ax, 0xffff	; DX:AX = -1
	mov	dx, ax
	ret

	; load word from ARM address CX:AX and deposit into the register
	; pointed to by DI.  Preserve DI.
ldr:	call	translate	; CX:AX: translated address, BX: handler
	xchg	ax, si		; CX:SI = CX:AX
	call	[bx+mem.ldr]	; DX:AX = mem[CX:SI]
	mov	[di], ax	; Rt = mem[CX:SI]
	mov	[di+hi], dx
	ret

	; load halfword from ARM address CX:AX and deposit into the register
	; pointed to by DI.
ldrh:	call	translate	; CX:AX: translated address, BX: handler
	xchg	ax, si		; CX:SI = CX:AX
	call	[bx+mem.ldrh]	; AX = mem[CX:SI]
	stosw			; Rt = mem[CX:SI]
	xor	ax, ax
	stosw
	ret

	; load signed halfword from ARM address CX:AX and deposit into the
	; the register pointed to by DI.
ldrsh:	call	translate	; CX:AX: translated address, BX: handler
	xchg	ax, si		; CX:SI = CX:AX
	call	[bx+mem.ldrh]	; AX = mem[CX:SI]
	cwd			; DX:AX = mem[CX:SI]
	stosw			; Rt = mem[CX:SI] (sign extended)
	mov	[di], dx
	ret

	; load byte from ARM address CX:AX, zero-extend and deposit into the
	; register pointed to by DI.
ldrb:	call	translate	; CX:AX: translated address, BX: handler
	xchg	ax, si		; CX:SI = CX:AX
	call	[bx+mem.ldrb]	; AL = mem[CX:SI]
	stosb			; Rt = mem[CX:SI]
	xor	ax, ax
	stosb
	stosw
	ret

	; load signed byte from ARM address CX:AX and deposit into the
	; the register pointed to by DI.
ldrsb:	call	translate	; CX:AX: translated address, BX: handler
	xchg	ax, si		; CX:SI = CX:AX
	call	[bx+mem.ldrb]	; AL = mem[CX:SI]
	cbw			; AX = mem[CX:SI]
	cwd			; DX:AX = mem[CX:SI]
	stosw			; Rt = mem[CX:SI] (sign extended)
	mov	[di], dx
	ret

	; store word from register pointed to by DI to ARM address CX:AX.
	; Preserve DI.
str:	call	translate	; CX:AX: translated address, BX: handler
	xchg	ax, si		; CX:SI = CX:AX
	mov	ax, [di]	; DX:AX = Rt
	mov	dx, [di+hi]
	jmp	[bx+mem.str]

	; store halfword from register pointed to by DI to ARM address CX:AX.
strh:	call	translate	; CX:AX: translated address, BX: handler
	xchg	ax, si		; CX:SI = CX:AX
	mov	ax, [di]	; AX = Rt
	jmp	[bx+mem.strh]

	; store byte from register pointed to by DI to ARM address CX:AX.
strb:	call	translate	; CX:AX: translated address, BX: handler
	xchg	ax, si		; CX:SI = CX:AX
	mov	al, [di]	; AL = Rt
	jmp	[bx+mem.strb]

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Address Space Conversion                                                   ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

	; Initialise memory map DI with consecutive segments starting with the
	; segment in AX.  Trashes AX, CX, and DI.
	section	.text
mminit:	mov	cx, mmsize	; fill one memory map
.loop:	stosw			; deposit segment into memory map
	add	ax, 0x1000	; advance to next segment
	loop	.loop		; and loop mmsize times
	ret

	; convert DX:AX into a linear address in DX:AX.  Trashes CX
seglin:	mov	cl, 4
	rol	dx, cl		; dx = ds >> 12 | ds << 4
	mov	cx, dx
	and	dx, 0xf		; dx = ds >> 12
	and	cl, 0xf0	; cx = ds <<  4
	add	ax, cx		; ax = ax + (ds >> 12)
	adc	dx, 0		; apply carry
	ret

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Exceptions, Events, and Interrupts                                         ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

	; generate an undefined instruction exception
	section	.text
undefined:
	todo			; TODO

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Escape Hatches                                                             ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

	; jump table for escape hatch instructions
	section	.data
	align	2, db 0
htB7	dw	hB700		; terminate emulation
	dw	hB701		; dump registers
	dw	hB702		; console output
	dw	hB703		; console input (no echo)
	dw	hB704		; check input status
B7max	equ	($-htB7-2)/2	; highest escape hatch number used

	; register dump template
	section	.data
dump	db	"R0  "
.r0	db          "XXXXXXXX  " ; start of the field for R0 within the dump
.field	equ	$-dump		; length of one register field
	db	              "R1  XXXXXXXX  R2  XXXXXXXX  R3  XXXXXXXX", 13, 10
	db	"R4  XXXXXXXX  R5  XXXXXXXX  R6  XXXXXXXX  R7  XXXXXXXX", 13, 10
	db	"R8  XXXXXXXX  R9  XXXXXXXX  R10 XXXXXXXX  R11 XXXXXXXX", 13, 10
	db	"R12 XXXXXXXX  SP  XXXXXXXX  LR  XXXXXXXX  PC  XXXXXXXX", 13, 10
.endr	db	"NZCV  "
.nzcv	db	"----", 13, 10, 0

	; b700 terminate emulation
	section	.text
hB700:	strlo	al, 0		; AL = R0(lo) (error level)
	mov	ah, 0x4c
	int	0x21		; 0x4c: TERMINATE PROGRAM

	; b701 dump registers
hB701:	call	fixflags	; set up flags
	mov	di, dump.r0	; load R0 value field
	lea	si, [bp+regs]	; SI = &R0
.regs:	lodsw			; AX = reg(lo)
	push	ax
	lodsw			; AX = reg(hi), advance to next register
	call	tohex		; convert high half to hex
	pop	ax		; AX = reg(lo)
	call	tohex		; convert low half to hex
	add	di, dump.field - 8 ; advance to next field
	cmp	di, dump.endr	; finished dumping registers?
	jb	.regs
	mov	di, dump.nzcv	; advance SI to NZCV field
	mov	ax, '--'	; clear all flags in the template
	stosw
	stosw
	push	word [bp+flags]	; set up flags according to emulator state
	popf
	jns	.nn		; is SF (N) set in flags?
	mov	byte [di-4], 'N'
.nn:	jnz	.nz		; is ZF (Z) set in flags?
	mov	byte [di-3], 'Z'
.nz:	jnc	.nc		; is CF (C) set in flags?
	mov	byte [di-2], 'C'
.nc:	jno	.nv		; is OF (V) set in flags?
	mov	byte [di-1], 'V'
.nv:	mov	si, dump	; load register dump template into DS:SI
	jmp	puts		; dump registers and return

	; b702 console output
hB702:	strlo	dl, 0		; AL = R0(lo)
	mov	ah, 0x02
	int	0x21		; 0x02: DISPLAY OUTPUT
	ret

	; b703 console input (no echo)
hB703:	lea	di, rlo(0)	; di = &R0
	mov	word [di+hi], 0	; R0(hi) = 0
	mov	ah, 0x08
	int	0x21		; 0x08: NO ECHO CONSOLE INPUT
	test	al, al		; is this extended ASCII?
	jz	.ext		; if yes, read again for ext. ASCII character
	mov	ah, 0x00
	stosw			; R0(lo) = character
	ret
.ext:	int	0x21		; 0x08: NO ECHO CONSOLE INPUT
	mov	ah, 0x01	; mark as extended ASCII
	stosw			; R0(lo) = 0x100 + character
	ret

	; b704 check input status
hB704:	mov	ah, 0x0b
	int	0x21		; 0x0B: CHECK INPUT STATUS
	cbw			; AX: input status
	lea	di, rlo(0)
	stosw			; R0 = input status (0 or -1)
	stosw
	ret

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; IO Routines                                                                ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

	; hexadecimal digits
	section	.data
hextab	db	"0123456789ABCDEF"

	; convert number in AX to hexadecimal and write to DS:DI
	; trashes AX, BX, CX, and DX.  Advances DI by 4 bytes.
	section	.text
tohex:	mov	bx, hextab	; base address for xlat
	mov	cl, 4		; for shifting
	mov	dx, ax		; make a copy for later use
	rol	ax, cl		; shift digit X000 into place
	and	al, 0xf		; isolate digit
	xlat			; translate to hex
	stosb			; deposit into string
	mov	al, dh		; load digit 0X00
	and	al, 0xf		; isolate digit
	xlat			; translate to hex
	stosb			; deposit into string
	mov	al, dl		; load digit 00X0
	shr	al, cl		; shift into place
	xlat			; translate to hex
	stosb			; deposit into string
	xchg	ax, dx		; load digit 000X
	and	al, 0xf		; isolate digit
	xlat			; translate to hex
	stosb			; deposit into string
	ret

	; print string in ds:si to stdout
puts:	lodsb
	test	al, al		; end of string reached?
	jz	.end
	xchg	ax, dx		; DOS wants the character in dl
	mov	ah, 0x02
	int	0x21		; 0x02: WRITE CHARACTER TO STDOUT
	jmp	puts

.end:	ret

	; print ds:si and then a colon and a space
	; then print a message for the error in AX
perror:	push	ax		; remember error code
	call	puts		; print string

	push	cs
	pop	ds		; prepare ds
	mov	si, colsp	; ": "
	call	puts

	pop	si		; reload error code
	add	si, si
	mov	si, [errors+si]	; load error code
	call	puts		; print error message

	mov	si, crlf	; load "\r\n"
	call	puts

	ret

	; DOS error codes and messages
	section	.data
	align	2

errors	dw	.E00, .E01, .E02, .E03, .E04, .E05, .E06, .E07
	dw	.E08, .E09, .E0A, .E0B, .E0C, .E0D, .E0E, .E0F
	dw	.E10, .E11, .E12

.E00	db	"unknown error",0
.E01	db	"function number invalid",0
.E02	db	"file not found",0
.E03	db	"path not found",0
.E04	db	"too many open files",0
.E05	db	"access denied",0
.E06	db	"invalid handle",0
.E07	db	"memory control block destroyed",0
.E08	db	"insufficient memory",0
.E09	db	"memory block address invalid",0
.E0A	db	"environment invalid",0
.E0B	db	"format invalid",0
.E0C	db	"access code invalid",0
.E0D	db	"data invalid",0
.E0E	equ	.E00		; error code E is reserved
.E0F	db	"invalid drive",0
.E10	db	"attempt to remove current directory",0
.E11	db	"not same device",0
.E12	db	"no more files",0

colsp	db	": ",0

; Palanquin -- Cortex-M0 emulator for 8086
; Copyright (c) 2020 Robert Clausecker <fuz@fuz.su>

	cpu	8086		; restrict nasm to 8086 instructions

	section	.data
ident	db	"Copyright (c) 2020 Robert Clausecker <fuz@fuz.su>", 10, 13, 0

	section	.bss
	align	2
edata	equ	$		; must be the first thing in .bss

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Parameters                                                                 ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

stack	equ	0x100		; emulator stack size in bytes (multiple of 16)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Macros and Constants                                                       ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

	; load value into ARM register
%macro	ldrlo	2
	mov	[%1*2+reglo], %2
%endmacro

%macro	ldrhi	2
	mov	[%1*2+reghi], %2
%endmacro

	; store value of ARM register
%macro	strlo	2
	mov	%1, [%2*2+reglo]
%endmacro

%macro	strhi	2
	mov	%1, [%2*2+reghi]
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
	org	0x100

	; relocate the stack
start:	mov	sp, end+stack	; beginning of stack

	; print copyright notice
	mov	si, ident
	call	puts

	; initialise .bss
	xor	ax, ax
	mov	di, edata
	mov	cx, (end-edata)/2 ; .bss section length in words
	rep	stosw		; clear .bss

	; configure emulator base address
	mov	dx, sp		; an address just past the end of the memory
	mov	cl, 4
	shr	dx, cl		; convert to paragraph count
	mov	ax, cs
	add	ax, dx		; emulator image base address
	mov	[imgbase], ax

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
.2:	call	seglin
	call	linseg		; normalise address (to give us some space)
	mov	ds, dx
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
	mov	bx, [imgbase] 	; load image base
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
	lodsw			; load initial SP, low half
	ldrlo	cs:13, ax
	lodsw			; load initial SP, high half
	ldrhi	cs:13, ax
	lodsw			; load initial PC (reset vector), low half
	ldrlo	cs:15, ax
	lodsw			; load initial PC (reset vector), high half
	ldrhi	cs:15, ax

	call	run		; emulate a Cortex M0

	strlo	al, cs:0	; load error level from R0
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

	section	.bss
	alignb	4
state	equ	$
reglo	resw	16		; ARM registers, low  hwords
reghi	resw	16		; ARM registers, high hwords
hi	equ	reghi-reglo	; to turn a reglo pointer into a reghi pointer
pcaddr	resd	1		; location of the next instruction as a segment/
				; offset pair.  The PC register in the register
				; set is only updated as needed to increase
				; performance.
imgbase	resw	1		; emulator image base segment
flags	resw	1		; CPU flags in 8086 format
				; only CF, ZF, SF, and OF are meaningful

	; instruction decoding state variables
	; immediate operands are zero/sign-extended to 16 bit
	; register operands are represented by a pointer to the appropriate
	; reglo array member
oprC	resw	1		; third operand (towards least significant bit)
oprB	resw	1		; second operand (middle of the instruction)
oprA	resw	1		; first operand (towards most significant bit)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Instruction Simulation                                                     ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

	section	.text
	; run the emulation until we need to stop for some reason
run:	push	cs		; set up es = ds = cs
	push	cs
	pop	ds
	pop	es
	call	pcseg		; find the address of the next instruction
.step:	call	step		; simulate one instruction
	jmp	.step		; do it again and again

	; simulate one instruction.  Assumes ES=DS=CS.
step:	mov	bx, pcaddr	; save some bytes in the next few instructions
	les	si, [bx]	; load the program counter into DS:SI
	es	lodsw		; load an instruction
	mov	[bx], ax	; update pcaddr with new offset
	test	si, si		; did we overflow the segment?
	jnz	.1		; if yes, apply overflow to pcaddr
	add	byte [bx+3], 0x10
.1:	push	ax		; push a copy of the current instruction
	mov	bx, ax		; and keep another one in AX
	mov	cl, 5		; mask out the instruction's top 4 bits
	rol	bx, cl		; and form a table offset
	and	bx, 0x1e	; bx = ([insn] & 0xf000) >> (16 - 4) << 1
	mov	dx, 0xe		; mask for use with the decode handlers
	mov	si, reglo	; for use with the decode handlers
	mov	di, oprC	; for use with the decode handlers
				; which also assume that AX=insn
	call	[dtXXXX+bx]	; decode operands
	pop	ax		; the current instruction
	jmp	[htXXXX+bx]	; execute behaviour

	section	.data
	align	2

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
	cmp	ch, 0x8		; are these set to 10?
	je	rrr		; if yes, decode as rrr
				; else fall through and decode as imm5rr

	; decode handler for imm5 / reg / reg
	; instruction layout: XXXX XAAA AABB BCCC
	; for d000, we don't treat the 00011 opcodes specially;
	; the handler for these instructions must manually decode the
	; register from oprA
imm5rr:	mov	cx, ax		; keep a copy of insn for later
	shl	ax, 1		; form an offset into the register table
	and	ax, dx		; mask out operand C
	add	ax, si		; form a pointer to reglo[C]
	stosw			; oprC = &reglo[C]
	mov	ax, cx
	shr	ax, 1
	shr	ax, 1
	and	ax, dx		; mask out operand B
	add	ax, si
	stosw			; oprB = &reglo[B]
	xchg	ax, cx		; free CX for use with shr
	mov	cl, 6		; prepare shift amount
	shr	ax, cl		; shift immediate into place
	and	ax, 0x1f	; and mask it out
	stosw			; oprA = A
	ret

	; decode handler for reg / imm8
	; instruction layout: XXXX XBBB CCCC CCCC
rimm8:	xor	cx, cx
	xchg	ah, cl		; AX=imm8, CX=reg
	stosw			; oprC=imm8
	xchg	ax, cx		; AX=reg
	shl	ax, 1		; AX = XXXX XXXX XXXX BBB0
	and	ax, dx		; AX = 0000 0000 0000 BBB0
	add	ax, si		; form a pointer to reglo[B]
	stosw			; oprB = &reglo[B]
	ret

	; decode handler for reg / reg / reg
	; instruction layout: XXXXXXXAAABBBCCC
rrr:	mov	cx, ax		; keep a copy of insn for later
	shl	ax, 1		; form an offset into the register table
	and	ax, dx		; mask out operand C
	add	ax, si		; form a pointer to reglo[C]
	stosw			; oprC = &reglo[C]
	mov	ax, cx
	shr	ax, 1
	shr	ax, 1
	and	ax, dx		; mask out operand B
	add	ax, si
	stosw			; oprB = &reglo[B]
	xchg	ax, cx		; free CX for use with shr
	mov	cl, 5		; prepare shift amount
	shr	ax, cl		; shift operand A into place
	and	ax, dx		; and mask it out
	add	ax, si
	stosw			; oprA = &reglo[A]
	ret

	; special decode handler for instructions starting with 0100
	; 010000... is decoded as imm5 / reg / reg (imm4, really)
	; 010001... is decoded in a special manner
	; 01001... is decoded as reg / imm8
d0100:	test	ah, 0x8		; is this 01001...?
	jnz	rimm8		; if yes, decode as reg / imm8
	test	ah, 0x4		; else, is this 010000...?
	jz	imm5rr		; if yes, decode as imm5 / reg / reg

	; if we get here, we have instruction 0100 01XX CBBB BCCC
	; note how the C operand is split in two!
	mov	cx, ax		; keep a copy of insn for later
	shl	ax, 1		; AX = XXXX XXXC BBBB CCC0
	and	ax, dx		; AX = 0000 0000 0000 CCC0
	shr	cx, 1
	shr	cx, 1		; CX = 00XX XXXX XXCB BBB0
	mov	dx, cx		; make a copy for masking
	shr	dx, 1		; DX = 000X XXXX XXXC BBBB
	and	dx, 0x10	; DX = 0000 0000 000C 0000
	or	ax, dx		; AX = 0000 0000 000C CCC0
	add	ax, si
	stosw			; oprC = &reglo[C]
	xchg	cx, ax
	and	ax, 0x1e	; AX = 0000 0000 000B BBB0
	add	ax, si
	stosw			; oprB = &reglo[B]
	; fallthrough

	; decode handlers that perform no decoding
dnone:	ret

	section	.data
	align	2

	; first level handler jump table: decode the top 4 instruction bits
htXXXX:	dw	h000		; 000XX shift immediate
	dw	h000		; 00011 add/subtract register/immediate
	dw	h001		; 001XX add/subtract/compare/move immediate
	dw	h001
	dw	h0100		; 010100XXXX data-processing register
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

	; jump table for instructions 000XXX
	; we decode the MSB of the immediate because shifting by 16 or more
	; places generally requires different code than shifting by less
ht000:	dw	h000000		; LSL immediate (< 16)
	dw	h000001		; LSL immediate (> 15)
	dw	h000010		; LSR immediate (< 16)
	dw	h000011		; LSR immediate (> 15)
	dw	h000100		; ASR immediate (< 16)
	dw	h000101		; ASR immediate (> 15)
	dw	h000110		; ADD/SUB register
	dw	h000111		; ADD/SUB immediate

	; jump table for instructions 000XX where imm8 == 0
	; this requires special treatment as some shifts treat a 0
	; immediate as 32 while others treat it as 0.
ht000z:	dw	h00000z		; MOVS Rd, Rm
	dw	h00001z		; LSRS Rd, Rm, #32
	dw	h00010z		; ASRS Rd, Rm, #32
	dw	h000110		; ADD/SUB register

	section	.text

	; 000XXAAAAABBBCCC shift immediate
	; 00011XYAAABBBCCC add/subtract register/immediate
h000:	mov	bl, ah		; BL = 000XXAAA
	shr	bl, 1		; BL = 0000XXAA
	and	bx, 0xe		; BL = 0000XXA0
	mov	si, [oprB]	; SI = &reglo[Rm]
	mov	di, [oprC]	; DI = &reglo[Rd]
	mov	cl, [oprA]	; CL = imm5
	test	cl, cl		; is imm8 == 0?
	jz	.zero		; if yes, perform special handling
	jmp	[ht000+bx]	; call instruction specific handler

.zero:	shr	bl, 1		; BL = 00000XX0
	jmp	[ht000z+bx]	; call handler for imm8 = 0

	; 0000000000BBBCCC MOVS Rd, Rm
	; CV is preserved, NZ are set according to Rm
h00000z:mov	al, [flags]	; load CF, SF, and ZF into AL
	and	al, CF		; and preserve CF
	mov	dx, [si+hi]	; DX = Rm(hi)
	mov	[di+hi], dx	; Rd(hi) = DX
	test	dx, dx		; set flags based on Rd(hi)
	lahf			; copy them to ah (CF is clear here)
	or	al, ah		; and accumulate
	mov	dx, [si]	; DX = Rm(lo)
	mov	[di], dx	; Rd(lo) = DX
	test	dx, dx		; set flags based on Rd(lo)
	lahf			; copy them to ah
	or	ah, ~ZF		; isolate the zero flag
	or	al, ah		; set ZF if Rd(lo) == Rd(hi) == 0
	mov	[flags], al	; update CF, ZF, and SF
	ret

	; 000000AAAABBBCCC LSLS Rd, Rm, #imm5 where 0 < imm5 < 16
	; V must be preserved and CNZ set (whew)
h000000:mov	bx, [si]	; DX = Rm(lo)
	mov	dx, bx		; keep a copy
	mov	si, [si+hi]	; SI = Rm(hi)
	shl	bx, cl		; BX = Rm(lo) << imm5
	mov	[di], bx	; Rd(lo) = Rm(lo) << imm5
	lahf			; load ZF(lo) based on Rd(lo)
	mov	al, ah		; into AL
	or	al, ~ZF		; isolate ZF
	shl	si, cl		; SI = Rm(hi) << imm5
	lahf			; remember CF, SF, and ZF(hi)
	and	al, ah		; and set ZF = Zf(lo) & ZF(hi)
	mov	[flags], al	; deposit ZF, CF, and SF into flags
	sub	cl, 16
	neg	cl		; CL = 16 - imm5
	shr	dx, cl		; DX = Rm(lo) >> 16 - imm5
	or	dx, si		; DX = Rm(hi) << imm5 | Rm(lo) >> 16 - imm5
				;    = Rm << imm5(hi)
	mov	[di+hi], dx	; deposit into Rd(hi)
	ret

	; 000001AAAABBBCCC LSLS Rd, Rm, #imm5 where imm5 > 16
h000001:sub	cl, 16		; CL = imm5 - 16
	mov	dx, [si]	; BX = Rm(lo)
	test	dx, dx		; make sure flags are set even if CL=0
	shl	dx, cl		; DX = Rm(lo) << imm5 - 16
	mov	word [di], 0	; Rd(lo) = 0
	mov	[di+hi], dx	; Rd(hi) = Rm(lo) << imm5 - 16
	lahf			; load CF, SF, and ZF into AH
	mov	[flags], ah	; update flags except for OF
	ret

	; 0000100000BBBCCC LSRS Rd, Rm, #32
h00001z:mov	ax, 0x80
	and	al, [si+hi+1]	; AL = Rd < 0 ? 0x80 : 0
	shl	al, 1		; AX = 0, flags = Rd < 0 ? {ZF, CF} : {ZF}
	mov	[di], ax	; Rd = 0
	mov	[di+hi], ax
	lahf			; update CF, SF, and ZF in flags
	mov	[flags], ah
	ret

	; 000010AAAABBBCCC LSRS Rd, Rm #imm5 where imm5 < 16
h000010:

	; 0001000000BBBCCC ASRS Rd, Rm, #32
h00010z:mov	ah, [si+hi+1]	; AH = Rd(hi) (high byte)
	cwd			; DX = Rd < 0 ? -1 : 0
	sar	dx, 1		; set flags depending on DX
	mov	[di], dx	; store result to Rd
	mov	[di+hi], dx
	lahf			; update CF, SF, and ZF in flags
	mov	[flags], ah
	ret

h000011:
h000100:
h000101:
h00011z:
h000110:
h000111:int3			; TODO

	; 10100BBBCCCCCCCC ADD Rd, PC, #imm8 (ADR Rd, label)
	; 10101BBBCCCCCCCC ADD Rd, SP, #imm8
h1010:	test	ah, 0x8		; is this ADD Rd, SP, #imm8?
	jnz	.sp		; if not, this is ADD Rd, PC, #imm8

	call	pclin		; set up R15 to the right program counter
	strlo	cx, 15		; load PC into DX:CX
	strhi	dx, 15
	jmp	.fi

.sp:	strlo	cx, 13		; load SP into DX:CX
	strhi	dx, 13

.fi:	mov	ah, 0		; AX = #imm8
	shl	ax, 1
	shl	ax, 1		; AX = #imm8
	add	ax, cx		; DX:AX == DX:CX + AX
	adc	dx, 0
	mov	di, [oprB]	; di = &Rd
	mov	[di], ax	; Rd = DX:AX
	mov	[di+hi], dx
	ret

	; instruction handlers that have not been implemented yet
h001:
h0100:
h0101:
h011:
h1000:
h1001:
h1011:
h1100:
h1101:
h1110:
h1111:	int3

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Memory Access                                                              ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Address Space Conversion                                                   ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

	section	.text

	; convert DX:AX into a linear address in DX:AX
	; trashes CX
seglin:	mov	cl, 4
	rol	dx, cl		; dx = ds >> 12 | ds << 4
	mov	cx, dx
	and	dx, 0xf		; dx = ds >> 12
	and	cl, 0xf0	; cx = ds <<  4
	add	ax, cx		; ax = ax + (ds >> 12)
	adc	dx, 0		; apply carry
	ret

	; convert linear address in DX:AX into a segmented address in DX:AX
	; the offset is normalised to 0x0000--0x00ff
	; ignores the high 12 bit of DX, trashes CL
linseg:	and	dx, 0xf
	mov	dh, ah
	mov	cl, 4
	ror	dx, cl
	mov	ds, dx		; ds = dx << 12 | ax >> 4 & 0x0ff0
	xor	ah, ah		; ax = ax & 0x00ff
	ret

	; determine the segmented address of the current instruction from PC
	; and load it into pcaddr.  Trashes CX.  Assumes DS=CS.
	; If PC cannot be represented as an address, an exception is caused.
pcseg:	strlo	ax, 15		; load PC into DX:AX
	strhi	dx, 15
	mov	ch, dh		; keep a copy of the top 4 bit of PC
	call	linseg		; set up linear address in DX:AX
	and	ch, 0xf0	; isolate address space nibble
	jnz	.not0		; address space 0 (adjusted)?
	add	dx, [imgbase]	; apply address space adjustment
	jmp	.wb
.not0:	cmp	ch, 2		; address space 2 (unadjusted)?
	jne	.wild		; if not, this address cannot be translated
.wb:	mov	[pcaddr], ax	; set up translated PC with DX:AX
	mov	[pcaddr+2], dx
	ret

.wild:	int3			; TODO: generate an exception or something
	jmp	.wild		; endless loop

	; determine the linear address of the current instruction from pcaddr
	; and load it into PC.  It is assumed that PC points into the right
	; address space already.  Trashes AX, BX, CX, and DX.  Assumes DS=CS.
	; Note that as this function is called after pcaddr has been incremented
	; to point right past the current instruction, there is a certain
	; asymmetry to pcseg which assumes that R15 points directly to the
	; instruction to execute.  As usual on ARM, R15 is updated to point
	; 4 bytes ahead of the current instruction, ie. 2 bytes ahead of pcaddr.
pclin:	mov	ax, [pcaddr]	; DX:AX = pcaddr
	mov	dx, [pcaddr+2]
	strhi	bh, 1+15	; load R15 high byte into BH
	and	bx, 0xf000	; isolate address space nibble
	jnz	.not0		; address space 0 (adjusted)?
	sub	dx, [imgbase]	; remove address space adjustment
	jmp	.wb
.not0:	cmp	bh, 2		; address space 2 (unadjusted)?
	jne	.wild		; if not, this address cannot be translated
.wb:	call	seglin		; convert into a linear address
	add	ax, 2		; advance to current insn + 4
	adc	dx, bx		; carry and apply address space nibble
	ldrlo	15, ax		; write DX:AX to R15
	ldrhi	15, dx
	ret

.wild:	int3			; TODO: generate an exception or something
	jmp	.wild		; endless loop

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; IO Routines                                                                ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

	section	.text

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
crlf	db	13,10,0

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Colophon                                                                   ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

	section	.bss
	alignb	16
end	equ	$		; end of program (on paragraph boundary)

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
zsreg	resw	1		; pointer to reglo according to which the zero
				; and sign flags shall be set or 0 if they are
				; already set up correctly.  This is never R15.

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
	push	ds		; remember DS to free a segment selector
	lds	si, [bx]	; load the program counter into DS:SI
	lodsw			; load an instruction
	pop	ds		; restore DS=CS
	mov	[bx], si	; update pcaddr with new offset
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

	; jump table for instructions 000XXX
	; we decode the MSB of the immediate because shifting by 16 or more
	; places requires different code than shifting by less
ht000XXX:
	dw	h000000		; LSL immediate (< 16)
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
	; This table is interleaved with the jump table for
	; add/subtract/compare/move immediate to save a shift.
ht000XXz:
	dw	h00000z		; MOVS Rd, Rm
ht001XX:
	dw	h00100		; MOVS Rd, #imm8
	dw	h00001z		; LSRS Rd, Rm, #32
	dw	h00101		; CMP  Rd, #imm8
	dw	h00010z		; ASRS Rd, Rm, #32
	dw	h00110		; ADDS Rd, #imm8
	dw	h000110		; ADDS Rd, Rm, R0
	dw	h00111		; SUBS Rd, #imm8

	; jump table for the miscellaneous instructions 1011XXXX
	; instructions in parentheses are not available on Cortex-M0
	; cores and generate an undefined instruction exception.
ht1011XXXX:
	dw	h10110000	; ADD/SUB SP, SP, #imm7
	dw	h10110001	; (CBZ Rn, #imm5)
	dw	h10110010	; SXTB/SXTH/UXTB/UXTH
	dw	h10110011	; (CBZ Rn, #imm5)
	dw	h10110100	; PUSH {...}
	dw	h10110101	; PUSH {..., LR}
	dw	h10110110	; CPS
	dw	h10110111	; escape hatch
	dw	h10111000	; undefined
	dw	h10111001	; (CBNZ Rn, #imm5)
	dw	h10111010	; REV/REV16/REVSH
	dw	h10111011	; (CBNZ Rn, #imm5)
	dw	h10111100	; POP {...}
	dw	h10111101	; POP {..., LR}
	dw	h10111110	; BKPT #imm8
	dw	h10111111	; (IT), hints

	; jump table for (un)signed byte/halfword extend
htB2	dw	hB200		; SXTH Rd, Rm
	dw	hB201		; SXTB Rd, Rm
	dw	hB210		; UXTH Rd, Rm
	dw	hB211		; UXTB Rd, Rm

	; jump table for reverse bytes
htBA	dw	hBA00		; REV Rd, Rm
	dw	hBA01		; undefined
	dw	hBA10		; REV16 Rd, Rm
	dw	hBA11		; REVSH Rd, Rm

	; jump table for escape hatch instructions
htB7	dw	hB700		; terminate emulation
	dw	hB701		; dump registers
B7max	equ	($-htB7-2)/2	; highest escape hatch number used

	section	.text

	; 000XXAAAAABBBCCC shift immediate
	; 00011XYAAABBBCCC add/subtract register/immediate
h000:	mov	bl, ah		; BL = 000XXAAA
	shr	bl, 1		; BL = 0000XXAA
	and	bx, 0xe		; BL = 0000XXA0
	mov	si, [oprB]	; SI = &reglo[Rm]
	mov	di, [oprC]	; DI = &reglo[Rd]
	mov	[zsreg], di	; set SF and ZF according to Rd
	mov	cx, [oprA]	; CL = imm5 (or Rm for 000110...)
	test	cl, cl		; is imm8 == 0?
	jz	.zero		; if yes, perform special handling
	jmp	[ht000XXX+bx]	; call instruction specific handler

.zero:	jmp	[ht000XXz+bx]	; call handler for imm8 = 0

	; 0000000000BBBCCC MOVS Rd, Rm
	; CV is preserved, NZ are set according to Rm
h00000z:lodsw			; Rd(lo) = Rm(lo)
	stosw
	mov	ax, [si+hi-2]	; AX = Rm(hi)
	mov	[di+hi-2], ax	; Rm(hi) = AX
	ret

	; 000000AAAABBBCCC LSLS Rd, Rm, #imm5 where 0 < imm5 < 16
	; V must be preserved and CNZ set (whew)
h000000:lodsw			; AX = Rm(lo)
	mov	dx, ax		; keep a copy
	shl	ax, cl		; AX = Rm(lo) << #imm5
	stosw			; Rd(lo) = Rm(lo) << #imm5
	mov	si, [si+hi-2]	; SI = Rm(hi)
	shl	si, cl		; SI = Rm(hi) << #imm5
	lahf			; update CF in flags
	mov	[flags], ah
	sub	cl, 16		; CL = 16 - CL
	neg	cl
	shr	dx, cl		; DX = Rm(lo) >> 16 - #imm5
	or	si, dx		; SI = Rm(hi) << #imm5 | Rm(lo) >> 16 - #imm5
	mov	[di+hi-2], si	; Rd(hi) = Rm << #imm5 (hi)
	ret

	; 000001AAAABBBCCC LSLS Rd, Rm, #imm5 where imm5 > 15
h000001:sub	cl, 16		; CL = imm5 - 16
	mov	al, [si+hi]	; AX = Rm(hi)
	shr	al, 1		; if #imm5=16, CF must be set to Rm(hi) & 1
	lodsw			; AX = Rm(lo),
	shl	ax, cl		; AX = Rm(lo) << imm5 - 16
	mov	word [di], 0	; Rd(lo) = 0
	mov	[di+hi], ax	; Rd(hi) = Rm(lo) << imm5 - 16
	lahf			; update CF in flags
	mov	[flags], ah
	ret

	; 0000100000BBBCCC LSRS Rd, Rm, #32
h00001z:mov	al, [si+hi+1]	; AL = Rm < 0 ? 0x80 : 0
	rol	al, 1		; move AL sign bit to CF position
	mov	[flags], al	; and deposit CF into flags
	xor	ax, ax
	stosw			; Rd = 0
	mov	[di+hi-2], ax
	ret

	; 000010AAAABBBCCC LSRS Rd, Rm, #imm5 where 0 < imm5 < 16
h000010:mov	dx, [si]	; AX = Rm(lo)
	shr	dx, cl		; AX = Rm(lo) >> #imm5
	lahf			; update CF in flags
	mov	[flags], ah
	mov	ax, [si+hi]	; AX = Rm(hi)
	mov	si, ax		; keep a copy
	shr	si, cl		; SI = Rm(hi) >> #imm5
	mov	[di+hi], si	; Rd(hi) = Rm(hi) >> #imm5
	sub	cl, 16		; CL = 16 - CL
	neg	cl
	shl	ax, cl		; AX = Rm(hi) << 16 - #imm5
	or	ax, dx		; AX = Rm(hi) << 16 - #imm5 | Rm(lo) >> #imm5
	stosw			; Rd(lo) = Rm >> #imm5 (lo)
	ret

	; 000011AAAABBBCCC LSRS Rd, Rm, #imm5 where imm5 > 15
h000011:sub	cl, 16		; CL = imm5 - 16
	lodsw			; AL = Rm(lo)
	shl	ax, 1		; if #imm5=16, CF must be set to Rm(lo)&0x8000
	mov	ax, [si+hi-2]	; AX = Rm(hi)
	shr	ax, cl		; AX = Rm(hi) >> imm5 - 16
	mov	word [di+hi], 0	; Rd(hi) = 0
	stosw			; Rd(lo) = Rm(hi) >> imm5 - 16
	lahf			; update CF, SF, and ZF in flags
	mov	[flags], ah
	ret

	; 0001000000BBBCCC ASRS Rd, Rm, #32
h00010z:mov	ah, [si+hi+1]	; AH = Rm(hi) (high byte)
	cwd			; DX = Rm < 0 ? -1 : 0
	xchg	ax, dx		; move DX to AX for better encoding
	mov	[flags], al	; set CF depending on DX
	stosw			; store result to Rd
	mov	[di+hi-2], ax
	ret

	; 000100AAAABBBCCC ASRS Rd, Rm, #imm5 where 0 < imm5 < 16
h000100:mov	dx, [si]	; AX = Rm(lo)
	shr	dx, cl		; AX = Rm(lo) >> #imm5
	lahf			; update CF in flags
	mov	[flags], ah
	mov	ax, [si+hi]	; AX = Rm(hi)
	mov	si, ax		; keep a copy
	sar	si, cl		; SI = Rm(hi) >> #imm5
	mov	[di+hi], si	; Rd(hi) = Rm(hi) >> #imm5
	sub	cl, 16		; CL = 16 - CL
	neg	cl
	shl	ax, cl		; AX = Rm(hi) << 16 - #imm5
	or	ax, dx		; AX = Rm(hi) << 16 - #imm5 | Rm(lo) >> #imm5
	stosw			; Rd(lo) = Rm >> #imm5 (lo)
	ret

	; 000101AAAABBBCCC ASRS Rd, Rm, #imm5 where imm5 > 16
h000101:sub	cl, 16		; CL = imm5 - 16
	lodsw			; AX = Rm(lo)
	shl	ax, 1		; if #imm5=16, CF must be set to Rm(lo)&0x8000
	mov	ax, [si+hi-2]	; AX = Rm(hi)
	sar	ax, cl		; AX = Rm(hi) >> imm5 - 16
	cwd			; DX = Rm(hi) < 0 ? -1 : 0
	mov	[di+hi], dx	; Rd(hi) = 0
	stosw			; Rd(lo) = Rm(hi) >> imm5 - 16
	mov	[flags], dl	; update CF in flags
	ret

	; 0001100AAABBBCCC ADDS Rd, Rn, Rm
	; 0001101AAABBBCCC SUBS Rd, Rn, Rm
h000110:mov	bx, cx		; need BX to form an address
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
	pop	word [flags]
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
.subs:	mov	dx, [si]	; AX:DX = Rn
	mov	ax, [si+hi]
	sub	dx, cx		; AX:DX = Rn - #imm3
	sbb	ax, 0
	cmc			; adjust CF to ARM conventions
.fi:	mov	[di], dx	; Rd = AX:DX
	mov	[di+hi], ax
	pushf			; remember all flags
	pop	word [flags]
	ret

	; 001XXAAABBBBBBBB add/subtract/compare/move immediate
h001:	mov	bl, ah		; BL = 001XXAAA
	shr	bl, 1		; BL = 0001XXAA
	and	bx, 0xc		; BL = 0000XX00
	xor	si, si		; SI = 0
	mov	di, [oprA]	; DI = &reglo[Rd]
	mov	ah, 0		; AX = #imm8
	mov	[zsreg], di	; set SF and ZF according to Rd
	jmp	[ht001XX+bx]	; call instruction specific handler

	; 00100AAABBBBBBBB MOVS Rd, #imm8
h00100:	stosw			; Rd = #imm8
	mov	[di+hi-2], si
	ret

	; 00101AAABBBBBBBB CMP Rn, #imm8
h00101:	mov	dx, [di+hi]
	cmp	[di], ax	; Rd(lo) - #imm8
	sbb	dx, si		; Rd - #imm8
	cmc			; adjust CF to ARM conventions
	pushf			; and remember flags
	pop	word [flags]
	ret

	; 00110AAABBBBBBBB ADDS Rd, #imm8
h00110:	add	[di], ax	; Rd += AX
	add	[di+hi], si
	pushf			; remember flags
	pop	word [flags]
	ret

	; 00111AAABBBBBBBB SUBS Rd, #imm8
h00111:	sub	[di], ax	; Rd -= AX
	sbb	[di+hi], si
	cmc			; adjust CF to ARM conventions
	pushf			; and remember flags
	pop	word [flags]
	ret

	; 10100BBBCCCCCCCC ADD Rd, PC, #imm8 (ADR Rd, label)
	; 10101BBBCCCCCCCC ADD Rd, SP, #imm8
h1010:	mov	di, [oprB]	; di = &Rd
	test	ah, 0x8		; is this ADD Rd, SP, #imm8?
	jnz	.sp		; if not, this is ADD Rd, PC, #imm8
	call	fixRd		; fix up flags to Rd if needed
	call	pclin		; DX:AX = R15
	jmp	.fi
.sp:	call	fixRd		; fix up flags to Rd if needed
	strlo	ax, 13		; load SP into DX:AX
	strhi	dx, 13
.fi:	mov	cx, [oprC]	; CX = #imm8 >> 2
	shl	cx, 1
	shl	cx, 1		; CX = #imm8
	add	ax, cx		; DX:AX == DX:AX + CX (= PC/SP + #imm8)
	adc	dx, 0
	stosw			; Rd = DX:AX
	mov	[di+hi-2], dx
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
	mov	di, reglo+2*13	; DI = &SP(lo) (shorter code)
	xor	ah, ah		; AX = 00000000XAAAAAAA
	shl	al, 1		; AX = 00000000AAAAAAA0, CF = ADD/SUB
	jc	.sub
	shl	ax, 1		; AX = #imm7 (in words)
	add	[di], ax	; R13 += #imm7
	adc	word [di+hi], 0
	ret
.sub:	shl	ax, 1		; AX = #imm7 (in words)
	sub	[di], ax	; R13 += #imm7
	sbb	word [di+hi], 0
	ret

h10110100:
h10110101:
h10110110:
h10111100:
h10111101:
h10111110:	todo

	; 101100A1AAAAABBB (CBZ Rd, #imm5)
h10110001 equ	undefined
h10110011 equ	undefined

	; 10110010XXAAABBB (un)signed extend byte/word
h10110010:
	mov	dx, 0xe		; mask for extracting the instruction fields
	mov	si, reglo
	mov	di, ax
	shl	di, 1		; form an offset into the register table
	and	di, dx		; mask out operand C
	add	di, si		; oprC = &reglo[C]

	mov	bx, ax
	shr	ax, 1
	shr	ax, 1		; form an offset into the register table
	and	ax, dx		; mask out operand B
	add	si, ax		; oprB = &reglo[B]

	mov	cl, 5
	shr	bx, cl		; BX = 0000010110010XXA
	and	bx, dx		; BX = 0000000000000XX0
	call	fixRd		; set flags on Rd if needed
	lodsw			; AX = Rm(lo), DI += 2
	jmp	[htB2+bx]	; jump to instruction handler

	; 1011001000AAABBB SXTH Rd, Rm
hB200:	cwd			; DX:AX = SXTH(Rm(lo))
	stosw			; Rd = DX:AX
	mov	[di+hi-2], dx
	ret

	; 1011001001AAABBB SXTB Rd, Rm
hB201:	cbw			; DX:AX = SXTB(Rm(lo))
	cwd
	stosw			; Rd = DX:AX
	mov	[di+hi-2], dx
	ret

hB210:	; 1011001010AAABBB UXTH Rd, Rm
	stosw			; Rd = UXTH(Rm(lo))
	mov	word [di+hi-2], 0
	ret

	; 1011001011AAABBB UXTB Rd, Rm
hB211:	stosb			; Rd = UXTB(Rm(lo))
	xor	ax, ax
	stosb
	mov	[di+hi-2], ax
	ret

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
h10111011 equ	undefined

	; 10111010XXAAABBB reverse bytes
h10111010:
	mov	dx, 0xe		; mask for extracting the instruction fields
	mov	si, reglo
	mov	di, ax
	shl	di, 1		; form an offset into the register table
	and	di, dx		; mask out operand C
	add	di, si		; oprC = &reglo[C]

	mov	bx, ax
	shr	ax, 1
	shr	ax, 1		; form an offset into the register table
	and	ax, dx		; mask out operand B
	add	si, ax		; oprB = &reglo[B]

	mov	cl, 5
	shr	bx, cl		; BX = 0000010111010XXA
	and	bx, dx		; BX = 0000000000000XX0
	call	fixRd		; set flags on Rd if needed
	lodsw			; AX = Rm(lo), DI += 2
	jmp	[htBA+bx]	; jump to instruction handler

	; 1011101000AAABBB REV Rd, Rm
hBA00:	xchg	ah, al		; reverse Rm(lo)
	mov	dx, [si+hi-2]	; DX = Rm(hi)
	xchg	dh, dl		; reverse Rm(hi)
	mov	[di], dx	; Rd = REV(Rm)
	mov	[di+hi], ax
	ret

	; 1011101001XXXXXX undefined
hBA01	equ	undefined

	; 1011101010AAABBB REV16 Rd, Rm
hBA10:	xchg	ah, al		; reverse Rm(lo)
	stosw			; Rd(lo) = REV(Rm(lo))
	mov	ax, [si+hi-2]	; AX = Rm(hi)
	xchg	ah, al		; reverse Rm(hi)
	mov	[di+hi-2], ax	; Rd(hi) = REV(Rm(hi))
	ret

	; 1011101011AAABBB REVSH Rd, Rm
hBA11:	xchg	ah, al		; reverse Rm(lo)
	cwd			; sign extend into DX:AX
	stosw			; Rd = DX:AX
	mov	[di+hi-2], dx
	ret

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

	; 1101AAAABBBBBBBB B<c> <label>
	; 11011110BBBBBBBB UDF #imm8
	; 11011111BBBBBBBB SVC #imm8
h1101:	mov	bl, ah		; BL = 1101AAAA
	xchg	bx, ax		; BX = insn, AL = 1101AAAA
	shl	al, 1		; AL = 101AAAA0
	shl	al, 1		; AL = 01AAAA00
	cbw			; AX = 01AAAA00
	add	ax, h1101xxxx-0x40
	xchg	di, ax		; DI = h1101xxxx[AAAA]
	call	fixflags	; set up true flags in flags
	push	word [flags]
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

.taken:	mov	si, bx		; remember insn as pclin trashes BX
	call	pclin		; DX:AX = R15
	xchg	ax, si		; AL = #imm8, DX:SI = R15
	mov	cx, dx		; CX:SI = R15
	cbw			; AX = #imm8
	cwd			; DX:AX = #imm8
	shl	ax, 1		; AX = #imm8:0
	add	ax, si		; DX:AX = R15 + #imm8
	adc	dx, cx
	ldrlo	15, ax		; R15 = DX:AX
	ldrhi	15, dx
	jmp	pcseg		; set up pcaddr according to R15

.svc:	todo			; todo

	; instruction handlers that have not been implemented yet
h0100:
h0101:
h011:
h1000:
h1001:
h1100:
h1110:
h1111:	todo

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Flag Manipulation                                                          ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

	; set ZF and SF in flags according to zsreg
	; trashes AX, DX, and SI
fixflags:
	mov	si, [zsreg]
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
	mov	al, [flags]
	and	ax, (ZF|SF)<<8|~(ZF|SF)
				; mask AL to all but ZF and SF,
				; AH to just ZF and SF
	or	al, ah		; merge the two
	mov	[flags], al	; write them back
	mov	[zsreg], dx	; and mark the flags as being fixed
.nofix:	ret

	; compare DI with [zsreg].  If both are equal, fix the flags.
	; trashes AX, DX, and SI.  Preserves DI which may not be zero.
	; the intent is to save the flags if Rd == [zsreg] and flag
	; recovery would otherwise be impossible.
fixRd:	mov	si, [zsreg]
	cmp	si, di		; is Rd == [zsreg]?
	je	fixflags.entry	; if yes, fix it up
	ret			; otherwise, go back to caller

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
	ror	dx, cl		; dx = dx << 12 | ax >> 4 & 0x0ff0
	xor	ah, ah		; ax = ax & 0x00ff
	ret

	; determine the segmented address of the current instruction from PC
	; and load it into pcaddr.  Trashes CX.  Assumes DS=CS.
	; If PC cannot be represented as an address, an exception is caused.
pcseg:	strlo	ax, 15		; load PC into DX:AX
	strhi	dx, 15
	mov	ch, dh		; keep a copy of the top 4 bit of PC
	and	al, ~1		; clear thumb bit if needed
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

.wild:	todo			; TODO: generate an exception or something
	jmp	.wild		; endless loop

	; determine the linear address of the current instruction from pcaddr
	; and load it into PC.  It is assumed that PC points into the right
	; address space already.  Trashes AX, BX, CX, and DX.  Returns the value
	; of R15 in DX:AX.  Assumes DS=CS.  Note that as this function is called
	; after pcaddr has been incremented to point right past the current
	; instruction, there is a certain asymmetry to pcseg which assumes that
	; R15 points directly to the instruction to execute.  As usual on ARM,
	; R15 is updated to point 4 bytes ahead of the current instruction, ie.
	; 2 bytes ahead of pcaddr.
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

.wild:	todo			; TODO: generate an exception or something
	jmp	.wild		; endless loop

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Exceptions, Events, and Interrupts                                         ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

	; generate an undefined instruction exception
undefined:
	todo			; TODO

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Escape Hatches                                                             ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

	; b700 terminate emulation
hB700:	strlo	al, 0		; AL = R0(lo) (error level)
	mov	ah, 0x4c
	int	0x21		; 0x4c: TERMINATE PROGRAM

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


	; b701 dump registers
	section	.text
hB701:	call	fixflags	; set up flags
	call	pclin		; set up R15
	mov	di, dump.r0	; load R0 value field
	mov	si, reglo	; for shorter instruction encodings
.regs:	mov	ax, [si+hi]	; AX = reg(hi)
	call	tohex		; convert high half to hex
	lodsw			; AX = reg(lo)
	call	tohex		; convert low half to hex
	add	di, dump.field - 8 ; advance to next field
	cmp	di, dump.endr	; finished dumping registers?
	jb	.regs
	mov	di, dump.nzcv	; advance SI to NZCV field
	mov	ax, '--'	; clear all flags in the template
	stosw
	stosw
	push	word [flags]	; set up flags according to emulator state
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

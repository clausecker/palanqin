@
@    Mecrisp-Stellaris - A native code Forth implementation for ARM-Cortex M microcontrollers
@    Copyright (C) 2013  Matthias Koch
@
@    This program is free software: you can redistribute it and/or modify
@    it under the terms of the GNU General Public License as published by
@    the Free Software Foundation, either version 3 of the License, or
@    (at your option) any later version.
@
@    This program is distributed in the hope that it will be useful,
@    but WITHOUT ANY WARRANTY; without even the implied warranty of
@    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
@    GNU General Public License for more details.
@
@    You should have received a copy of the GNU General Public License
@    along with this program.  If not, see <http://www.gnu.org/licenses/>.
@

@ Double number support

@------------------------------------------------------------------------------
@ --- Double stack jugglers ---
@------------------------------------------------------------------------------

@ -----------------------------------------------------------------------------
  Wortbirne Flag_foldable_2|Flag_allocator, "2dup" @ ( 2 1 -- 2 1 2 1 )
@ -----------------------------------------------------------------------------
  ldr r0, [psp]
  pushdatos
  subs psp, #4
  str r0, [psp]
  bx lr
    push {lr}
    bl over_allocator
    bl over_allocator
    pop {pc}

@ -----------------------------------------------------------------------------
  Wortbirne Flag_foldable_2|Flag_inline|Flag_allocator, "2drop" @ ( 2 1 -- )
ddrop_vektor:
@ -----------------------------------------------------------------------------
  adds psp, #4
  drop
  bx lr
    push {lr}
    bl drop_allocator
    bl drop_allocator
    pop {pc}

@ -----------------------------------------------------------------------------
  Wortbirne Flag_foldable_4|Flag_allocator, "2swap" @ ( 4 3 2 1 -- 2 1 4 3 )
dswap:
@ -----------------------------------------------------------------------------
  ldm psp!, {r0, r1, r2}
  subs psp, #4
  str r0, [psp]
  pushdatos
  subs psp, #4
  str r2, [psp]
  movs tos, r1
  bx lr

dswap_allocator:
    push {lr} @ Spezialeinsprung des Registerallokators:

    bl expect_four_elements

    ldr r2, [r0, #offset_state_tos]
    ldr r3, [r0, #offset_state_3os]
    str r3, [r0, #offset_state_tos]
    str r2, [r0, #offset_state_3os]

    ldr r2, [r0, #offset_constant_tos]
    ldr r3, [r0, #offset_constant_3os]
    str r3, [r0, #offset_constant_tos]
    str r2, [r0, #offset_constant_3os]

    ldr r2, [r0, #offset_state_nos]
    ldr r3, [r0, #offset_state_4os]
    str r3, [r0, #offset_state_nos]
    str r2, [r0, #offset_state_4os]

    ldr r2, [r0, #offset_constant_nos]
    ldr r3, [r0, #offset_constant_4os]
    str r3, [r0, #offset_constant_nos]
    str r2, [r0, #offset_constant_4os]

    pop {pc}

@ -----------------------------------------------------------------------------
  Wortbirne Flag_foldable_2|Flag_inline|Flag_allocator, "2nip" @ ( 4 3 2 1 -- 2 1 )
dnip:
@ -----------------------------------------------------------------------------
  ldm psp!, {r0, r1, r2}
  subs psp, #4
  str r0, [psp]
  bx lr

    push {lr}
    bl expect_four_elements
    bl dswap_allocator
    bl eliminiere_tos
    bl eliminiere_tos
    pop {pc}

@ -----------------------------------------------------------------------------
  Wortbirne Flag_foldable_4, "2over" @ ( 4 3 2 1 -- 4 3 2 1 4 3 )
@ -----------------------------------------------------------------------------
  ldr r0, [psp, #8]
  pushdatos
  subs psp, #4
  str r0, [psp]
  ldr tos, [psp, #12]
  bx lr

@ -----------------------------------------------------------------------------
  Wortbirne Flag_foldable_4, "2tuck" @ ( 4 3 2 1 -- 2 1 4 3 2 1 )
@ -----------------------------------------------------------------------------
  ldm psp!, {r0, r1, r2} @ w=2 x=3 y=4
  subs psp, #4
  str r0, [psp]
  pushdatos
  subs psp, #4
  str r2, [psp]
  subs psp, #4
  str r1, [psp]
  subs psp, #4
  str r0, [psp]
  bx lr

@ -----------------------------------------------------------------------------
  Wortbirne Flag_foldable_6, "2rot" @ ( 6  5 4 3 2 1  -- 4  3 2 1 6 5 ) ( x w y -- w y x )
                                    @  16 12 8 4 0 tos  16 12 8 4 0 tos
@ -----------------------------------------------------------------------------
  ldr r0, [psp]
  ldr r1, [psp, #8]
  ldr r2, [psp, #16]

  str r0, [psp, #8]
  str r1, [psp, #16]
  str r2, [psp]

  ldr r1, [psp, #4]
  str tos, [psp, #4]
  ldr tos, [psp, #12]
  str r1, [psp, #12]

  bx lr

@ -----------------------------------------------------------------------------
  Wortbirne Flag_foldable_6, "2-rot" @ ( 6  5 4 3 2 1 --  2  1 6 5 4 3 ( x w y -- y x w )
                                     @  16 12 8 4 0 tos  16 12 8 4 0 tos
@ -----------------------------------------------------------------------------
  ldr r0, [psp]
  ldr r1, [psp, #8]
  ldr r2, [psp, #16]

  str r0, [psp, #16]
  str r1, [psp]
  str r2, [psp, #8]

  ldr r1, [psp, #12]
  str tos, [psp, #12]
  ldr tos, [psp, #4]
  str r1, [psp, #4]

  bx lr


@------------------------------------------------------------------------------
@ --- Double return stack jugglers ---
@------------------------------------------------------------------------------

@  : p 3 4 .s 2>r .s 2r@ .s . . 2r> .s 2drop .s ;
@  : 2>r swap >r >r inline ;
@  : 2r> r> r> swap inline ;

@------------------------------------------------------------------------------
  Wortbirne Flag_inline|Flag_allocator, "2>r" @ Puts the two top elements of stack on returnstack.
                               @ Equal to swap >r >r
@------------------------------------------------------------------------------
  ldm psp!, {r0}
  push {r0}
  push {tos}
  ldm psp!, {tos}
  bx lr
    push {lr}
    bl push_lr_nachholen
    bl swap_allocator
    bl allocator_to_r
    bl allocator_to_r
    pop {pc}

@------------------------------------------------------------------------------
  Wortbirne Flag_inline|Flag_allocator, "2r>" @ Fetches back two elements of returnstack.
                               @ Equal to r> r> swap
@------------------------------------------------------------------------------
  pushdatos
  pop {tos}
  pop {r0}
  subs psp, #4
  str r0, [psp]
  bx lr
    push {lr}
    bl push_lr_nachholen
    bl allocator_r_from
    bl allocator_r_from
    bl swap_allocator
    pop {pc}

@------------------------------------------------------------------------------
  Wortbirne Flag_inline|Flag_allocator, "2r@" @ Copies the two top elements of returnsteack
@------------------------------------------------------------------------------
  pushdatos
  ldr tos, [sp, #4]
  pushdatos
  ldr tos, [sp]
  bx lr
    push {lr}
    bl push_lr_nachholen
    pushdaconstw 0x9801
    bl loop_j_allocator
    bl rfetch_allocator
    pop {pc}

@------------------------------------------------------------------------------
  Wortbirne Flag_inline|Flag_allocator, "2rdrop" @ Entfernt die obersten beiden Element des Returnstacks
@------------------------------------------------------------------------------
  add sp, #8
  bx lr
    push {lr}
    bl push_lr_nachholen
    pushdaconstw 0xB002  @ Opcode add sp, #8
    bl hkomma
    pop {pc}

  .ltorg

@------------------------------------------------------------------------------
@ --- Double calculations ---
@------------------------------------------------------------------------------

@------------------------------------------------------------------------------
  Wortbirne Flag_visible|Flag_foldable_2, "d2/"
@------------------------------------------------------------------------------
  ldr r0, [psp]
  lsls r1, tos, #31 @ Prepare Carry
  asrs tos, #1     @ Shift signed high part right
  lsrs r0, #1       @ Shift low part
  orrs r0, r1
  str r0, [psp]
  bx lr

@------------------------------------------------------------------------------
  Wortbirne Flag_inline|Flag_foldable_2|Flag_allocator, "d2*"
@------------------------------------------------------------------------------
  ldr r0, [psp]
  adds r0, r0
  adcs tos, tos
  str r0, [psp]
  bx lr
    b.n 1f

@------------------------------------------------------------------------------
  Wortbirne Flag_visible|Flag_foldable_2, "dshr"
@------------------------------------------------------------------------------
  ldr r0, [psp]
  lsls r1, tos, #31 @ Prepare Carry
  lsrs tos, #1     @ Shift unsigned high part right
  lsrs r0, #1       @ Shift low part
  orrs r0, r1
  str r0, [psp]
  bx lr

@------------------------------------------------------------------------------
  Wortbirne Flag_inline|Flag_foldable_2|Flag_allocator, "dshl"
@------------------------------------------------------------------------------
  ldr r0, [psp]
  adds r0, r0
  adcs tos, tos
  str r0, [psp]
  bx lr

1:  push {lr}
    bl expect_two_elements

    bl expect_tos_in_register
    bl make_tos_changeable

    bl swap_allocator
    bl expect_tos_in_register
    bl make_tos_changeable

    pushdaconst 0x0040 @ lsls r0, r0, #1
    bl smalltworegisters

    bl swap_allocator
    pushdaconstw 0x4140 @ adcs r0, r0
    bl smalltworegisters
    pop {pc}

@------------------------------------------------------------------------------
  Wortbirne Flag_visible|Flag_foldable_2, "dabs"
@------------------------------------------------------------------------------
dabs:
  cmp tos, #0   @ Check sign in high-part
  bmi.n dnegate @ Not negative ? Nothing to do !
  bx lr

@------------------------------------------------------------------------------
@  Wortbirne Flag_foldable_3, "?dnegate" @ Negate a double number if top element on stack is negative.
@------------------------------------------------------------------------------
@   popda r0
@   cmp r0, #0
@   bmi.n dnegate
@   bx lr

@------------------------------------------------------------------------------
  Wortbirne Flag_foldable_2, "dnegate"
@------------------------------------------------------------------------------
dnegate:
  ldr r0, [psp]
  movs r1, #0
  mvns r0, r0
  mvns tos, tos
  adds r0, #1
  adcs tos, r1
  str r0, [psp]
  bx lr

@------------------------------------------------------------------------------
  Wortbirne Flag_inline|Flag_foldable_4|Flag_allocator, "d-" @ ( 1L 1H 2L 2H )
@------------------------------------------------------------------------------
  ldm psp!, {r0, r1, r2}
  subs r2, r0     @  Low-part first
  sbcs r1, tos   @ High-part with carry
  movs tos, r1

  subs psp, #4
  str r2, [psp]
  bx lr

    push {lr}
    bl expect_four_elements
    bl rot_allocator @ ( 1L 2L 2H 1H )
    bl expect_tos_in_register
    bl make_tos_changeable
    bl swap_allocator @ ( 1L 2L 1H 2H )
    bl expect_tos_in_register
    bl make_tos_changeable
    bl dswap_allocator @ ( 1H 2H 1L 2L )
    bl minus_allocator  @ ( 1H 2H L )
    bl minusrot_allocator @ ( L 1H 2H )
    pushdaconstw 0x4180 @ sbcs r0, r0
    bl alloc_unkommutativ @ ( L H )
    pop {pc}

@------------------------------------------------------------------------------
  Wortbirne Flag_inline|Flag_foldable_4|Flag_allocator, "d+" @ ( 1L 1H 2L 2H )
@------------------------------------------------------------------------------
  ldm psp!, {r0, r1, r2}
  adds r2, r0
  adcs tos, r1
  subs psp, #4
  str r2, [psp]
  bx lr

    push {lr}
    bl expect_four_elements
    bl rot_allocator @ ( 1L 2L 2H 1H )
    bl expect_tos_in_register
    bl make_tos_changeable
    bl swap_allocator @ ( 1L 2L 1H 2H )
    bl expect_tos_in_register
    bl make_tos_changeable
    bl dswap_allocator @ ( 1H 2H 1L 2L )
    bl plus_allocator  @ ( 1H 2H L )
    bl minusrot_allocator @ ( L 1H 2H )
    pushdaconstw 0x4140 @ adcs r0, r0
    bl alloc_kommutativ @ ( L H )
    pop {pc}

@------------------------------------------------------------------------------
  Wortbirne Flag_inline|Flag_foldable_1|Flag_allocator, "s>d" @ ( n - dl dh ) Single --> Double conversion
@------------------------------------------------------------------------------
  pushdatos
  movs tos, tos, asr #31    @ Turn MSB into 0xffffffff or 0x00000000
  bx lr
    push {lr}
    bl dup_allocator
    bl make_tos_changeable
    pushdaconstw 0x17C0 @ asrs r0, r0, #31
    bl smalltworegisters
    pop {pc}

 .ltorg

@------------------------------------------------------------------------------
@ --- Double star and slash ---
@------------------------------------------------------------------------------


  .ifdef m0core

@------------------------------------------------------------------------------
  Wortbirne Flag_visible|Flag_foldable_2, "um*"
  @ Multiply unsigned 32*32 = 64
  @ ( u u -- ud )
um_star:
@------------------------------------------------------------------------------
  ldm psp!, {r0}
  movs r2, tos
  movs tos, #0

  b.n ud_star_late_entry

  .else

@------------------------------------------------------------------------------
  Wortbirne Flag_inline|Flag_foldable_2|Flag_allocator, "um*"
  @ Multiply unsigned 32*32 = 64
  @ ( u u -- ud )
um_star:
@------------------------------------------------------------------------------

    ldr r0, [psp]
    umull r0, tos, r0, tos @ Unsigned long multiply 32*32=64
    str r0, [psp]
    bx lr

    pushdatos
    ldr tos, =0xFBA00000

alloc_multiplikation_m3:

    push {lr}
    bl expect_two_elements
    bl expect_tos_in_register
    bl expect_nos_in_register

    @ Baue den Opcode zusammen:

    ldr r1, [r0, #offset_state_tos]
    orrs tos, r1 @ Quellregister 1 hinzufügen

    ldr r1, [r0, #offset_state_nos]
    orrs tos, tos, r1, lsl #16 @ Quellregister 2 hinzufügen

    @ Zweimal den Register wechseln !
    bl eliminiere_tos
    bl eliminiere_tos

          bl befreie_tos
          bl get_free_register
          str r3, [r0, #offset_state_tos]

    orrs tos, tos, r3, lsl #12 @ Zielregister Low hinzufügen

          bl befreie_tos
          bl get_free_register
          str r3, [r0, #offset_state_tos]

    orrs tos, tos, r3, lsl #8 @ Zielregister High hinzufügen

    bl reversekomma
    pop {pc}

  .endif


  .ifdef m0core

@------------------------------------------------------------------------------
  Wortbirne Flag_visible|Flag_foldable_2, "m*"
  @ Multiply signed 32*32 = 64
  @ ( n n -- d )
m_star:
@------------------------------------------------------------------------------

  movs r0, tos
  movs tos, tos, asr #31 @ Turn MSB into 0xffffffff or 0x00000000

  ldm psp!, {r2}
  movs r1, r2, asr #31 @ Turn MSB into 0xffffffff or 0x00000000

  b.n ud_star_registers

  .else

@------------------------------------------------------------------------------
  Wortbirne Flag_inline|Flag_foldable_2|Flag_allocator, "m*"
  @ Multiply signed 32*32 = 64
  @ ( n n -- d )
m_star:
@------------------------------------------------------------------------------
    ldr r0, [psp]
    smull r0, tos, r0, tos @ Signed long multiply 32*32=64
    str r0, [psp]
    bx lr

    pushdatos
    ldr tos, =0xFB800000
    b.n alloc_multiplikation_m3

  .endif

@------------------------------------------------------------------------------
  Wortbirne Flag_visible|Flag_foldable_4, "ud*"
ud_star:
         @ Unsigned multiply 64*64 = 64
         @ ( ud1 ud2 -- ud )
@------------------------------------------------------------------------------
  @ Multiply r1:r0 and r3:r2 and return the product in r1:r0
  @          tos w      x y

@ r1:r0  r3:r2 -->  r1:r0
@ tos r0 r1 r2 -->  tos r0

        ldm psp!, {r0, r1, r2}

ud_star_registers:

	muls	tos, r2        @ High-1 * Low-2 --> tos
	muls	r1, r0         @ High-2 * Low-1 --> r1
	adds	tos, r1        @                    Sum into tos

ud_star_late_entry:

	lsrs	r1, r0, #16
	lsrs	r3, r2, #16
	muls	r1, r3
	adds	tos, r1

	lsrs	r1, r0, #16
	uxth	r0, r0
	uxth	r2, r2
	muls	r1, r2
	muls	r3, r0
	muls	r0, r2

	movs	r2, #0
	adds	r1, r3
	adcs	r2, r2
	lsls	r2, #16
	adds	tos, r2

	lsls	r2, r1, #16
	lsrs	r1, #16
	adds	r0, r2
	adcs	tos, r1

        subs psp, #4
        str r0, [psp]

        bx lr

@------------------------------------------------------------------------------
  Wortbirne Flag_visible|Flag_foldable_4, "udm*"
udm_star: @ Unsigned multiply 64*64 = 128
          @ ( ud1 ud2 -- udl udh )
@------------------------------------------------------------------------------
  @ Auf dem Datenstack: ( 1L 1H 2L 2H -- LL  L  H HH )
  @                       12  8  4  0 nach pushdatos
  @                        d  c  b  a    r0 r1 r2 r3
  @ Benötige einen langen Ergebnisregister !

  push {r4, lr}
  movs r4, #0 @ For Carry addition

  @ ( d c b a )
  pushdatos
  ldr tos, [psp, #4]    @ b
  pushdatos
  ldr tos, [psp, #12+4] @ d
  bl um_star
  @ ( d c b a  b*d-Low b*d-High )
  popda r1 @ b*d-High
  popda r0 @ b*d-Low, finished value

  @ ( d c b a )

  pushdatos
  ldr tos, [psp, #0]   @ a
  pushdatos
  ldr tos, [psp, #8+4] @ c
  push {r0, r1}
    bl um_star
  pop {r0, r1}
  @ ( d c b a  a*c-Low a*c-High )
  popda r3 @ a*c-High
  popda r2 @ a*c-Low

  @ ( d c b a )

  pushdatos
  ldr tos, [psp, #0]    @ a
  pushdatos
  ldr tos, [psp, #12+4] @ d

  push {r0, r1, r2, r3}
    bl um_star
  pop {r0, r1, r2, r3}
  @ ( d c b a  a*d-Low a*d-High )

  adds r2, tos @ a*c-Low + a*d-High
  adcs r3, r4  @ Carry
  drop

  adds r1, tos @ a*d-Low + b*d-High
  adcs r2, r4  @ Carry
  adcs r3, r4  @ Carry
  drop

  @ ( d c b a )

  pushdatos
  ldr tos, [psp, #4]    @ b
  pushdatos
  ldr tos, [psp, #8+4]  @ c

  push {r0, r1, r2, r3}
    bl um_star
  pop {r0, r1, r2, r3}
  @ ( d c b a  b*c-Low b*c-High )

  adds r2, tos @ a*c-Low + b*c-High + a*d-High
  adcs r3, r4  @ Carry
  drop

  adds r1, tos @ b*c-Low + a*d-Low + b*d-High
  adcs r2, r4  @ Carry
  adcs r3, r4  @ Carry
  drop

  @ ( d c b tos: a )
  movs tos, r3
  str r2, [psp, #0]
  str r1, [psp, #4]
  str r0, [psp, #8]

  pop {r4, pc}


@------------------------------------------------------------------------------
  Wortbirne Flag_visible|Flag_foldable_3, "*/" @ Signed scalar
  @ ( u1 u2 u3 -- u1*u2/u3 ) With double length intermediate result
@------------------------------------------------------------------------------
  push {lr}
  to_r
  bl m_star
  r_from
  bl m_slash_mod
  nip
  pop {pc}

@------------------------------------------------------------------------------
  Wortbirne Flag_visible|Flag_foldable_3, "*/mod" @ Signed scalar
  @ ( u1 u2 u3 -- u1*u2/u3 ) With double length intermediate result
@------------------------------------------------------------------------------
  push {lr}
  to_r
  bl m_star
  r_from
  bl m_slash_mod
  pop {pc}

@ : u*/  ( u1 u2 u3 -- u1 * u2 / u3 )  >r um* r> um/mod nip 3-foldable ;
@------------------------------------------------------------------------------
  Wortbirne Flag_visible|Flag_foldable_3, "u*/" @ Unsigned scalar
  @ ( u1 u2 u3 -- u1*u2/u3 ) With double length intermediate result
@------------------------------------------------------------------------------
  push {lr}
  to_r
  bl um_star
  r_from
  bl um_slash_mod
  nip
  pop {pc}

@------------------------------------------------------------------------------
  Wortbirne Flag_visible|Flag_foldable_3, "u*/mod" @ Unsigned scalar
  @ ( u1 u2 u3 -- u1*u2/u3 ) With double length intermediate result
@------------------------------------------------------------------------------
  push {lr}
  to_r
  bl um_star
  r_from
  bl um_slash_mod
  pop {pc}

@------------------------------------------------------------------------------
  Wortbirne Flag_visible|Flag_foldable_3, "um/mod"
um_slash_mod: @ ( ud u -- u u ) Dividend Divisor -- Rest Ergebnis
             @ 64/32 = 32 Rest 32
@------------------------------------------------------------------------------
  push {lr}
  pushdaconst 0
  bl ud_slash_mod
  drop
  nip
  pop {pc}

@------------------------------------------------------------------------------
  Wortbirne Flag_visible|Flag_foldable_3, "m/mod"
              @ Signed symmetric divide 64/32 = 32 remainder 32
m_slash_mod:  @ ( d n -- n n )
@------------------------------------------------------------------------------
  push {lr}
  pushdatos                 @ s>d
  movs tos, tos, asr #31    @ Turn MSB into 0xffffffff or 0x00000000
  bl d_slash_mod
  drop
  nip
  pop {pc}

@------------------------------------------------------------------------------
@ Tool for ud/mod
@------------------------------------------------------------------------------

  .macro division_step
    @ Shift the long chain of four registers.
    lsls r0, #1
    adcs r1, r1
    adcs r2, r2
    adcs r3, r3

    @ Compare Divisor with top two registers
    cmp r3, r5 @ Check high part first
    bhi 1f
    blo 2f

    cmp r2, r4 @ High part is identical. Low part decides.
    blo 2f

    @ Subtract Divisor from two top registers
1:  subs r2, r4 @ Subtract low part
    sbcs r3, r5 @ Subtract high part with carry

    @ Insert a bit into Result which is inside LSB of the long register.
    adds r0, #1
2:
  .endm

@------------------------------------------------------------------------------
  Wortbirne Flag_visible|Flag_foldable_4, "ud/mod"
         @ Unsigned divide 64/64 = 64 remainder 64
         @ ( ud1 ud2 -- ud ud)
         @ ( 1L 1H 2L tos: 2H -- Rem-L Rem-H Quot-L tos: Quot-H )
@------------------------------------------------------------------------------
ud_slash_mod:
   push {r4, r5}

   @ ( DividendL DividendH DivisorL DivisorH -- RemainderL RemainderH ResultL ResultH )
   @   8         4         0        tos      -- 8          4          0       tos


   @ Shift-High Shift-Low Dividend-High Dividend-Low
   @         r3        r2            r1           r0

   movs r3, #0
   movs r2, #0
   ldr  r1, [psp, #4]
   ldr  r0, [psp, #8]

   @ Divisor-High Divisor-Low
   @          r5           r4

ud_slash_mod_internal:
   movs r5, tos
   ldr  r4, [psp, #0]

   @ For this long division, we need 64 individual division steps.
   movs tos, #64

3: division_step
   subs tos, #1
   bne 3b

   @ Now place all values to their destination.
   movs tos, r1       @ Result-High
   str  r0, [psp, #0] @ Result-Low
   str  r3, [psp, #4] @ Remainder-High
   str  r2, [psp, #8] @ Remainder-Low

   pop {r4, r5}
   bx lr

@------------------------------------------------------------------------------
@  Wortbirne Flag_visible|Flag_foldable_4, "uf/mod" @ Internal helper only.
uf_slash_mod: @ Divide 64/64 = 64 Remainder 64. Puts decimal point in the middle. Overflow possible.
         @ ( ud1 ud2 -- ud ud)
         @ ( 1L 1H 2L tos: 2H -- Rem-L Rem-H Quot-L tos: Quot-H )
@------------------------------------------------------------------------------
   push {r4, r5}

   movs r3, #0
   ldr  r2, [psp, #4]
   ldr  r1, [psp, #8]
   movs r0, #0

   b.n ud_slash_mod_internal

@------------------------------------------------------------------------------
  Wortbirne Flag_visible|Flag_foldable_4, "d/mod"
              @ Signed symmetric divide 64/64 = 64 remainder 64
              @ ( d1 d2 -- d d )
d_slash_mod:  @ ( 1L 1H 2L tos: 2H -- Rem-L Rem-H Quot-L tos: Quot-H )
@------------------------------------------------------------------------------
  @ Check Divisor
  push {lr}
  movs r0, tos, asr #31 @ Turn MSB into 0xffffffff or 0x00000000
  beq 2f
    @ ? / -
    bl dnegate
    bl dswap
    movs r0, tos, asr #31 @ Turn MSB into 0xffffffff or 0x00000000
    beq 1f
    @ - / -
    bl dnegate
    bl dswap
    bl ud_slash_mod

    bl dswap
    bl dnegate @ Negative remainder
    bl dswap
    pop {pc}

1:  @ + / -
    bl dswap
    bl ud_slash_mod
    bl dnegate  @ Negative result
    pop {pc}

2:  @ ? / +
    bl dswap
    movs r0, tos, asr #31 @ Turn MSB into 0xffffffff or 0x00000000
    beq 3f
    @ - / +
    bl dnegate
    bl dswap

    bl ud_slash_mod

    bl dnegate @ Negative result
    bl dswap
    bl dnegate @ Negative remainder
    bl dswap
    pop {pc}

3:  @ + / +
    bl dswap
    bl ud_slash_mod
    pop {pc}

@------------------------------------------------------------------------------
  Wortbirne Flag_visible|Flag_foldable_4, "d/"
@------------------------------------------------------------------------------
  push {lr}
  bl d_slash_mod
  bl dnip
  pop {pc}

@------------------------------------------------------------------------------
@ --- s31.32 calculations ---
@------------------------------------------------------------------------------


@------------------------------------------------------------------------------
  Wortbirne Flag_visible|Flag_foldable_4, "f*"
f_star: @ Signed multiply s31.32
        @ ( fi fi -- fi )
        @ Overflow possible. Sign wrong in this case.
@------------------------------------------------------------------------------
  push {lr}
  movs r0, tos, asr #31 @ Turn MSB into 0xffffffff or 0x00000000
  beq 1f
  @ - * ?
    bl dnegate
    bl dswap
    movs r0, tos, asr #31 @ Turn MSB into 0xffffffff or 0x00000000
    beq 2f @ - * +

    @ - * -
    bl dnegate

3:  @ + * +, - * -
    bl udm_star
    @ ( LL L H HH )
    drop
    ldmia psp!, {r0}
    str r0, [psp]
    @ ( L H )
    pop {pc}

1:@ + * ?
    bl dswap
    movs r0, tos, asr #31 @ Turn MSB into 0xffffffff or 0x00000000
    beq 3b @ + * +

    bl dnegate

    @ - * + or + * -
2:  bl udm_star
    @ ( LL L H HH )
    drop
    ldmia psp!, {r0}
    str r0, [psp]
    @ ( L H )
    bl dnegate
  pop {pc}

@------------------------------------------------------------------------------
  Wortbirne Flag_visible|Flag_foldable_4, "f/"
  @ Signed divide for s31.32. Overflow possible. Sign wrong in this case.
@------------------------------------------------------------------------------
  @ Take care of sign ! ( 1L 1H 2L 2H - EL EH )
  push {lr}
  movs r0, tos, asr #31 @ Turn MSB into 0xffffffff or 0x00000000
  beq 2f
  @ ? / -
    bl dnegate
    bl dswap
    movs r0, tos, asr #31 @ Turn MSB into 0xffffffff or 0x00000000
    beq 3f @ + / -

    @ - / -
    bl dnegate
1:  bl dswap @ - / - or + / +
    bl uf_slash_mod
    bl dnip
    pop {pc}

2:@ ? / +
  bl dswap
  movs r0, tos, asr #31 @ Turn MSB into 0xffffffff or 0x00000000
  beq 1b @ + / +

  @ - / +
  bl dnegate
3:bl dswap @ - / + or + / -
  bl uf_slash_mod
  bl dnegate
  bl dnip
  pop {pc}


@------------------------------------------------------------------------------
@ --- Double memory ---
@------------------------------------------------------------------------------

@------------------------------------------------------------------------------
  Wortbirne Flag_inline|Flag_allocator, "2!" @ Store ( d addr -- )
@------------------------------------------------------------------------------
  ldmia psp!, {r1, r2}
  str r1, [tos]
  str r2, [tos, #4]
  drop
  bx lr

    push {lr}
    bl expect_three_elements

    ldr r1, [r0, #offset_state_tos]
    ldr r2, [r0, #offset_constant_tos]
    push {r1, r2}

    bl allocator_4store @ High-Teil wegstauen.

    @ Das Schreiben sollte keinen der Register mit Ausnahme der Konstantenregister verändern.
    bl befreie_tos
    pop {r1, r2}
    str r1, [r0, #offset_state_tos]

    cmp r1, #constant
    beq 1f
      @ Tos ist im Register - dazu kann ich dann einfach einen anderen Opcode verwenden:
      pushdaconstw 0x6040 @ str r0, [r0, #4] Opcode
      bl allocator_4store_anderer_opcode
      pop {pc}

1:  @ Tos ist eine Konstante ! Wie fein :-)
    adds r2, #4
    str r2, [r0, #offset_constant_tos]
    bl allocator_4store
    pop {pc}

@------------------------------------------------------------------------------
  Wortbirne Flag_inline|Flag_allocator, "2@" @ Fetch ( addr -- d )
@------------------------------------------------------------------------------
  subs psp, #4
  ldr r0, [tos, #4]
  str r0, [psp]
  ldr tos, [tos]
  bx lr

    push {lr}
    bl dup_allocator

    ldr r1, [r0, #offset_state_tos]
    cmp r1, #constant
    beq 1f
      @ TOS ist ein Register.
      @ So wird es Zeit für einen Register, #4 Opcode !
      pushdaconstw 0x6840 @ ldr r0, [r0, #4] Opcode
      bl allocator_4fetch_anderer_opcode

      bl swap_allocator
      bl allocator_4fetch
      pop {pc}

1:  @ TOS ist eine Konstante. Kann sie also wunderbar weiterverwenden ! Ergibt aber einzeln eine lange Aufräum-Sequenz, falls r6 nicht frei ist.
    ldr r1, [r0, #offset_constant_tos]
    adds r1, #4
    str r1, [r0, #offset_constant_tos]
    bl allocator_4fetch

    bl swap_allocator
    bl allocator_4fetch
    pop {pc}

@------------------------------------------------------------------------------
@ --- Double comparisions ---
@------------------------------------------------------------------------------

@------------------------------------------------------------------------------
Wortbirne Flag_visible|Flag_foldable_4, "du<"
  @ ( 2L 2H 1L 1H -- Flag )
  @   r2 r1 r0 tos
@------------------------------------------------------------------------------

  ldm psp!, {r0, r1, r2}
  subs r2, r0    @  Low-part first
  sbcs r1, tos   @ High-part with carry

  sbcs tos, tos  @ Create carry flag on TOS
  bx lr

@------------------------------------------------------------------------------
Wortbirne Flag_visible|Flag_foldable_4, "du>"  @ Just swapped the order of registers
@------------------------------------------------------------------------------

  ldm psp!, {r0, r1, r2}
  subs r0, r2    @  Low-part first
  sbcs tos, r1   @ High-part with carry

  sbcs tos, tos  @ Create carry flag on TOS
  bx lr


@------------------------------------------------------------------------------
Wortbirne Flag_visible|Flag_foldable_4, "d<"
@------------------------------------------------------------------------------

  ldm psp!, {r0, r1, r2}
  subs r2, r0    @  Low-part first
  sbcs r1, tos   @ High-part with carry

  blt 1f
  movs tos, #0
  bx lr

@------------------------------------------------------------------------------
Wortbirne Flag_visible|Flag_foldable_4, "d>"  @ Just swapped the order of registers
@------------------------------------------------------------------------------

  ldm psp!, {r0, r1, r2}
  subs r0, r2    @  Low-part first
  sbcs tos, r1   @ High-part with carry

  blt 1f
  movs tos, #0
  bx lr

1:movs tos, #0   @ True
  mvns tos, tos
  bx lr

@------------------------------------------------------------------------------
  Wortbirne Flag_inline|Flag_foldable_2|Flag_allocator, "d0<" @ ( 1L 1H -- Flag ) Is double number negative ?
@------------------------------------------------------------------------------
  adds psp, #4
  movs TOS, TOS, asr #31    @ Turn MSB into 0xffffffff or 0x00000000
  bx lr

    push {lr}
    bl nip_allocator
    bl alloc_nullkleiner
    pop {pc}

@------------------------------------------------------------------------------
  Wortbirne Flag_foldable_2|Flag_inline|Flag_allocator, "d0=" @ ( 1L 1H -- Flag )
@------------------------------------------------------------------------------
  ldm psp!, {r0}
  orrs tos, r0
  subs tos, #1
  sbcs tos, tos
  bx lr

    push {lr}
    pushdaconstw 0x4300 @ orrs r0, r0      Opcode
    bl alloc_kommutativ
    bl allocator_equal_zero
    pop {pc}

@------------------------------------------------------------------------------
  Wortbirne Flag_foldable_4|Flag_allocator, "d<>" @ ( 1L 1H 2L 2H -- Flag )
@------------------------------------------------------------------------------
  ldm psp!, {r0, r1, r2}

  eors r0, r2
  eors tos, r1
  orrs tos, r0

  subs tos, #1
  sbcs tos, tos
  mvns tos, tos

  bx lr

    push {lr}
    bl dgleichungleich_common
    bl allocator_unequal_zero
    pop {pc}

@------------------------------------------------------------------------------
  Wortbirne Flag_foldable_4|Flag_allocator, "d=" @ ( 1L 1H 2L 2H -- Flag )
@------------------------------------------------------------------------------
  ldm psp!, {r0, r1, r2}

  eors r0, r2
  eors tos, r1
  orrs tos, r0

  subs tos, #1
  sbcs tos, tos

  bx lr

    push {lr}
    bl dgleichungleich_common
    bl allocator_equal_zero
    pop {pc}

dgleichungleich_common:
  push {lr}
  bl expect_four_elements @ ( 1L 1H 2L 2H )
  bl rot_allocator        @ ( 1L 2L 2H 1H )
  bl xor_allocator        @ ( 1L 2L H )
  bl minusrot_allocator   @ ( H 1L 2L )
  bl xor_allocator        @ ( L H )
  bl or_allocator         @ ( ? )
  pop {pc}

  .ltorg

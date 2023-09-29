nf_crlf        macro
    ifne _DEBUG
    move.l d0, -(sp)
    move.l  #nf_stderr_crlf,-(sp)
    move.l  nf_stderr_id,-(sp)
    pea     0
    DC.W    $7301
    lea     12(SP),SP
    move.l  (sp)+, d0
    endif
    endm

nf_str      macro
    ifne _DEBUG
    move.l  d0, -(sp)
    move.l  #\1,-(sp)
    move.l  nf_stderr_id,-(sp)
    pea     0
    DC.W    $7301
    lea     12(SP),SP
    move.l  (sp)+, d0
    endif
    endm

;print to stderr on Host-system
; \1 = string address
; \2 = Length of the extra hex number to print. 0 = no extra hex number
; d7(8,16,32) = extra hex number to print
nf_stderr_lit      macro
    ifne _DEBUG
    tst.l   nf_has_flag
    beq.s   \@.nf_stderr_lit_ignore
	movem.l	d0-d7/a0-a6,-(sp)
    move.l  #\1,-(sp)
    move.l  nf_stderr_id,-(sp)
    pea     0
    DC.W    $7301
    lea     12(SP),SP

    ifne \2

    moveq #\2-1,d4

    ifeq \2-4
    swap d7
    endif

    ifeq \2-2
    ror.l	#8,d7
    endif 

    lea nf_hexnum_buff,a0
\@.nf_stderr_lit.loop
	rol.l	#4,d7
	move.w	d7,d0
	and.w	#$f,d0
	
	cmp.w	#$a,d0
	blt.b	\@.nf_stderr_lit.digit

	add.w	#'A'-$a,d0
	bra.b	\@.nf_stderr_lit.prtd1

\@.nf_stderr_lit.digit
	add.w	#'0',d0

\@.nf_stderr_lit.prtd1
	move.b	d0,(a0)+
	dbra	d4,\@.nf_stderr_lit.loop
    clr.b (a0)

    subq.l  #\2,a0
    move.l  a0,-(sp)
    move.l  nf_stderr_id,-(sp)
    pea     0
    DC.W    $7301
    lea     12(SP),SP

    endif
 \@.nf_stderr_lit_noextra:

    movem.l	(sp)+,d0-d7/a0-a6
\@.nf_stderr_lit_ignore:
    endif
    endm

; print to stderr on Host-system
; a0 = string address
nf_stderr_reg      macro
    ifne _DEBUG
    tst.l   nf_has_flag
    beq.s   \@.nf_stderr_reg_ignore
    move.l d0, -(sp)
    move.l  a0,-(sp)
    move.l  nf_stderr_id,-(sp)
    pea     0
    DC.W    $7301
    lea     12(SP),SP
    move.w (sp)+, d0
\@.nf_stderr_reg_ignore:
    endif
    endm

nf_debugger     macro
    ifne _DEBUG
    move.l  nf_debugger_id,-(sp)
    pea     0
    DC.W    $7301
    lea     8(SP),SP
    endif
    endm

nf_status       macro
    ifne _DEBUG
    move.l d7, -(sp)
    move.l d0, d7
    nf_stderr_lit regd0, 8
    nf_crlf
    move.l d1, d7
    nf_stderr_lit regd1, 8
    nf_crlf
    move.l d2, d7
    nf_stderr_lit regd2, 8
    nf_crlf
    move.l (sp)+, d7
    nf_stderr_lit regd7, 8
    nf_crlf
    move.l d4, d7
    nf_stderr_lit regd4, 8
    nf_crlf
    move.l d5, d7
    nf_stderr_lit regd5, 8
    nf_crlf
    move.l d6, d7
    nf_stderr_lit regd6, 8
    nf_crlf
    move.l d7, d7
    nf_stderr_lit regd7, 8
    nf_crlf
    move.l a0, d7
    nf_stderr_lit rega0, 8
    nf_crlf
    move.l a1, d7
    nf_stderr_lit rega1, 8
    nf_crlf
    move.l a2, d7
    nf_stderr_lit rega2, 8
    nf_crlf
    move.l a3, d7
    nf_stderr_lit rega3, 8
    nf_crlf
    move.l a4, d7
    nf_stderr_lit rega4, 8
    nf_crlf
    move.l a5, d7
    nf_stderr_lit rega5, 8
    nf_crlf
    move.l a6, d7
    nf_stderr_lit rega6, 8
    nf_crlf
    move.l sp, d7
    nf_stderr_lit regsp, 8
    nf_crlf
    endif
    endm

    ifne _DEBUG
    regd0: dc.b "d0: ",0
    regd1: dc.b "d1: ",0
    regd2: dc.b "d2: ",0
    regd3: dc.b "d3: ",0
    regd4: dc.b "d4: ",0
    regd5: dc.b "d5: ",0
    regd6: dc.b "d6: ",0
    regd7: dc.b "d7: ",0
    rega0: dc.b "a0: ",0
    rega1: dc.b "a1: ",0
    rega2: dc.b "a2: ",0
    rega3: dc.b "a3: ",0
    rega4: dc.b "a4: ",0
    rega5: dc.b "a5: ",0
    rega6: dc.b "a6: ",0
    regsp: dc.b "sp: ",0
    endif

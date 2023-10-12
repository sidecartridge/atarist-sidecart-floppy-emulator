; SidecarT Floppy Disk Drive (FDD) Emulator
; (C) 2023 by Diego Parrilla
; License: GPL v3

; Emulate a Floppy Disk Drive (FDD) from the SidecarT

; Bootstrap the code in ASM

    XDEF   rom_function

    ifne _DEBUG
    XREF    read_buff_msg_set_bpb
    XREF    read_buff_msg_end_xbios_rw
    XREF    read_buff_msg_end_gemdos_rw
    XREF    nf_has_flag
    XREF    nf_stderr_crlf
    XREF    nf_stderr_id
    XREF    nf_hexnum_buff
    XREF    nf_debugger_id
    endif

    ifne _DEBUG
        XREF    do_transfer_ramdisk
        XREF    setBPB
        XREF    random_token
        XREF    random_token_seed
        XREF    image
        XREF    BPB_data
        XREF    old_XBIOS_trap
        XREF    old_hdv_bpb
        XREF    old_hdv_rw
        XREF    old_hdv_mediach
        XREF    disk_number
        XREF    secptrack
        XREF    secpcyl

        include inc/tos.s
        include inc/debug.s
    else
        random_token:        equ $FA7000
        random_token_seed:   equ random_token + 4 ; random_token + 0 bytes
        BPB_data:            equ random_token_seed + 4 ; random_token + 4 bytes
        trackcnt:            equ BPB_data + 18 ; BPB_data + 18 bytes
        sidecnt:             equ trackcnt + 2 ; trackcnt + 2 bytes
        secpcyl:             equ sidecnt + 2 ; sidecnt + 2 bytes
        secptrack:           equ secpcyl + 2 ; secpcyl + 2 bytes
        disk_number:         equ secptrack + 8 ; secptrack + 8 bytes
        old_XBIOS_trap:      equ disk_number + 2 ; disk_number + 2 bytes
        old_hdv_bpb:         equ old_XBIOS_trap + 4 ; old_XBIOS_trap + 4 bytes
        old_hdv_rw:          equ old_hdv_bpb + 4 ; old_hdv_bpb + 4 bytes
        old_hdv_mediach:     equ old_hdv_rw + 4 ; old_hdv_rw + 4 bytes
        image:               equ $FA8000 ; 
    endif

; CONSTANTS
XBIOS_trap      equ $b8     ; TRAP #14 Handler (XBIOS)
_bootdev        equ $446    ; This value represents the device from which the system was booted (0 = A:, 1 = B:, etc.)
screenpt        equ $45E    ; If this value is non-zero then at the next vertical blank, 
                            ; the value stored here will be loaded into the hardware register which points to the base of the physical screen
hdv_bpb         equ $472    ; This vector is used when Getbpb() is called. A value of 0 indicates that no hard disk is attached.
hdv_rw          equ $476    ; This vector is used when Rwabs() is called. A value of 0 here indicates that no hard disk is attached
hdv_mediach     equ $47e    ; This vector is used when Mediach() is called. A value of 0 here indicates that no hard disk is attached.
_nflops         equ $4a6    ; This value indicates the number of floppy drives currently connected to the system
_drvbits        equ $4c2    ; Each of 32 bits in this longword represents a drive connected to the system. Bit #0 is A, Bit #1 is B and so on.

RANDOM_SEED     equ $1284FBCD ; Random seed for the random number generator. Should be provided by the pico in the future
PING_WAIT_TIME  equ 8         ; Number of seconds (aprox) to wait for a ping response from the Sidecart. Power of 2 numbers. Max 127.

ROM3_START_ADDR     equ $FB0000 ; ROM3 start address
CMD_MAGIC_NUMBER    equ (ROM3_START_ADDR + $ABCD)   ; Magic number to identify a command
APP_FLOPPYEMUL      equ $0200                       ; MSB is the app code. Floppy emulator is $02
CMD_SAVE_VECTORS    equ ($0 + APP_FLOPPYEMUL)     ; Command code to save the old vectors
CMD_SET_BPB         equ ($1 + APP_FLOPPYEMUL)     ; Command code to set the BPB of the emulated disk
CMD_READ_SECTOR     equ ($2 + APP_FLOPPYEMUL)     ; Command code to read a sector from the emulated disk
CMD_WRITE_SECTOR    equ ($3 + APP_FLOPPYEMUL)     ; Command code to write a sector to the emulated disk
CMD_PING            equ ($4 + APP_FLOPPYEMUL)     ; Command code to ping to the Sidecart

    ifne _RELEASE
        include inc/tos.s
    endif

	section code

    ifne _RELEASE
        org $FA0040
    endif
rom_function:
    print floppy_emulator_msg
    print loading_image_msg

    ifeq _DEBUG
    bsr ping
    tst.w d0
    beq.s _ping_ok

    asksil error_sidecart_comm_msg
    endif

_ping_ok:
    print save_vectors_msg

    bsr save_vectors
    tst.w d0
    beq.s _init_BPB
 
    asksil error_save_vectors_msg
    rts

_init_BPB:
    print setup_BPB_msg

    bsr setBPB
    tst.w d0
    beq.s _init_vectors

    asksil error_setup_BPB_msg
    rts

_init_vectors:

    print init_vectors_msg

    bsr set_new_vectors

    print boot_disk_msg

    bra boot_disk

; Simple PING command to test the communication between the ATARI ST and the RP2040
ping:
    move.w #PING_WAIT_TIME, d2           ; Wait for a while until ping responds
    lsl.w #2, d2
_reping:
    move.w d2, -(sp)                 
    move.w #CMD_PING,d0              ; Command code PING
    move.w #0,d1                     ; Payload size is 0 bytes. No payload
    bsr send_sync_command_to_sidecart
    move.w (sp)+, d2
    tst.w d0
    beq.s _exit_ping
    dbf d2, _reping
_exit_ping:
    rts

; Save the old vectors and install the new ones
; For testing purposes this is also implemented in the ATARI ST side
; but the final implementation should be in the RP2040 side
; sending a command to the RP2040 to do store this data in a data structure
; that will be consumed by the ATARI ST side at boot time.
save_vectors:

; Save the old vectors in memory of the ATARI ST in debug/testing mode
    ifne _DEBUG
        move.l  hdv_bpb.w,old_hdv_bpb
        move.l  hdv_rw.w,old_hdv_rw
        move.l  hdv_mediach.w,old_hdv_mediach
        move.l  XBIOS_trap.w,old_XBIOS_trap
        clr.w   d0
    else
; Send the old vectors to the RP2040 side to create the data structure
; that will be consumed by the ATARI ST side at boot time.
        move.w #CMD_SAVE_VECTORS,d0         ; Command code SAVE_VECTORS
        move.w #16,d1                       ; Payload size is 16 bytes
        move.l hdv_bpb.w,d3                 ; Payload is the old hdv_bpb
        move.l hdv_rw.w,d4                  ; Payload is the old hdv_rw
        move.l hdv_mediach.w,d5             ; Payload is the old hdv_mediach
        move.l XBIOS_trap.w,d6              ; Payload is the old XBIOS_trap
        bsr send_sync_command_to_sidecart
    endif

    rts

; Restore the old vectors
; Only implemented for testing purposes in the ATARI ST side
restore_vectors:
    move.l  old_hdv_bpb,hdv_bpb.w
    move.l  old_hdv_rw,hdv_rw.w
    move.l  old_hdv_mediach,hdv_mediach.w
    move.l  old_XBIOS_trap,XBIOS_trap.w
    rts

; Set the new vectors
set_new_vectors:
    move.l  #new_hdv_bpb_routine,hdv_bpb.w
    move.l  #new_hdv_rw_routine,hdv_rw.w
    move.l  #new_hdv_mediac_routine,hdv_mediach.w
    move.l  #new_XBIOS_trap_routine,XBIOS_trap.w
    rts

; Call to the Sidecart RP2040 to calculate the BPB of the emulated disk
    ifeq    _DEBUG
setBPB:
        move.w #CMD_SET_BPB,d0              ; Command code SET_BPB
        move.w #0,d1                        ; Payload size is 0 bytes. No payload
        bsr send_sync_command_to_sidecart
        rts
    endif

boot_disk:
    moveq   #1, d0
    clr.w _bootdev.w        ; Set emulated A as bootdevice
    move.w #2,_nflops.w     ; Simulate that floppy B is attached (it will be physical A)
    clr.l screenpt.w        ; ??? OK! Profibuch error?

    ; load bootsector and execute it if checksum is $1234
    move.w #1,-(sp)         ; sector 1
    clr.w -(sp)             ; side 0
    clr.w -(sp)             ; track 0
    move.w #1,-(sp)         ; sector 1 (boot)
    clr.w -(sp)             ; Drive A
    clr.l -(sp)             ; dummy
    pea $2000               ; Star reading at $2000
    move.w #8,-(sp)         ; floppy read
    trap #14                ; XBIOS
    lea $14(sp),sp          ; Restore stack

    ; Test checksum

    lea $2000,a1            ; Start reading at $2000
    move.l a1,a0
    move.w #255,d1          ; Read 512 bytes
    clr.l d2
_checksum_loop:             ; Calculate checksum
    add.w (a1)+,d2  
    dbf d1,_checksum_loop

    cmp.w #$1234,d2         ; Compare to the magic numnber
    bne.s _dont_boot        ; If equal, boot at $2000
    jmp (a0)                

_dont_boot:
    rts


; New hdv_bpb routine
; Load the BPB of the emulated disk if it's drive A or B
; While testing the code in the ATARI ST side, first built the BPB in the ATARI ST side
; But the final implementation should be in the RP2040 side
new_hdv_bpb_routine:
    move.w 4(sp),d0             ; Read the drive doing the stuff
    cmp.w disk_number,d0        ; Is this the disk_number we are emulating? 
    beq.s load_emul_bpp         ; If is the disk to emulate, load the BPB built 
    cmp.b #1,d0                 ; Is it the Drive B?
    bne.s not_emul_bpp          ; If not, jump to the old routine
    clr.w 4(sp)                 ; Map B as A
not_emul_bpp:
    move.l old_hdv_bpb,a0
    jmp (a0)

load_emul_bpp:
    move.l #BPB_data,d0         ; Load the emulated BPP
    rts  

; New hdv_rw routine
; Read/Write the emulated disk if it's drive A or B
; While testing the code in the ATARI ST side, read the data from the sample image
; But the final implementation should read from the RP2040 side thanks to the 
; commands sent from the ATARI ST side
new_hdv_rw_routine:
    move.w 14(sp),d0            ; Read the drive doing the stuff
    cmp.w disk_number,d0        ; Is this the disk_number we are emulating?
    beq.s exe_emul_rw           ; If is the disk to emulate, read/write the emulated disk
    cmp.b #1,d0                 ; Is it the Drive B?
    bne.s not_exe_emul_rw       ; If not, jump to the old routine
    clr.w 14(sp)                ; Map B as A

not_exe_emul_rw:
    move.l old_hdv_rw,a0
    jmp (a0)

exe_emul_rw:
    sf d3                       ; flag for RWABS call
exe_emul_rw_all:
    move.l 6(sp),a0             ; Buffer address
    move.l a0,d2
    tst.l d2                    ; zbog blokade pri "open A if all via C"
    beq no_buffer_error       ; just skip by error?

    moveq #0,d0
    move.l d0,d1
    move.l d0,d2
    move.l d0,d4
    move.w 12(sp),d0            ; recno, logical sector number
    move.w 10(sp),d1            ; Number of sectors to read/write
    move.w BPB_data, d2         ; Sector size
    move.w 4(sp), d4            ; rwflag

    ifne _DEBUG
        bsr do_transfer_ramdisk     ; Do the transfer ramdisk
    else
        bsr do_transfer_sidecart    ; Do the transfer sidecart
    endif

no_buffer_error:
    clr.l d0
    tst.b d3
    bne.s xbios_exit
    ifne _DEBUG
        nf_str read_buff_msg_end_gemdos_rw
        nf_crlf
    endif
    rts
xbios_exit:
    ifne _DEBUG
        nf_str read_buff_msg_end_xbios_rw
        nf_crlf
    endif
    lea 20(sp),sp
    movem.l (sp)+,d1-d7/a1 
    rte

; New hdv_mediac routine
new_hdv_mediac_routine:
    move.w 4(sp),d0             ; Read the drive doing the stuff
    cmp.w disk_number,d0    ; Is this the disk_number we are emulating?
    beq.s media_changed         ; If is the disk we are emulating, go to media has changed
    cmp.b #1,d0                 ; Is it the Drive B?
    bne.s not_emul_media_change ; If not, jump to the old routine
    clr.w 4(sp)                 ; Map B as A

not_emul_media_change:
    move.l old_hdv_mediach,a0
    jmp (a0)

media_changed:
    moveq #0,d0
    rts

*New XBIOS map A calls to RAMdisk, B to physical A

new_XBIOS_trap_routine:
    move.l sp,a0
    btst #5,(sp)                    ; check if called from user mode
    bne.s _not_user                 ; if not, do not correct stack pointer
    move.l usp,a0                   ; if yes, correct stack pointer
    subq.l #6,a0                    ; correct stack pointer
_not_user:
    move.w 6(a0),d0                 ; get XBIOS call number
    cmp.w #8,d0                     ; is it XBIOS call 8?
    beq.s _floppy_read              ; if yes, go to flopppy read
    cmp.w #9,d0                     ; is it XBIOS call 9?
    bne.s _continue_xbios           ; if not, continue with XBIOS call
 
_floppy_read:
    cmp.w #1,16(a0)
    bne.s _floppy_read_emulated_a   ; if is not B then is A
    clr.w 16(a0)                    ; Map B drive to physical A  
_continue_xbios:
    move.l old_XBIOS_trap,a0        ; get old XBIOS vector
    jmp (a0)

_floppy_read_emulated_a:   
    movem.l a1/d1-d7,-(sp)
    lea -20(sp),sp                  ; If called from super mode
    st d3                           ; flag for XBIOS call
    addq.l #6,a0
    move.l  2(a0),6(sp)             ; buffer -OK
    move.w 18(a0),10(sp)            ; sector count - OK
    clr.l d0
    move.w 12(a0),d0                ; start sect no
    subq.w #1,d0                    ; one less because flosector starts  with 1, not 0

; Now need to calc logical sector #
    clr.l d1
    move.w 14(a0),d1                ; track no
    clr.l d2
    move.w 16(a0),d2                ; side no
    clr.l d4
    move.w secpcyl,d4               ; sec/track
    mulu d4,d1
    add.w d1,d0                     ; track no * sec/track + start sect no

    move.w secptrack,d4             ; sec/cyl
    mulu d4,d2
    add.w d2,d0                     ; side no * sec/cyl + track no * sec/track + start sect no
    move.w d0,12(sp)                ; logical start sector for RAMdisk
    cmp.w #9,(a0)                   ; is write?
    beq.s _do_emulated_write
    clr.w 4(sp)                     ; flag for read
    bra exe_emul_rw_all 
_do_emulated_write:
    move.w #1,4(sp)
    bra exe_emul_rw_all   


; Perform the transference of the data from/to the emulated disk in RP2040 
; to the computer
; Input registers;
;  d0: logical sector number to start the transfer
;  d1: number of sectors to read/write
;  d2: sector size in bytes
;  d4: rwflag for read/write
;  a0: buffer address in the computer memory
; Output registers:
;  none
do_transfer_sidecart:
    tst.w d4                    ; test rwflag
    bne write_sidecart          ; if not, write
read_sidecart:
    move.l a0, a1
    bra read_sectors_from_sidecart
write_sidecart:
    move.l a0, a4
    bra write_sectors_from_sidecart  

; Read sectors from the sidecart
; Input registers:
;  d0: logical sector number to start the transfer
;  d1: number of sectors to read
;  d2: sector size in bytes
;  a0: address in the computer memory to store the data
; Output registers:
;  d0: error code, 0 if no error
;  a1: next address in the computer memory to store the data
read_sectors_from_sidecart:
    subq.w #1,d1                ; one less
_sectors_to_read:
    bsr read_sector_from_sidecart
    addq #1,d0
    dbf d1, _sectors_to_read
    rts

; Write sectors from the sidecart
; Input registers:
;  d0: logical sector number to start the transfer
;  d1: number of sectors to write
;  d2: sector size in bytes
;  a4: address in the computer memory to retrieve the data
; Output registers:
;  d0: error code, 0 if no error
;  a4: next address in the computer memory to retrieve the data
write_sectors_from_sidecart:
    subq.w #1,d1                ; one less
_sectors_to_write:
    bsr write_sector_to_sidecart
    addq #1,d0
    dbf d1, _sectors_to_write
    rts

; Read a sector from the sidecart
; Input registers:
;  d0: logical sector number to start the transfer
;  d2: sector size in bytes
;  a0: address in the computer memory to write
; Output registers:
;  d0: error code, 0 if no error
;  a1: next address in the computer memory to store
read_sector_from_sidecart:
    ; Implement the code to read a sector from the sidecart
    movem.w d0-d3, -(sp)                ; Save the number of sectors to read
    move.w d2, d4                       ; Save in d4 the number of bytes to copy
    move.w d0,d3                        ; Payload is the logical sector number
    swap d3
    move.w d2,d3                        ; Payload is the sector size
    move.w #CMD_READ_SECTOR,d0          ; Command code READ_SECTOR
    move.w #4,d1                        ; Payload size is 4 bytes
    movem.l d4/a0, -(sp)                ; Save the address of the buffer
    bsr send_sync_command_to_sidecart   ; Call the sync command to read a sector
    movem.l (sp)+, d4/a0                ; Restore the address of the buffer
    tst.w d0
    bne.s _error_reading_sector

    move.l #image, a1
    lsr.w #2, d4
    subq.w #1,d4                ; one less
_copy_sector_byte:
    move.l (a1)+, (a0)+
    dbf d4, _copy_sector_byte
    clr.w d0                    ; Clear the error code
_error_reading_sector:
    movem.w (sp)+, d0-d3           ; Recover the number of sectors to read
    rts

; Write a sector to the sidecart
; Input registers:
;  d0: logical sector number to start the transfer
;  d2: sector size in bytes
;  a4: address in the computer memory to write
; Output registers:
;  d0: error code, 0 if no error
;  a4: next address in the computer memory to retrieve
write_sector_to_sidecart:
    ; Implement the code to write a sector to the sidecart
    movem.w d0-d3, -(sp)                ; Save the number of sectors to read
    move.w d2, d4                       ; Save in d4 the number of bytes to copy
    move.w d0,d3                        ; Payload is the logical sector number
    swap d3
    move.w d2,d3                        ; Payload is the sector size
    move.w #CMD_WRITE_SECTOR,d0         ; Command code WRITE_SECTOR
    move.w d2, d1
    addq.w #4,d1                        ; Payload size is 4 bytes (d3.l) + the sector buffer size to transfer

    bsr send_sync_write_command_to_sidecart   ; Call the sync command to read a sector
    tst.w d0
    bne.s _error_writing_sector

    clr.w d0                       ; Clear the error code
_error_writing_sector:
    movem.w (sp)+, d0-d3           ; Recover the number of sectors to read
    rts




; Send an async command to the Sidecart
; Fire and forget style
; Input registers:
; d0.w: command code
; d1.w: payload size
; From d2 to d7 the payload based on the size of the payload field d1.w
; The order is: d2.l d2.h d3.l d3.h d4.l d4.h d5.l d5.h d6.l d6.h d7.l d7.h
; the limit is not 12 words, but since this code is going to be executed in the
; Atari ST ROM, its difficult to not use a buffer in RAM
; Output registers:
; d1-d7 are modified. a0-a3 modified.
send_async_command_to_sidecart:
    move.l #_end_async_code_in_stack - _start_async_code_in_stack, d7
    lea -(_end_async_code_in_stack - _start_async_code_in_stack)(sp), sp
    move.l sp, a2
    lea _start_async_code_in_stack, a1    ; a1 points to the start of the code in ROM
    lsr.w #2, d7
    subq #1, d7
_copy_async_code:
    move.l (a1)+, (a2)+
    dbf d7, _copy_async_code
    jsr (a3)                                                            ; Jump to the code in the stack
    lea (_end_async_code_in_stack - _start_async_code_in_stack)(sp), sp
    rts

; Send an sync command to the Sidecart
; Wait until the command sets a response in the memory with a random number used as a token
; Input registers:
; d0.w: command code
; d1.w: payload size
; From d3 to d7 the payload based on the size of the payload field d1.w
; Output registers:
; d0: error code, 0 if no error
; d1-d7 are modified. a0-a3 modified.
send_sync_command_to_sidecart:
    move.l #_end_sync_code_in_stack - _start_sync_code_in_stack, d7
    lea -(_end_sync_code_in_stack - _start_sync_code_in_stack)(sp), sp
    move.l sp, a2
    move.l sp, a3
    lea _start_sync_code_in_stack, a1    ; a1 points to the start of the code in ROM
    lsr.w #2, d7
    subq #1, d7
_copy_sync_code:
    move.l (a1)+, (a2)+
    dbf d7, _copy_sync_code
    move.w #$4e71, (_no_async_return - _start_sync_code_in_stack)(a3)   ; Put a NOP when sync
    jsr (a3)                                                            ; Jump to the code in the stack
    lea (_end_sync_code_in_stack - _start_sync_code_in_stack)(sp), sp
    rts

_start_sync_code_in_stack:
    ; The sync command synchronize with a random token
    move.l random_token_seed,d2
    mulu  #221,d2
    add.b #53, d2                       ; Save the random number in d2
    addq.w #4, d1                       ; Add 4 bytes to the payload size to include the token

_start_async_code_in_stack:
    move.l #ROM3_START_ADDR, a0 ; Start address of the ROM3

    ; SEND HEADER WITH MAGIC NUMBER
    swap d0                     ; Save the command code in the high word of d0         
    move.b CMD_MAGIC_NUMBER, d0; Command header. d0 is a scratch register

    ; SEND COMMAND CODE
    swap d0                     ; Recover the command code
    move.l a0, a1               ; Address of the ROM3
    add.w d0, a1                ; We can use add because the command code msb is 0 and there is no sign extension            
    move.b (a1), d0             ; Command code. d0 is a scratch register

    ; SEND PAYLOAD SIZE
    move.l a0, d0               ; Address of the ROM3 in d0    
    or.w d1, d0                 ; OR high and low words in d0
    move.l d0, a1               ; move to a1 ready to read from this address
    move.b (a1), d0             ; Command payload size. d0 is a scratch register
    tst.w d1
    beq _no_more_payload_stack        ; If the command does not have payload, we are done.

    ; SEND PAYLOAD
    move.l a0, d0
    or.w d2, d0
    move.l d0, a1
    move.b (a1), d0           ; Command payload low d2
    cmp.w #2, d1
    beq _no_more_payload_stack

    swap d2
    move.l a0, d0
    or.w d2, d0
    move.l d0, a1
    move.b (a1), d0           ; Command payload high d2
    cmp.w #4, d1
    beq _no_more_payload_stack

    move.l a0, d0
    or.w d3, d0
    move.l d0, a1
    move.b (a1), d0           ; Command payload low d3
    cmp.w #6, d1
    beq _no_more_payload_stack

    swap d3
    move.l a0, d0
    or.w d3, d0
    move.l d0, a1
    move.b (a1), d0           ; Command payload high d3
    cmp.w #8, d1
    beq _no_more_payload_stack

    move.l a0, d0
    or.w d4, d0
    move.l d0, a1
    move.b (a1), d0           ; Command payload low d4
    cmp.w #10, d1
    beq _no_more_payload_stack

    swap d4
    move.l a0, d0
    or.w d4, d0
    move.l d0, a1
    move.b (a1), d0           ; Command payload high d4
    cmp.w #12, d1
    beq.s _no_more_payload_stack

    move.l a0, d0
    or.w d5, d0
    move.l d0, a1
    move.b (a1), d0           ; Command payload low d5
    cmp.w #14, d1
    beq.s _no_more_payload_stack

    swap d5
    move.l a0, d0
    or.w d5, d0
    move.l d0, a1
    move.b (a1), d0           ; Command payload high d5
    cmp.w #16, d1
    beq.s _no_more_payload_stack

    move.l a0, d0
    or.w d6, d0
    move.l d0, a1
    move.b (a1), d0           ; Command payload low d6
    cmp.w #18, d1
    beq.s _no_more_payload_stack

    swap d6
    move.l a0, d0
    or.w d6, d0
    move.l d0, a1
    move.b (a1), d0           ; Command payload high d6

_no_more_payload_stack:
    swap d2                   ; D2 is the only register that is not used as a scratch register
_no_async_return:
    rts                 ; if the code is SYNC, we will NOP this
_end_async_code_in_stack:

    move.l #$FFFFFFFF, d7
_wait_sync_for_token_a_lot:
    swap d7
_wait_sync_for_token:
    cmp.l random_token, d2              ; Compare the random number with the token
    beq.s _sync_token_found                  ; Token found, we can finish succesfully
    dbf d7, _wait_sync_for_token
    swap d7
    dbf d7, _wait_sync_for_token_a_lot
_sync_token_not_found:
    moveq #-1, d0                     ; Timeout
    rts
_sync_token_found:
    clr.w d0                            ; Clear the error code
    rts
    nop

_end_sync_code_in_stack:

    rts

; Send an sync write command to the Sidecart
; Wait until the command sets a response in the memory with a random number used as a token
; Input registers:
; d0.w: command code
; d1.w: payload size
; d3.l: size sector and logical sector number
; a4: address of the buffer to write in the sidecart
; Output registers:
; d0: error code, 0 if no error
; a4: next address in the computer memory to retrieve
; d1-d7 are modified. a0-a3 modified.
send_sync_write_command_to_sidecart:
    move.l #_end_sync_write_code_in_stack - _start_sync_write_code_in_stack, d7
    lea -(_end_sync_write_code_in_stack - _start_sync_write_code_in_stack)(sp), sp
    move.l sp, a2
    move.l sp, a3
    lea _start_sync_write_code_in_stack, a1    ; a1 points to the start of the code in ROM
    lsr.w #2, d7
    subq #1, d7
_copy_write_sync_code:
    move.l (a1)+, (a2)+
    dbf d7, _copy_write_sync_code
    move.w #$4e71, (_no_async_write_return - _start_sync_write_code_in_stack)(a3)   ; Put a NOP when sync
    jsr (a3)                                                            ; Jump to the code in the stack
    lea (_end_sync_write_code_in_stack - _start_sync_write_code_in_stack)(sp), sp
    rts

_start_sync_write_code_in_stack:
    ; The sync write command synchronize with a random token
    move.l random_token_seed,d2
    mulu  #221,d2
    add.b #53, d2                       ; Save the random number in d2
    addq.w #4, d1                       ; Add 4 bytes to the payload size to include the token

_start_async_write_code_in_stack:
    move.l #ROM3_START_ADDR, a0 ; Start address of the ROM3

    ; SEND HEADER WITH MAGIC NUMBER
    swap d0                     ; Save the command code in the high word of d0         
    move.b CMD_MAGIC_NUMBER, d0; Command header. d0 is a scratch register

    ; SEND COMMAND CODE
    swap d0                     ; Recover the command code
    move.l a0, a1               ; Address of the ROM3
    add.w d0, a1                ; We can use add because the command code msb is 0 and there is no sign extension            
    move.b (a1), d0             ; Command code. d0 is a scratch register

    ; SEND PAYLOAD SIZE
    move.l a0, d0               ; Address of the ROM3 in d0    
    or.w d1, d0                 ; OR high and low words in d0
    move.l d0, a1               ; move to a1 ready to read from this address
    move.b (a1), d0             ; Command payload size. d0 is a scratch register

    ; SEND PAYLOAD
    move.l a0, d0
    or.w d2, d0
    move.l d0, a1
    move.b (a1), d0           ; Command payload low d2

    swap d2
    move.l a0, d0
    or.w d2, d0
    move.l d0, a1
    move.b (a1), d0           ; Command payload high d2

    move.l a0, d0
    or.w d3, d0
    move.l d0, a1
    move.b (a1), d0           ; Command payload low d3

    swap d3
    move.l a0, d0
    or.w d3, d0
    move.l d0, a1
    move.b (a1), d0           ; Command payload high d3

    ; SEND MEMORY BUFFER TO WRITE
    lsr.w #1, d4              ; Copy two bytes each iteration
    subq.w #1, d4             ; one less
_write_to_sidecart_loop:
    move.w (a4)+, d3
    move.l a0, d0
    or.w d3, d0
    move.l d0, a1
    move.b (a1), d0           ; Write the memory to the sidecart
    dbf d4, _write_to_sidecart_loop

    ; End of the command loop. Now we need to wait for the token
    swap d2                   ; D2 is the only register that is not used as a scratch register
_no_async_write_return:
    rts                 ; if the code is SYNC, we will NOP this
_end_async_write_code_in_stack:

    move.l #$FFFFFFFF, d7
_wait_sync_write_for_token_a_lot:
    swap d7
_wait_sync_write_for_token:
    cmp.l random_token, d2                         ; Compare the random number with the token
    beq.s _sync_write_token_found                  ; Token found, we can finish succesfully
    dbf d7, _wait_sync_write_for_token
    swap d7
    dbf d7, _wait_sync_write_for_token_a_lot
_sync_write_token_not_found:
    moveq #-1, d0                     ; Timeout
    rts
_sync_write_token_found:
    clr.w d0                            ; Clear the error code
    rts
    nop

_end_sync_write_code_in_stack:

    rts




floppy_emulator_msg:
        dc.b	"SidecarT Floppy Emulator - "
        
version:
        dc.b    "v0.0.1",$d,$a

spacing:
        dc.b    "+" ,$d,$a,0

loading_image_msg:
        dc.b	"+- Loading the image...",$d,$a,0

error_sidecart_comm_msg:
        dc.b	"Sidecart error communication. Reset!",$d,$a,0

save_vectors_msg:
        dc.b	"+- Saving the old vectors",$d,$a,0

error_save_vectors_msg:
        dc.b	"Error saving the old vectors",$d,$a,0

init_vectors_msg:
        dc.b	"+- Initializing new vectors",$d,$a,0

setup_BPB_msg:
        dc.b	"+- Setting BPB of the emulated disk",$d,$a,0

error_setup_BPB_msg:
        dc.b	"Error setting BPB of the emulated disk",$d,$a,0

boot_disk_msg:
        dc.b	"+- Booting emulated disk",$d,$a,0

        even
rom_function_end:
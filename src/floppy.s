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
        XREF    sidecart_read_buf
        XREF    BPB_data_A
        XREF    BPB_data_B
        XREF    old_XBIOS_trap
        XREF    old_hdv_bpb
        XREF    old_hdv_rw
        XREF    old_hdv_mediach
        XREF    disk_number_A
        XREF    secptrack_A
        XREF    secpcyl_A
        XREF    disk_number_B
        XREF    secptrack_B
        XREF    secpcyl_B

        include inc/tos.s
        include inc/debug.s
    else
        ROM3_START_ADDR:     equ $FB0000 ; ROM3 start address
        sidecart_read_buf:   equ (ROM3_START_ADDR + $1000) ; random_token + $1000 bytes
        random_token:        equ ROM3_START_ADDR ; ROM3_START_ADDR + 0 bytes
        random_token_seed:   equ (random_token + 4) ; random_token + 0 bytes
        BUFFER_TYPE:         equ (random_token_seed + 4)       ; random_token_seed + 4 byte

        BPB_data_A:            equ (BUFFER_TYPE + 4) ; Buffer type + 4 bytes
        trackcnt_A:            equ BPB_data_A + 18 ; BPB_data + 18 bytes
        sidecnt_A:             equ trackcnt_A + 2 ; trackcnt + 2 bytes
        secpcyl_A:             equ sidecnt_A + 2 ; sidecnt + 2 bytes
        secptrack_A:           equ secpcyl_A + 2 ; secpcyl + 2 bytes
        disk_number_A:         equ secptrack_A + 8 ; secptrack + 8 bytes

        BPB_data_B:            equ (disk_number_A + 2) ; disk_number_A + 4 bytes
        trackcnt_B:            equ BPB_data_B + 18 ; BPB_data + 18 bytes
        sidecnt_B:             equ trackcnt_B + 2 ; trackcnt + 2 bytes
        secpcyl_B:             equ sidecnt_B + 2 ; sidecnt + 2 bytes
        secptrack_B:           equ secpcyl_B + 2 ; secpcyl + 2 bytes
        disk_number_B:         equ secptrack_B + 8 ; secptrack + 8 bytes


        old_XBIOS_trap:      equ disk_number_B + 6 ; disk_number + 4 bytes +  2 bytes alignment
        old_hdv_bpb:         equ old_XBIOS_trap + 4 ; old_XBIOS_trap + 4 bytes
        old_hdv_rw:          equ old_hdv_bpb + 4 ; old_hdv_bpb + 4 bytes
        old_hdv_mediach:     equ old_hdv_rw + 4 ; old_hdv_rw + 4 bytes
        hardware_type:       equ old_hdv_mediach + 4 ; old_hdv_mediach + 4 bytes

        read_checksum:       equ hardware_type + 4 ; network_timeout_sec + 4 bytes

        IP_ADDRESS:          equ read_checksum + 4   ; read_checksum + 4 bytes
        HOSTNAME:            equ IP_ADDRESS + 128    ; ip_address + 128 bytes

        SHARED_VARIABLES:    equ random_token + 512         ; random token + 512 bytes to the shared variables area

        SVAR_DO_TRANSFER:    equ SHARED_VARIABLE_SHARED_FUNCTIONS_SIZE + 0             ; Entry address of the transfer function
        SVAR_EXIT_TRANSFER:  equ SHARED_VARIABLE_SHARED_FUNCTIONS_SIZE + 1             ; Exit address of the transfer function
        SVAR_XBIOS_TRAP_ENABLED: equ SHARED_VARIABLE_SHARED_FUNCTIONS_SIZE + 2          ; XBIOS trap enabled
        SVAR_BOOT_ENABLED:   equ SHARED_VARIABLE_SHARED_FUNCTIONS_SIZE + 3             ; Boot sector enabled
        SVAR_PING_STATUS:    equ SHARED_VARIABLE_SHARED_FUNCTIONS_SIZE + 4          ; Ping status
        SVAR_PING_TIMEOUT:   equ SHARED_VARIABLE_SHARED_FUNCTIONS_SIZE + 5          ; Ping timeout

        SVAR_MEDIA_CHANGED_A:   equ SHARED_VARIABLE_SHARED_FUNCTIONS_SIZE + 6       ; Media changed A
        SVAR_MEDIA_CHANGED_B:   equ SHARED_VARIABLE_SHARED_FUNCTIONS_SIZE + 7       ; Media changed B
        SVAR_EMULATION_MODE:    equ SHARED_VARIABLE_SHARED_FUNCTIONS_SIZE + 8       ; Emulation mode

    endif

; CONSTANTS
XBIOS_trap      equ $b8     ; TRAP #14 Handler (XBIOS)
_membot         equ $432    ; This value represents last memory used by the TOS, and start of the heap area available
_bootdev        equ $446    ; This value represents the device from which the system was booted (0 = A:, 1 = B:, etc.)

hdv_bpb         equ $472    ; This vector is used when Getbpb() is called. A value of 0 indicates that no hard disk is attached.
hdv_rw          equ $476    ; This vector is used when Rwabs() is called. A value of 0 here indicates that no hard disk is attached
hdv_mediach     equ $47e    ; This vector is used when Mediach() is called. A value of 0 here indicates that no hard disk is attached.
_nflops         equ $4a6    ; This value indicates the number of floppy drives currently connected to the system
_drvbits        equ $4c2    ; Each of 32 bits in this longword represents a drive connected to the system. Bit #0 is A, Bit #1 is B and so on.
_dskbufp        equ $4c6    ; Address of the disk buffer pointer    
_longframe      equ $59e    ; Address of the long frame flag. If this value is 0 then the processor uses short stack frames, otherwise it uses long stack frames.


RANDOM_SEED     equ $1284FBCD ; Random seed for the random number generator. Should be provided by the pico in the future

ROM_EXCHG_BUFFER_ADDR equ (ROM3_START_ADDR)               ; ROM3 buffer address
RANDOM_TOKEN_ADDR:        equ (ROM_EXCHG_BUFFER_ADDR)
RANDOM_TOKEN_SEED_ADDR:   equ (RANDOM_TOKEN_ADDR + 4) ; RANDOM_TOKEN_ADDR + 0 bytes
RANDOM_TOKEN_POST_WAIT:   equ $64        ; Wait this cycles after the random number generator is ready
WRITE_COMMAND_RETRIES:    equ 3          ; Number of retries to send a command to the RP2040


CMD_MAGIC_NUMBER    equ (ROM3_START_ADDR + $ABCD)   ; Magic number to identify a command
APP_FLOPPYEMUL      equ $0200                       ; MSB is the app code. Floppy emulator is $02
CMD_SAVE_VECTORS    equ ($0 + APP_FLOPPYEMUL)     ; Command code to save the old vectors
CMD_READ_SECTOR     equ ($1 + APP_FLOPPYEMUL)     ; Command code to read a sector from the emulated disk
CMD_WRITE_SECTOR    equ ($2 + APP_FLOPPYEMUL)     ; Command code to write a sector to the emulated disk
CMD_PING            equ ($3 + APP_FLOPPYEMUL)     ; Command code to ping to the Sidecart
CMD_SAVE_HARDWARE   equ ($4 + APP_FLOPPYEMUL)     ; Command code to save the hardware type of the ATARI computer
CMD_SET_SHARED_VAR  equ ($5 + APP_FLOPPYEMUL)     ; Command code to set a shared variable
CMD_RESET           equ ($6 + APP_FLOPPYEMUL)     ; Command code to reset the floppy emulator before starting the boot process
CMD_MOUNT_DRIVE_A   equ ($7 + APP_FLOPPYEMUL)     ; Mount the drive A of the floppy emulator
CMD_UNMOUNT_DRIVE_A equ ($8 + APP_FLOPPYEMUL)     ; Unmount the drive A of the floppy emulator
CMD_MOUNT_DRIVE_B   equ ($9 + APP_FLOPPYEMUL)     ; Mount the drive B of the floppy emulator
CMD_UNMOUNT_DRIVE_B equ ($A + APP_FLOPPYEMUL)     ; Unmount the drive B of the floppy emulator
CMD_SHOW_VECTOR_CALL equ ($B + APP_FLOPPYEMUL)    ; Command code to show the XBIOS vector call 

    ifne _RELEASE
        include inc/tos.s
    endif
    include inc/sidecart_macros.s

	section code

    ifne _RELEASE
        org $FA0040
    endif
rom_function:
    print floppy_emulator_msg

    ; Test if the floppy emulator was configured, if so, reset it
    tst.l (SHARED_VARIABLES + (SVAR_PING_STATUS * 4))
    beq.s _reset_bypass    
    send_sync CMD_RESET,0

_reset_bypass:
    ifeq _DEBUG
    print loading_image_msg
    move.l (SHARED_VARIABLES + (SVAR_PING_TIMEOUT * 4))  , d7      ; retries of the ping command
_ping_retry:

    print xbios_params_msg
    tst.l (SHARED_VARIABLES + (SVAR_XBIOS_TRAP_ENABLED * 4))       ; 0: XBIOS trap disabled, Not 0: XBIOS trap enabled 
    beq.s _ping_disable_xbios_trap
    print on_msg
    bra.s _ping_bootsec
_ping_disable_xbios_trap:
    print off_msg

_ping_bootsec:
    print boot_params_msg
    tst.l (SHARED_VARIABLES + (SVAR_BOOT_ENABLED * 4))
    beq.s _ping_boot_sector_disabled
    print on_msg
    bra.s _ping_memory_buff
_ping_boot_sector_disabled:
    print off_msg

_ping_memory_buff:
    print memory_buff_msg
    tst.l (SHARED_VARIABLES + (SHARED_VARIABLE_BUFFER_TYPE * 4))
    beq.s _ping_memory_buff_diskbuf
    print memory_buff_stack_msg
    bra.s _ping_web
_ping_memory_buff_diskbuf:
    print memory_buff_dskbuf_msg

_ping_web:
    tst.b IP_ADDRESS
    beq.s _ping_noweb
    print cr
    print web_ip_msg
    pea	IP_ADDRESS
	gemdos	Cconws,6
    print web_host_msg
    pea HOSTNAME
    gemdos	Cconws,6
    print web_drive_msg
    print cr
    bra.s _ping_continue
_ping_noweb:
    print cr
    print cr
    print cr

    ; Test with the PING command if the RP2040 is ready to serve or not
_ping_continue:
    move.w d7, -(sp)
    send_sync CMD_PING, 0
    move.w (sp)+, d7
    tst.l (SHARED_VARIABLES + (SVAR_PING_STATUS * 4))
    beq.s _ping_configuring
    print boot_ready_msg
    bra.s _ping_configuration_ready
_ping_configuring:
    print boot_wait_msg
_ping_configuration_ready:

    ; Show the count down message
    print boot_countdown_msg
    move.w d7,d0                    ; Pass the number of seconds to print
    print_num                       ; Print the decimal number
    print boot_countdown_secs_msg
    print backwards_msg
    move.w d7, -(sp)                ; Save the number of seconds to wait    

    wait_sec_or_key_press

    cmp.b #'x', d0                  ; Check if 'x' is pressed otherwise continue
    bne.s _ping_check_boot

    ; Toogle the XBIOS trap
    move.l #SVAR_XBIOS_TRAP_ENABLED, d3
    move.l (SHARED_VARIABLES + (SVAR_XBIOS_TRAP_ENABLED * 4)), d4
    not.l d4
    send_sync CMD_SET_SHARED_VAR, 8
    bra.s _ping_ok

_ping_check_boot:
    cmp.b #'b', d0                  ; Check if 'b' is pressed otherwise continue
    bne.s _ping_check_mem_type

    ; Toogle the BOOT trap
    move.l #SVAR_BOOT_ENABLED, d3
    move.l (SHARED_VARIABLES + (SVAR_BOOT_ENABLED * 4)), d4
    not.l d4
    send_sync CMD_SET_SHARED_VAR, 8
    bra.s _ping_ok

_ping_check_mem_type:
    cmp.b #'m', d0                  ; Check if 'm' is pressed otherwise continue
    bne.s _ping_check_esc

    ; Toogle the Memory buffer type
    move.l #SHARED_VARIABLE_BUFFER_TYPE, d3
    move.l (SHARED_VARIABLES + (SHARED_VARIABLE_BUFFER_TYPE * 4)), d4
    not.l d4
    send_sync CMD_SET_SHARED_VAR, 8
    bra.s _ping_ok

_ping_check_esc:
    cmp.b #27,d0                    ; Check if ESC is pressed and continue
    beq.s _ping_check_final_status

_ping_ok:
    print boot_up_msg
    move.w (sp)+, d7                ; Restore the number of retries
    dbf d7, _ping_retry
    bra.s _ping_check_ready_to_save_vectors

_ping_check_final_status:
    move.w (sp)+, d7                ; Restore the number of retries
_ping_check_ready_to_save_vectors:
    ; Test if it is intialized or not
    send_sync CMD_PING, 0
    tst.l (SHARED_VARIABLES + (SVAR_PING_STATUS * 4))
    bne.s _save_vectors

    ; There was an intialization error, so we can't continue
    asksil error_sidecart_comm_msg
    rts

    endif

_save_vectors:
    print up_msg
    print up_msg
    print boot_up_msg
    print show_ok_msg
    print cr
    print cr
    print save_vectors_msg

    bsr save_vectors
    tst.w d0
    beq.s _detect_hw_type
 
    asksil error_save_vectors_msg
    rts

_detect_hw_type:
    print ok_msg
    print detect_hardware_msg

    bsr _detect_hw
    tst.w d0
    beq.s _display_buffer_type

    asksil error_detect_hardware_msg
    rts

_display_buffer_type:
    print ok_msg
    tst.l (SHARED_VARIABLES + (SHARED_VARIABLE_BUFFER_TYPE * 4))
    beq.s _display_diskbuf_buffer_type
    print stack_buffer_msg
    bra.s _init_vectors
_display_diskbuf_buffer_type:
    print dskbuf_buffer_msg

_init_vectors:
    print ok_msg
    bsr set_new_vectors

    print boot_disk_msg
    print cr
    print cr
    print cr

    bra boot_disk

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
        move.l hdv_bpb.w,d3                 ; Payload is the old hdv_bpb
        move.l hdv_rw.w,d4                  ; Payload is the old hdv_rw
        move.l hdv_mediach.w,d5             ; Payload is the old hdv_mediach
        move.l XBIOS_trap.w,d6              ; Payload is the old XBIOS_trap
        send_sync CMD_SAVE_VECTORS, 16
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
    tst.l   (SHARED_VARIABLES + (SVAR_XBIOS_TRAP_ENABLED * 4))  ; 0: XBIOS trap disabled, Not 0: XBIOS trap enabled
    beq.s _dont_set_xbios_trap 
    move.l  #new_XBIOS_trap_routine,XBIOS_trap.w
_dont_set_xbios_trap:
    rts

boot_disk:
    clr.w _bootdev.w        ; Set emulated A as bootdevice    
    ; Configure drive A
    btst   #0, (SHARED_VARIABLES + (SVAR_EMULATION_MODE * 4) + 3) ; Bit 0: Emulate A
    beq.s _boot_disk_drive_b
    ; Emulate A
    tst.w _nflops.w                 ; Check if there are any floppies attached
    bne.s _boot_disk_drive_a_to_b   ; Since A is emulated and there is a drive, map physical A to B
    move.l #1,_drvbits.w            ; Create the drive A bit
    move.w #1,_nflops.w             ; Simulate that floppy A is attached
    bra.s _boot_disk_drive_b
_boot_disk_drive_a_to_b:
    move.l #3, _drvbits.w           ; Enable both drives
    move.w #2,_nflops.w             ; Simulate that floppy B is attached (it will be physical A)

_boot_disk_drive_b:
    ; Configure drive B
    btst   #1, (SHARED_VARIABLES + (SVAR_EMULATION_MODE * 4) + 3) ; Bit 1: Emulate B
    beq.s _start_boot
    ; Emulate B
    tst.w _nflops.w                 ; Check if there are any floppies attached
    beq.s _boot_disk_drive_b_not_keep_a ; Since B is emulated and there is a drive, Keep A as it is
    move.l #3,_drvbits.w            ; Create the drive B bit
    move.w #2,_nflops.w             ; Simulate that floppy B is attached and keep A
    bra.s _start_boot

_boot_disk_drive_b_not_keep_a:      ; If we are here is because B is emulated and there is no physical drive
    move.l #2, _drvbits.w           ; Enable both drives
    move.w #1, _nflops.w            ; Simulate that floppy A is not attached but B is attached

    ; load bootsector and execute it if checksum is $1234
_start_boot:
; Check if there is drive A
    btst #0, (_drvbits + 3).w
    beq.s _dont_boot
; Check if there is a physical drive A and not boot if so
    btst   #0, (SHARED_VARIABLES + (SVAR_EMULATION_MODE * 4) + 3) ; Bit 0: Emulate A
    beq.s _dont_boot

; Read sectors from the sidecart. Don't use XBIOS call

    moveq #0, d0            ; Start reading at sector 0
    move.l d0, d4           ; Read from drive A
    move.l d0, d2           ; clear d2.l 
    move.w BPB_data_A, d2   ; Sector size of the emulated drive A
    move.l _membot.w,a0        ; Start reading at $2000
    bsr read_sector_from_sidecart

    ; Test checksum

    move.l _membot.w,a1        ; Start reading at $2000
    move.l a1,a0
    move.w #255,d1          ; Read 512 bytes
    clr.l d2
_checksum_loop:             ; Calculate checksum
    add.w (a1)+,d2  
    dbf d1,_checksum_loop

    tst.l (SHARED_VARIABLES + (SVAR_BOOT_ENABLED * 4))  ; 0: Boot sector enabled,  Not 0: Boot sector disabled
    bne.s .boot_sector_enabled
    rts

.boot_sector_enabled:
    cmp.w #$1234,d2         ; Compare to the magic numnber
    bne.s _dont_boot        ; If equal, boot at $2000
    jmp (a0)                

_dont_boot:
    rts

_detect_hw:
    bsr detect_hw
    move.l d0, d3                       ; hardware type
    move.l #do_transfer_sidecart, d4    ; Address of the start function to overwrite the speed change
    move.l #exit_transfer_sidecart, d5  ; Address of the end function to overwrite the speed change
    send_sync CMD_SAVE_HARDWARE, 12
    rts

; New hdv_bpb routine
; Load the BPB of the emulated disk if it's drive A or B
; While testing the code in the ATARI ST side, first built the BPB in the ATARI ST side
; But the final implementation should be in the RP2040 side
new_hdv_bpb_routine:
    cmp.w #0,4(sp)              ; Is this the disk_number we are emulating? 
    beq.s _load_emul_bpp_A      ; If is the disk A to emulate, load the BPB A built 
    cmp.w #1,4(sp)              ; Is it the Drive B?
    beq.s _load_emul_bpp_B      ; If is the disk B to emulate, load the BPB B built 
_not_emul_bpp:
    move.l old_hdv_bpb,-(sp)
    rts

_load_emul_bpp_A:
    ; Test Drive A
    btst   #0, (SHARED_VARIABLES + (SVAR_EMULATION_MODE * 4) + 3) ; Bit 0: Emulate A
    beq.s _not_emul_bpp
    ; Emulate A
    move.l #BPB_data_A,d0         ; Load the emulated BPP A
    rts  

_load_emul_bpp_B:
    ; Test Drive B
    btst   #1, (SHARED_VARIABLES + (SVAR_EMULATION_MODE * 4) + 3) ; Bit 1: Emulate B
    beq.s _not_emul_bpp_B
    move.l #BPB_data_B,d0         ; Load the emulated BPP B
    rts  

_not_emul_bpp_B:
    clr.w 4(sp)                   ; Map B as A
    move.l old_hdv_bpb,-(sp)
    rts

; New hdv_rw routine
; Read/Write the emulated disk if it's drive A or B
; While testing the code in the ATARI ST side, read the data from the sample image
; But the final implementation should read from the RP2040 side thanks to the 
; commands sent from the ATARI ST side
new_hdv_rw_routine:
    cmp.w #0,14(sp)             ; Is this the disk_number we are emulating?
    beq.s _exe_emul_rw_A        ; If is the disk A to emulate, read/write the emulated disk
    cmp.w #1,14(sp)             ; Is it the Drive B?
    beq.s _exe_emul_rw_B        ; If is the disk B to emulate, read/write the emulated disk
    move.l old_hdv_rw,-(sp)     ; If is not the disk A or B, restore the old hdv_rw
    rts

_exe_emul_rw_A:
    ; Test Drive A
    btst   #0, (SHARED_VARIABLES + (SVAR_EMULATION_MODE * 4) + 3) ; Bit 0: Emulate A
    beq.s _not_emul_rw_A
    move.w BPB_data_A,d2       ; Sector size of the emulated drive A
    moveq #0, d4               ; Use A:
    bra.s exe_emul_rw
_not_emul_rw_A:
    move.l old_hdv_rw,-(sp)
    rts

_exe_emul_rw_B:
    ; Test Drive B
    btst   #1, (SHARED_VARIABLES + (SVAR_EMULATION_MODE * 4) + 3) ; Bit 1: Emulate B
    beq.s _not_emul_rw_B
    move.w BPB_data_B,d2       ; Sector size of the emulated drive B
    moveq #1, d4               ; Use B:
    bra.s exe_emul_rw
_not_emul_rw_B:
    clr.w 14(sp)                ; Map B as A
    move.l old_hdv_rw,-(sp)
    rts

exe_emul_rw:
    sf d3                       ; Set FALSE flag for RWABS call
exe_emul_rw_all:
    tst.l 6(sp)                 ; Check if buffer address is 0
    beq.s no_buffer_error       ; just skip by error?

    and.l #$0000FFFF,d2         ; limit the size of the sectors to 65535
    moveq #0,d0
    move.l d0,d1
    move.l d0,d5
    move.w 12(sp),d0            ; recno, logical sector number
    move.w 10(sp),d1            ; Number of sectors to read/write
    move.w 4(sp), d5            ; rwflag
    move.l 6(sp), a0            ; Buffer address

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
    lea 52(sp),sp
    movem.l (sp)+,d1-d7/a1-a6
    rte

; New hdv_mediac routine
new_hdv_mediac_routine:
    cmp.w #0,4(sp)              ; Is this the disk_number we are emulating? 
    beq.s _media_changed_A      ; If is the disk A to emulate, media changed A
    cmp.w #1,4(sp)              ; Is it the Drive B?
    beq.s _media_changed_B      ; If is the disk B to emulate, media changed B
_not_emul_media_change:
    move.l old_hdv_mediach,-(sp)
    rts

_media_changed_A:
    ; Test Drive A
    btst   #0, (SHARED_VARIABLES + (SVAR_EMULATION_MODE * 4) + 3) ; Bit 0: Emulate A
    beq.s _not_emul_media_change
    move.l (SHARED_VARIABLES + (SVAR_MEDIA_CHANGED_A * 4)),d0
    rts

_media_changed_B:
    ; Test Drive B
    btst   #1, (SHARED_VARIABLES + (SVAR_EMULATION_MODE * 4) + 3) ; Bit 1: Emulate B
    beq.s _not_emul_media_change_B
    move.l (SHARED_VARIABLES + (SVAR_MEDIA_CHANGED_B * 4)),d0
    rts
_not_emul_media_change_B:
    clr.w 4(sp)                 ; Map B as A
    move.l old_hdv_mediach,-(sp)
    rts

; New XBIOS map A calls to Sidecart,
; B to physical A if it exists
; The XBIOS, like the BIOS may utilize registers D0-D2 and A0-A2 as scratch registers and their
; contents should not be depended upon at the completion of a call. In addition, the function
; opcode placed on the stack will be modified.
new_XBIOS_trap_routine:
    btst #5, (sp)                         ; Check if called from user mode
    beq.s _user_mode                      ; if so, do correct stack pointer
_not_user_mode:
    move.l sp,a0                          ; Move stack pointer to a0
    bra.s _check_cpu
_user_mode:
    move.l usp,a0                          ; if user mode, correct stack pointer
    subq.l #6,a0
;
; This code checks if the CPU is a 68000 or not
;
_check_cpu:
    tst.w _longframe                          ; Check if the CPU is a 68000 or not
    beq.s _notlong
_long:
    addq.w #2, a0                             ; Correct the stack pointer parameters for long frames 
_notlong:

;   For debugging purposes
;    movem.l d0-d7/a0-a6,-(sp)
;    move.w 6(a0), d3                     ; get XBIOS call number
;    send_sync CMD_SHOW_VECTOR_CALL, 2    ; Send the command to the Sidecart. 2 bytes of payload
;    movem.l (sp)+, d0-d7/a0-a6
                                    ; get XBIOS call number
    cmp.w #8,6(a0)                  ; is it XBIOS call Flopwr?
    beq.s _floppy_rw                ; if yes, go to flopppy read
    cmp.w #9,6(a0)                  ; is it XBIOS call Floprd?
    beq.s _floppy_rw                ; if yes, go to flopppy read
    cmp.w #13,6(a0)                 ; is it XBIOS call Flopver?
    beq.s _floppy_verify            ; if yes, go to flopppy verify
    cmp.w #10,6(a0)                 ; is it XBIOS call Flopfmt?
    beq.s _floppy_format            ; if yes, go to flopppy format
    move.l old_XBIOS_trap, -(sp)    ; if not, continue with XBIOS call
    rts 

    ;
    ; Trapped XBIOS call 13 with the floppy disk drive verify function
    ; Return always verified successfully
    ;
_floppy_verify:
    cmp.w #1,16(a0)
    bne.s _floppy_verify_emulated_a   ; if is not B then is A
    clr.w 16(a0)                      ; Map B drive to physical A
    move.l old_XBIOS_trap, -(sp)      ; continue with XBIOS call
    rts 

_floppy_verify_emulated_a:
    clr.l d0                           ; If SidecarT, always verified successfully
    move.l  d0, 8(a0)                  ; Set the error code to 0 in buff
    rte

    ;
    ; Trapped XBIOS call 10 with the floppy disk drive format function
    ; Return always formatted with errors
_floppy_format:
    cmp.w #1,16(a0)
    bne.s _floppy_format_emulated_a   ; if is not B then is A
    clr.w 16(a0)                      ; Map B drive to physical A
    move.l old_XBIOS_trap, -(sp)      ; continue with XBIOS call
    rts
_floppy_format_emulated_a:
    moveq #-1, d0                     ; If SidecarT, always formatted with errors
    clr.l 8(a0)                       ; Set the error code to 0 in the buff
    rte

    ;
    ; Trapped XBIOS calls 8 and 9 with the floppy disk drive read and write functions
    ;
_floppy_rw:
    cmp.w #1,16(a0)
    bne.s _floppy_read_emulated_a     ; if is not B then is A
    clr.w 16(a0)                      ; Map B drive to physical A
    move.l old_XBIOS_trap, -(sp)      ; continue with XBIOS call
    rts 

_floppy_read_emulated_a:   
    movem.l d1-d7/a1-a6,-(sp)
    lea -52(sp),sp                  ; Rewind to the beginning of the stack parameters of the call
    addq.l #6,a0
    move.l  2(a0),6(sp)             ; buffer
    move.w 18(a0),10(sp)            ; sector count
    clr.l d0
    move.w 12(a0),d0                ; start sect no
    subq.w #1,d0                    ; one less because flosector starts  with 1, not 0

; Now need to calc logical sector #
    clr.l d1
    move.w 14(a0),d1                ; track no
    clr.l d2
    move.w 16(a0),d2                ; side no
    clr.l d4
    mulu secpcyl_A,d1               ; sec/track
    add.w d1,d0                     ; track no * sec/track + start sect no

    mulu secptrack_A,d2             ; sec/cyl
    add.w d2,d0                     ; side no * sec/cyl + track no * sec/track + start sect no
    move.w d0,12(sp)                ; logical start sector for read from emulated disk
    move.w #-1, 4(sp)               ; flag for read
    st d3                           ; Set TRUE flag for XBIOS call
    move.w BPB_data_A, d2           ; Sector size of the emulated drive A
    cmp.w #9,(a0)                   ; is write?
    beq exe_emul_rw_all
    clr.w 4(sp)                     ; No, is read
    bra exe_emul_rw_all 

; Perform the transference of the data from/to the emulated disk in RP2040 
; to the computer
; Input registers;
;  d0: logical sector number to start the transfer
;  d1: number of sectors to read/write
;  d2: sector size in bytes
;  d4: disk drive number to read (0 = A:, 1 = B:)
;  d5: rwflag for read/write
;  a0: buffer address in the computer memory
; Output registers:
;  none
do_transfer_sidecart:
    ; START: WE MUST 'NOP' 16 BYTES HERE
    move.b $ffff8e21.w, -(sp)        ; Save the old value of cpu speed. 4 BYTES
	and.b #%00000001,$ffff8e21.w     ; disable MSTe cache. 6 BYTES
	bclr.b #0,$ffff8e21.w            ; set CPU speed at 8mhz. 6 BYTES
    ; END: WE MUST 'NOP' 16 BYTES HERE
    tst.w d5                    ; test rwflag
    bne write_sidecart          ; if not, write
read_sidecart:
    move.l a0, a1
    bsr read_sectors_from_sidecart
    bra.s exit_transfer_sidecart
write_sidecart:
    move.l a0, a4
    bsr write_sectors_from_sidecart

exit_transfer_sidecart:
    ; START: WE MUST 'NOP' 4 BYTES HERE
    move.b (sp)+, $ffff8e21.w   ; Restore the old value of cpu speed. 4 BYTES
    ; END: WE MUST 'NOP' 4 BYTES HERE
    rts

; Read sectors from the sidecart
; Input registers:
;  d0: logical sector number to start the transfer
;  d1: number of sectors to read
;  d2: sector size in bytes
;  d4: disk drive number to read (0 = A:, 1 = B:)
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
;  d4: disk drive number to read (0 = A:, 1 = B:)
;  a4: address in the computer memory to retrieve the data
; Output registers:
;  d0: error code, 0 if no error
;  a4: next address in the computer memory to retrieve the data
write_sectors_from_sidecart:
    subq.w #1,d1                ; one less
_sectors_to_write:
    move.l d0, -(sp)
    bsr write_sector_to_sidecart
    tst.w d0
    bne.s _error_sectors_to_write
    move.l (sp)+, d0
    addq #1,d0
    dbf d1, _sectors_to_write
    rts
_error_sectors_to_write:
    move.l (sp)+, d0
    rts

; Read a sector from the sidecart
; Input registers:
;  d0: logical sector number to start the transfer
;  d2: sector size in bytes
;  d4: disk drive number to read (0 = A:, 1 = B:)
;  a0: address in the computer memory to write
; Output registers:
;  d0: error code, 0 if no error
;  a1: next address in the computer memory to store
read_sector_from_sidecart:
    ; Implement the code to read a sector from the sidecart
    movem.w d0-d5, -(sp)                ; Save the number of sectors to read
    move.w d2, d5                       ; Save in d5 the number of bytes to copy
    move.w d0,d3                        ; Payload is the logical sector number
    swap d3
    move.w d2,d3                        ; Payload is the sector size
    movem.l d1-d7/a0-a6, -(sp)          ; Save the address of the buffer
    send_sync CMD_READ_SECTOR, 8
    movem.l (sp)+, d1-d7/a0-a6          ; Recover the address of the buffer
    tst.w d0
    bne.s _error_reading_sector

    clr.l d0                            ; Clear the error code
    move.l #sidecart_read_buf, a1
    lsr.w #2, d5
    subq.w #1,d5                        ; one less
    move.l a0, d3
    btst #0,d3                          ; If it's even, take the fast lane. If it's odd, take the slow lane
    bne.s _copy_sector_byte_odd
_copy_sector_byte_even:
    move.l (a1)+, (a0)+
    dbf d5, _copy_sector_byte_even
_error_reading_sector:
    movem.w (sp)+, d0-d5           ; Recover the number of sectors to read
    rts
_copy_sector_byte_odd:
    move.b (a1)+, (a0)+
    move.b (a1)+, (a0)+
    move.b (a1)+, (a0)+
    move.b (a1)+, (a0)+
    dbf d5, _copy_sector_byte_odd
    bra.s _error_reading_sector



; Write a sector to the sidecart
; Input registers:
;  d0: logical sector number to start the transfer
;  d2: sector size in bytes
;  d4: disk drive number to read (0 = A:, 1 = B:)
;  a4: address in the computer memory to write
; Output registers:
;  d0: error code, 0 if no error
;  a4: next address in the computer memory to retrieve
write_sector_to_sidecart:
    ; Implement the code to write a sector to the sidecart
    moveq #WRITE_COMMAND_RETRIES, d7          ; Number of retries 
_write_sector_to_sidecart_once:
    movem.l d1-d7/a0-a6, -(sp)                ; Save the address of the buffer
    move.w d2, d6                             ; argument with the number of bytes to send
    move.w d0,d3                              ; Payload is the logical sector number
    swap d3
    move.w d2,d3                              ; Payload is the sector size
    move.w #CMD_WRITE_SECTOR,d0               ; Command code WRITE_SECTOR
    ; We don't set here d1.w with the size of the payload because the sync_write command ALWAYS
    ; sends the same payload in the d3.l, d4.l and d5.l registers
    ; So we need to assume in the RP2040 that we have to process or skip these values to read the
    ; sector buffer data sent with this command
    bsr send_sync_write_command_to_sidecart   ; Call the sync command to read a sector
    tst.w d0
    bne.s _error_writing_sector
    clr.w d0                                   ; Clear the error code
    movem.l (sp)+, d1-d7/a0-a6          ; Recover the address of the buffer
    and.l #$0000FFFF,d2                 ; limit the size of the sectors to 65535
    add.l d2, a4                        ; Move the address to the next sector
    rts
_error_writing_sector:
    movem.l (sp)+, d1-d7/a0-a6               ; Recover the address of the buffer
    dbf d7, _write_sector_to_sidecart_once   ; Retry the write operation
    rts




; Shared functions included at the end of the file
; Don't forget to include the macros for the shared functions at the top of file
    include "inc/sidecart_functions.s"


floppy_emulator_msg:
        dc.b	"SidecarTridge Multi-device",$d,$a
        dc.b    "Floppy Emulator - "
        
version:
        dc.b    "v"
        dc.b    VERSION_MAJOR
        dc.b    "."
        dc.b    VERSION_MINOR
        dc.b    "."
        dc.b    VERSION_PATCH
        dc.b    $d,$a

spacing:
        dc.b    $d,$a,0

cr:
        dc.b    $d,$a,27,"K", 0

loading_image_msg:
        dc.b	"[..] Initializing. Press key to toogle.", $d, $a, $a, 0

error_sidecart_comm_msg:
        dc.b	$d, 27, "KCouldn't initialize. Firmware not ready",$d,$a,0

xbios_params_msg:
        dc.b	27, "K[X]BIOS trap is ", 0

boot_params_msg:
        dc.b	27, "K[B]oot sector is ", 0

memory_buff_msg:
        dc.b	27, "KBuffer [M]emory type: ", 0

memory_buff_stack_msg:
        dc.b	"HEAP",$d,$a,0

memory_buff_dskbuf_msg:
        dc.b	"_DSKBUF",$d,$a,0

boot_wait_msg:
        dc.b	$a, 27, "KConfiguring... Please wait...",$d,$a,$a,0

boot_ready_msg: 
        dc.b    $a, 27, "KEmulator ready.",$d,$a,$a,0

on_msg:
        dc.b	"ON ",$d,$a,0

off_msg:
        dc.b	"OFF",$d,$a,0

boot_countdown_msg:
        dc.b	27, "KPress [ESC] to boot now, or wait ", 0

boot_countdown_secs_msg:
        dc.b	"s.", 0

boot_up_msg:
        dc.b    27, "A",27, "A", 27, "A",27, "A", 27, "A", 27, "A", 27, "A", 27, "A", 27, "A",  13, 0
up_msg:
        dc.b	 27, "A", 0

show_ok_msg:
        dc.b	27, "K[OK] Initialized",$d,$a,0

detect_hardware_msg:
        dc.b	$d, "[..] Detecting hardware...", 0

error_detect_hardware_msg:
        dc.b	$d, "[KO] Error detecting hardware",$d,$a,0
 
save_vectors_msg:
        dc.b	"[..] Saving the old vectors...", 0

error_save_vectors_msg:
        dc.b	$d, "[KO] Error saving the old vectors",$d,$a,0

stack_buffer_msg:
        dc.b	"[..] Using stack as temp buffer...",0

dskbuf_buffer_msg:
        dc.b	"[..] Using _dskbuf as temp buffer...",0

boot_disk_msg:
        dc.b	"[..] Booting emulated disk...",$d,$a,0

web_ip_msg:
        dc.b	27, "KConnect to http://",0

web_host_msg:
        dc.b	" or",$d, $a, 27, "Khttp://",0

web_drive_msg:
        dc.b	" to manage drives.",0

backwards_msg:
        dc.b    $8, $8,0

ok_msg:
        dc.b	$d, "[OK]",$d,$a,0

countdown:
	dc.b "5",8,0
	dc.b "4",8,0
	dc.b "3",8,0
	dc.b "2",8,0
	dc.b "1",8,0
	dc.b "0",8,0

        even
rom_function_end:
; SidecarT Floppy Disk Drive (FDD) Emulator
; (C) 2023 by Diego Parrilla
; License: GPL v3

; Emulate a Floppy Disk Drive (FDD) from the SidecarT

; Bootstrap the code in ASM

    XREF    rom_function

    XDEF    do_transfer_ramdisk
    XDEF    setBPB
    XDEF    read_buff_msg_set_bpb
    XDEF    read_buff_msg_end_xbios_rw
    XDEF    read_buff_msg_end_gemdos_rw
    XDEF    read_buff_msg_dest_end
    XDEF    read_buff_msg_source_end
    XDEF    read_buff_msg_dest
    XDEF    read_buff_msg_source
    XDEF    read_buff_msg_bytes
    XDEF    read_buff_msg_write
    XDEF    read_buff_msg_read
    XDEF    read_buff_msg_logic_sector
    XDEF    read_buff_msg_sector_size
    XDEF    read_buff_msg_sector
    XDEF    nf_has_flag
    XDEF    nf_stderr_crlf
    XDEF    nf_stderr_id
    XDEF    nf_hexnum_buff
    XDEF    nf_debugger_id


    XDEF    random_token
    XDEF    random_token_seed
    XDEF    sidecart_read_buf
    XDEF    BPB_data
    XDEF    old_XBIOS_trap
    XDEF    old_hdv_bpb
    XDEF    old_hdv_rw
    XDEF    old_hdv_mediach

    XDEF    disk_number
    XDEF    secptrack
    XDEF    secpcyl

  	include inc/tos.s
    include inc/debug.s

	section code

main:

    move.l  4(sp),a0        ; Pointer to BASEPAGE

    move.l #mystack + (end_stack - mystack - 4),sp     ; Set the stack

    move.l    #$100,d0      ; Length of basepage
    add.l     $c(a0),d0     ; Length of the TEXT segment
    add.l     $14(a0),d0    ; Length of the DATA segment
    add.l     $1c(a0),d0    ; Length of the BSS segment

    move.l    d0, reserved_mem      ; Save the length to use in Ptermres()
    move.l    d0,-(sp)      ; Return to the stack
    move.l    a0,-(sp)      ; Basepage address to stack
    clr.w   -(sp)           ; Fill parameter
    move.w  #$4A,-(sp)      ; Mshrink
    trap    #1              ; Call GEMDOS 
    lea     $c(sp), sp      ; Correct stack

    ifeq _DEBUG
    tst.l   d0              ; Check for errors
    bne     prg_memory_error; Exit if error
    endif

	; Supervisor mode
	EnterSuper

    ifne _DEBUG
	; Save the ID for triggering stderr
	pea     nf_stderr         ;dc.b "NF_STDERR",
	clr.l   -(a7)                   ;dummy because of C API
	dc.w    $7300                   ;query natfeats
	move.l  d0,nf_stderr_id ;save ID. ID is 0 if not supported.
	or.l d0, nf_has_flag
	addq.l  #8,a7

	; Save the ID for triggering debugger
	pea     nf_debugger_name        ;dc.b "NF_DEBUGGER",0
	clr.l   -(a7)                   ;dummy because of C API
	dc.w    $7300                   ;query natfeats
	move.l  d0,nf_debugger_id ;save ID. ID is 0 if not supported.
	or.l d0, nf_has_flag
	addq.l  #8,a7
    endif

    ; Load the image file only when testing in Hatari with natfeats
    ifne _DEBUG
    bsr.s load_image
    endif

    bsr rom_function

    ; Exit supervisor mode
    ExitSuper

    ifeq _DEBUG
        move.l reserved_mem, d0             ; Get length of memory to keep
    else
        move.l (end_main-main), d0    ; Get length of memory to keep as the size of the executable
    endif
    move.w #0, -(sp)                    ; Return value of the program
    move.l  d0,-(sp)                    ; Length of memory to keep
    move.w  #$31,-(sp)                  ; Ptermres
    trap    #1                          ; Call GEMDOS


; Load binary file and reserves memory for it
load_image:
; Open the file
	clr.w -(sp)                     ; Open mode S_READONLY
	pea image_file_name             ; Filename
	move.w #$3d,-(sp)               ; GEMDOS function Fopen
	trap #$01                       ; Call GEMDOS
	addq.l #8,sp                    ; Correct stack
	tst.l d0                        ; Check for errors
	bmi load_error                ; Load error

    move.w d0,image_handle          ; Store the file handle

; Get image_length with fseek
    move.w #2,-(sp)                 ; SEEK_END mode
    move.w d0,-(sp)                 ; handle
    clr.l -(sp)                     ; offset
    move.w #$42,-(sp)               ; GEMDOS function Fseek
    trap #1                         ; Call GEMDOS
    lea 10(sp), sp                  ; Correct stack

    move.l d0,image_length          ; Store the image length  

; It works in non debug mode, I don't know why yet...
    ifeq    _DEBUG
; Reserve the memory for the image
        move.l d0, -(sp)                ; Number of bytes to reserve
        move.w #$48, -(sp)              ; GEMDOS function Malloc
        trap    #1                      ; Call GEMDOS
        addq.l  #6, sp                  ; Correct stack
        tst.l d0                
        beq   memory_error              ; Memory error
        move.l d0, sidecart_read_buf                ; Save start of the memory for the image
    else
; In debug mode, we assume the 4MB of RAM in the system and we place the image at the top of the memory
        move.l  $42e.w, d0   ; Load the long-word value at address $42E into data register D0
        sub.l #(1024 * 1024 + 32768), d0      ; Subtract 32768 from the value in D0
        move.l d0, sidecart_read_buf           ; Save start of the memory for the image after the executable
    endif

; Go back at the beginning of the file
    clr.w -(sp)                     ; SEEK_SET mode
    move.w image_handle,-(sp)       ; File handle
    clr.l -(sp)                     ; offset
    move.w #$42,-(sp)               ; GEMDOS function Fseek
    trap #1                         ; Call GEMDOS
    lea 10(sp), sp                  ; Correct stack

; Now read the file content into memory
    move.l  sidecart_read_buf,-(sp)                ; Destination buffer
    move.l  image_length,-(sp)      ; Number of bytes to read
    move.w  image_handle,-(sp)      ; File handle
    move.w  #$3f,-(sp)              ; GEMDOS function Fread
    trap    #1                      ; Call GEMDOS
    lea     12(sp),sp               ; Correct stack
    tst.l   d0                      ; Check for errors
    bmi     load_error              ; Load error

; Close the file
    move.w image_handle,-(sp)       ; File handle
    move.w  #$3E,-(sp)              ; GEMDOS function Fclose
    trap    #1                      ; Call GEMDOS
    addq.l  #4,sp                   ; Correct stack

	pea image_file_name
	move	#9,-(sp)	; Cconws
	trap	#1
	addq.l	#6,sp

	pea image_load_success
	move	#9,-(sp)	; Cconws
	trap	#1
	addq.l	#6,sp

	move.w	#7,-(sp)	; Crawcin
	trap	#1
	addq.l	#2,sp
    
    rts

; Perform the transference of the data from/to the emulated disk to the computer
; Input registers;
;  d0: logical sector number to start the transfer
;  d1: number of sectors to read/write
;  d2: sector size in bytes
;  d4: rwflag for read/write
;  a0: buffer address in the computer memory
; Output registers:
;  none
do_transfer_ramdisk:
    move.l sidecart_read_buf,a1             ; Disk start.  (bootsector)
    clr.l  d7
    move.w d0, d7               ; Calculate the address of the sector to read/write
    mulu d2, d7                 ; Sector size x logical sector number to get the mem address
    add.l d7,a1                 ; plus relative address in RAM

    move.l d1, d7
    nf_stderr_lit read_buff_msg_sector, 4

    move.w d2, d7
    nf_stderr_lit read_buff_msg_sector_size, 4

    move.l d0, d7
    nf_stderr_lit read_buff_msg_logic_sector, 4

    tst.w d4                    ; test rwflag
    bne.s write_buff            ; if not, write
read_buff:
    exg a1,a0                   ; read, so swap source and destination
    nf_str read_buff_msg_read

    ifne _DEBUG
    bra.s write_buff_2
    endif
write_buff:  

    ifne _DEBUG
    nf_str read_buff_msg_write
write_buff_2:
    endif
    move.l a0, d7
    nf_stderr_lit read_buff_msg_source, 8

    move.l a1, d7
    nf_stderr_lit read_buff_msg_dest, 8

    move.l d1, d7
    mulu d2, d7
    nf_stderr_lit read_buff_msg_bytes, 8

    subq.w #1,d1                ; one less
    subq.w #1,d2                ; one less
_copy_sectors:
    move.w d2, d7               ; Sector size to scratch register d7
_copy_sector_content:            ; copy the sector bytes
    move.b (a0)+,(a1)+
    dbf d7, _copy_sector_content
    dbf d1, _copy_sectors

    move.l a0, d7
    nf_stderr_lit read_buff_msg_source_end, 8

    move.l a1, d7
    nf_stderr_lit read_buff_msg_dest_end, 8

    rts

prg_memory_error:
    pea prg_memory_error_msg
    bra.s exit_failure
memory_error:
    pea memory_error_msg
    bra.s exit_failure
load_error:
	pea load_error_msg
exit_failure:
	move	#9,-(sp)	; Cconws
	trap	#1
	addq.l	#6,sp

	move.w	#7,-(sp)	; Crawcin
	trap	#1
	addq.l	#2,sp

    move.w  d0,-(sp)                ; Error code
    move.w  #$4C,-(sp)              ; GEMDOS function Pterm
    trap    #1                      ; Call GEMDOS
    rts

; Set BPB values according to boot sector:& parameter fields
; For testing purposes this is also implemented in the ATARI ST side
; But it should be implemented in the RP2040 side and prepare all the
; data structures ready to be consumed Read Only by the ATARI ST side
setBPB:
        move.l  sidecart_read_buf, a0
        lea     BPB_data+2, a1
        clr.l   d0
        move.b  13(a0),d0   ; Sect/clust
        move.w  d0,d3
        move.w  d0,(a1)+    ; 1: clsiz:   Cluster size in sectors   
        lsl.w #2,d0
        lsl.w #7,d0         ; x512
        move.w d0,(a1)+     ; 2: clsizb:  Cluster size in bytes
        move.b 18(a0),d0    ; Track count
        lsl.w #8,d0
        move.b 17(a0),d0    ; ?????
        lsr.w #4,d0         ; /16
        move.w d0,(a1)+     ; 3: rdlen:   Root Directory length in sectors
        move.w d0,d1
        clr.l d0
        move.b 22(a0),d0
        move.w d0,(a1)+     ; 4: fsiz:    FAT size in sectors
        move.w d0,d2
        addq.w #1,d0
        move.w d0,(a1)+     ; 5: fatrec:  Sector number of second FAT

        add.w d1,d0
        add.w d0,d2
        move.w d2,(a1)+     ; 6: datrec:  Sector number of first data cluster


        move.b 20(a0),d0
        lsl.w #8,d0
        move.b 19(a0),d0
        sub.w d2,d0
        divu d3,d0
        move.w d0,(a1)+     ; 7: numcl:   Number of data clusters on the disk

        addq.l #4,a1        ; skip FAT type flag and track count
        clr.l d0
        move.b 26(a0),d0
        move.w d0,(a1)+     ; Side count

        clr.l d1
        move.b 24(a0),d1    ; sec/track
        move.w d1,d2

        mulu d0,d1          ; sec/cyl
        move.w d1,(a1)+     ; sect/cylinder
        move.w d2,(a1)+     ; sect/track
        clr.w (a1)          ; 

        nf_str read_buff_msg_set_bpb
        nf_crlf

        clr.w d0            ; Success
        
        rts


; ---- data section

data:
        even
random_token: dc.l $12345678   ; Random token to check if the command returns a value
random_token_seed: dc.l $12345678   ; Random token seed passed to the command

        even
BPB_data:
        dc.w 512      ; 0: recsize: Sector size in bytes  
        dc.w 2        ; 1: clsiz:   Cluster size in sectors
        dc.w 1024     ; 2: clsizb:  Cluster size in bytes
        dc.w 8        ; 3: rdlen:   Root Directory length in sectors
        dc.w 6        ; 4: fsiz:    FAT size in sectors
        dc.w 7        ; 5: fatrec:  Sector number of second FAT
        dc.w 21       ; 6: datrec:  Sector number of first data cluster
        dc.w 1015     ; 7: numcl:   Number of data clusters on the disk
        dc.w 0        ; 8: bflags:  Magic flags

; Keep this structure as it is to implement it in the RP2040 side
;unsigned int BpbData[] = {
;     512  /*         0: recsize: Sector size in bytes                */
;    ,2    /* Was 1   1: clsiz:   Cluster size in sectors             */
;    ,1024 /* Was 512 2: clsizb:  Cluster size in bytes               */
;    ,8    /* Was 4   3: rdlen:   Root Directory length in sectors    */
;    ,6    /*         4: fsiz:    FAT size in sectors                 */
;    ,7    /*         5: fatrec:  Sector number of second FAT         */
;    ,21   /* Was 17  6: datrec:  Sector number of first data cluster */
;    ,1015 /* Was2030 7: numcl:   Number of data clusters on the disk */
;    ,0    /*         8: bflags:  Magic flags                         */
;};


trackcnt:
        dc.w 0
sidecnt:
        dc.w 0
secpcyl:
        dc.w 0
secptrack:
        dc.w 0
        dc.w 0
        dc.w 0
        dc.w 0
disk_number:
        dc.w 0  ;drive A!

; Debugging
nf_has_flag         dc.l 0              ; Natfeats flag. 0 = not installed, Not zero = installed
nf_stderr           dc.b "NF_STDERR",0  ; Natfeats stderr device name
nf_debugger_name    dc.b "NF_DEBUGGER",0; Natfeats debugger device name
nf_stderr_crlf      dc.b 13,10,0        ; Carriage return + line feed

        even
; Messages
read_buff_msg_sector:       dc.b "SECTOR: $",0
read_buff_msg_sector_size:  dc.b " / S_SIZE: $",0
read_buff_msg_logic_sector: dc.b " / L_SECTOR: $",0
read_buff_msg_read:         dc.b " / READ ",0
read_buff_msg_write:        dc.b " / WRITE ",0
read_buff_msg_source:       dc.b " / SRC: $",0
read_buff_msg_dest:         dc.b " / DST: $",0
read_buff_msg_source_end:   dc.b " / SRC_END: $",0
read_buff_msg_dest_end:     dc.b " / DST_END: $",0
read_buff_msg_bytes:        dc.b " / BYTES: $",0
read_buff_msg_end_gemdos_rw:dc.b " / GEMDOS",0
read_buff_msg_end_xbios_rw: dc.b " / XBIOS",0
read_buff_msg_media_change: dc.b "MEDIA CHANGED!",0
read_buff_msg_set_bpb:      dc.b "SET BPB",0

floppy_emulator_msg:
          	dc.b	"SidecarT Floppy Emulator",$d,$a,0

load_error_msg:
            dc.b "Error loading the image file",$d,$a,0

memory_error_msg:
            dc.b "Error reserving memory for the image",$d,$a,0

image_load_success:
            dc.b " loaded successfully",$d,$a,0

prg_memory_error_msg:
            dc.b "Error reserving memory for the program",$d,$a,0


image_file_name:
            dc.b "XENON2_3.st",0
;image_file:
;            even
;          	incbin images/XENON2.ST
;           incbin images/RAMPAGE.ST
;            incbin images/GFA.st

; ---- BSS section
bss:
            even
old_hdv_bpb:        dc.l 0
old_hdv_rw:         dc.l 0
old_hdv_mediach:    dc.l 0
old_XBIOS_trap:     dc.l 0

sidecart_read_buf:              ds.l 1
image_length:       ds.l 1
image_handle:       ds.w 1

; Debugging
nf_stderr_id:       ds.l  1
nf_debugger_id:     ds.l  1
nf_hexnum_buff:     ds.b  10    ; 8 hex digits + 0 + blank

            even
changed:            ds.w    1
reserved_mem:       ds.l    1
savestack:          ds.l    1


mystack:
            ds.l    2000          ; 8000 bytes stack
end_stack:  

end_main:

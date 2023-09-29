; SidecarT Floppy emulator loader
; (C) 2023 by Diego Parrilla
; License: GPL v3

; Some technical info about the header format https://www.atari-forum.com/viewtopic.php?t=14086

; $FA0000 - CA_MAGIC. Magic number, always $abcdef42 for ROM cartridge. There is a special magic number for testing: $fa52235f.
; $FA0004 - CA_NEXT. Address of next program in cartridge, or 0 if no more.
; $FA0008 - CA_INIT. Address of optional init. routine. See below for details.
; $FA000C - CA_RUN. Address of program start. All optional inits are done before. This is required only if program runs under GEMDOS.
; $FA0010 - CA_TIME. File's time stamp. In GEMDOS format.
; $FA0012 - CA_DATE. File's date stamp. In GEMDOS format.
; $FA0014 - CA_SIZE. Lenght of app. in bytes. Not really used.
; $FA0018 - CA_NAME. DOS/TOS filename 8.3 format. Terminated with 0 .

; CA_INIT holds address of optional init. routine. Bits 24-31 aren't used for addressing, and ensure in which moment by system init prg. will be initialized and/or started. Bits have following meanings, 1 means execution:
; bit 24: Init. or start of cartridge SW after succesfull HW init. System variables and vectors are set, screen is set, Interrupts are disabled - level 7.
; bit 25: As by bit 24, but right after enabling interrupts on level 3. Before GEMDOS init.
; bit 26: System init is done until setting screen resolution. Otherwise as bit 24.
; bit 27: After GEMDOS init. Before booting from disks.
; bit 28: -
; bit 29: Program is desktop accessory - ACC .
; bit 30: TOS application .
; bit 31: TTP

;Rom cartridge

	org $FA0000

	dc.l $abcdef42 					; magic number
first:
	dc.l $0
	dc.l $08000000 + floppy_emulator		; After GEMDOS init. Before booting from disks.
	dc.l floppy_emulator
	dc.w GEMDOS_TIME 				;time
	dc.w GEMDOS_DATE 				;date
	dc.l floppy_emulator_end - floppy_emulator
	dc.b "EMUL",0
	even

floppy_emulator:
; Add a slight delay before reading the keyboard
	moveq	#63, d7
.ddell:
    move.w 	#37,-(sp)
    trap 	#14
    addq.l 	#2,sp
    dbf 	d7,.ddell

; Now check the left shift key. If pressed, exit.
    pea 	$BFFFF
    trap 	#13
    addq.l 	#4,sp

    btst 	#1,d0
    beq 	$FA0040
	; Not pressed. Bye bye!
	rts


floppy_emulator_end:
rom_function_start:


;**********************************************************
;* Serial Bootloader for Atmel AVR Controller             *
;*                                                        *
;* ATmegaBOOT.S                                           *
;*                                                        *
;* Version: 0.02                                          *
;*                                                        *
;* Target = Atmel ATMega8,88,168,328,..                   *
;*                                                        *
;* Copyright (c) 2003, Jason P. Kyle                      *
;* Hacked by DojoCorp - ZGZ - MMX - IVR                   *
;* Hacked by David A. Mellis                              *
;* 256 word ASM shrink-port by A-Z-E A. Ziemer            *
;*                                                        *
;* This program is free software; you can redistribute it *
;* and/or modify it under the terms of the GNU General    *
;* Public License as published by the Free Software       *
;* Foundation; either version 2 of the License, or        *
;* (at your option) any later version.                    *
;*                                                        *
;* This program is distributed in the hope that it will   *
;* be useful, but WITHOUT ANY WARRANTY; without even the  *
;* implied warranty of MERCHANTABILITY or FITNESS FOR A   *
;* PARTICULAR PURPOSE.  See the GNU General Public        *
;* License for more details.                              *
;*                                                        *
;* You should have received a copy of the GNU General     *
;* Public License along with this program; if not, write  *
;* to the Free Software Foundation, Inc.,                 *
;* 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA *
;*                                                        *
;* Licence can be viewed at                               *
;* http://www.fsf.org/licenses/gpl.txt                    *
;**********************************************************
;major hacking by R.Wiersma, use at on own risk, nothing is documented ;P
					.list

#define __SFR_OFFSET		0			/* !!!!!!!!!! */
#include <avr/io.h>						/* for CPU I/O register definitions and vectors */

; Features & Settings
#ifndef F_CPU
# define F_CPU				8000000
#endif

;#ifndef BAUD_RATE
;# define BAUD_RATE			9600
;#endif


#define	UART_TXDDR DDRA
#define	UART_TXPORT PORTA
#define UART_TXPINNumber  PA1			
		
#define	UART_RXDDR  DDRA
#define	UART_RXPORT PORTA
#define	UART_RXPIN  PINA
#define	UART_RXPINNumber PA2			

;***** Global register variables

;bit counter
;#define	bitcnt r16

;temporary storage register
;#define	temp r17




;#ifndef BOOTUART
;# define BOOTUART			1	/* for MCUs with more than one async UART */
;#endif

;#ifndef USE_LED
#define USE_LED			1	/* 1: turn LED on when entering the bootloader */
;#endif

;#if (USE_LED==1)
;/* Default to the onboard LED connected to pin PB5 */
;# ifndef LED_NO
#define LED_NO			PINB0
;# endif
#define LED_DDR			DDRB
;# endif
;# ifndef LED_PORT
#define LED_PORT			PORTB
;# endif
;# ifndef LED_PIN
#define LED_PIN			PINB
;# endif
;#endif

;#ifndef BL_CHECK
# define BL_CHECK			1	/* 1: check for bootloader pin being low on reset -> bootloader if yes */
;#endif

;#if (BL_CHECK==1)
;/* Default to pin PB3 (MOSI) */
;# ifndef BL_NO
#  define BL_NO				PINA6
;# endif
;# ifndef BL_DDR
#  define BL_DDR			DDRA
;# endif
;# ifndef BL_PORT
#  define BL_PORT			PORTA
;# endif
;# ifndef BL_PIN
#  define BL_PIN			PINA
;# endif
;#endif

#ifndef BL_TIMEOUT
# define BL_TIMEOUT			5	/* bootloader timeout in seconds */
#endif

;#ifndef CODE_CHECK
;# if defined(__AVR_ATmega48__)
;#  define CODE_CHECK		0	/* 1: check if CODE location 0x0000 is 0xFF -> bootloader if yes */
;# else
;#  define CODE_CHECK		1	/* 1: check if CODE location 0x0000 is 0xFF -> bootloader if yes */
;# endif
;#elif defined(__AVR_ATmega48__) && (CODE_CHECK==1)
;# warning CODE_CHECK with ATmega48 not possible!
;# undef CODE_CHECK
;# define CODE_CHECK		0	/* 1: check if CODE location 0x0000 is 0xFF -> bootloader if yes */
;#endif

#ifndef WDT_CHECK
# define WDT_CHECK			1	/* 1: do NOT enter bootloader if a WDT or BOD reset was detected */
#endif

#ifndef RAMSTART
# define RAMSTART			(RAMEND-0x1FF)	/* this expects at least 512 bytes of SRAM, which all used ATmegas have */
#endif

#ifndef MAX_TIME_COUNT
# if BL_TIMEOUT > (0x1000000/(F_CPU>>3))
#  warning	"BL_TIMEOUT maximized..."
#  define	MAX_TIME_COUNT	0x1000000
# elif BL_TIMEOUT > 0
#  define	MAX_TIME_COUNT	(BL_TIMEOUT*(F_CPU>>3))
# endif
#endif



;#if !defined (MAX_TIME_COUNT) && (BL_CHECK==0) && (CODE_CHECK==0) && (WDT_CHECK==0)
;							// If neither CODE_CHECK nor BL_CHECK is defined, we always enter the bootloader
;							// so we better should have a timeout... ! We use 5s here
;# warning	"setting BL_TIMEOUT to 5s"
;# define	MAX_TIME_COUNT	(5*(F_CPU>>3))
;#endif

; SW_MAJOR and MINOR needs to be updated from time to time to avoid warning message from AVR Studio
#define HW_VER				0x02
#define SW_MAJOR			0x01
#define SW_MINOR			0x12

; Register usage defines
#define user_l				r12
#define user_h				r13
#define temp2				r16
#define to_cnt0				r17
#define to_cnt1				r18
#define to_cnt2				r19
#define accu				r20			/* main parameter and return value register */
#define mem_area			r21
#define tmp1				r22
#define tmp2				r23
#define len_l				r24
#define len_h				r25
#define count_l				r26			/* X */
#define count_h				r27
#define buf_l				r28			/* Y */
#define buf_h				r29
#define addr_l				r30			/* Z */
#define addr_h				r31
#define bitcnt				r32

					.text

;attiny#if defined(__AVR_ATmega48__)
					.org	0x0000

					rjmp	startup

					.org	0x1DE0
;#endif

startup:
;#if (CODE_CHECK==1)
;					ldi		addr_l, 0x00
;					ldi		addr_h, 0x00
;					lpm		accu, Z						; read FLASH address 0x0000
;					cpi		accu, 0xFF					;
;					breq	_bootloader					; is 0xFF (erased) -> bootloader
;#endif
#if (BL_CHECK==1)
					sbi		BL_PORT, BL_NO				; activate the pullup on BL pin, DDR bit is already clear (input) after RESET
					sbis	BL_PIN, BL_NO
					rjmp	_bootloader					; tied to ground -> bootloader
#endif
#if (WDT_CHECK==1)
;attiny#if defined(__AVR_ATmega128__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega48__) || defined(__AVR_ATmega88__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
					in		accu, MCUSR
;#else
;					in		accu, MCUCSR
;#endif
					andi	accu, (1<<WDRF) | (1<<BORF)	; was it either WDT or BOR reset ?
					brne	_bootloader					; no -> bootloader
#endif
					; if none is defined always enter the bootloader
;#if (CODE_CHECK==1) || (BL_CHECK==1) || (WDT_CHECK==1)
;					;rjmp	reboot						; start user code
;#endif

_bootloader:		ldi		buf_l, lo8( RAMEND )		; set up STACK pointer to end of RAM
					ldi		buf_h, hi8( RAMEND )
					out		SPL, buf_l
					out		SPH, buf_h
#if (USE_LED==1)
					sbi		LED_DDR, LED_NO				; LED pin as output
					sbi		LED_PORT, LED_NO			; LED on
#endif

					; initialize UART
sbi _SFR_IO_ADDR(UART_TXDDR),UART_TXPINNumber
sbi _SFR_IO_ADDR(UART_TXPORT),UART_TXPINNumber
cbi _SFR_IO_ADDR(UART_RXDDR),UART_RXPINNumber
		
;attiny#if defined(__AVR_ATmega48__) || defined(__AVR_ATmega88__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
;# ifdef BOOTUART
;#  undef BOOTUART
;# endif
;# define BOOTUART	1
;					cbi		DDRD, PIND0
;					sbi		PORTD, PIND0				; pull-up on pin D0 (RX) - supress noise that prevents the bootloader from timing out
;#endif

;#if defined(__AVR_ATmega128__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega48__) || defined(__AVR_ATmega88__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
;# if (BOOTUART==1)
;#  if (BAUD_RATE > 38400)
;#   define UBRVAL		(((F_CPU/BAUD_RATE)/8)-1)
;					ldi		accu, (1<<U2X0)
;#  else
;#   define UBRVAL		(((F_CPU/BAUD_RATE)/16)-1)
;					ldi		accu, 0x00
;#  endif
;					sts		UCSR0A, accu				; Single/Double speed mode

;					ldi		accu, hi8( UBRVAL )			; baud rate
;					sts		UBRR0H, accu

;					ldi		accu, lo8( UBRVAL )
;					sts		UBRR0L, accu

;					ldi		accu, (1<<UCSZ01) | (1<<UCSZ00)	; set to 8N1
;					sts		UCSR0C, accu

;					ldi		accu, (1<<RXEN0) | (1<<TXEN0)	; enable Rx & Tx
;					sts		UCSR0B, accu
;# elif (BOOTUART==2)
;#  if (BAUD_RATE > 38400)
;#   define UBRVAL		(((F_CPU/BAUD_RATE)/8)-1)
;					ldi		accu, (1<<U2X1)
;#  else
;#   define UBRVAL		(((F_CPU/BAUD_RATE)/16)-1)
;					ldi		accu, 0x00
;#  endif
;					sts		UCSR1A, accu				; Single/Double speed mode

;					ldi		accu, hi8( UBRVAL )			; baud rate
;					sts		UBRR1H, accu

;					ldi		accu, lo8( UBRVAL )
;					sts		UBRR1L, accu

;					ldi		accu, (1<<UCSZ11) | (1<<UCSZ10)	; set to 8N1
;					sts		UCSR1C, accu

;					ldi		accu, (1<<RXEN1) | (1<<TXEN1)	; enable Rx & Tx
;					sts		UCSR1B, accu
;# endif

;#else				/* all others */
;# if (BAUD_RATE > 38400)
;#  define UBRVAL		(((F_CPU/BAUD_RATE)/8)-1)
;					ldi		accu, (1<<U2X)
;# else
;#  define UBRVAL		(((F_CPU/BAUD_RATE)/16)-1)
;					ldi		accu, 0x00
;# endif
;					out		UCSRA, accu					; Single/Double speed mode

;					ldi		accu, hi8( UBRVAL )			; baud rate
;					out		UBRRH, accu

;					ldi		accu, lo8( UBRVAL )
;					out		UBRRL, accu

;					ldi		accu, (1<<RXEN) | (1<<TXEN)	; enable Rx & Tx
;					out		UCSRB, accu
					
;					ldi		accu, (1<<URSEL) | (1<<UCSZ1) | (1<<UCSZ0)	; set to 8N1
;					out		UCSRC, accu
;#endif

;*******************************************************
_main_loop:			rcall	getch						; get character from UART

;-------------------------------------------------------
_chk_1:				cpi		accu, '1'					; Request programmer ID
					brne	_chk_u

                    ldi		addr_l, lo8(ISP_ID)			; buffer address
					ldi		tmp1, 7						; string length

strout:				rcall	resp_start
					brne	_main_loop

                    ldi		addr_h, hi8(ISP_ID)			; all fixed string MUST reside in the same 256 byte code page!

_str_1:				lpm		accu, Z+
					rcall	putch
					dec		tmp1
					brne	_str_1

					rjmp	resp_end

ISP_ID:				.byte	'A','V','R',' ','I','S','P'
SIG_ID:				.byte	SIGNATURE_0, SIGNATURE_1, SIGNATURE_2

;-------------------------------------------------------
_chk_u:				cpi		accu, 'u'					; Get device signature bytes
					brne	_chk_at

                    ldi		addr_l, lo8(SIG_ID)			; buffer address
					ldi		tmp1, 3						; string length

					rjmp	strout

;-------------------------------------------------------
empty_response:		rcall	getch						; getch() checks for ' '
					brne	_main_loop

empty_resp_nochk:	ldi		accu, 0x14
resp_last_end:		rcall	putch

resp_end:			ldi		accu, 0x10
					rcall	putch

					rjmp	_main_loop

;-------------------------------------------------------
resp_start:			rcall	getch						; getch() checks for ' '
					brne	_resp_st_exit

					ldi		accu, 0x14
					rcall	putch

					eor		accu, accu
_resp_st_exit:		ret									; returns with Z flag set on success

;-------------------------------------------------------
_chk_at:			cpi		accu, '@'					; AVR ISP/STK500 commands - DON'T CARE, so default response
					brne	_chk_v

					rcall	getch
					cpi		accu, 0x86
					brcs	_chk_at_end					; if next byte > 0x85 
					rcall	getch						; get another byte

_chk_at_end:		rjmp	empty_response

;-------------------------------------------------------
_chk_v:				cpi		accu, 'v'					; Read oscillator calibration byte
					breq	byte_resp_00

;-------------------------------------------------------
_chk_A:				cpi		accu, 'A'					; AVR ISP/STK500 requests
					brne	_chk_B

					rcall	getch
					cpi		accu, 0x80
					brne	_chk_A_81
					ldi		tmp2, HW_VER				; Hardware version
					rjmp	byte_response

_chk_A_81:			cpi		accu, 0x81					; Software major version
					brne	_chk_A_82
					ldi		tmp2, SW_MAJOR
					rjmp	byte_response

_chk_A_82:			cpi		accu, 0x82					; Software minor version
					brne	byte_resp_00
					ldi		tmp2, SW_MINOR
					rjmp	byte_response

byte_resp_00:		ldi		tmp2, 0x00					; Covers various responses we don't care about

byte_response:		rcall	resp_start
					brne	_main_loop

					mov		accu, tmp2
					rjmp	resp_last_end

;-------------------------------------------------------
_chk_B:				cpi		accu, 'B'					; Device Parameters - DON'T CARE
					brne	_chk_E

					ldi		tmp2, 0x14					; eat next 20 chars

_null_1:			rcall	getch
					dec		tmp2
					brne	_null_1
					
					rjmp	empty_response

;-------------------------------------------------------
_chk_E:				cpi		accu, 'E'					; Parallel programming stuff - DON'T CARE
					brne	_chk_U

					ldi		tmp2, 0x05					; eat next 5 chars
					rjmp	_null_1

;-------------------------------------------------------
_chk_U:				cpi		accu, 'U'					; Set address, little endian. EEPROM in bytes, FLASH in words
					brne	_chk_V

					rcall	getch
					mov		addr_l, accu				; low byte -> Zl

					rcall	getch
					mov		addr_h, accu				; high byte -> Zh

					rjmp	empty_response

;-------------------------------------------------------
_chk_V:				cpi		accu, 'V'					; Universal SPI programming commands, used for fuses and lock bits - DISABLED
					brne	_chk_t

					rcall	getch
					mov		tmp2, accu
					
					rcall	getch
					
					rcall	getch
					mov		tmp1, accu
					
					rcall	getch

					cpi		tmp2, '0'
					breq	_chk_V_0
					rjmp	byte_resp_00

_chk_V_0:			cpi		tmp1, 0x00
					brne	_chk_V_1
					ldi		tmp2, SIGNATURE_0
					rjmp	byte_response

_chk_V_1:			cpi		tmp1, 0x01
					brne	_chk_V_def
					ldi		tmp2, SIGNATURE_1
					rjmp	byte_response

_chk_V_def:			ldi		tmp2, SIGNATURE_2
					rjmp	byte_response

;-------------------------------------------------------
_chk_t:				cpi		accu, 't'					; Read memory block mode, length is big endian.
					brne	_chk_d

					rcall	get_len_area				; length should better not be zero...

					rcall	resp_start
					brne	_d_abort

					rcall	wait_ee						; eventually wait

_t_loop:			cpi		mem_area, 'E'
					brne	_t_flash

					;---------------------
_t_eeprom:			rcall	set_ee_addr
					sbi		EECR, EERE					; enable EEPROM read
					in		accu, EEDR

					rjmp	_t_lp_com

					;---------------------
_t_flash:			lpm		accu, Z+

_t_lp_com:			rcall	put_n_dec
					brne	_t_loop

					rjmp	resp_end
	
;-------------------------------------------------------
_chk_d:				cpi		accu, 'd'					; Write memory, length is big endian and is in bytes
					breq	_chk_d_0

;					cpi		accu, '0'					; Is there anybody out there ?
;					cpi		accu, 'P'					; Enter programming mode
;					cpi		accu, 'Q'					; Leave programming mode
;					cpi		accu, 'R'					; Erase device - don't care as we will erase one page at a time anyway.
					rjmp	empty_response				; respond with empty packet to all other messages

;-------------------------------------------------------
_chk_d_0:			rcall	get_len_area				; length should better not be zero...

					; Store data in a temporary buffer, as we couldn't keep up with the
					; serial data stream whilst programming pages
_d_buf_loop:		rcall	getch
					st		Y+, accu
					rcall	dec_count
					brne	_d_buf_loop

					rcall	getch
					breq	_d_loop_action
_d_abort:			rjmp	_main_loop

					;-----------------------------------
_d_loop_action:		rcall	loop_setup					; reload X and Y

					cpi		mem_area, 'E'
					brne	_d_flash

					;-----------------------------------
					; Write to EEPROM one byte at a time
_d_eeprom:			rcall	set_ee_addr
					ld		accu, Y+
					out		EEDR, accu
;atyiny #if defined(__AVR_ATmega128__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega48__) || defined(__AVR_ATmega88__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
					sbi		EECR, EEMPE					; enable EEPROM write
					sbi		EECR, EEPE					; start write procedure
;#else
;					sbi		EECR, EEMWE					; enable EEPROM write
;					sbi		EECR, EEWE					; start write procedure
;#endif
					rcall	dec_count
					brne	_d_eeprom

_d_com_end:			rjmp	empty_resp_nochk

					;-----------------------------------
					; Write to FLASH one page at a time
_d_flash:			sbrc	len_l, 0					; X: Length of data to be written (in bytes) - Even up an odd number of bytes
					adiw	len_l, 0x01					; this eventually programs a dummy byte to the FLASH

					ldi		count_l, 0x00				; clear page_word_count

					; Main flash loop, repeat for number of words in block							 							 
length_loop:		rcall	wait_ee						; Eventually wait for any previous EEPROM writes to complete

					cpi		count_l, 0x00				; If page_word_count == 0 then erase page
					brne	no_page_erase

					ldi		accu, 0x03					; Erase page pointed to by Z
					rcall	do_spm
					
					ldi		accu, 0x11					; Re-enable RWW section
					rcall	do_spm

no_page_erase:		ld		r0, Y+						; Write next 2 bytes from 'buff' into page buffer
					ld		r1, Y+
					ldi		accu, 0x01					; Load r0,r1 into FLASH page buffer
					rcall	do_spm

					inc		count_l						; increment page_word_count
					cpi		count_l, (SPM_PAGESIZE/2)
					brlo	same_page					; Still same page in FLASH ?

write_page:			clr		count_l						; New page, write current one first
					
					ldi		accu, 0x05					; Write page pointed to by Z (addr_h/addr_l)
					rcall	do_spm
					
					ldi		accu, 0x11					; Re-enable RWW section
					rcall	do_spm

same_page:			adiw	addr_l, 2					; Next word in FLASH
					sbiw	len_l, 2					; length -= 2
					brne	length_loop					; Finished ?

final_write:		cpi		count_l, 0x00
					breq	_d_com_end
					adiw	len_l, 0x02					; length += 2, fool above check on length after short page write
					rjmp	write_page

;-------------------------------------------------------
do_spm:				lds		tmp1, 0x0057				; Wait for previous spm to complete (0x0057 = SPMCR)
					andi	tmp1, 0x01
					brne	do_spm
					sts		0x0057, accu
					spm
					ret

;-------------------------------------------------------
wait_ee:			
;attiny #if defined(__AVR_ATmega128__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega48__) || defined(__AVR_ATmega88__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
					sbic	EECR, EEPE					; wait while eeprom is not ready
;#else
;					sbic	EECR, EEWE					; wait while eeprom is not ready
;#endif
					rjmp	wait_ee
					ret

;-------------------------------------------------------
set_ee_addr:        rcall	wait_ee						; wait until eeprom is ready
					out		EEARH, addr_h				; emit address
					out		EEARL, addr_l
					adiw	addr_l, 0x01				; increment address
					ret

;-------------------------------------------------------
get_len_area:		rcall	getch
					mov		len_h, accu

					rcall	getch
					mov		len_l, accu

					rcall	getch
					mov		mem_area, accu				; is 'E' or 'F'

					cpi		mem_area, 'E'				; Address of FLASH location is in words.
					breq	loop_setup

					lsl		addr_l					   	; Address * 2 -> byte location
					rol		addr_h

loop_setup:			movw	count_l, len_l				; create a copy of length in X

					ldi		buf_l, lo8( RAMSTART )		; set up buffer address -> Y
					ldi		buf_h, hi8( RAMSTART )
					ret

;-------------------------------------------------------
put_n_dec:			rcall	putch
dec_count:			sbiw	count_l, 0x01
					mov		accu, count_l
					or		accu, count_h
					ret

;************************************************************************
putch:
	ldi	temp2,10	;put 10 bits (start+stop+payload
	com	accu		;Inverte everything
	sec			;Start bit

UART_putchar0:	
	brcc	UART_putchar1	;If carry set
	cbi	_SFR_IO_ADDR(UART_TXPORT),UART_TXPINNumber	;    send a '0'
	rjmp	UART_putchar2	;else	

UART_putchar1:	
	sbi	_SFR_IO_ADDR(UART_TXPORT),UART_TXPINNumber	;    send a '1'
	nop

UART_putchar2:	
	rcall UART_delay	;One bit delay
	rcall UART_delay
	lsr	accu		;Get next bit
	dec	temp2		;If not all bit sent
	brne	UART_putchar0	;   send next
				;else
	ret			;   return


;#if defined(__AVR_ATmega128__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega48__) || defined(__AVR_ATmega88__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
;# if (BOOTUART==1)
;					lds		tmp3, UCSR0A
;					sbrs	tmp3, UDRE0					; wait while transmitting
;# else
;					lds		tmp3, UCSR1A
;					sbrs	tmp3, UDRE1					; wait while transmitting
;# endif
;#else
;					sbis	UCSRA, UDRE					; wait while UCSRA.UDRE == 0
;#endif   			
;					rjmp	putch
;#if defined(__AVR_ATmega128__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega48__) || defined(__AVR_ATmega88__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
;# if (BOOTUART==1)
;					sts		UDR0, accu
;# else
;					sts		UDR1, accu
;# endif
;#else
;					out		UDR, accu
;#endif
;					ret

;************************************************************************
getch:
  ldi 	temp2,9	;8 data bit + 1 stop bit
;#ifndef MAX_TIME_COUNT
;_getc_1:
;#else
					ldi		to_cnt0, lo8( MAX_TIME_COUNT )
					ldi		to_cnt1, lo8( MAX_TIME_COUNT >> 8 ) 
					ldi		to_cnt2, lo8( MAX_TIME_COUNT >> 16) 
					;ldi		to_cnt3, lo8( MAX_TIME_COUNT >> 24) 

_getc_1:			subi	to_cnt0, 0x01				; 1
					sbci	to_cnt1, 0x00				; 1
					sbci	to_cnt2, 0x00				; 1
					;sbci	to_cnt3, 0x00				; 1
					brcs	reboot						; 1
;#endif

UART_getchar1:	
	sbic 	_SFR_IO_ADDR(UART_RXPIN),UART_RXPINNumber	;Wait for start bit
	rjmp 	_getc_1
	rcall UART_delay	;0.5 bit delay

UART_getchar2:	
	rcall UART_delay	;1 bit delay
	rcall UART_delay		
	clc			;clear carry
	sbic 	_SFR_IO_ADDR(UART_RXPIN),UART_RXPINNumber	;if RX pin high
	sec			;
	dec 	temp2		;If bit is stop bit
	breq 	UART_getchar3	;   return
				;else
	ror 	accu		;   shift bit into Rxbyte
	rjmp 	UART_getchar2	;   go get next

UART_getchar3:	
	ret

;#if defined(__AVR_ATmega128__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega48__) || defined(__AVR_ATmega88__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
;# if (BOOTUART==1)
;					lds		tmp3, UCSR0A
;					sbrs	tmp3, RXC0					; 1 set if a character has arrived
;# else
;					lds		tmp3, UCSR1A
;					sbrs	tmp3, RXC1					; 1 set if a character has arrived
;# endif
;#else
;					sbis	UCSRA, RXC					; 1 set if a character has arrived
;#endif
;					rjmp	_getc_1						; 2 -> 8 cycles per loop

;#if defined(__AVR_ATmega128__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega48__) || defined(__AVR_ATmega88__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
;# if (BOOTUART==1)
;					lds		accu, UDR0
;# else
;					lds		accu, UDR1
;# endif
;#else
;					in		accu, UDR
;#endif
;					cpi		accu, ' '
;					ret

;#if defined(MAX_TIME_COUNT) || (WDT_CHECK==1)
reboot:
;# if defined(__AVR_ATmega48__)
					movw	addr_l, user_l
;# else
;					ldi		addr_l, 0x00				; set Z to point at FLASH address 0x0000
;					ldi		addr_h, 0x00
;# endif
					ijmp								; jump to user code
;#endif
	
UART_delay:	
	ldi	mem_area,135
UART_delay1:	
	dec	mem_area
	brne	UART_delay1
	ret
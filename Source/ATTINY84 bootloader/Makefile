###############################################################
# Reduced size Bootloader
#
# for bugs, fixes and comments contact aziemer@a-z-e.de
###############################################################

###############################################################
# User tuneable part
###############################################################

# the MCU used
MCU			= attiny84

# MCU clock rate (crystal frequency)
CPUCLOCK	= 8000000

# reset address (in bytes) -
# 0x0000 for ATmega48 - no bootblock, reset address gets copied in bootloader
# 0x1e00 for 8k devices, 0x3e00 for 16k, 0x7e00 for 32k (bootblock size 256 words)
# 0xfc00 for 64k devices (bootblock size 512 words !! 256 words not available here)
RESETADDR	= 0x0000

# communication baudrate
#BAUDRATE	= 9600

# for MCUs with more than one async UART (i.e. 128k devices)
#BOOTUART	= 1

# if 1: check if CODE location 0x0000 is 0xFF -> start bootloader if yes
#CODE_CHECK	= 0

# if 1: check for bootloader pin being low on reset -> start bootloader if yes
#BL_CHECK	= 0
# port assignment, where the BL pin is
#BL_PORT		= B
#BL_PIN		= 3

# if 1: do NOT enter bootloader if a watchdog or brown-out reset was detected
# Exception: if CODE_CHECK or BL_CHECK is set and the corresponding state is met
WDT_CHECK	= 1

# bootloader times out after this seconds, and starts user code, if 0 -> no timeout
# NOTE: if 0, then CODE_CHECK and/or BL_CHECK should be set, or the bootloader never runs...
BL_TIMEOUT	= 5

# if 1: turn LED on when entering the bootloader
#USE_LED		= 0
# port assignment for the LED
#LED_PORT	= B
#LED_PIN		= 5

###############################################################
# nothing should need being changed below here
###############################################################
PROJECT	= ArduinoBOOT

SOURCE	= $(PROJECT).asm
OBJECT	= $(PROJECT).o 
TARGET	= $(PROJECT).elf

## Options common to compile, link and assembly rules
COMMON	 = -mmcu=$(MCU) -DMCU=$(MCU) -DF_CPU=$(CPUCLOCK)
#COMMON	+= -DBAUD_RATE=$(BAUDRATE) -DBOOTUART=$(BOOTUART)
#COMMON	+= -DUSE_LED=$(USE_LED) -DLED_DDR=DDR$(LED_PORT) -DLED_PORT=PORT$(LED_PORT) -DLED_PIN=PIN$(LED_PORT) -DLED_NO=PIN$(LED_PORT)$(LED_PIN)
COMMON	+= -DBL_TIMEOUT=$(BL_TIMEOUT)
#COMMON	+= -DCODE_CHECK=$(CODE_CHECK)
COMMON	+= -DWDT_CHECK=$(WDT_CHECK)
#COMMON	+= -DBL_CHECK=$(BL_CHECK) -DBL_DDR=DDR$(BL_PORT) -DBL_PORT=PORT$(BL_PORT) -DBL_PIN=PIN$(BL_PORT) -DBL_NO=PIN$(BL_PORT)$(BL_PIN)

## Assembly specific flags
ASMFLAGS  = $(COMMON)
ASMFLAGS += -x assembler-with-cpp -Wa,-gdwarf2

## Linker flags
LDFLAGS	 = $(COMMON)
LDFLAGS	+= -nostdlib -Wl,-Map=$(PROJECT).map,--section-start=.text=$(RESETADDR)

## Intel Hex file production flags
HEXFLAGS = -R .eeprom -R .fuse -R .lock -R .signature

## Build
all: $(PROJECT).hex $(PROJECT).lss

## Compile
$(OBJECT): $(SOURCE) Makefile
	avr-gcc -c $(INCLUDES) $(ASMFLAGS) $(SOURCE)

## Link
$(TARGET): $(OBJECT)
	avr-gcc $(LDFLAGS) $(OBJECT) -o $(TARGET)

%.hex: $(TARGET)
	avr-objcopy -O ihex $(HEXFLAGS) $< $@

%.lss: $(TARGET)
	avr-objdump -h -S $< > $@

## Clean target
.PHONY: clean
clean:
	-rm -rf $(OBJECT) $(PROJECT).elf dep/* $(PROJECT).hex $(PROJECT).eep $(PROJECT).lss $(PROJECT).map

## Other dependencies
-include $(shell mkdir dep 2>/dev/null) $(wildcard dep/*)


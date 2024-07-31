CC = avr-gcc
OBJCOPY = avr-objcopya
DUDE = avrdude
WARNINGS_FLAGS = -Wall -Wextra
MAIN= Demo.c
OUT = $(MAIN:.c=.elf)
IC= t85
ISP = usbtiny
SAFE_CLK = 125kHz
MMCU=attiny85
IC_CLOCK=8000000UL

default: build

FILES_TO_BUILD := $(wildcard *.c)
build:
	@echo "[BUILD] Generating ELF file"
	@$(CC) -O1 $(FILES_TO_BUILD) -DF_CPU=$(IC_CLOCK) -mmcu=$(MMCU) -o $(OUT) $(WARNINGS_FLAGS)


BUILD_RESULT := $(wildcard $(OUT))
write:
ifneq ($(BUILD_RESULT),)
	@echo "[WRITE] Writing program to controller"
	@$(DUDE) -p $(IC) -c $(ISP) -B $(SAFE_CLK) -U flash:w:$(OUT) -F
else
	@echo "[Write] $(OUT) is missing, try <make build> first"
endif

# Cleaning routines
FILES_FOR_DELETION := $(wildcard *.elf)
clean:
	@echo "[CLEAN] Removing compiled files from previous builds"
ifneq ($(FILES_FOR_DELETION),)
	@echo "Removing the following files : "$(FILES_FOR_DELETION)
	@rm $(FILES_FOR_DELETION)
else
	@echo "Nothing to delete!"
endif
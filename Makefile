#set this
TARGETS=  \
	a2_n10446711.hex
	
	

all: $(TARGETS)

#set this
CAB202_TEENSY_FOLDER = ../cab202_teensy 

# do not modify below this line             ----------------


TEENSY_LIBS = -lcab202_teensy -lprintf_flt -lm 
TEENSY_DIRS =-I$(CAB202_TEENSY_FOLDER) -L$(CAB202_TEENSY_FOLDER)
TEENSY_FLAGS = \
	-std=gnu99 \
	-mmcu=atmega32u4 \
	-DF_CPU=8000000UL \
	-funsigned-char \
	-funsigned-bitfields \
	-ffunction-sections \
	-fpack-struct \
	-fshort-enums \
	-Wall \
	-Werror \
	-Wl,-u,vfprintf \
	-Os 

clean:
	rm -f *.csrc; \
	rm -f *.hsrc; \
	for f in $(TARGETS); do \
		if [ -f $$f ]; then rm $$f; fi; \
		if [ -f $$f.exe ]; then rm $$f.exe; fi; \
		if [ -f $$f.elf ]; then rm $$f.elf; fi; \
		if [ -f $$f.obj ]; then rm $$f.obj; fi; \
		if [ -f $$f.hex ]; then rm $$f.hex; fi; \
	done

rebuild: clean all


%.hex : %.c 
	avr-gcc $< $(TEENSY_FLAGS) $(TEENSY_DIRS) $(TEENSY_LIBS) -o $@.obj
	avr-objcopy -O ihex $@.obj $@


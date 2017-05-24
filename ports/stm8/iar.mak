############
# Settings #
############

# Set up build environment (using GNU Make)
#   set PATH=%PATH%;C:\Program Files\GNU_MAKE;C:\Program Files\IAR Systems\Embedded Workbench 6.0\stm8\bin
#   set MAKE_MODE=DOS

# Build all test applications:
#   make -f iar.mak


# Location of build tools and atomthreads sources
EWSTM8_DIR=C:\Program Files\IAR Systems\Embedded Workbench 6.0\stm8
KERNEL_DIR=../../kernel
TESTS_DIR=../../tests
PERIPHS_DIR=stm8s-periphs
CC=iccstm8
ASM=iasmstm8
LINK=ilinkstm8
HEX=ielftool

# CPU part number
PART=STM8S105

# Enable stack-checking
STACK_CHECK=true

# Directory for built objects
BUILD_DIR=build-iar

# Port/application object files
APP_OBJECTS = atomport.o tests-main.o uart.o
APP_ASM_OBJECTS = atomport-asm-iar.o

# STM8S Peripheral driver object files
PERIPH_OBJECTS = stm8s_gpio.o stm8s_tim1.o stm8s_clk.o stm8s_uart2.o

# Kernel object files
KERNEL_OBJECTS = atomkernel.o atomsem.o atommutex.o atomtimer.o atomqueue.o
KERNEL_OBJECTS += atomevent.o

# Collection of built objects (excluding test applications)
ALL_OBJECTS = $(APP_OBJECTS) $(APP_ASM_OBJECTS) $(PERIPH_OBJECTS) $(KERNEL_OBJECTS)
BUILT_OBJECTS = $(patsubst %,$(BUILD_DIR)/%,$(ALL_OBJECTS))

# Test object files (dealt with separately as only one per application build)
TEST_OBJECTS = $(notdir $(patsubst %.c,%.o,$(wildcard $(TESTS_DIR)/*.c)))

# Target application filenames (.elf) for each test object
TEST_ELFS = $(patsubst %.o,%.elf,$(TEST_OBJECTS))
TEST_S19S = $(patsubst %.o,%.s19,$(TEST_OBJECTS))

# Search build/output directory for dependencies
vpath %.o .\$(BUILD_DIR)
vpath %.elf .\$(BUILD_DIR)
vpath %.hex .\$(BUILD_DIR)

# Compiler/Assembler flags
CFLAGS=-e -Oh --code_model small --data_model medium \
	--dlib_config "$(EWSTM8_DIR)\lib\dlstm8smn.h" -D NDEBUG -D $(PART) \
	--diag_suppress Pa050  
DBG_CFLAGS=-e -Ol --no_cse --no_unroll --no_inline --no_code_motion --no_tbaa \
	--no_cross_call --debug --code_model small --data_model medium \
	--dlib_config "$(EWSTM8_DIR)\lib\dlstm8smn.h" -D $(PART) \
	--diag_suppress Pa050 

ASMFLAGS=-M'<>' -ld $(BUILD_DIR)\list --diag_suppress Pa050 --code_model small \
	--data_model medium 
DBG_ASMFLAGS=-M'<>' -r -ld $(BUILD_DIR)\list --diag_suppress Pa050 --code_model small \
	--data_model medium 

LINKFLAGS=--redirect _Printf=_PrintfSmall --redirect _Scanf=_ScanfSmall \
	--config "$(EWSTM8_DIR)\config\lnkstm8s105c6.icf" --config_def \
	_CSTACK_SIZE=0x100 --config_def _HEAP_SIZE=0x100 \
	--entry __iar_program_start 
DBG_LINKFLAGS=--redirect _Printf=_PrintfSmall --redirect _Scanf=_ScanfSmall \
	--config "$(EWSTM8_DIR)\config\lnkstm8s105c6.icf" --config_def \
	_CSTACK_SIZE=0x100 --config_def _HEAP_SIZE=0x100 \
	--entry __iar_program_start 

# Enable stack-checking (disable if not required)
ifeq ($(STACK_CHECK),true)
CFLAGS += -D ATOM_STACK_CHECKING
DBG_CFLAGS += -D ATOM_STACK_CHECKING
endif


#################
# Build targets #
#################

# All tests
all: $(BUILD_DIR) $(TEST_S19S) iar.mak

# Make build/output directory
$(BUILD_DIR):
	mkdir $(BUILD_DIR)

# Test HEX files (one application build for each test)
$(TEST_S19S): %.s19: %.elf
	@echo Building $@
	$(HEX) $(BUILD_DIR)/$(notdir $<) $(BUILD_DIR)/$@ --srec

# Test ELF files (one application build for each test)
$(TEST_ELFS): %.elf: %.o $(KERNEL_OBJECTS) $(PERIPH_OBJECTS) $(APP_OBJECTS) $(APP_ASM_OBJECTS)
	$(LINK) $(BUILD_DIR)/$(notdir $<) $(BUILT_OBJECTS) $(LINKFLAGS) -o $(BUILD_DIR)/$@

# Kernel objects builder
$(KERNEL_OBJECTS): %.o: $(KERNEL_DIR)/%.c
	$(CC) $< $(CFLAGS) -I . -I $(PERIPHS_DIR) -o $(BUILD_DIR)

# Test objects builder
$(TEST_OBJECTS): %.o: $(TESTS_DIR)/%.c
	$(CC) $< $(CFLAGS) -I . -I $(KERNEL_DIR) -I $(PERIPHS_DIR) -o $(BUILD_DIR)

# Peripheral objects builder
$(PERIPH_OBJECTS): %.o: $(PERIPHS_DIR)/%.c
	$(CC) $< $(CFLAGS) -I . -I $(PERIPHS_DIR) -o $(BUILD_DIR)

# Application C objects builder
$(APP_OBJECTS): %.o: ./%.c
	$(CC) $< $(CFLAGS) -I $(KERNEL_DIR) -I $(TESTS_DIR) -I $(PERIPHS_DIR) -o $(BUILD_DIR)

# Application asm objects builder
$(APP_ASM_OBJECTS): %.o: ./%.s
	$(ASM) $< $(ASMFLAGS) -I $(KERNEL_DIR) -o $(BUILD_DIR)/$(notdir $@)

# Clean
clean:
	rm -f *.o *.elf *.map *.hex *.bin *.lst *.stm8 *.s19 *.out
	rm -rf doxygen-kernel
	rm -rf doxygen-stm8
	rm -rf build-iar

doxygen:
	doxygen $(KERNEL_DIR)/Doxyfile
	doxygen ./Doxyfile

############
# Settings #
############

# Set up build environment (using GNU Make)
#   set PATH=%PATH%;C:\Program Files\GNU_MAKE;C:\Program Files\Raisonance\Ride\bin
#   set MAKE_MODE=DOS

# Build all test applications:
#   make -f raisonance.mak


# Location of build tools and atomthreads sources
KERNEL_DIR=../../kernel
TESTS_DIR=../../tests
PERIPHS_DIR=stm8s-periphs
LIBS_DIR=C:\Program Files\Raisonance\Ride
CC=rcstm8
ASM=mastm8
LINK=rlstm8
MAKEHEX=omf2hex

# CPU part number
PART=STM8S105

# Enable stack-checking
STACK_CHECK=true

# Directory for built objects
BUILD_DIR=build-raisonance

# Port/application object files
APP_OBJECTS = atomport.o tests-main.o uart.o
APP_ASM_OBJECTS = atomport-asm-raisonance.o

# STM8S Peripheral driver object files
PERIPH_OBJECTS = stm8s_gpio.o stm8s_tim1.o stm8s_clk.o stm8s_uart2.o

# Kernel object files
KERNEL_OBJECTS = atomkernel.o atomsem.o atommutex.o atomtimer.o atomqueue.o

# Collection of built objects (excluding test applications)
ALL_OBJECTS = $(APP_OBJECTS) $(APP_ASM_OBJECTS) $(PERIPH_OBJECTS) $(KERNEL_OBJECTS)
BUILT_OBJECTS = $(patsubst %,$(BUILD_DIR)/%,$(ALL_OBJECTS))
comma := ,
space :=
space +=
BUILT_OBJECTS_CS = $(subst $(space),$(comma),$(BUILT_OBJECTS))

# Test object files (dealt with separately as only one per application build)
TEST_OBJECTS = $(notdir $(patsubst %.c,%.o,$(wildcard $(TESTS_DIR)/*.c)))

# Target application filenames (.aof and .hex) for each test object
TEST_AOFS = $(patsubst %.o,%.aof,$(TEST_OBJECTS))
TEST_HEXS = $(patsubst %.o,%.hex,$(TEST_OBJECTS))

# Search build/output directory for dependencies
vpath %.o .\$(BUILD_DIR)
vpath %.aof .\$(BUILD_DIR)
vpath %.hex .\$(BUILD_DIR)

# Compiler/Assembler flags
CFLAGS=WRV(0) STM8(SMALL) DGC(data) AUTO OT(7,SIZE) CD CO SB LAOB DF($(PART))
DBG_CFLAGS=WRV(0) STM8(SMALL) DEBUG DGC(data) AUTO OT(0) CD CO SB LAOB DF($(PART))
ASMFLAGS=QUIET NOPR ERRORPRINT MODESTM8
DBG_ASMFLAGS=QUIET ERRORPRINT DEBUG MODESTM8
LDFLAGS=NODEBUGLINES NODEBUGPUBLICS NODEBUGSYMBOLS DATASTART(0x0) RAMSIZE(0x800) CODESTART(0x8000) CODESIZE(0x8000) STACKTOP(0x800) STACKSIZE(0x200) EEPROMSTART(0x4000) EEPROMSIZE(0x400)
DBG_LDFLAGS=DEBUGLINES DEBUGPUBLICS DEBUGSYMBOLS DATASTART(0x0) RAMSIZE(0x800) CODESTART(0x8000) CODESIZE(0x8000) STACKTOP(0x800) STACKSIZE(0x200) EEPROMSTART(0x4000) EEPROMSIZE(0x400)

# Enable stack-checking (disable if not required)
ifeq ($(STACK_CHECK),true)
CFLAGS += DF(ATOM_STACK_CHECKING)
DBG_CFLAGS += DF(ATOM_STACK_CHECKING)
endif


#################
# Build targets #
#################

# All tests
all: $(BUILD_DIR) $(TEST_HEXS) raisonance.mak

# Make build/output directory
$(BUILD_DIR):
	mkdir $(BUILD_DIR)

# Test HEX files (one application build for each test)
$(TEST_HEXS): %.hex: %.aof
	@echo Building $@
	$(MAKEHEX) $(BUILD_DIR)/$<

# Test ELF files (one application build for each test)
$(TEST_AOFS): %.aof: %.o $(KERNEL_OBJECTS) $(PERIPH_OBJECTS) $(APP_OBJECTS) $(APP_ASM_OBJECTS)
	$(LINK) -P "$(BUILD_DIR)/$(notdir $<),$(BUILT_OBJECTS_CS)" TO($(BUILD_DIR)/$@) LIBPATH("$(LIBS_DIR)\Lib\ST7") PR($(BUILD_DIR)/$(basename $@).map) $(LDFLAGS)

# Kernel objects builder
$(KERNEL_OBJECTS): %.o: $(KERNEL_DIR)/%.c
	$(CC) $< OBJECT($(BUILD_DIR)/$@) $(CFLAGS) PR($(BUILD_DIR)/$(basename $(notdir $<)).lst) PIN(".") PIN("$(PERIPHS_DIR)") PIN("$(LIBS_DIR)\inc") PIN("$(LIBS_DIR)\inc\ST7")

# Test objects builder
$(TEST_OBJECTS): %.o: $(TESTS_DIR)/%.c
	$(CC) $< OBJECT($(BUILD_DIR)/$@) $(CFLAGS) PR($(BUILD_DIR)/$(basename $(notdir $<)).lst) PIN(".") PIN("$(KERNEL_DIR)") PIN("$(PERIPHS_DIR)") PIN("$(LIBS_DIR)\inc") PIN("$(LIBS_DIR)\inc\ST7")

# Peripheral objects builder
$(PERIPH_OBJECTS): %.o: $(PERIPHS_DIR)/%.c
	$(CC) $< OBJECT($(BUILD_DIR)/$@) $(CFLAGS) PR($(BUILD_DIR)/$(basename $(notdir $<)).lst) PIN(".") PIN("$(PERIPHS_DIR)") PIN("$(LIBS_DIR)\inc") PIN("$(LIBS_DIR)\inc\ST7")

# Application C objects builder
$(APP_OBJECTS): %.o: ./%.c
	$(CC) $< OBJECT($(BUILD_DIR)/$@) $(CFLAGS) PR($(BUILD_DIR)/$(basename $(notdir $<)).lst) PIN(".") PIN("$(KERNEL_DIR)") PIN("$(TESTS_DIR)") PIN("$(PERIPHS_DIR)") PIN("$(LIBS_DIR)\inc") PIN("$(LIBS_DIR)\inc\ST7")

# Application asm objects builder
$(APP_ASM_OBJECTS): %.o: ./%.asm
	$(ASM) $< OBJECT($(BUILD_DIR)/$@) $(ASMFLAGS) PR($(BUILD_DIR)/$(basename $(notdir $<)).lst) PIN(".") PIN("$(KERNEL_DIR)") PIN("$(LIBS_DIR)\inc") PIN("$(LIBS_DIR)\inc\ST7")

# Clean
clean:
	rm -f *.o *.O *.elf *.aof *.map *.hex *.bin *.lst *.LST *.stm8 *.s19
	rm -rf doxygen-kernel
	rm -rf doxygen-stm8
	rm -rf build-raisonance

doxygen:
	doxygen $(KERNEL_DIR)/Doxyfile
	doxygen ./Doxyfile

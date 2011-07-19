############
# Settings #
############

# Build all test applications:
#   make

# Location of build tools and atomthreads sources
KERNEL_DIR=../../kernel
TESTS_DIR=../../tests
CC=mips-linux-gnu-gcc
OBJCOPY=mips-linux-gnu-objcopy

# Check if verbosity is ON for build process
VERBOSE_DEFAULT    := 0
CMD_PREFIX_DEFAULT := @
ifdef VERBOSE
	ifeq ("$(origin VERBOSE)", "command line")
		VB := $(VERBOSE)
	else
		VB := $(VERBOSE_DEFAULT)
	endif
else
	VB := $(VERBOSE_DEFAULT)
endif
ifeq ($(VB), 1)
	V :=
else
	V := $(CMD_PREFIX_DEFAULT)
endif

# Enable stack-checking. WARNING: the full automated test suite currently
# requires a little over 1KB RAM with stack-checking enabled. If you are 
# using a device with 1KB internal SRAM and no external SRAM then you
# must disable stack-checking to run all of the automated tests.
#STACK_CHECK=true

# Directory for built objects
BUILD_DIR=build

# Port/application object files
APP_OBJECTS = atomport.o tests-main.o 8250-serial.o printk.o string.o vsprintf.o io.o atomport-interrupts.o atomport-timer.o

APP_ASM_OBJECTS = atomport-entry.o atomport-asm.o

# Kernel object files
KERNEL_OBJECTS = atomkernel.o atomsem.o atommutex.o atomtimer.o atomqueue.o

# Collection of built objects (excluding test applications)
ALL_OBJECTS = $(APP_ASM_OBJECTS) $(APP_OBJECTS) $(KERNEL_OBJECTS)
BUILT_OBJECTS = $(patsubst %,$(BUILD_DIR)/%,$(ALL_OBJECTS))

# Test object files (dealt with separately as only one per application build)
TEST_OBJECTS = $(notdir $(patsubst %.c,%.o,$(wildcard $(TESTS_DIR)/*.c)))

# Target application filenames (.elf and .hex) for each test object
TEST_ELFS = $(patsubst %.o,%.elf,$(TEST_OBJECTS))
TEST_HEXS = $(patsubst %.o,%.hex,$(TEST_OBJECTS))

# Search build/output directory for dependencies
vpath %.o ./$(BUILD_DIR)
vpath %.elf ./$(BUILD_DIR)
vpath %.hex ./$(BUILD_DIR)

# GCC flags
CFLAGS= -g \
	-Wall \
	-Werror \
	-O \
	-fstrength-reduce \
	-fomit-frame-pointer \
	-finline-functions \
	-nostdinc \
	-fno-builtin \
	-fno-stack-protector

# Enable stack-checking (disable if not required)
ifeq ($(STACK_CHECK),true)
CFLAGS += -DATOM_STACK_CHECKING
endif

#################
# Build targets #
#################

# All tests
all: $(BUILD_DIR) $(TEST_HEXS) Makefile

# Make build/output directory
$(BUILD_DIR):
	mkdir $(BUILD_DIR)

# Test HEX files (one application build for each test)
$(TEST_HEXS): %.hex: %.elf
	$(if $(V), @echo " (HEX)        $(subst $(build_dir)/,,$@)")
	$(V)$(OBJCOPY) -j .text -j .data -O ihex $(BUILD_DIR)/$< $(BUILD_DIR)/$@

# Test ELF files (one application build for each test)
$(TEST_ELFS): %.elf: %.o $(APP_ASM_OBJECTS) $(KERNEL_OBJECTS) $(APP_OBJECTS)
	$(if $(V), @echo " (ELF)        $(subst $(build_dir)/,,$@)")
	$(V)$(CC) $(CFLAGS) -nostdlib -nodefaultlibs $(BUILD_DIR)/$(notdir $<) $(BUILT_OBJECTS) --output $(BUILD_DIR)/$@ -Wl -T linker.ld

# Kernel objects builder
$(KERNEL_OBJECTS): %.o: $(KERNEL_DIR)/%.c
	$(if $(V), @echo " (CC)         $(subst $(build_dir)/,,$@)")
	$(V)$(CC) -c $(CFLAGS) -I. -I$(KERNEL_DIR) $< -o $(BUILD_DIR)/$(notdir $@)

# Test objects builder
$(TEST_OBJECTS): %.o: $(TESTS_DIR)/%.c
	$(if $(V), @echo " (CC)         $(subst $(build_dir)/,,$@)")
	$(V)$(CC) -c $(CFLAGS) -I. -I$(KERNEL_DIR) $< -o $(BUILD_DIR)/$(notdir $@)

# Application C objects builder
$(APP_OBJECTS): %.o: ./%.c
	$(if $(V), @echo " (CC)         $(subst $(build_dir)/,,$@)")
	$(V)$(CC) -c $(CFLAGS) -I. -I$(KERNEL_DIR) -I$(TESTS_DIR) $< -o $(BUILD_DIR)/$(notdir $@)

# Application asm objects builder
$(APP_ASM_OBJECTS): %.o: ./%.s
	$(if $(V), @echo " (AS)         $(subst $(build_dir)/,,$@)")
	$(V)$(CC) -c $(CFLAGS) -D__ASSEMBLY__ -x assembler-with-cpp -I. -I$(KERNEL_DIR) $< -o $(BUILD_DIR)/$(notdir $@)

# .lst file builder
%.lst: %.c
	$(if $(V), @echo " (LST)        $(subst $(build_dir)/,,$@)")
	$(V)$(CC) $(CFLAGS) -I. -I$(KERNEL_DIR) -I$(TESTS_DIR) -Wa,-al $< > $@

# Clean
clean:
	$(V)rm -f *.o *.elf *.map *.hex *.bin *.lst
	rm -rf doxygen-kernel
	rm -rf doxygen-mips
	rm -rf build

doxygen:
	doxygen $(KERNEL_DIR)/Doxyfile
	doxygen ./Doxyfile

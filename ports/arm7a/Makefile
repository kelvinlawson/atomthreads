############
# Settings #
############

# Build all test applications:
#   make
#
# Program a test application using UISP (appname => test app e.g. sems1):
#   make program app=appname

# Location of build tools and atomthreads sources
kernel_dir=../../kernel
tests_dir=../../tests

# Directory for built objects
build_dir=build

CPU=cortex-a8
BOARD=pb-a8
CC=$(CROSS_COMPILE)gcc
OBJCOPY=$(CROSS_COMPILE)objcopy

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

# Port/application object files
APP_OBJECTS = arm_irq.o 
APP_OBJECTS += arm_gic.o 
APP_OBJECTS += arm_timer.o 
APP_OBJECTS += arm_uart.o 
APP_OBJECTS += arm_main.o 
APP_OBJECTS += atomport.o
APP_OBJECTS += printk.o 
APP_OBJECTS += string.o 
APP_OBJECTS += vsprintf.o
APP_ASM_OBJECTS = arm_entry.o
APP_ASM_OBJECTS += atomport-asm.o 

# Kernel object files
KERNEL_OBJECTS = atomkernel.o 
KERNEL_OBJECTS += atomsem.o 
KERNEL_OBJECTS += atommutex.o 
KERNEL_OBJECTS += atomtimer.o 
KERNEL_OBJECTS += atomqueue.o

# Collection of built objects (excluding test applications)
ALL_OBJECTS = $(APP_ASM_OBJECTS) $(APP_OBJECTS) $(KERNEL_OBJECTS)
BUILT_OBJECTS = $(patsubst %,$(build_dir)/%,$(ALL_OBJECTS))

# Test object files (dealt with separately as only one per application build)
TEST_OBJECTS = $(notdir $(patsubst %.c,%.o,$(wildcard $(tests_dir)/*.c)))

# Target application filenames .elf for each test object
TEST_ELFS = $(patsubst %.o,%.elf,$(TEST_OBJECTS))

# Search build/output directory for dependencies
#vpath %.o ./$(build_dir)
#vpath %.elf ./$(build_dir)

# GCC flags
CFLAGS= -g \
	-Wall \
	-Werror \
	-mcpu=$(CPU) \
	-nostdinc \
	-nostdlib \
	-nodefaultlibs \
	-fno-builtin

# Enable stack-checking (disable if not required)
ifeq ($(STACK_CHECK),true)
CFLAGS += -DATOM_STACK_CHECKING
endif

#################
# Build targets #
#################

# All tests
all: $(TEST_ELFS) Makefile

# Test ELF files (one application build for each test)
$(TEST_ELFS): %.elf: %.o $(APP_ASM_OBJECTS) $(KERNEL_OBJECTS) $(APP_OBJECTS)
	$(V)mkdir -p `dirname $(build_dir)/$@`
	$(if $(V), @echo " (ELF)        $(subst $(build_dir)/,,$@)")
	$(V)$(CC) $(CFLAGS) $(build_dir)/$(notdir $<) $(BUILT_OBJECTS) -static-libgcc -lgcc --output $(build_dir)/$@ -Wl -T linker.ld

# Kernel objects builder
$(KERNEL_OBJECTS): %.o: $(kernel_dir)/%.c
	$(V)mkdir -p `dirname $(build_dir)/$(notdir $@)`
	$(if $(V), @echo " (CC)         $(subst $(build_dir)/,,$@)")
	$(V)$(CC) -c $(CFLAGS) -I. -I$(kernel_dir) $< -o $(build_dir)/$(notdir $@)

# Test objects builder
$(TEST_OBJECTS): %.o: $(tests_dir)/%.c
	$(V)mkdir -p `dirname $(build_dir)/$(notdir $@)`
	$(if $(V), @echo " (CC)         $(subst $(build_dir)/,,$@)")
	$(V)$(CC) -c $(CFLAGS) -I. -I$(kernel_dir) $< -o $(build_dir)/$(notdir $@)

# Application C objects builder
$(APP_OBJECTS): %.o: ./%.c
	$(V)mkdir -p `dirname $(build_dir)/$(notdir $@)`
	$(if $(V), @echo " (CC)         $(subst $(build_dir)/,,$@)")
	$(V)$(CC) -c $(CFLAGS) -I. -I$(kernel_dir) -I$(tests_dir) $< -o $(build_dir)/$(notdir $@)

# Application asm objects builder
$(APP_ASM_OBJECTS): %.o: ./%.s
	$(V)mkdir -p `dirname $(build_dir)/$(notdir $@)`
	$(if $(V), @echo " (AS)         $(subst $(build_dir)/,,$@)")
	$(V)$(CC) -c $(CFLAGS) -D__ASSEMBLY__ -x assembler-with-cpp -I. -I$(kernel_dir) $< -o $(build_dir)/$(notdir $@)

# .lst file builder
%.lst: %.c
	$(V)mkdir -p `dirname $@`
	$(if $(V), @echo " (LST)        $(subst $(build_dir)/,,$@)")
	$(V)$(CC) $(CFLAGS) -I. -I$(kernel_dir) -I$(tests_dir) -Wa,-al $< > $@

# Clean
clean:
	$(V)rm -f *.o *.elf *.map *.bin *.lst
	rm -rf doxygen-kernel
	rm -rf doxygen-avr
	rm -rf $(build_dir)

doxygen:
	doxygen $(kernel_dir)/Doxyfile
	doxygen ./Doxyfile

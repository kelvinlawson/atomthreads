##########################################
# Toplevel makefile for all ARM7a boards #
##########################################

# Build directory
ifdef O
 build_dir=$(shell readlink -f $(O))
else
 build_dir=$(CURDIR)/build
endif

# Source directory
src_dir=$(CURDIR)

# Configuration
ifndef CPU
CPU=cortex-a8
endif
ifndef BOARD
BOARD=pb-a8
endif
CC=$(CROSS_COMPILE)gcc
OBJCOPY=$(CROSS_COMPILE)objcopy
# Enable stack-checking. WARNING: the full automated test suite currently
# requires a little over 1KB RAM with stack-checking enabled. If you are 
# using a device with 1KB internal SRAM and no external SRAM then you
# must disable stack-checking to run all of the automated tests.
#STACK_CHECK=true

# Location of atomthreads sources
board_dir=$(src_dir)/$(BOARD)
kernel_dir=$(src_dir)/../../kernel
tests_dir=$(src_dir)/../../tests

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

# object files
objs = arm_irq.o 
objs += arm_main.o 
objs += atomport.o
objs += arm_entry.o
objs += atomport-asm.o 

# include board makefile for board specific objects
-include $(board_dir)/Makefile

# library object files
objs += printk.o 
objs += string.o 
objs += vsprintf.o

# Kernel object files
objs += atomkernel.o 
objs += atomsem.o 
objs += atommutex.o 
objs += atomtimer.o 
objs += atomqueue.o
objs += atomevent.o

# Collection of built objects (excluding test applications)
build_objs = $(foreach obj,$(objs),$(build_dir)/$(obj))

# Target application filenames .elf for each test object
tobjs = $(notdir $(patsubst %.c,%.o,$(wildcard $(tests_dir)/*.c)))
telfs = $(patsubst %.o,%.elf,$(tobjs))
tbins = $(patsubst %.o,%.bin,$(tobjs))
build_tobjs = $(foreach tobj,$(tobjs),$(build_dir)/$(tobj))
build_telfs = $(foreach telf,$(telfs),$(build_dir)/$(telf))
build_tbins = $(foreach tbin,$(tbins),$(build_dir)/$(tbin))

# GCC flags
CFLAGS= -g \
	-Wall \
	-Werror \
	-mcpu=$(CPU) \
	-nostdinc \
	-nostdlib \
	-nodefaultlibs \
	-fno-builtin \
	-I$(src_dir) \
	-I$(board_dir) \
	-I$(kernel_dir) \
	-I$(tests_dir)

# Enable stack-checking (disable if not required)
ifeq ($(STACK_CHECK),true)
CFLAGS += -DATOM_STACK_CHECKING
endif

# All
.PHONY: all
all: $(build_tbins) $(build_telfs) $(build_tobjs) $(build_objs) Makefile

$(build_dir)/%.bin: $(build_dir)/%.elf
	$(V)mkdir -p `dirname $@`
	$(if $(V), @echo " (OBJCOPY)   $(subst $(build_dir)/,,$@)")
	$(V)$(OBJCOPY) -O binary $< $@

$(build_dir)/%.elf: $(build_dir)/%.o $(build_objs) $(board_dir)/linker.ld
	$(V)mkdir -p `dirname $@`
	$(if $(V), @echo " (ELF)       $(subst $(build_dir)/,,$@)")
	$(V)$(CC) $(CFLAGS) $(build_objs) $< -static-libgcc -lgcc  -Wl -T $(board_dir)/linker.ld -o $@

$(build_dir)/%.o: $(src_dir)/%.S
	$(V)mkdir -p `dirname $@`
	$(if $(V), @echo " (AS)        $(subst $(build_dir)/,,$@)")
	$(V)$(CC) $(CFLAGS) -D__ASSEMBLY__ -I`dirname $<` -c $< -o $@

$(build_dir)/%.o: $(src_dir)/%.c
	$(V)mkdir -p `dirname $@`
	$(if $(V), @echo " (CC)        $(subst $(build_dir)/,,$@)")
	$(V)$(CC) $(CFLAGS) $(CFLAGS) -I`dirname $<` -c $< -o $@

$(build_dir)/%.o: $(kernel_dir)/%.c
	$(V)mkdir -p `dirname $@`
	$(if $(V), @echo " (CC)        $(subst $(build_dir)/,,$@)")
	$(V)$(CC) $(CFLAGS) $(CFLAGS) -I`dirname $<` -c $< -o $@

$(build_dir)/%.o: $(tests_dir)/%.c
	$(V)mkdir -p `dirname $@`
	$(if $(V), @echo " (CC)        $(subst $(build_dir)/,,$@)")
	$(V)$(CC) $(CFLAGS) $(CFLAGS) -I`dirname $<` -c $< -o $@

# Clean
.PHONY: clean
clean:
	rm -rf doxygen-kernel
	rm -rf doxygen-armv7a
	rm -rf $(build_dir)
# Docs
.PHONY: doxygen
doxygen:
	doxygen $(kernel_dir)/Doxyfile
	doxygen ./Doxyfile

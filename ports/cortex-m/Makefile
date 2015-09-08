################################################
# Toplevel makefile for all Cortex-M targets   #
################################################

ifeq ($(V),)
	Q := @
	# Do not print "Entering directory ...".
	MAKEFLAGS += --no-print-directory
endif

# Build directory
ifdef O
	build_dir=$(shell readlink -f $(O))
else
	build_dir=$(CURDIR)/build
endif

# Source directory
src_dir=$(CURDIR)

# Clean object list before including board makefile
objs := 
aobjs :=

# set default board if none is given
ifeq ($(BOARD),)
	BOARD = nucleo-f103rb
endif

include $(src_dir)/boards/$(BOARD)/Makefile.include

# Make sure target MCU is set
ifndef TARGET
	$(error TARGET undefined)
endif

# Configure toolchain
CROSS_COMPILE	?=	arm-none-eabi-

CC		:= $(CROSS_COMPILE)gcc
CXX		:= $(CROSS_COMPILE)g++
LD		:= $(CROSS_COMPILE)gcc
AR		:= $(CROSS_COMPILE)ar
AS		:= $(CROSS_COMPILE)as
OBJCOPY		:= $(CROSS_COMPILE)objcopy
OBJDUMP		:= $(CROSS_COMPILE)objdump
GDB		:= $(CROSS_COMPILE)gdb
STFLASH		= $(shell which st-flash)

LDSCRIPT	?= linker/$(TARGET).ld

# Enable stack-checking. WARNING: the full automated test suite currently
# requires a little over 1KB RAM with stack-checking enabled. If you are 
# using a device with 1KB internal SRAM and no external SRAM then you
# must disable stack-checking to run all of the automated tests.
#STACK_CHECK=true

# Location of atomthreads sources
board_dir=$(src_dir)/boards/$(BOARD)
common_dir=$(src_dir)/common
kernel_dir=$(src_dir)/../../kernel
tests_dir=$(src_dir)/../../tests

# Check if user wants to use external opencm3 lib or if we have to build
# it ourselves
ifeq ($(OPENCM3_DIR),)
	OPENCM3_DIR = $(src_dir)/libopencm3
	build_lib = true
endif

ifneq ($(V),)
    $(info Using $(OPENCM3_DIR) as path to opencm3 library)
endif

# Object files needed by all applications
objs += atomport.o 
objs += atomport-asm.o 

# Kernel object files
objs += atomkernel.o 
objs += atomsem.o 
objs += atommutex.o 
objs += atomtimer.o 
objs += atomqueue.o

# Collection of built objects (excluding test applications)
build_objs = $(foreach obj,$(objs),$(build_dir)/$(obj))

# Object needed by all test applications, but not user apps
tmobjs = tests-main.o
build_tmobjs    = $(foreach obj,$(tmobjs),$(build_dir)/$(obj))
build_tmobjs   += $(build_objs)

# Set up search paths for libopencm3
INCLUDE_DIR	= $(OPENCM3_DIR)/include
LIB_DIR		= $(OPENCM3_DIR)/lib
SCRIPT_DIR	= $(OPENCM3_DIR)/scripts

# GCC flags
CFLAGS		=  -Os -g
CFLAGS		+= -Wall -Werror
CFLAGS		+= -Wredundant-decls -Wstrict-prototypes
CFLAGS		+= -fno-common -ffunction-sections -fdata-sections

# Enable stack-checking (disable if not required)
ifeq ($(STACK_CHECK),true)
CFLAGS += -DATOM_STACK_CHECKING -DTESTS_LOG_STACK_USAGE
endif

# C & C++ preprocessor common flags
CPPFLAGS	+= -MD
CPPFLAGS	+= -Wall -Wundef -Werror
CPPFLAGS	+= -I$(INCLUDE_DIR) $(DEFS)
CPPFLAGS    += -I$(board_dir) -I$(common_dir) -I$(src_dir) -I$(kernel_dir) -I$(tests_dir)

# Assembler flags
ASFLAGS     += -D__ASSEMBLY__ 
ASFLAGS     += -D__NEWLIB__

# Linker flags
LDFLAGS		+= --static -nostartfiles
LDFLAGS		+= -L$(LIB_DIR)
LDFLAGS		+= -T$(LDSCRIPT)
LDFLAGS		+= -Wl,-Map=$(build_dir)/$(*).map
LDFLAGS		+= -Wl,--gc-sections
ifeq ($(V),99)
LDFLAGS		+= -Wl,--print-gc-sections
endif

## Used libraries
# Target specific version libopencm3
LDLIBS		+= -l$(LIBNAME)

ifneq ($(BOARD),qemu)
## Flags for newlibc.    
# This uses the version of newlib optimised for size and without libnosys.
# This usually is the version of newlib you want to use, although it has some
# limitations, like printf not having support for floating point vars.
LDLIBS		+= -Wl,--start-group -lc_nano -lgcc -Wl,--end-group

# This is the fully bloated version of newlib. Adds about 20k compared to
# libc_nano. 
#LDLIBS		+= -Wl,--start-group -lc -lgcc -Wl,--end-group

# Use small newlib with libnosys if you do not want to use the provided stubs.
# Be advised that heap management will probaly break in interesting ways.
# This is because libnosys' standard _sbrk() expects the stack to start at 
# the top end of memory while the threads' stacks are positioned inside 
# the BSS.
#LDLIBS		+= -Wl,--start-group -lc_nano -lgcc -lnosys -Wl,--end-group
else
# Special LDLIBS for qemu target to enable semi-hosting. 
# TODO: Check if this is also useful for real hardware  
LDLIBS		+= -Wl,--start-group --specs=rdimon.specs -lc -lrdimon -lgcc -Wl,--end-group
endif

.PHONY: all
all: build_all

# Generate a direct make target for a test application 
define build_test
.PHONY: $(1)
$(1): $(build_dir)/$(1).elf
endef

# Target application filenames .elf for each test object
tobjs = $(notdir $(patsubst %.c,%.o,$(wildcard $(tests_dir)/*.c)))
telfs = $(patsubst %.o,%.elf,$(tobjs))
tbins = $(patsubst %.o,%.bin,$(tobjs))
thexs = $(patsubst %.o,%.hex,$(tobjs))
build_tobjs = $(foreach tobj,$(tobjs),$(build_dir)/$(tobj))
build_telfs = $(foreach telf,$(telfs),$(build_dir)/$(telf))
build_tbins = $(foreach tbin,$(tbins),$(build_dir)/$(tbin))
build_thexs = $(foreach thex,$(thexs),$(build_dir)/$(thex))

# Add a direct make target for every test app
$(foreach test,$(patsubst %.o,%,$(tobjs)),$(eval $(call build_test,$(test))))

# Define an explicit target and linker rule for an application not in the
# test suite. This will be a stand-alone applications located in the board
# or common directory
define build_app
.PHONY: $(1)
$(1): $(build_dir)/$(1).elf

$(build_dir)/$(1).elf $(build_dir)/$(1).map: $(LIB_DIR)/lib$(LIBNAME).a $(build_dir)/$(1).o $(build_objs) $(LDSCRIPT)
	$$(Q)mkdir -p `dirname $$@`
	$$(if $$(Q), @echo " (ELF)       $$(subst $$(build_dir)/,,$$@)")
	$$(Q)$$(LD) $$(LDFLAGS) $$(ARCH_FLAGS) $$(build_objs) $$(build_dir)/$(1).o $(LDLIBS) -o $$@
endef

# Target application filenames .elf for each user app object
aelfs  = $(patsubst %.o,%.elf,$(aobjs))
abins  = $(patsubst %.o,%.bin,$(aobjs))
ahexs  = $(patsubst %.o,%.hex,$(aobjs))
build_aobjs = $(foreach aobj,$(aobjs),$(build_dir)/$(aobj))
build_aelfs = $(foreach aelf,$(aelfs),$(build_dir)/$(aelf))
build_abins = $(foreach abin,$(abins),$(build_dir)/$(abin))
build_ahexs = $(foreach ahex,$(ahexs),$(build_dir)/$(ahex))

# add a direct make target for every standalone app
$(foreach app,$(patsubst %.o,%,$(aobjs)),$(eval $(call build_app,$(app))))

# Build all test and user applications
all_bins	= $(build_tbins) $(build_abins)
all_hexs	= $(build_thexs) $(build_ahexs)
all_elfs	= $(build_telfs) $(build_aelfs)
all_objs	= $(build_tobjs) $(build_aobjs) $(build_tmobjs)

.PHONY: build_all
build_all: $(LIB_DIR)/lib$(LIBNAME).a $(all_bins) $(all_hexs) $(all_elfs) $(all_objs) Makefile $(board_dir)/Makefile.include

# Add build dependency for local libopencm3 if no external libopencm3 is used 
ifeq ($(build_lib),true)
$(LIB_DIR)/lib$(LIBNAME).a: 
	$(Q)if [ ! -d libopencm3 ] ; then \
		printf "######## ERROR ########\n"; \
		printf "\tlibopencm3 is not initialized.\n"; \
		printf "\tPlease run:\n"; \
		printf "\t$$ git submodule add https://github.com/libopencm3/libopencm3.git\n"; \
		printf "\t$$ git submodule init\n"; \
		printf "\t$$ git submodule update\n"; \
		printf "\tbefore running make.\n"; \
		printf "######## ERROR ########\n"; \
		exit 1; \
		fi
	$(Q)$(MAKE) -C libopencm3 V=$(V)
endif

$(build_dir)/%.bin: $(build_dir)/%.elf
	$(Q)mkdir -p `dirname $@`
	$(if $(Q), @echo " (OBJCOPY)   $(subst $(build_dir)/,,$@)")
	$(Q)$(OBJCOPY) -O binary $< $@

$(build_dir)/%.hex: $(build_dir)/%.elf
	$(Q)mkdir -p `dirname $@`
	$(if $(Q), @echo " (OBJCOPY)   $(subst $(build_dir)/,,$@)")
	$(Q)$(OBJCOPY) -O ihex $< $@

$(build_dir)/%.srec: $(build_dir)/%.elf
	$(Q)mkdir -p `dirname $@`
	$(if $(Q), @echo " (OBJCOPY)   $(subst $(build_dir)/,,$@)")
	$(Q)$(OBJCOPY) -O srec $< $@

$(build_dir)/%.list: $(build_dir)/%.elf
	$(Q)mkdir -p `dirname $@`
	$(if $(Q), @echo " (OBJDUMP)   $(subst $(build_dir)/,,$@)")
	$(Q)$(OBJDUMP) -S $< > $@

# This is the default rule for linking the test suite applications.
# User applications defined in a board Makefile fragment are linked
# by an explicitly generated rule.
$(build_dir)/%.elf $(build_dir)/%.map: $(LIB_DIR)/lib$(LIBNAME).a $(build_dir)/%.o $(build_tmobjs) $(LDSCRIPT)
	$(Q)mkdir -p `dirname $@`
	$(if $(Q), @echo " (ELF)       $(subst $(build_dir)/,,$@)")
	$(Q)$(LD) $(LDFLAGS) $(ARCH_FLAGS) $(build_tmobjs) $(build_dir)/$(*).o $(LDLIBS) -o $@

$(build_dir)/%.o: $(src_dir)/%.S Makefile $(board_dir)/Makefile.include
	$(Q)mkdir -p `dirname $@`
	$(if $(Q), @echo " (AS)        $(subst $(build_dir)/,,$@)")
	$(Q)$(CC) $(CFLAGS) $(CPPFLAGS) $(ARCH_FLAGS) $(ASFLAGS) -I`dirname $<` -c $< -o $@

$(build_dir)/%.o: $(src_dir)/%.c Makefile $(board_dir)/Makefile.include
	$(Q)mkdir -p `dirname $@`
	$(if $(Q), @echo " (CC)        $(subst $(build_dir)/,,$@)")
	$(Q)$(CC) $(CFLAGS) $(CPPFLAGS) $(ARCH_FLAGS) -I`dirname $<` -c $< -o $@

$(build_dir)/%.o: $(board_dir)/%.S Makefile $(board_dir)/Makefile.include
	$(Q)mkdir -p `dirname $@`
	$(if $(Q), @echo " (AS)        $(subst $(build_dir)/,,$@)")
	$(Q)$(CC) $(CFLAGS) $(CPPFLAGS) $(ARCH_FLAGS) $(ASFLAGS) -I`dirname $<` -c $< -o $@

$(build_dir)/%.o: $(board_dir)/%.c Makefile $(board_dir)/Makefile.include
	$(Q)mkdir -p `dirname $@`
	$(if $(Q), @echo " (CC)        $(subst $(build_dir)/,,$@)")
	$(Q)$(CC) $(CFLAGS) $(CPPFLAGS) $(ARCH_FLAGS) -I`dirname $<` -c $< -o $@

$(build_dir)/%.o: $(common_dir)/%.S Makefile $(board_dir)/Makefile.include
	$(Q)mkdir -p `dirname $@`
	$(if $(Q), @echo " (AS)        $(subst $(build_dir)/,,$@)")
	$(Q)$(CC) $(CFLAGS) $(CPPFLAGS) $(ARCH_FLAGS) $(ASFLAGS) -I`dirname $<` -c $< -o $@

$(build_dir)/%.o: $(common_dir)/%.c Makefile $(board_dir)/Makefile.include
	$(Q)mkdir -p `dirname $@`
	$(if $(Q), @echo " (CC)        $(subst $(build_dir)/,,$@)")
	$(Q)$(CC) $(CFLAGS) $(CPPFLAGS) $(ARCH_FLAGS) -I`dirname $<` -c $< -o $@

$(build_dir)/%.o: $(kernel_dir)/%.c Makefile $(board_dir)/Makefile.include
	$(Q)mkdir -p `dirname $@`
	$(if $(Q), @echo " (CC)        $(subst $(build_dir)/,,$@)")
	$(Q)$(CC) $(CFLAGS) $(CPPFLAGS) $(ARCH_FLAGS) -I`dirname $<` -c $< -o $@

$(build_dir)/%.o: $(tests_dir)/%.c Makefile $(board_dir)/Makefile.include
	$(Q)mkdir -p `dirname $@`
	$(if $(Q), @echo " (CC)        $(subst $(build_dir)/,,$@)")
	$(Q)$(CC) $(CFLAGS) $(CPPFLAGS) $(ARCH_FLAGS) -I`dirname $<` -c $< -o $@

# Clean. Remove only atomthread's object files and images
.PHONY: clean
clean:
	rm -rf doxygen-kernel
	rm -rf doxygen-opencm3
	rm -rf $(build_dir)

# Real clean. Also clean libopencm3 if it was built in tree
.PHONY: realclean
realclean: clean
ifeq ($(build_lib),true)
	$(Q)$(MAKE) -C libopencm3 V=$(V) clean
endif

# Docs
.PHONY: doxygen
doxygen:
	doxygen $(kernel_dir)/Doxyfile
ifeq ($(build_lib),true)
	$(Q)$(MAKE) -C libopencm3 V=$(V) doc
endif

#################################################################################
# Target flashing recipes "borrowed" from libopencm3-examples. Mostly untested. #
#################################################################################

%.stlink-flash: $(build_dir)/%.bin
	$(if $(Q), @echo " (FLASH)     $(subst $(build_dir)/,,$<)")
	$(Q)$(STFLASH) write $< 0x8000000

ifeq ($(STLINK_PORT),)
ifeq ($(BMP_PORT),)
ifeq ($(OOCD_SERIAL),)
%.flash: $(build_dir)/%.hex
	$(if $(Q), @echo " (FLASH)     $(subst $(build_dir)/,,$<)")
	@# IMPORTANT: Don't use "resume", only "reset" will work correctly!
	-$(Q)$(OOCD) -f interface/$(OOCD_INTERFACE).cfg \
		    -f board/$(OOCD_BOARD).cfg \
		    -c "init" -c "reset init" \
		    -c "flash write_image erase $<" \
		    -c "reset" \
		    -c "shutdown" $(NULL)
else
%.flash: $(build_dir)/%.hex
	$(if $(Q), @echo " (FLASH)     $(subst $(build_dir)/,,$<)")
	@# IMPORTANT: Don't use "resume", only "reset" will work correctly!
	-$(Q)$(OOCD) -f interface/$(OOCD_INTERFACE).cfg \
		    -f board/$(OOCD_BOARD).cfg \
		    -c "ft2232_serial $(OOCD_SERIAL)" \
		    -c "init" -c "reset init" \
		    -c "flash write_image erase $<" \
		    -c "reset" \
		    -c "shutdown" $(NULL)
endif
else
%.flash: $(build_dir)/%.elf
	$(if $(Q), @echo " (GDB)       $(subst $(build_dir)/,,$<)")
	$(Q)$(GDB) --batch \
		   -ex 'target extended-remote $(BMP_PORT)' \
		   -x $(SCRIPT_DIR)/black_magic_probe_flash.scr \
		   $<
endif
else
%.flash: $(build_dir)/%.elf
	$(if $(Q), @echo " (GDB)       $(subst $(build_dir)/,,$<)")
	$(Q)$(GDB) --batch \
		   -ex 'target extended-remote $(STLINK_PORT)' \
		   -x $(SCRIPT_DIR)/stlink_flash.scr \
		   $<
endif

# Include auto-generated dependencies
-include $(all_objs:.o=.d)

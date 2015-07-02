############
# Settings #
############

# Build all test applications:
#   make
#
# Run all tests communicating via UART
#   make tests

# Location of build tools and atomthreads sources
KERNEL_DIR=../../../../kernel
TESTS_DIR=../../../../tests
PORT_DIR=../..
CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
ARCHIVE=arm-none-eabi-ar

# Folder delete command (OS-specific)
ifeq ($(OS),Windows_NT)
	RMDIR=rd /s /q
else
	RMDIR=rm -rf
endif

# Location of TFTP root folder for running tests via U-Boot/TFTP.
# Note, you may need to run the Makefile as root in order to write
# to this folder.
TESTS_TFTPROOT=/var/lib/tftpboot

# TTY device and baudrate for automated tests.
TESTS_TTYDEV=/dev/ttyUSB0
TESTS_TTYBAUD=115200

# Enable stack-checking.
#STACK_CHECK=true

# Test programs: Log stack usage to UART (if STACK_CHECK is enabled)
#TESTS_LOG_STACK=true

# Directory for built objects
BUILD_DIR=build

# Platform-specific object files
PLATFORM_OBJECTS = atomport-private.o uart.o timer.o
PLATFORM_ASM_OBJECTS = startup.o

# Port-specific object files
PORT_OBJECTS = atomport.o tests-main.o syscalls.o
PORT_ASM_OBJECTS = atomport-asm.o

# Kernel object files
KERNEL_OBJECTS = atomkernel.o atomsem.o atommutex.o atomtimer.o atomqueue.o
KERNEL_OBJECTS += atomevent.o

# Collection of built objects (excluding test applications)
ALL_OBJECTS = $(PLATFORM_OBJECTS) $(PLATFORM_ASM_OBJECTS) $(PORT_OBJECTS) $(PORT_ASM_OBJECTS) $(KERNEL_OBJECTS)
BUILT_OBJECTS = $(patsubst %,$(BUILD_DIR)/%,$(ALL_OBJECTS))

# Test object files (dealt with separately as only one per application build)
TEST_OBJECTS = $(notdir $(patsubst %.c,%.o,$(wildcard $(TESTS_DIR)/*.c)))

# Target application filenames for each test object
TEST_UIMAGES = $(patsubst %.o,%.uImage,$(TEST_OBJECTS))
TEST_BINS = $(patsubst %.o,%.bin,$(TEST_OBJECTS))
TEST_ELFS = $(patsubst %.o,%.elf,$(TEST_OBJECTS))

# Search build/output directory for dependencies
vpath %.o ./$(BUILD_DIR)
vpath %.elf ./$(BUILD_DIR)

# GCC flags
CFLAGS=-c -mcpu=arm926ej-s -ffreestanding -Wall -Werror -Wno-unused-but-set-variable
AFLAGS=$(CFLAGS) -x assembler-with-cpp
LFLAGS=-mcpu=arm926ej-s -Tsystem.ld -Wall

# Enable stack-checking options (disable if not required)
ifeq ($(STACK_CHECK),true)
CFLAGS += -DATOM_STACK_CHECKING
endif
ifeq ($(TESTS_LOG_STACK),true)
CFLAGS += -DTESTS_LOG_STACK_USAGE
endif


#################
# Build targets #
#################

# All tests
all: $(BUILD_DIR) $(TEST_ELFS) $(TEST_BINS) $(TEST_UIMAGES) Makefile

# Build archive for linking with external application
libatomthreads.a: $(BUILD_DIR) $(ALL_OBJECTS) Makefile
	$(ARCHIVE) cr $(BUILD_DIR)/$@ $(BUILT_OBJECTS)

# Make build/output directory
$(BUILD_DIR):
	mkdir $(BUILD_DIR)

# Test ELF files (one application build for each test)
$(TEST_ELFS): %.elf: %.o $(ALL_OBJECTS)
	$(CC) $(LFLAGS) $(BUILD_DIR)/$(notdir $<) $(BUILT_OBJECTS) --output $(BUILD_DIR)/$@ -Wl,-Map,$(BUILD_DIR)/$(basename $@).map

# Test BIN files (one application build for each test)
$(TEST_BINS): %.bin: %.elf $(TEST_ELFS)
	$(OBJCOPY) -O binary $(BUILD_DIR)/$(basename $@).elf $(BUILD_DIR)/$(basename $@).bin

# Test uImage files (one application build for each test)
$(TEST_UIMAGES): %.uImage: %.bin $(TEST_BINS)
	gzip -9 -c $(BUILD_DIR)/$(basename $@).bin > $(BUILD_DIR)/$(basename $@).bin.gz
	mkimage -A arm -T kernel -C gzip -a 0x80000000 -e 0x80000000 -n "Atomthreads application" -d $(BUILD_DIR)/$(basename $@).bin.gz $(BUILD_DIR)/$(basename $@).uImage

# Kernel objects builder
$(KERNEL_OBJECTS): %.o: $(KERNEL_DIR)/%.c
	$(CC) -c $(CFLAGS) -I. -I$(PORT_DIR) $< -o $(BUILD_DIR)/$(notdir $@)

# Test objects builder
$(TEST_OBJECTS): %.o: $(TESTS_DIR)/%.c
	$(CC) -c $(CFLAGS) -I. -I$(PORT_DIR) -I$(KERNEL_DIR) $< -o $(BUILD_DIR)/$(notdir $@)

# Platform C objects builder
$(PLATFORM_OBJECTS): %.o: ./%.c
	$(CC) -c $(CFLAGS) -I. -I$(PORT_DIR) -I$(KERNEL_DIR) -I$(TESTS_DIR) $< -o $(BUILD_DIR)/$(notdir $@)

# Platform asm objects builder
$(PLATFORM_ASM_OBJECTS): %.o: ./%.s
	$(CC) -c $(AFLAGS) -I. -I$(PORT_DIR) -I$(KERNEL_DIR) $< -o $(BUILD_DIR)/$(notdir $@)

# Port C objects builder
$(PORT_OBJECTS): %.o: $(PORT_DIR)/%.c
	$(CC) -c $(CFLAGS) -I. -I$(PORT_DIR) -I$(KERNEL_DIR) -I$(TESTS_DIR) $< -o $(BUILD_DIR)/$(notdir $@)

# Port asm objects builder
$(PORT_ASM_OBJECTS): %.o: $(PORT_DIR)/%.s
	$(CC) -c $(AFLAGS) -I. -I$(PORT_DIR) -I$(KERNEL_DIR) $< -o $(BUILD_DIR)/$(notdir $@)

# .lst file builder
%.lst: %.c
	$(CC) $(CFLAGS) -I. -I$(PORT_DIR) -I$(KERNEL_DIR) -I$(TESTS_DIR) -Wa,-al $< > $@

# Clean
clean:
	$(RMDIR) build
	$(RMDIR) doxygen-kernel
	$(RMDIR) doxygen-arm
	$(RMDIR) doxygen-platform

# Generate Doxygen documentation
doxygen:
	doxygen $(KERNEL_DIR)/Doxyfile
	doxygen ../../Doxyfile
	doxygen ./Doxyfile

# Run tests on target with expect and serial output
phony_test_bins = $(addsuffix .sim, $(TEST_BINS))
tests: $(phony_test_bins)
.PHONY: tests $(phony_test_bins)
$(phony_test_bins):
	cp $(BUILD_DIR)/$(basename $@) $(TESTS_TFTPROOT)/test.bin
	@echo Running test $(basename $@)
	./run_test.exp $(TESTS_TTYDEV) $(TESTS_TTYBAUD)

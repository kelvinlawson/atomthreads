############
# Settings #
############

# Build all test applications:
#   make
#
# Run all tests within QEMU
#   make qemutests

# Location of build tools and atomthreads sources
KERNEL_DIR=../../../../kernel
TESTS_DIR=../../../../tests
PORT_DIR=../..
CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
ARCHIVE=arm-none-eabi-ar
QEMU=qemu-system-arm

# Enable stack-checking.
#STACK_CHECK=true

# Test programs: Log stack usage to UART (if STACK_CHECK is enabled)
#TESTS_LOG_STACK=true

# Directory for built objects
BUILD_DIR=build

# Platform-specific object files
PLATFORM_OBJECTS = modules.o uart.o
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
TEST_ELFS = $(patsubst %.o,%.elf,$(TEST_OBJECTS))

# Search build/output directory for dependencies
vpath %.o ./$(BUILD_DIR)
vpath %.elf ./$(BUILD_DIR)

# GCC flags
CFLAGS=-g -c -mcpu=arm926ej-s -ffreestanding -Wall -Werror
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
all: $(BUILD_DIR) $(TEST_ELFS) Makefile

# Build archive for linking with external application
libatomthreads.a: $(BUILD_DIR) $(ALL_OBJECTS) Makefile
	$(ARCHIVE) cr $(BUILD_DIR)/$@ $(BUILT_OBJECTS)

# Make build/output directory
$(BUILD_DIR):
	mkdir $(BUILD_DIR)

# Test ELF files (one application build for each test)
$(TEST_ELFS): %.elf: %.o $(ALL_OBJECTS)
	$(CC) $(LFLAGS) $(BUILD_DIR)/$(notdir $<) $(BUILT_OBJECTS) --output $(BUILD_DIR)/$@ -Wl,-Map,$(BUILD_DIR)/$(basename $@).map

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
	rm -f *.o *.elf *.map *.lst
	rm -rf doxygen-kernel
	rm -rf doxygen-arm
	rm -rf doxygen-platform
	rm -rf build

# Generate Doxygen documentation
doxygen:
	doxygen $(KERNEL_DIR)/Doxyfile
	doxygen ../../Doxyfile
	doxygen ./Doxyfile

# Run tests within QEMU simulator
phony_qemu_elfs = $(addsuffix .sim, $(TEST_ELFS))
qemutests: $(phony_qemu_elfs)
.PHONY: qemutests $(phony_qemu_elfs)
$(phony_qemu_elfs):
	./run_test.exp $(QEMU) $(BUILD_DIR)/$(basename $@)

############
# Settings #
############

# Build all test applications:
#   make

# Location of build tools and atomthreads sources
KERNEL_DIR=../../../../kernel
TESTS_DIR=../../../../tests
PORT_DIR=../..
CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy

# Enable stack-checking.
#STACK_CHECK=true

# Test programs: Log stack usage to UART (if STACK_CHECK is enabled)
#TESTS_LOG_STACK=true

# Directory for built objects
BUILD_DIR=build

# Platform-specific object files
PLATFORM_OBJECTS = atomport-private.o uart.o
PLATFORM_ASM_OBJECTS = startup.o

# Port-specific object files
PORT_OBJECTS = atomport.o tests-main.o syscalls.o
PORT_ASM_OBJECTS = atomport-asm.o

# Kernel object files
KERNEL_OBJECTS = atomkernel.o atomsem.o atommutex.o atomtimer.o atomqueue.o

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
all: $(BUILD_DIR) $(TEST_ELFS) $(TEST_BINS) $(TEST_UIMAGES) Makefile

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


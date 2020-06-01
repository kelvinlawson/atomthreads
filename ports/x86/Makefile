############
# Settings #
############

# Build all test applications:
#   make
#
# Run all tests communicating via UART
#   make tests

# Location of build tools and atomthreads sources
KERNEL_DIR=../../kernel
TESTS_DIR=../../tests
PORT_DIR=.
CC=i686-elf-gcc
OBJCOPY=i686-elf-objcopy
ARCHIVE=i686-elf-ar

# Folder delete command (OS-specific)
ifeq ($(OS),Windows_NT)
	RMDIR=rd /s /q
else
	RMDIR=rm -rf
endif

# Enable stack-checking.
#STACK_CHECK=true

# Directory for built objects
BUILD_DIR=build
PLAT_DIR=plat
PORT_UTIL_DIR=utils
# Platform-specific object files
PLAT_OBJECTS = irq_handlers.o pic.o pit.o terminal.o 

# Port-specific object files
PORT_OBJECTS = atomport.o c_traps.o main.o gdt.o
PORT_ASM_OBJECTS = atomport-asm.o  boot.o traps.o
PORT_UTIL_OBJECTS = string.o printf.o

# Kernel object files
KERNEL_OBJECTS = atomkernel.o atomsem.o atommutex.o atomtimer.o atomqueue.o

# Collection of built objects (excluding test applications)
ALL_OBJECTS = $(PLAT_OBJECTS) $(PLAT_ASM_OBJECTS) $(PORT_OBJECTS) $(PORT_ASM_OBJECTS) $(PORT_UTIL_OBJECTS) $(KERNEL_OBJECTS)
BUILT_OBJECTS = $(patsubst %,$(BUILD_DIR)/%,$(ALL_OBJECTS))

# Test object files (dealt with separately as only one per application build)
TEST_OBJECTS = $(notdir $(patsubst %.c,%.o,$(wildcard $(TESTS_DIR)/*.c)))
TEST_ALL = $(patsubst %.o,%,$(TEST_OBJECTS))

# Search build/output directory for dependencies
vpath %.o ./$(BUILD_DIR)

# GCC flags
CFLAGS=-c -s -ffreestanding -std=gnu99 -Wall -Wextra -Werror \
       -Wno-unused-parameter \
       -Wno-unused-but-set-variable \
       -Wno-unused-variable \
       -Wno-sign-compare
       #-O2 

AFLAGS=$(CFLAGS) -x assembler-with-cpp
LFLAGS=-Tlinker.ld -Wall -ffreestanding -nostdlib 

# Enable stack-checking options (disable if not required)
ifeq ($(STACK_CHECK),true)
CFLAGS += -DATOM_STACK_CHECKING
endif
ifeq ($(TESTS_LOG_STACK),true)
CFLAGS += -DTESTS_LOG_STACK_USAGE -I$(PORT_UTIL_DIR) 
endif


#################
# Build targets #
#################

main: $(BUILD_DIR) $(TEST_ALL)
	
$(TEST_ALL): $(TEST_OBJECTS) $(BUILD_DIR) $(ALL_OBJECTS) 
	$(CC) $(LFLAGS) $(BUILT_OBJECTS) $(BUILD_DIR)/$(notdir $@.o) --output $(BUILD_DIR)/$@.bin

# Build archive for linking with external application
libatomthreads.a: $(BUILD_DIR) $(ALL_OBJECTS) Makefile
	$(ARCHIVE) cr $(BUILD_DIR)/$@ $(BUILT_OBJECTS)

# Make build/output directory
$(BUILD_DIR):
	mkdir $(BUILD_DIR)
	
# Kernel objects builder
$(KERNEL_OBJECTS): %.o: $(KERNEL_DIR)/%.c
	$(CC) -c $(CFLAGS) -I. -I$(PORT_DIR) $< -o $(BUILD_DIR)/$(notdir $@)

# Test objects builder
$(TEST_OBJECTS): %.o: $(TESTS_DIR)/%.c
	$(CC) -c $(CFLAGS) -I. -I$(PORT_DIR)  -I$(PORT_UTIL_DIR) -I$(KERNEL_DIR) $< -o $(BUILD_DIR)/$(notdir $@)

# Platform C objects builder
$(PLAT_OBJECTS): %.o: $(PLAT_DIR)/%.c
	$(CC) -c $(CFLAGS) -I. -I$(PORT_DIR) -I$(KERNEL_DIR) -I$(PORT_UTIL_DIR) -I$(TESTS_DIR) $< -o $(BUILD_DIR)/$(notdir $@)

# Platform asm objects builder
$(PLAT_ASM_OBJECTS): %.o: $(PLAT_DIR)/%.s
	$(CC) -c $(AFLAGS) -I. -I$(PORT_DIR) -I$(KERNEL_DIR) $< -o $(BUILD_DIR)/$(notdir $@)

# Port C objects builder
$(PORT_OBJECTS): %.o: $(PORT_DIR)/%.c
	$(CC) -c $(CFLAGS) -I. -Imachine/ -I$(PORT_DIR) -I$(KERNEL_DIR) -I$(PORT_UTIL_DIR) -I$(TESTS_DIR) $< -o $(BUILD_DIR)/$(notdir $@)

# Port asm objects builder
$(PORT_ASM_OBJECTS): %.o: $(PORT_DIR)/%.s
	$(CC) -c $(AFLAGS) -I. -I$(PORT_DIR) -I$(KERNEL_DIR) $< -o $(BUILD_DIR)/$(notdir $@)

# Port C objects builder
$(PORT_UTIL_OBJECTS): %.o: $(PORT_UTIL_DIR)/%.c
	$(CC) -c $(CFLAGS) -I../ -I$(PORT_DIR) -I$(KERNEL_DIR) -I$(PORT_UTIL_DIR) -I$(TESTS_DIR) $< -o $(BUILD_DIR)/$(notdir $@)

# Clean
clean:
	$(RMDIR) build/*


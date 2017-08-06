KERNEL_DIR=../../kernel
TESTS_DIR=../../tests
PERIPHS_DIR=stm8s-periphs

CC=sdcc
ASM=sdasstm8
LINK=sdcc

# CPU part number
PART=STM8S105

# Enable stack-checking
STACK_CHECK=true

# Directory for built objects
BUILD_DIR=build-sdcc

# Port/application object files
APP_OBJECTS = atomport.rel tests-main.rel uart.rel
APP_ASM_OBJECTS = atomport-asm-sdcc.rel

# STM8S Peripheral driver object files
PERIPH_OBJECTS = stm8s_gpio.rel stm8s_tim1.rel stm8s_clk.rel stm8s_uart2.rel

# Kernel object files
KERNEL_OBJECTS = atomkernel.rel atomsem.rel atommutex.rel atomtimer.rel atomqueue.rel

# Collection of built objects (excluding test applications)
ALL_OBJECTS = $(APP_OBJECTS) $(APP_ASM_OBJECTS) $(PERIPH_OBJECTS) $(KERNEL_OBJECTS)
BUILT_OBJECTS = $(patsubst %,$(BUILD_DIR)/%,$(ALL_OBJECTS))

# Test object files (dealt with separately as only one per application build)
TEST_OBJECTS = $(notdir $(patsubst %.c,%.rel,$(wildcard $(TESTS_DIR)/*.c)))

# Target application filenames (.elf) for each test object
TEST_HEXS = $(patsubst %.rel,%.ihx,$(TEST_OBJECTS))
TEST_ELFS = $(patsubst %.rel,%.elf,$(TEST_OBJECTS))

# Search build/output directory for dependencies
vpath %.rel .\$(BUILD_DIR)
vpath %.elf .\$(BUILD_DIR)
vpath %.hex .\$(BUILD_DIR)

# Compiler/Assembler flags
CFLAGS= -mstm8 -c -D $(PART) --opt-code-size
DBG_CFLAGS= -mstm8 -c -D $(PART) --opt-code-size
ASMFLAGS= -loff
DBG_ASMFLAGS= -loff
LINKFLAGS= -mstm8
DBG_LINKFLAGS= --debug -mstm8

# Enable stack-checking (disable if not required)
ifeq ($(STACK_CHECK),true)
CFLAGS += -D ATOM_STACK_CHECKING
DBG_CFLAGS += --debug -D ATOM_STACK_CHECKING
endif

#################
# Build targets #
#################

# All tests
all: $(BUILD_DIR) $(TEST_HEXS) sdcc.mak

# Make build/output directory
$(BUILD_DIR):
	mkdir $(BUILD_DIR)

# Test HEX files (one application build for each test)
$(TEST_HEXS): %.ihx: %.rel $(KERNEL_OBJECTS) $(PERIPH_OBJECTS) $(APP_OBJECTS) $(APP_ASM_OBJECTS)
	$(LINK) $(BUILD_DIR)/$(notdir $<) $(BUILT_OBJECTS) $(LINKFLAGS) -o $(BUILD_DIR)/$@

# Test ELF files (one application build for each test)
$(TEST_ELFS): %.elf: %.rel $(KERNEL_OBJECTS) $(PERIPH_OBJECTS) $(APP_OBJECTS) $(APP_ASM_OBJECTS)
	$(LINK) $(BUILD_DIR)/$(notdir $<) $(BUILT_OBJECTS) $(LINKFLAGS) --out-fmt-elf -o $(BUILD_DIR)/$@

# Kernel objects builder
$(KERNEL_OBJECTS): %.rel: $(KERNEL_DIR)/%.c
	$(CC) $< $(CFLAGS) -I . -I $(PERIPHS_DIR) -o $(BUILD_DIR)/$*.rel

# Test objects builder
$(TEST_OBJECTS): %.rel: $(TESTS_DIR)/%.c
	$(CC) $< $(CFLAGS) -I . -I $(KERNEL_DIR) -I $(PERIPHS_DIR) -o $(BUILD_DIR)/$*.rel

# Peripheral objects builder
$(PERIPH_OBJECTS): %.rel: $(PERIPHS_DIR)/%.c
	$(CC) $< $(CFLAGS) -I . -I $(PERIPHS_DIR) -o $(BUILD_DIR)/$*.rel

# Application C objects builder
$(APP_OBJECTS): %.rel: ./%.c
	$(CC) $< $(CFLAGS) -I . -I $(KERNEL_DIR) -I $(TESTS_DIR) -I $(PERIPHS_DIR) -o $(BUILD_DIR)/$*.rel

# Application asm objects builder
$(APP_ASM_OBJECTS): %.rel: ./%.s
	$(ASM) $(ASMFLAGS) $(BUILD_DIR)/$(notdir $@) $<

# Clean
clean:
	rm -f *.o *.elf *.map *.hex *.bin *.lst *.stm8 *.s19
	rm -rf doxygen-kernel
	rm -rf doxygen-stm8
	rm -rf build-sdcc

doxygen:
	doxygen $(KERNEL_DIR)/Doxyfile
	doxygen ./Doxyfile


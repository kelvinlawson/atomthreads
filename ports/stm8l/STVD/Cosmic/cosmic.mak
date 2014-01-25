############
# Settings #
############

# Set up build environment (using GNU Make)
#   set PATH=%PATH%;C:\Program Files\GNU_MAKE;C:\Program Files\COSMIC\CXSTM8_16K
#   set MAKE_MODE=DOS

# Build all test applications:
#   make -f cosmic.mak


# Location of build tools and atomthreads sources
KERNEL_DIR=../../kernel
TESTS_DIR=../../tests
PERIPHS_DIR=stm8s-periphs
LIBS_DIR="C:\Program Files\COSMIC\CXSTM8_16K\Lib"
CC=cxstm8
ASM=castm8
LINK=clnk
CHEX=chex

# CPU part number
PART=STM8S105

# Enable stack-checking
STACK_CHECK=true

# Directory for built objects
BUILD_DIR=build-cosmic

# Port/application object files
APP_OBJECTS = atomport.o tests-main.o stm8_interrupt_vector.o uart.o
APP_ASM_OBJECTS = atomport-asm-cosmic.o

# STM8S Peripheral driver object files
PERIPH_OBJECTS = stm8s_gpio.o stm8s_tim1.o stm8s_clk.o stm8s_uart2.o

# Kernel object files
KERNEL_OBJECTS = atomkernel.o atomsem.o atommutex.o atomtimer.o atomqueue.o

# Collection of built objects (excluding test applications)
ALL_OBJECTS = $(APP_OBJECTS) $(APP_ASM_OBJECTS) $(PERIPH_OBJECTS) $(KERNEL_OBJECTS)
BUILT_OBJECTS = $(patsubst %,$(BUILD_DIR)/%,$(ALL_OBJECTS))

# Test object files (dealt with separately as only one per application build)
TEST_OBJECTS = $(notdir $(patsubst %.c,%.o,$(wildcard $(TESTS_DIR)/*.c)))

# Target application filenames (.elf and .hex) for each test object
TEST_STM8S = $(patsubst %.o,%.stm8,$(TEST_OBJECTS))
TEST_S19S = $(patsubst %.o,%.s19,$(TEST_OBJECTS))

# Search build/output directory for dependencies
vpath %.o .\$(BUILD_DIR)
vpath %.elf .\$(BUILD_DIR)
vpath %.hex .\$(BUILD_DIR)

# Compiler/Assembler flags
CFLAGS=+modsl0 +split -pp -d$(PART)
DBG_CFLAGS=+modsl0 +split +debug -pxp -no -pp -l -d$(PART)
ASMFLAGS=
DBG_ASMFLAGS=-xx -u

# Enable stack-checking (disable if not required)
ifeq ($(STACK_CHECK),true)
CFLAGS += -dATOM_STACK_CHECKING
DBG_CFLAGS += -dATOM_STACK_CHECKING
endif


#################
# Build targets #
#################

# All tests
all: $(BUILD_DIR) $(TEST_S19S) cosmic.mak

# Make build/output directory
$(BUILD_DIR):
	mkdir $(BUILD_DIR)

# Test HEX files (one application build for each test)
$(TEST_S19S): %.s19: %.stm8
	@echo Building $@
	$(CHEX) -fm -o $(BUILD_DIR)/$@ $(BUILD_DIR)/$<

# Test ELF files (one application build for each test)
$(TEST_STM8S): %.stm8: %.o $(KERNEL_OBJECTS) $(PERIPH_OBJECTS) $(APP_OBJECTS) $(APP_ASM_OBJECTS)
	$(LINK) -l$(LIBS_DIR) -o $(BUILD_DIR)/$@ -m $(BUILD_DIR)/$(basename $@).map atomthreads.lkf $(BUILD_DIR)/$(notdir $<)

# Kernel objects builder
$(KERNEL_OBJECTS): %.o: $(KERNEL_DIR)/%.c
	$(CC) $(CFLAGS) -i. -i$(PERIPHS_DIR) -co$(BUILD_DIR) $<

# Test objects builder
$(TEST_OBJECTS): %.o: $(TESTS_DIR)/%.c
	$(CC) $(CFLAGS) -i. -i$(KERNEL_DIR) -i$(PERIPHS_DIR) -co$(BUILD_DIR) $<

# Peripheral objects builder
$(PERIPH_OBJECTS): %.o: $(PERIPHS_DIR)/%.c
	$(CC) $(CFLAGS) -i. -i$(PERIPHS_DIR) -co$(BUILD_DIR) $<

# Application C objects builder
$(APP_OBJECTS): %.o: ./%.c
	$(CC) $(CFLAGS) -i. -i$(KERNEL_DIR) -i$(TESTS_DIR) -i$(PERIPHS_DIR) -co$(BUILD_DIR) $<

# Application asm objects builder
$(APP_ASM_OBJECTS): %.o: ./%.s
	$(ASM) $(ASMFLAGS) -i. -i$(KERNEL_DIR) -o$(BUILD_DIR)/$(notdir $@) $<

# Clean
clean:
	rm -f *.o *.elf *.map *.hex *.bin *.lst *.stm8 *.s19
	rm -rf doxygen-kernel
	rm -rf doxygen-stm8
	rm -rf build-cosmic

doxygen:
	doxygen $(KERNEL_DIR)/Doxyfile
	doxygen ./Doxyfile

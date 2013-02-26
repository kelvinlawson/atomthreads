ifeq ($(TARGET_NAME),)
TARGET_NAME=boot
endif
ifeq ($(ATOMTHREADS),)
ATOMTHREADS = $(shell pwd)/../../
endif
ifeq ($(TEST_NAME),)
TEST_NAME = kern1
endif



CC = arm-none-eabi-gcc
LN = arm-none-eabi-gcc
AS = arm-none-eabi-gcc

CFLAGS := $(CFLAGS) -Wall -g -c -mcpu=arm926ej-s -ffreestanding 
AFLAGS := $(CFLAGS) -Wall -g -c -mcpu=arm926ej-s -ffreestanding 
LFLAGS := $(LFLAGS) -Wall -mcpu=arm926ej-s -Wl,-Map=system.map -Tsystem.ld 

CDEFS := $(CDEFS)  -DATOMTHREADS_TEST='"$(TEST_NAME)"' 
ADEFS := $(ADEFS)

LLIBS := $(LLIBS) 


SRCS := $(SRCS) \
		modules.c \
		syscalls.c \
		main.c \
		$(ATOMTHREADS)/tests/$(TEST_NAME).c

ASMS := $(ASMS) \
		startup.S \

INCLUDES := $(INCLUDES) \
			-I$(ATOMTHREADS)

include $(ATOMTHREADS)/ports/arm/Makefile

OBJS = $(SRCS:.c=.o) $(ASMS:.S=.o)

include 	../rules.mk


run_test: clean all
	echo "START TEST $(TEST_NAME)"
	qemu-system-arm -M integratorcp -kernel boot.elf  -semihosting | tee atomthreads_test.out

all_tests:
	echo "Starting atomthreads test suite" > atomthreads_test.out
	make run_test "TEST_NAME=mutex1"
	make run_test "TEST_NAME=mutex2"
	make run_test "TEST_NAME=mutex3"
	make run_test "TEST_NAME=mutex4"
	make run_test "TEST_NAME=mutex5"
	make run_test "TEST_NAME=mutex6"
	make run_test "TEST_NAME=mutex7"
	make run_test "TEST_NAME=mutex8"
	make run_test "TEST_NAME=mutex9"
	make run_test "TEST_NAME=kern1"
	make run_test "TEST_NAME=kern2"
	make run_test "TEST_NAME=kern3"
	make run_test "TEST_NAME=kern4"
	make run_test "TEST_NAME=timer1"
	make run_test "TEST_NAME=timer2"
	make run_test "TEST_NAME=timer3"
	make run_test "TEST_NAME=timer4"
	make run_test "TEST_NAME=timer5"
	make run_test "TEST_NAME=timer6"
	make run_test "TEST_NAME=timer7"
	make run_test "TEST_NAME=queue1"
	make run_test "TEST_NAME=queue2"
	make run_test "TEST_NAME=queue3"
	make run_test "TEST_NAME=queue4"
	make run_test "TEST_NAME=queue5"
	make run_test "TEST_NAME=queue6"
	make run_test "TEST_NAME=queue7"
	make run_test "TEST_NAME=queue8"
	make run_test "TEST_NAME=queue9"
	make run_test "TEST_NAME=queue10"
	make run_test "TEST_NAME=sem1"
	make run_test "TEST_NAME=sem2"
	make run_test "TEST_NAME=sem3"
	make run_test "TEST_NAME=sem4"
	make run_test "TEST_NAME=sem5"
	make run_test "TEST_NAME=sem6"
	make run_test "TEST_NAME=sem7"
	make run_test "TEST_NAME=sem8"
	make run_test "TEST_NAME=sem9"

run_last:
	qemu-system-arm -M integratorcp -kernel boot.elf   -monitor stdio -semihosting


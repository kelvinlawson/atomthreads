ATOMTHREADS_PORT = .../libraries/atomthreads/ports/armv7
ATOMTHREADS_KERNEL = .../libraries/atomthreads/kernel

INCLUDES := $(INCLUDES) \
			-I$(ATOMTHREADS_KERNEL) \
			-I$(ATOMTHREADS_PORT)
			
SRCS := $(SRCS) \
		$(ATOMTHREADS_KERNEL)/atomkernel.c \
		$(ATOMTHREADS_KERNEL)/atommutex.c \
		$(ATOMTHREADS_KERNEL)/atomqueue.c \
		$(ATOMTHREADS_KERNEL)/atomsem.c \
		$(ATOMTHREADS_KERNEL)/atomtimer.c \
		$(ATOMTHREADS_PORT)/atomport.c

ASMS := $(ASMS) \
		$(ATOMTHREADS_PORT)/atomport_arm.asm
			
		

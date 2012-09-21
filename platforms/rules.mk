#
# The following part of the makefile is generic; it can be used to 
# build any executable just by changing the definitions above and by
# deleting dependencies appended to the file from 'make depend'
#
.SUFFIXES: .asm .elf .hex .lst .o .S .s .c .cpp
.PHONY: depend clean

dump:
		@echo "Target: "
		@echo $(TARGET_NAME)
		@echo "Source files: "
		@echo  $(SRCS)
		@echo  $(ASMS)
		@echo "Object files: "
		@echo  $(OBJS)

all:  target 

target: $(OBJS)
		$(LN) $(LFLAGS)  $(LIBFLAGS) $(OBJS) $(LLIBS) -o $(TARGET_NAME).elf
		@echo $(TARGET_NAME).elf was compiled

clean:
		rm -f $(OBJS)


# this is a suffix replacement rule for building .o's from .c's
# it uses automatic variables $<: the name of the prerequisite of
# the rule(a .c file) and $@: the name of the target of the rule (a .o file) 
# (see the gnu make manual section about automatic variables)

.c.o:
		$(CC) $(CDEFS) $(CFLAGS) $(INCLUDES) -c $<  -o  $@
		
.cpp.o:
		$(CC) $(CDEFS) $(CFLAGS) $(INCLUDES) -c $<  -o  $@
		
.S.o:
		$(AS) $(ADEFS) $(AFLAGS) $(INCLUDES) -c $<  -o  $@

.s.o:
		$(AS) $(ADEFS) $(AFLAGS) $(INCLUDES) -c $<  -o  $@

DEPFILE=.depends
DEPTOKEN='\# MAKEDEPENDS'
DEPFLAGS=-Y -f $(DEPFILE) -s $(DEPTOKEN) -p $(OUTDIR)/


depend:
	rm -f $(DEPFILE)
	make $(DEPFILE)

$(DEPFILE):
	@echo $(DEPTOKEN) > $(DEPFILE)
	makedepend $(DEPFLAGS) -- $(CFLAGS) -- $(SRCS) >&/dev/null
	
# put this file in the last line of your Makefile
sinclude $(DEPFILE)
 
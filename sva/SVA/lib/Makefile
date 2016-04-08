#
# Makefile
#

#
# Include make.conf to configure the locations of the compiler
#
include ../../make.conf

CFLAGS := -I../include -v -target armv6hf-gnueabihf-freebsd11.0 -O2 -mcmodel=kernel -msoft-float -ffreestanding

OFILES = init.o secmem.o handlers.o mmu.o interrupt.o state.o debug.o \
         stateasm.o invoke.o invokeasm.o checks.o keys.o

libsva.a: $(OFILES)
	$(AR) -r $@ $(OFILES)

clean:
	rm -f *.a *.o


CSOURCES= $(shell find -name *.c)
CXXSOURCES= $(shell find -name *.cpp ! -wholename '*uacpi/*')
ASMSOURCES= $(shell find -name *.asm)
OBJS=$(CSOURCES:.c=.o) $(CXXSOURCES:.cpp=.o) $(ASMSOURCES:.asm=.asmo)
TARGET=x86-64
INCDIR=$(shell pwd)/include
ARCHINCDIR=$(shell pwd)/include/$(TARGET)/
ARCHDIR=$(shell pwd)/arch/$(TARGET)/
LDFLAGS=-ffreestanding -nostdlib -lgcc -Wl,-T,"$(ARCHDIR)/kernel.ld" -g
# required for mpaland printf so it doesn't compile with floating point support, which results in build failures
PRINTFOPTS=-DPRINTF_DISABLE_SUPPORT_FLOAT
# flanterm includes break if the headers are moved into the include dir in the tree
FLANTERMINCDIR=$(shell pwd)/flanterm
UACPIINCDIR=$(shell pwd)/io/acpi/uacpi/include
UACPIOPTS=-DUACPI_OVERRIDE_LIBC -DUACPI_KERNEL_INITIALIZATION
UHDAINCDIR=$(shell pwd)/io/audio/uhda/include
KERNELCONFIG=#-DX86_64_ENABLE_E9 -DSYSCALL_LOGGING
CFLAGS=-g -ffreestanding -mcmodel=kernel -O2 -mno-red-zone -mgeneral-regs-only -mno-mmx -mno-sse -mno-sse2 -nostdlib -Wall -MMD -I "${INCDIR}" -I "$(ARCHINCDIR)" -I "$(FLANTERMINCDIR)" -I "$(UACPIINCDIR)" -I "$(UHDAINCDIR)" $(UACPIOPTS) $(PRINTFOPTS) $(KERNELCONFIG)
CXXFLAGS=$(CFLAGS) -fno-exceptions -fno-rtti -std=c++20
ASFLAGS=-felf64
DEPS=$(CSOURCES:.c=.d) $(CXXSOURCES:.cpp=.d)

.PHONY: all
all:
	# build build.h file
	echo "#ifndef _BUILD_H" > include/build.h 
	echo "#define _BUILD_H" >> include/build.h 
	echo "	#define BUILD_DATE \"$(shell date -u)\"" >> include/build.h
	echo "	#define BUILD_TARGET \"$(TARGET)\"" >> include/build.h
	echo "#endif" >> include/build.h 
	make astral

astral:	$(OBJS)
	@echo [LD] $@
	@$(LD) $(LDFLAGS) -o $@ $^

%.o: %.c
	@echo [CC] $@
	@$(CC) $(CFLAGS) -c -o $@ $<

%.o: %.cpp
	@echo [CXX] $@
	@$(CXX) $(CXXFLAGS) -c -o $@ $<

%.asmo: %.asm
	@echo [AS] $@
	@nasm $(ASFLAGS) -o $@ $<

-include $(DEPS)

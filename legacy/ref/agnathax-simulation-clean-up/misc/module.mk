RELATIVE_ROOT = ../..
include $(RELATIVE_ROOT)/util.mk

CSRC = $(lastword $(subst /, ,$(CURDIR))).c
CFLAGS = -Wall -fPIC
OBJ = $(CSRC:.c=.o)
DEP = $(CSRC:.c=.d)

PLATFORMS = linux

windows/$(OBJ) windows/$(DEP) : CC = i586-mingw32msvc-cc
windows/$(OBJ) windows/$(DEP) : CFLAGS = -Wall

robot/$(OBJ) robot/$(DEP) : CC = arm-elf-gcc
robot/$(OBJ) robot/$(DEP) : CFLAGS = -Wall

all: $(PLATFORMS:%=%/$(OBJ))

.PRECIOUS: %/stamp
%/stamp:
	mkdir -p $(@D) && touch $@

clean:
	-rm -rf windows linux robot

ifneq ($(MAKECMDGOALS),clean)
-include $(PLATFORMS:%=%/$(DEP))
endif

CC = /usr/bin/g++

.SECONDEXPANSION:
%.d: $$(notdir $$*.c) $$(dir $$@)stamp
	@$(CC) -MM -MF $@ -MT $@ -MT $(@:.d=.o) -MP $(CFLAGS) $<
$(PLATFORMS:%=%/$(OBJ)): $$(dir $$@)stamp
	$(CC) $(CFLAGS) $(@F:.o=.c) -c -o $@

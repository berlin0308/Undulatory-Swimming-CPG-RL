LIB_NAME = misc
MODULES = history log gauss matrix

STATIC = lib$(LIB_NAME).a
STATIC_WIN = lib$(LIB_NAME)_win.a
STATIC_ROBOT = lib$(LIB_NAME)_robot.a

OBJECTS = $(foreach module,$(MODULES),$(module)/$(PLATFORM)/$(module).o)
HEADERS = *.h $(MODULES:%=%/*.h)

PLATFORM = linux

$(STATIC_WIN) : PLATFORM = windows
$(STATIC_WIN) : AR = i586-mingw32msvc-ar

$(STATIC_ROBOT) : PLATFORM = robot
$(STATIC_ROBOT) : AR = arm-elf-ar

# all: $(MODULES) $(STATIC) $(STATIC_WIN) $(STATIC_ROBOT)
all: $(MODULES) $(STATIC)

$(MODULES):
	$(MAKE) -sC $@
$(MODULES:=/clean):
	$(MAKE) -sC $(@D) clean
$(STATIC) $(STATIC_WIN) $(STATIC_ROBOT): $(OBJECTS)
	$(AR) rcs $@ $(OBJECTS)
clean: $(MODULES:=/clean)
	-$(RM) $(STATIC) $(STATIC_WIN) $(STATIC_ROBOT)

.PHONY: $(MODULES) all clean $(MODULES:=/clean)

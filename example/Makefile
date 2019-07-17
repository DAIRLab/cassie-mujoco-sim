# Build for linux by default
PLATFORM := LINUX

# Compilation settings
INC     := -Iinclude
CFLAGS  := -std=gnu11 -Wall -Wextra -O3 -march=sandybridge -flto
LDFLAGS := -L.
LIBS    := -llcm -lcassiemujoco

# Platform-specific settings
ifeq ($(PLATFORM), WIN)
CC      := x86_64-w64-mingw32-gcc
SIMOUT  := cassiesim.exe
else
CC      := gcc
LDFLAGS += -Wl,-rpath,'$$ORIGIN'
SIMOUT  := cassiesim
endif

# Default target
all: $(SIMOUT)

# Normal targets
clean:
	rm -f $(SIMOUT)

$(SIMOUT): cassiesim.c
	$(CC) cassiesim.c $(INC) $(CFLAGS) $(LDFLAGS) $(LIBS) -o $(SIMOUT)

# Virtual targets
.PHONY: all clean

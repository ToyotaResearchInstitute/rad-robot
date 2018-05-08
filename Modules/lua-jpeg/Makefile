.PHONY: all clean
OBJS=lua_jpeg.o
TARGET=jpeg.so


LUA = $(shell pkg-config --list-all | egrep -o "^lua-?(jit|5\.?[123])" | sort -r | head -n1)
LUA_INCDIR ?= . $(shell pkg-config $(LUA) --cflags-only-I)
LUA_LIBDIR ?= . $(shell pkg-config $(LUA) --libs-only-L)
CFLAGS ?= -fPIC -O2 $(shell pkg-config $(LUA) --cflags-only-other)

ifndef OSTYPE
OSTYPE=$(shell uname -s | tr '[:upper:]' '[:lower:]')
endif

ifeq ($(OSTYPE),darwin) # OSX linking and installation
LIBFLAG ?= -bundle -undefined dynamic_lookup -all_load -macosx_version_min 10.13
else # Linux linking and installation
LIBFLAG ?= -shared
endif


ifdef USE_TORCH
	CFLAGS+=-DTORCH=1
	LDFLAGS+=-ltorch
endif

all: $(TARGET)
	@echo --- build
	@echo CC: $(CC)
	@echo CPP: $(CPP)
	@echo CFLAGS: $(CFLAGS)
	@echo LIBFLAG: $(LIBFLAG)
	@echo LUA_LIBDIR: $(LUA_LIBDIR)
	@echo LUA_BINDIR: $(LUA_BINDIR)
	@echo LUA_INCDIR: $(LUA_INCDIR)

$(TARGET): $(OBJS)
	$(LD) $(LIBFLAG) -o $@ -L$(LUA_LIBDIR) $(LDFLAGS) $(OBJS) $(shell pkg-config --libs libjpeg)

%.o: %.c
	$(CC) -c -o $@ $< -I$(LUA_INCDIR) $(CFLAGS) $(shell pkg-config --cflags libjpeg)
  
install: $(TARGET)
	@echo --- install
	@echo INST_PREFIX: $(INST_PREFIX)
	@echo INST_BINDIR: $(INST_BINDIR)
	@echo INST_LIBDIR: $(INST_LIBDIR)
	@echo INST_LUADIR: $(INST_LUADIR)
	@echo INST_CONFDIR: $(INST_CONFDIR)
	@echo Copying $< ...
	cp $< $(INST_LIBDIR)

clean:
	-rm -f $(OBJS)
	-rm -f $(TARGET)

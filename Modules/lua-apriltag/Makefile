.PHONY: all clean
OBJS=lua_apriltag.o
TARGET=apriltag.so

# NOTE: Only Linux has a v4l2 API
ifndef OSTYPE
OSTYPE = $(shell uname -s | tr '[:upper:]' '[:lower:]')
endif

LUA = $(shell pkg-config --list-all | egrep -o "^lua-?(jit|5\.?[123])" | sort -r | head -n1)
LUA_INCDIR ?= . $(shell pkg-config $(LUA) --cflags-only-I)
LUA_LIBDIR ?= . $(shell pkg-config $(LUA) --libs-only-L)
CFLAGS ?= -Wall -fPIC -O2 $(shell pkg-config $(LUA) --cflags-only-other) $(shell pkg-config --cflags apriltag)
# CFLAGS ?= -Wall -fPIC -g $(shell pkg-config $(LUA) --cflags-only-other) $(shell pkg-config --cflags apriltag) -DDEBUG

ifeq ($(OSTYPE),darwin)
LIBFLAG ?= -bundle -undefined dynamic_lookup -all_load -macosx_version_min 10.14
else # Linux linking and installation
LIBFLAG ?= -shared -lrt
endif

all: $(TARGET)
	@echo LUA: $(LUA)
	@echo --- build
	@echo CFLAGS: $(CFLAGS)
	@echo LIBFLAG: $(LIBFLAG)
	@echo LUA_LIBDIR: $(LUA_LIBDIR)
	@echo LUA_BINDIR: $(LUA_BINDIR)
	@echo LUA_INCDIR: $(LUA_INCDIR)

$(TARGET): $(OBJS)
	$(LD) $(LIBFLAG) -lm $(shell pkg-config --libs apriltag) -o $@ -L$(LUA_LIBDIR) $(OBJS)

%.o: %.c
	$(CC) -c -o $@ $< -I$(LUA_INCDIR) $(CFLAGS)

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

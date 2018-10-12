.PHONY: all clean
OBJS=dijkstra.o

ifndef OSTYPE
OSTYPE = $(shell uname -s | tr '[:upper:]' '[:lower:]')
endif

LUA = $(shell pkg-config --list-all | egrep -o "^lua-?(jit|5\.?[123])" | sort -r | head -n1)
LUA_INCDIR ?= . $(shell pkg-config $(LUA) --cflags-only-I)
LUA_LIBDIR ?= . $(shell pkg-config $(LUA) --libs-only-L)
CFLAGS ?= -fPIC -O2 $(shell pkg-config $(LUA) --cflags-only-other)
CXXFLAGS ?= -fPIC -O2 $(shell pkg-config $(LUA) --cflags-only-other)

ifeq ($(OSTYPE),darwin)
TARGET=libdijkstra.dylib
# LIBFLAG ?= -bundle -undefined dynamic_lookup -all_load -macosx_version_min 10.13 -lc++
LIBFLAG ?= -dylib -undefined dynamic_lookup -macosx_version_min 10.13 -lc++
else # Linux linking and installation
TARGET=libdijkstra.so
LIBFLAG ?= -shared
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
	$(LD) $(LIBFLAG) -o $@ -L$(LUA_LIBDIR) $(OBJS) -lm

%.o: %.c
	$(CC) -c -o $@ $< -I$(LUA_INCDIR) $(CFLAGS)

%.o: %.cc
	$(CXX) -c -o $@ $< -I$(LUA_INCDIR) $(CXXFLAGS) -DUSE_BACKWARDS -DUSE_SHORT_CIRCUIT -DUSE_RAYTRACE_NEIGHBOR_COSTS

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

.PHONY: all clean
OBJS=lua_dalsa.o
TARGET=dalsa.so

ifndef OSTYPE
OSTYPE = $(shell uname -s | tr '[:upper:]' '[:lower:]')
endif

LUA = $(shell pkg-config --list-all | egrep -o "^lua-?(jit|5\.?[123])" | sort -r | head -n1)
LUA_INCDIR ?= . $(shell pkg-config $(LUA) --cflags-only-I)
LUA_LIBDIR ?= . $(shell pkg-config $(LUA) --libs-only-L)
CFLAGS ?= -fPIC -O2 $(shell pkg-config $(LUA) --cflags-only-other)
CFLAGS +=-Wall -Wno-parentheses -Wno-missing-braces \
		-Wno-unknown-pragmas -Wno-cast-qual -Wno-unused-function \
		-Wno-unused-label -Wno-unused-but-set-variable

ifeq ($(OSTYPE),darwin)
LIBFLAG ?= -bundle -undefined dynamic_lookup -all_load -macosx_version_min 10.13
else # Linux linking and installation
LIBFLAG ?= -shared
endif
GIGE_V=/usr/dalsa/GigeV

all: $(TARGET)
	@echo LUA: $(LUA)
	@echo --- build
	@echo CFLAGS: $(CFLAGS)
	@echo LIBFLAG: $(LIBFLAG)
	@echo LUA_LIBDIR: $(LUA_LIBDIR)
	@echo LUA_BINDIR: $(LUA_BINDIR)
	@echo LUA_INCDIR: $(LUA_INCDIR)

$(TARGET): $(OBJS)
	$(LD) $(LIBFLAG) -o $@ -L$(LUA_LIBDIR) -L$(GIGE_V)/lib $(OBJS) \
	-lGevApi -lrt -fhosted \
	$(shell pkg-config --libs cuda-9.0) -lcudart -lnppicc -lnppig

%.o: %.c
	$(CC) -g -c -o $@ $< -I$(LUA_INCDIR) -I$(GIGE_V)/include $(CFLAGS) \
	$(shell pkg-config --cflags cuda-9.0) -DUSE_NPP \
	-DDEBUG -DUSE_SYNCHRONOUS_BUFFER_CYCLING \
	-DUSE_SHM \
	-DTUNE_STREAMING_THREADS \
	-DUSE_RESIZE

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

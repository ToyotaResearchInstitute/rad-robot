.PHONY: all clean
SRC=lua_python.c py_lua.c common.c
PY_OBJS=lua_python.o common.o
LUA_OBJS=py_lua.o common.o
ALL_OBJS=lua_python.o py_lua.o common.o
TARGET=python.so

ifndef OSTYPE
OSTYPE = $(shell uname -s | tr '[:upper:]' '[:lower:]')
endif

PY_SUFFIX=$(shell python3-config --extension-suffix)

ifeq ($(OSTYPE),darwin)
LUA = $(shell pkg-config --list-all | egrep -o "^lua-?(5\.?[123])" | sort -r | head -n1)
else
LUA = $(shell pkg-config --list-all | egrep -o "^lua-?(jit|5\.?[123])" | sort -r | head -n1)
endif
LUA_INCDIR ?= . $(shell pkg-config $(LUA) --cflags-only-I)
LUA_LIBDIR ?= . $(shell pkg-config $(LUA) --libs-only-L)
LUA_LIBS ?= $(shell pkg-config $(LUA) --libs-only-l)

CFLAGS ?= -fPIC -O2 $(shell pkg-config $(LUA) --cflags-only-other)

PYTHON = python3
PYTHON_CFLAGS = $(shell $(PYTHON)-config --cflags | sed s/-fno-plt//g)
PYTHON_LIBS = $(shell $(PYTHON)-config --libs)
PYTHON_LIBDIR=$(shell python3-config --prefix)/lib

NUMPY_INCDIR=$(shell python -c 'import numpy;print(numpy.get_include())')
ifeq ($(NUMPY_INCDIR),)
endif


ifeq ($(OSTYPE),darwin)
LIBFLAG ?= -bundle -undefined dynamic_lookup -all_load -macosx_version_min 10.13
else # Linux linking and installation
LIBFLAG ?= -shared
endif

all: $(TARGET) lua
	@echo --- build
	@echo $(LUA) $(PYTHON)
	@echo CFLAGS: $(CFLAGS)
	@echo LIBFLAG: $(LIBFLAG)
	@echo LUA_LIBDIR: $(LUA_LIBDIR)
	@echo LUA_BINDIR: $(LUA_BINDIR)
	@echo LUA_INCDIR: $(LUA_INCDIR)

ifeq ($(OSTYPE),darwin)
%.o: %.c
	$(CC) -c -o $@ $< -I$(LUA_INCDIR) $(CFLAGS) $(PYTHON_CFLAGS)
$(TARGET): $(ALL_OBJS)
	$(LD) $(LIBFLAG) $(PYTHON_LIBS) -L$(PYTHON_LIBDIR) -L$(LUA_LIBDIR) -rpath $(PYTHON_LIBDIR) -o $@ $(ALL_OBJS)
lua: $(ALL_OBJS)
	$(LD) $(LIBFLAG) $(LUA_LIBS) -L$(PYTHON_LIBDIR) -L$(LUA_LIBDIR) -o $@$(PY_SUFFIX) $(ALL_OBJS)
else
$(TARGET):
	$(CC) $(SRC) -Bdynamic $(LIBFLAG) $(CFLAGS) $(PYTHON_CFLAGS) -I$(LUA_INCDIR) -L$(LUA_LIBDIR) -L$(PYTHON_LIBDIR) $(PYTHON_LIBS) -Wl,-rpath=$(PYTHON_LIBDIR) -o $@
lua:
	$(CC) $(SRC) -Bdynamic $(LIBFLAG) $(CFLAGS) $(PYTHON_CFLAGS) -I$(LUA_INCDIR) -L$(LUA_LIBDIR) -L$(PYTHON_LIBDIR) $(PYTHON_LIBS) $(LUA_LIBS) -Wl,-rpath=$(PYTHON_LIBDIR) -o $@$(PY_SUFFIX)
endif

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
	-rm -f $(ALL_OBJS)
	-rm -rf $(TARGET) $(TARGET).so.dSYM *$(PY_SUFFIX)

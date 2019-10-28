.PHONY: all clean
SRC=lua_python.c py_lua.c common.c
PY_OBJS=lua_python.o common.o
LUA_OBJS=py_lua.o common.o
ALL_OBJS=lua_python.o py_lua.o common.o
TARGET=python.so

ifndef OSTYPE
OSTYPE = $(shell uname -s | tr '[:upper:]' '[:lower:]')
endif

LUA = $(shell pkg-config --list-all | egrep -o "^lua-?(jit|5\.?[123])" | sort -r | head -n1)
# LUA = $(shell pkg-config --list-all | egrep -o "^lua-?(5\.?[23])" | sort -r | head -n1)
LUA_INCDIR ?= . $(shell pkg-config $(LUA) --cflags-only-I)
LUA_LIBDIR ?= . $(shell pkg-config $(LUA) --libs-only-L)
LUA_LIBS ?= $(shell pkg-config $(LUA) --libs-only-l)

ifeq ($(OSTYPE),darwin)
LDFLAGS ?= -bundle -undefined dynamic_lookup -all_load -macosx_version_min 10.14
else # Linux linking and installation
LDFLAGS ?= -shared
endif

CCWARN= -Wall
CCWARN+= -Wextra -Wdeclaration-after-statement -Wredundant-decls -Wshadow -Wpointer-arith
CCWARN += -Werror=vla
CFLAGS ?= $(CCWARN) -fPIC -O2 $(shell pkg-config $(LUA) --cflags-only-other)

PYTHON = python3
PYTHON_CFLAGS = $(shell $(PYTHON)-config --cflags | sed s/-fno-plt//g)
PYTHON_LIBS = $(shell $(PYTHON)-config --libs)
PYTHON_LIBDIR=$(shell $(PYTHON)-config --prefix)/lib
PY_SUFFIX=$(shell $(PYTHON)-config --extension-suffix)

NUMPY_INCDIR=$(shell python -c 'import numpy;print(numpy.get_include())')
ifeq ($(NUMPY_INCDIR),)
endif

all: $(TARGET) lua
	@echo --- build
	@echo $(LUA) $(PYTHON)
	@echo CFLAGS: $(CFLAGS)
	@echo LDFLAGS: $(LDFLAGS)
	@echo LUA_LIBDIR: $(LUA_LIBDIR)
	@echo LUA_BINDIR: $(LUA_BINDIR)
	@echo LUA_INCDIR: $(LUA_INCDIR)

%.o: %.c
	$(CC) -c -o $@ $< -I$(LUA_INCDIR) $(CFLAGS) $(PYTHON_CFLAGS)

ifeq ($(OSTYPE),darwin)
$(TARGET): $(ALL_OBJS)
	$(LD) $(LDFLAGS) $(PYTHON_LIBS) -L$(PYTHON_LIBDIR) -L$(LUA_LIBDIR) -rpath $(PYTHON_LIBDIR) -o $@ $(ALL_OBJS)
lua: $(ALL_OBJS)
	$(LD) $(LDFLAGS) $(LUA_LIBS) -L$(PYTHON_LIBDIR) -L$(LUA_LIBDIR) -o $@$(PY_SUFFIX) $(ALL_OBJS)
else
$(TARGET):
	$(CC) $(SRC) -Bdynamic $(LDFLAGS) $(CFLAGS) $(PYTHON_CFLAGS) -I$(LUA_INCDIR) -L$(LUA_LIBDIR) -L$(PYTHON_LIBDIR) $(PYTHON_LIBS) -Wl,-rpath=$(PYTHON_LIBDIR) -o $@
lua:
	$(CC) $(SRC) -Bdynamic $(LDFLAGS) $(CFLAGS) $(PYTHON_CFLAGS) -I$(LUA_INCDIR) -L$(LUA_LIBDIR) -L$(PYTHON_LIBDIR) $(PYTHON_LIBS) $(LUA_LIBS) -Wl,-rpath=$(PYTHON_LIBDIR) -o $@$(PY_SUFFIX)
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

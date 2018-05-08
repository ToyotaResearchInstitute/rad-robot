#pragma once

/* need this to build with Lua 5.2: defines luaL_register() macro */
#define LUA_COMPAT_MODULE

#include <lua.h>
#include <lualib.h>
#include <lauxlib.h>
#include <Python.h>

// Compatibility options for Python 2/3, Lua 5.x
#if PY_MAJOR_VERSION < 3
  #define PyBytes_Check           PyString_Check
  #define PyBytes_AsStringAndSize PyString_AsStringAndSize
  #define PyInit_lua initlua
#endif
PyMODINIT_FUNC PyInit_lua(void);

#define POBJECT "POBJECT"

typedef struct
{
    PyObject *o;
    int asindx;
} py_object;

typedef struct
{
    PyObject_HEAD
    int ref;
    int refiter;
} LuaObject;

#define LuaObject_Check(op) PyObject_TypeCheck(op, &LuaObject_Type)

// Convert a Lua object at a stack position to a Python object
PyObject *LuaConvert(lua_State *L, int n);
py_object* luaPy_to_pobject(lua_State *L, int n);

// Convert a Python object to a Lua object and push onto the stack
int py_convert_custom(lua_State *L, PyObject *o, int asindx);
int py_convert(lua_State *L, PyObject *o);

// Initialize the Lua library, given a Lua state in which to place the library
LUA_API int luaopen_python(lua_State *L);

PyObject *LuaObject_New(lua_State *L, int n);

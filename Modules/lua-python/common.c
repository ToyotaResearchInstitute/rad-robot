#include "common.h"

lua_State *LuaState;
extern PyTypeObject LuaObject_Type;

py_object* luaPy_to_pobject(lua_State *L, int n)
{
  if (!lua_getmetatable(L, n)) {
    return NULL;
  }
  luaL_getmetatable(L, POBJECT);
  int is_pobject = lua_rawequal(L, -1, -2);
  lua_pop(L, 2);
  return is_pobject ? (py_object *) lua_touserdata(L, n) : NULL;
}

// Convert a Lua object at a stack position to a Python object
PyObject *LuaConvert(lua_State *L, int n) {
  PyObject *ret = NULL;

  switch (lua_type(L, n)) {

  case LUA_TNIL:
    Py_INCREF(Py_None);
    ret = Py_None;
    break;

  case LUA_TSTRING: {
    size_t len;
    const char *s = lua_tolstring(L, n, &len);
    ret = PyUnicode_FromStringAndSize(s, len);
    if (!ret) {
      PyErr_Clear();
      ret = PyBytes_FromStringAndSize(s, len);
    }
    break;
  }

  case LUA_TNUMBER: {
    lua_Number num = lua_tonumber(L, n);
    if (num != (long)num) {
      ret = PyFloat_FromDouble(num);
    } else {
      ret = PyLong_FromLong((long)num);
    }
    break;
  }

  case LUA_TBOOLEAN:
    ret = lua_toboolean(L, n) ? Py_True : Py_False;
    Py_INCREF(ret);
    break;

  case LUA_TUSERDATA: { /* go on and handle as custom if no metatable */
    // py_object* obj = (py_object *)luaL_checkudata(L, n, POBJECT);
    // luaL_argcheck(L, obj != NULL, narg, "invalid pobject userdata");
    // Py_INCREF(obj->o);
    // ret = obj->o;
    // break;

    py_object *obj = luaPy_to_pobject(L, n);
    if (obj) {
      Py_INCREF(obj->o);
      ret = obj->o;
      break;
    }
  }
  

  // case LUA_TLIGHTUSERDATA: {
  //   Py_ssize_t size = (Py_ssize_t) luaL_checkinteger(L, 2);

  //   // Check writability
  //   int flags;
  //   if (lua_isboolean(L, 3)) {
  //     ret = PyMemoryView_FromMemory(
  //       lua_touserdata(L, 1), size, lua_toboolean(L, 3) ? PyBUF_WRITE : PyBUF_READ);
  //   } else {
  //     ret = PyMemoryView_FromMemory(lua_touserdata(L, 1), size, PyBUF_READ);
  //   }
  // }


  default:
    ret = LuaObject_New(L, n);
    break;
  }

  return ret;
}

// TODO: Does this need to be static?
int py_convert_custom(lua_State *L, PyObject *o, int asindx) {
  py_object *obj = (py_object*) lua_newuserdata(L, sizeof(py_object));
  if (!obj) {
    luaL_error(L, "failed to allocate userdata object");
  }

  Py_INCREF(o);
  obj->o = o;
  obj->asindx = asindx;
  luaL_getmetatable(L, POBJECT);
  lua_setmetatable(L, -2);

  return 1;
}


int py_convert(lua_State *L, PyObject *o)
{
  int ret = 0;
  if (o == Py_None)
  {
    /* Not really needed, but this way we may check
     * for errors with ret == 0. */
    lua_pushnil(L);
    ret = 1;
  } else if (o == Py_True) {
    lua_pushboolean(L, 1);
    ret = 1;
  } else if (o == Py_False) {
    lua_pushboolean(L, 0);
    ret = 1;
  } else if (PyUnicode_Check(o) || PyBytes_Check(o)) {
    PyObject *bstr = PyUnicode_AsEncodedString(o, "utf-8", NULL);
    Py_ssize_t len;
    char *s;

    PyErr_Clear();
    PyBytes_AsStringAndSize(bstr ? bstr : o, &s, &len);
    lua_pushlstring(L, s, len);
    if (bstr) Py_DECREF(bstr);
    ret = 1;
#if PY_MAJOR_VERSION < 3
  } else if (PyInt_Check(o)) {
    lua_pushnumber(L, (lua_Number)PyInt_AsLong(o));
    ret = 1;
#endif
  } else if (PyLong_Check(o)) {
    lua_pushnumber(L, (lua_Number)PyLong_AsLong(o));
    ret = 1;
  } else if (PyFloat_Check(o)) {
    lua_pushnumber(L, (lua_Number)PyFloat_AsDouble(o));
    ret = 1;
  } else if (LuaObject_Check(o)) {
      lua_rawgeti(L, LUA_REGISTRYINDEX, ((LuaObject*)o)->ref);
      ret = 1;
  } else {
    int asindx = PyDict_Check(o) || PyList_Check(o) || PyTuple_Check(o);
    ret = py_convert_custom(L, o, asindx);
  }
  return ret;
}

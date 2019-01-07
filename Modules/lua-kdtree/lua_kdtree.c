/*
Author: Stephen McGill <stephen.mcgill@tri.global> 2018
*/
#include <lauxlib.h>
#include <lua.h>
#include <lualib.h>

#include "kdtree.h"
#define MT_NAME "kdtree_mt"

struct kdtree **lua_checkkdtree(lua_State *L, int narg) {
  void *ud = luaL_checkudata(L, narg, MT_NAME);
  luaL_argcheck(L, ud != NULL, narg, "Invalid kdtree userdata");
  return ud;
}

static int lua_kdtree_index(lua_State *L) {
  if (!lua_getmetatable(L, 1)) {
    /* push metatable */
    lua_pop(L, 1);
    return 0;
  }
  lua_pushvalue(L, 2); /* copy key */
  lua_rawget(L, -2);   /* get metatable function */
  lua_remove(L, -2);   /* delete metatable */
  return 1;
}

static int lua_kdtree_create(lua_State *L) {
  int k = luaL_checkinteger(L, 1);
  struct kdtree *kd = kd_create(k);
  if (!kd) {
    lua_pushboolean(L, 0);
    return 1;
  }

  // Make accessible to lua
  struct kdtree **ud = lua_newuserdata(L, sizeof(struct kdtree *));
  *ud = kd;

  luaL_getmetatable(L, MT_NAME);
  lua_setmetatable(L, -2);
  return 1;
}

static int lua_kdtree_clear(lua_State *L) {
  struct kdtree *kd = *lua_checkkdtree(L, 1);
  if (!kd) {
    lua_pushboolean(L, 0);
    return 1;
  }
  kd_clear(kd);
  lua_pushboolean(L, 1);
  return 1;
}

static int lua_kdtree_free(lua_State *L) {
  struct kdtree *kd = *lua_checkkdtree(L, 1);
  if (!kd) {
    lua_pushboolean(L, 0);
    return 1;
  }
  kd_clear(kd);
  kd_free(kd);
  lua_pushboolean(L, 1);
  return 1;
}

static int lua_kdtree_insert(lua_State *L) {
  struct kdtree *kd = *lua_checkkdtree(L, 1);

  if (!lua_istable(L, 2)) {
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "Need a table.");
    return 2;
  }
  // Take values in the table of the first k dimensions
  int k = kd_get_dimension(kd);
#if LUA_VERSION_NUM == 501
  if (lua_objlen(L, 2) < k)
#else
  if (lua_rawlen(L, 2) < k)
#endif
  {
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "Not enough dimensions");
    return 2;
  }

  // VLA for the table
  int i;
  double vals[k];
  for (i = 0; i < k; i++) {
    lua_rawgeti(L, 2, i + 1);
    // TODO: Check that each value is, in fact, a number
    vals[i] = lua_tonumber(L, -1);
    lua_pop(L, 1);
  }

  // NOTE: This is kinda dangerous, but gives some good features :)
  void *user = NULL;
  if (lua_isnumber(L, 3)) {
    user = (void *)lua_tointeger(L, 3);
  }

  // NOTE: http://www.lua.org/pil/27.3.2.html
  // fprintf(stderr, "Inserting... [%p] { ", user);
  // for (i = 0; i < k; i++) {
  //   fprintf(stderr, "%f ", vals[i]);
  // }
  // fprintf(stderr, "}\n");
  int res = kd_insert(kd, vals, user);
  // fprintf(stderr, "Done!\n");
  if (res == -1) {
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "Cannot insert into the KD tree");
    return 2;
  }
  lua_pushboolean(L, 1);
  return 1;
}

static int lua_kdtree_nearest(lua_State *L) {
  struct kdtree *kd = *lua_checkkdtree(L, 1);

  int k = kd_get_dimension(kd);

  if (!lua_istable(L, 2)) {
    lua_pushboolean(L, 0);
    return 1;
  }
#if LUA_VERSION_NUM == 501
  if (lua_objlen(L, 2) < k)
#else
  if (lua_rawlen(L, 2) < k)
#endif
  {
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "Not enough dimensions");
    return 2;
  }
  // VLA
  int i;
  double vals0[k];
  for (int i = 1; i <= k; i++) {
    lua_rawgeti(L, 2, i);
    vals0[i - 1] = luaL_checknumber(L, -1);
    lua_pop(L, 1);
  }

  struct kdres *set;
  if (lua_isnumber(L, 3)) {
    double range = lua_tonumber(L, 3);
    set = kd_nearest_range(kd, vals0, range);
  } else {
    set = kd_nearest(kd, vals0);
  }

  int n_res = kd_res_size(set);
  if (n_res == 0) {
    kd_res_free(set);
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "Nobody nearby");
    return 2;
  }

  // Push results
  lua_createtable(L, n_res, 0);
  i = 1;
  double vals[k];
  double dist_sq;
  void *data;
  while (!kd_res_end(set)) {
    /* get the data and position of the current result item */
    // data = kd_res_item(set, vals);
    data = kd_res_item_dist_sq(set, vals, &dist_sq);
    // Nested table
    lua_createtable(L, k, 2);
    for (int ik = 0; ik < k; ik++) {
      lua_pushnumber(L, vals[ik]);
      lua_rawseti(L, -2, ik + 1);
    }
    lua_pushliteral(L, "user");
    lua_pushinteger(L, (lua_Integer)data);
    lua_settable(L, -3);
    lua_pushliteral(L, "dist_sq");
    lua_pushnumber(L, dist_sq);
    lua_settable(L, -3);
    lua_rawseti(L, -2, i++);

    /* go to the next entry */
    kd_res_next(set);
  }

  // Returns the table of points
  return 1;
}

static int lua_kdtree_tostring(lua_State *L) {
  struct kdtree *kd = *lua_checkkdtree(L, 1);
  lua_pushfstring(L, "KD-Tree | %d dimensional | %d elements | %d depth",
                  kd_get_dimension(kd), kd_get_size(kd), kd_get_depth(kd));
  return 1;
}

static int lua_kdtree_size(lua_State *L) {
  struct kdtree *kd = *lua_checkkdtree(L, 1);
  lua_pushinteger(L, kd_get_size(kd));
  return 1;
}

static int lua_kdtree_dim(lua_State *L) {
  struct kdtree *kd = *lua_checkkdtree(L, 1);
  lua_pushinteger(L, kd_get_dimension(kd));
  return 1;
}

static const struct luaL_Reg kdtree_lib[] = {{"create", lua_kdtree_create},
                                             {NULL, NULL}};

static const struct luaL_Reg kdtree_methods[] = {
    {"__index", lua_kdtree_index},
    {"__gc", lua_kdtree_free},
    {"__tostring", lua_kdtree_tostring},
    {"insert", lua_kdtree_insert},
    {"dim", lua_kdtree_dim},
    {"size", lua_kdtree_size},
#if LUA_VERSION_NUM > 501
    {"__len", lua_kdtree_size},
#endif
    {"clear", lua_kdtree_clear},
    {"nearest", lua_kdtree_nearest},
    {NULL, NULL}};

#ifdef __cplusplus
extern "C"
#endif
    int
    luaopen_kdtree(lua_State *L) {
  luaL_newmetatable(L, MT_NAME);
#if LUA_VERSION_NUM == 501
  luaL_register(L, NULL, kdtree_methods);
  luaL_register(L, "kdtree", kdtree_lib);
#else
  luaL_setfuncs(L, kdtree_methods, 0);
  luaL_newlib(L, kdtree_lib);
#endif
  return 1;
}

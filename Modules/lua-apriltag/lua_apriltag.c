/*
Author: Stephen McGill <stephen.mgill@tri.global>, 05/2019
*/

#define LUA_COMPAT_MODULE
#include <lauxlib.h>
#include <lua.h>
#include <lualib.h>

#include <fcntl.h>
#include <math.h>
#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/time.h>
#include <unistd.h>

#include <apriltag/apriltag.h>
#include <apriltag/apriltag_pose.h>
#include <apriltag/common/zarray.h>
#include <apriltag/tag16h5.h>

#define NBUFFERS 2
/* metatable name for apriltags */
#define MT_NAME "apriltags_mt"

typedef struct {
  apriltag_detector_t *td;
  apriltag_family_t *tf;
  apriltag_detection_info_t *info;
} apriltag_data;

static apriltag_data *lua_checkapriltags(lua_State *L, int narg) {
  void *ud = luaL_checkudata(L, narg, MT_NAME);
  luaL_argcheck(L, ud != NULL, narg, "invalid apriltags userdata");
  return (apriltag_data *)ud;
}

static int lua_apriltags_index(lua_State *L) {
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

static int lua_apriltags_delete(lua_State *L) {
#ifdef DEBUG
  fprintf(stderr, "Cleaning...\n");
#endif
  apriltag_data *ud = lua_checkapriltags(L, 1);

  if (ud->td) {
    apriltag_detector_destroy(ud->td);
    ud->td = NULL;
  }
  if (ud->tf) {
    tag16h5_destroy(ud->tf);
    ud->tf = NULL;
  }

#ifdef DEBUG
  fprintf(stderr, "Cleaning done!\n");
#endif
  return 0;
}

static int lua_apriltags_init(lua_State *L) {
  apriltag_data *ud = lua_newuserdata(L, sizeof(apriltag_data));
  // Zero the data so that any initial access is NULL
  bzero(ud, sizeof(apriltag_data));
  // Populate the userdata
  // int n = lua_gettop(L);
  // const char *video_device = lua_isstring(L, 1) ? lua_tostring(L, 1) : NULL;
  // ud->width = luaL_optint(L, 2, 320);
  // ud->height = luaL_optint(L, 3, 240);
  // ud->pixelformat = luaL_optstring(L, 4, "yuyv");
  // /* default 15 fps */
  // ud->fps_num = luaL_optint(L, 5, 1);
  // ud->fps_denum = luaL_optint(L, 6, 15);
  // _Bool make_usb = lua_isboolean(L, 7) && lua_toboolean(L, 3);

  apriltag_detector_t *td = apriltag_detector_create();
  apriltag_family_t *tf = tag16h5_create();
  apriltag_detector_add_family(td, tf);

  // TODO: Take table input
  td->quad_decimate = 2.0;
  td->quad_sigma = 0.0;
  td->refine_edges = 1;
  td->decode_sharpening = 0.25;

  // Set the pose information
  apriltag_detection_info_t *info =
      calloc(1, sizeof(apriltag_detection_info_t));
  // info.det = det;
  info->tagsize = 0.10; // tagsize;
  info->fx = 320;       // fx;
  info->fy = 240;       // fy;
  info->cx = 160;       // cx;
  info->cy = 120;       // cy;

  // Set in our userdata
  ud->td = td;
  ud->tf = tf;
  ud->info = info;
  luaL_getmetatable(L, MT_NAME);
  lua_setmetatable(L, -2);
  return 1;
}

static int lua_apriltags_detect(lua_State *L) {
  apriltag_data *ud = lua_checkapriltags(L, 1);
// lua_pushboolean(L, 0);
// lua_pushliteral(L, "apriltags_start_streaming: unable to start stream");
// return 2;
#ifdef DEBUG
  fprintf(stderr, "Start detect...\n");
#endif
  image_u8_t *im = NULL;
  zarray_t *detections = apriltag_detector_detect(ud->td, im);

  for (int i = 0; i < zarray_size(detections); i++) {
    apriltag_detection_t *det;
    zarray_get(detections, i, &det);
    ud->info->det = det;

    // Then call estimate_tag_pose.
    apriltag_pose_t pose;
    double err = estimate_tag_pose(ud->info, &pose);
  }

  apriltag_detections_destroy(detections);

  lua_pushboolean(L, 1);
  return 1;
}

static const struct luaL_Reg apriltags_functions[] = {
    {"init", lua_apriltags_init}, {NULL, NULL}};

static const struct luaL_Reg apriltags_methods[] = {
    {"detect", lua_apriltags_detect},
    {"__index", lua_apriltags_index},
    {"__gc", lua_apriltags_delete},
    {NULL, NULL}};

#ifdef __cplusplus
extern "C"
#endif
    int
    luaopen_apriltags(lua_State *L) {
  /* create metatable for apriltags module */
  luaL_newmetatable(L, MT_NAME);
#if LUA_VERSION_NUM == 501
  luaL_register(L, NULL, apriltags_methods);
#else
  luaL_setfuncs(L, apriltags_methods, 0);
#endif
  lua_pop(L, 1);

#if LUA_VERSION_NUM == 501
  luaL_register(L, "apriltags", apriltags_functions);
#else
  luaL_newlib(L, apriltags_functions);
#endif

  return 1;
}

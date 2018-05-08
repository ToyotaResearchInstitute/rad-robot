/*
 * Copyright 2002-2010 Guillaume Cottenceau.
 *
 * This software may be freely redistributed under the terms
 * of the X11 license.
 *
 * lua wrapper by Yida Zhang <yida@seas.upenn.edu>
 * Stephen McGill <smcgill3@seas.upenn.edu>
 * University of Pennsylvania
 */

// http://www.libpng.org/pub/png/libpng-1.2.5-manual.html

#include <lauxlib.h>
#include <lua.h>
#include <lualib.h>

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define MT_NAME "png_mt"
#include <png.h>

typedef struct {
  int width, height, stride;
  png_byte color_type;
  png_byte bit_depth;
  png_structp png_ptr;
  png_infop info_ptr;
  int number_of_passes;
  png_bytep *row_pointers;
} structPNG;

struct mem_encode {
  char *buffer;
  size_t size;
  size_t written;
};

void abort_(const char *s, ...) {
  va_list args;
  va_start(args, s);
  vfprintf(stderr, s, args);
  fprintf(stderr, "\n");
  va_end(args);
  abort();
}

static structPNG *lua_checkpng(lua_State *L, int narg) {
  void *ud = luaL_checkudata(L, narg, MT_NAME);
  luaL_argcheck(L, *(structPNG **)ud != NULL, narg, "invalid png");
  return (structPNG *)ud;
}

static int lua_png_delete(lua_State *L) {
  structPNG *p = lua_checkpng(L, 1);
  /* cleanup heap allocation */
  int y;
  for (y = 0; y < p->height; y++) {
    free(p->row_pointers[y]);
  }
  free(p->row_pointers);
  return 1;
}

static int lua_png_dims(lua_State *L) {
  structPNG *p = lua_checkpng(L, 1);
  lua_pushinteger(L, p->width);
  lua_pushinteger(L, p->height);
  lua_pushinteger(L, p->color_type);
  lua_pushinteger(L, p->bit_depth);
  return 4;
}

void lua_png_write_string(png_structp png_ptr, png_bytep data,
                          png_size_t length) {
  struct mem_encode *p = (struct mem_encode *)png_get_io_ptr(png_ptr);
  size_t nsize = p->written + length;
  // fprintf(stderr, "Writing...\n");
  if (p->size < nsize) {
    char *buf = (char *)realloc(p->buffer, nsize);
    if (!p->buffer) {
      png_error(png_ptr, "Realloc Buffer Error");
    } else {
      p->buffer = buf;
      p->size = nsize;
    }
  }

  memcpy(p->buffer + p->written, data, length);
  p->written += length;
}

static int lua_png_compress(lua_State *L) {

  unsigned char *data = NULL;
  size_t sz = 0;
  int width, height, bytes_per_pixel;
  int bit_depth = 8;
  int compression_level = 3;

  switch (lua_type(L, 1)) {

  case LUA_TLIGHTUSERDATA:
    data = (unsigned char *)lua_touserdata(L, 1);
    sz = luaL_checkinteger(L, 2);
    width = luaL_checkinteger(L, 3);
    height = luaL_checkinteger(L, 4);
    bytes_per_pixel = luaL_optinteger(L, 5, 3);
    compression_level = luaL_optinteger(L, 6, 3);
    break;

  case LUA_TSTRING:
    data = (unsigned char *)lua_tolstring(L, 1, &sz);
    width = luaL_checkinteger(L, 2);
    height = luaL_checkinteger(L, 3);
    bytes_per_pixel = luaL_optinteger(L, 4, 3);
    compression_level = luaL_optinteger(L, 5, 3);
    break;

  case LUA_TNUMBER:
    data = (unsigned char *)lua_tointeger(L, 1);
    sz = luaL_checkinteger(L, 2);
    width = luaL_checkinteger(L, 3);
    height = luaL_checkinteger(L, 4);
    bytes_per_pixel = luaL_optinteger(L, 5, 3);
    compression_level = luaL_optinteger(L, 6, 3);
    break;

  default:
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "Input type is not supported");
    return 2;
  }

  png_byte color_type;
  switch (bytes_per_pixel) {
  case 1:
    color_type = PNG_COLOR_TYPE_GRAY;
    break;
  case 2:
    color_type = PNG_COLOR_TYPE_GRAY_ALPHA;
    break;
  case 3:
    color_type = PNG_COLOR_TYPE_RGB;
    break;
  case 4:
    color_type = PNG_COLOR_TYPE_RGB_ALPHA;
    break;
  default:
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "Bad PNG Color type");
    return 2;
  }

  int stride = width * bytes_per_pixel;

  /* initialize stuff */
  png_structp png_ptr =
      png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
  png_set_compression_level(png_ptr, compression_level);

  if (!png_ptr) {
    abort_("[write_png_file] png_create_write_struct failed");
  }

  png_infop info_ptr = png_create_info_struct(png_ptr);
  if (!info_ptr)
    abort_("[write_png_file] png_create_info_struct failed");

  if (setjmp(png_jmpbuf(png_ptr)))
    abort_("[write_png_file] Error during init_io");

  /* write header */
  if (setjmp(png_jmpbuf(png_ptr)))
    abort_("[write_png_file] Error during writing header");

  png_set_IHDR(png_ptr, info_ptr, width, height, bit_depth, color_type,
               PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_BASE,
               PNG_FILTER_TYPE_BASE);

  // fprintf(stderr, "About to write...\n");
  struct mem_encode state;
  state.size = sz >> 2;
  state.written = 0;
  state.buffer = (char *)malloc(state.size); // Guess 25% compression
  png_set_write_fn(png_ptr, &state, lua_png_write_string, NULL);

  png_write_info(png_ptr, info_ptr);

  /* write bytes */
  if (setjmp(png_jmpbuf(png_ptr)))
    abort_("[write_png_file] Error during writing bytes");

  png_bytep *row_pointers = (png_bytep *)malloc(sizeof(png_bytep) * height);

  int y;
  for (y = 0; y < height; y++) {
    row_pointers[y] = data;
    data += stride;
  }

  png_write_image(png_ptr, row_pointers);

  /* end write */
  if (setjmp(png_jmpbuf(png_ptr))) {
    if (state.buffer) {
      free(state.buffer);
      state.buffer = NULL;
    }
    abort_("[write_png_file] Error during end of write");
  }

  png_write_end(png_ptr, NULL);

  if (state.size > 0) {
    lua_pushlstring(L, (const char *)state.buffer, state.written);
  } else {
    lua_pushboolean(L, 0);
  }

  if (state.buffer) {
    free(state.buffer);
  }
  free(row_pointers);

  return 1;
}

void lua_png_read_string(png_structp png_ptr, png_bytep data,
                         png_size_t length) {
  struct mem_encode *a = (struct mem_encode *)png_get_io_ptr(png_ptr);
  memcpy(data, a->buffer, length);
  a->buffer += length;
}

static int lua_png_uncompress(lua_State *L) {
  struct mem_encode state;
  state.buffer = (char *)luaL_checkstring(L, 1);
  state.size = 0;

  structPNG *ud = (structPNG *)lua_newuserdata(L, sizeof(structPNG));

  char header[8]; // 8 is the maximum size that can be checked
  memcpy(header, state.buffer, 8);
  state.buffer += 8;
  state.size += 8;

  if (png_sig_cmp((const png_bytep)header, 0, 8))
    abort_("[read_png_stream] stream is not recognized as a PNG file");

  /* initialize stuff */
  ud->png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);

  if (!ud->png_ptr)
    abort_("[read_png_file] png_create_read_struct failed");

  ud->info_ptr = png_create_info_struct(ud->png_ptr);
  if (!ud->info_ptr)
    abort_("[read_png_file] png_create_info_struct failed");

  png_set_read_fn(ud->png_ptr, &state, lua_png_read_string);

  if (setjmp(png_jmpbuf(ud->png_ptr)))
    abort_("[read_png_file] Error during init_io");

  png_set_sig_bytes(ud->png_ptr, 8);
  png_read_info(ud->png_ptr, ud->info_ptr);
  png_set_keep_unknown_chunks(ud->png_ptr, 1, NULL, 0);

  ud->width = png_get_image_width(ud->png_ptr, ud->info_ptr);
  ud->height = png_get_image_height(ud->png_ptr, ud->info_ptr);
  ud->color_type = png_get_color_type(ud->png_ptr, ud->info_ptr);
  ud->bit_depth = png_get_bit_depth(ud->png_ptr, ud->info_ptr);

  ud->number_of_passes = png_set_interlace_handling(ud->png_ptr);
  png_read_update_info(ud->png_ptr, ud->info_ptr);

  /* read file */
  if (setjmp(png_jmpbuf(ud->png_ptr)))
    abort_("[read_png_file] Error during read_image");

  ud->row_pointers = (png_bytep *)malloc(sizeof(png_bytep) * ud->height);

  int y;
  ud->stride = png_get_rowbytes(ud->png_ptr, ud->info_ptr);
  for (y = 0; y < ud->height; y++) {
    ud->row_pointers[y] =
        (png_byte *)malloc(png_get_rowbytes(ud->png_ptr, ud->info_ptr));
  }

  png_read_image(ud->png_ptr, ud->row_pointers);

  luaL_getmetatable(L, MT_NAME);
  lua_setmetatable(L, -2);
  return 1;
}

static const struct luaL_Reg png_Functions[] = {
    {"compress", lua_png_compress},
    {"uncompress", lua_png_uncompress},
    {NULL, NULL}};

static const struct luaL_Reg png_Methods[] = {
    {"dims", lua_png_dims}, {"__gc", lua_png_delete}, {NULL, NULL}};

#ifdef __cplusplus
extern "C"
#endif
    int
    luaopen_png(lua_State *L) {
  luaL_newmetatable(L, MT_NAME);

#if LUA_VERSION_NUM == 501
  luaL_register(L, NULL, png_Methods);
  luaL_register(L, "png", png_Functions);
#else
  luaL_setfuncs(L, png_Methods, 0);
  luaL_newlib(L, png_Functions);
#endif
  return 1;
}

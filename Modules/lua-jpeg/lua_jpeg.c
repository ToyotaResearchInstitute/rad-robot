/*
 * (c) 2013 Dan Lee, Alex Kushlyev, Steve McGill, Yida Zhang
 * ddlee@seas.upenn.edu, smcgill3@seas.upenn.edu
 * University of Pennsylvania
 * */

#include <lauxlib.h>
#include <lua.h>

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <jpeglib.h>

/* UDP Friendly size (2^16) */
#define BUF_SZ 65536
//#define BUF_SZ 131072
#define DEFAULT_QUALITY 85
#define USE_JFIF FALSE
// TurboJPEG: fastest
#define DEFAULT_DCT_METHOD JDCT_FASTEST
//#define DEFAULT_DCT_METHOD JDCT_IFAST

/* Define the functions to use */
void error_exit_compress(j_common_ptr cinfo);
void init_destination(j_compress_ptr cinfo);
void term_destination(j_compress_ptr cinfo);
boolean empty_output_buffer(j_compress_ptr cinfo);

// Define the metatable for JPEGs
#define MT_NAME "jpeg_mt"
typedef struct {
  j_common_ptr cinfo;
  JOCTET *buffer;
  size_t buffer_sz;
  uint8_t subsample; // Downsampling if needed
  uint8_t fmt;
} structJPEG;
// Be able to check the input of a jpeg
static structJPEG *lua_checkjpeg(lua_State *L, int narg) {
  void *ud = luaL_checkudata(L, narg, MT_NAME);
  luaL_argcheck(L, ud != NULL, narg, "invalid jpeg");
  return (structJPEG *)ud;
}

void error_exit_compress(j_common_ptr cinfo) {
#ifdef DEBUG
  fprintf(stderr, "Error exit!\n");

#endif
  (*cinfo->err->output_message)(cinfo);
  jpeg_destroy_compress((j_compress_ptr)cinfo);
}
void init_destination(j_compress_ptr cinfo) {
#ifdef DEBUG
  fprintf(stderr, "Init destination!\n");

#endif
  structJPEG *ud = (structJPEG *)cinfo->client_data;

  if (ud->buffer == NULL) {
    ud->buffer = (JOCTET *)malloc(BUF_SZ);
#ifdef DEBUG
    fprintf(stderr, "Malloc! %p\n", (void *)ud->buffer);

#endif
  }
  ud->buffer_sz = BUF_SZ;
  cinfo->dest->next_output_byte = ud->buffer;
  cinfo->dest->free_in_buffer = BUF_SZ;
#ifdef DEBUG
  printf("nb: %p, %zu\n", (void *)cinfo->dest->next_output_byte,
         cinfo->dest->free_in_buffer);
#endif
}
void term_destination(j_compress_ptr cinfo) {
#ifdef DEBUG
  fprintf(stderr, "Term destination!\n");

#endif

  structJPEG *ud = (structJPEG *)cinfo->client_data;
  JOCTET *destBuf = ud->buffer;

  // How much left
  int len = ud->buffer_sz - cinfo->dest->free_in_buffer;
  // TODO: Unsure what this does... (align?)
  while (len % 2 != 0)
    destBuf[len++] = 0xFF;

  ud->buffer_sz = len;
// Reallocate is unnecessary to smaller size
/*
ud->buffer = realloc(ud->buffer, len);
if(ud->buffer == NULL){
#ifdef DEBUG
  fprintf(stderr,"Bad realloc!\n");

#endif
  free(destBuf);
}
*/
#ifdef DEBUG
  fprintf(stderr, "Done Term destination!\n");

#endif
}
boolean empty_output_buffer(j_compress_ptr cinfo) {
#ifdef DEBUG
  fprintf(stderr, "Error buffer too small!\n");

#endif
  structJPEG *ud = (structJPEG *)cinfo->client_data;
  JOCTET *destBuf = ud->buffer;
  // TODO: Use realloc instead of the vector, for C only
  // Reallocate
  size_t new_size = ud->buffer_sz * 2;
  ud->buffer = realloc(ud->buffer, new_size);
  if (ud->buffer == NULL) {
#ifdef DEBUG
    fprintf(stderr, "Bad realloc!\n");

#endif
    free(destBuf);
  }

  cinfo->dest->free_in_buffer = ud->buffer_sz;
  cinfo->dest->next_output_byte = ud->buffer + ud->buffer_sz;
  ud->buffer_sz *= 2;

  return TRUE;
}

/* Initialize a new compressor struct */
j_compress_ptr new_cinfo(structJPEG *ud) {

  // Allocate on the heap
  // TODO: Garbage collection on the structJPEG metatable
  j_compress_ptr compress_info = malloc(sizeof(struct jpeg_compress_struct));
  j_common_ptr cinfo = (j_common_ptr)compress_info;

  // Setup error handling
  // TODO: Why malloc this?
  struct jpeg_error_mgr *jerr = malloc(sizeof(struct jpeg_error_mgr));
  cinfo->err = jpeg_std_error(jerr);
  cinfo->err->error_exit = error_exit_compress;

  // Initialize the struct for compression
  jpeg_create_compress(compress_info);
  if (compress_info->dest == NULL) {
#ifdef DEBUG
    fprintf(stderr, "Make pool!\n");

#endif
    compress_info->dest =
        (struct jpeg_destination_mgr *)(*cinfo->mem->alloc_small)(
            cinfo, JPOOL_PERMANENT, sizeof(struct jpeg_destination_mgr));
  }
#ifdef DEBUG
  printf("dest: %p\n", (void *)compress_info->dest);
#endif
  compress_info->dest->init_destination = init_destination;
  compress_info->dest->empty_output_buffer = empty_output_buffer;
  compress_info->dest->term_destination = term_destination;

  // set the client data for the compressor
  cinfo->client_data = ud;
  // Some compression properties
  compress_info->write_JFIF_header = USE_JFIF;
  compress_info->dct_method = DEFAULT_DCT_METHOD;

  jpeg_set_quality(compress_info, DEFAULT_QUALITY, TRUE);

  // NULL the buffer for use by jpeg
  ud->buffer = NULL;
  ud->cinfo = cinfo;

  return compress_info;
}

// Just compress a 2D array that is already in memory
// Crop the image, though
static int lua_jpeg_compress_crop(lua_State *L) {

  JDIMENSION width, height, w0, w_cropped, h0, h_cropped;
  JSAMPLE *data;
  size_t stride;
  // JPEG struct with buffer and cinfo is our first
  structJPEG *ud = lua_checkjpeg(L, 1);

  // Access the JPEG compression settings
  j_compress_ptr cinfo = (j_compress_ptr)ud->cinfo;

  if (lua_isstring(L, 2)) {
    size_t sz = 0;
    data = (JSAMPLE *)lua_tolstring(L, 2, &sz);
    width = luaL_checkinteger(L, 3);
    height = luaL_checkinteger(L, 4);
    stride = cinfo->input_components * width;
  } else if (lua_islightuserdata(L, 2)) {
    data = (JSAMPLE *)lua_touserdata(L, 2);
    width = luaL_checkinteger(L, 3);
    height = luaL_checkinteger(L, 4);
    stride = cinfo->input_components * width;
  } else {
    return luaL_error(L, "Bad JPEG Compress 16 input");
  }

  // Start and stop for crop (put into C indices)
  w0 = luaL_optint(L, 5, 1) - 1;
  h0 = luaL_optint(L, 6, 1) - 1;
  w_cropped = luaL_checkinteger(L, 7);
  h_cropped = luaL_checkinteger(L, 8);

  if (h_cropped + h0 > height || w_cropped + w0 > width) {
    return luaL_error(L, "Bad crop");
  }

#ifdef DEBUG
  fprintf(stderr, "w: %d, h: %d | %d %d\n", width, height, cinfo->image_width,
          cinfo->image_height);
  fprintf(stderr, "Data: %p\n", (void *)data);

#endif

  // Colorspace of the OUTPUTTED jpeg same is input (save space/speed?)
  // YCbCr and Grayscale are JFIF. RGB and others are Adobe
  jpeg_set_colorspace(cinfo, cinfo->in_color_space);

  // Copy the reference to data for pointer arithmetic
  JDIMENSION nlines;
  JSAMPROW row_pointer[1];
  JSAMPLE *img_ptr = data;

  size_t remainder = 0;
  if (ud->fmt == 3 || ud->fmt == 4) {
    // Safe cropping for YUYV (align to a pixel)
    if (ud->subsample == 2) {
      w0 -= (w0 % 4);
      w_cropped -= (w_cropped % 4);
    } else {
      w0 -= w0 % 2;
      w_cropped -= (w_cropped % 2);
    }
    // YUYV is 2 bytes per pix
    img_ptr += 2 * h0 * width + w0;
    remainder = 2 * (width - w_cropped);
  } else {
    img_ptr += cinfo->input_components * (width * h0 + w0);
  }

  // Set the width and height for compression
  cinfo->image_width = w_cropped >> ud->subsample;
  cinfo->image_height = h_cropped >> ud->subsample;
  int w = cinfo->image_width;
  int h = cinfo->image_height;

#ifdef DEBUG
  int line = 0;
  printf("remainder %zu", remainder);
#endif

  // Begin compression
  jpeg_start_compress(cinfo, TRUE);

  // YUYV is special
  if (ud->fmt == 3 || ud->fmt == 4) {
    size_t img_stride = 2 * width * ud->subsample;
    img_stride += remainder;
    JSAMPLE *yuv_row;
    if (ud->fmt == 3) {
      yuv_row = (JSAMPLE *)malloc(3 * w * sizeof(JSAMPLE));
    } else {
      yuv_row = (JSAMPLE *)malloc(w * sizeof(JSAMPLE));
    }
    if (yuv_row == NULL) {
      return luaL_error(L, "Bad malloc of yuv_row");
    }
    *row_pointer = yuv_row;
    int i;
    uint8_t y0, u, y1, v;
    while (cinfo->next_scanline < h) {
      JSAMPLE *yuv_pixel = yuv_row;
#ifdef DEBUG
      line++;
#endif
      i = 0;
      while (i < w) {
        y0 = *img_ptr++;
        u = *img_ptr++;
        y1 = *img_ptr++;
        v = *img_ptr++;
        if (ud->subsample == 2) {
          // Skip the next pixel, too
          img_ptr += 4;
        }
        //
        *yuv_pixel = y0;
        yuv_pixel++;
        if (ud->fmt == 3) {
          *yuv_pixel = u;
          yuv_pixel++;
          *yuv_pixel = v;
          yuv_pixel++;
        }
        if (ud->subsample) {
          // If subsampling, then we add only one pixel
          i++;
        } else {
          // Ignore this pixel if subsampling
          *yuv_pixel = y1;
          yuv_pixel++;
          if (ud->fmt == 3) {
            *yuv_pixel = u;
            yuv_pixel++;
            *yuv_pixel = v;
            yuv_pixel++;
          }
          // 2 pixels
          i += 2;
        }

#ifdef DEBUG
/*
        if(line<2){
          printf("line: (%d,%d): Y: %d, U: %d, Y: %d, V: %d, %d\n",line,
   i,y0,u,y1,v,yuv_row[0]);
        }
*/
#endif
      }
      nlines = jpeg_write_scanlines(cinfo, row_pointer, 1);
      if (ud->subsample) {
        // Skip a row if this subsample level
        img_ptr += img_stride;
      }
    }
    free(yuv_row);
  } else {
    while (cinfo->next_scanline < h) {
      *row_pointer = img_ptr;
      nlines = jpeg_write_scanlines(cinfo, row_pointer, 1);
      img_ptr += stride;
    }
  }

#ifdef DEBUG
  fprintf(stderr, "len: %zu %zu %p\n", cinfo->dest->free_in_buffer,
          ud->buffer_sz, ud->buffer);

#endif

  jpeg_finish_compress(cinfo);
  lua_pushlstring(L, (const char *)ud->buffer, ud->buffer_sz);
  return 1;
}

// Just compress a 2D array that is already in memory
static int lua_jpeg_compress(lua_State *L) {

  // JPEG struct with buffer and cinfo is our first
  structJPEG *ud = lua_checkjpeg(L, 1);

  // We will not modify the data
  // Use the JPEG typedef notations
  JSAMPLE *data;
  JDIMENSION width, height;
  size_t len = 0;

  switch (lua_type(L, 2)) {
  case LUA_TLIGHTUSERDATA:
    data = (JSAMPLE *)lua_touserdata(L, 2);
    len = luaL_checkinteger(L, 3);
    width = luaL_checkinteger(L, 4);
    height = luaL_checkinteger(L, 5);
    break;
  case LUA_TSTRING:
    data = (JSAMPLE *)lua_tolstring(L, 2, &len);
    width = luaL_checkinteger(L, 3);
    height = luaL_checkinteger(L, 4);
    break;
  case LUA_TNUMBER:
    data = (JSAMPLE *)lua_tointeger(L, 2);
    len = luaL_checkinteger(L, 3);
    width = luaL_checkinteger(L, 4);
    height = luaL_checkinteger(L, 5);
    break;
  default:
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "Input type is not supported");
    return 2;
  }

  // Access the JPEG compression settings
  j_compress_ptr cinfo = (j_compress_ptr)ud->cinfo;
  // Set the width and height for compression
  cinfo->image_width = width >> ud->subsample;
  cinfo->image_height = height >> ud->subsample;
  // TODO: Does stride require subsample?
  size_t stride = cinfo->input_components * width;

  // Colorspace of the OUTPUTTED jpeg same is input (save space/speed?)
  // YCbCr and Grayscale are JFIF. RGB and others are Adobe
  jpeg_set_colorspace(cinfo, cinfo->in_color_space);

  // Copy the reference to data for pointer arithmetic
  JDIMENSION nlines;
  JSAMPROW row_pointer[1];
  JSAMPLE *img_ptr = data;

  // Begin compression
  jpeg_start_compress(cinfo, TRUE);

  // YUYV is special
  int w = cinfo->image_width;
  int h = cinfo->image_height;
  if (ud->fmt == 3 && ud->subsample == 0) {
    JSAMPLE *yuv_row = malloc(3 * w * sizeof(JSAMPLE));
    if ((*row_pointer = yuv_row) == NULL) {
      return luaL_error(L, "Bad malloc of yuv_row");
    }
    uint8_t y0, u, y1, v;
    while (cinfo->next_scanline < h) {
      JSAMPLE *yuv_pixel = yuv_row;
      // Set two pixels each loop
      for (int iw = 0; iw < w; iw += 2) {
        y0 = *img_ptr++;
        u = *img_ptr++;
        y1 = *img_ptr++;
        v = *img_ptr++;
        *yuv_pixel++ = y0;
        *yuv_pixel++ = u;
        *yuv_pixel++ = v;
        *yuv_pixel++ = y1;
        *yuv_pixel++ = u;
        *yuv_pixel++ = v;
      }
      nlines = jpeg_write_scanlines(cinfo, row_pointer, 1);
    }
    free(yuv_row);
  } else if (ud->fmt == 4 && ud->subsample == 0) {
    JSAMPLE *yuv_row = malloc(w * sizeof(JSAMPLE));
    if ((*row_pointer = yuv_row) == NULL) {
      return luaL_error(L, "Bad malloc of yuv_row");
    }
    while (cinfo->next_scanline < h) {
      JSAMPLE *yuv_pixel = yuv_row;
      // Set two pixels each loop
      for (int iw = 0; iw < w; iw += 2) {
        *yuv_pixel++ = *img_ptr;   // y0
        *yuv_pixel++ = img_ptr[2]; // y1
        img_ptr += 4;
      }
      nlines = jpeg_write_scanlines(cinfo, row_pointer, 1);
    }
    free(yuv_row);
  } else if (ud->fmt == 3 && ud->subsample == 1) {
    JSAMPLE *yuv_row = malloc(3 * w * sizeof(JSAMPLE));
    if ((*row_pointer = yuv_row) == NULL) {
      return luaL_error(L, "Bad malloc of yuv_row");
    }
    size_t img_stride = 2 * width;
    uint8_t y0, u, y1, v;
    while (cinfo->next_scanline < h) {
      JSAMPLE *yuv_pixel = yuv_row;
      // If subsampling, then we add only one pixel each loop
      for (int iw = 0; iw < w; iw += 1) {
        // fprintf(stderr, "iw: %d\n", iw);
        y0 = *img_ptr++;
        u = *img_ptr++;
        y1 = *img_ptr++;
        v = *img_ptr++;
        // TODO: img_ptr at the next row has yuyv values, too, for averaging
        *yuv_pixel++ = ((int)y0 + y1) / 2; // Avoid Overflow
        *yuv_pixel++ = u;
        *yuv_pixel++ = v;
      }
      nlines = jpeg_write_scanlines(cinfo, row_pointer, 1);
      img_ptr += img_stride;
    }
    free(yuv_row);
  } else if (ud->fmt == 4 && ud->subsample == 1) {
    JSAMPLE *yuv_row = malloc(w * sizeof(JSAMPLE));
    if ((*row_pointer = yuv_row) == NULL) {
      return luaL_error(L, "Bad malloc of yuv_row");
    }
    size_t img_stride = 2 * width;
    while (cinfo->next_scanline < h) {
      JSAMPLE *yuv_pixel = yuv_row;
      // If subsampling, then we add only one pixel each loop
      for (int iw = 0; iw < w; iw += 1) {
        *yuv_pixel++ = ((int)img_ptr[0] + img_ptr[2]) / 2; // mean(y0, y1)
        img_ptr += 4;
      }
      nlines = jpeg_write_scanlines(cinfo, row_pointer, 1);
      img_ptr += img_stride;
    }
    free(yuv_row);
  } else if (ud->fmt == 3 || ud->fmt == 4) {
    size_t img_stride = 2 * width * ud->subsample;
    JSAMPLE *yuv_row;
    if (ud->fmt == 3) {
      yuv_row = malloc(3 * w * sizeof(JSAMPLE));
    } else {
      yuv_row = malloc(w * sizeof(JSAMPLE));
    }
    if (yuv_row == NULL) {
      return luaL_error(L, "Bad malloc of yuv_row");
    }
    *row_pointer = yuv_row;
    int i;
    uint8_t y0, u, y1, v;
    while (cinfo->next_scanline < h) {
      JSAMPLE *yuv_pixel = yuv_row;
      i = 0;
      while (i < w) {
        y0 = *img_ptr++;
        u = *img_ptr++;
        y1 = *img_ptr++;
        v = *img_ptr++;
        if (ud->subsample == 2) {
          // Skip the next pixel, too
          img_ptr += 4;
        }
        //
        *yuv_pixel++ = y0;
        if (ud->fmt == 3) {
          *yuv_pixel++ = u;
          *yuv_pixel++ = v;
        }
        if (ud->subsample) {
          // If subsampling, then we add only one pixel
          i++;
        } else { // Add two pixels when not subsampling
          i += 2;
          *yuv_pixel++ = y1;
          if (ud->fmt == 3) {
            *yuv_pixel++ = u;
            *yuv_pixel++ = v;
          }
        }
      }
      nlines = jpeg_write_scanlines(cinfo, row_pointer, 1);
      if (ud->subsample) {
        // Skip a row if this subsample level
        img_ptr += img_stride;
      }
    }
    free(yuv_row);
  } else {
    while (cinfo->next_scanline < h) {
      *row_pointer = img_ptr;
      nlines = jpeg_write_scanlines(cinfo, row_pointer, 1);
      img_ptr += stride;
    }
  }

  jpeg_finish_compress(cinfo);
  lua_pushlstring(L, (const char *)ud->buffer, ud->buffer_sz);
  return 1;
}

static int lua_jpeg_new_compressor(lua_State *L) {
  // Form the struct of metadata
  // The function also pushes it to the stack
  structJPEG *ud = lua_newuserdata(L, sizeof(structJPEG));
  // Compressor format
  const char *fmt = luaL_checkstring(L, 1);
  // Form the compressor
  j_compress_ptr cinfo = new_cinfo(ud);
  ud->subsample = 0;

  if (strncmp(fmt, "rgb", 3) == 0) {
    cinfo->in_color_space = JCS_RGB;
    cinfo->input_components = 3;
    ud->fmt = 0;
  } else if (strncmp(fmt, "gray", 4) == 0) {
    cinfo->in_color_space = JCS_GRAYSCALE;
    cinfo->input_components = 1;
    ud->fmt = 1;
  } else if (strncmp(fmt, "yuv", 4) == 0) {
    cinfo->in_color_space = JCS_YCbCr;
    cinfo->input_components = 3;
    ud->fmt = 2;
  } else if (strncmp(fmt, "yuyv", 4) == 0) {
    cinfo->in_color_space = JCS_YCbCr;
    cinfo->input_components = 3;
    ud->fmt = 3;
  } else if (strncmp(fmt, "y", 3) == 0) {
    // Use only the y channel
    cinfo->in_color_space = JCS_GRAYSCALE;
    cinfo->input_components = 1;
    ud->fmt = 4;
  } else {
    return luaL_error(L, "Unsupported format.");
  }

  // Set the defaults based on the colorspace
  // http://refspecs.linuxbase.org/LSB_3.1.0/LSB-Desktop-generic/LSB-Desktop-generic/libjpeg.jpeg.set.defaults.1.html
  jpeg_set_defaults(cinfo);

  // Set the metatable information
  luaL_getmetatable(L, MT_NAME);
  lua_setmetatable(L, -2);
  // only pushed the metatable
  return 1;
}

static int lua_jpeg_quality(lua_State *L) {
  structJPEG *ud = lua_checkjpeg(L, 1);
  int quality = luaL_checkinteger(L, 2);
  if (quality < 1 || quality > 98)
    return luaL_error(L, "Quality must be in range of 1 to 98");
  jpeg_set_quality((j_compress_ptr)ud->cinfo, quality, TRUE);
  return 0;
}

static int lua_jpeg_downsampling(lua_State *L) {
  structJPEG *ud = lua_checkjpeg(L, 1);
  if (ud->fmt != 3 && ud->fmt != 4) {
    return luaL_error(L, "Only YUYV subsampling!");
  }
  int dsample = luaL_checkinteger(L, 2);
  if (dsample < 0 || dsample > 2) {
    return luaL_error(L, "Bad subsample factor!");
  }
  ud->subsample = dsample;
  return 0;
}

static int lua_jpeg_delete(lua_State *L) {
  structJPEG *ud = lua_checkjpeg(L, 1);
  /* cleanup heap allocations */
  if (ud->buffer != NULL) {
    free(ud->buffer);
  }
  if (ud->cinfo->err != NULL) {
    free(ud->cinfo->err);
  }
  if (ud->cinfo != NULL) {
    jpeg_destroy_compress((j_compress_ptr)ud->cinfo);
    free(ud->cinfo);
  }
  return 1;
}

static int lua_jpeg_tostring(lua_State *L) {
  structJPEG *ud = lua_checkjpeg(L, 1);
  j_compress_ptr cinfo = (j_compress_ptr)ud->cinfo;
  char buffer[64];
  switch (cinfo->in_color_space) {
  case JCS_RGB:
    sprintf(buffer, "RGB: %d channels.", cinfo->input_components);
    break;
  case JCS_GRAYSCALE:
    sprintf(buffer, "Gray: %d channels.", cinfo->input_components);
    break;
  case JCS_YCbCr:
    sprintf(buffer, "YCbCr: %d channels.", cinfo->input_components);
    break;
  default:
    sprintf(buffer, "Unknown: %d channels.", cinfo->input_components);
  }
  lua_pushstring(L, buffer);

  return 1;
}

static int lua_jpeg_index(lua_State *L) {
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

static const struct luaL_Reg jpeg_Methods[] = {
    {"compress", lua_jpeg_compress}, {"compress_crop", lua_jpeg_compress_crop},
    {"quality", lua_jpeg_quality},   {"downsampling", lua_jpeg_downsampling},
    {"__gc", lua_jpeg_delete},       {"__tostring", lua_jpeg_tostring},
    {"__index", lua_jpeg_index},     {NULL, NULL}};

static const struct luaL_Reg jpeg_Functions[] = {
    {"compressor", lua_jpeg_new_compressor}, {NULL, NULL}};

#ifdef __cplusplus
extern "C"
#endif
    int
    luaopen_jpeg(lua_State *L) {
  luaL_newmetatable(L, MT_NAME);

#if LUA_VERSION_NUM == 501
  luaL_register(L, NULL, jpeg_Methods);
  luaL_register(L, "jpeg", jpeg_Functions);
#else
  luaL_setfuncs(L, jpeg_Methods, 0);
  luaL_newlib(L, jpeg_Functions);
#endif

  return 1;
}

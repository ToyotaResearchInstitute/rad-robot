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

#include <libusb-1.0/libusb.h>
#include <libuvc/libuvc.h>

#define NBUFFERS 2
/* metatable name for uvc */
#define MT_NAME "uvc_mt"

typedef struct {
  int init;
  int width;
  int height;
  unsigned int count;
  const char *pixelformat;
  int fps_num;
  int fps_denum;
  // Pipe filedescriptor
  int pipefd[2];
  void *buffer;
  size_t buf_sz;
  /*
    // Buffer management
    int index;
    void** buffer;
    int* buf_len;
    int* buf_used;
  */
  // USB context management
  struct libusb_context *usb_ctx;
  uvc_context_t *ctx;
  uvc_device_t *dev;
  uvc_device_handle_t *devh;
  const struct libusb_pollfd **fds;
  int nfds;
} camera_device;

void cb(uvc_frame_t *frame, void *ptr) {
  camera_device *ud = (camera_device *)ptr;
  // Not populated (at least on macOS)
  // int64_t utime =
  //     1e6 * (int64_t)frame->capture_time.tv_sec +
  //     frame->capture_time.tv_usec;

#ifdef DEBUG
  fprintf(stderr, "Frame #%2d (%d x %d) %zu bytes\n", frame->sequence,
          frame->width, frame->height, frame->data_bytes);
#endif
  /*
#ifdef DEBUG
  fprintf(stderr, "Buffer [%d]\n", ud->index);
#endif
    int index_buffer_next = ud->index;
    if (index_buffer_next >= NBUFFERS) {
      index_buffer_next = 0;
    }
  // Copy and set the index
  memcpy(ud->buffer[index_buffer_next], frame->data, frame->data_bytes);
  ud->buf_used[index_buffer_next] = frame->data_bytes;
  ud->index = index_buffer_next;
  */
  ud->buffer = frame->data;
  ud->buf_sz = frame->data_bytes;
  ud->count = frame->sequence;
  ssize_t ret = write(ud->pipefd[1], &ud->count, sizeof(ud->count));
}

void added_cb(int fd, short events, void *user_data) {
#ifdef DEBUG
  fprintf(stderr, "\n\n\nAdded fd %d: %d\n", fd, events);
#endif
}
void removed_cb(int fd, void *user_data) {
#ifdef DEBUG
  fprintf(stderr, "\n\n\nRemoved fd %d\n", fd);
#endif
}

static camera_device *lua_checkuvc(lua_State *L, int narg) {
  void *ud = luaL_checkudata(L, narg, MT_NAME);
  luaL_argcheck(L, ud != NULL, narg, "invalid uvc userdata");
  return (camera_device *)ud;
}

static int lua_uvc_index(lua_State *L) {
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

static int lua_uvc_delete(lua_State *L) {
#ifdef DEBUG
  fprintf(stderr, "Cleaning...\n");
#endif
  camera_device *ud = lua_checkuvc(L, 1);
#ifdef DEBUG
  fprintf(stderr, "De-init...\n");
#endif
  ud->init = 0;

  if (ud->fds) {
#ifdef DEBUG
    fprintf(stderr, "Cleaning fds\n");
#endif
    libusb_free_pollfds(ud->fds);
    ud->fds = NULL;
  }

  // This runs uvc_close on all ctx->open_devices
  if (ud->ctx) {
#ifdef DEBUG
    fprintf(stderr, "Cleaning uvc_exit\n");
#endif
    uvc_exit(ud->ctx);
    ud->ctx = NULL;
    ud->devh = NULL;
  }

  if (ud->usb_ctx) {
    libusb_exit(ud->usb_ctx);
    ud->usb_ctx = NULL;
  }

  // Image buffers: Conservative size of RGB, right now...
  /*
    if (ud->buffer) {
      for (int ibuf = 0; ibuf < NBUFFERS; ibuf++) {
        if (ud->buffer[ibuf]) {
  #ifdef DEBUG
          fprintf(stderr, "Cleaning buffer %d\n", ibuf);
  #endif
          free(ud->buffer[ibuf]);
          ud->buffer[ibuf] = NULL;
        }
      }
  #ifdef DEBUG
      fprintf(stderr, "Cleaning buffer\n");
  #endif
      free(ud->buffer);
      ud->buffer = NULL;
    }
    if (ud->buf_used) {
  #ifdef DEBUG
      fprintf(stderr, "Cleaning buf_used\n");
  #endif
      free(ud->buf_used);
      ud->buf_used = NULL;
    }
    */
  // No buffer available, now
  ud->buffer = NULL;
  ud->buf_sz = 0;

  // Pipe
  close(ud->pipefd[0]);
  ud->pipefd[0] = 0;
  close(ud->pipefd[1]);
  ud->pipefd[1] = 0;
#ifdef DEBUG
  fprintf(stderr, "Cleaning done!\n");
#endif
  return 0;
}

static int lua_uvc_init(lua_State *L) {
  camera_device *ud = lua_newuserdata(L, sizeof(camera_device));
  // Zero the data so that any initial access is NULL
  bzero(ud, sizeof(camera_device));
  // Populate the userdata
  // int n = lua_gettop(L);
  const char *video_device = lua_isstring(L, 1) ? lua_tostring(L, 1) : NULL;
  ud->width = luaL_optint(L, 2, 320);
  ud->height = luaL_optint(L, 3, 240);
  ud->pixelformat = luaL_optstring(L, 4, "yuyv");
  /* default 15 fps */
  ud->fps_num = luaL_optint(L, 5, 1);
  ud->fps_denum = luaL_optint(L, 6, 15);
  _Bool make_usb = lua_isboolean(L, 7) && lua_toboolean(L, 3);

  uvc_error_t res;
  uvc_context_t *ctx = ud->ctx;
  uvc_device_t *dev = ud->dev;

  if (make_usb) {
#ifdef DEBUG
    fprintf(stderr, "Initializing libusb...\n");
#endif
    if (libusb_init(&ud->usb_ctx) != 0) {
      lua_pushboolean(L, 0);
      lua_pushliteral(L, "libusb_init: Could not initialize libusb");
      return 2;
    }
    if (libusb_pollfds_handle_timeouts(ud->usb_ctx) == 0) {
#ifdef DEBUG
      fprintf(stderr,
              "libusb_pollfds_handle_timeouts says the system is old\n");
#endif
    } else {
#ifdef DEBUG
      fprintf(stderr,
              "libusb_pollfds_handle_timeouts says the system is new\n");
#endif
    }
#ifdef DEBUG
    fprintf(stderr, "Set libusb poll notifiers\n");
#endif
    libusb_set_pollfd_notifiers(ud->usb_ctx, &added_cb, &removed_cb, ud);
  } else {
    ud->usb_ctx = NULL;
  }

  ud->fds = NULL;

#ifdef DEBUG
  fprintf(stderr, "Initializing UVC [%p]\n", ud->usb_ctx);
#endif
  res = uvc_init(&ctx, ud->usb_ctx);
  if (res != UVC_SUCCESS) {
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "uvc_init");
    return 2;
  }

#ifdef DEBUG
  fprintf(stderr, "Finding device...\n");
#endif
  if (uvc_find_device(ctx, &dev, 0, 0, video_device) < 0) {
    uvc_exit(ctx);
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "uvc_find_device: no devices found");
    return 2;
  }

#ifdef DEBUG
  fprintf(stderr, "Opening device...\n");
#endif
  res = uvc_open(dev, &ud->devh);
  if (res != UVC_SUCCESS) {
#ifdef DEBUG
    fprintf(stderr, "Could not open device: %d / %d\n", res,
            LIBUSB_ERROR_ACCESS);
#endif
    uvc_exit(ctx);
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "uvc_open: unable to open device");
    return 2;
  }
  // NOTE: Now the handler thread is started!
  // We need to handle events properly if using our own usb context
  // while (!ctx->kill_handler_thread)
  //    libusb_handle_events_completed(ctx->usb_ctx, &ctx->kill_handler_thread);

#ifdef DEBUG
  // These diagnostics are important for keeping USB utilization in check
  fprintf(stderr, "Diagnostics...\n");
  uvc_print_diag(ud->devh, stderr);
#endif

#ifdef DEBUG
  fprintf(stderr, "Initializing memory\n");
#endif

  // Image buffers: Conservative size of RGB, right now...
  // NOTE: Actually, use the libuvc buffer, for now
  /*
    ud->buffer = calloc(NBUFFERS, sizeof(void*));
    for (int ibuf = 0; ibuf < NBUFFERS; ibuf++) {
      ud->buffer[ibuf] = malloc(ud->width * ud->height * 3);
    }
    ud->buf_used = calloc(NBUFFERS, sizeof(size_t));
  */

  // Form communication channel for polling
  if (pipe(ud->pipefd) != 0) {
    uvc_exit(ctx);
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "Cannot pipe");
  }
  // Set the read end to non blocking mode
  fcntl(ud->pipefd[0], F_SETFL, O_NONBLOCK);

  ud->init = 1;

  luaL_getmetatable(L, MT_NAME);
  lua_setmetatable(L, -2);
  return 1;
}

static int lua_uvc_stream_on(lua_State *L) {
  camera_device *ud = lua_checkuvc(L, 1);

  // Update settings...
  // ud->width = luaL_optint(L, 2, 320);
  // ud->height = luaL_optint(L, 3, 240);
  // ud->pixelformat = luaL_optstring(L, 4, "yuyv");
  // /* default 15 fps */
  // ud->fps_num = luaL_optint(L, 5, 1);
  // ud->fps_denum = luaL_optint(L, 6, 15);

  uvc_error_t res;
  enum uvc_frame_format fmt;
  if (strcmp(ud->pixelformat, "yuyv") == 0) {
    fmt = UVC_FRAME_FORMAT_YUYV;
  } else if (strcmp(ud->pixelformat, "mjpeg") == 0) {
    fmt = UVC_FRAME_FORMAT_MJPEG;
  } else if (strcmp(ud->pixelformat, "rgb") == 0) {
    fmt = UVC_FRAME_FORMAT_RGB;
  } else {
    fmt = UVC_FRAME_FORMAT_UNCOMPRESSED;
  }

#ifdef DEBUG
  fprintf(stderr, "Get stream...\n");
#endif
  uvc_stream_ctrl_t ctrl;
  res = uvc_get_stream_ctrl_format_size(
      ud->devh, &ctrl, fmt, ud->width, ud->height, ud->fps_denum / ud->fps_num);

  if (res != UVC_SUCCESS) {
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "uvc_get_stream_ctrl_format_size: device doesn't "
                       "provide a matching stream");
  }

#ifdef DEBUG
  fprintf(stderr, "Print stream...\n");
  uvc_print_stream_ctrl(&ctrl, stderr);
#endif
#ifdef DEBUG
  fprintf(stderr, "Start stream...\n");
#endif
  res = uvc_start_streaming(ud->devh, &ctrl, cb, (void *)ud, 0);
  if (res != UVC_SUCCESS) {
    uvc_perror(res, "uvc_start_streaming");
#ifdef DEBUG
    fprintf(stderr, "Could not stream...\n");
#endif
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "uvc_start_streaming: unable to start stream");
  }

  if (ud->usb_ctx) {
    libusb_handle_events_completed(ud->usb_ctx, NULL);
  }
  lua_pushboolean(L, 1);
  return 1;
}

static int lua_uvc_stream_off(lua_State *L) {
  camera_device *ud = lua_checkuvc(L, 1);
  uvc_stop_streaming(ud->devh);
  lua_pushboolean(L, 1);
  return 1;
}

static int lua_uvc_fd(lua_State *L) {
  camera_device *ud = lua_checkuvc(L, 1);
  // Give the reading end
  lua_pushinteger(L, ud->pipefd[0]);
  if (!ud->usb_ctx) {
    return 1;
  }
  // Add the other file descriptors, if owning the USB context
  ud->fds = libusb_get_pollfds(ud->usb_ctx);
  if (!ud->fds) {
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "No file descriptors");
    return 2;
  }
  lua_newtable(L);
  int i_pfd = 0;
  int n_pfd = 0;
  while (ud->fds[i_pfd]) {
#ifdef DEBUG
    fprintf(stderr, "[%d] pfd[%d]: %d %d [POLLIN=%d]\n", n_pfd, i_pfd,
            ud->fds[i_pfd]->fd, ud->fds[i_pfd]->events, POLLIN);
#endif
    if (ud->fds[i_pfd]->events & POLLIN) {
      n_pfd++;
      lua_pushinteger(L, ud->fds[i_pfd]->fd);
      lua_rawseti(L, -2, n_pfd);
    }
    i_pfd++;
  }
  // Set the number of file descriptors available
  ud->nfds = n_pfd;
  return 2;
}

static int lua_uvc_get_width(lua_State *L) {
  camera_device *ud = lua_checkuvc(L, 1);
  if (ud->init) {
    lua_pushinteger(L, ud->width);
  } else {
    return 0;
  }
  return 1;
}

static int lua_uvc_get_height(lua_State *L) {
  camera_device *ud = lua_checkuvc(L, 1);
  if (ud->init) {
    lua_pushinteger(L, ud->height);
  } else {
    return 0;
  }
  return 1;
}

static int lua_uvc_set_param(lua_State *L) {
  camera_device *ud = lua_checkuvc(L, 1);
  const char *param = luaL_checkstring(L, 2);
  int value = luaL_checkinteger(L, 3);

#ifdef DEBUG
  fprintf(stderr, "Set [%s] to [%d] | devh: [%p]\n", param, value, ud->devh);
#endif
  uvc_error_t ret;
  if (strcmp(param, "ae_mode") == 0) {
    /*
    UVC_AUTO_EXPOSURE_MODE_MANUAL (1) - manual exposure time, manual iris
    UVC_AUTO_EXPOSURE_MODE_AUTO (2) - auto exposure time, auto iris
    UVC_AUTO_EXPOSURE_MODE_SHUTTER_PRIORITY (4) - manual exposure time, auto
    iris UVC_AUTO_EXPOSURE_MODE_APERTURE_PRIORITY (8) - auto exposure time,
    manual iris
    */
    ret = uvc_set_ae_mode(ud->devh, value);
    lua_pushboolean(L, 1);
    return 1;
  } else if (strcmp(param, "ae_priority") == 0) {
    ret = uvc_set_ae_priority(ud->devh, value);
    lua_pushboolean(L, 1);
    return 1;
  } else if (strcmp(param, "exposure_abs") == 0) {
    // In milliseconds
    ret = uvc_set_exposure_abs(ud->devh, value * 10);
    lua_pushboolean(L, 1);
    return 1;
  } else if (strcmp(param, "iris_abs") == 0) {
    ret = uvc_set_iris_abs(ud->devh, value);
    lua_pushboolean(L, 1);
    return 1;
  } else if (strcmp(param, "white_balance_temperature_auto") == 0) {
    ret = uvc_set_white_balance_temperature_auto(ud->devh, value);
    lua_pushboolean(L, 1);
    return 1;
  } else if (strcmp(param, "white_balance_temperature") == 0) {
    ret = uvc_set_white_balance_temperature(ud->devh, value);
    lua_pushboolean(L, 1);
    return 1;
  } else {
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "Unknown control");
    return 2;
  }
}

static int lua_uvc_get_param(lua_State *L) {
  camera_device *ud = lua_checkuvc(L, 1);
  const char *param = luaL_checkstring(L, 2);
  const char *req_type = luaL_optstring(L, 3, "cur");

  enum uvc_req_code uvc_req_code;
  if (!req_type || strcmp(req_type, "cur") == 0) {
    uvc_req_code = UVC_GET_CUR;
  } else if (strcmp(req_type, "max") == 0) {
    uvc_req_code = UVC_GET_MAX;
  } else {
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "Unknown request");
    return 2;
  }
#ifdef DEBUG
  fprintf(stderr, "Get [%s] devh: [%p]\n", param, ud->devh);
#endif
  uvc_error_t ret;
  if (strcmp(param, "ae_mode") == 0) {
    /*
    UVC_AUTO_EXPOSURE_MODE_MANUAL (1) - manual exposure time, manual iris
    UVC_AUTO_EXPOSURE_MODE_AUTO (2) - auto exposure time, auto iris
    UVC_AUTO_EXPOSURE_MODE_SHUTTER_PRIORITY (4) - manual exposure time, auto
    iris UVC_AUTO_EXPOSURE_MODE_APERTURE_PRIORITY (8) - auto exposure time,
    manual iris
    */
    uint8_t mode;
    ret = uvc_get_ae_mode(ud->devh, &mode, uvc_req_code);
    lua_pushinteger(L, mode);
    return 1;
  } else if (strcmp(param, "ae_priority") == 0) {
    uint8_t priority;
    ret = uvc_get_ae_priority(ud->devh, &priority, uvc_req_code);
    lua_pushinteger(L, priority);
    return 1;
  } else if (strcmp(param, "exposure_abs") == 0) {
    uint32_t exposure;
    ret = uvc_get_exposure_abs(ud->devh, &exposure, uvc_req_code);
    // In milliseconds
    lua_pushnumber(L, exposure / 10.0);
    return 1;
  } else if (strcmp(param, "iris_abs") == 0) {
    uint16_t iris;
    ret = uvc_get_iris_abs(ud->devh, &iris, uvc_req_code);
    lua_pushinteger(L, iris);
    return 1;
  } else if (strcmp(param, "white_balance_temperature_auto") == 0) {
    uint8_t wb_auto;
    ret = uvc_get_white_balance_temperature_auto(ud->devh, &wb_auto,
                                                 uvc_req_code);
    lua_pushinteger(L, wb_auto);
    return 1;
  } else if (strcmp(param, "white_balance_temperature") == 0) {
    uint16_t wb;
    ret = uvc_get_white_balance_temperature(ud->devh, &wb, uvc_req_code);
    lua_pushinteger(L, wb);
    return 1;
  } else {
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "Unknown control");
    return 2;
  }
}

static int lua_uvc_timeout_ms(lua_State *L) {
  camera_device *ud = lua_checkuvc(L, 1);
  struct timeval tv_timeout;
  if (!ud->usb_ctx) {
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "Do not own the context");
    return 2;
  }
  int has_timeout = libusb_get_next_timeout(ud->usb_ctx, &tv_timeout);
  if (has_timeout == 0) {
#ifdef DEBUG
    fprintf(stderr, "No timeouts\n");
#endif
    lua_pushnumber(L, 0);
  } else if (has_timeout == 1) {
#ifdef DEBUG
    fprintf(stderr, "tv_timeout %ld %ld\n", tv_timeout.tv_sec,
            tv_timeout.tv_usec);
#endif
    lua_pushnumber(L, tv_timeout.tv_sec * 1e3 + tv_timeout.tv_usec / 1.0e3);
  }
  return 1;
}

static int lua_uvc_get_image(lua_State *L) {
  camera_device *ud = lua_checkuvc(L, 1);
  // Default is to poll until a frame occurs
  int timeout_ms = luaL_optint(L, 2, -1);
  if (timeout_ms != 0) {
#ifdef DEBUG
    fprintf(stderr, "Poll...\n");
#endif
    struct pollfd pfd;
    pfd.fd = ud->pipefd[0];
    pfd.events = POLLIN;
    int ret = poll(&pfd, 1, timeout_ms);
#ifdef DEBUG
    fprintf(stderr, "pfd.events %d\n", pfd.events);
    fprintf(stderr, "pfd.revents %d\n", pfd.revents);
#endif
    if (ret < 0) {
      lua_pushboolean(L, 0);
      lua_pushliteral(L, "Bad poll");
      return 2;
    } else if (ret == 0) {
      lua_pushboolean(L, 0);
      lua_pushliteral(L, "No FDs");
      return 2;
    }
  }
  unsigned int count;
  // count and ud->count must be the same type :)
  ssize_t ret = read(ud->pipefd[0], &count, sizeof(count));

  size_t buf_sz = ud->buf_sz;
  if (buf_sz == 0 || !ud->buffer) {
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "No data");
    return 2;
  }
  // Can give the string or the raw buffer
  int use_string = lua_toboolean(L, 3);
  if (use_string) {
    lua_pushlstring(L, ud->buffer, buf_sz);
  } else {
    lua_pushlightuserdata(L, ud->buffer);
  }
  // Check that the sequence count matches...
  unsigned int buffer_count = ud->count;
  if (count != buffer_count) {
    // Remove the data from the stack
    lua_pop(L, 1);
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "Corrupted data");
    return 2;
  }
  // Add information
  lua_pushinteger(L, buf_sz);
  lua_pushinteger(L, buffer_count);
  return 3;
}

static const struct luaL_Reg uvc_functions[] = {{"init", lua_uvc_init},
                                                {NULL, NULL}};

static const struct luaL_Reg uvc_methods[] = {
    {"descriptor", lua_uvc_fd},
    {"timeout_ms", lua_uvc_timeout_ms},
    {"close", lua_uvc_delete},
    {"stream_on", lua_uvc_stream_on},
    {"stream_off", lua_uvc_stream_off},
    {"get_width", lua_uvc_get_width},
    {"get_height", lua_uvc_get_height},
    {"set_param", lua_uvc_set_param},
    {"get_param", lua_uvc_get_param},
    {"get_image", lua_uvc_get_image},
    {"__index", lua_uvc_index},
    {"__gc", lua_uvc_delete},
    {NULL, NULL}};

#ifdef __cplusplus
extern "C"
#endif
    int
    luaopen_uvc(lua_State *L) {
  /* create metatable for uvc module */
  luaL_newmetatable(L, MT_NAME);
#if LUA_VERSION_NUM == 501
  luaL_register(L, NULL, uvc_methods);
#else
  luaL_setfuncs(L, uvc_methods, 0);
#endif
  lua_pop(L, 1);

#if LUA_VERSION_NUM == 501
  luaL_register(L, "uvc", uvc_functions);
#else
  luaL_newlib(L, uvc_functions);
#endif

  return 1;
}

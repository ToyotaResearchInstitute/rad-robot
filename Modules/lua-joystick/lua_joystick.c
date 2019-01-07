/**
 * Lua module for usb joystick interface
 */
#include <lua.h>
#include <lualib.h>
#include <lauxlib.h>

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <libusb.h>
#include <linux/joystick.h>

#include <sys/time.h>
#include <sys/types.h>

// thread variables
static pthread_t jsThread;
void *joystick_thread_func(void *ctx);
int joystick_thread_cleanup(void);

// initialized flag
int init = 0;
volatile int running = 0;
volatile int stopRequest = 0;
int jsFD;

// number of buttons and axes
char nbutton;
char naxis;
// arrays for button and axis data
int16_t *buttons;
int16_t *axes;
// time of last event (in milliseconds)
uint32_t *tbutton;
uint32_t *taxis;

void process_event(struct js_event* event) {

  if ((event->type & ~JS_EVENT_INIT) == JS_EVENT_BUTTON) {
    buttons[event->number] = event->value;
    tbutton[event->number] = event->time;
  }
  if ((event->type & ~JS_EVENT_INIT) == JS_EVENT_AXIS) {
    axes[event->number] = event->value;
    taxis[event->number] = event->time;
  }
}

void *joystick_thread_func(void *ctx) {
  fprintf(stderr, "starting joystick thread\n");

  /*
  sigset_t sigs;
  sigfillset(&sigs);
  pthread_sigmask(SIG_BLOCK, &sigs, NULL);
  */

  while (!stopRequest) {
    struct js_event event;
    int nrd = read(jsFD, &event, sizeof(struct js_event));
    if (nrd != sizeof(struct js_event)) {
      fprintf(stderr, "read %d bytes but expected %lu bytes\n",
        nrd, sizeof(struct js_event));
      break;
    }
    process_event(&event);
    // sleep for 1ms
#ifdef USE_POLL

#else
    usleep(1000);
#endif
  }

  joystick_thread_cleanup();
  running = 0;
}

int joystick_open(const char* dev) {
  int ret;

  // open joystick device
  fprintf(stderr, "opening joystick: %s...", dev);
  jsFD = open(dev, O_RDONLY);
  fprintf(stderr, "done! fd: %d\n", jsFD);
  if (jsFD < 0) {
    return jsFD;
  }

  // query the number of buttons and axes
  char name[256];
  ret = ioctl(jsFD, JSIOCGNAME(256), name);
  if (ret < 0) {
    fprintf(stderr, "error querying device identifier string: %d\n", ret);
    return ret;
  }
  ret = ioctl(jsFD, JSIOCGBUTTONS, &nbutton);
  if (ret < 0) {
    fprintf(stderr, "error querying number of buttons: %d\n", ret);
    return -1;
  }
  ret = ioctl(jsFD, JSIOCGAXES, &naxis);
  if (ret < 0) {
    fprintf(stderr, "error querying number of axes: %d\n", ret);
  }

  fprintf(stderr, "opened %s with %d buttons and %d axes\n",
    name, nbutton, naxis);

  // allocate array data
  buttons = calloc(nbutton, sizeof(int16_t));
  if (!buttons) {
    fprintf(stderr, "unable to allocate button data array\n");
    return -1;
  }
  axes = calloc(naxis, sizeof(int16_t));
  if (!axes) {
    fprintf(stderr, "unable to allocate axes data array\n");
    return -1;
  }
  tbutton = calloc(nbutton, sizeof(uint32_t));
  if (!tbutton) {
    fprintf(stderr, "unable to allocate button time array\n");
    return -1;
  }
  taxis = calloc(naxis, sizeof(uint32_t));
  if (!taxis) {
    fprintf(stderr, "unable to allocate axes time array\n");
    return -1;
  }
  return ret;
}

int joystick_thread_init() {
  // start receiver thread
  fprintf(stderr, "creating joystick thread\n");
  running = 1;
  int ret = pthread_create(&jsThread, NULL, joystick_thread_func, NULL);
  if (ret != 0) {
    fprintf(stderr, "error creating joystick thread: %d\n", ret);
    return -1;
  }

  init = 1;

  return 0;
}

int joystick_thread_cleanup() {
  fprintf(stderr, "cleaning up joystick thread\n");
  if (init) {
    // set initialized to false
    init = 0;
    // close the fd
    close(jsFD);
    // free arrays
    nbutton = 0;
    naxis = 0;
    free(buttons);
    free(axes);
    free(tbutton);
    free(taxis);
  }
  return 0;
}

static int lua_joystick_open(lua_State *L) {
  const char *dev = luaL_optstring(L, 1, "/dev/input/js0");
  int ret = joystick_open(dev);
  if (ret < 0) {
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "Could not open device");
    return 1;
  }

  // True gives a thread
  if (!lua_toboolean(L, 2)) {
    lua_pushinteger(L, jsFD);
    return 1;
  }

  // Try three times to open
  int i;
  for (i=0; i<3; i++){
    if (init) {
      // a joystick is already open, close it first
      joystick_thread_cleanup();
    }
    if (joystick_thread_init() == 0) {
      // lua_pushboolean(L, 1);
      lua_pushinteger(L, jsFD);
      return 1;
    }
    usleep(2000);
  }

  // Not detected
  lua_pushboolean(L, 0);
  return 1;
}

static int lua_joystick_close(lua_State *L) {
  // stop thread
  stopRequest = 1;

  // wait for it to actually stop
  while (running) {
    usleep(1000);
  }
  stopRequest = 0;

  return 0;
}

static int lua_joystick_num_buttons(lua_State *L) {
  lua_pushinteger(L, nbutton);

  return 1;
}

static int lua_joystick_num_axes(lua_State *L) {
  lua_pushinteger(L, naxis);

  return 1;
}

static int lua_joystick_button(lua_State *L) {
  int ind = luaL_optinteger(L, 1, -1);
  if (ind <= 0 || ind > nbutton) {
    // return array will all button states
    lua_createtable(L, nbutton, 0);
    int i;
    for (i = 0; i < (int)nbutton; i++) {
      lua_pushinteger(L, buttons[i]);
      lua_rawseti(L, -2, i + 1);
    }
  } else {
    // return only indexed button state
    lua_pushinteger(L, buttons[ind - 1]);
  }
  return 1;
}

static int lua_joystick_axis(lua_State *L) {
  int ind = luaL_optinteger(L, 1, -1);
  if (ind <= 0 || ind > naxis) {
    // return array will all axes states
    int i;
    lua_createtable(L, naxis, 0);
    for (i = 0; i < (int)naxis; i++) {
      lua_pushinteger(L, axes[i]);
      lua_rawseti(L, -2, i + 1);
    }
  } else {
    // return only indexed axes state
    lua_pushinteger(L, axes[ind - 1]);
  }
  return 1;
}

static int lua_joystick_button_time(lua_State *L) {
  int ind = luaL_optinteger(L, 1, -1);
  if (ind <= 0 || ind > nbutton) {
    // return array will all button states
    lua_createtable(L, nbutton, 0);
    int i;
    for (i = 0; i < (int)nbutton; i++) {
      lua_pushinteger(L, tbutton[i]);
      lua_rawseti(L, -2, i + 1);
    }
  } else {
    // return only indexed button state
    lua_pushinteger(L, tbutton[ind - 1]);
  }
  return 1;
}

static int lua_joystick_axis_time(lua_State *L) {
  int ind = luaL_optinteger(L, 1, -1);
  if (ind <= 0 || ind > naxis) {
    // return array will all axes states
    lua_createtable(L, naxis, 0);
    int i;
    for (i = 0; i < (int)naxis; i++) {
      lua_pushinteger(L, taxis[i]);
      lua_rawseti(L, -2, i + 1);
    }
  } else {
    // return only indexed axes state
    lua_pushinteger(L, taxis[ind - 1]);
  }
  return 1;
}

static int lua_joystick_axis2(lua_State *L) {
  if (!running) {
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "Thread not running");
    return 2;
  }
  int ind = luaL_optinteger(L, 1, -1);
  if (ind <= 0 || ind > naxis) {
    // return array will all axes states
    int i;
    lua_createtable(L, naxis, 0);
    for (i = 0; i < (int)naxis; i++) {
      lua_pushinteger(L, axes[i]);
      lua_rawseti(L, -2, i + 1);
    }
    lua_createtable(L, naxis, 0);
    for (i = 0; i < (int)naxis; i++) {
      lua_pushinteger(L, taxis[i]);
      lua_rawseti(L, -2, i + 1);
    }
  } else {
    // return only indexed axes state
    lua_pushinteger(L, axes[ind - 1]);
    lua_pushinteger(L, taxis[ind - 1]);
  }
  return 2;
}

static int lua_joystick_button2(lua_State *L) {
  if (!running) {
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "Thread not running");
    return 2;
  }
  int ind = luaL_optinteger(L, 1, -1);
  if (ind <= 0 || ind > nbutton) {
    lua_createtable(L, nbutton, 0);
    int i;
    for (i = 0; i < (int)nbutton; i++) {
      lua_pushinteger(L, buttons[i]);
      lua_rawseti(L, -2, i + 1);
    }
    lua_createtable(L, nbutton, 0);
    for (i = 0; i < (int)nbutton; i++) {
      lua_pushinteger(L, tbutton[i]);
      lua_rawseti(L, -2, i + 1);
    }
  } else {
    lua_pushinteger(L, buttons[ind - 1]);
    lua_pushinteger(L, tbutton[ind - 1]);
  }
  return 2;
}

// Data to four tables
static int lua_joystick_parse(lua_State *L) {
  size_t nrd;
  struct js_event* data = (struct js_event*) lua_tolstring(L, 1, &nrd);
  if (data == NULL) {
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "No data!");
    return 2;
  } else if (nrd != sizeof(struct js_event)) {
    fprintf(stderr, "read %zu bytes but expected %lu bytes\n",
      nrd, sizeof(struct js_event));
    lua_pushboolean(L, 0);
    // lua_pushliteral(L, "qMid");
    return 1;
  }
  process_event(data);
  int i;
  lua_createtable(L, naxis, 0);
  for (i = 0; i < (int)naxis; i++) {
    lua_pushinteger(L, axes[i]);
    lua_rawseti(L, -2, i + 1);
  }
  lua_createtable(L, nbutton, 0);
  for (i = 0; i < (int)nbutton; i++) {
    lua_pushinteger(L, buttons[i]);
    lua_rawseti(L, -2, i + 1);
  }
  lua_createtable(L, naxis, 0);
  for (i = 0; i < (int)naxis; i++) {
    lua_pushinteger(L, taxis[i]);
    lua_rawseti(L, -2, i + 1);
  }
  lua_createtable(L, nbutton, 0);
  for (i = 0; i < (int)nbutton; i++) {
    lua_pushinteger(L, tbutton[i]);
    lua_rawseti(L, -2, i + 1);
  }
  return 4;
}

static const struct luaL_Reg joystick_lib[] = {
  {"open", lua_joystick_open},
  {"close", lua_joystick_close},
  {"button", lua_joystick_button},
  {"axis", lua_joystick_axis},
  {"button_time", lua_joystick_button_time},
  {"axis_time", lua_joystick_axis_time},
  {"num_buttons", lua_joystick_num_buttons},
  {"num_axes", lua_joystick_num_axes},
  {"button2", lua_joystick_button2},
  {"axis2", lua_joystick_axis2},
  {"parse", lua_joystick_parse},
  {NULL, NULL}
};

#ifdef __cplusplus
extern "C"
#endif
  int luaopen_joystick(lua_State *L) {
#if LUA_VERSION_NUM > 501
  luaL_newlib(L, joystick_lib);
#else
  luaL_register(L, "joystick", joystick_lib);
#endif
  return 0;
}

/*
 * lsignal.c -- Signal Handler Library for Lua
 *
 * Copyright (C) 2010  Patrick J. Donnelly (batrick@batbytes.com)
 *
 * This software is distributed under the same license as Lua 5.0:
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE. 
*/

#define LUA_LIB_NAME      "signal"
#define LUA_LIB_VERSION   "1.2.0"
#define LUA_SIGNAL_NAME   "LUA_SIGNAL"

#if !(defined(_POSIX_SOURCE) || defined(sun) || defined(__sun))
  #define INCLUDE_KILL  1
  #define INCLUDE_PAUSE 1
  #define USE_SIGACTION 1
#endif

#include <lua.h>
#include <lauxlib.h>

#include <assert.h>
#include <errno.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>

typedef const struct const_info {
  const char *name; /* name of the signal */
  const int value; /* the signal */
} const_info;

void lua_install_constants(lua_State *L, const_info constants[]) {
  int i;
  for (i = 0; constants[i].name; i++) {
    lua_pushstring(L, constants[i].name);
    lua_pushinteger(L, constants[i].value);
    lua_rawset(L, -3);
  }
}

static const struct const_info lua_signals[] = {
  /* ANSI C signals */
#ifdef SIGABRT
  {"SIGABRT", SIGABRT},
#endif
#ifdef SIGFPE
  {"SIGFPE", SIGFPE},
#endif
#ifdef SIGILL
  {"SIGILL", SIGILL},
#endif
#ifdef SIGINT
  {"SIGINT", SIGINT},
#endif
#ifdef SIGSEGV
  {"SIGSEGV", SIGSEGV},
#endif
#ifdef SIGTERM
  {"SIGTERM", SIGTERM},
#endif
  /* posix signals */
#ifdef SIGHUP
  {"SIGHUP", SIGHUP},
#endif
#ifdef SIGQUIT
  {"SIGQUIT", SIGQUIT},
#endif
#ifdef SIGTRAP
  {"SIGTRAP", SIGTRAP},
#endif
#ifdef SIGKILL
  {"SIGKILL", SIGKILL},
#endif
#ifdef SIGUSR1
  {"SIGUSR1", SIGUSR1},
#endif
#ifdef SIGUSR2
  {"SIGUSR2", SIGUSR2},
#endif
#ifdef SIGPIPE
  {"SIGPIPE", SIGPIPE},
#endif
#ifdef SIGALRM
  {"SIGALRM", SIGALRM},
#endif
#ifdef SIGCHLD
  {"SIGCHLD", SIGCHLD},
#endif
#ifdef SIGCONT
  {"SIGCONT", SIGCONT},
#endif
#ifdef SIGSTOP
  {"SIGSTOP", SIGSTOP},
#endif
#ifdef SIGTSTP
  {"SIGTSTP", SIGTSTP},
#endif
#ifdef SIGTTIN
  {"SIGTTIN", SIGTTIN},
#endif
#ifdef SIGTTOU
  {"SIGTTOU", SIGTTOU},
#endif
  /* some BSD signals */
#ifdef SIGIOT
  {"SIGIOT", SIGIOT},
#endif
#ifdef SIGBUS
  {"SIGBUS", SIGBUS},
#endif
#ifdef SIGCLD
  {"SIGCLD", SIGCLD},
#endif
#ifdef SIGURG
  {"SIGURG", SIGURG},
#endif
#ifdef SIGXCPU
  {"SIGXCPU", SIGXCPU},
#endif
#ifdef SIGXFSZ
  {"SIGXFSZ", SIGXFSZ},
#endif
#ifdef SIGVTALRM
  {"SIGVTALRM", SIGVTALRM},
#endif
#ifdef SIGPROF
  {"SIGPROF", SIGPROF},
#endif
#ifdef SIGWINCH
  {"SIGWINCH", SIGWINCH},
#endif
#ifdef SIGPOLL
  {"SIGPOLL", SIGPOLL},
#endif
#ifdef SIGIO
  {"SIGIO", SIGIO},
#endif
  /* add odd signals */
#ifdef SIGSTKFLT
  {"SIGSTKFLT", SIGSTKFLT}, /* stack fault */
#endif
#ifdef SIGSYS
  {"SIGSYS", SIGSYS},
#endif
  {NULL, 0}
};

/*
 *  The signal counts in the 1st half of the array are modified by
 *  the handler.  The corresponding signal counts in the 2nd half
 *  are modifed by the hook routine.
 */
static volatile sig_atomic_t *signal_stack = NULL;
static int signal_stack_top;
static lua_State *ML = NULL;
static struct hook {
  lua_Hook hook;
  int mask;
  int count;
} old_hook = {NULL, 0, 0};

static void hook (lua_State *L, lua_Debug *ar)
{
  fprintf(stderr, "hook\n");
  int i, j;
  // TODO: I think we can omit this assert? Checks if in another Lua state
  // assert(L == ML);
  for (i = 0; i < signal_stack_top; i++)
    while (signal_stack[i] != signal_stack[i+signal_stack_top])
    {
      // Grab the dictionary of the signal
      lua_getfield(L, LUA_REGISTRYINDEX, LUA_SIGNAL_NAME);
      lua_pushinteger(L, i);
      lua_rawget(L, -2);
      // Unsure why this replace...?
      lua_replace(L, -2); /* replace _R.LUA_SIGNAL_NAME */
      // TODO: Remove all asserts and replace with something else
      assert(lua_isfunction(L, -1));
      for (j = 0; lua_signals[j].name != NULL; j++)
        if (lua_signals[j].value == i)
        {
          lua_pushstring(L, lua_signals[j].name);
          break;
        }
      if (lua_signals[j].name == NULL) lua_pushliteral(L, "");
      lua_pushinteger(L, i);
      lua_call(L, 2, 0);
      signal_stack[i+signal_stack_top]++;
    }
  lua_sethook(ML, old_hook.hook, old_hook.mask, old_hook.count);
  old_hook.hook = NULL;
}

static void handle (int sig)
{
  fprintf(stderr, "handle\n");
  assert(ML != NULL);
  if (old_hook.hook == NULL) /* replace it */
  {
    old_hook.hook = lua_gethook(ML);
    old_hook.mask = lua_gethookmask(ML);
    old_hook.count = lua_gethookcount(ML);
    lua_sethook(ML, hook, LUA_MASKCOUNT, 1);
  }
  signal_stack[sig]++;
}

static int get_signal (lua_State *L, int idx)
{
  fprintf(stderr, "get_signal\n");
  switch (lua_type(L, idx))
  {
    case LUA_TNUMBER:
      fprintf(stderr, "signal number\n");
      return lua_tointeger(L, idx);
    case LUA_TSTRING:
      fprintf(stderr, "signal string\n");
      lua_pushvalue(L, idx);
      lua_rawget(L, LUA_REGISTRYINDEX);
      if (!lua_isnumber(L, -1))
        return luaL_argerror(L, idx, "invalid signal string");
      lua_replace(L, idx);
      return (int) lua_tointeger(L, idx);
    default:
      return luaL_argerror(L, idx, "expected signal string/number");
  }
}

static int status (lua_State *L, int s)
{
  fprintf(stderr, "status\n");
  if (s)
  {
    lua_pushboolean(L, 1);
    return 1;
  }
  else
  {
    lua_pushnil(L);
    lua_pushstring(L, strerror(errno));
    return 2;
  }
}

static void stackDump (lua_State *L) {
  fprintf(stderr, "\n!! STACK DUMP !!\n");
  int i;
  int top = lua_gettop(L);
  for (i = 1; i <= top; i++) {  /* repeat for each level */
    fprintf(stderr, "%d: ", i);
    int t = lua_type(L, i);
    switch (t) {

      case LUA_TSTRING:  /* strings */
        fprintf(stderr, "`%s'", lua_tostring(L, i));
        break;

      case LUA_TBOOLEAN:  /* booleans */
        fprintf(stderr, lua_toboolean(L, i) ? "true" : "false");
        break;

      case LUA_TNUMBER:  /* numbers */
        fprintf(stderr, "%g", lua_tonumber(L, i));
        break;

      default:  /* other values */
        fprintf(stderr, "%s", lua_typename(L, t));
        break;

    }
    fprintf(stderr, "\n");  /* put a separator */
  }
  fprintf(stderr, "!! STACK DUMP END!!\n");
}

/*
 * old_handler[, err] == signal(signal [, func])
 *
 * signal = signal number or string
 * func/"ignore"/"default" = Lua function to call
*/  
static int l_signal (lua_State *L)
{
  fprintf(stderr, "l_signal start\n");
  enum {IGNORE, DEFAULT, SET};
  static const char *options[] = {"ignore", "default", NULL};
  int sig = get_signal(L, 1);
  int option;

  fprintf(stderr, "l_signal get option\n");
  if (lua_isstring(L, 2)) {
    option = luaL_checkoption(L, 2, NULL, options);
  } else if (lua_isnil(L, 2)) {
    option = DEFAULT;
  } else {
    luaL_checktype(L, 2, LUA_TFUNCTION);
    option = SET;
  }
  stackDump(L);

  fprintf(stderr, "l_signal option %d\n", option);

  // Push the LUA_SIGNAL_NAME from the registry
  lua_getfield(L, LUA_REGISTRYINDEX, LUA_SIGNAL_NAME);
  lua_pushvalue(L, 1);

  fprintf(stderr, "l_signal get old handler\n");
  stackDump(L);
  lua_rawget(L, -2); /* return old handler */
  // lua_rawget(L, LUA_REGISTRYINDEX); /* return old handler */
  
  // TODO: Check if old and new are equal
  // Remove the old handler...
  lua_pop(L, 1);

  fprintf(stderr, "l_signal prepare to set handler\n");
  stackDump(L);

  lua_pushvalue(L, 1);

  stackDump(L);

  switch (option)
  {
    case IGNORE:
      lua_pushnil(L);
      // lua_rawset(L, LUA_REGISTRYINDEX);
      lua_rawset(L, -3);
      signal(sig, SIG_IGN);
      signal_stack[sig+signal_stack_top] = signal_stack[sig] = 0;
      break;
    case DEFAULT:
      lua_pushnil(L);
      // lua_rawset(L, LUA_REGISTRYINDEX);
      lua_rawset(L, -3);
      signal(sig, SIG_DFL);
      signal_stack[sig+signal_stack_top] = signal_stack[sig] = 0;
      break;
    case SET:
      fprintf(stderr, "l_signal set\n");
      // NOTE: This value is a function
      lua_pushvalue(L, 2);
      // lua_rawset(L, LUA_REGISTRYINDEX);
      lua_rawset(L, -3);

#if USE_SIGACTION
      {
        struct sigaction act;
        act.sa_handler = handle;
        sigemptyset(&act.sa_mask);
        act.sa_flags = 0;
        if (sigaction(sig, &act, NULL))
          return status(L, 0);
      }
#else
      if (signal(sig, handle) == SIG_ERR) {
        return status(L, 0);
      }
#endif
      break;
    default: assert(0);
  }
  fprintf(stderr, "l_signal done\n");

  // Should return the old handler?
  return 1;
}

/*
 * status, err = raise(signal)
 *
 * signal = signal number or string
*/  
static int l_raise (lua_State *L)
{
  return status(L, raise(get_signal(L, 1)) == 0);
}

#if INCLUDE_KILL

/* define some posix only functions */

/*
 * status, err = kill(pid, signal)
 *
 * pid = process id
 * signal = signal number or string
*/  
static int l_kill (lua_State *L)
{
  return status(L, kill(luaL_checkinteger(L, 1), get_signal(L, 2)) == 0);
}

#endif

#if INCLUDE_PAUSE

static int l_pause (lua_State *L) /* race condition free */
{
  sigset_t mask, old_mask;
  if (sigfillset(&mask) == -1) return status(L, 0);
  if (sigprocmask(SIG_BLOCK, &mask, &old_mask) == -1) return status(L, 0);
  if (sigsuspend(&old_mask) != -1) abort(); /* that's strange */
  return status(L, 0);
}

#endif

// static int interrupted (lua_State *L)
// {
//   return luaL_error(L, "interrupted!");
// }

static int library_gc (lua_State *L)
{
  fprintf(stderr, "library_gc\n");
  lua_getfield(L, LUA_REGISTRYINDEX, LUA_SIGNAL_NAME);
  lua_pushnil(L);
  while (lua_next(L, -2))
  {
    if (lua_isnumber(L, -2)) /* <signal, function> */
      signal((int) lua_tointeger(L, -2), SIG_DFL);
    lua_pop(L, 1); /* value */
  }
  signal_stack = NULL;
  ML = NULL;
  old_hook.hook = NULL;
  signal_stack_top = 0;
  return 0;
}

static const struct luaL_Reg signal_lib[] = {
    {"signal", l_signal},
    {"raise", l_raise},
#if INCLUDE_KILL
    {"kill", l_kill},
#endif
#if INCLUDE_PAUSE
    {"pause", l_pause},
#endif
    {NULL, NULL}
  };

int luaopen_signal (lua_State *L)
{

  // int i;
  // int max_signal;

  ML = L;
  if (lua_pushthread(L))
    lua_pop(L, 1);
  else
    luaL_error(L, "library should be opened by the main thread");

  /* add the library */
#if LUA_VERSION_NUM == 501
  luaL_register(L, LUA_LIB_NAME, signal_lib);
#else
  luaL_newlib(L, signal_lib);
#endif

  lua_pushliteral(L, LUA_LIB_VERSION);
  lua_setfield(L, -2, "version");

  // Add the constants for adding signals
  lua_install_constants(L, lua_signals);

  // Setup the signal_stack, and add to the library table
  size_t n_signals = sizeof(lua_signals)/sizeof(lua_signals[0]) - 1;
  signal_stack_top = n_signals;
  signal_stack = lua_newuserdata(L, sizeof(volatile sig_atomic_t)*n_signals*2);
  bzero((void*)signal_stack, sizeof(volatile sig_atomic_t)*n_signals*2);
  lua_setfield(L, -2, "signal_stack");

  // Add a signal library metatable
  lua_newtable(L);
  lua_pushcfunction(L, library_gc);
  lua_setfield(L, -2, "__gc");
  lua_setmetatable(L, -2); /* when userdata is gc'd, close library */

  /* environment */
  lua_newtable(L);
  // // Place the table as the LUA_ENVIRONINDEX
  // lua_replace(L, LUA_ENVIRONINDEX);
  // lua_pushvalue(L, LUA_ENVIRONINDEX);
  // // Set the registry table to reg_tbl[LUA_SIGNAL_NAME] = LUA_ENVIRONINDEX
  lua_setfield(L, LUA_REGISTRYINDEX, LUA_SIGNAL_NAME); /* for hooks */
  
  // lua_pushboolean(L, 1);
  // lua_rawset(L, LUA_ENVIRONINDEX);

  /* set default interrupt handler */
  // Get the signal function from the newlib signal library
  // lua_getfield(L, -1, "signal");
  // lua_pushinteger(L, SIGINT);
  // lua_pushcfunction(L, interrupted);
  // lua_call(L, 2, 0);

  return 1;
}

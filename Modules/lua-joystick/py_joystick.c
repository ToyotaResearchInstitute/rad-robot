#include <Python.h>

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <libusb.h>

#include <sys/time.h>
#include <sys/types.h>

#include <linux/joystick.h>

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

// thread variables
static pthread_t jsThread;

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
      fprintf(stderr,
        "read %d bytes but expected %lu bytes\n",
        nrd, sizeof(struct js_event));
      break;
    }

    if ((event.type & ~JS_EVENT_INIT) == JS_EVENT_BUTTON) {
      buttons[event.number] = event.value;
      tbutton[event.number] = event.time;
    }
    if ((event.type & ~JS_EVENT_INIT) == JS_EVENT_AXIS) {
      axes[event.number] = event.value;
      taxis[event.number] = event.time;
    }

    // sleep for 1ms
    usleep(1000);
  }

  joystick_thread_cleanup();
  running = 0;

  return NULL;
}

int joystick_thread_init(char *dev) {
  if (init) {
    // a joystick is already open, close it first
    joystick_thread_cleanup();
  }
  int ret;

  // open joystick device
  fprintf(stderr, "opening joystick: %s...", dev);
  jsFD = open(dev, O_RDONLY);
  fprintf(stderr, "done! fd: %d\n", jsFD);

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
  buttons = (int16_t *)malloc(nbutton * sizeof(int16_t));
  if (!buttons) {
    fprintf(stderr, "unable to allocate button data array\n");
    return -1;
  }
  axes = (int16_t *)malloc(naxis * sizeof(int16_t));
  if (!axes) {
    fprintf(stderr, "unable to allocate axes data array\n");
    return -1;
  }
  tbutton = (uint32_t *)malloc(nbutton * sizeof(uint32_t));
  if (!tbutton) {
    fprintf(stderr, "unable to allocate button time array\n");
    return -1;
  }
  taxis = (uint32_t *)malloc(naxis * sizeof(uint32_t));
  if (!taxis) {
    fprintf(stderr, "unable to allocate axes time array\n");
    return -1;
  }

  // start receiver thread
  fprintf(stderr, "creating joystick thread\n");
  running = 1;
  ret = pthread_create(&jsThread, NULL, joystick_thread_func, NULL);
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

// Default should be "/dev/input/js0"
static PyObject*
py_joystick_open(PyObject* self, PyObject* args) {
  const char* dev;
  if (!PyArg_ParseTuple(args, "s", &dev)) {
    return NULL;
  }

  // Try three times to open
  int i;
  for (i=0; i<3; i++){
    if (joystick_thread_init((char*)dev) == 0) {
      Py_RETURN_TRUE;
    }
    usleep(2000);
  }

  Py_RETURN_FALSE;
}


static PyObject*
py_joystick_close(PyObject* self, PyObject* args) {
  // stop thread
  stopRequest = 1;

  // wait for it to actually stop
  while (running) {
    usleep(1000);
  }
  stopRequest = 0;

  Py_RETURN_NONE;
}

static PyObject*
py_joystick_num_buttons(PyObject* self, PyObject* args) {
  return Py_BuildValue("i", nbutton);
}

static PyObject*
py_joystick_num_axes(PyObject* self, PyObject* args) {
  return Py_BuildValue("i", naxis);
}

static PyObject*
py_joystick_button(PyObject* self, PyObject* args) {
  int ind;
  if (!PyArg_ParseTuple(args, "i", &ind)) {
    return NULL;
  }

  if (ind < 0 || ind>=nbutton) {
    PyObject *lst = PyList_New(nbutton);
    int i;
    for (i = 0; i < nbutton; i++) {
      PyList_SetItem(lst, i, PyLong_FromLong(buttons[i]));
    }
    return Py_BuildValue("O", lst);
  } else {
    // return only indexed axes state
    return Py_BuildValue("i", buttons[ind]);
  }
}

static PyObject*
py_joystick_axis(PyObject* self, PyObject* args) {
  int ind;
  if (!PyArg_ParseTuple(args, "i", &ind)) {
    return NULL;
  }

  if (ind < 0 || ind>=naxis) {
    PyObject *lst = PyList_New(naxis);
    int i;
    for (i = 0; i < naxis; i++) {
      PyList_SetItem(lst, i, PyLong_FromLong(axes[i]));
    }
    return Py_BuildValue("O", lst);
  } else {
    // return only indexed axes state
    return Py_BuildValue("i", axes[ind]);
  }
}

static PyObject*
py_joystick_button_time(PyObject* self, PyObject* args) {
  int ind;
  if (!PyArg_ParseTuple(args, "i", &ind)) {
    return NULL;
  }

  if (ind < 0 || ind>=nbutton) {
    PyObject *lst = PyList_New(nbutton);
    int i;
    for (i = 0; i < nbutton; i++) {
      PyList_SetItem(lst, i, PyLong_FromUnsignedLong(tbutton[i]));
    }
    return Py_BuildValue("O", lst);
  } else {
    // return only indexed axes state
    return Py_BuildValue("I", tbutton[ind]);
  }
}

static PyObject*
py_joystick_axis_time(PyObject* self, PyObject* args) {
  int ind;
  if (!PyArg_ParseTuple(args, "i", &ind)) {
    return NULL;
  }

  if (ind < 0 || ind>=naxis) {
    PyObject *lst = PyList_New(naxis);
    int i;
    for (i = 0; i < naxis; i++) {
      PyList_SetItem(lst, i, PyLong_FromUnsignedLong(taxis[i]));
    }
    return Py_BuildValue("O", lst);
  } else {
    // return only indexed axes state
    return Py_BuildValue("I", taxis[ind]);
  }
}

static PyMethodDef JoystickMethods[] =
{
  {"open", (PyCFunction) py_joystick_open, METH_VARARGS, "Does stuff"},
  {"close", (PyCFunction) py_joystick_close, METH_NOARGS, "Does stuff"},
  {"num_buttons", py_joystick_num_buttons, METH_NOARGS, "Does stuff"},
  {"num_axes", py_joystick_num_axes, METH_NOARGS, "Does stuff"},
  {"button", py_joystick_button, METH_VARARGS, "Does stuff"},
  {"axis", py_joystick_axis, METH_VARARGS, "Does stuff"},
  {"button_time", py_joystick_button_time, METH_VARARGS, "Does stuff"},
  {"axis_time", py_joystick_axis_time, METH_VARARGS, "Does stuff"},
  {NULL, NULL, 0, NULL}
};

// https://docs.python.org/3/howto/cporting.html
struct module_state {
  PyObject *error;
};
#if PY_MAJOR_VERSION >= 3
#define GETSTATE(m) ((struct module_state*)PyModule_GetState(m))
static int joystick_traverse(PyObject *m, visitproc visit, void *arg) {
  Py_VISIT(GETSTATE(m)->error);
  return 0;
}
static int joystick_clear(PyObject *m) {
  Py_CLEAR(GETSTATE(m)->error);
  return 0;
}
static struct PyModuleDef JoystickDef = {
  PyModuleDef_HEAD_INIT,
  "joystick",
  NULL,
  sizeof(struct module_state),
  JoystickMethods,
  NULL,
  joystick_traverse,
  joystick_clear,
  NULL
};
#else
#define GETSTATE(m) (&_state)
static struct module_state _state;
#endif

#if PY_MAJOR_VERSION >= 3
PyMODINIT_FUNC PyInit_joystick(void) {
  PyObject *module = PyModule_Create(&JoystickDef);
  if (module == NULL){
    return NULL;
  }
  // Check for exceptions
  struct module_state *st = GETSTATE(module);
  st->error = PyErr_NewException("joystick.Error", NULL, NULL);
  if (st->error == NULL) {
    Py_DECREF(module);
    return NULL;
  }
  // Finish
  return module;
}
#else
PyMODINIT_FUNC initjoystick(void) {
  PyObject *module = Py_InitModule("joystick", JoystickMethods);
  if (module == NULL){
    return;
  }
  // Check for exceptions
  struct module_state *st = GETSTATE(module);
  st->error = PyErr_NewException("joystick.Error", NULL, NULL);
  if (st->error == NULL) {
    Py_DECREF(module);
    return;
  }
  // Finish
  return;
}
#endif

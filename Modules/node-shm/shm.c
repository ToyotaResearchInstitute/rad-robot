#include <assert.h>
#include <stdio.h>
// For shm access and mmap
#include <sys/mman.h>
// ftruncate
#include <unistd.h>
// mode_t
#include <sys/stat.h>
// open
#include <fcntl.h>
// Filename limits
#include <limits.h>
// stpcpy
#include <string.h>
// errno
#include <errno.h>
// malloc
#include <stdlib.h>
// Node API
#include <node_api.h>

typedef struct shm_info {
  int fd;
  int is_write;
  size_t sz;
  char name[NAME_MAX];
} shm_info;

// Finalizer
static void Close(napi_env env, void *data, void *hint) {
  // fprintf(stderr, "Cleaning up... %x [%x]\n", data, hint);
  if (!hint) {
    return;
  }
  shm_info *my_shm = (shm_info *)hint;
  if (munmap(data, my_shm->sz) < 0) {
    perror("Finalizer munmap");
  }
  if (close(my_shm->fd) < 0) {
    perror("Finalizer close");
  }
  // Remove shm segment if unused
  if (my_shm->is_write) {
    // fprintf(stderr, "Unlinking %s\n", my_shm->name);
    shm_unlink(my_shm->name);
  }
  free(hint);
  // fprintf(stderr, "Done cleaning!\n");
}

napi_value Open(napi_env env, napi_callback_info info) {
  napi_status status;

  size_t argc = 2;
  napi_value args[2];
  status = napi_get_cb_info(env, info, &argc, args, NULL, NULL);
  assert(status == napi_ok);

  // Ensure two arguments
  int open_flags;
  int is_write = 0;
  if (argc == 2) {
    // Create
    open_flags = O_CREAT | O_RDWR | O_EXCL;
    is_write = 1;
  } else if (argc == 1) {
    // Read only
    open_flags = O_RDONLY;
  } else {
    napi_throw_type_error(env, NULL, "Expected two arguments");
    return NULL;
  }

  // Grab the types of the arguments
  napi_valuetype arg_filename;
  status = napi_typeof(env, args[0], &arg_filename);
  assert(status == napi_ok);

  // Ensure the correct types
  if (arg_filename != napi_string) {
    napi_throw_type_error(env, NULL, "Invalid name argument: string expected");
    return NULL;
  }

  // We just want a name - *not* a path
  char shm_filename[NAME_MAX];
  size_t result;
  status =
      napi_get_value_string_utf8(env, args[0], shm_filename, NAME_MAX, &result);
  assert(status == napi_ok);

  // Ensure a leading slash
  if (shm_filename[0] != '/') {
    napi_throw_error(env, NULL, "Name identifier should start with a slash.");
    return NULL;
  }

  // SHM description
  int shm_fd;
  off_t shm_filesize = 0;

  mode_t mode_urw = S_IRUSR | S_IWUSR;
  if (is_write) {
    napi_valuetype arg_filesize;
    status = napi_typeof(env, args[1], &arg_filesize);
    assert(status == napi_ok);
    if (arg_filesize != napi_number) {
      napi_throw_type_error(env, NULL,
                            "Invalid size argument: number expected");
      return NULL;
    }
    int64_t shm_size;
    status = napi_get_value_int64(env, args[1], &shm_size);
    assert(status == napi_ok);
    if (shm_size <= 0) {
      napi_throw_error(
          env, NULL,
          "Invalid size argument: positive non-zero number expected");
      return NULL;
    }
    // fprintf(stderr, "Size: %lld\n", shm_size);
    shm_filesize = (off_t)shm_size;
    // fprintf(stderr, "File Size: %lld\n", shm_filesize);
  }

  // Open the file
  shm_fd = shm_open(shm_filename, open_flags, mode_urw);
  // fprintf(stderr, "FD: %d\n", shm_fd);
  if (shm_fd < 0) {
    int err = errno;
    perror("shm_open");
    if (err == EEXIST && ((open_flags & O_RDWR) == O_RDWR)) {
      napi_throw_error(env, NULL, "File exists! Try removing, first...");
      return NULL;
    }
    napi_throw_error(env, NULL, "shm_open");
    return NULL;
  }

  // Get or set the size
  if (is_write) {
    // fprintf(stderr, "Truncating: %llu\n", shm_filesize);
    if (ftruncate(shm_fd, shm_filesize) < 0) {
      perror("ftruncate");
      close(shm_fd);
      napi_throw_error(env, NULL, "ftruncate SHM");
      return NULL;
    }
  } else {
    // fprintf(stderr, "Getting size: ");
    struct stat shm_stat;
    if (fstat(shm_fd, &shm_stat) < 0) {
      perror("fstat");
      napi_throw_error(env, NULL, "fstat file size");
      return NULL;
    }
    shm_filesize = shm_stat.st_size;
    // fprintf(stderr, "%llu\n", shm_filesize);
  }

  // Map into memory
  int shm_prot = PROT_READ;
  if (is_write) {
    shm_prot |= PROT_WRITE;
  }
  void *shm_data = mmap(NULL, shm_filesize, shm_prot, MAP_SHARED, shm_fd, 0);
  // fprintf(stderr, "mmap: %x\n", (uint8_t*)shm_data);
  if (shm_data == MAP_FAILED) {
    perror("mmap");
    close(shm_fd);
    napi_throw_error(env, NULL, "Failed mmap");
    return NULL;
  }

  // Add finalizer hint for proper mem management
  shm_info *hint = (shm_info *)malloc(sizeof(shm_info));
  hint->sz = shm_filesize;
  hint->fd = shm_fd;
  hint->is_write = is_write;
  stpncpy(hint->name, shm_filename, NAME_MAX);

  // Allow JavaScript to access the data
  napi_value shm_arraybuffer;
  status = napi_create_external_arraybuffer(env, shm_data, shm_filesize, Close,
                                            hint, &shm_arraybuffer);
  assert(status == napi_ok);

  return shm_arraybuffer;
}

napi_value Remove(napi_env env, napi_callback_info info) {
  napi_status status;

  size_t argc = 1;
  napi_value args[1];
  status = napi_get_cb_info(env, info, &argc, args, NULL, NULL);
  assert(status == napi_ok);

  // Ensure one argument
  if (argc != 1) {
    napi_throw_type_error(env, NULL, "Expected one argument: name");
    return NULL;
  }

  // Grab the types of the arguments
  napi_valuetype arg_filename;
  status = napi_typeof(env, args[0], &arg_filename);
  assert(status == napi_ok);

  // Ensure the correct types
  if (arg_filename != napi_string) {
    napi_throw_type_error(env, NULL, "Invalid name argument: string expected");
    return NULL;
  }

  // We just want a name - *not* a path
  char shm_filename[NAME_MAX];
  size_t result;
  status =
      napi_get_value_string_utf8(env, args[0], shm_filename, NAME_MAX, &result);
  assert(status == napi_ok);

  // Ensure a leading slash
  if (shm_filename[0] != '/') {
    napi_throw_error(env, NULL, "Name identifier should start with a slash.");
    return NULL;
  }

  napi_value shm_result;
  int ret = shm_unlink(shm_filename);
  if (ret < 0) {
    perror("shm_unlink");
  }
  status = napi_create_int32(env, ret, &shm_result);
  assert(status == napi_ok);
  return shm_result;
}

napi_value Init(napi_env env, napi_value exports) {
  napi_status status;
  napi_property_descriptor descriptors[] = {
      {"open", NULL, Open, 0, 0, 0, napi_default, 0},
      {"remove", NULL, Remove, 0, 0, 0, napi_default, 0}};
  status = napi_define_properties(env, exports, 2, descriptors);
  assert(status == napi_ok);

  return exports;
}

NAPI_MODULE(NODE_GYP_MODULE_NAME, Init)

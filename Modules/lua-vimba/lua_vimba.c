/*
Author: Stephen McGill <stephen.g.mcgill@gmail.com> 02/17
*/

// Lua
#include <lauxlib.h>
#include <lua.h>
#include <lualib.h>
/* metatable name for vimba */
#define MT_NAME "vimba_mt"

// Vimba
#include <VimbaC.h>
#include <VmbCommonTypes.h>

// Standards and UNIX
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
//
#include <sys/types.h>
#include <unistd.h>

// Malloc and Shared Memory
#include <string.h>
//
#include <fcntl.h>
#include <sys/mman.h>

// Timing
#include <sys/time.h>

#define N_BUFFERS 2

typedef struct camera {
  VmbCameraInfo_t pCamera;
  VmbHandle_t cameraHandle;
  VmbFrame_t *cameraFrames;
  char channel[256];
  uint8_t *shm_ptr;
  size_t shm_sz;
#ifdef RETAIN_MMAP_FD
  int shm_fd;
#endif
} camera_t;

// Global camera handle
static VmbHandle_t g_CameraHandle = (void *)1;

// Calbacks
void VMB_CALL frame_callback(const VmbHandle_t cameraHandle,
    VmbFrame_t *pFrame) {
  VmbError_t err;
  camera_t *camera = (camera_t *)(pFrame->context[0]);
  if (!camera->shm_ptr) {
#ifdef DEBUG
    fprintf(stderr, "NO SHM!!\n");
#endif
    // Re-enqueue frame
    err = VmbCaptureFrameQueue(cameraHandle, pFrame, &frame_callback);
    return;
  }
#ifdef DEBUG
  fprintf(stderr, "Callback | Handle: %p Buffer: %p\n", 
          (void *)cameraHandle, (void *)pFrame);
  fprintf(stderr, "|\tshm_ptr: %p shm_sz: %zu, pFrame->buffer: %p\n",
      camera->shm_ptr, camera->shm_sz, pFrame->buffer);
#endif

  if (pFrame->receiveStatus != VmbFrameStatusComplete) {
#ifdef DEBUG
    switch (pFrame->receiveStatus) {
      case VmbFrameStatusIncomplete:
        fprintf(stderr, "Frame is incomplete\n");
        break;
      case VmbFrameStatusTooSmall:
        fprintf(stderr, "Frame is too small\n");
        break;
      case VmbFrameStatusInvalid:
        fprintf(stderr, "Frame is invalid\n");
        break;
      default:
        fprintf(stderr, "Frame is unknown\n");
        break;
    }
#endif
    // Re-enqueue frame
    err = VmbCaptureFrameQueue(cameraHandle, pFrame, &frame_callback);
    return;
  }

  // Grab the time of the frame
  struct timeval t;
  gettimeofday(&t, NULL);
  int64_t utime = 1e6 * (int64_t)t.tv_sec + t.tv_usec;

#ifdef ENABLE_SHM
  uint8_t* dest = camera->shm_ptr;
  memcpy(dest, pFrame->buffer, camera->shm_sz);
#else
  // TODO: Allow Lua user to access the correct buffer
#endif

  // Re-enqueue frame
  err = VmbCaptureFrameQueue(cameraHandle, pFrame, &frame_callback);

}

// Update camera parameters
VmbError_t update_buffers(camera_t* camera) {
  VmbError_t err;
  VmbInt64_t width, height, payload_sz;

  //
  err = VmbFeatureIntGet(camera->cameraHandle, "Width", &width);
  if (err != VmbErrorSuccess) {
    perror("Width");
    return err;
  } else {
#ifdef DEBUG
    fprintf(stderr, "Width: %ld\n", (int64_t)width);
#endif
  }

  err = VmbFeatureIntGet(camera->cameraHandle, "Height", &height);
  if (err != VmbErrorSuccess) {
    perror("Height");
    return err;
  } else {
#ifdef DEBUG
    fprintf(stderr, "Height: %ld\n", (int64_t)height);
#endif
  }

  // The payload size can be larger than the actual image size.
  err = VmbFeatureIntGet(camera->cameraHandle, "PayloadSize", &payload_sz);
  if (err != VmbErrorSuccess) {
    perror("PayloadSize");
    return err;
  } else {
#ifdef DEBUG
    fprintf(stderr, "PayloadSize: %ld\n", (int64_t)payload_sz);
#endif
  }

  VmbFrame_t *frame_buffers = camera->cameraFrames;
  int ib;
  for (ib = 0; ib < N_BUFFERS; ib++) {
    if (frame_buffers[ib].bufferSize != payload_sz) {
      // TODO: realloc can fail
      frame_buffers[ib].buffer =
        realloc(frame_buffers[ib].buffer, payload_sz);
      frame_buffers[ib].bufferSize = payload_sz;
      // Save the camera pointer for accessing other informaiton
      frame_buffers[ib].context[0] = (void *)camera;
      err = VmbFrameAnnounce(camera->cameraHandle, frame_buffers + ib,
          sizeof(VmbFrame_t));
      if (err != VmbErrorSuccess) {
        fprintf(stderr, "[VmbFrameAnnounce]: ");
        switch(err) {
          case VmbErrorSuccess:
            fprintf(stderr, "VmbErrorSuccess [%d]\n", ib);
            break;
          case VmbErrorApiNotStarted:
            fprintf(stderr, "VmbErrorApiNotStarted [%d]\n", ib);
            break;
          case VmbErrorBadHandle:
            fprintf(stderr, "VmbErrorBadHandle [%d]\n", ib);
            break;
          case VmbErrorBadParameter:
            fprintf(stderr, "VmbErrorBadParameter [%d]\n", ib);
            break;
          case VmbErrorStructSize:
            fprintf(stderr, "VmbErrorStructSize [%d]\n", ib);
            break;
          case VmbErrorOther:
            fprintf(stderr, "VmbErrorOther [%d]\n", ib);
            break;
          default:
            fprintf(stderr, "VmbFrameAnnounce [%d]: %d\n", ib, err);
        }
      } else {
        return err;
      }
    }
  }

  const char * enumBuf;
  err = VmbFeatureEnumGet(camera->cameraHandle, "PixelFormat", &enumBuf);
#ifdef DEBUG
  fprintf(stderr, "PixelFormat: %s\n", enumBuf);
#endif

#ifdef ENABLE_SHM
  // TODO: Resize the Shared memory segment with ftruncate
  camera->shm_sz = payload_sz;
#endif

  return VmbErrorSuccess;
}

static camera_t *lua_checkvimba(lua_State *L, int narg) {
  void *ud = luaL_checkudata(L, narg, MT_NAME);
  luaL_argcheck(L, ud != NULL, narg, "Invalid vimba userdata");
  return (camera_t *)ud;
}

static int lua_vimba_index(lua_State *L) {
  if (!lua_getmetatable(L, 1)) {
    lua_pop(L, 1);
    return 0;
  }
  lua_pushvalue(L, 2);
  lua_rawget(L, -2);
  lua_remove(L, -2);
  return 1;
}

static int lua_vimba_start(lua_State *L) {
  VmbError_t err;
  camera_t *camera = lua_checkvimba(L, 1);

  err = VmbCaptureStart(camera->cameraHandle);
  if (err != VmbErrorSuccess) {
    return luaL_error(L, "VmbCaptureStart");
  }

#ifdef DEBUG
  fprintf(stderr, "Capturing!!\n");
#endif

  err = VmbCaptureFrameQueue(camera->cameraHandle,
                             camera->cameraFrames,
                             &frame_callback);
  if (err != VmbErrorSuccess) {
    return luaL_error(L, "VmbCaptureFrameQueue");
  }

  err = VmbFeatureCommandRun(camera->cameraHandle, "AcquisitionStart");
  if (err != VmbErrorSuccess) {
    return luaL_error(L, "AcquisitionStart");
  }

  // Return true when done
  lua_pushboolean(L, 1);
  return 1;
}

static int lua_vimba_stop(lua_State *L) {
  VmbError_t err;
  camera_t *camera = lua_checkvimba(L, 1);

  err = VmbFeatureCommandRun(camera->cameraHandle, "AcquisitionStop");
  if (err != VmbErrorSuccess) {
    perror("AcquisitionStop");
    //    lua_pushboolean(L, 0);
    //    return 1;
  }

  err = VmbCaptureEnd(camera->cameraHandle);
  if (err != VmbErrorSuccess) {
    perror("VmbCaptureEnd");
    //    lua_pushboolean(L, 0);
    //    return 1;
  }

  err = VmbCaptureQueueFlush(camera->cameraHandle);
  if (err != VmbErrorSuccess) {
    perror("VmbCaptureQueueFlush");
    //    lua_pushboolean(L, 0);
    //    return 1;
  }

  // TODO: Return true when done
  // lua_pushboolean(L, 1);
  // return 1;
  return 0;
}

static int lua_vimba_close(lua_State *L) {

  // First, stop the camera
  lua_vimba_stop(L);

  VmbError_t err;
  camera_t *camera = lua_checkvimba(L, 1);

  // Destroy the frame buffers
  VmbHandle_t cameraHandle = camera->cameraHandle;
  VmbFrame_t *frame_buffers = camera->cameraFrames;
  if (frame_buffers) {
    int ifr;
    for (ifr = 0; ifr < N_BUFFERS; ifr++) {
      if (cameraHandle) {
        VmbFrameRevoke(cameraHandle, frame_buffers + ifr);
      }
      if (frame_buffers[ifr].buffer) {
        free(frame_buffers[ifr].buffer);
        frame_buffers[ifr].buffer = NULL;
      }
    }
    free(frame_buffers);
    camera->cameraFrames = NULL;
  }

  if (camera->cameraHandle) {
    VmbCameraClose(camera->cameraHandle);
  }
  if (camera->shm_ptr) {
    munmap(camera->shm_ptr, camera->shm_sz);
  }
#ifdef RETAIN_MMAP_FD
  if (camera->shm_fd) {
    close(camera->shm_fd);
  }
#endif
  // TODO: Do we ever call shm_unlink(shm_filename) ?
}

// This should be able to be called whenever a parameter is changed
// Parameters include width/height (any changes to payload size)
static int lua_vimba_channel(lua_State *L) {
  VmbError_t err;
  camera_t *camera = lua_checkvimba(L, 1);
  // TODO: Check the validity of the string.
  const char *shm_filename = luaL_checkstring(L, 2);
  // Name the channel
  strcpy(camera->channel, shm_filename);

#ifdef ENABLE_SHM
  int fd_shm = shm_open(shm_filename, O_CREAT | O_RDWR, S_IRUSR | S_IWUSR);
  if (fd_shm == -1) {
    perror("shm_open");
    lua_pushboolean(L, 0);
    return 1;
  }
  fprintf(stderr, "SHM_SZ: %zu\n", camera->shm_sz);
  if (ftruncate(fd_shm, (off_t)camera->shm_sz) != 0) {
    perror("ftruncate");
    lua_pushboolean(L, 0);
    return 1;
  }
  // TODO: Do we ever call shm_unlink(shm_filename) ?
  uint8_t *ptr = mmap(NULL, (size_t)camera->shm_sz, PROT_READ | PROT_WRITE,
      MAP_SHARED, fd_shm, 0);
  if (ptr == MAP_FAILED) {
    perror("mmap");
    lua_pushboolean(L, 0);
    return 1;
  }
  camera->shm_ptr = ptr;
#ifdef RETAIN_MMAP_FD
  camera->shm_fd = fd_shm;
#else
  close(fd_shm);
#endif
#endif

  // Return true when done
  lua_pushboolean(L, 1);
  return 1;
}

static int lua_vimba_load_xml(lua_State *L) {
  VmbError_t err;
  camera_t *camera = lua_checkvimba(L, 1);
  const char *xml_filename = luaL_checkstring(L, 2);
  // Check that we can access the file
  if (access(xml_filename, R_OK) != 0) {
    perror("XML file does not exist");
    lua_pushboolean(L, 0);
    return 1;
  }

  // Load the settings
  VmbFeaturePersistSettings_t settings;
  settings.loggingLevel = 4;
  settings.maxIterations = 5;
  settings.persistType = VmbFeaturePersistNoLUT;
  //
  err = VmbCameraSettingsLoad(camera->cameraHandle, xml_filename, &settings,
      sizeof(settings));
  if (err != VmbErrorSuccess) {
    perror("VmbCameraSettingsLoad");
    lua_pushboolean(L, 0);
    return 1;
  }

  // Update our width/height
  VmbError_t ret = update_buffers(camera);
  if (ret!=VmbErrorSuccess ){
    lua_pushboolean(L, 0);
    return 1;
    // return luaL_error(L, "Update buffers");
  }

  // Return true when done
  lua_pushboolean(L, 1);
  return 1;
}

static int lua_vimba_open(lua_State *L) {
  VmbError_t err;

  // Make the userdata
  camera_t *camera = (camera_t *)lua_newuserdata(L, sizeof(camera_t));
  VmbCameraInfo_t *pCamera = &(camera->pCamera);

  // Query the camera
  if (lua_isstring(L, 1)==0){
    // If no id string given, so list all cameras
    VmbUint32_t nCamera;
    VmbCameraInfo_t *pCameras;
    err = VmbCamerasList(NULL, 0, &nCamera, sizeof(pCameras));
    if (nCamera == 0) {
      lua_pushboolean(L, 0);
      return 1;
    }
    pCameras = malloc(sizeof(VmbCameraInfo_t));
#ifdef DEBUG
    fprintf(stderr, "Found %u cameras\n", nCamera);
#endif
    VmbCamerasList(pCameras, nCamera, &nCamera, sizeof(pCameras));
    const char *cam_id = pCameras[0].cameraIdString;
#ifdef DEBUG
  fprintf(stderr, "Querying: %s\n", cam_id);
#endif
    err = VmbCameraInfoQuery(cam_id, pCamera, (VmbUint32_t)sizeof(VmbCameraInfo_t));
    free(pCameras);
  } else {
    // ID string is given as an input
    const char *cam_id = lua_tostring(L, 1);
#ifdef DEBUG
  fprintf(stderr, "Querying: %s\n", cam_id);
#endif
    err = VmbCameraInfoQuery(cam_id, pCamera, (VmbUint32_t)sizeof(VmbCameraInfo_t));
  }

  if (err != VmbErrorSuccess) {
#ifdef DEBUG
    switch (err) {
      case VmbErrorApiNotStarted:
        fprintf(stderr, "VmbErrorApiNotStarted: %d\n", err);
        break;
      case VmbErrorNotFound:
        fprintf(stderr, "VmbErrorNotFound: %d\n", err);
        break;
      case VmbErrorStructSize:
        fprintf(stderr, "VmbErrorStructSize: %d\n", err);
        break;
      case VmbErrorBadParameter:
        fprintf(stderr, "VmbErrorBadParameter: %d\n", err);
        break;
    }
#endif
    perror("VmbCameraInfoQuery");
    lua_pushboolean(L, 0);
    return 1;
  }
#ifdef DEBUG
    fprintf(stderr, "Opening %s\n", pCamera->cameraIdString);
#endif

  // Open the camera
  err = VmbCameraOpen(pCamera->cameraIdString, VmbAccessModeFull,
      &camera->cameraHandle);
  if (err != VmbErrorSuccess) {
#ifdef DEBUG
    switch (err) {
      case VmbErrorApiNotStarted:
        fprintf(stderr, "VmbErrorApiNotStarted: %d\n", err);
        break;
      case VmbErrorNotFound:
        fprintf(stderr, "VmbErrorNotFound: %d\n", err);
        break;
      case VmbErrorInvalidAccess:
        fprintf(stderr, "VmbErrorInvalidAccess: %d\n", err);
        break;
      case VmbErrorInvalidCall:
        fprintf(stderr, "VmbErrorInvalidCall: %d\n", err);
        break;
      case VmbErrorBadParameter:
        fprintf(stderr, "VmbErrorBadParameter: %d\n", err);
        break;
    }
#endif
    perror("VmbCameraOpen");
    lua_pushboolean(L, 0);
    return 1;
  }

  // Adjust the network settings
  err = VmbFeatureCommandRun(camera->cameraHandle, "GVSPAdjustPacketSize");
  if (err != VmbErrorSuccess) {
    perror("GVSPAdjustPacketSize");
    lua_pushboolean(L, 0);
    return 1;
  }
  uint8_t adjust = 0;
  while (adjust == 0) {
    err = VmbFeatureCommandIsDone(camera->cameraHandle, "GVSPAdjustPacketSize",
        &adjust);
    if (err != VmbErrorSuccess) {
      perror("GVSPAdjustPacketSize");
      lua_pushboolean(L, 0);
      return 1;
    }
  }

  // Zero-fill the buffer structs so we have NULL pointers in
  // buffer and context pointers
  camera->cameraFrames = calloc(N_BUFFERS, sizeof(VmbFrame_t));

  // Update our width/height
  VmbError_t ret = update_buffers(camera);
  if (ret!=VmbErrorSuccess ){
    lua_pushboolean(L, 0);
    return 1;
    // return luaL_error(L, "Update buffers");
  }

  // Give to the user
  luaL_getmetatable(L, MT_NAME);
  lua_setmetatable(L, -2);
  return 1;
}

static int lua_vimba_init(lua_State *L) {
  VmbError_t err;

  // Start the Vimba server
  err = VmbStartup();
  if (err != VmbErrorSuccess) {
    perror("VmbStartup");
    lua_pushboolean(L, 0);
    return 1;
  }

  // Ensure proper settings
  uint8_t is_vtl;
  err = VmbFeatureBoolGet(gVimbaHandle, "GeVTLIsPresent", &is_vtl);
  if (err != VmbErrorSuccess) {
    perror("GeVTLIsPresent");
    //return luaL_error(L, "Could not get GeVTLIsPresent");
    lua_pushboolean(L, 0);
    return 1;
  }

  // Return true when done
  lua_pushboolean(L, 1);
  return 1;
}

static int lua_vimba_shutdown(lua_State *L) {
  VmbShutdown();
  return 0;
}

static const struct luaL_Reg vimba_functions[] = {
  {"init", lua_vimba_init},
  {"open", lua_vimba_open},
  {"shutdown", lua_vimba_shutdown},
  {NULL, NULL}
};

static const struct luaL_Reg vimba_methods[] = {
  {"__index", lua_vimba_index},
  {"__gc", lua_vimba_close},
  {"channel", lua_vimba_channel},
  {"close", lua_vimba_close},
  {"load_xml", lua_vimba_load_xml},
  {"start", lua_vimba_start},
  {"stop", lua_vimba_stop},
  /*
     {"descriptor", lua_vimba_fd},
     {"get_width", lua_vimba_get_width},
     {"get_height", lua_vimba_get_height},
     {"set_param", lua_vimba_set_param},
     {"get_param", lua_vimba_get_param},
     {"get_image", lua_vimba_get_raw},
     */
  {NULL, NULL}
};

#ifdef __cplusplus
extern "C"
#endif
int luaopen_vimba(lua_State *L) {
  /* create metatable for vimba module */
  luaL_newmetatable(L, MT_NAME);
#if LUA_VERSION_NUM == 502
  luaL_setfuncs(L, vimba_methods, 0);
  luaL_newlib(L, vimba_functions);
#else
  luaL_register(L, NULL, vimba_methods);
  luaL_register(L, "vimba", vimba_functions);
#endif
  return 1;
}

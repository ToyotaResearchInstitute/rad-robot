/*
Author: Stephen McGill <stephen.mcgill@tri.global> 2018
*/

// Lua
#include <lauxlib.h>
#include <lua.h>
#include <lualib.h>

// Standards and UNIX
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
//
#include <sys/types.h>
#include <unistd.h>

// Malloc and Shared Memory
#include <string.h>

#ifdef USE_SHM
#include <fcntl.h>
#include <sys/mman.h>
#endif

#ifdef USE_NPP
#include <cuda_runtime.h>
#include <npp.h>
#endif

// Timing
#include <sys/time.h>

// DALSA
#include <gevapi.h>

#include <arpa/inet.h>

/* metatable name for dalsa */
#define MT_NAME "dalsa_mt"

#define N_BUFFERS 2
#define SERIAL_NUMBER_SZ 65

typedef struct camera {
  GEV_CAMERA_HANDLE cameraHandle;
  char serial[SERIAL_NUMBER_SZ];
  uint8_t **image_buffers;
  // GEV_BUFFER_OBJECT *cameraFrames;
  // Image properties
  uint32_t width;
  uint32_t height;
  uint8_t depth;
  // Camera state
  uint8_t is_streaming;
  uint8_t is_open;
#ifdef USE_NPP
  size_t gpuBayerPitch;
  size_t gpuRGBPitch;
#ifdef USE_RESIZE
  size_t gpuRGBPitchResized;
#endif
  size_t cpuRGBPitch;
  Npp8u *gpuBayer;
  Npp8u *gpuRGB;
#ifdef USE_RESIZE
  Npp8u *gpuRGBResized;
#endif
  uint8_t *cpuRGB;
#endif
#ifdef USE_SHM
  uint8_t *shm_ptr;
  size_t shm_sz;
#ifdef RETAIN_MMAP_FD
  int shm_fd;
#endif
  char channel[NAME_MAX];
#endif
} camera_t;

static camera_t *lua_checkdalsa(lua_State *L, int narg) {
  void *ud = luaL_checkudata(L, narg, MT_NAME);
  luaL_argcheck(L, ud != NULL, narg, "Invalid dalsa userdata");
  return (camera_t *)ud;
}

static int lua_dalsa_index(lua_State *L) {
  if (!lua_getmetatable(L, 1)) {
    lua_pop(L, 1);
    return 0;
  }
  lua_pushvalue(L, 2);
  lua_rawget(L, -2);
  lua_remove(L, -2);
  return 1;
}

static int lua_dalsa_get_xml(lua_State *L) {
  camera_t *camera = lua_checkdalsa(L, 1);
  GEV_STATUS status;
  int nret = 0;

  char xml_filename[MAX_PATH];
  status =
      GevGetGenICamXML_FileName(camera->cameraHandle, MAX_PATH, xml_filename);
  if (status != GEVLIB_OK) {
    perror("GevGetGenICamXML_FileName");
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "GevGetGenICamXML_FileName");
    return 2;
  }
#ifdef DEBUG
  fprintf(stderr, "XML stored as %s\n", xml_filename);
#endif
  lua_pushstring(L, xml_filename);
  nret++;

  int sz_xml, is_xml_compressed;
  status = Gev_RetrieveXMLData(camera->cameraHandle, 0, NULL, &sz_xml,
                               &is_xml_compressed);
#ifdef DEBUG
  fprintf(stderr, "Size required: %d | %d\n", sz_xml, is_xml_compressed);
#endif

  char *xml_data = malloc(sz_xml + 1);
  status = Gev_RetrieveXMLData(camera->cameraHandle, sz_xml, xml_data, &sz_xml,
                               &is_xml_compressed);
  if (status != GEVLIB_OK) {
#ifdef DEBUG
    fprintf(stderr, "Cannot get XML data\n");
#endif
    free(xml_data);
    return nret;
  }

  // lua_pushlstring(L, xml_data, sz_xml);
  xml_data[sz_xml] = 0;
  lua_pushstring(L, xml_data);
  nret++;
  free(xml_data);
  return nret;
}

static int lua_dalsa_get_parameter(lua_State *L) {
  camera_t *camera = lua_checkdalsa(L, 1);
  if (!camera->is_open) {
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "Camera is not open");
    return 2;
  }

  const char *key_str = luaL_checkstring(L, 2);

  GEV_STATUS status;
  int value_type;
#define VALUE_BUFFER_SZ 256
  char value_str[VALUE_BUFFER_SZ];
  status = GevGetFeatureValueAsString(camera->cameraHandle, key_str,
                                      &value_type, VALUE_BUFFER_SZ, value_str);

  switch (status) {
  case GEVLIB_OK:
    break;
  default:
#ifdef DEBUG
    fprintf(stderr, "GevGetFeatureValue: %d\n", status);
#endif
    perror("GevGetFeatureValue");
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "Cannot GevGetFeatureValue");
    return 2;
  }

  lua_pushstring(L, value_str);

  return 1;
}

static int lua_dalsa_set_parameter(lua_State *L) {
  camera_t *camera = lua_checkdalsa(L, 1);
  if (!camera->is_open) {
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "Camera is not open");
    return 2;
  } else if (camera->is_streaming) {
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "Cannot set while camera is streaming");
    return 2;
  }

  const char *key_str = luaL_checkstring(L, 2);
  const char *value_str = lua_tostring(L, 3);
  GEV_STATUS status;
  status = GevSetFeatureValueAsString(camera->cameraHandle, key_str, value_str);

  switch (status) {
  case GEVLIB_OK:
    lua_pushboolean(L, 1);
    return 1;
  default:
    perror("GevSetImageParameters");
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "GevSetImageParameters failed");
    return 2;
  }
}

static int lua_dalsa_get_image(lua_State *L) {

  camera_t *camera = lua_checkdalsa(L, 1);
  if (!camera->is_open) {
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "Camera is not open");
    return 2;
  } else if (!camera->is_streaming) {
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "Camera is not streaming");
    return 2;
  }

  GEV_STATUS status;
  GEV_BUFFER_OBJECT *img = NULL;
  status = GevWaitForNextImage(camera->cameraHandle, &img, 1000);

  // Grab the time of the frame
  // struct timeval t;
  // gettimeofday(&t, NULL);
  // int64_t utime = 1e6 * (uint64_t)t.tv_sec + t.tv_usec;

  switch (status) {
  case (GEV_STATUS)GEVLIB_OK:
    break;
  case (GEV_STATUS)GEVLIB_ERROR_TIME_OUT:
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "Timeout");
    return 2;
  case (GEV_STATUS)GEVLIB_ERROR_NULL_PTR:
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "Null image");
    return 2;
  case (GEV_STATUS)GEVLIB_ERROR_INVALID_HANDLE:
  default:
    perror("GevStopImageTransfer");
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "GevStopImageTransfer");
    return 2;
  }

  if (img == NULL) {
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "No image received");
    return 2;
  } else if (img->status != 0) {
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "Error in image");
    return 2;
  }

#ifdef USE_NPP
  // http://www.orangeowlsolutions.com/archives/613
  cudaError_t cuErr;
  NppStatus nvStatus;
  Npp8u *cpuBayer = img->address;
  size_t cpuBayerPitch = camera->width * sizeof(Npp8u);

#ifdef DEBUG
  fprintf(stderr, "cudaMemcpy2D -> dev...\n");
#endif

  cuErr = cudaMemcpy2D(camera->gpuBayer, camera->gpuBayerPitch, cpuBayer,
                       cpuBayerPitch, cpuBayerPitch, camera->height,
                       cudaMemcpyHostToDevice);
  if (cuErr) {
    perror("cudaMemcpy2D -> dev");
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "Failed to cudaMemcpy2D onto the device.");
    return 2;
  }

#ifdef DEBUG
  fprintf(stderr, "nppiCFAToRGB_8u_C1C3R...\n");
#endif
  NppiSize osize;
  osize.width = camera->width;
  osize.height = camera->height;
  NppiRect orect;
  orect.x = 0;
  orect.y = 0;
  orect.width = osize.width;
  orect.height = osize.height;

  nvStatus = nppiCFAToRGB_8u_C1C3R(
      camera->gpuBayer, camera->gpuBayerPitch, osize, orect, camera->gpuRGB,
      camera->gpuRGBPitch, NPPI_BAYER_RGGB, NPPI_INTER_UNDEFINED);
  switch (nvStatus) {
  case NPP_SUCCESS:
    break;
  case NPP_STEP_ERROR:
    perror("nppiCFAToRGB_8u_C1C3R [NPP_STEP_ERROR]");
    break;
  case NPP_NOT_EVEN_STEP_ERROR:
    perror("nppiCFAToRGB_8u_C1C3R [NPP_NOT_EVEN_STEP_ERROR]");
    break;
  case NPP_NULL_POINTER_ERROR:
    perror("nppiCFAToRGB_8u_C1C3R [NPP_NULL_POINTER_ERROR]");
    break;
  case NPP_ALIGNMENT_ERROR:
    perror("nppiCFAToRGB_8u_C1C3R [NPP_ALIGNMENT_ERROR]");
    break;
  case NPP_SIZE_ERROR:
    perror("nppiCFAToRGB_8u_C1C3R [NPP_SIZE_ERROR]");
    break;
  default:
    perror("nppiCFAToRGB_8u_C1C3R");
    break;
  }

#ifdef USE_RESIZE
#ifdef DEBUG
  fprintf(stderr, "nppiResize_8u_C3R...\n");
#endif
  NppiSize psize;
  psize.width = camera->width / 2;
  psize.height = camera->height / 2;
  NppiRect prect;
  prect.x = 0;
  prect.y = 0;
  prect.width = psize.width;
  prect.height = psize.height;

  nvStatus = nppiResize_8u_C3R(
      camera->gpuRGB, camera->gpuRGBPitch, osize, orect, camera->gpuRGBResized,
      camera->gpuRGBPitchResized, psize, prect, NPPI_INTER_LINEAR);

  switch (nvStatus) {
  case NPP_SUCCESS:
    break;
  case NPP_STEP_ERROR:
    perror("nppiResize_8u_C3R [NPP_STEP_ERROR]");
    break;
  case NPP_NOT_EVEN_STEP_ERROR:
    perror("nppiResize_8u_C3R [NPP_NOT_EVEN_STEP_ERROR]");
    break;
  case NPP_NULL_POINTER_ERROR:
    perror("nppiResize_8u_C3R [NPP_NULL_POINTER_ERROR]");
    break;
  case NPP_ALIGNMENT_ERROR:
    perror("nppiResize_8u_C3R [NPP_ALIGNMENT_ERROR]");
    break;
  case NPP_SIZE_ERROR:
    perror("nppiResize_8u_C3R [NPP_SIZE_ERROR]");
    break;
  default:
    perror("nppiResize_8u_C3R");
    break;
  }
#endif

#ifdef DEBUG
  fprintf(stderr, "cudaMemcpy2D -> host...\n");
#endif
// Copy back to the host
#ifdef USE_RESIZE
  cuErr = cudaMemcpy2D(camera->cpuRGB, camera->cpuRGBPitch / 2,
                       camera->gpuRGBResized, camera->gpuRGBPitchResized,
                       camera->cpuRGBPitch / 2, camera->height / 2,
                       cudaMemcpyDeviceToHost);
#else
  cuErr = cudaMemcpy2D(camera->cpuRGB, camera->cpuRGBPitch, camera->gpuRGB,
                       camera->gpuRGBPitch, camera->cpuRGBPitch, camera->height,
                       cudaMemcpyDeviceToHost);
#endif

  switch (cuErr) {
  case cudaSuccess:
    break;
  case cudaErrorInvalidValue:
    perror("cudaMemcpy2D [cudaErrorInvalidValue]");
    break;
  case cudaErrorInvalidPitchValue:
    perror("cudaMemcpy2D [cudaErrorInvalidPitchValue]");
    break;
  case cudaErrorInvalidMemcpyDirection:
    perror("cudaMemcpy2D [cudaErrorInvalidMemcpyDirection]");
    break;
  default:
    perror("cudaMemcpy2D");
    break;
  }
#else
// No NPP for Bayer -> RGB
#ifdef USE_SHM
#ifdef DEBUG
  fprintf(stderr, "shm memcpy %p <- %p [%zu]\n", camera->shm_ptr, img->address,
          camera->shm_sz);
#endif
#ifdef USE_RESIZE
  memcpy(camera->shm_ptr, img->address, camera->shm_sz);
#else
  memcpy(camera->shm_ptr, img->address, camera->shm_sz / 4);
#endif

#endif
#endif

#ifdef USE_SHM
  lua_pushlightuserdata(L, camera->shm_ptr);
#ifdef USE_RESIZE
  lua_pushlstring(L, (const char *)camera->shm_ptr, camera->shm_sz / 4);
#else
  lua_pushlstring(L, (const char *)camera->shm_ptr, camera->shm_sz);
#endif

#else

// No SHM
#ifdef USE_NPP
  size_t szRGB = camera->cpuRGBPitch * camera->height;
  lua_pushlightuserdata(L, camera->cpuRGB);
#ifdef USE_RESIZE
  lua_pushlstring(L, (const char *)camera->cpuRGB, szRGB / 4);
#else
  lua_pushlstring(L, (const char *)camera->cpuRGB, szRGB);
#endif
#else
  // Not really a stable spot
  lua_pushlightuserdata(L, img->address);
  lua_pushnil(L);
#endif

#endif

  // int gev_depth = GevGetPixelDepthInBits(img->format);
  status = GevReleaseFrame(camera->cameraHandle, img);
  // TODO: Expiration time... run a memcpy?
  // TODO: Check the status

  return 2;
}

static int lua_dalsa_stream_off(lua_State *L) {
  camera_t *camera = lua_checkdalsa(L, 1);
  if (!camera->is_streaming) {
    lua_pushboolean(L, 1);
    return 1;
  }

  GEV_STATUS status;
  status = GevStopTransfer(camera->cameraHandle);
  switch (status) {
  case (GEV_STATUS)GEVLIB_OK:
    break;
  case (GEV_STATUS)GEVLIB_ERROR_INVALID_HANDLE:
  default:
    perror("GevStopTransfer");
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "GevStopTransfer");
    return 2;
  }
  status = GevAbortTransfer(camera->cameraHandle);
  switch (status) {
  case (GEV_STATUS)GEVLIB_OK:
    break;
  case (GEV_STATUS)GEVLIB_ERROR_INVALID_HANDLE:
  default:
    perror("GevAbortTransfer");
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "GevAbortTransfer");
    return 2;
  }

  status = GevFreeTransfer(camera->cameraHandle);
  switch (status) {
  case (GEV_STATUS)GEVLIB_OK:
    break;
  case (GEV_STATUS)GEVLIB_ERROR_INVALID_HANDLE:
  default:
    perror("GevFreeTransfer");
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "GevFreeTransfer");
    return 2;
  }
  camera->is_streaming = FALSE;

#ifdef USE_NPP
#ifdef DEBUG
  fprintf(stderr, "cudaFree...\n");
#endif
  cudaError_t cuErr;
  cuErr = cudaFree(camera->gpuBayer);
  switch (cuErr) {
  case cudaSuccess:
    break;
  case cudaErrorInvalidDevicePointer:
    perror("cudaFree(gpuBayer) [cudaErrorInvalidDevicePointer]");
    break;
  case cudaErrorInitializationError:
    perror("cudaFree(gpuBayer) [cudaErrorInitializationError]");
    break;
  default:
    perror("cudaFree(gpuBayer)");
    break;
  }
  cuErr = cudaFree(camera->gpuRGB);
  switch (cuErr) {
  case cudaSuccess:
    break;
  case cudaErrorInvalidDevicePointer:
    perror("cudaFree(gpuRGB) [cudaErrorInvalidDevicePointer]");
    break;
  case cudaErrorInitializationError:
    perror("cudaFree(gpuRGB) [cudaErrorInitializationError]");
    break;
  default:
    perror("cudaFree(gpuRGB)");
    break;
  }

#ifdef USE_RESIZE
  cuErr = cudaFree(camera->gpuRGBResized);
#endif

#ifdef DEBUG
  fprintf(stderr, "free CPU...\n");
#endif
#ifndef USE_SHM
  free(camera->cpuRGB);
#endif

#endif

  // free the buffers
  if (camera->image_buffers) {
    for (int ib = 0; ib < N_BUFFERS; ib++) {
      if (camera->image_buffers[ib]) {
        free(camera->image_buffers[ib]);
        camera->image_buffers[ib] = NULL;
      }
    }
  }

  lua_pushboolean(L, 1);
  return 1;
}

static int lua_dalsa_stream_on(lua_State *L) {
  camera_t *camera = lua_checkdalsa(L, 1);
  if (!camera->is_open) {
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "Camera not open");
    return 1;
  } else if (camera->is_streaming) {
    lua_pushboolean(L, 1);
    return 1;
  }
  GEV_STATUS status;

  // Updating the buffers requires knowledge of the image
  uint32_t x_offset, y_offset;
  uint32_t format;
  status =
      GevGetImageParameters(camera->cameraHandle, &camera->width,
                            &camera->height, &x_offset, &y_offset, &format);

  switch (status) {
  case GEVLIB_OK:
    break;
  default:
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "Cannot get image parameters to update buffers");
    return 2;
  }

  camera->depth = GetPixelSizeInBytes(format);
  size_t payload_sz = camera->depth * camera->width * camera->height;

  // Resize the buffers as needed
  int ib;
  for (ib = 0; ib < N_BUFFERS; ib++) {
    uint8_t *new_ptr = realloc(camera->image_buffers[ib], payload_sz);
    if (new_ptr) {
      camera->image_buffers[ib] = new_ptr;
    } else if (camera->image_buffers[ib]) {
      free(camera->image_buffers[ib]);
      camera->image_buffers[ib] = NULL;
      lua_pushboolean(L, 0);
      lua_pushliteral(L, "Bad buffer!");
      return 2;
    }
  }

  size_t img_sz;
#ifdef USE_NPP
  // RGB image is stored
  img_sz = camera->width * camera->height * 3;
#else
  img_sz = payload_sz;
#endif

#ifdef USE_SHM
  // Name the shm channel
  const char *shm_filename = luaL_checkstring(L, 2);

  // Check if we already have the segment open
  int fd_shm = shm_open(shm_filename, O_CREAT | O_RDWR, S_IRUSR | S_IWUSR);
  if (fd_shm == -1) {
    perror("shm_open");
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "Could not open shared memory");
    return 2;
  }
  int ret = ftruncate(fd_shm, (off_t)img_sz);
  if (ret != 0) {
    perror("ftruncate");
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "Could not set shared memory size");
    return 2;
  }
  uint8_t *ptr_shm =
      mmap(NULL, img_sz, PROT_READ | PROT_WRITE, MAP_SHARED, fd_shm, 0);
  if (ptr_shm == MAP_FAILED) {
    perror("mmap");
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "Could not mmap");
    return 2;
  }
// Update the struct
#ifdef RETAIN_MMAP_FD
  camera->shm_fd = fd_shm;
#else
  close(fd_shm);
#endif
  strcpy(camera->channel, shm_filename);
  camera->shm_sz = img_sz;
  camera->shm_ptr = ptr_shm;
#endif

#ifdef USE_NPP
  cudaError_t cuErr;

#ifdef DEBUG
  fprintf(stderr, "cudaMallocPitch...\n");
#endif
  size_t cpuBayerPitch = camera->width * sizeof(Npp8u);
  // http://www.orangeowlsolutions.com/archives/613
  cuErr = cudaMallocPitch((void **)&camera->gpuBayer, &camera->gpuBayerPitch,
                          cpuBayerPitch, camera->height);
  if (cuErr) {
    perror("cudaMallocPitch");
  }

  camera->cpuRGBPitch = camera->width * 3;
#ifdef USE_SHM
  if (camera->shm_ptr) {
    camera->cpuRGB = camera->shm_ptr;
  } else {
#endif
    // TODO: Ensure rgb_sz == img_sz
    size_t rgb_sz = camera->cpuRGBPitch * camera->height * sizeof(uint8_t);
    camera->cpuRGB = malloc(rgb_sz);
#ifdef USE_SHM
  }
#endif

  // RGB in the CUDA pipeline
  cuErr = cudaMallocPitch((void **)&camera->gpuRGB, &camera->gpuRGBPitch,
                          camera->cpuRGBPitch, camera->height);
  if (cuErr) {
    perror("cudaMallocPitch");
  }

#ifdef USE_RESIZE
  // Resized RGB in the CUDA pipeline
  cuErr = cudaMallocPitch((void **)&camera->gpuRGBResized,
                          &camera->gpuRGBPitchResized, camera->cpuRGBPitch / 2,
                          camera->height / 2);
  if (cuErr) {
    perror("cudaMallocPitch");
  }
#endif

// cudaIpcMemHandle_t ipcHandle;
// cuErr = cudaIpcGetMemHandle(&ipcHandle, void* devPtr);

#endif // USE_NPP

#ifdef USE_SYNCHRONOUS_BUFFER_CYCLING
  // Initialize a transfer with synchronous buffer handling.
  status = GevInitImageTransfer(camera->cameraHandle, SynchronousNextEmpty,
                                N_BUFFERS, camera->image_buffers);
#else
  // Initialize a transfer with asynchronous buffer handling.
  status = GevStartTransfer(camera->cameraHandle, Asynchronous, N_BUFFERS,
                            camera->image_buffers);
#endif

  // Continuous frames
  status = GevStartTransfer(camera->cameraHandle, -1);
  if (status != GEVLIB_SUCCESS) {
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "GevStartTransfer: Cannot begin streaming");
    return 2;
  }
  camera->is_streaming = TRUE;

  // Return true when done
  lua_pushboolean(L, 1);
  return 1;
}

// This also runs on garbage collect
static int lua_dalsa_close(lua_State *L) {

  camera_t *camera = lua_checkdalsa(L, 1);

  GEV_STATUS status;

  if (camera->is_streaming) {
    lua_dalsa_stream_off(L);
  }

  if (camera->is_open) {
    status = GevCloseCamera(&camera->cameraHandle);
    switch (status) {
    case (GEV_STATUS)GEVLIB_OK:
      break;
    case (GEV_STATUS)GEVLIB_ERROR_INVALID_HANDLE:
      fprintf(stderr, "Close | GEVLIB_ERROR_INVALID_HANDLE 0x%0x [%d]\n",
              status, status);
    default:
      perror("GevCloseCamera");
      lua_pushboolean(L, 0);
      lua_pushliteral(L, "GevCloseCamera");
      return 2;
    }
    camera->is_open = FALSE;
  }

// Free up the memory mapping
#ifdef USE_SHM
  if (camera->shm_ptr) {
    int ret = munmap(camera->shm_ptr, camera->shm_sz);
    if (ret != 0) {
      perror("munmap");
    }
    camera->shm_sz = 0;
    camera->shm_ptr = NULL;
  }
#ifdef RETAIN_MMAP_FD
  if (camera->shm_fd > 0) {
    close(camera->shm_fd);
    camera->shm_fd = -1;
  }
#endif
  // Remove the SHM chunk
  if (strnlen(camera->channel, NAME_MAX) > 0) {
    shm_unlink(camera->channel);
    bzero(camera->channel, NAME_MAX);
  }
#endif

  if (camera->image_buffers) {
    free(camera->image_buffers);
    camera->image_buffers = NULL;
  }

  // NOTE: Keep the serial number, for re-opening

  lua_pushboolean(L, 1);
  return 1;
}

static int lua_dalsa_shutdown(lua_State *L) {
  // Close down the API.
  GevApiUninitialize();

  // Close socket API
  _CloseSocketAPI(); // must close API even on error

  lua_pushboolean(L, 1);
  return 1;
}

static int lua_dalsa_initialize(lua_State *L) {
  GEV_STATUS status;
  // Close down the API.
  status = GevApiInitialize();
  if (status != GEVLIB_OK) {
    lua_pushboolean(L, 0);
    return 1;
  }
#ifdef DEBUG
  {
    GEVLIB_CONFIG_OPTIONS options = {0};
    GevGetLibraryConfigOptions(&options);
    // options.logLevel = GEV_LOG_LEVEL_OFF;
    options.logLevel = GEV_LOG_LEVEL_TRACE;
    // options.logLevel = GEV_LOG_LEVEL_NORMAL;
    // options.logLevel = GEV_LOG_LEVEL_DEBUG;
    GevSetLibraryConfigOptions(&options);
  }
#endif

  if (lua_istable(L, -1)) {
    uint32_t maxInterfaces;
#if LUA_VERSION_NUM == 501
    maxInterfaces = lua_objlen(L, 1);
#else
    maxInterfaces = lua_rawlen(L, 1);
#endif
#ifdef DEBUG
    fprintf(stderr, "Enumerating %d interfaces\n", maxInterfaces);
#endif
    int i;
    struct in_addr ip_addr;
    GEV_NETWORK_INTERFACE *pIPAddr = malloc(
        maxInterfaces *
        sizeof(GEV_NETWORK_INTERFACE));
    for (i = 0; i < maxInterfaces; i++) {
      lua_rawgeti(L, -1, i + 1);
      inet_aton(lua_tostring(L, -1), &ip_addr);
      lua_pop(L, 1);
#ifdef DEBUG
      fprintf(stderr, "Interface %d: [%d]\n", i, ip_addr.s_addr);
#endif
      pIPAddr[i].ipAddr = ip_addr.s_addr;
    }
    uint32_t numInterfaces;
    status =
      GevEnumerateNetworkInterfaces(pIPAddr, maxInterfaces, &numInterfaces);
#ifdef DEBUG
    fprintf(stderr, "Enumerated %d interfaces\n", numInterfaces);
#endif
    // One camera per interface
    uint32_t maxDevices = maxInterfaces;
    GEV_CAMERA_INFO* pDevice = malloc(maxDevices * sizeof(GEV_CAMERA_INFO));
    uint32_t numDevices;
    for(i=0;i<numInterfaces;i++){
      status = GevEnumerateGevDevices(pIPAddr + i, 3e3, pDevice, maxDevices, &numDevices);
#ifdef DEBUG
      ip_addr.s_addr = htonl(pIPAddr[i].ipAddr);
      fprintf(stderr, "Enumerated %d devices on %s\n", numDevices, inet_ntoa(ip_addr));
#endif
      if (status!=GEV_STATUS_SUCCESS) {
        perror("GevEnumerateGevDevices");
      }
    }
    free(pDevice);
    free(pIPAddr);

  }



  lua_pushboolean(L, 1);
  return 1;
}

static int lua_dalsa_open(lua_State *L) {
  GEV_STATUS status;


  // Make the userdata
  camera_t *camera = (camera_t *)lua_newuserdata(L, sizeof(camera_t));
  camera->is_open = FALSE;
  camera->is_streaming = FALSE;
#ifdef USE_SHM
  camera->shm_ptr = NULL;
  bzero(camera->channel, NAME_MAX);
  bzero(camera->serial, SERIAL_NUMBER_SZ);
  camera->shm_sz = 0;
#ifdef RETAIN_MMAP_FD
  shm_fd = -1;
#endif
#endif

#ifdef DEBUG
  fprintf(stderr, "In type: %s\n", lua_typename(L, lua_type(L, 1)));
#endif

  if (lua_isuserdata(L, 1)) {
// Check if given a camera object
#ifdef DEBUG
    fprintf(stderr, "Re-opening camera...\n");
#endif
    // camera_t *camera = lua_checkdalsa(L, 1);
    // status = GevOpenCameraBySN(camera->serial, GevExclusiveMode,
    //                            &camera->cameraHandle);
    // Open it, again
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "Reopening not implemented, yet");
    return 2;
  } else if (lua_isstring(L, 1)) {
    // Serial number is given
    const char *serial_no = lua_tostring(L, 1);
#ifdef DEBUG
    fprintf(stderr, "Querying serial_no: %s\n", serial_no);
#endif
    status = GevOpenCameraBySN((char *)serial_no, GevExclusiveMode,
                               &camera->cameraHandle);
  } else if (lua_isnumber(L, 1)) {
    // IP address hex number is given
    uint32_t ip_address = lua_tointeger(L, 1);
#ifdef DEBUG
    fprintf(stderr, "Querying Address: %x\n", ip_address);
#endif
    status = GevOpenCameraByAddress(ip_address, GevExclusiveMode,
                                    &camera->cameraHandle);
  } else {

#ifdef DEBUG
    fprintf(stderr, "Opening new camera\n");
#endif
    int nCamera = GevDeviceCount();
#ifdef DEBUG
    fprintf(stderr, "Found %u cameras\n", nCamera);
#endif
    if (nCamera == 0) {
      lua_pushboolean(L, 0);
      lua_pushliteral(L, "No cameras found.");
      return 2;
    }
    // If no id string given, open the first camera seen
    GEV_DEVICE_INTERFACE *pCameras = malloc(nCamera * sizeof(GEV_DEVICE_INTERFACE));
    status = GevGetCameraList(pCameras, nCamera, &nCamera);
    if (status != GEVLIB_SUCCESS || nCamera == 0) {
      free(pCameras);
      lua_pushboolean(L, 0);
      lua_pushliteral(L, "GevGetCameraList failed.");
      return 2;
    }
    // First camera is index 0
    int idx = 0;
    status =
        GevOpenCamera(&pCameras[idx], GevExclusiveMode, &camera->cameraHandle);
    free(pCameras);
  }
#ifdef DEBUG
  uint16_t v = status;
  fprintf(stderr, "GevOpenCamera status: %d\n", *((int16_t *)&v));
#endif

  switch (status) {
  case (GEV_STATUS)GEVLIB_SUCCESS:
    break;
  case (GEV_STATUS)GEVLIB_ERROR_API_NOT_INITIALIZED:
    perror("GEVLIB_ERROR_API_NOT_INITIALIZED");
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "GEVLIB_ERROR_API_NOT_INITIALIZED");
    return 2;
  case (GEV_STATUS)GEVLIB_ERROR_INVALID_HANDLE:
    perror("GEVLIB_ERROR_INVALID_HANDLE");
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "GEVLIB_ERROR_INVALID_HANDLE");
    return 2;
  case (GEV_STATUS)GEVLIB_ERROR_INSUFFICIENT_MEMORY:
    perror("GEVLIB_ERROR_INSUFFICIENT_MEMORY");
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "GEVLIB_ERROR_INSUFFICIENT_MEMORY");
    return 2;
  case (GEV_STATUS)GEVLIB_ERROR_NO_CAMERA:
    perror("GEVLIB_ERROR_NO_CAMERA");
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "GEVLIB_ERROR_NO_CAMERA");
    return 2;
  case (GEV_STATUS)GEVLIB_ERROR_DEVICE_NOT_FOUND:
    perror("GEVLIB_ERROR_DEVICE_NOT_FOUND");
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "GEVLIB_ERROR_DEVICE_NOT_FOUND");
    return 2;
  case (GEV_STATUS)GEV_STATUS_ACCESS_DENIED:
    perror("GEV_STATUS_ACCESS_DENIED");
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "GEV_STATUS_ACCESS_DENIED");
    return 2;
  default:
    perror("Opening camera");
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "Failed to open camera");
    return 2;
  }
  camera->is_open = TRUE;

  GEV_CAMERA_INFO *cameraInfo = GevGetCameraInfo(camera->cameraHandle);
#ifdef DEBUG
  fprintf(stderr, "Opened serial number %s\n", cameraInfo->serial);
#endif
  // Save the serial number
  memcpy(camera->serial, cameraInfo->serial, SERIAL_NUMBER_SZ);

  // TODO: Adjust the network settings
  {
    GEV_CAMERA_OPTIONS camOptions = {0};

    // Adjust the camera interface options if desired (see the manual)
    GevGetCameraInterfaceOptions(camera->cameraHandle, &camOptions);
    // For debugging (delay camera timeout while in debugger)
    camOptions.heartbeat_timeout_ms = 90000;

#ifdef TUNE_STREAMING_THREADS
    // Some tuning can be done here. (see the manual)
    // Internal timeout for frame reception.
    camOptions.streamFrame_timeout_ms = 1001;
    // Buffer frames internally.
    camOptions.streamNumFramesBuffered = 4;
    // Adjust packet memory buffering limit.
    camOptions.streamMemoryLimitMax = 64 * 1024 * 1024;
    // Adjust the GVSP packet size.
    camOptions.streamPktSize = 9180;
    // Add usecs between packets to pace arrival at NIC.
    camOptions.streamPktDelay = 10;

    // Assign specific CPUs to threads (affinity) - if required for better
    // performance.
    {
      int numCpus = _GetNumCpus();
      if (numCpus > 1) {
        camOptions.streamThreadAffinity = numCpus - 1;
        camOptions.serverThreadAffinity = numCpus - 2;
      }
    }
#endif
    // Write the adjusted interface options back.
    GevSetCameraInterfaceOptions(camera->cameraHandle, &camOptions);
  }

  // Setting XML early is important: Cannot get parameters otherwise
  status = GevInitGenICamXMLFeatures(camera->cameraHandle, TRUE);
  if (status != GEVLIB_OK) {
    perror("GevInitGenICamXMLFeatures");
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "GevInitGenICamXMLFeatures");
    return 2;
  }

  // TODO: Add turbo

  // Zero-fill the buffer structs so we have NULL pointers in
  // buffer and context pointers
  camera->image_buffers = calloc(N_BUFFERS, sizeof(uint8_t *));

  // Give to the user
  luaL_getmetatable(L, MT_NAME);
  lua_setmetatable(L, -2);

  // Also push out the serial number
  lua_pushstring(L, cameraInfo->serial);
  return 2;
}

static int lua_dalsa_list(lua_State *L) {
  GEV_STATUS status;
#ifdef DEBUG
  fprintf(stderr, "Opening new camera\n");
#endif
  int nCamera = GevDeviceCount();
#ifdef DEBUG
  fprintf(stderr, "Found %u cameras\n", nCamera);
#endif
  if (nCamera == 0) {
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "No cameras found.");
    return 2;
  }
  // If no id string given, open the first camera seen
  GEV_DEVICE_INTERFACE *pCameras = malloc(nCamera * sizeof(GEV_DEVICE_INTERFACE));
  status = GevGetCameraList(pCameras, nCamera, &nCamera);
  if (status != GEVLIB_SUCCESS || nCamera == 0) {
    free(pCameras);
    lua_pushboolean(L, 0);
    lua_pushliteral(L, "GevGetCameraList failed.");
    return 2;
  }
  lua_createtable(L, nCamera, 0);
  for (int i = 0; i < nCamera; i++) {
    /*
    lua_pushliteral(L, "serial");
    lua_pushlstring(L, pCameras[i].serial, 65);
    lua_pushliteral(L, "addr");
    lua_pushinteger(L, pCameras[i].ipAddr);
    */
    struct in_addr ip_addr;
    ip_addr.s_addr = pCameras[i].ipAddr;
    lua_pushstring(L, inet_ntoa(ip_addr));
    lua_rawseti(L, -2, i + 1);
  }
  return 1;
}

static const struct luaL_Reg dalsa_functions[] = {
    {"initialize", lua_dalsa_initialize},
    {"list", lua_dalsa_list},
    {"open", lua_dalsa_open},
    {"shutdown", lua_dalsa_shutdown},
    {NULL, NULL}};

static const struct luaL_Reg dalsa_methods[] = {
    {"__index", lua_dalsa_index},
    {"open", lua_dalsa_open}, // Re-open
    {"close", lua_dalsa_close},
    {"__gc", lua_dalsa_close},
    {"stream_on", lua_dalsa_stream_on},
    {"stream_off", lua_dalsa_stream_off},
    {"get_parameter", lua_dalsa_get_parameter},
    {"set_parameter", lua_dalsa_set_parameter},
    {"get_xml", lua_dalsa_get_xml},
    {"get_image", lua_dalsa_get_image},
    {NULL, NULL}};

#ifdef __cplusplus
extern "C"
#endif
    int
    luaopen_dalsa(lua_State *L) {
  /* create metatable for dalsa module */
  luaL_newmetatable(L, MT_NAME);
#if LUA_VERSION_NUM == 501
  luaL_register(L, NULL, dalsa_methods);
  luaL_register(L, "dalsa", dalsa_functions);
#else
  luaL_setfuncs(L, dalsa_methods, 0);
  luaL_newlib(L, dalsa_functions);
#endif
  return 1;
}

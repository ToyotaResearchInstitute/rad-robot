/*
  + This is a wrapper for matlab std I/O functions

  + Supported Types (LOAD):
        mxCELL_CLASS
        mxSTRUCT_CLASS
        mxLOGICAL_CLASS
        mxCHAR_CLASS      Y
        mxDOUBLE_CLASS    Y
        mxSINGLE_CLASS    Y
        mxINT8_CLASS      Y
        mxUINT8_CLASS     Y
        mxINT16_CLASS     Y
        mxUINT16_CLASS    Y (casts to INT16)
        mxINT32_CLASS     Y
        mxUINT32_CLASS    Y (casts to INT32)
        mxINT64_CLASS
        mxUINT64_CLASS
        mxFUNCTION_CLASS

  + Supported Types (SAVE):
        mxCELL_CLASS
        mxSTRUCT_CLASS
        mxLOGICAL_CLASS
        mxCHAR_CLASS
        mxDOUBLE_CLASS    Y
        mxSINGLE_CLASS
        mxINT8_CLASS
        mxUINT8_CLASS
        mxINT16_CLASS
        mxUINT16_CLASS
        mxINT32_CLASS
        mxUINT32_CLASS
        mxINT64_CLASS
        mxUINT64_CLASS
        mxFUNCTION_CLASS

  -
*/

// To load this lib in LUA:
// require 'libmatlab'

#include <torch/luaT.h>
#include <torch/TH/TH.h>

#include "mat.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// Loader
static int load_l(lua_State *L) {
  // get args
  const char *path = lua_tostring(L,1);

  // open file
  MATFile *file = matOpen(path, "r");
  if (file == NULL) THError("Error opening file %s", file);

  // create table to hold loaded variables
  lua_newtable(L);  // vars = {}
  int vars = lua_gettop(L);
  //int varidx = 1;

  // extract each var
  while (true) {
    // get var+name
    const char *name;
    mxArray *pa = matGetNextVariable(file, &name);
    if (pa == NULL) break;

    // get dimensions
    mwSize ndims = mxGetNumberOfDimensions(pa);
    const mwSize *dims = mxGetDimensions(pa);

    // infer size and stride
    int k;
    THLongStorage *size = THLongStorage_newWithSize(ndims);
    THLongStorage *stride = THLongStorage_newWithSize(ndims);
    for (k=0; k<ndims; k++) {
      THLongStorage_set(size, ndims-k-1, dims[k]);
      if (k > 0)
        THLongStorage_set(stride, ndims-k-1, dims[k-1]*THLongStorage_get(stride,ndims-k));
      else
        THLongStorage_set(stride, ndims-k-1, 1);
    }

    // depending on type, create equivalent Lua/torch data structure
    if (mxGetClassID(pa) == mxDOUBLE_CLASS) {
      THDoubleTensor *tensor = THDoubleTensor_newWithSize(size, stride);
      memcpy((void *)(THDoubleTensor_data(tensor)),
             (void *)(mxGetPr(pa)), THDoubleTensor_nElement(tensor) * sizeof(double));
      lua_pushstring(L, name);
      luaT_pushudata(L, tensor, luaT_typenameid(L, "torch.DoubleTensor"));
      lua_rawset(L, vars);

    } else if (mxGetClassID(pa) == mxSINGLE_CLASS) {
      THFloatTensor *tensor = THFloatTensor_newWithSize(size, stride);
      memcpy((void *)(THFloatTensor_data(tensor)),
             (void *)(mxGetPr(pa)), THFloatTensor_nElement(tensor) * sizeof(float));
      lua_pushstring(L, name);
      luaT_pushudata(L, tensor, luaT_typenameid(L, "torch.FloatTensor"));
      lua_rawset(L, vars);

    } else if (mxGetClassID(pa) == mxINT32_CLASS) {
      THIntTensor *tensor = THIntTensor_newWithSize(size, stride);
      memcpy((void *)(THIntTensor_data(tensor)),
             (void *)(mxGetPr(pa)), THIntTensor_nElement(tensor) * sizeof(int));
      lua_pushstring(L, name);
      luaT_pushudata(L, tensor, luaT_typenameid(L, "torch.IntTensor"));
      lua_rawset(L, vars);

    } else if (mxGetClassID(pa) == mxUINT32_CLASS) {
      THIntTensor *tensor = THIntTensor_newWithSize(size, stride);
      memcpy((void *)(THIntTensor_data(tensor)),
             (void *)(mxGetPr(pa)), THIntTensor_nElement(tensor) * sizeof(int));
      lua_pushstring(L, name);
      luaT_pushudata(L, tensor, luaT_typenameid(L, "torch.IntTensor"));
      lua_rawset(L, vars);

    } else if ((mxGetClassID(pa) == mxINT16_CLASS)) {
      THShortTensor *tensor = THShortTensor_newWithSize(size, stride);
      memcpy((void *)(THShortTensor_data(tensor)),
             (void *)(mxGetPr(pa)), THShortTensor_nElement(tensor) * sizeof(short));
      lua_pushstring(L, name);
      luaT_pushudata(L, tensor, luaT_typenameid(L, "torch.ShortTensor"));
      lua_rawset(L, vars);

    } else if ((mxGetClassID(pa) == mxUINT16_CLASS)) {
      THShortTensor *tensor = THShortTensor_newWithSize(size, stride);
      memcpy((void *)(THShortTensor_data(tensor)),
             (void *)(mxGetPr(pa)), THShortTensor_nElement(tensor) * sizeof(short));
      lua_pushstring(L, name);
      luaT_pushudata(L, tensor, luaT_typenameid(L, "torch.ShortTensor"));
      lua_rawset(L, vars);

    } else if ((mxGetClassID(pa) == mxINT8_CLASS) || (mxGetClassID(pa) == mxCHAR_CLASS)) {
      THCharTensor *tensor = THCharTensor_newWithSize(size, stride);
      memcpy((void *)(THCharTensor_data(tensor)),
             (void *)(mxGetPr(pa)), THCharTensor_nElement(tensor) * sizeof(char));
      lua_pushstring(L, name);
      luaT_pushudata(L, tensor, luaT_typenameid(L, "torch.CharTensor"));
      lua_rawset(L, vars);

    } else if ((mxGetClassID(pa) == mxUINT8_CLASS)) {
      THByteTensor *tensor = THByteTensor_newWithSize(size, stride);
      memcpy((void *)(THByteTensor_data(tensor)),
             (void *)(mxGetPr(pa)), THByteTensor_nElement(tensor) * sizeof(char));
      lua_pushstring(L, name);
      luaT_pushudata(L, tensor, luaT_typenameid(L, "torch.ByteTensor"));
      lua_rawset(L, vars);

    } else if ((mxGetClassID(pa) == mxLOGICAL_CLASS)) {
      THByteTensor *tensor = THByteTensor_newWithSize(size, stride);
      memcpy((void *)(THByteTensor_data(tensor)),
             (void *)(mxGetPr(pa)), THByteTensor_nElement(tensor) * sizeof(char));
      lua_pushstring(L, name);
      luaT_pushudata(L, tensor, luaT_typenameid(L, "torch.ByteTensor"));
      lua_rawset(L, vars);

    } else {
      lua_pushstring(L, name);
      if ((mxGetClassID(pa) == mxCELL_CLASS)) {
        lua_pushstring(L, "unsupported type: mxCELL_CLASS");
      } else if ((mxGetClassID(pa) == mxSTRUCT_CLASS)) {
        lua_pushstring(L, "unsupported type: mxSTRUCT_CLASS");
      } else if ((mxGetClassID(pa) == mxINT64_CLASS)) {
        lua_pushstring(L, "unsupported type: mxINT64_CLASS");
      } else if ((mxGetClassID(pa) == mxUINT64_CLASS)) {
        lua_pushstring(L, "unsupported type: mxUINT64_CLASS");
      } else if ((mxGetClassID(pa) == mxFUNCTION_CLASS)) {
        lua_pushstring(L, "unsupported type: mxFUNCTION_CLASS");
      } else {
        lua_pushstring(L, "unknown type");
      }
      lua_rawset(L, vars);
    }
    mxDestroyArray(pa);
  }

  // cleanup
  matClose(file);

  // return table 'vars'
  return 1;
}

// Save single tensor
static int save_tensor_l(lua_State *L) {
  // open file for output
  const char *path = lua_tostring(L,1);
  MATFile *file = matOpen(path, "w");

  // load tensor
  THDoubleTensor *tensor = (THDoubleTensor *)luaT_checkudata(L, 2, luaT_typenameid(L, "torch.DoubleTensor"));
  THDoubleTensor *tensorc = THDoubleTensor_newContiguous(tensor);

  // infer size and stride
  int k;
  mwSize size[] = {-1,-1,-1,-1,-1,-1,-1,-1};
  const long ndims = tensorc->nDimension;
  for (k=0; k<ndims; k++) {
    size[k] = tensor->size[ndims-k-1];
  }

  // create matlab array
  mxArray *pm = mxCreateNumericArray(ndims, size, mxDOUBLE_CLASS, mxREAL);

  // copy tensor
  memcpy((void *)(mxGetPr(pm)),
         (void *)(THDoubleTensor_data(tensor)),
         THDoubleTensor_nElement(tensor) * sizeof(double));

  // save it, in a dummy var named 'x'
  const char *name = "x";
  matPutVariable(file, name, pm);

  // done
  THDoubleTensor_free(tensorc);
  matClose(file);
  return 0;
}

// Table of numbers at -1 (Top of the Lua stack)
mxArray* vector2mxArray(lua_State *L) {
  #if LUA_VERSION_NUM == 502
    int nval = lua_rawlen(L, -1);
  #else
    int nval = lua_objlen(L, -1);
  #endif
  mwSize size[] = {nval};
  mxArray *pm = mxCreateNumericArray(
    1, size, mxDOUBLE_CLASS, mxREAL);
  double* pm_ptr = mxGetPr(pm);
  for (int i = 0; i < nval; i++) {
    lua_rawgeti(L, -1, i+1);
    pm_ptr[i] = lua_tonumber(L, -1);
    //printf("pm_ptr[i] = %lf\n", pm_ptr[i]);
    lua_pop(L, 1);
  }
  return pm;
}

// Torch tensor at -1 (Top of the Lua stack)
mxArray* tensor2mxArray(lua_State *L){
  THDoubleTensor *tensor = (THDoubleTensor *)
    luaT_checkudata(L, -1,
      luaT_typenameid(L, "torch.DoubleTensor")
    );
  // NOTE: This is silly... just for memcpy ability
  THDoubleTensor *tensorc =
    THDoubleTensor_newContiguous(tensor);

  // infer size and stride
  mwSize size[] = {-1,-1,-1,-1,-1,-1,-1,-1};
  const long ndims = tensorc->nDimension;
  int k;
  for (k=0; k<ndims; k++) {
    size[k] = tensor->size[ndims-k-1];
  }

  // create matlab array
  mxArray* pm = mxCreateNumericArray(
    ndims, size, mxDOUBLE_CLASS, mxREAL);

  //printf("nTensor: %ld\n", THDoubleTensor_nElement(tensor));

  // copy tensor into array
  memcpy((void *)(mxGetPr(pm)),
         (void *)(THDoubleTensor_data(tensorc)),
         THDoubleTensor_nElement(tensor) * sizeof(double));
  // cleanup
  THDoubleTensor_free(tensorc);
  return pm;
}

// Save table of tensors
static int save_table_l(lua_State *L) {
  // open file for output
  const char *path = lua_tostring(L,1);
  MATFile *file = matOpen(path, "w");

  // 1024 is Max number of arrays
  mxArray **pms;
  mxArray *pmi;
  pms = (mxArray**) malloc(sizeof(mxArray*)*1024);
  int counter = 0;
  // table is in the stack at index 2 (2nd var)
  lua_pushnil(L);  // first key
  while (lua_next(L, 2) != 0) {
    // uses 'key' (at index -2) and 'value' (at index -1)
    const char *name = lua_tostring(L, -2);
    pmi = NULL;
    // Check the type of the value
    int t = lua_type(L, -1);
    if (t==LUA_TUSERDATA) { // torch data
      // Refer to this array
      pmi = tensor2mxArray(L);
    } else if(t==LUA_TNUMBER) {
      pmi = mxCreateDoubleScalar(lua_tonumber(L, -1));
    } else if(t==LUA_TTABLE) {
      // Check the first element type
      lua_rawgeti(L, -1, 1);
      int tt = lua_type(L, -1);
      lua_pop(L, 1);
      // Process, given the type of the first element
      if(tt==LUA_TNUMBER){
        // Table of numbers
        pmi = vector2mxArray(L);
      } else if(tt==LUA_TUSERDATA) {
        // Table of torch
        #if LUA_VERSION_NUM == 502
          int nval = lua_rawlen(L, -1);
        #else
          int nval = lua_objlen(L, -1);
        #endif
        pmi = mxCreateCellMatrix(nval, 1);
        for (int i = 0; i < nval; i++) {
          lua_rawgeti(L, -1, i+1);
          mxSetCell(pmi, i, tensor2mxArray(L));
          lua_pop(L, 1);
        }
      } else if(tt==LUA_TTABLE) {
        // Table of vectors
        #if LUA_VERSION_NUM == 502
          int nval = lua_rawlen(L, -1);
        #else
          int nval = lua_objlen(L, -1);
        #endif
        pmi = mxCreateCellMatrix(nval, 1);
        for (int i = 0; i < nval; i++) {
          lua_rawgeti(L, -1, i+1);
          mxSetCell(pmi, i, vector2mxArray(L));
          lua_pop(L, 1);
        }
      }
      //else { return luaL_error(L, "Bad mattorch array elements"); }
    }
//else { return luaL_error(L, "Bad mattorch element type"); }

    if(pmi){
      // store it
      pms[counter++] = pmi;
      matPutVariable(file, name, pmi);
    }

    // removes 'value'; keeps 'key' for next iteration
    lua_pop(L, 1);
  }
  int i = 0;
  for(i=0; i<counter;i++){
    mxDestroyArray(pms[i]);
  }

  free(pms);

  // cleanup
  lua_pop(L, 1);
  matClose(file);
  return 0;
}

static int save_tensor_ascii_l(lua_State *L)
{
  // get file descriptor
  THFile *file = (THFile *)luaT_checkudata(L, 1, luaT_typenameid(L, "torch.File"));

  // load tensor
  THDoubleTensor *tensor = (THDoubleTensor *)luaT_checkudata(L, 2, luaT_typenameid(L, "torch.DoubleTensor"));
  THDoubleTensor *tensorc = THDoubleTensor_newContiguous(tensor);
  double *tensor_data = THDoubleTensor_data(tensorc);

  // get sizes
  const long ndims = tensorc->nDimension;
  if (ndims > 2) {
    THError("matlab ascii only supports 1d or 2d tensors");
  }

  // write all
  int i;
  if (ndims == 2) {
    for (i = 0; i < tensorc->size[0]; i ++) {
      THFile_writeDoubleRaw(file, tensor_data, tensorc->size[1]);
      tensor_data += tensorc->size[1];
    }
  } else {
    for (i = 0; i < tensorc->size[0]; i ++) {
      THFile_writeDoubleRaw(file, tensor_data, 1);
      tensor_data += 1;
    }
  }

  // cleanup
  THDoubleTensor_free(tensorc);
  return 0;
}

// Register functions in LUA
static const struct luaL_reg mattorch [] = {
  {"load", load_l},
  {"saveTensor", save_tensor_l},
  {"saveTable", save_table_l},
  {"saveTensorAscii", save_tensor_ascii_l},
  {NULL, NULL}  /* sentinel */
};

int luaopen_mattorch (lua_State *L) {
  luaL_openlib(L, "mattorch", mattorch, 0);
  return 1;
}

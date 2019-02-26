#!/usr/bin/env luajit
local ffi = require'ffi'
local slicot = ffi.load"slicot"
local has_matrix, matrix = pcall(require, 'matrix')
if not has_matrix then
  io.stderr:write"No Matrix\n"
end
local has_lapack, lapack = pcall(require, 'lapack')
if not has_lapack then
  io.stderr:write"No LAPACK\n"
end

-- Input: Row major
-- Output: Column major
local function mtx2fortran(mtx)
  local n_rows = #mtx
  local n_cols = #mtx[1]
  local nrnc = ffi.new("int[2]", {n_rows, n_cols})
  local mtx_cdata = ffi.new(string.format("double[%d][%d]",
                                          n_cols, n_rows))
  for ir, row in ipairs(mtx) do
    for ic, v in ipairs(row) do
      mtx_cdata[ic-1][ir-1] = v
    end
  end
  return mtx_cdata, nrnc
end
-- Input: Column major
-- Output: Row major
local function fortran2mtx(mtx_cdata, rc)
  local n_row =rc[0]
  local n_col =rc[1]
  local mtx = {}
  for ir=1,n_row do
    local row = {}
    for ic=1,n_col do
      row[ic] = mtx_cdata[ic-1][ir-1]
    end
    mtx[ir] = row
  end
  return mtx
end

ffi.cdef[[
/* Printing as a simple funtion */
void ud01md_(
  int* M, int* N, int* L, int* NOUT,
  double* A, int* LDA, unsigned char* TEXT,
  int* INFO
);
]]
local function print_matrix(mtx, header)
  local A, rc = mtx2fortran(mtx)
  local l = ffi.new("int[1]", {math.min(rc[0], 5)})
  local text = ffi.new("unsigned char[72]")
  ffi.copy(text, header:sub(1, 72))
  -- stdout is 6 in FORTRAN
  local nout = ffi.new("int[1]", {6})
  -- lda is the number of rows:
  -- http://icl.cs.utk.edu/lapack-forum/viewtopic.php?f=2&t=346
  local lda = rc
  local info = ffi.new("int[1]", {1})
  slicot.ud01md_(rc+1, rc+0, l, nout, ffi.cast("double*", A), lda, text, info)
  return info[0]
end

ffi.cdef[[
void sb02mt_(
  /* Mode parameters */
  unsigned char* JOBG, unsigned char* JOBL, unsigned char* FACT, unsigned char* UPLO,
  /* Input/Output Parameters */
  int* N, int* M,
  double* A, int* LDA, double* B, int* LDB,
  double* Q, int* LDQ, double* R, int* LDR,
  double* L, int* LDL,
  int* IPIV, int* OUFACT,
  double* G, int* LDG,
  /* Workspace */
  int* IWORK, double* DWORK, int* LDWORK,
  /* Error Indicator */
  int* INFO
);
]]
ffi.cdef[[
void sb02md_(
  /* Mode parameters */
  unsigned char* DICO, unsigned char* HINV, unsigned char* UPLO, unsigned char* SCAL, unsigned char* SORT,
  /* Input/Output Parameters */
  int* N,
  double* A, int* LDA,
  double* G, int* LDG,
  double* Q, int* LDQ,
  double* RCOND, double* WR, double* WI,
  double* S, int* LDS,
  double* U, int* LDU,
  /* Workspace */
  int* IWORK, double* DWORK, int* LDWORK, _Bool* BWORK,
  /* Error Indicator */
  int* INFO
);
]]

-- https://en.wikipedia.org/wiki/Linear–quadratic_regulator#Infinite-horizon,_continuous-time_LQR
-- https://github.com/RobotLocomotion/drake/blob/a4d9661244fbbda0e817b784e56951268350e045/systems/controllers/linear_quadratic_regulator.cc
local function lqr(A0, B0, Q0, R0, L0)
  -- Solve CARE
  local A_fdata, A_rc = mtx2fortran(A0) -- n_states * n_states
  local B_fdata, B_rc = mtx2fortran(B0) -- n_states * n_inputs
  local n_states, n_inputs = B_rc[0], B_rc[1]
  local Q_fdata, Q_rc = mtx2fortran(Q0)
  local R_fdata, R_rc = mtx2fortran(R0)
  local L_fdata, L_rc = mtx2fortran(L0)
  --
  local ipiv = ffi.new("int[?]", n_inputs)
  local oufact = ffi.new("int[1]")
  -- Create the output matrix
  local G_fdata = ffi.new(string.format("double[%d][%d]", n_states, n_states))
  local G_rc = ffi.new("int[2]", {n_states, n_states})
  --
  local iwork = ffi.new("int[?]", 2 * n_inputs)
  local ldwork = ffi.new("int[1]",
    math.max(2, 3 * n_inputs, n_states * n_inputs))
  local dwork = ffi.new("double[?]", ldwork[0])
  --
  local info = ffi.new("int[1]")
  --
  local jobg = ffi.new("unsigned char[1]", 'G')
  local jobl = ffi.new("unsigned char[1]", 'Z')
  local fact = ffi.new("unsigned char[1]", 'N')
  local uplo = ffi.new("unsigned char[1]", 'U')
  -- Call the function
  slicot.sb02mt_(
    jobg, jobl, fact, uplo,
    --
    B_rc + 0, B_rc + 1,
    ffi.cast('double*', A_fdata), A_rc, -- A is not referenced when jobl is Z
    ffi.cast('double*', B_fdata), B_rc,
    --
    ffi.cast('double*', Q_fdata), Q_rc,
    ffi.cast('double*', R_fdata), R_rc,
    --
    ffi.cast('double*', L_fdata), L_rc,
    --
    ipiv, oufact,
    ffi.cast('double*', G_fdata), G_rc,
    --
    iwork, dwork, ldwork,
    --
    info
  )
  local ret = info[0]
  if ret ~=0 then
    return false, "Bad prep"
  end

  local dico = ffi.new("unsigned char[1]", 'C') -- Discrete/Continuous
  local hinv = ffi.new("unsigned char[1]", 'D')
  local scaling = ffi.new("unsigned char[1]", 'G')
  local sorting = ffi.new("unsigned char[1]", 'S')
  --
  iwork = ffi.new("int[?]", 2 * n_states)
  ldwork = ffi.new("int[1]", 6 * n_states)
  dwork = ffi.new("double[?]", ldwork[0])
  local bwork = ffi.new("_Bool[?]", 2 * n_states)
  --
  local rcond = ffi.new("double[1]")
  local wr = ffi.new("double[?]", 2 * n_states)
  local wi = ffi.new("double[?]", 2 * n_states)
  --
  local S_fdata = ffi.new(string.format("double[%d][%d]", 2*n_states, 2*n_states))
  local S_rc = ffi.new("int[2]", {2 * n_states, 2 * n_states})
  --
  local U_fdata = ffi.new(string.format("double[%d][%d]", 2*n_states, 2*n_states))
  local U_rc = ffi.new("int[2]", {2 * n_states, 2 * n_states})
  --
  slicot.sb02md_(
    dico, hinv, uplo, scaling, sorting,
    --
    A_rc,
    ffi.cast('double*', A_fdata), A_rc, -- Inverted on exit
    ffi.cast('double*', G_fdata), G_rc,
    ffi.cast('double*', Q_fdata), Q_rc, -- Solution, X, on exit
    --
    rcond, wr, wi,
    ffi.cast('double*', S_fdata), S_rc,
    ffi.cast('double*', U_fdata), U_rc,
    --
    iwork, dwork, ldwork, bwork,
    --
    info
  )
  ret = info[0]
  if ret ~=0 then
    return false, "Bad Riccati"
  end
  local P = fortran2mtx(Q_fdata, Q_rc)
  -- print("solve_pd", lapack.solve_pd)
  local K, err = lapack.solve_pd(R0, matrix.transpose(B0) * matrix:new(P))
  if not K then return false, err end
  print("***")
  print(matrix:new(K))
  print("===")
  return P
end


local A, B, Q, R
-- https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathTracking/lqr_steer_control/lqr_steer_control.py#L67
-- https://github.com/python-control/python-control/blob/master/control/statefbk.py
-- https://en.wikipedia.org/wiki/Linear–quadratic–Gaussian_control#Discrete_time
-- https://en.wikipedia.org/wiki/Linear–quadratic_regulator
local use_vehicle = true
if use_vehicle then
  local dt = 0.1 -- 10 Hz
  local v = 0.25 -- Vehicle velocity in meters per second
  local L = 0.35 -- Wheelbase
  A = matrix:new(4, 4)
  A[1][1] = 1
  A[1][2] = dt
  A[2][3] = v
  A[3][3] = 1
  A[3][4] = dt
  B = matrix:new(4, 1)
  B[4][1] = v / L
  Q = matrix:new(4, 'I')
  R = matrix:new(1, 'I')
elseif use_dense_dummy then
-- https://github.com/python-control/python-control/blob/7bed77f2526dc5b9fde4870e035b7a64a3e81448/control/tests/matlab_test.py
  A = matrix:new{{1, -2,}, {3, -4}}
  B = matrix:new{{5,}, {7}}
  Q = matrix:new(2, 'I')
  R = matrix:new(1, 'I')
else
-- https://www.mathworks.com/help/control/ref/care.html;jsessionid=9555ecabe363bd3abaa892cd5bc1
  A = matrix:new{{-3, 2,}, {1, 1}}
  B = matrix:new{{0}, {1}}
  local C = matrix{{1, -1}}
  Q = C:transpose() * C
  R = matrix:new{{3}}
end

print("== A ==")
print(A)
print("== B ==")
print(B)
print("== Q ==")
print(Q)
print("== R ==")
print(R)

local nstates = #B -- States: 4 (x, y, yaw, velocity)
local ninputs = #B[1] -- Inputs: 1

-- Cross-weighting matrix
-- For now - use *no* cross weight (i.e. zero)
local N = matrix:new(nstates, ninputs)
assert(nstates==#N, "Bad N")
assert(ninputs==#N[1], "Bad N")

assert(nstates==#A, "Bad A")
assert(nstates==#A[1], "Bad A")

-- Outputs?

assert(nstates==#Q, "Bad Q")
assert(nstates==#Q[1], "Bad Q")

assert(ninputs==#R, "Bad R")
assert(ninputs==#R[1], "Bad R")



local X = assert(lqr(A, B, Q, R, N))
print(matrix:new(X))
#!/usr/bin/env luajit

local lib = {}
-- TODO: https://github.com/flame/blis/
-- TODO: https://github.com/flame/libflame/
-- TODO: https://www.edx.org/course/linear-algebra-foundations-to-frontiers
local ffi = require'ffi'
local sformat = require'string'.format
-- local matrix = require'matrix'

local function transpose_mtx(mtx)
  local n_rows = #mtx
  local n_cols = #mtx[1]
  local xtm = {}
  for ic=1,n_cols do
    local xtm_row = {}
    for ir=1,n_rows do
      xtm_row[ir] = mtx[ir][ic]
    end
    xtm[ic] = xtm_row
  end
  return xtm
end
-- Transpose cdata into fdata and vice versa
local function transpose_data(data, rc)
  local n_rows = rc[0]
  local n_cols = rc[1]
  local atad = ffi.new(sformat("double[%d][%d]", n_cols, n_rows))
  for ir=0,n_rows-1 do
    for ic=0,n_cols-1 do
      atad[ic][ir] = data[ir][ic]
    end
  end
  return atad, ffi.new("int[2]", {n_cols, n_rows})
end

-- Input: Row major
-- Output: Row major
local function mtx2cdata(mtx)
  local n_rows = #mtx
  local n_cols = #mtx[1]
  local mtx_cdata = ffi.new(sformat("double[%d][%d]", n_rows, n_cols), mtx)
  return mtx_cdata, ffi.new("int[2]", {n_rows, n_cols})
end
-- Input: Row major
-- Output: Row major
local function cdata2mtx(mtx_cdata, rc)
  local n_rows = rc[0]
  local n_cols = rc[1]
  local mtx = {}
  for ir=1,n_rows do
    local row = {}
    for ic=1,n_cols do
      row[ic] = mtx_cdata[ir-1][ic-1]
    end
    mtx[ir] = row
  end
  return mtx
end
-- Input: Row major
-- Output: Column major
local function mtx2fortran(mtx)
  local n_rows = #mtx
  local n_cols = #mtx[1]
  local mtx_cdata = ffi.new(sformat("double[%d][%d]", n_cols, n_rows))
  for ir, row in ipairs(mtx) do
    for ic, v in ipairs(row) do
      mtx_cdata[ic-1][ir-1] = v
    end
  end
  return mtx_cdata, ffi.new("int[2]", {n_rows, n_cols})
end
-- Input: Column major
-- Output: Row major
local function fortran2mtx(mtx_cdata, rc)
  local n_row = rc[0]
  local n_col = rc[1]
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
typedef int lapack_int;
lapack_int LAPACKE_dsyev(int matrix_layout, char jobz, char uplo,
                         lapack_int n, double* a,
                         lapack_int lda, double* w);
lapack_int LAPACKE_dpstrf(int matrix_layout, char uplo,
                          lapack_int n, double* a,
                          lapack_int lda, lapack_int* piv, lapack_int* rank,
                          double tol);
lapack_int LAPACKE_dpotrf(int matrix_layout, char uplo,
                          lapack_int n, double* a,
                          lapack_int lda);
/* Linear solver for positive definite matrices */
lapack_int LAPACKE_dposv(int matrix_layout, char uplo, lapack_int n,
                         lapack_int nrhs, double* a, lapack_int lda, double* b,
                         lapack_int ldb );
]]
local lapack = ffi.load('/usr/local/opt/openblas/lib/liblapack.dylib')
local LAPACK_ROW_MAJOR = 101
local LAPACK_COL_MAJOR = 102
local LAPACK_UPPER = require'string'.byte'U'
local LAPACK_LOWER = require'string'.byte'L'

-- Solve positive definite matrix equation
local function solve_pd(A_mtx, B_mtx)
  local A_fdata, A_rc = mtx2fortran(A_mtx)
  local B_fdata, B_rc = mtx2fortran(B_mtx)
  local ret = lapack.LAPACKE_dposv(
    LAPACK_COL_MAJOR, LAPACK_UPPER,
    B_rc[0], B_rc[1],
    ffi.cast('double*', A_fdata), A_rc[0],
    ffi.cast('double*', B_fdata), B_rc[0]
  )
  if ret < 0 then
    return false, "Invalid"
  elseif ret > 0 then
    return false, string.format("Failed: %d", ret)
  end
  return fortran2mtx(B_fdata, B_rc)
end
lib.solve_pd = solve_pd

-- Compute the eigenvalues and eigenvectors
local function eigs(mtx)
  local mtx_cdata, rc = mtx2cdata(mtx)
  local eigvals = ffi.new("double[?]", rc[0])
  local ret = lapack.LAPACKE_dsyev(
    LAPACK_ROW_MAJOR, string.byte'V', LAPACK_LOWER,--LAPACK_UPPER,
    rc[0], ffi.cast('double*', mtx_cdata),
    rc[0], eigvals
  )
  if ret<0 then
    return false, "Invalid"
  elseif ret>0 then
    return false, string.format("Failed: %d", ret)
  end
  local evals = {}
  for j=1,rc[0] do
    evals[j] = eigvals[j-1]
  end
  local emtx = cdata2mtx(mtx_cdata, rc)
  return evals, emtx
end
lib.eigs = eigs

-- Cholesky factorization using lower triangular matrices
local function chol(mtx)
  local mtx_cdata, rc = mtx2cdata(mtx)
  local ret = lapack.LAPACKE_dpotrf(
    LAPACK_ROW_MAJOR, LAPACK_LOWER,
    rc[0], ffi.cast('double*', mtx_cdata),
    rc[0]
  )
  if ret<0 then
    return false, "Invalid"
  elseif ret>0 then
    return false, string.format("Failed: %d", ret)
  end
  -- Zero the non-triangular
  local lower_mtx = cdata2mtx(mtx_cdata, rc)
  for j=1, rc[1] do
    for i=j+1, rc[0] do
      lower_mtx[j][i] = 0
    end
  end
  return lower_mtx
end
lib.chol = chol

ffi.cdef[[
/* FORTRAN matrix multiply interface */
void dsymm_(
  unsigned char* SIDE,
  unsigned char*   UPLO,
  int* M,
  int* N,
  double * ALPHA,
  double * A, int* LDA,
  double * B, int* LDB,
  double * BETA,
  double * C, int* LDC
);
]]
local openblas = ffi.load('/usr/local/opt/openblas/lib/libopenblas.dylib')


local has_slicot, slicot = pcall(ffi.load, "slicot")
if has_slicot then
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
    /* NOTE: BWORK LOGICAL is *NOT* _Bool/bool, but int, internally
    http://www.yolinux.com/TUTORIALS/LinuxTutorialMixingFortranAndC.html
    https://gcc.gnu.org/onlinedocs/gfortran/Internal-representation-of-LOGICAL-variables.html
    */
    int* IWORK, double* DWORK, int* LDWORK, int* BWORK,
    /* Error Indicator */
    int* INFO
  );
  ]]
end
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
  local bwork = ffi.new("int[?]", 2 * n_states)
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

  B_fdata, B_rc = mtx2fortran(B0)
  local side = ffi.new("unsigned char[1]", 'L')
  local alpha = ffi.new("double[1]", {1})
  local beta = ffi.new("double[1]", {0})
  local ptb_rc = ffi.new("int[2]", {n_states, n_inputs})
  local ptb_fdata = ffi.new(string.format("double[%d][%d]", ptb_rc[1], ptb_rc[0]))
  openblas.dsymm_(
    side,
    uplo,
    ptb_rc + 0,
    ptb_rc + 1,
    alpha,
    ffi.cast("double*", Q_fdata), Q_rc,
    ffi.cast("double*", B_fdata), B_rc,
    beta,
    ffi.cast("double*", ptb_fdata), ptb_rc
  )
  -- local K, err = solve_pd(R0, matrix.transpose(B0) * matrix:new(fortran2mtx(Q_fdata, Q_rc)))
  -- local btp = matrix.transpose(fortran2mtx(ptb_fdata, ptb_rc))
  local btp = transpose_mtx(fortran2mtx(ptb_fdata, ptb_rc))
  local K, err = solve_pd(R0, btp)
  if not K then
    return false, err
  end
  -- Optimal feedback: u = -K * x
  return K
end
lib.lqr = has_slicot and lqr

return lib

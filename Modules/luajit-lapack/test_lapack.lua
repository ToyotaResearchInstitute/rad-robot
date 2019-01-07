#!/usr/bin/env luajit

local lapack = require'lapack'


local mtx_eig = {{1,0,0},{0,2,0},{0,0,3}}
local evals, evecs = lapack.eigs(mtx_eig)
print("Eigenvalues", table.concat(evals, ', '))
print("Eigenvectors")
for i=1,#evecs do
  print(table.concat(evecs[i], ", "))
end

print()
print("*****")
print()

-- https://en.wikipedia.org/wiki/Cholesky_decomposition#Example
local mtx_chol = {
{4,12,-16},
{12,37,-43},
{-16,-43,98}
}
print("Chol Input")
for i=1,#mtx_chol do
  print(table.concat(mtx_chol[i], ", "))
end
local lower = lapack.chol(mtx_chol)
print("Lower")
for i=1,#lower do
  print(table.concat(lower[i], ", "))
end

local has_matrix, matrix = pcall(require, 'matrix')

if has_matrix then
  lower = matrix:new(lower)
  print("Reconstruction")
  local recon = lower * lower:transpose()
  for i=1,#recon do
    print(table.concat(recon[i], ", "))
  end
end

if lapack.lqr then
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

  assert(nstates==#A, "Bad A rows")
  assert(nstates==#A[1], "Bad A cols")
  assert(nstates==#Q, "Bad Q rows")
  assert(nstates==#Q[1], "Bad Q cols")
  assert(ninputs==#R, "Bad R rows")
  assert(ninputs==#R[1], "Bad R cols")

  local X = assert(lapack.lqr(A, B, Q, R, N))
  print("Optimal Feedback")
  print(matrix:new(X))
end
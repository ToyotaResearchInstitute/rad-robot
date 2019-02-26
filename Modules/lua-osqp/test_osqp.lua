#!/usr/bin/env luajit
local osqp = require'osqp'
local demo = osqp.new()
print("MPC", demo)

local P = {
  4, 1,
  1, 2,
}
local n = 2
local A = {
  1,1,
  1,0,
  0,1
}
local m = 3

local l = {1,0,0}
local u = {1, 0.7, 0.7}
local q = {1, 1,}

assert(demo:set_problem{
  n = n,
  m = m,
  P = P,
  A = A,
  l = l,
  u = u,
  q = q
})
print("MPC", demo)

local solution = assert(demo:solve())
print("Solution", type(solution))
for k, v in pairs(solution) do
  print(k, v)
end

-- MPC
local has_matrix, matrix = pcall(require, 'matrix')
local mpc = osqp.new()
-- Number of timesteps
-- Say 5 seconds at 10Hz
local nSteps = 50
-- Steering and Acceleration
local nInputs = 2
-- x, y, theta, dtheta
local nState = 4
-- Form the Costs
local nVariables = nSteps * (nState + nInputs)
local P = matrix:new(nVariables, nVariables)

for t=1,nSteps-1 do
  local Q = matrix:new(nState, "I")
  local R = matrix:new(nInputs, "I")
end

matrix.flatten(P)
assert(demo:set_problem{
  n = nVariables,
  P = P,
})
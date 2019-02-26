#!/usr/bin/env luajit
local kalman = require 'kalman'
local vector = require'vector'
local unpack = unpack or require'table'.unpack
-- set the seed
math.randomseed(1234)

if TEST_LINEAR then
  -- 4 dimensional kalman filter: {px, py vx vy}
  local sz_state = 4
  local nIter = 1e3

  -- Initialize the filter
  local kalman1 = kalman.rolling({
    nDim = sz_state,
    state = {0,0,1,0}
  })
  -- Control input
  local sz_input = 0
  local u_k_input = sz_input > 0 and vector.zeros(sz_input)
  -- Set the observations
  local sz_obs = 2
  local obs1 = vector.zeros(sz_obs)

  local x, P = kalman1:get_state()

  -- Print the initial state
  print("Initial state:", table.concat(x, ", "))

  -- Begin the test loop
  for i=1,nIter do
    print()
    print('Iteration',i)

    -- Perform prediction
    kalman1:predict(u_k_input)
    local x_pred, P_pred = kalman1:get_prior()
    print("Prior:", table.concat(x_pred, ", "))
    print(P_pred)

    -- Make an observation
    obs1[1] = i + .2*(math.random()-.5)
    for p=2,sz_obs-1 do
      obs1[p] = i/p + 1/(5*p)*(math.random()-.5)
    end
    print("Observation:", table.concat(obs1, ", "))

    -- Perform correction
    kalman1:correct( obs1 )
    x, P = kalman1:get_state()

    print("Corrected:", table.concat(x, ", "))
    print(P)

    print("Kalman gain")
    print(kalman1.K_k)
    print(kalman1.A)

  end

  x,P = kalman1:get_state()
  print("Final state:", table.concat(x, ", "))
end

print()
print()
print("********** Unscented Kalman Filter **********")

-- Test the UKF
local lapack = require'lapack'
local matrix = require'matrix'
local quaternion = require'quaternion'
local ukf = assert(kalman.ukf())
local dt = 0.01

local function pukf()
  print(string.format("RPY: %+.5f, %+.5f, %+.5f",
    unpack(180/math.pi * vector.new(quaternion.to_rpy(ukf.orientation)))))
  -- print(string.format("Covariance: %+.2f, %+.2f, %+.2f",
  --     unpack(180/math.pi * vector.new(matrix.diag(ukf.orientation_cov)))))
  print("Covariance")
  print(180/math.pi * ukf.orientation_cov)
  -- print(ukf.orientation_cov)

  -- local evals, evecs = lapack.eigs(ukf.orientation_cov)
  -- print("evals", vector.new(evals))
  -- print("evecs")
  -- print(matrix:new(evecs))
end

pukf()
for i=1,2 do
  print(string.format("\n== Iteration %d | %f sec", i, i * dt))
  local gyro = vector.zeros(3)
  gyro = gyro + 1 * (vector.rand(3) - 0.5) * 0.125 * math.pi/180
  -- print(string.format("-- Gyro %+.2f, %+.2f, %+.2f", unpack(gyro * 180/math.pi)))
  local accel = vector.new{0,0,1}
  local pertub_accel = 2 * (vector.rand(3) - 0.5) * 0.01
  accel = vector.unit(accel + pertub_accel)
  print(string.format("-- Accel %+.5f, %+.5f, %+.5f", unpack(accel)))
  --
  assert(ukf:motion_gyro(gyro, dt))
  pukf()
  assert(ukf:correct_gravity(accel))
  pukf()
end
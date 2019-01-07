-- Torch/Lua Unscented Kalman Filter
-- (c) 2017, 2018 Stephen McGill

local lib = {}

-- NOTE: Populates the global namespace
local has_torch, torch = pcall(require, 'torch')
if not has_torch then
  io.stderr:write"No torch support!\n"
end

local lapack = require'lapack'
local matrix = require'matrix'
local quaternion = require'quaternion'
local vector = require'vector'
local vnew = vector.new
local vcopy = vector.copy
local vdeg = vector.deg
local vmean = vector.mean
local vrad = vector.rad
local vsub = vector.sub
local vnorm = vector.norm
local sqrt = require'math'.sqrt
local tinsert = require'table'.insert
local USE_ITERATIVE_QMEAN = true

-- Gravity
local vUp = {0, 0, 1}
-- Measurement noise covariance
-- local cov_gravity_measurement = vrad{10, 10, 10}
local cov_gravity_measurement = vrad{2, 2, 2}
-- Process noise covariance
local cov_gyro_process = vrad{0.25, 0.25, 0.25}
if has_torch then
  cov_gravity_measurement = torch.diag(torch.Tensor(cov_gravity_measurement):pow(2))
  cov_gyro_process = torch.diag(torch.Tensor(cov_gyro_process):pow(2))
else
  cov_gravity_measurement = matrix.diag(cov_gravity_measurement)
  cov_gravity_measurement = cov_gravity_measurement * cov_gravity_measurement
  cov_gyro_process = matrix.diag(cov_gyro_process)
  cov_gyro_process = cov_gyro_process * cov_gyro_process
end

local function get_weights(n)

  local alpha = 1
  local beta = 2
  local kappa = 0.75

  -- No use of the center
  -- beta, kappa = 0, 0

  local lambda = (alpha ^ 2) * (n + kappa) - n
  local gamma = sqrt(n + lambda)

  -- Common weight for all sigmas except center sigma
  local w_0 = lambda / (n + lambda)
  local w_i = 1 / (2 * (n + lambda))

  local w_mean = vnew{w_0}
  local w_cov = vnew{w_0}
  for _ = 1, 2*n do
    tinsert(w_mean, w_i)
    tinsert(w_cov, w_i)
  end
  w_cov[1] = w_cov[1] + (1 + beta - alpha^2)

  return gamma, w_mean, w_cov

end

local function sigma_points(covariance, gamma)
  local columns = {}
  if has_torch then
    local factorized = torch.pstrf(covariance, 'L')
    factorized:mul(gamma)
    local n_col = factorized:size(2)
    for i_col=1,n_col do
      tinsert(columns, factorized:select(2, i_col):totable())
    end
  else
    -- local factorized = matrix.sqrt(covariance)
    local factorized, rank = lapack.chol(covariance)
    if not factorized then return false, rank end
    factorized = matrix:new(factorized)
    factorized = gamma * factorized
    columns = matrix.transpose(factorized)
  end
  local n_col = #columns

  local chi = {quaternion.unit()}
  for i_col=1,n_col do
    local col = vnew(columns[i_col])
    tinsert(chi, quaternion.from_rotation_vector(col))
    tinsert(chi, quaternion.from_rotation_vector(-col))
  end
  return chi
end

local function from_gyro(w, dt)
  return quaternion.from_angle_axis(dt*vnorm(w), w)
end

local function just_gyro(self, measured_gyro, dt)
  local dq_body = from_gyro(measured_gyro, dt)
  -- Rotation by our current orientation
  -- followed by rotation by the gyro in body frame
  self.orientation = self.orientation * dq_body
  return self
end

local function motion_gyro(self, measured_gyro, dt)

  -- Too many checks
  --[[
  if type(measured_gyro)~='table' then
    return false, "Bad measurement"
  end
  if type(dt)~='number' then
    return false, "Bad dt"
  end
  --]]

  -- Body frame rotation
  local dq_body = from_gyro(measured_gyro, dt)

  -- Add the noise, first, per Kraft's Quaternion UKF
  -- TODO: Project noise to world frame should be better
  self.orientation_cov = self.orientation_cov + dt^2 * cov_gyro_process
  local chi, err = sigma_points(self.orientation_cov, self.gamma)
  if not chi then return false, err end
  -- Sigma points are in the *absolute* global world frame
  for i, dq in ipairs(chi) do chi[i] = dq * self.orientation end

  -- Now, move each sigma point via the process
  -- Rotation by our current orientation
  -- followed by rotation by the gyro in body frame
  for k, q in ipairs(chi) do chi[k] = q * dq_body end

  -- Find the mean. The iterating mean gives chiDiff
  -- in the form of its error metric
  local qMean, chiDiff
  if USE_ITERATIVE_QMEAN then
    qMean, chiDiff = quaternion.mean(chi, self.w_mean)
    chiDiff = nil
  else
    qMean = quaternion.mean2(chi, self.w_mean)
  end

  if not chiDiff then
    -- Find the variance in the *absolute* global world frame
    chiDiff = {}
    local qMeanInv = quaternion.conjugate(qMean)
    for k, q in ipairs(chi) do
      chiDiff[k] = quaternion.log(q * qMeanInv)
    end
  end

  local Pxx = has_torch and torch.zeros(3, 3) or matrix:new(3, 3)
  -- Use with Lua matrix module
  for i, chiD in ipairs(chiDiff) do
    chiD = has_torch and torch.Tensor(chiD) or matrix:new{chiD}
    local outer = matrix.transpose(chiD) * chiD
    Pxx = Pxx + self.w_cov[i] * outer
  end

  self.orientation = qMean
  self.orientation_cov = Pxx
  self.chi = chi
  self.chiDiff = chiDiff

  return self
end

local function correct_gravity(self, measured_accel)
  if type(measured_accel)~='table' then
    return false, "Bad input"
  end

  local chi = self.chi
  local chiDiff = self.chiDiff
  if not chi then
    chi = sigma_points(self.orientation_cov, self.gamma)
    chiDiff = nil
    -- Sigma points are in the *absolute* global world frame
    for i, dq in ipairs(chi) do chi[i] = dq * self.orientation end
  end
  if not chiDiff then
    local qMeanInv = quaternion.conjugate(self.orientation)
    chiDiff = {}
    for k, q in ipairs(chi) do
      chiDiff[k] = quaternion.log(q * qMeanInv)
    end
  end

  -- Find the expected gravity vectors for each sigma point
  -- Find z in the world frame
  local zExpected = {}
  for k, q in ipairs(chi) do
    zExpected[k] = quaternion.rotate(q, measured_accel)
    print("zM", vnew(measured_accel))
    print("qS["..k.."]", vdeg(quaternion.to_rpy(q)))
    print("zE["..k.."]", vnew(zExpected[k]))
  end
  local zMean = vmean(zExpected, self.w_mean)
  -- zMean = vector.unit(zMean)
  print("zMean", zMean)

  local zDiff = {}
  for i, zE in ipairs(zExpected) do zDiff[i] = vsub(zE, zMean) end

  local Pzz
  local Pxz
  if has_torch then
    Pzz = torch.zeros(3, 3)
    Pxz = torch.zeros(3, 3)
    for i, zD in ipairs(zDiff) do
      zD = torch.Tensor(zD)
      local chiD = torch.Tensor(chiDiff[i])
      Pzz:add(self.w_cov[i], torch.ger(zD, zD))
      Pxz:add(self.w_cov[i], torch.ger(chiD, zD))
    end
  else
    Pzz = matrix:new(3, 3)
    Pxz = matrix:new(3, 3)
    for i, zD in ipairs(zDiff) do
      zD = matrix:new{zD}
      local outer = matrix.mul(matrix.transpose(zD), zD)
      Pzz = Pzz + outer * self.w_cov[i]
      local chiD = matrix:new{chiDiff[i]}
      outer = matrix.mul(matrix.transpose(zD), chiD)
      Pxz = Pxz + outer * self.w_cov[i]
    end
  end

  -- Form Pvv
  local Pvv = Pzz + cov_gravity_measurement

  -- Kalman gain calculation
  local K = Pxz * (has_torch and torch.inverse(Pvv) or matrix.invert(Pvv))
  -- Posterior
  local innovation = vsub(zMean, vUp)
  -- local innovation = vsub(vUp, zMean)
  -- print("Innovation", innovation)
  local dOrientation = vector.mv(has_torch and K:totable() or K, innovation)

  -- The correction is done in the world frame
  local dqCorrect = quaternion.from_rotation_vector(dOrientation)

  print("Correction", vnew(quaternion.to_rpy(dqCorrect)) * 180 / math.pi)

  self.orientation = dqCorrect * self.orientation

  -- local cov_correct = K * Pvv * K:transpose()
  local cov_correct = Pxz * K:transpose()
  self.orientation_cov = self.orientation_cov - cov_correct

  -- Check the covariance
  local evals, evecs = lapack.eigs(self.orientation_cov)
  local needs_fix = false
  for i=1,#evals do
    local v = evals[i]
    if v<=0 then
      evals[i] = math.rad(1)^2
      needs_fix = true
    elseif v > 2 * math.pi ^ 3 / 3 then
      print("Big uncertainty")
      -- evals[i] = 2 * math.pi ^ 3 / 3
      -- needs_fix = true
    end
  end
  if needs_fix then
    evecs = matrix:new(evecs)
    self.orientation_cov = evecs * matrix.diag(evals) * matrix.invert(evecs)
    -- local eps = matrix:new(3, "I") * math.rad(0.5) ^ 2
    -- self.orientation_cov = self.orientation_cov + eps
    -- self.orientation_cov = (self.orientation_cov + self.orientation_cov:transpose()) / 2
  end
  -- Global Z is uncertain without yaw correction
  --[[
  self.orientation_cov[1][3] = 0
  self.orientation_cov[2][3] = 0
  self.orientation_cov[3][1] = 0
  self.orientation_cov[3][2] = 0
  self.orientation_cov[3][3] = math.rad(5) ^ 2
  --]]

  -- Cached sigma points are now invalidated after a covariance update
  self.chi = nil
  self.chiDiff = nil

  return self
end

function lib.ukf(parameters)
  if type(parameters)~='table' then
    parameters = {}
  end
  local orientation_cov
  if type(parameters.orientation_cov)=='table' then
    if type(parameters.orientation_cov[1])=='number' then
      orientation_cov = matrix.diag(parameters.orientation_cov)
      orientation_cov = orientation_cov * orientation_cov
    else
      orientation_cov = matrix:new(parameters.orientation_cov)
    end
  else
    orientation_cov = vrad{2, 2, 2}
    orientation_cov = matrix.diag(orientation_cov)
    orientation_cov = orientation_cov * orientation_cov
  end
  if has_torch then
    orientation_cov = torch.Tensor(orientation_cov)
  end

  local position_cov
  if type(parameters.position_cov)=='table' then
    if type(parameters.position_cov[1])=='number' then
      position_cov = matrix.diag(parameters.position_cov)
      position_cov = position_cov * position_cov
    else
      position_cov = matrix:new(parameters.position_cov)
    end
  else
    position_cov = vnew{
      0.05,0.05,0.05,
      0.01,0.01,0.01
    }
    position_cov = matrix.diag(position_cov)
    position_cov = position_cov * position_cov
  end
  if has_torch then
    position_cov = torch.Tensor(position_cov)
  end

  -- State:
  -- Position (world to body): x, y, z
  -- Velocity: vx, vy, vz
  -- quaternion (world to body, rotatation valid on ) (4 elements)
  -- rotational velocity (3 elements)
  local sz_cov = 3
  local gamma, w_mean, w_cov = get_weights(sz_cov)

  local position0 = vnew(6)
  if type(parameters.position0)=='table' then
    position0 = vcopy(parameters.position0)
  end
  local orientation0 = quaternion.unit()
  if type(parameters.orientation0)=='table' then
    orientation0 = quaternion.unit(parameters.orientation0)
  end

  return {
    just_gyro = just_gyro,
    motion_gyro = motion_gyro,
    correct_gravity = correct_gravity,
    -- Position in meters
    -- Velocity: meters per second
    position = position0,
    position_cov = position_cov,
    orientation = orientation0,
    orientation_cov = orientation_cov,
    -- Weights
    gamma = gamma,
    w_mean = w_mean,
    w_cov = w_cov,
    n = sz_cov
  }
end

-- Accessor Methods
local function get_prior(self)
  return self.x_k_minus, self.P_k_minus
end
local function get_state(self)
  return self.x_k, self.P_k
end

-- Simple (i.e. mallocing memory each time) way
local function predict_slow(self, u_k)
  self.x_k_minus = vector.mv(self.A, self.x_k_minus)
  -- Check if we have a control input
  if u_k then
    self.x_k_minus = self.x_k_minus + vector.mv(self.B, u_k)
  end
  self.P_k_minus = self.A * self.P_k_minus * self.A:transpose() + self.Q
  return self
end

-- Simple (i.e. malloc'ing memory each time) way
local function correct_slow(self, z_k)
  local cov_innovation = self.H * self.P_k_minus * self.H:transpose() + self.R
  -- Find the optimal Kalman gain
  self.K_k = self.P_k_minus * self.H:transpose() * matrix.invert(cov_innovation)
  -- self.P_k = (self.I - self.K_k * self.H) * self.P_k_minus
  -- self.P_k = (self.I - self.K_k * self.H) * self.P_k_minus * (self.I - self.K_k * self.H):transpose() + self.K_k * self.R * self.K_k:transpose()

  local state_innovation = z_k - vector.mv(self.H, self.x_k_minus)
  -- Updated (a posteriori) state
  self.x_k = self.x_k_minus + vector.mv(self.K_k, state_innovation)
  -- Post fit residual
  local residual = z_k - vector.mv(self.H, self.x_k)
  -- Save the prior for next time
  self.x_k_minus = self.x_k
  self.P_k_minus = self.P_k
  return self
end

local function extended(parameters)
  if type(parameters)~='table' then
    parameters = {}
  end
  local obj = {}
  if type(parameters.system) then

  end
  return obj
end
lib.extended = extended

-- Generic filter with default initialization
local function linear(parameters)
  parameters = parameters or {}
  -- One dimensional is default
  local nDim = tonumber(parameters.nDim) or 1
  local state0
  if type(parameters.state)=='table' then
    if #parameters.state~=nDim then
      return false, "Bad dimension sizing"
    end
    state0 = vector.copy(parameters.state)
  else
    state0 = vector.zeros(nDim)
  end
  local uncertainty0
  if type(parameters.uncertainty)=='table' then
    if #parameters.uncertainty~=nDim then
      return false, "Bad dimension sizing"
    end
    uncertainty0 = matrix:copy(parameters.uncertainty)
  else
    uncertainty0 = matrix:new(nDim, "I")
  end
  -- return initialize_temporary_variables( initialize_filter( nDim ) )
  local obj = {
    sz_state = nDim,
    I = matrix:new(nDim, "I"),
    -- Process
    A = matrix:new(nDim, "I"), -- State process w/o input
    B = matrix:new(nDim, "I"), -- Control input to state effect
    Q = matrix:new(nDim, "I"), -- Additive uncertainty
    -- Measurement
    R = matrix:new(nDim, "I"), -- Measurement uncertainty
    H = matrix:new(nDim, "I"),
    -- State
    P_k = uncertainty0,
    x_k = state0,
    -- Prior
    P_k_minus = uncertainty0,
    x_k_minus = state0,
    -- Methods
    predict = predict_slow,
    correct = correct_slow,
    get_prior = get_prior,
    get_state = get_state,
  }
  return obj
end
lib.linear = linear

function lib.rolling(parameters)
  parameters = parameters or {}
  local filter = linear(parameters)
  -- Position stays the same
  -- Velocity stays the same
  local dynamics = matrix:new(filter.sz_state, "I")
  local nCoords = filter.sz_state / 2

  local DECAY = 0.9
  for d=1, nCoords do
    -- Position needs additive state of the velocity
    dynamics[d][d + nCoords] = 1 -- TODO: Needs delta_time
    -- Velocity decays over time due to friction
    dynamics[d + nCoords][d + nCoords] = DECAY
  end
  filter.A = dynamics
  -----------------
  -- Modify the Measurement update
  -----------------
  -- We only measure the state positions, not velocities
  filter.R = matrix:new(nCoords, "I")
  filter.H = matrix:new(nCoords, filter.sz_state)
  for d=1, nCoords do filter.H[d][d] = 1 end

  return filter
end

return lib

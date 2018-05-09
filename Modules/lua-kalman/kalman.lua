-- Torch/Lua Unscented Kalman Filter
-- (c) 2017, 2018 Stephen McGill

local lib = {}

-- NOTE: Populates the global namespace
local has_torch, torch = pcall(require, 'torch')
if not has_torch then
  print("No torch support!")
  print(package.path)
end

local quaternion = require'quaternion'
local vector = require'vector'
local vnew = vector.new
local vmean = vector.mean
local vsub = vector.sub
local vunit = vector.unit
local vnorm = vector.norm
local sqrt = require'math'.sqrt
local PI = require'math'.pi
local tinsert = require'table'.insert
local DEG_TO_RAD = PI / 180
local RAD_TO_DEG = 180 / PI

-- Gravity
local vUp = {0, 0, 1}

-- Measurement noise covariance
local cov_gravity_measurement = vnew{
  10 * DEG_TO_RAD, 10 * DEG_TO_RAD, 10 * DEG_TO_RAD
}
-- Process noise covariance
local cov_gyro_process = vnew{
  0.125 * DEG_TO_RAD, 0.125 * DEG_TO_RAD, 0.125 * DEG_TO_RAD
}
if has_torch then
  cov_gravity_measurement = torch.diag(torch.Tensor(cov_gravity_measurement):pow(2))
  cov_gyro_process = torch.diag(torch.Tensor(cov_gyro_process):pow(2))
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
  for i = 1, 2*n do
    tinsert(w_mean, w_i)
    tinsert(w_cov, w_i)
  end
  w_cov[1] = w_cov[1] + (1 + beta - alpha^2)

  return gamma, w_mean, w_cov

end

local function sigma_points(covariance, gamma)
  local factorized = torch.pstrf(covariance, 'U')
  factorized:mul(gamma)

  local chi = {quaternion.unit()}
  for i=1,factorized:size(2) do
    -- Grab a column
    local col = vnew(factorized:select(2, i):totable())
    --
    tinsert(chi, quaternion.from_rotation_vector(col))
    tinsert(chi, quaternion.from_rotation_vector(-col))
  end
  return chi
end

local function from_gyro(w, dt)
  return quaternion.from_angle_axis(dt*vnorm(w), w)
end

local function just_gyro(self, measured_gyro, dt, measured_accel)
  local dq_body = from_gyro(measured_gyro, dt)

  -- Rotation by our current orientation
  -- followed by rotation by the gyro in body frame
  self.orientation = self.orientation * dq_body

  --[[
  if measured_accel then
    -- From the body frame to the world frame
    local vFront = vnew(quaternion.rotate(self.orientation, {1,0,0}))
    print("gyro ~ front", vFront)

    measured_accel = vunit(measured_accel)
    local vUp_g = vnew(quaternion.rotate(self.orientation, measured_accel))
    print("gyro ~ up", vUp_g)
    local roll = math.atan2(
      measured_accel[2], math.sqrt(measured_accel[1]^2 + measured_accel[3]^2))
    local pitch = math.atan2(
      measured_accel[1], math.sqrt(measured_accel[2]^2 + measured_accel[3]^2))
    print("accel roll", roll * RAD_TO_DEG)
    print("accel pitch", pitch * RAD_TO_DEG)
  end
  local z = vnew(quaternion.inv_rotate(self.orientation, vUp))
  print("gyro ~ accel", z)
  --]]
  return self
end

local function motion_gyro(self, measured_gyro, dt)
  -- local gyro_motion = vnew(measured_gyro) * dt
  -- print("Gyro motion", gyro_motion * RAD_TO_DEG)

  -- Body frame rotation
  local dq_body = from_gyro(measured_gyro, dt)

  -- Add the noise, first, per Kraft's Quaternion UKF
  local chi = self.chi
  if not self.chi then
    -- TODO: Project to world frame should be better
    self.orientation_cov:add(dt^2, cov_gyro_process)
    chi = sigma_points(self.orientation_cov, self.gamma)
    -- Sigma points are in the *absolute* global world frame
    for i, dq in ipairs(chi) do chi[i] = dq * self.orientation end
  end

  -- Now, move each sigma point via the process
  -- Rotation by our current orientation
  -- followed by rotation by the gyro in body frame
  for k, q in ipairs(chi) do chi[k] = q * dq_body end

  -- Find the mean. The iterating mean gives chiDiff
  -- in the form of its error metric
  local qMean, chiDiff
  local USE_ITERATIVE_QMEAN = true
  if USE_ITERATIVE_QMEAN then
    qMean, chiDiff = quaternion.mean(chi, self.w_mean)
    -- chiDiff = nil
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

  local Pxx = torch.zeros(3, 3)
  for i, chiD in ipairs(chiDiff) do
    chiD = torch.Tensor(chiD)
    local outer = torch.ger(chiD, chiD)
    Pxx:add(self.w_cov[i], outer)
  end

  self.orientation = qMean
  self.orientation_cov = Pxx
  self.chi = chi
  self.chiDiff = chiDiff

  return self
end

local function correct_gravity(self, measured_accel)

  local chi = self.chi
  local chiDiff = self.chiDiff
  if not chi then
    chi = sigma_points(self.orientation_cov, self.gamma)
    -- Sigma points are in the *absolute* global world frame
    for i, dq in ipairs(chi) do chi[i] = dq * self.orientation end
    self.chi = chi
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
    -- print("q["..k.."]", RAD_TO_DEG * vnew(quaternion.to_rpy(q)))
    -- print("zExpected["..k.."]", vnew(zExpected[k]))
  end
  local zMean = vmean(zExpected, self.w_mean)

  local zDiff = {}
  for i, zE in ipairs(zExpected) do
    zDiff[i] = vsub(zE, zMean)
  end

  local Pzz = torch.zeros(3, 3)
  local Pxz = torch.zeros(3, 3)
  for i, zD in ipairs(zDiff) do
    zD = torch.Tensor(zD)
    local chiD = torch.Tensor(chiDiff[i])
    --
    Pzz:add(self.w_cov[i], torch.ger(zD, zD))
    Pxz:add(self.w_cov[i], torch.ger(chiD, zD))
  end

  -- Form Pvv
  local Pvv = Pzz + cov_gravity_measurement

  -- Kalman gain calculation
  local K = Pxz * torch.inverse(Pvv)
  -- Posterior
  local innovation = vsub(vUp, zMean)
  local dOrientation = K * torch.Tensor(innovation)
  -- local cov_correct = K * Pvv * K:t()
  local cov_correct = Pxz * K:t()

  dOrientation = vnew(dOrientation:totable())

  -- The correction is done in the world frame
  local dqCorr = quaternion.from_rotation_vector(dOrientation)

  self.orientation = dqCorr * self.orientation
  self.orientation_cov:add(-1, cov_correct)

  -- Cached sigma points are now invalidated after a covariance update
  self.chi = nil
  self.chiDiff = nil

  -- Small hacks
  --[[
  -- self.orientation_cov = (self.orientation_cov + self.orientation_cov:t()) / 2
  local eps = torch.eye(3) * math.pow(1 * DEG_TO_RAD, 2)
  self.orientation_cov:add(eps)
  print("orientation_cov updated")
  print(self.orientation_cov)
  --]]

  return self
end

function lib.ukf()
  local orientation_cov = torch.diag(torch.Tensor{
    2 * DEG_TO_RAD, 2 * DEG_TO_RAD, 2 * DEG_TO_RAD,
    }:pow(2))
  -- local orientation_factorized = torch.potrf(orientation_cov, 'L')
  -- State:
  -- Position (world to body): x, y, z
  -- Velocity: vx, vy, vz
  -- quaternion (world to body, rotatation valid on ) (4 elements)
  -- rotational velocity (3 elements)
  local sz_cov = 3
  local gamma, w_mean, w_cov = get_weights(sz_cov)

  return {
    just_gyro = just_gyro,
    motion_gyro = motion_gyro,
    correct_gravity = correct_gravity,
    -- Position in meters
    -- Velocity: meters per second
    position = {0,0,0, 0,0,0},
    position_cov = torch.diag(torch.Tensor{
      0.05,0.05,0.05,
      0.01,0.01,0.01
      }:pow(2)),
    orientation = quaternion.unit(),
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

-- Form a state estimate prior based on the process and input
-- Complicated (i.e. fast in-memory) way
local function predict(self, u_k)
  -- Evolve the state
  self.tmp_state:mv( self.A, self.x_k_minus )
  self.x_k_minus:copy( self.tmp_state )
  if u_k then
    self.tmp_input:mv( self.B, u_k )
    self.x_k_minus:add( self.tmp_input )
  end
  self.tmp_covar:mm( self.A, self.P_k_minus )
  self.P_k_minus:mm( self.tmp_covar, self.A:t() ):add( self.Q )
  return self
end

-- Correct the state estimate based on a measurement
-- Complicated (i.e. fast in-memory) way
local function correct( self, z_k )
  self.tmp_pcor1:mm( self.H, self.P_k_minus )
  self.tmp_pcor2:mm( self.tmp_pcor1, self.H:t() ):add(self.R)
  torch.inverse( self.tmp_pcor3, self.tmp_pcor2 )
  self.tmp_pcor4:mm(self.P_k_minus, self.H:t() )
  self.K_k:mm( self.tmp_pcor4, self.tmp_pcor3 )
  self.K_update:mm( self.K_k, self.H ):mul(-1):add(self.I)
  self.P_k:mm( self.K_update, self.P_k_minus )
  self.tmp_scor:mv( self.H, self.x_k_minus ):mul(-1):add(z_k)
  self.x_k:mv( self.K_k, self.tmp_scor ):add(self.x_k_minus)

  -- Duplicate Values
  self.x_k_minus:copy(self.x_k)
  self.P_k_minus:copy(self.P_k)
  return self
end

-- Simple (i.e. mallocing memory each time) way
local function predict_slow(self, u_k)
  self.x_k_minus = self.A * self.x_k_minus + self.B * (u_k or 0)
  self.P_k_minus = self.A * self.P_k_minus * self.A:t() + self.Q
  return self
end

-- Simple (i.e. malloc'ing memory each time) way
local function correct_slow( self, z_k )
  local tmp1 = self.H * self.P_k_minus * self.H:t()
  local tmp = tmp1 + self.R
  self.K_k = self.P_k_minus * self.H:t() * torch.inverse(tmp)
  self.P_k = (self.I - self.K_k * self.H) * self.P_k_minus
  self.x_k = self.x_k_minus + self.K_k * (z_k - self.H * self.x_k_minus)
end

-- Filter initialization code
local function initialize_filter( nDim, state0 )
  if state0 then assert(#state0==nDim, 'Oops!') end
  local filter = {}
  -- Utility
  filter.I = torch.eye(nDim)
  -- Process
  filter.A = torch.eye(nDim) -- State process w/o input
  filter.B = torch.eye(nDim) -- Control input to state effect
  filter.Q = torch.eye(nDim) -- Additive uncertainty
  -- Measurement
  filter.R = torch.eye(nDim) -- Measurement uncertainty
  filter.H = torch.eye(nDim) 
  -- Prior
  filter.P_k_minus = torch.eye(nDim)
  filter.x_k_minus = state0 and torch.Tensor(state0) or torch.Tensor(nDim):zero()
  -- State
  filter.P_k = torch.Tensor( nDim, nDim ):copy( filter.P_k_minus )
  filter.x_k = torch.Tensor(nDim):copy( filter.x_k_minus )
  ----------
  -- Methods
  ----------
  filter.predict = predict
  filter.correct = correct
  filter.get_prior = get_prior
  filter.get_state = get_state
  return filter
end
lib.initialize_filter = initialize_filter

-- Temporary Variables for complicated fast memory approach
local function initialize_temporary_variables( filter )
  filter.tmp_input = torch.Tensor( filter.B:size(1) )
  filter.tmp_state = torch.Tensor( filter.A:size(1) )
  filter.tmp_covar = torch.Tensor( filter.A:size(1), filter.P_k_minus:size(2) )
  filter.tmp_pcor1 = torch.Tensor( filter.H:size(1), filter.P_k_minus:size(2) )
  filter.tmp_pcor2 = torch.Tensor( filter.tmp_pcor1:size(1), filter.H:size(1) )
  filter.tmp_pcor3 = torch.Tensor( filter.tmp_pcor1:size(1), filter.H:size(1) )
  filter.tmp_pcor4 = torch.Tensor( filter.P_k_minus:size(1), filter.H:size(1) )
  filter.K_k = torch.Tensor( filter.P_k_minus:size(1), filter.H:size(1) )
  filter.K_update = torch.Tensor( filter.K_k:size(1), filter.H:size(2) )
  filter.tmp_scor  = torch.Tensor( filter.H:size(1) )
  return filter
end
lib.initialize_temporary_variables = initialize_temporary_variables

-- Generic filter with default initialization
function lib.linear( nDim )
  local f = initialize_temporary_variables( initialize_filter( nDim ) )
  return f
end

return lib

require('Body')
require('Config')
require('Kinematics')
require('MotionState')
require('trajectory')
require('vector')
require('mcm')
require('pcm')
require('rmp')

--------------------------------------------------------------------------------
-- Rhythmic Movement Primitive Step Controller
--------------------------------------------------------------------------------

-- Setup 
--------------------------------------------------------------------------------

step = MotionState.new(...)
local dcm = step.dcm
step:set_joint_access(0, 'all')
step:set_joint_access(1, 'legs')

local torso_rmp = rmp.new(
  3,                                       -- number of rmp dimensions
  30,                                      -- number of basis functions
  {'periodic', 'antiperiodic', 'periodic'} -- type of basis functions
)

-- Utilities
--------------------------------------------------------------------------------

local function zeros(n)
  local t = {}
  for i = 1,n do
    t[i] = 0
  end
  return t
end

local function ones(n)
  local t = {}
  for i = 1,n do
    t[i] = 1
  end
  return t
end

local function copy_array(ts, td)
  if not ts then return nil end
  local td = td or {}
  for i = 1,#ts do
    td[i] = ts[i]
  end
  return td
end

-- Parameters
--------------------------------------------------------------------------------
step.parameters = {
  step_duration               = 0.70,   -- seconds
  step_height                 = 0.03,   -- meters
  step_ds_ratio               = 0.40,   -- ratio
  x_foot_offset               = 0.00,   -- meters
  y_foot_offset               = 0.097,  -- meters
  z_foot_offset               = -0.77,  -- meters
  rmp_parameters              = torso_rmp:get_parameters(),
  nominal_rmp_state           = torso_rmp:get_state(),
}

-- Private
--------------------------------------------------------------------------------

-- define config variables
local l_foot_sole_transform      = Config.mechanics.l_foot.sole_transform
local r_foot_sole_transform      = Config.mechanics.r_foot.sole_transform
local l_foot_sole_offset         = l_foot_sole_transform:get_pose6D()
local r_foot_sole_offset         = r_foot_sole_transform:get_pose6D()

-- define step variables
local active                     = false
local velocity                   = zeros(3)
local support_foot               = 'l'
local nominal_initialization     = false
local t0                         = nil
local ss_begin_t                 = nil 
local ss_end_t                   = nil 
local swing_foot_offset          = nil
local swing_foot_sole_offset     = nil 
local swing_foot_start_pose      = nil
local swing_foot_start_twist     = nil
local support_foot_offset        = nil
local support_foot_sole_offset   = nil 
local support_foot_start_pose    = nil
local support_foot_start_twist   = nil

-- define control variables
local cop_state                  = {zeros(3), zeros(3), zeros(3)}
local cop_via_state              = {zeros(3), zeros(3), zeros(3)}
local cop_goal_state             = {zeros(3), zeros(3), zeros(3)}
local swing_foot_state           = {zeros(3), zeros(3), zeros(3)}
local swing_foot_via_state       = {zeros(3), zeros(3), zeros(3)}
local swing_foot_goal_state      = {zeros(3), zeros(3), zeros(3)}
local torso_state                = {zeros(3), zeros(3), zeros(3)}
local torso_reference_trajectory = nil

-- define local parameters
local step_duration           = step.parameters.step_duration 
local step_height             = step.parameters.step_height 
local step_ds_ratio           = step.parameters.step_ds_ratio 
local x_foot_offset           = step.parameters.x_foot_offset
local y_foot_offset           = step.parameters.y_foot_offset
local z_foot_offset           = step.parameters.z_foot_offset
local rmp_parameters          = step.parameters.rmp_parameters
local nominal_rmp_state       = step.parameters.nominal_rmp_state

local function update_parameters()
  step_duration = step.parameters.step_duration 
  step_height = step.parameters.step_height 
  step_ds_ratio = step.parameters.step_ds_ratio 
  x_foot_offset = step.parameters.x_foot_offset
  y_foot_offset = step.parameters.y_foot_offset
  z_foot_offset = step.parameters.z_foot_offset
  rmp_parameters = step.parameters.rmp_parameters
  nominal_rmp_state = step.parameters.nominal_rmp_state
end

local function initialize_step_variables()

  t0 = Body.get_time() 
  ss_begin_t = step_duration*(step_ds_ratio/2)
  ss_end_t = step_duration*(1 - step_ds_ratio/2)

  local l_foot_offset = {x_foot_offset, y_foot_offset, z_foot_offset}
  local r_foot_offset = {x_foot_offset,-y_foot_offset, z_foot_offset}

  if (support_foot == 'r') then
    swing_foot_offset = l_foot_offset
    swing_foot_sole_offset = l_foot_sole_offset
    swing_foot_start_pose = pcm:get_l_foot_pose() 
    swing_foot_start_twist = pcm:get_l_foot_twist() 

    support_foot_offset = r_foot_offset
    support_foot_sole_offset = r_foot_sole_offset
    support_foot_start_pose = pcm:get_r_foot_pose() 
    support_foot_start_twist = pcm:get_r_foot_twist() 
  else
    swing_foot_offset = r_foot_offset
    swing_foot_sole_offset = r_foot_sole_offset
    swing_foot_start_pose = pcm:get_r_foot_pose() 
    swing_foot_start_twist = pcm:get_r_foot_twist() 

    support_foot_offset = l_foot_offset
    support_foot_sole_offset = l_foot_sole_offset
    support_foot_start_pose = pcm:get_l_foot_pose() 
    support_foot_start_twist = pcm:get_l_foot_twist() 
  end
end

local function initialize_torso_variables()

  -- intialize torso reference trajectory
  torso_reference_trajectory = function(t)
    local reference_position = {}
    for i = 1, 3 do
      reference_position[i] = velocity[i]*(t - step_duration/2)
                            - support_foot_offset[i]
    end
    return reference_position
  end

  -- intialize torso reference state
  local reference_start_position = torso_reference_trajectory(0)
  local reference_start_velocity = velocity

  -- initialize torso rmp
  torso_rmp:reset()
  torso_rmp:set_period(step_duration)
  torso_rmp:set_parameters(rmp_parameters)
  torso_rmp:set_time_step(Body.get_time_step())
  if (support_foot == 'r') then
    -- TEMPORARY HACK (FOR FORWARD/VERTICAL STEPPING ONLY !)
    torso_rmp:set_phase(2*math.pi)
  end
  
  -- intialize torso rmp state
  if (nominal_initialization) then
    -- TEMPORARY HACK (FOR FORWARD/VERTICAL STEPPING ONLY !)
    local rmp_state = {{}, {}, {}} 
    if (support_foot == 'r') then
      for i = 1, 3 do
        rmp_state[1][i] = nominal_rmp_state[1][i]
        rmp_state[2][i] =-nominal_rmp_state[2][i]
        rmp_state[3][i] = nominal_rmp_state[3][i]
      end
    else
      for i = 1, 3 do
        rmp_state[1][i] = nominal_rmp_state[1][i]
        rmp_state[2][i] = nominal_rmp_state[2][i]
        rmp_state[3][i] = nominal_rmp_state[3][i]
      end
    end
    torso_rmp:set_state(rmp_state)
  else
    local rmp_state = {{}, {}, {}}
    for i = 1, 3 do
      -- TEMPORARY HACK (WHY DOES SUPPORT FOOT TWIST HAVE OPPOSITE SIGN?)
      rmp_state[i][1] =-support_foot_start_pose[i] - reference_start_position[i]
      rmp_state[i][2] = support_foot_start_twist[i] - reference_start_velocity[i]
      rmp_state[i][3] = 0
    end
    torso_rmp:set_state(rmp_state)
  end

  -- intialize torso state
  for i = 1, 3 do
    torso_state[1][i] = reference_start_position[i] + torso_rmp:get_position(i)
    torso_state[2][i] = reference_start_velocity[i] + torso_rmp:get_velocity(i) 
    torso_state[3][i] = torso_rmp:get_acceleration(i)
  end
end

local function initialize_swing_foot_variables()

  local reference_prev_position = torso_reference_trajectory(-step_duration/2)
  local reference_next_position = torso_reference_trajectory(3*step_duration/2)

  -- initialize swing foot state
  if (nominal_initialization) then
    for i = 1, 3 do
      swing_foot_state[1][i] = reference_prev_position[i] + swing_foot_offset[i]
      swing_foot_state[2][i] = 0
      swing_foot_state[3][i] = 0
    end
  else
    for i = 1, 3 do
      swing_foot_state[1][i] = swing_foot_start_pose[i] - support_foot_start_pose[i]
      swing_foot_state[2][i] = 0
      swing_foot_state[3][i] = 0
    end
  end

  -- initialize goal state
  for i = 1, 3 do
    swing_foot_goal_state[1][i] = reference_next_position[i] + swing_foot_offset[i]
    swing_foot_goal_state[2][i] = 0
    swing_foot_goal_state[3][i] = 0
  end

  -- initialize via state
  for i = 1, 3 do
    swing_foot_via_state[1][i] = 0.5*(swing_foot_state[1][i]
                               + swing_foot_goal_state[1][i])
    swing_foot_via_state[2][i] = (swing_foot_goal_state[1][i]
                               - swing_foot_state[1][i])/(ss_end_t - ss_begin_t)
    swing_foot_via_state[3][i] = 0
  end
  swing_foot_via_state[1][3] = swing_foot_via_state[1][3] + step_height
end

local function initialize_cop_variables()
  
  local swing_sole_start_position = {}
  local swing_sole_goal_position = {}
  local support_sole_position = {}
  for i = 1, 3 do
    swing_sole_start_position[i] = swing_foot_state[1][i]
                                 + swing_foot_sole_offset[i]
    swing_sole_goal_position[i] = swing_foot_goal_state[1][i]
                                + swing_foot_sole_offset[i]
    support_sole_position[i] = support_foot_sole_offset[i]
  end

  -- initialize state
  for i = 1, 3 do
    local initial_state = {
      trajectory.minimum_jerk(
        swing_sole_start_position[i],
        support_sole_position[i],
        2*ss_begin_t
      )(ss_begin_t)
    }
    cop_state[1][i] = initial_state[1]
    cop_state[2][i] = initial_state[2]
    cop_state[3][i] = initial_state[3]
  end

  -- intialize goal state
  for i = 1, 3 do
    cop_goal_state[1][i] = swing_sole_goal_position[i]
    cop_goal_state[2][i] = 0 
    cop_goal_state[3][i] = 0 
  end

  -- initialize via state
  for i = 1, 3 do
    cop_via_state[1][i] = support_sole_position[i] 
    cop_via_state[2][i] = 0 
    cop_via_state[3][i] = 0 
  end
end

local function update_torso_state(t, dt)

  local reference_start_position = torso_reference_trajectory(t)
  local reference_start_velocity = velocity

  -- update torso state via rhythmic movement primitive
  torso_rmp:set_time_step(dt)
  torso_rmp:integrate()

  for i = 1, 3 do
    torso_state[1][i] = reference_start_position[i] + torso_rmp:get_position(i)
    torso_state[2][i] = reference_start_velocity[i] + torso_rmp:get_velocity(i) 
    torso_state[3][i] = torso_rmp:get_acceleration(i)
  end
end

local function update_swing_foot_state(t, dt)

  local duration, desired_state

  if (t < ss_begin_t) then
    -- swing foot remains stationary relative to support foot 
    return
  elseif (t < step_duration/2) then
    -- swing foot moves toward via point
    duration = step_duration/2 - t
    desired_state = swing_foot_via_state
  elseif (t < ss_end_t) then
    -- swing foot moves toward goal position
    duration = ss_end_t - t
    desired_state = swing_foot_goal_state
  else
    -- swing foot remains stationary relative to support foot 
    return
  end

  -- update swing foot state via minimum jerk trajectory
  for i = 1, 3 do
    local s0 = {
      swing_foot_state[1][i],
      swing_foot_state[2][i],
      swing_foot_state[3][i],
    }
    local s1 = {
      desired_state[1][i],
      desired_state[2][i],
      desired_state[3][i],
    }
    local s = {trajectory.minimum_jerk_step(s0, s1, duration, dt)}
    swing_foot_state[1][i] = s[1]
    swing_foot_state[2][i] = s[2]
    swing_foot_state[3][i] = s[3]
  end
end

local function update_cop_state(t, dt)

  local duration, desired_state

  if (t < ss_begin_t) then
    -- cop moves toward center of support foot
    duration = ss_begin_t - t
    desired_state = cop_via_state
  elseif (t < ss_end_t) then
    -- cop remains stationary at center of support foot 
    return
  else
    -- cop moves toward center of next support foot
    duration = step_duration + ss_begin_t - t
    desired_state = cop_goal_state
  end

  -- update desired cop state via minimum jerk trajectory
  for i = 1, 3 do
    local s0 = {
      cop_state[1][i],
      cop_state[2][i],
      cop_state[3][i],
    }
    local s1 = {
      desired_state[1][i],
      desired_state[2][i],
      desired_state[3][i],
    }
    local s = {trajectory.minimum_jerk_step(s0, s1, duration, dt)}
    cop_state[1][i] = s[1]
    cop_state[2][i] = s[2]
    cop_state[3][i] = s[3]
  end
end

-- Public
--------------------------------------------------------------------------------

function step:set_nominal_initialization(bool)
  nominal_initialization = bool
end

function step:set_support_foot(foot_id) -- l or r
  support_foot = string.lower(foot_id:match('^%a'))
end

function step:set_velocity(velocity_vector)
  copy_array(velocity_vector, velocity)
end

function step:set_foothold(pose)
  -- TODO
end

function step:set_rmp(rmp_object)
  torso_rmp = rmp_object
end

function step:set_rmp_parameters(parameters, dim)
  if (dim) then
    self.parameters.rmp_parameters[dim] = parameters
  else
    self.parameters.rmp_parameters = parameters
  end
end

function step:get_support_foot()
  return support_foot
end

function step:get_velocity()
  return copy_array(velocity)
end

function step:get_foothold()
  -- TODO
end

function step:get_rmp()
  return torso_rmp
end

function step:get_rmp_parameters(dim)
  if (dim) then
    return self.parameters.rmp_parameters[dim]
  else
    return self.parameters.rmp_parameters
  end
end

function step:get_torso_state()
  return torso_state
end

function step:get_swing_foot_state()
  return swing_foot_state
end

function step:get_cop_state()
  return cop_state
end

function step:get_configuration()
  -- get current joint positions for both legs
    local torso_frame = Transform.pose6D(torso_state[1])
    local l_foot_frame, r_foot_frame
    if (support_foot == 'r') then
      r_foot_frame = Transform.pose6D({0, 0, 0})
      l_foot_frame = Transform.pose6D(swing_foot_state[1])
    else
      l_foot_frame = Transform.pose6D({0, 0, 0})
      r_foot_frame = Transform.pose6D(swing_foot_state[1])
    end
    return Kinematics.inverse_pos_legs(l_foot_frame, r_foot_frame, torso_frame)
end

function step:learn_torso_orbit(xdata, tdata)
  -- learn rmp parameters from periodic training signal
  torso_rmp:learn_trajectory(xdata, tdata)
  self.parameters.rmp_parameters = torso_rmp:get_parameters()
  self.parameters.nominal_rmp_state = torso_rmp:get_state()
end

function step:initialize()
  update_parameters()
  initialize_step_variables()
  initialize_torso_variables()
  initialize_swing_foot_variables()
  initialize_cop_variables()
end

function step:start()
  step:initialize()
  active = true
end

function step:stop()
  active = false
end

function step:is_active()
  return active
end

function step:entry()
  local q0 = dcm:get_joint_position_sensor('legs')
  dcm:set_joint_force(0, 'legs')
  dcm:set_joint_position(q0, 'legs')
  dcm:set_joint_velocity(0, 'legs')
  dcm:set_joint_stiffness(1, 'legs')
  dcm:set_joint_damping(0, 'legs')
  update_parameters()
end

function step:update()
  if active then

    local t = Body.get_time() - t0
    local dt = Body.get_time_step()

    update_torso_state(t, dt)
    update_swing_foot_state(t, dt)
    update_cop_state(t, dt)

    -- update actuators
    local q = self:get_configuration()
    dcm:set_joint_position(q, 'legs')

    -- update desired center of pressure
    local desired_cop = {} 
    for i = 1, 3 do
      desired_cop[i] = cop_state[1][i] - torso_state[1][i]
    end
    mcm:set_desired_cop(desired_cop)

    if (t >= step_duration) then
      active = false 
    end
  end
end

function step:exit()
end

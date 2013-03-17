require('Transform')
require('cbuffer')
require('Config')
require('webots')
require('vector')
require('util')
require('dcm')

Platform = {}

local function limit(x, min, max)
  return math.min(math.max(x, min), max)
end

-- Setup
---------------------------------------------------------------------------

local joint = Config.joint
local simulator_iterations = 2

-- servo controller parameters
local max_force = 100
local max_position_p_gain = 20000
local max_position_d_gain = 0  -- damping disabled due to ODE instability
local max_velocity = 7
local max_acceleration = 70

local tags = {} -- webots tags
local time_step = nil
local torso_twist_updated = 0
local torso_twist = vector.zeros(6)

local servoNames = { -- webots servo names
  'l_hip_yaw', 'l_hip_roll', 'l_hip_pitch', 
  'l_knee_pitch', 'l_ankle_pitch', 'l_ankle_roll',
  'r_hip_yaw', 'r_hip_roll', 'r_hip_pitch', 
  'r_knee_pitch', 'r_ankle_pitch', 'r_ankle_roll',
--[[
  'waist_yaw', 
  'l_shoulder_pitch', 'l_shoulder_roll', 'l_shoulder_yaw',
  'l_elbow_pitch', 'l_wrist_yaw', 'l_wrist_pitch',
  'r_shoulder_pitch', 'r_shoulder_roll', 'r_shoulder_yaw',
  'r_elbow_pitch', 'r_wrist_yaw', 'r_wrist_pitch',
  'neck_yaw', 'head_pitch'
--]]
}

local velocity_p_gain = 0.25*vector.ones(#servoNames)
local joint_ff_force = vector.zeros(#servoNames)
local joint_p_force = vector.zeros(#servoNames)

-- Actuator / sensor interface 
----------------------------------------------------------------

local function initialize_devices()
  -- intialize webots devices
  tags.robot = webots.wb_supervisor_node_get_from_def('Ash')
  tags.robot_translation = webots.wb_supervisor_node_get_field(
    tags.robot, "translation")
  tags.robot_rotation = webots.wb_supervisor_node_get_field(
    tags.robot, "rotation")
  tags.gyro = webots.wb_robot_get_device('gyro')
  webots.wb_gyro_enable(tags.gyro, time_step)
  tags.accel = webots.wb_robot_get_device('accelerometer')
  webots.wb_accelerometer_enable(tags.accel, time_step)
  tags.physics_receiver = webots.wb_robot_get_device('physics_receiver')
  webots.wb_receiver_enable(tags.physics_receiver, time_step)
  tags.physics_emitter = webots.wb_robot_get_device('physics_emitter')

  tags.servo = {}
  for i = 1,#servoNames do
    tags.servo[i] = webots.wb_robot_get_device(servoNames[i])
    webots.wb_servo_enable_position(tags.servo[i], time_step)
    webots.wb_servo_enable_motor_force_feedback(tags.servo[i], time_step)
    webots.wb_servo_set_control_p(tags.servo[i], 100)
    webots.wb_servo_set_position(tags.servo[i], 1/0)
    webots.wb_servo_set_velocity(tags.servo[i], 0)
    webots.wb_servo_set_motor_force(tags.servo[i], 1e-15)
    webots.wb_servo_set_acceleration(tags.servo[i], max_acceleration)
  end
end

local function update_actuators()
  -- update webots actuator values 
  local joint_enable = dcm:get_joint_enable()
  local joint_force_desired = dcm:get_joint_force()
  local joint_position_desired = dcm:get_joint_position()
  local joint_velocity_desired = dcm:get_joint_velocity()
  local joint_position_actual = dcm:get_joint_position_sensor()
  local joint_velocity_actual = dcm:get_joint_velocity_sensor()
  local joint_position_p_gain = dcm:get_joint_position_p_gain()
  local joint_position_i_gain = dcm:get_joint_position_i_gain()
  local joint_position_d_gain = dcm:get_joint_position_d_gain()
  local joint_velocity_p_gain = dcm:get_joint_velocity_p_gain()
  local position_error = vector.zeros(#servoNames)

  -- calculate joint forces
  for i = 1,#servoNames do
    if (joint_enable[i] == 0) then
      -- zero forces
      joint_ff_force[i] = 0 
      joint_p_force[i] = 0
      webots.wb_servo_set_force(tags.servo[i], 0)
    else
      -- calculate feedforward force 
      joint_ff_force[i] = joint_force_desired[i]
      joint_ff_force[i] = limit(joint_ff_force[i], -max_force, max_force)
      -- calculate spring force 
      position_error[i] = joint_position_desired[i] - joint_position_actual[i]
      joint_position_p_gain[i] = limit(joint_position_p_gain[i], 0, 1)
      joint_p_force[i] = joint_position_p_gain[i]*max_position_p_gain*position_error[i]
      joint_p_force[i] = limit(joint_p_force[i], -max_force, max_force)
    end
  end

  -- update spring force using motor velocity controller
  for i = 1,#servoNames do
    local max_servo_force = math.abs(joint_p_force[i])
    local servo_velocity = velocity_p_gain[i]*position_error[i]*(1000/time_step)
    servo_velocity = limit(servo_velocity, -max_velocity, max_velocity)
    webots.wb_servo_set_motor_force(tags.servo[i], max_servo_force)
    webots.wb_servo_set_velocity(tags.servo[i], servo_velocity)
  end

  local buffer = cbuffer.new((12 + 7)*8)

  -- update leg feedforward forces using physics plugin
  for i = 1,12 do
    local servo_force = joint_ff_force[i]
    servo_force =  limit(servo_force, -max_force, max_force)
    buffer:set('double', servo_force, (i-1)*8)
  end

  -- update torso twist using physics plugin
  buffer:set('double', torso_twist_updated, (12)*8)
  for i = 1,6 do
    buffer:set('double', torso_twist[i], (12 + i)*8)
  end
  torso_twist_updated = 0

  webots.wb_emitter_send(tags.physics_emitter, tostring(buffer))
end

local function update_sensors()
  -- update webots sensor values
  local joint_enable = dcm:get_joint_enable()
  for i = 1,#servoNames do
    if (joint_enable[i] == 0) then
      dcm:set_joint_force_sensor(0, i)
    else
      dcm:set_joint_force_sensor(webots.wb_servo_get_motor_force_feedback(
          tags.servo[i]) + joint_ff_force[i], i)
    end
    dcm:set_joint_position_sensor(webots.wb_servo_get_position(tags.servo[i]), i)
  end
  
  -- update imu readings
  local t = {}
  local orientation = webots.wb_supervisor_node_get_orientation(tags.robot)
  t[1] = vector.new({orientation[1], orientation[2], orientation[3], 0})
  t[2] = vector.new({orientation[4], orientation[5], orientation[6], 0})
  t[3] = vector.new({orientation[7], orientation[8], orientation[9], 0})
  t[4] = vector.new({0, 0, 0, 1});
  local euler_angles = Transform.get_euler(t)
  dcm:set_ahrs(webots.wb_gyro_get_values(tags.gyro), 'gyro')
  dcm:set_ahrs(webots.wb_accelerometer_get_values(tags.accel), 'accel')
  dcm:set_ahrs(euler_angles, 'euler')

  -- update force-torque readings and joint velocities using physics plugin
  local buffer = cbuffer.new(webots.wb_receiver_get_data(tags.physics_receiver))
  local joint_velocity = {}
  for i = 1,12 do
    joint_velocity[i] = buffer:get('double', (i-1)*8)
  end
  dcm:set_joint_velocity_sensor(joint_velocity)
  local l_fts = {}
  local r_fts = {}
  for i = 1,6 do
    l_fts[i] = buffer:get('double', (i-1)*8 + 12*8)
    r_fts[i] = buffer:get('double', (i-1)*8 + 12*8 + 48)
  end
  dcm:set_force_torque(l_fts, 'l_foot')
  dcm:set_force_torque(r_fts, 'r_foot')
  webots.wb_receiver_next_packet(tags.physics_receiver)
end

-- User interface 
---------------------------------------------------------------------------

Platform.get_time = webots.wb_robot_get_time

function Platform.set_time_step(t)
  -- for compatibility
end

function Platform.get_time_step()
  return time_step*simulator_iterations/1000
end

function Platform.get_update_rate()
  return 1000/(time_step*simulator_iterations)
end

function Platform.reset_simulator()
  webots.wb_supervisor_simulation_revert()
end

function Platform.reset_simulator_physics()
  webots.wb_supervisor_simulation_physics_reset()
end

function Platform.set_simulator_torso_frame(frame)
  local pose = frame:get_pose()
  webots.wb_supervisor_field_set_sf_vec3f(tags.robot_translation,
    {pose[1], pose[2], pose[3]})
  webots.wb_supervisor_field_set_sf_rotation(tags.robot_rotation, 
    {0, 0, 1, pose[4]})
end

function Platform.set_simulator_torso_twist(twist)
  torso_twist = twist
  torso_twist_updated = 1
end

function Platform.entry()
  -- initialize webots devices
  webots.wb_robot_init()
  time_step = webots.wb_robot_get_basic_time_step()
  initialize_devices()

  -- initialize shared memory 
  dcm:set_joint_enable(1, 'all')
  dcm:set_joint_position_p_gain(1, 'all') -- position control
  dcm:set_joint_position_i_gain(0, 'all')
  dcm:set_joint_position_d_gain(0, 'all')
  dcm:set_joint_velocity_p_gain(0, 'all')
  dcm:set_joint_force(0, 'all')
  dcm:set_joint_position(0, 'all')
  dcm:set_joint_velocity(0, 'all')
  dcm:set_joint_force_sensor(0, 'all')
  dcm:set_joint_position_sensor(0, 'all')
  dcm:set_joint_velocity_sensor(0, 'all')

  -- initialize sensor shared memory
  Platform.update()
end

function Platform.update()
  for i = 1, simulator_iterations do
    update_actuators()
    if (webots.wb_robot_step(time_step) < 0) then
      os.exit()
    end
    update_sensors()
  end
end

function Platform.exit()
end

return Platform

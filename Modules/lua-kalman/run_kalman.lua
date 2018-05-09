#!/usr/bin/env luajit

local vnew = require'vector'.new
local vunit = require'vector'.unit
local vnorm = require'vector'.norm
local DEG_TO_RAD = math.pi/180
local RAD_TO_DEG = 180/math.pi
local cos = require'math'.cos
local sin = require'math'.sin
local floor = require'math'.floor
local pow = require'math'.pow
local sqrt = require'math'.sqrt
local tinsert = require'table'.insert
--
local kalman = require'kalman'
local tf = require'transform'
local quaternion = require'quaternion'
local time = require'unix'.time
local usleep = require'unix'.usleep
local time_us = require'unix'.time_us
local has_png, png = pcall(require, 'png')
--
local racecar = require'racecar'
local flags = require'racecar'.parse_arg(arg)
local log_announce = require'racecar'.log_announce
--
local logger = require'logger'
local fnames = {}
for i,flag in ipairs(flags) do
  if flag:match(logger.iso8601_match) then
    tinsert(fnames, flag)
  end
end

local update = {} -- callbacks

local realtime = flags.realtime ~= 0

----------- IMU -----------
local ukf = kalman.ukf()
local ukf_gyro = kalman.ukf()
local gyro_bias = {count = 0, mean=vnew{0, 0, 0}, variance=vnew{0,0,0}}
local acc_bias = {count = 0, mean=vnew{0, 0, 0}, variance=vnew{0,0,0}}
local q_imu_to_rbt = quaternion.from_angle_axis(
      math.pi, {1,1,0})
local q_imu_to_rbt_inv = quaternion.conjugate(q_imu_to_rbt)
local rbt0_to_rbt = tf.eye()

local ENABLE_IMU_REMAP = true
local ENABLE_AUTO_BIAS = true

local t_last_imu, accel_last, gyro_last
function update.imu(obj)
  local t_imu = obj.timeM
  local dt_imu = tonumber(t_imu - (t_last_imu or t_imu)) / 1e3
  t_last_imu = t_imu

  -- Perform the re-mapping for the robot
  local q_imu = obj.q and #obj.q==4 and quaternion.unit(obj.q * q_imu_to_rbt_inv)
  local accel, gyro
  if ENABLE_IMU_REMAP then
    accel = vnew{obj.accel[2], obj.accel[1], -obj.accel[3]}
    gyro = DEG_TO_RAD * vnew{obj.gyro[2], obj.gyro[1], -obj.gyro[3]}
  else
    accel, gyro = vnew(obj.accel), vnew(obj.gyro) * DEG_TO_RAD
  end

  -- Find the accelerometer bias at startup
  if ENABLE_AUTO_BIAS and (acc_bias.count < 300) then
    acc_bias.count = acc_bias.count + 1
    if acc_bias.count == 1 then
      acc_bias.mean = vnew{unpack(accel)}
      acc_bias.variance = 0 * vnew{unpack(accel)}
    else
      local mean0 = acc_bias.mean
      local mean1 = mean0 + (accel - mean0) / acc_bias.count
      acc_bias.mean = mean1
      -- Independent variances
      for i, w in ipairs(accel) do
        acc_bias.variance[i] = acc_bias.variance[i] + (w - mean0[i]) * (w - mean1[i])
      end
    end
  elseif ENABLE_AUTO_BIAS and (acc_bias.count == 300) then
    acc_bias.variance = acc_bias.variance / (acc_bias.count - 1)
    gyro_bias.variance = gyro_bias.variance / (gyro_bias.count - 1)
    ENABLE_AUTO_BIAS = false
  end

  gyro[1] = gyro[1] - gyro_bias.mean[1]
  gyro[2] = gyro[2] - gyro_bias.mean[2]
  gyro[3] = gyro[3] - gyro_bias.mean[3]

  accel = accel * 0.25 + 0.75 * (accel_last or accel)
  accel_last = accel

  if realtime then
    print()
    print("MPU:", q_imu and vnew(quaternion.to_rpy(q_imu)) * RAD_TO_DEG, dt_imu)
    print("Gyro", gyro)
    print("gyro norm", vnorm(gyro))
    print("Accel", accel)
    print("Gyro bias")
    for k, v in pairs(gyro_bias) do print(k, v) end
    print("Accel bias")
    for k, v in pairs(acc_bias) do print(k, v) end
  end

  -- SLAM
  if dt_imu < 0.20 then

    ukf:motion_gyro(gyro, dt_imu)
    local gNorm = vnorm(accel)
    local gNormErr = math.abs(gNorm - 1)
    if gNormErr <= 0.04 then
      -- TODO: Give a unit vector with noise filtering
      accel = vunit(accel)
      ukf:correct_gravity(accel)
    else
      print("Jerk", gNorm, accel)
      -- ukf:correct_gravity(accel)
    end

    ukf_gyro:just_gyro(gyro, dt_imu, accel)
  end

  -- The roll/pitch estimates are not great
  -- rbt0_to_rbt = tf.from_rpy_trans({rpy[1], rpy[2], 0})

  if realtime then
    local rpy_ukf = vnew(quaternion.to_rpy(ukf.orientation))
    print("UKF:", rpy_ukf * RAD_TO_DEG)
    local rpy_ukf_gyro = vnew(quaternion.to_rpy(ukf_gyro.orientation))
    print("ukf_gyro:", rpy_ukf_gyro * RAD_TO_DEG)
    log_announce(false, {
      dt = dt_imu,
      accel = accel,
      gyro = gyro,
      --
      q = q_imu,
      rpy = q_imu and quaternion.to_rpy(q_imu) or rpy_ukf,
      --
      q_gyro = ukf_gyro.orientation,
      q_ukf = ukf.orientation,
    }, "inertial")
  end

end

if #fnames==0 then
  print("Listening on socket")
  racecar.listen(update)
else
  print("Plating files")
  local realtime0 = realtime
  racecar.play(fnames, realtime, update, function(dt)
    dt = dt/1e6
    print("dt", dt)
    -- realtime = realtime0 and dt>24
    -- if dt>0.025 then os.exit() end
    -- if dt>0.065 then os.exit() end
    -- if dt>6.55 then os.exit() end
    -- if dt>20 then os.exit() end
    -- if dt>25 then os.exit() end
    -- if dt>25.87 then os.exit() end
    -- if dt>26 then os.exit() end
    -- if dt>27 then os.exit() end
    -- if dt>35 then os.exit() end
    end)
end

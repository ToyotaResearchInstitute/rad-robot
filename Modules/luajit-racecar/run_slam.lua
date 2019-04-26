#!/usr/bin/env luajit
--[[
Usage:
./run_slam.lua --realtime true ../logs/GoodFriday/good_friday_20190419T173135Z-000.lmp
--]]

local racecar = require'racecar'
local flags = racecar.parse_arg(arg)
local ENABLE_AUTO_BIAS = flags.enable_auto_bias and flags.enable_auto_bias ~= 0
local ENABLE_IMU_REMAP = flags.enable_imu_remap and flags.enable_imu_remap ~= 0
local realtime = flags.realtime

racecar.init()
local log_announce = require'racecar'.log_announce

local unpack = unpack or require'table'.unpack

local quaternion = require'quaternion'
local vnew = require'vector'.new
local vnorm = require'vector'.norm
local vrad = require'vector'.rad
local vunit = require'vector'.unit
local time = require'unix'.time
local kalman = require'kalman'
local slam = require'slam'
local tf = require'transform'

-- GLOBALS
local filter_slam = slam.new({
  xmin = -10,
  ymin = -10,
  xmax = 10,
  ymax = 10,
  scale = 0.025,
})
local filter_ukf = kalman.ukf()

-- callbacks
local cb_tbl = setmetatable({}, {
  __index = function(_, k)
    return function(obj)
      if realtime then log_announce(false, obj, k) end
    end
  end
})

----------- IMU -----------
local gyro_bias = {count = 0, mean=vnew{0, 0, 0}, variance=vnew{0,0,0}}
local acc_bias = {count = 0, mean=vnew{0, 0, 0}, variance=vnew{0,0,0}}
local q_imu_to_rbt = quaternion.from_angle_axis(
      math.pi, {1,1,0})
local q_imu_to_rbt_inv = quaternion.conjugate(q_imu_to_rbt)
local rbt0_to_rbt = tf.eye()

local t_last_imu, accel_last, gyro_last
function cb_tbl.imu(obj)
  local t_imu = obj.timeM
  local dt_imu = tonumber(t_imu - (t_last_imu or t_imu)) / 1e3
  t_last_imu = t_imu

  -- Perform the re-mapping for the robot
  local q_imu = obj.q and #obj.q==4 and quaternion.unit(obj.q * q_imu_to_rbt_inv)
  local accel
  local gyroSensitivity = 131-- 131, 65.5, 32.8, 16.4
  local gyro = vnew(obj.gyro) / gyroSensitivity
  if ENABLE_IMU_REMAP then
    accel = vnew{obj.accel[2], obj.accel[1], -obj.accel[3]}
    gyro = vrad{gyro[2], gyro[1], -gyro[3]} / gyroSensitivity
  else
    accel, gyro = vnew(obj.accel), vrad(gyro) / gyroSensitivity
  end
  -- print(dt_imu, "gyro", unpack(gyro))

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

  gyro = gyro - gyro_bias.mean
  gyro = gyro * 0.8 + 0.2 * (gyro_last or gyro)
  gyro_last = gyro

  accel = accel * 0.8 + 0.2 * (accel_last or accel)
  accel_last = accel

  -- SLAM
  if dt_imu < 0.20 then
    filter_ukf:motion_gyro(gyro, dt_imu)
    local gNorm = vnorm(accel)
    local gNormErr = math.abs(gNorm - 1)
    if gNormErr <= 0.06 then
      -- TODO: Give a unit vector with noise filtering
      accel = vunit(accel)
      filter_ukf:correct_gravity(accel)
    else
      -- print("Jerk", gNorm, accel)
      -- filter_ukf:correct_gravity(accel)
    end
    local gyro_dt = gyro * dt_imu
    -- print("dt_imu", dt_imu)
    -- print("gyro_dt", math.deg(gyro_dt[1]), math.deg(gyro_dt[2]), math.deg(gyro_dt[3]))
    filter_slam:update_gyro(gyro_dt)
  end

  -- The roll/pitch estimates are not great
  -- rbt0_to_rbt = tf.from_rpy_trans({rpy[1], rpy[2], 0})

  if realtime then
    log_announce(false, {
      dt = dt_imu,
      rpy = quaternion.to_rpy(q_imu and q_imu or filter_ukf.orientation),
      accel = accel,
      gyro = gyro,
      q_ukf = filter_ukf.orientation,
    }, "inertial")
  end

end

----------- Laser -----------
local d2p = require'hokuyo'.distances2points
local lsr_to_rbt0 = tf.trans(0.28, 0, 0.115)
function cb_tbl.hokuyo(obj)
  local lsr_to_rbt = rbt0_to_rbt * lsr_to_rbt0
  -- Find the points in the frame of the hokuyo
  local xs_hok, ys_hok, zs_hok, hits = d2p(obj.distances)
  local intensities = {}
  if obj.intensities then
    for i,int in ipairs(obj.intensities) do
      intensities[i] = hits[i] and int or 0
    end
  else
    for i,h in ipairs(hits) do
      intensities[i] = h and 1 or 0
    end
  end
  -- Find the points in the frame of the robot
  -- NOTE: Weird rotations going on here...
  local pts_rbt = tf.batch_mul(
    lsr_to_rbt, {xs_hok, ys_hok, zs_hok})
  -- Now, ask SLAM to use the valid points
  filter_slam:update_laser(pts_rbt, hits)

  if realtime then
    log_announce(false, {pts=pts_rbt, int=intensities}, 'point_cloud_hokuyo')
  end
end

----------- Motor Controller -----------
local last_tach_t, last_tach = false, 0
local TACH_FACTOR = 8 -- No idea why this factor...
function cb_tbl.vesc(obj, t_us)
  -- for k, v in pairs(obj) do print(k, v) end
  if obj.sensor_request then return end
  local dt_tach = tonumber(t_us - (last_tach_t or t_us)) / 1e6
  last_tach_t = t_us
  -- Remap
  local tach = obj.tach
  -- tach = -1 * tach
  --
  local d_tach = tach - (last_tach or 0)
  last_tach = tach
  -- Kill off values that had too much time in b/t
  if d_tach==0 then return end
  if dt_tach < 0.15 then
    local dx_odom = d_tach * dt_tach / TACH_FACTOR
    -- print("dx_odom", dx_odom)
    local d_odom = vnew{dx_odom, 0, 0}
    filter_slam:update_odometry(d_odom)
  end
end

-- Send the map every second
local pkt = {}
local dt_us_send = 1e6
local t_us_send_last = 0
local function cb_loop(t_us)
  local dt_us = t_us - t_us_send_last
  if dt_us < dt_us_send then return end
  t_us_send_last = t_us

  local pose_xya = filter_slam:get_pose()
  pkt = {
    t = tonumber(t_us),
    pose = pose_xya
  }
  -- local t_save0 = time()
  pkt.png = assert(filter_slam.omap.gridmap:save("png"))
  -- local t_save1 = time()

  if realtime then log_announce(false, pkt, 'slam') end
  -- if t - t0 > 27 then os.exit() end
  -- filter_slam.omap.gridmap:save(string.format("/tmp/map%03d.pgm", dt0_log/1e6))
end

local t0
local function cb_debug(t_us)
  local t = tonumber(t_us)/1e6
  if not t0 then t0 = t end
  local px, py, pa = unpack(filter_slam:get_pose())
  if pkt.png then
    print(string.format("%.2f ratio -> %d bytes", #pkt.png / filter_slam.omap.gridmap.n_cells, #pkt.png))
    local fname_map = string.format(racecar.ROBOT_HOME.."/logs/map%03d.png", t - t0)
    io.open(fname_map, "w"):write(pkt.png):close()
  end
  local info = {
    -- string.format("Path: %s", desired_path),
    string.format("Pose: x=%.2fm, y=%.2fm, a=%.2f°", px, py, math.deg(pa))
  }
  return table.concat(info, "\n")
end

-- Actually play the log
-- flags: commandline list of log files
local fnames = {unpack(flags)}
if #fnames==1 then fnames = fnames[1] end

-- run_update
assert(racecar.replay(fnames, {
  realtime=realtime,
  channel_callbacks=cb_tbl,
  fn_loop = cb_loop,
  fn_debug = cb_debug
}))

-- racecar.listen{
--   channel_callbacks = cb_tbl,
--   loop_rate = 100, -- 100ms loop
--   -- fn_loop = cb_loop,
--   fn_debug = cb_debug
-- }

-- ffmpeg -r 1 -i map%03d.png -pix_fmt yuv420p map.m4v

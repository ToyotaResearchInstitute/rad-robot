local libWorld = {}

--SJ: a dummy world file
--we can add slam, odometry, and so on later

-- TODO: Add Attack bearing
-- TODO: Add Webots ground truth knowledge
local Body   = require'Body'
local vector = require'vector'
local util = require'util'
local odomScale = Config.world.odomScale
local odomDrift = Config.world.odomDrift or -0.0001

local ballFilter = require'ballFilter'
local poseFilter = require'poseFilter'
local RESAMPLE_PERIOD = Config.world.resample_period
local RESAMPLE_COUNT = Config.world.resample_count

require'wcm'
require'gcm'
require'mcm'
wcm.set_robot_use_imu_yaw(Config.world.use_imu_yaw and 1 or 0)

-- Timestamps
local t_entry
-- Cycle count
local count
-- Objects
local ball, goal, obstacle, line
-- Obstacle filters
local OF = {}

-- Initial odometry
local uOdometry0 = vector.zeros(3)
-- Save the resampling times
local t_resample = 0


local obstacles={}
local function reset_obstacles()
  obstacles={}
  wcm.set_obstacle_num(0)
end

local function new_obstacle(a,r)
  --2nd order stat for the obstacle group
  return {asum=a,rsum=r,asqsum=a*a,rsqsum=r*r,count=1,aave=a,rave=r}
end

local function add_obstacle(a, r, da, dr)
  local min_dist = math.huge
  local min_index=0
  for i=1,#obstacles do
    --Just check angle to cluster observation
    --TODO: we can use distance too
    local adist = math.abs(util.mod_angle(obstacles[i].aave-a))
    if adist<min_dist then min_dist,min_index = adist,i end
  end
  if min_index==0 or min_dist>10*DEG_TO_RAD then
    obstacles[#obstacles+1] = new_obstacle(a,r)
  else
    obstacles[min_index].count = obstacles[min_index].count + 1
    obstacles[min_index].asum = obstacles[min_index].asum + a
    obstacles[min_index].rsum = obstacles[min_index].rsum + r
		-- TODO: these are not used for filtering
    obstacles[min_index].asqsum = obstacles[min_index].asqsum + a*a
    obstacles[min_index].rsqsum = obstacles[min_index].rsqsum + r*r
    obstacles[min_index].aave = obstacles[min_index].asum/obstacles[min_index].count
    obstacles[min_index].rave = obstacles[min_index].rsum/obstacles[min_index].count
  end
end

local function update_obstacles()
  local counts={}
  for i=1,#obstacles do
--    print(string.format("Obstacle %d angle: %d dist: %.1f count: %d",
--      i,obstacles[i].aave*180/math.pi, obstacles[i].rave, obstacles[i].count))
    counts[i]=obstacles[i].count
  end
  --Sort the obstacles by their count
  table.sort(counts, function (a,b) return a>b end)
  --write top 3 obstacles to wcm
  local pose = wcm.get_robot_pose()
  for i=1, math.min(3,#obstacles) do
    local not_found,j = true,1
    while not_found and j< #obstacles+1 do
      if obstacles[j].count==counts[i] then
				--print('counts:',counts[i])
        local x = obstacles[j].rave * math.cos(obstacles[j].aave )
        local y = obstacles[j].rave * math.sin(obstacles[j].aave )

        local obs_global = util.pose_global({x,y,0},pose)
        wcm['set_obstacle_v'..i]({obs_global[1],obs_global[2]})
        not_found = false
      end
      j=j+1
    end
  end
  wcm.set_obstacle_num(math.min(2,#obstacles))
end


local yaw0 = 0
local function update_odometry(uOdometry)
  -- Scale the odometry

  --odometry update is called every frame (~120hz)
  --we drift away 1cm back every step (~0.8sec)
  --then we have 0.01m / 0.8sec / 120 hz drift per step

  uOdometry[1] = odomScale[1] * uOdometry[1] + odomDrift
  uOdometry[2] = odomScale[2] * uOdometry[2]
  uOdometry[3] = odomScale[3] * uOdometry[3]
  -- Next, grab the gyro yaw

--  if Config.use_imu_yaw and mcm.get_walk_ismoving()>0 then

  if Config.use_imu_yaw then
    if IS_WEBOTS then
      gps_pose = wcm.get_robot_pose_gps()
      uOdometry[3] = gps_pose[3] - yaw0
      yaw0 = gps_pose[3]
    else
      local yaw = Body.get_rpy()[3]
      uOdometry[3] = yaw - yaw0
      yaw0 = yaw
    end
  end
  yaw0 = Body.get_rpy()[3] --We need to keep update this (to use the increment only while walking)

  --Update pose using odometry info for now
  local pose = wcm.get_robot_pose()
  wcm.set_robot_pose(util.pose_global(uOdometry,pose))

  --TODO: slam or wall detection-based pose


end

local function update_vision(detected)

  if not detected then return end

  local t = unix.time()
  if t - t_resample > RESAMPLE_PERIOD or count%RESAMPLE_COUNT==0 then
    poseFilter.resample()
    if mcm.get_walk_ismoving()==1 then poseFilter.addNoise() end
  end
  -- If the ball is detected
	ball = detected.ball


  if Config.disable_ball_when_lookup and wcm.get_ball_disable()>0 then
--    print("BALL DISABLED")
  else
    if ball then
      ballFilter.observation_xy(ball.v[1], ball.v[2], ball.dr, ball.da, ball.t)
    end
  end


  -- We cannot find the ball.
  --add fake observation at behind the robot so that robot can start turning
  if wcm.get_ball_notvisible()==1 then
    wcm.set_ball_notvisible(0)
    if gcm.get_game_role()==1 then
      ballFilter.observation_xy(-0.5,0, 0.5, 20*math.pi/180, t)
    end
  end



--------------------------------------------------------------------------------
-- TODO: fix nan bug with this
  -- If the goal is detected
	goal = detected.posts
  if goal then
    if goal[1].type == 3 then
      if Config.debug.goalpost then
        print(string.format("Two post observation: type %d v1(%.2f %.2f) v2(%.2f %.2f)",
          goal[1].type,
          goal[1].v[1],goal[1].v[2],
          goal[2].v[1],goal[2].v[2]
          ))
      end
      if (not Config.disable_goal_vision) then
        if Config.goalie_odometry_only and gcm.get_game_role()==0 then
          print("Goalie, goal update disabled")
        else
          goal_type_to_filter[goal[1].type]({goal[1].v, goal[2].v})
        end
      end
    else
      if Config.debug.goalpost then
        print(string.format("Single post observation: type %d v(%.2f %.2f)",
          goal[1].type,
          goal[1].v[1],goal[1].v[2]
          ))
      end
      if not Config.disable_goal_vision then
        if Config.goalie_odometry_only and gcm.get_game_role()==0 then
          print("Goalie, goal update disabled")
        else
          goal_type_to_filter[goal[1].type]({goal[1].v, vector.zeros(4)})
        end
      end
    end
  end
--------------------------------------------------------------------------------


  if wcm.get_obstacle_reset()==1 then
    reset_obstacles()
    wcm.set_obstacle_reset(0)
  end

  -- If the obstacle is detected
  obstacle = detected.obstacles
  if obstacle then
    --SJ: we keep polar coordinate statstics of the observed obstacles
    -- print("detected obstacles:",#obstacle.xs)
    for i=1,#obstacle.xs do
      local x, y = obstacle.xs[i], obstacle.ys[i]
      local r =math.sqrt(x^2+y^2)
      local a = math.atan2(y,x)
      --local dr, da = 0.1*r, 15*DEG_TO_RAD -- webots
      local dr, da = 0.1*r, 5*DEG_TO_RAD -- TODO
      add_obstacle(a,r, da,dr)
    end
    update_obstacles()

  end  -- end of obstacle

end

function libWorld.pose_reset()
  wcm.set_robot_reset_pose(0)
  wcm.set_robot_pose({0,0,0})
  wcm.set_robot_odometry({0,0,0})
  yaw0 = Body.get_rpy()[3]
  if IS_WEBOTS then
    gps_pose = wcm.get_robot_pose_gps()
    yaw0 = gps_pose[3]
    wcm.set_robot_pose_gps0(wcm.get_robot_pose_gps())
  end
end

function libWorld.entry()
	wcm.set_robot_use_imu_yaw(Config.world.use_imu_yaw and 1 or 0)
	t_entry = unix.time()
  -- Initialize the pose filter
  -- poseFilter.initialize_unified()
  poseFilter.initialize()
  -- Save this resampling time
  t_resample = t_entry

  -- Set the initial odometry
  wcm.set_robot_pose({0,0,0})
  wcm.set_robot_odometry({0,0,0})
  wcm.set_robot_traj_num(0)
  wcm.set_obstacle_num(0)
  wcm.set_ball_notvisible(0)

  libWorld.pose_reset()
  -- Processing count
  count = 0
end

local function print_pose()
  if not Config.debug.world then return end
  local pose = wcm.get_robot_pose()
  local gpspose1 = wcm.get_robot_pose_gps()
  local gpspose0 = wcm.get_robot_pose_gps0()
  local gpspose = util.pose_relative(gpspose1,gpspose0)
  print(string.format(
    "pose: %.3f %.3f %d gps: %.3f %.3f %d",
    pose[1],pose[2],pose[3]*180/math.pi,
    gpspose[1],gpspose[2],gpspose[3]*180/math.pi))
  local uTorso = mcm.get_status_uTorso()
  print("uTOrso:",unpack(uTorso))
end

function libWorld.update(uOdom, detection)
  local t = unix.time()
  -- Run the updates
  if wcm.get_robot_reset_pose()==1 then
    print("libWorld | POSE RESET!")
    libWorld.pose_reset()
  end
  if IS_WEBOTS and Config.use_gps_pose then
    local gpspose1 = wcm.get_robot_pose_gps()
    local gpspose0 = wcm.get_robot_pose_gps0()
    local gpspose = util.pose_relative(gpspose1,gpspose0)

    --subtract automatic compensation
    comoffset = mcm.get_stance_COMoffset()
    comoffset[3]=0
    gpspose = util.pose_global(comoffset,gpspose)
    wcm.set_robot_pose(gpspose)
    print_pose()
  elseif wcm.get_robot_reset_pose()==1 or (gcm.get_game_state()~=3 and gcm.get_game_state()~=6) then
    if gcm.get_game_role()==0 then
      --Goalie initial pos
      local factor2 = 0.99
      poseFilter.initialize({-Config.world.xBoundary*factor2,0,0},{0,0,0})
      wcm.set_robot_pose({-Config.world.xBoundary*factor2,0,0})
      wcm.set_robot_odometry({-Config.world.xBoundary*factor2,0,0})
    else --Attacker initial pos
      poseFilter.initialize({0,0,0},{0,0,0})
      wcm.set_robot_pose({0,0,0})
      wcm.set_robot_odometry({0,0,0})
    end

    if use_imu_yaw then
      if IS_WEBOTS then
        gps_pose = wcm.get_robot_pose_gps()
        yaw0 = gps_pose[3]
      else
        local yaw = Body.get_rpy()[3]
        yaw0 = yaw
      end
    end
  else
    update_odometry(uOdom)
    print_pose()
  end

  update_vision(detection)


  if Config.use_gps_pose then
    wcm.set_robot_pose(wcm.get_robot_pose_gps())
    wcm.set_obstacle_num(2)
    wcm.set_obstacle_v1(wcm.get_robot_gpsobs1())
    wcm.set_obstacle_v2(wcm.get_robot_gpsobs2())

    local pose = wcm.get_robot_pose_gps()
    local ballglobal = wcm.get_robot_gpsball()
    wcm.set_robot_ballglobal(ballglobal)

    local balllocal = util.pose_relative({ballglobal[1],ballglobal[2],0},pose)
    wcm.set_ball_x(balllocal[1])
    wcm.set_ball_y(balllocal[2])
    wcm.set_ball_t(Body.get_time())

--    print("global ball:",unpack(ballglobal))
  else

    local pose =wcm.get_robot_pose()
    local ball_x = wcm.get_ball_x()
    local ball_y = wcm.get_ball_y()

    local ballglobal = util.pose_global({ball_x,ball_y,0},pose)

    wcm.set_robot_ballglobal(ballglobal)
    -- Update pose in wcm
    wcm.set_robot_pose(vector.pose{poseFilter.get_pose()})
  end


  -- Increment the process count
  count = count + 1
end


function libWorld.send()
  local to_send = {}
  to_send.info = ''
  -- Robot info
  to_send.pose = vector.new(wcm.get_robot_pose())
  to_send.time = Body.get_time()
  to_send.info = to_send.info..string.format(
    'Pose: %.2f %.2f (%.1f)\n', to_send.pose[1], to_send.pose[2], to_send.pose[3]*RAD_TO_DEG)
  to_send.time = Body.get_time()
  return to_send
end

function libWorld.exit()
end

function libWorld.get_pose()
--TODO
--  return wcm.get_robot_pose(wcm.get_robot_pose_gps())
  --return vector.pose({0,0,0})
  --return vector.pose{poseFilter.get_pose()}
  return wcm.get_robot_pose()
end

libWorld.update_odometry = update_odometry
libWorld.update_vision = update_vision

return libWorld

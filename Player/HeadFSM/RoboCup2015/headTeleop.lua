local Body = require'Body'
local t_entry, t_update
local state = {}
state._NAME = ...
require'hcm'
require'gcm'
local util = require'util'

-- Neck limits
local dqNeckLimit = Config.fsm.dqNeckLimit or {60*DEG_TO_RAD, 60*DEG_TO_RAD}

function state.entry()
  print(state._NAME..' Entry' ) 
  -- When entry was previously called
  local t_entry_prev = t_entry
  -- Update the time of entry
  t_entry = Body.get_time()
  t_update = t_entry
  -- Reset the human position
  hcm.set_motion_headangle(Body.get_head_position())
  wcm.set_ball_disable(0)  
  wcm.set_goal_disable(0)
end

function state.update()
  -- print(_NAME..' Update' )
  -- Get the time of update
  local t = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t 

  -- Grab the target
 -- local neckAngleTarget = hcm.get_motion_headangle()

  --HACK FOR ROBOCUP
  local neckAngleTarget = {0,Config.fsm.headObstacleScan.pitchUp}
  if gcm.get_game_role()==3 then --look down when testing
    neckAngleTarget = {0,45*DEG_TO_RAD}
  end

  --print('Neck angle',unpack(neckAngleTarget))
  -- Grab where we are
  local qNeck = Body.get_head_command_position()
  local headBias = hcm.get_camera_bias()
  qNeck[1] = qNeck[1] - headBias[1]  


  -- Go!
  local qNeck_approach, doneNeck = 
    util.approachTol( qNeck, neckAngleTarget, dqNeckLimit, dt )
  -- Update the motors



  local headBias = hcm.get_camera_bias()
  Body.set_head_command_position({qNeck_approach[1]+headBias[1],qNeck_approach[2]})

  --5 is the idle state! 
  -- Ready, set, playing states

  --Game playing, get out of teleop state 
  if gcm.get_game_state()==3 then 
    --print("Headteleop: game state",gcm.get_game_state())
    if Config.enable_obstacle_scan then
      return 'scanobs'
    else
      return 'scan' 
    end
  end

end

function state.exit()  
  local t = Body.get_time()
  
  print(state._NAME..' Exit, total time ',t-t_entry )
end

return state

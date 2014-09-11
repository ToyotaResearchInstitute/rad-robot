--Stance state is basically a Walk controller
--Without any torso or feet update
--We share the leg joint generation / balancing code 
--with walk controllers

local state = {}
state._NAME = ...

local Body   = require'Body'
local K      = Body.Kinematics
local vector = require'vector'
local unix   = require'unix'
local util   = require'util'
local moveleg = require'moveleg'
require'mcm'

-- Keep track of important times
local t_entry, t_update, t_last_step


local qLArm0, qRArm0,qLLeg0,qRLeg0,qWaist0
local qLArm1, qRArm1,qLLeg1,qRLeg1,qWaist1

local keyframe={
  {
    qLArm = {150,0,0,-140,   90,70,0},
    qRArm = {150,0,0,-140,   -90,-70,0},
    qWaist={0,0},
    duration = 2.0,
  },
  {    
    qLArm = {130,0,0,-140,   90,70,0},
    qRArm = {130,0,0,-140,   -90,-70,0},    
    qLLeg={0,0,15,30,-100,0},
    qRLeg={0,0,15,30,-100,0},
    duration = 4.0,
  },

  {    
    qLArm = {0,0,0,-20,   90,85,0},
    qRArm = {0,0,0,-20,   -90,-85,0},    
    qLLeg={0,0,-45,90,-100,0},
    qRLeg={0,0,-45,90,-100,0},
    qWaist={0,-45},
    duration = 4.0,
  },
  
 {    
    qLArm = {-20,0,0,-20,   90,  60,0},
    qRArm = {-20,0,0,-20,   -90, -60,0},    
    qLLeg={0,0,-85,160,-100,0},
    qRLeg={0,0,-85,160,-100,0},
    qWaist={0,-60},
    duration = 4.0,
  },
  
--larm fold back 
  {    
    qLArm = {40,0,0,  -100,   90,  30,0},    
    duration = 2.0,
  },
--larm place 
  {    
    qLArm = {35,0,0,  -70,   90,  0,0},    
    duration = 2.0,
  },

--rarm fold back 
  {    
    qRArm = {40,0,0,  -100,   -90,  -30,0},    
    duration = 2.0,
  },
--rarm place 
  {    
    qRArm = {35,0,0,  -70,   -90,  0,0},    
    duration = 2.0,
  },


--both arm push
  {    
    qLArm = {50,0,0,  -20,   90,  0,0},    
    qRArm = {50,0,0,  -20,   -90,  0,0},    
    qWaist={0,-20},
    duration = 4.0,
  },

--arm backward
  {    
    qLArm = {130,0,0,  0,   90,  0,0},    
    qRArm = {130,0,0,  0,   -90,  0,0},            
    qWaist={0,-0},
    duration = 4.0,
  },

--torso straight
  {    
    qLLeg={0,0,-50,160,-100,0},
    qRLeg={0,0,-50,160,-100,0},
    duration = 4.0,
  },

--Stand up
  {    
    qLArm = {140, 21, 5, -98, 42, 16, -62},    
    qRArm = {140, -21, 5, -98, -42, -16, -62},    
    qLLeg={0,0,-40,80,-45,0},
    qRLeg={0,0,-40,80,-45,0},
    duration = 4.0,
  }
}

local stage
local DEG_TO_RAD = math.pi/180
---------------------------
-- State machine methods --
---------------------------
function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_start = t_entry
  
  qLArm1 = vector.new(Body.get_larm_command_position())  
  qRArm1 = vector.new(Body.get_rarm_command_position())  
  qLLeg1 = vector.new(Body.get_lleg_command_position())  
  qRLeg1 = vector.new(Body.get_rleg_command_position())  
  qWaist1 = vector.new(Body.get_waist_command_position())  

print(unpack(vector.new(qLArm1)/DEG_TO_RAD))

  stage = 0  
end

function state.update()
  -- Get the time of update
  local t = Body.get_time()
  local t_diff = t - t_start
  if stage > #keyframe then return "done" end

  if stage>0 then    
    local ph = math.min(1, t_diff/keyframe[stage].duration)
    Body.set_larm_command_position((1-ph)* qLArm0 + ph * qLArm1)
    Body.set_rarm_command_position((1-ph)*qRArm0 + ph * qRArm1)
    Body.set_lleg_command_position((1-ph)*qLLeg0 + ph * qLLeg1)
    Body.set_rleg_command_position((1-ph)*qRLeg0 + ph * qRLeg1)
    Body.set_waist_command_position((1-ph)*qWaist0 + ph * qWaist1)
  end

  if stage==0 or 
   (t_diff>keyframe[stage].duration and hcm.get_state_proceed()==1) 
    then
--    hcm.set_state_proceed(0)
    t_start=t
    stage = stage +1
    if stage > #keyframe then return "done" end

    qLArm0 = qLArm1
    qRArm0 = qRArm1
    qLLeg0 = qLLeg1
    qRLeg0 = qRLeg1
    qWaist0 = qWaist1
    if keyframe[stage].qLArm then qLArm1 = vector.new(keyframe[stage].qLArm)*DEG_TO_RAD end
    if keyframe[stage].qRArm then qRArm1 = vector.new(keyframe[stage].qRArm)*DEG_TO_RAD end
    if keyframe[stage].qLLeg then qLLeg1 = vector.new(keyframe[stage].qLLeg)*DEG_TO_RAD end
    if keyframe[stage].qRLeg then qRLeg1 = vector.new(keyframe[stage].qRLeg)*DEG_TO_RAD end
    if keyframe[stage].qWaist then qWaist1 = vector.new(keyframe[stage].qWaist or qWaist0)*DEG_TO_RAD end
  end
end -- walk.update

function state.exit()
  print(state._NAME..' Exit')
  -- TODO: Store things in shared memory?
end

return state

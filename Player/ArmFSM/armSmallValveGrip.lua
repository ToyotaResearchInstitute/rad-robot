local state = {}
state._NAME = ...
require'hcm'
local vector = require'vector'
local util   = require'util'
local movearm = require'movearm'
local libArmPlan = require 'libArmPlan'
local arm_planner = libArmPlan.new_planner()
local T      = require'Transform'

--Initial hand angle
local lhand_rpy0 = {0,0*Body.DEG_TO_RAD, 0*Body.DEG_TO_RAD}
local rhand_rpy0 = {0,0*Body.DEG_TO_RAD, 0*Body.DEG_TO_RAD}

local trLArm0, trRArm0, trLArm1, trRArm1, qLArm0, qRarm0
local gripL, gripR = 1,1
local stage



local angle1, angle2

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry

  mcm.set_arm_lhandoffset(Config.arm.handoffset.chopstick)

  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()

  qLArm0 = qLArm
  qRArm0 = qRArm
  
  trLArm0 = Body.get_forward_larm(qLArm0)
  trRArm0 = Body.get_forward_rarm(qRArm0)  
  
  qLArm1 = Body.get_inverse_arm_given_wrist( qLArm, {0,0,0, unpack(lhand_rpy0)})  
  qRArm1 = Body.get_inverse_arm_given_wrist( qRArm, {0,0,0, unpack(rhand_rpy0)})
  
  trLArm1 = Body.get_forward_larm(qLArm1)
  trRArm1 = Body.get_forward_rarm(qRArm1)  

  arm_planner:set_shoulder_yaw_target(nil,qRArm0[3])--unlock left shoulder

  hcm.set_largevalve_model(Config.armfsm.valveonearm.default_model_small)

  hcm.set_state_tstartactual(unix.time()) 
  hcm.set_state_tstartrobot(Body.get_time())

  local wrist_seq = {{'wrist',trLArm1, nil}}
  if arm_planner:plan_arm_sequence(wrist_seq) then stage = "wristturn" end
  hcm.set_state_proceed(1)
end

local function update_model()
  local trLArmTarget = hcm.get_hands_left_tr_target()
  local trLArm = hcm.get_hands_left_tr()
  local valve_model = hcm.get_largevalve_model()
  valve_model[1],valve_model[2],valve_model[3]=
    valve_model[1] + trLArmTarget[1]-trLArm[1],
    valve_model[2] + trLArmTarget[2]-trLArm[2],
    valve_model[3] + trLArmTarget[3]-trLArm[3]
  hcm.set_largevalve_model(valve_model)

  print("Valve update: pos ",valve_model[1],valve_model[2],valve_model[3])

  angle1, angle2 = valve_model[5], valve_model[6]
  hcm.set_state_proceed(0)
end

local function update_override()
  local overrideTarget = hcm.get_state_override_target()
  local override = hcm.get_state_override()
  local valve_model = hcm.get_largevalve_model()

  valve_model[1],valve_model[2],valve_model[3], valve_model[4], valve_model[5]=
    valve_model[1] + overrideTarget[1]-override[1],
    valve_model[2] + overrideTarget[2]-override[2],
    valve_model[3] + overrideTarget[3]-override[3],
    0, --This should be the radius 
    valve_model[5] + overrideTarget[5]-override[5] --this is the init angle
    
  hcm.set_largevalve_model(valve_model)
  hcm.set_state_override({valve_model[1],valve_model[2],valve_model[3],valve_model[4],valve_model[5]}) -- 5 elements

  print( util.color('Valve model:','yellow'), 
      string.format("%.2f %.2f %.2f / %.1f",
      valve_model[1],valve_model[2],valve_model[3],
      valve_model[4]*180/math.pi   ))

  angle1, angle2 = valve_model[5], valve_model[6]
  hcm.set_state_proceed(0)
end





function state.update()
--  print(state._NAME..' Update' )
  -- Get the time of update  
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  --if t-t_entry > timeout then return'timeout' end
  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()
  local trLArm = Body.get_forward_larm(qLArm)
  local trRArm = Body.get_forward_rarm(qRArm)  
  
  if stage=="wristturn" then --Turn yaw angles first
    gripL,doneL = util.approachTol(gripL,1,2,dt)  --Close gripper
    gripR,doneL = util.approachTol(gripR,1,2,dt)  --Close gripper
    Body.set_lgrip_percent(gripL*0.8)
    Body.set_rgrip_percent(gripR*0.8)

    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then 
        local model = hcm.get_largevalve_model()
        local arm_seq
        if model[3]>Config.armfsm.valveonearm.heights[2] then --high
          arm_seq = {
            {'move',Config.armfsm.valveonearm.arminit[1], nil},
            {'move',Config.armfsm.valveonearm.arminit[2], nil},
            {'move',Config.armfsm.valveonearm.arminit[3], nil},
          }
        elseif model[3]>Config.armfsm.valveonearm.heights[1] then --mid
          arm_seq = {
            {'move',Config.armfsm.valveonearm.arminit[1], nil},
            {'move',Config.armfsm.valveonearm.arminit[2], nil},           
          }
        else --low
          arm_seq = {{'move',Config.armfsm.valveonearm.arminit[1], nil}}
        end        
        if arm_planner:plan_arm_sequence(arm_seq) then stage="armready" end        
      elseif hcm.get_state_proceed()==-1 then 
        arm_planner:set_shoulder_yaw_target(qLArm0[3],qRArm0[3])
        local wrist_seq = {{'wrist',trLArm0, nil}}
        if arm_planner:plan_arm_sequence(wrist_seq) then stage = "armbacktoinitpos" end
      end
    end
  elseif stage=="armready" then        
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then 
        update_model()
        hcm.set_state_proceed(1)  
        local trLArmTarget = movearm.getLargeValvePositionSingle(angle1,Config.armfsm.valveonearm.clearance, 1)
        local arm_seq = {{'move',trLArmTarget, nil}}
        if arm_planner:plan_arm_sequence(arm_seq) then stage="pregrip" end
      elseif hcm.get_state_proceed()==-1 then               
        --local arm_seq = {{'move',trLArm1, nil}}
        local arm_seq
        local model = hcm.get_largevalve_model()
        if model[3]>Config.armfsm.valveonearm.heights[2] then --high
          arm_seq = {{'move',Config.armfsm.valveonearm.arminit[2], nil},
              {'move',Config.armfsm.valveonearm.arminit[1], nil},
              {'move',trLArm1, nil},}
        elseif model[3]>Config.armfsm.valveonearm.heights[1] then --mid
          arm_seq = {{'move',Config.armfsm.valveonearm.arminit[1], nil},
            {'move',trLArm1, nil},}
        else
          arm_seq = {{'move',trLArm1, nil},}
        end
        if arm_planner:plan_arm_sequence(arm_seq) then stage="wristturn" end
      end
    end
  elseif stage=="pregrip" then 
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then 
        local trLArmTarget = movearm.getLargeValvePositionSingle(angle1,0,1)
        local arm_seq = {{'move',trLArmTarget, nil}}
        if arm_planner:plan_arm_sequence(arm_seq) then stage="inposition" end
      elseif hcm.get_state_proceed()==-1 then
        local model = hcm.get_largevalve_model()
        local arm_seq
        if model[3]>Config.armfsm.valveonearm.heights[2] then --high
          arm_seq = {{'move',Config.armfsm.valveonearm.arminit[3], nil}}
        elseif model[3]>Config.armfsm.valveonearm.heights[1] then --mid
          arm_seq = {{'move',Config.armfsm.valveonearm.arminit[2], nil}}
        else
          arm_seq = {{'move',Config.armfsm.valveonearm.arminit[1], nil}}
        end
        if arm_planner:plan_arm_sequence(arm_seq) then stage="armready" end
      elseif hcm.get_state_proceed()==2 then      
        update_model()
        local trLArmTarget = movearm.getLargeValvePositionSingle(angle1,Config.armfsm.valveonearm.clearance, 1)
        local arm_seq = {{'move',trLArmTarget, nil}}
        if arm_planner:plan_arm_sequence(arm_seq) then stage="pregrip" end

      elseif hcm.get_state_proceed()==3 then --NEW OVERRIDE
        update_override()
        local trLArmTarget = movearm.getLargeValvePositionSingle(angle1,Config.armfsm.valveonearm.clearance, 1)     
        local arm_seq = {{'move',trLArmTarget, nil}}
        if arm_planner:plan_arm_sequence(arm_seq) then stage="pregrip" end

      end
    end
    
  elseif stage=="inposition" then 
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then 
        arm_planner:save_valveparam({angle1,0,1,0})
        local valve_seq={          
          {'valveonearm',angle2,0,1,0},
          {'valveonearm',angle2,Config.armfsm.valveonearm.clearance,1,0},          
          {'valveonearm',angle1,Config.armfsm.valveonearm.clearance,1,0},          
        }
        if arm_planner:plan_arm_sequence(valve_seq) then stage="valveturn" end
      elseif hcm.get_state_proceed()==-1 then 
        local trLArmTarget, trRArmTarget = movearm.getLargeValvePositionSingle(
          angle1,Config.armfsm.valveonearm.clearance,1,0)
        
        local arm_seq = {{'move',trLArmTarget, trRArmTarget}}
        if arm_planner:plan_arm_sequence(arm_seq) then stage="pregrip" end
      elseif hcm.get_state_proceed()==2 then --teleop signal
        print("update")
        update_model()
        local trLArmTarget = movearm.getLargeValvePositionSingle(angle1,0,1)
        local arm_seq = {{'move',trLArmTarget, nil}}
        if arm_planner:plan_arm_sequence(arm_seq) then stage="inposition" end

      elseif hcm.get_state_proceed()==3 then --NEW OVERRIDE
        update_override()
        local trLArmTarget = movearm.getLargeValvePositionSingle(angle1,0, 1)     
        local arm_seq = {{'move',trLArmTarget, nil}}
        if arm_planner:plan_arm_sequence(arm_seq) then stage="inposition" end

      end
    end
    hcm.set_state_proceed(0)
  elseif stage=="valveturn" then 
    if arm_planner:play_arm_sequence(t) then 
      hcm.set_state_success(1) --Report success
      stage="pregrip"
    end
  elseif stage=="armbacktoinitpos" then 
    if arm_planner:play_arm_sequence(t) then return "done" end
  end  
end

function state.exit()    
  print(state._NAME..' Exit' )
end

return state

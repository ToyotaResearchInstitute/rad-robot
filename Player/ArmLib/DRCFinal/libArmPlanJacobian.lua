-- libArmPlan
-- (c) 2013 Seung-Joon Yi
-- Arm movement planner

local vector = require'vector'
local util = require'util'
require'mcm'

local movearm = require'movearm'
local libTransform = require'libTransform'
local sformat = string.format
local K = Body.Kinematics
--debug_on = true
debug_on = false
debug_on_2 = false
debugmsg = true


--print(unpack(Config.arm.iklookup.x))

local function tr_dist(trA,trB)
  return math.sqrt(  (trA[1]-trB[1])^2+(trA[2]-trB[2])^2+(trA[3]-trB[3])^2)
end

local function movfunc(cdist,dist)
  if dist==0 then return 0 end  

  
  local min_vel = 0.02 --min speed: 2 cm/s
  local max_vel = 0.04 --max speed: 8 cm/s
  local ramp1 = 0.04 --accellerate over 4cm
  local ramp2 = 0.04 --desccellerate over 4cm

  local vel = math.min(
    max_vel,
    min_vel + (max_vel-min_vel) * (cdist/ramp1), 
    min_vel + (max_vel-min_vel) * ((dist-cdist)/ramp2) 
    )
  vel = math.max(min_vel,vel)
  return vel
end


local function print_arm_plan(arm_plan)
  print("Arm plan: total ", #arm_plan.LAP)  
  --  local arm_plan = {LAP = LAPs, RAP = RAPs,  uTP =uTPs, WP=WPs}
end


local function get_admissible_dt_vel(qMovement, velLimit)
  local dt_min = -math.huge
  for j = 1,7 do dt_min = math.max(dt_min, util.mod_angle(qMovement[j])/velLimit[j]) end
  return dt_min
end

--now linear joint level accelleration
local function filter_arm_plan(plan)
  local num = #plan.LAP
  local velLimit = dqArmMax
  local t0 =unix.time()
  
  local velLimit0 = Config.arm.vel_angular_limit
  local accLimit0 = vector.new({20,20,20,20,30,30,30})*DEG_TO_RAD --accelleration value
  local accMax = vector.new({60,60,60,60,90,90,90})

  local dt = {}
  local qLArmMov,qRArmMov = {},{}
  local qLArmVel,qRArmVel = {},{}

  --Initial filtering based on max velocity
  for i=1,num-1 do    
    qLArmMov[i] =  vector.new(plan.LAP[i+1][1]) - vector.new(plan.LAP[i][1])
    qRArmMov[i] =  vector.new(plan.RAP[i+1][1]) - vector.new(plan.RAP[i][1])
    local dt_min_left  = get_admissible_dt_vel(qLArmMov[i], velLimit0)
    local dt_min_right = get_admissible_dt_vel(qRArmMov[i], velLimit0)
--    dt[i] = math.max(dt_min_left, dt_min_right,0.01)
    dt[i] = math.max(dt_min_left, dt_min_right, Config.arm.plan.dt_step_min)
    qLArmVel[i] = qLArmMov[i]/dt[i]    
    qRArmVel[i] = qLArmMov[i]/dt[i]
  end

  local total_time1,total_time2=0,0

  for i=1,num-1 do 
    total_time1=total_time1+plan.LAP[i][2]
    plan.LAP[i][2] = dt[i] 
    total_time2 = total_time2+dt[i]
  end

  local t1 =unix.time()
  print(sformat("%d segments, Filtering time: %.2f ms\nExecution time Org: %.fs : Filtered %.1fs",
    num-1,(t1-t0)*1000,total_time1,total_time2))  
end

local function set_hand_mass(self,mLeftHand, mRightHand) self.mLeftHand, self.mRightHand = mLeftHand, mRightHand end

local function get_torso_compensation(qLArm,qRArm,qWaist,massL,massR)
  local uLeft = mcm.get_status_uLeft()
  local uRight = mcm.get_status_uRight()
  local uTorso = mcm.get_status_uTorso()
  local zLeg = mcm.get_status_zLeg()
  local zSag = mcm.get_walk_zSag()
  local zLegComp = mcm.get_status_zLegComp()
  local zLeft,zRight = zLeg[1]+zSag[1]+zLegComp[1],zLeg[2]+zSag[2]+zLegComp[2]


  local pLLeg = vector.new({uLeft[1],uLeft[2],zLeft,0,0,uLeft[3]})
  local pRLeg = vector.new({uRight[1],uRight[2],zRight,0,0,uRight[3]})  
  local aShiftX = mcm.get_walk_aShiftX()
  local aShiftY = mcm.get_walk_aShiftY()
  local torsoX    = Config.walk.torsoX

  local count,revise_max = 1,4
  local adapt_factor = 1.0

 --Initial guess 
  local uTorsoAdapt = util.pose_global(vector.new({-torsoX,0,0}),uTorso)
  local pTorso = vector.new({
    uTorsoAdapt[1], uTorsoAdapt[2], mcm.get_stance_bodyHeight(),
            0,mcm.get_stance_bodyTilt(),uTorsoAdapt[3]})
  local qLegs = K.inverse_legs(pLLeg, pRLeg, pTorso,aShiftX,aShiftY)
  
  -------------------Incremental COM filtering
  while count<=revise_max do
    local qLLeg = vector.slice(qLegs,1,6)
    local qRLeg = vector.slice(qLegs,7,12)
    com = K.calculate_com_pos(qWaist,qLArm,qRArm,qLLeg,qRLeg,0,0,0)
    local uCOM = util.pose_global(
      vector.new({com[1]/com[4], com[2]/com[4],0}),uTorsoAdapt)

   uTorsoAdapt[1] = uTorsoAdapt[1]+ adapt_factor * (uTorso[1]-uCOM[1])
   uTorsoAdapt[2] = uTorsoAdapt[2]+ adapt_factor * (uTorso[2]-uCOM[2])
   local pTorso = vector.new({
            uTorsoAdapt[1], uTorsoAdapt[2], mcm.get_stance_bodyHeight(),
            0,mcm.get_stance_bodyTilt(),uTorsoAdapt[3]})
   qLegs = K.inverse_legs(pLLeg, pRLeg, pTorso, aShiftX, aShiftY)
   count = count+1
  end
  local uTorsoOffset = util.pose_relative(uTorsoAdapt, uTorso)
  return {uTorsoOffset[1],uTorsoOffset[2]}
  
end

local function reset_torso_comp(self,qLArm,qRArm)
  local qWaist = Body.get_waist_command_position()
  self.torsoCompBias = get_torso_compensation(qLArm,qRArm,qWaist,0,0)  
  mcm.set_stance_uTorsoCompBias(self.torsoCompBias)  
  self:save_boundary_condition({qLArm,qRArm,qLArm,qRArm,{0.0}})
end

local function get_armangle_jacobian(self,qArm,trArmTarget,isLeft, qWaist,dt_step, linear_vel, debug)  
  if not qArm or not trArmTarget then return end
  local handOffsetY=Config.arm.handoffset.gripper3[2]  --positive Y value is inside
  local trArm
  
  if isLeft ==1 then 
    handOffsetY=-handOffsetY 
    trArm= Body.get_forward_larm(qArm)
  else
    trArm= Body.get_forward_rarm(qArm)
  end
  local JacArm=K.calculate_arm_jacobian(
    qArm,qWaist,
    {0,0,0}, --rpy angle
    isLeft,
    Config.arm.handoffset.gripper3[1],
    handOffsetY,
    Config.arm.handoffset.gripper3[3]
    );  --tool xyz
  local trArmDiff = util.diff_transform(trArmTarget,trArm)  

--Calculate target velocity
  local trArmVelTarget={
    0,0,0,
    util.procFunc(-trArmDiff[4],0,30*math.pi/180),
    util.procFunc(-trArmDiff[5],0,30*math.pi/180),
    util.procFunc(-trArmDiff[6],0,30*math.pi/180),
  }  
  local linear_dist = util.norm(trArmDiff,3)
  local total_angular_vel = 
     math.abs(trArmVelTarget[4])+math.abs(trArmVelTarget[5])+math.abs(trArmVelTarget[6])
  
  if linear_dist>0 then
    trArmVelTarget[1],trArmVelTarget[2],trArmVelTarget[3]=
    trArmDiff[1]/linear_dist *linear_vel,
    trArmDiff[2]/linear_dist *linear_vel,
    trArmDiff[3]/linear_dist *linear_vel    
  end
    
  if linear_dist<0.001 and total_angular_vel<1*math.pi/180 then 
--    print("reached")
    return qArm,true 
    end --reached

  local J= torch.Tensor(JacArm):resize(6,7)  
  local JT = torch.Tensor(J):transpose(1,2)
  local e = torch.Tensor(trArmVelTarget)
  local I = torch.Tensor():resize(7,7):zero()
  local I2 = torch.Tensor():resize(7,6):zero()
  local qArmVel = torch.Tensor(7):fill(0)
  
  -- lambda_i = c*((2*q-qmin-qmax)/(qmax-qmin))^p + (1/w_i)
  -- todo: hard limit joint angles

  local lambda=torch.eye(7)
  local c, p = 2,10  

  local joint_limits={
    {-math.pi/2, math.pi},
    {0,math.pi/2},
    {-math.pi/2, math.pi/2},
    {-math.pi, -0.2}, --temp value
    {-math.pi, math.pi},
    {-math.pi/2, math.pi/2},
    {-math.pi, math.pi}
  }
  if isLeft==0 then
    joint_limits[2]={-math.pi/2,0}
  end

  for i=1,7 do
    lambda[i][i]=0.1*0.1 + c*
      ((2*qArm[i]-joint_limits[i][1]-joint_limits[i][2])/
       (joint_limits[i][2]-joint_limits[i][1]))^p
  end

  I:addmm(JT,J):add(1,lambda)
  local Iinv=torch.inverse(I)  
  I2:addmm(Iinv,JT)   
  qArmVel:addmv(I2,e)
  local qArmTarget = vector.new(qArm)+vector.new(qArmVel)*dt_step

  if debug and isLeft==0 then
    local trArmNext = Body.get_forward_rarm(qArmTarget)
    local trArmDiffActual = util.diff_transform(trArmNext,trArm)
    local linear_dist = util.norm(trArmDiffActual,3)

    print("---")
    print("Target:"..util.print_transform(trArmTarget))
    print("Current"..util.print_transform(trArmNext))
    print(sformat(" dist:%.3f vel:T%.3f A:%.3f",
        linear_dist,linear_vel,linear_dist/dt_step)
      )
  end
  return qArmTarget,false
end


local function get_next_movement_jacobian(self, init_cond, trLArm1,trRArm1, dt_step, waistYaw, waistPitch, velL, velR)

  local default_hand_mass = Config.arm.default_hand_mass or 0
  local dqVelLeft = mcm.get_arm_dqVelLeft()
  local dqVelRight = mcm.get_arm_dqVelRight()
  local velTorsoComp = Config.arm.torso_comp_limit
  local velYaw = Config.arm.shoulder_yaw_limit
  local massL, massR = self.mLeftHand + default_hand_mass, self.mRightHand + default_hand_mass
  local qLArm,qRArm, qLArmComp , qRArmComp, uTorsoComp = unpack(init_cond)
  
  local qLArmNext, qRArmNext = qLArm, qRArm
  local doneL,doneR = true,true
  local qWaist = {waistYaw, waistPitch}
  local endpoint_compensation = mcm.get_arm_endpoint_compensation()

  if endpoint_compensation[1]>0 then
    qLArmNext,doneL = self:get_armangle_jacobian(qLArm,trLArm1, 1, qWaist,dt_step, velL,true)
  end

  if endpoint_compensation[2]>0 then
    qRArmNext,doneR = self:get_armangle_jacobian(qRArm,trRArm1, 0, qWaist,dt_step, velR,true)
  end

  if not qLArmNext or not qRArmNext then return end
  local trLArmNext = Body.get_forward_larm(qLArmNext,mcm.get_stance_bodyTilt(),qWaist)
  local trRArmNext = Body.get_forward_rarm(qRArmNext,mcm.get_stance_bodyTilt(),qWaist)

  local vec_comp = vector.new({-uTorsoComp[1],-uTorsoComp[2],0,0,0,0})


--hack - no compensation
vec_comp=vector.zeros(6)



  local trLArmNextComp = vector.new(trLArmNext) + vec_comp
  local trRArmNextComp = vector.new(trRArmNext) + vec_comp


  local qLArmNextComp, qRArmNextComp = qLArmNext, qRArmNext

  --Actual arm angle considering the torso compensation
  if endpoint_compensation[1]>0 then
    qLArmNextComp = self:get_armangle_jacobian(qLArmComp,trLArmNextComp, 1, qWaist,dt_step,velL)
  end
  if endpoint_compensation[2]>0 then
    qRArmNextComp = self:get_armangle_jacobian(qRArmComp,trRArmNextComp, 0, qWaist,dt_step,velR)
  end

  if not qLArmNextComp or not qRArmNextComp or not qLArmNext or not qRArmNext then 
    print("ERROR")
    return 
  else
    local dt_step_current = dt_step
    local uTorsoCompNextTarget = get_torso_compensation(qLArmNext,qRArmNext,qWaist, massL,massR)
    local uTorsoCompNext, torsoCompDone = util.approachTol(uTorsoComp, uTorsoCompNextTarget, velTorsoComp, dt_step_current )
    local new_cond = {qLArmNext, qRArmNext, qLArmNextComp, qRArmNextComp, uTorsoCompNext, waistYaw, waistPitch}
    return new_cond, dt_step_current, 
          torsoCompDone and doneL and doneR,
          trLArmNext, trRArmNext
  end
end


local function plan_unified(self, plantype, init_cond, init_param, target_param)
  local dpVelLeft = mcm.get_arm_dpVelLeft()
  local dpVelRight = mcm.get_arm_dpVelRight()

  local t00 = unix.time()


  --param: {trLArm,trRArm} for move
  --param: {trLArm,trRArm} for wrist

  if not init_cond then return end
  local done, failed = false, false, false

  local qWaist = {init_cond[6] or Body.get_waist_command_position()[1],init_cond[7] or Body.get_waist_command_position()[2]}
  local current_cond = {init_cond[1],init_cond[2],init_cond[3],init_cond[4],{init_cond[5][1],init_cond[5][2]},qWaist[1],qWaist[2]}
  local trLArm = Body.get_forward_larm(init_cond[1],mcm.get_stance_bodyTilt(),qWaist) 
  local trRArm =Body.get_forward_rarm(init_cond[2],mcm.get_stance_bodyTilt(),qWaist) 


  local qLArm0, qRArm0 = init_cond[1],init_cond[2]

  --Insert initial arm joint angle to the queue
  local dt_step0 = Config.arm.plan.dt_step0
  local dt_step = Config.arm.plan.dt_step
  local qLArmQueue,qRArmQueue, uTorsoCompQueue = {{init_cond[3],dt_step0}}, {{init_cond[4],dt_step0}}, {init_cond[5]}

--now pitch too
  local qWaistQueue={{current_cond[6],current_cond[7]}}

  local current_param={unpack(init_param)}
  local qArmCount = 2
  local vel_param  

  if plantype=="move" then
    current_param[3] = current_cond[6]
    current_param[4] = current_cond[7]
    target_param[3] = target_param[3] or current_cond[6]
    target_param[4] = target_param[4] or current_cond[7]

    vel_param = {dpVelLeft,dpVelRight,Config.arm.vel_waist_limit[1],Config.arm.vel_waist_limit[2]}
  elseif plantype=="wrist" then

  end
  
  local done, torsoCompDone = false, false
  local trLArmNext, trRArmNext, waistNext
  local new_param = {}

  local t0 = unix.time()  
  local t_robot = 0
  local done2 = false

  while not done and not failed  do --we were skipping the last frame

    local t01 = unix.time()
    local time_passed=t01-t00    
    if time_passed>1 then
      print("SOMETHING VERY WRONG!!!!!!")
      return
    end
    local new_cond, dt_step_current, torsoCompDone, t10, t11
    local distL = tr_dist(init_param[1],target_param[1])
    local distR = tr_dist(init_param[2],target_param[2])
    if plantype=="move" then
      local cdistL = tr_dist(init_param[1],trLArm)
      local cdistR = tr_dist(init_param[2],trRArm)
      local velL,velR = movfunc(cdistL,distL),movfunc(cdistR,distR)

      --Waist yaw and pitch
      new_param[3],done3 = util.approachTol(current_param[3],target_param[3],vel_param[3],dt_step )
      new_param[4],done4 = util.approachTol(current_param[4],target_param[4],vel_param[4],dt_step )
      waistNext = {new_param[3], new_param[4]}
  
      t10 = unix.time() 
      new_cond, dt_step_current, torsoCompDone, trLArmNext, trRArmNext=    
        self:get_next_movement_jacobian(
          current_cond, target_param[1],target_param[2], dt_step, waistNext[1], waistNext[2], velL, velR)
      t11 = unix.time() 
      done = done3 and done4

    elseif plantype=="wrist" then
      trLArmNext,doneL = util.approachTolWristTransform(trLArm, target_param[1], dpVelLeft, dt_step )      
      trRArmNext,doneR = util.approachTolWristTransform(trRArm, target_param[2], dpVelRight, dt_step )
      done = doneL and doneR
      local cdistL = tr_dist(init_param[1],trLArmNext)
      local cdistR = tr_dist(init_param[2],trRArmNext)
      local velL,velR = 0.02, 0.02 --for some reason, accelleration does not work very well with this
      local qLArmTemp = Body.get_inverse_arm_given_wrist( qLArm0, trLArmNext)
      local qRArmTemp = Body.get_inverse_arm_given_wrist( qRArm0, trRArmNext)
      trLArmNext = Body.get_forward_larm(qLArmTemp)
      trRArmNext = Body.get_forward_rarm(qRArmTemp)  
      waistNext = {current_cond[6], current_cond[7]}

      t10 = unix.time() 
      new_cond, dt_step_current, torsoCompDone=    
        self:get_next_movement_jacobian(current_cond, trLArmNext, trRArmNext, dt_step, waistNext[1], waistNext[2],velL,velR)
      t11 = unix.time()
    end


    done = done and torsoCompDone
    if not new_cond then       
      failed = true    
    else
      t_robot = t_robot + dt_step_current
      trLArm, trRArm = trLArmNext, trRArmNext
      qLArmQueue[qArmCount] = {new_cond[3],dt_step_current}
      qRArmQueue[qArmCount] = {new_cond[4],dt_step_current}
      qWaistQueue[qArmCount] = waistNext 
      uTorsoCompQueue[qArmCount] = {new_cond[5][1],new_cond[5][2]}      
      current_cond = new_cond
      current_param = new_param
      qArmCount = qArmCount + 1
    end    
  end

  local t1 = unix.time()  

  if failed then return end
  
  if debug_on_2 then
    print("trLArm:",self.print_transform( Body.get_forward_larm( qLArmQueue[1][1]  ) ))
    print("trRArm:",self.print_transform( Body.get_forward_rarm( qRArmQueue[1][1]  )))
    print(string.format("TorsoComp: %.3f %.3f",uTorsoCompQueue[1][1],uTorsoCompQueue[1][2]) )
  end


  return qLArmQueue,qRArmQueue, uTorsoCompQueue, qWaistQueue, current_cond, current_param
end



local function plan_arm_sequence(self,arm_seq, current_stage_name,next_stage_name)
  --This function plans for a arm sequence using multiple arm target positions
  --and initializes the playback if it is possible
  
--  {
--    {'move', trLArm, trRArm}  : move arm transform to target
--    {'wrist', trLArm, trRArm} : rotate wrist to target transform
--    {'valve', }      
--  }
  local t0 =unix.time()


  local init_cond = self:load_boundary_condition()
  local LAPs, RAPs, uTPs, WPs = {},{},{},{}
  local counter = 1

  for i=1,#arm_seq do
    local trLArm = Body.get_forward_larm(init_cond[1])
    local trRArm = Body.get_forward_rarm(init_cond[2])
    local LAP, RAP, uTP,end_cond
    local WP, end_doorparam --for door
    if arm_seq[i][1] =='move' then
      LAP, RAP, uTP, WP, end_cond  = self:plan_unified('move',
        init_cond,   {trLArm,trRArm},
        {arm_seq[i][2] or trLArm, arm_seq[i][3] or trRArm, arm_seq[i][4], arm_seq[i][5],} )

    elseif arm_seq[i][1] =='move0' then
      local trLArmTarget,trRArmTarget = trLArm,trRArm
      local lOffset,rOffset = mcm.get_arm_lhandoffset(),mcm.get_arm_rhandoffset()
      if arm_seq[i][2] then trLArmTarget = libTransform.trans6D(arm_seq[i][2],lOffset) end 
      if arm_seq[i][3] then trRArmTarget = libTransform.trans6D(arm_seq[i][3],rOffset) end
      LAP, RAP, uTP, WP, end_cond  = self:plan_unified('move',
        init_cond,   {trLArm,trRArm},
        {trLArmTarget,trRArmTarget, arm_seq[i][4], arm_seq[i][5]} )

    elseif arm_seq[i][1] =='wrist' then
      LAP, RAP, uTP, WP, end_cond  = self:plan_unified('wrist',
        init_cond, {trLArm,trRArm},
        {arm_seq[i][2] or trLArm,  arm_seq[i][3] or trRArm} )
    end
    if not LAP then 
      hcm.set_state_success(-1) --Report plan failure
      print("PLAN FAIL")
      return nil,current_stage_name
    end
    init_cond = end_cond    
    if end_doorparam then self.init_doorparam = end_doorparam end
    if end_valveparam then self.init_valveparam = end_valveparam end
    
    for j=1,#LAP do
      LAPs[counter],RAPs[counter],uTPs[counter],WPs[counter]= LAP[j],RAP[j],uTP[j],WP[j]
      counter = counter+1
    end
  end

  self:save_boundary_condition(init_cond)
  local arm_plan = {LAP = LAPs, RAP = RAPs,  uTP =uTPs, WP=WPs}

  filter_arm_plan(arm_plan)
  print_arm_plan(arm_plan)

  self:init_arm_sequence(arm_plan,Body.get_time())

  local t1 =unix.time()
  print(sformat("Total planning time: %.2f ms",(t1-t0)*1000))  


  return true, next_stage_name
end






local function init_arm_sequence(self,arm_plan,t0)
  if not arm_plan then return end
  self.leftArmQueue = arm_plan.LAP
  self.rightArmQueue = arm_plan.RAP
  self.torsoCompQueue = arm_plan.uTP
  
  self.t_last = t0
  self.armQueuePlayStartTime = t0
  self.armQueuePlayEndTime = t0 + self.leftArmQueue[2][2]

  self.qLArmStart = self.leftArmQueue[1][1]
  self.qLArmEnd = self.leftArmQueue[2][1]
  self.qRArmStart = self.rightArmQueue[1][1]
  self.qRArmEnd = self.rightArmQueue[2][1]
  self.uTorsoCompStart=vector.new(self.torsoCompQueue[1])
  self.uTorsoCompEnd=vector.new(self.torsoCompQueue[2])

  if arm_plan.WP then
    self.waistQueue = arm_plan.WP
    self.waistStart = self.waistQueue[1]
    self.waistEnd = self.waistQueue[2]
  else
    self.waistQueue = nil
  end
  self.armQueuePlaybackCount = 2


  self:print_segment_info() 
end

local function print_segment_info(self)
  if debug_on then
    print(string.format("%d uTC: %.3f %.3f to %.3f %.3f, t=%.2f",
    self.armQueuePlaybackCount,
    self.uTorsoCompStart[1],self.uTorsoCompStart[2],
    self.uTorsoCompEnd[1],self.uTorsoCompEnd[2],
    self.armQueuePlayEndTime - self.armQueuePlayStartTime 
     ))
  end
end

local function play_arm_sequence(self,t)
  if not self.t_last then return true end
  local dt =  t - self.t_last
  self.t_last = t
  if #self.leftArmQueue < self.armQueuePlaybackCount then
    return true
  else


   --Skip keyframes if needed
    while t>self.armQueuePlayEndTime do        
      self.armQueuePlaybackCount = self.armQueuePlaybackCount +1        
      if #self.leftArmQueue < self.armQueuePlaybackCount then
        --Passed the end of the queue. return the last joint angle
        return 
          self.leftArmQueue[#self.leftArmQueue][1],
          self.rightArmQueue[#self.leftArmQueue][1],
          self.torsoCompQueue[#self.leftArmQueue]
      end
      --Update the frame start and end time
      self.armQueuePlayStartTime = self.armQueuePlayEndTime        
      self.armQueuePlayEndTime = self.armQueuePlayStartTime + 
          self.leftArmQueue[self.armQueuePlaybackCount][2]
      --Update initial and final joint angle
      self.qLArmStart = vector.new(self.leftArmQueue[self.armQueuePlaybackCount-1][1])
      self.qLArmEnd = vector.new(self.leftArmQueue[self.armQueuePlaybackCount][1])
      self.qRArmStart = vector.new(self.rightArmQueue[self.armQueuePlaybackCount-1][1])
      self.qRArmEnd = vector.new(self.rightArmQueue[self.armQueuePlaybackCount][1])
      self.uTorsoCompStart = vector.new(self.torsoCompQueue[self.armQueuePlaybackCount-1])
      self.uTorsoCompEnd = vector.new(self.torsoCompQueue[self.armQueuePlaybackCount])

      if self.waistQueue then
        self.waistStart = self.waistQueue[self.armQueuePlaybackCount-1]
        self.waistEnd = self.waistQueue[self.armQueuePlaybackCount]
      end

      self:print_segment_info()      
    end
    local ph = (t-self.armQueuePlayStartTime)/ 
              (self.armQueuePlayEndTime-self.armQueuePlayStartTime)
--    print(sformat("%d/%d ph:%.2f",self.armQueuePlaybackCount,#self.leftArmQueue,ph))
    local qLArm,qRArm,qWaist={},{}
    for i=1,7 do
      qLArm[i] = self.qLArmStart[i] + ph * (util.mod_angle(self.qLArmEnd[i]-self.qLArmStart[i]))
      qRArm[i] = self.qRArmStart[i] + ph * (util.mod_angle(self.qRArmEnd[i]-self.qRArmStart[i]))
    end
    local uTorsoComp = (1-ph)*self.uTorsoCompStart + ph*self.uTorsoCompEnd


    --Update transform information
    local trLArmComp = Body.get_forward_larm(qLArm)
    local trRArmComp = Body.get_forward_rarm(qRArm)
    local trLArm = vector.new(trLArmComp)+
              vector.new({uTorsoComp[1],uTorsoComp[2],0, 0,0,0})
    local trRArm = vector.new(trRArmComp)+
              vector.new({uTorsoComp[1],uTorsoComp[2],0, 0,0,0})
    hcm.set_hands_left_tr(trLArm)
    hcm.set_hands_right_tr(trRArm)
    hcm.set_hands_left_tr_target(trLArm)
    hcm.set_hands_right_tr_target(trRArm)

    --Move joints
    movearm.setArmJoints(qLArm,qRArm,dt)
    mcm.set_stance_uTorsoComp(uTorsoComp)    

    if self.waistQueue then
      local qWaist={}
      qWaist[1] = self.waistStart[1] + ph * (self.waistEnd[1] - self.waistStart[1])
      qWaist[2] = self.waistStart[2] + ph * (self.waistEnd[2] - self.waistStart[2])
      Body.set_waist_command_position(qWaist)
    end
  end
  return false
end

local function save_boundary_condition(self,arm_end)
  mcm.set_arm_qlarm(arm_end[1])
  mcm.set_arm_qrarm(arm_end[2])        
  mcm.set_arm_qlarmcomp(arm_end[3])
  mcm.set_arm_qrarmcomp(arm_end[4])
end

local function load_boundary_condition(self)
  local qLArm=mcm.get_arm_qlarm()
  local qRArm=mcm.get_arm_qrarm()        
  local qLArmComp=mcm.get_arm_qlarmcomp()
  local qRArmComp=mcm.get_arm_qrarmcomp()
  local uTorsoComp = mcm.get_stance_uTorsoComp()
  local init_cond = {qLArm,qRArm,qLArmComp,qRArmComp,uTorsoComp}
  return init_cond
end

local function save_doorparam(self,doorparam)
  self.init_doorparam=doorparam
end

local function save_valveparam(self,valveparam)
  self.init_valveparam=valveparam
end

local function set_shoulder_yaw_target(self,left,right)
  self.shoulder_yaw_target_left = left
  self.shoulder_yaw_target_right = right
end





local libArmPlan={}

libArmPlan.new_planner = function (params)

  params = params or {}
  local s = {}
  --member variables
  s.armQueue = {}
  s.armQueuePlaybackCount = 1
  s.armQueuePlayStartTime = 0
  s.mLeftHand = 0
  s.mRightHand = 0
  s.shoulder_yaw_target_left = nil
  s.shoulder_yaw_target_right = nil

  s.torsoCompBias = {0,0}



  s.leftArmQueue={}
  s.rightArmQueue={}
  s.torsoCompQueue={}
  s.waistQueue={}

  s.init_cond = {}
  s.current_plan = {}
  s.current_endcond = {}

  s.init_doorparam = {}
  s.init_valveparam = {0,0,0,0}

  --member functions
  s.print_transform = print_transform
  s.print_jangle = print_jangle
  s.print_segment_info = print_segment_info

  s.calculate_margin = calculate_margin
  s.search_shoulder_angle = search_shoulder_angle    
  s.set_hand_mass = set_hand_mass
  s.reset_torso_comp = reset_torso_comp
  s.get_next_movement = get_next_movement

  s.plan_arm_sequence = plan_arm_sequence
  s.plan_arm_sequence2 = plan_arm_sequence
  
  s.plan_unified = plan_unified

  s.init_arm_sequence = init_arm_sequence
  s.play_arm_sequence = play_arm_sequence

  s.save_boundary_condition=save_boundary_condition
  s.load_boundary_condition=load_boundary_condition

  s.save_doorparam = save_doorparam  
  s.save_valveparam = save_valveparam
  s.set_shoulder_yaw_target = set_shoulder_yaw_target

  s.range_test=range_test

  s.get_armangle_jacobian=get_armangle_jacobian
  s.get_next_movement_jacobian=get_next_movement_jacobian
  return s
end

return libArmPlan
local moveleg={}
local Body   = require'Body'
local K      = Body.Kinematics
local T      = require'Transform'
local util   = require'util'
local vector = require'vector'

require'mcm'

-- SJ: Shared library for 2D leg trajectory generation
-- So that we can reuse them for different controllers

local footY    = Config.walk.footY
local supportX = Config.walk.supportX
local supportY = Config.walk.supportY
local torsoX    = Config.walk.torsoX

-- Gyro stabilization parameters
local ankleImuParamX = Config.walk.ankleImuParamX
local ankleImuParamY = Config.walk.ankleImuParamY
local kneeImuParamX  = Config.walk.kneeImuParamX
local hipImuParamY   = Config.walk.hipImuParamY

-- Hip sag compensation parameters
local hipRollCompensation = Config.walk.hipRollCompensation or 0
local ankleRollCompensation = Config.walk.ankleRollCompensation or 0
local anklePitchCompensation = Config.walk.anklePitchCompensation or 0
local kneePitchCompensation = Config.walk.kneePitchCompensation or 0
local hipPitchCompensation = Config.walk.hipPitchCompensation or 0



function moveleg.get_gyro_feedback( uLeft, uRight, uTorsoActual, supportLeg )
  local body_yaw
  if supportLeg == 0 then  -- Left support
    body_yaw = uLeft[3]  - uTorsoActual[3]
  else
    body_yaw = uRight[3] - uTorsoActual[3]
  end
  -- Ankle stabilization  gyro feedback
  --local imu_roll0, imu_pitch0, imu_yaw0 = unpack(Body.get_sensor_imu())
  --math.sin(imuPitch)*bodyHeight, -math.sin(imuRoll)*bodyHeight
  local gyro, gyro_t = Body.get_gyro()
  local gyro_roll0, gyro_pitch0, gyro_yaw0 = unpack(gyro)
  -- Get effective gyro angle considering body yaw offset
  -- Rotate the Roll and pitch about the intended body yaw
  local gyro_roll  = gyro_roll0  * math.cos(body_yaw) - gyro_pitch0 * math.sin(body_yaw)
  local gyro_pitch = gyro_pitch0 * math.cos(body_yaw) - gyro_roll0  * math.sin(body_yaw)


  -- Give these parameters
  return {gyro_roll, gyro_pitch, gyro_yaw0}
end



function moveleg.get_leg_compensation(supportLeg, ph, gyro_rpy,angleShift)
  local gyro_pitch = gyro_rpy[2]
  local gyro_roll = gyro_rpy[1]

  -- Ankle feedback
  local ankleShiftX = util.procFunc(gyro_pitch*ankleImuParamX[2],ankleImuParamX[3],ankleImuParamX[4])
  local ankleShiftY = util.procFunc(gyro_roll*ankleImuParamY[2],ankleImuParamY[3],ankleImuParamY[4])

  -- Ankle shift is filtered... thus a global
  angleShift[1] = angleShift[1] + ankleImuParamX[1]*(ankleShiftX-angleShift[1])
  angleShift[2] = angleShift[2] + ankleImuParamY[1]*(ankleShiftY-angleShift[2])

  -- Knee feedback
  local kneeShiftX = util.procFunc(gyro_pitch*kneeImuParamX[2],kneeImuParamX[3],kneeImuParamX[4])
  angleShift[3] = angleShift[3] + kneeImuParamX[1]*(kneeShiftX-angleShift[3])

  -- Hip feedback
  local hipShiftY=util.procFunc(gyro_roll*hipImuParamY[2],hipImuParamY[3],hipImuParamY[4])
  angleShift[4] = angleShift[4]+hipImuParamY[1]*(hipShiftY-angleShift[4])

  local delta_legs = vector.zeros(12)
  -- Change compensation in the beginning of the phase (first 10%)
  -- Saturate compensation afterwards
  -- Change compensation at the beginning of the phase (first 10%)
  -- Same sort of trapezoid at double->single->double support shape

  --SJ: now we apply the compensation during DS too
  local phComp1 = 0.1
  local phComp2 = 0.9
  local phSingleComp = math.min( math.max(ph-phComp1, 0)/(phComp2-phComp1), 1)

  local phComp = 10 * math.min( phSingleComp, .1, 1-phSingleComp)
--[[
  if supportLeg == 0 then
    -- Left support
    delta_legs[2] = angleShift[4] + hipRollCompensation*phComp
    delta_legs[4] = angleShift[3]
    delta_legs[5] = angleShift[1]
    delta_legs[6] = angleShift[2]*phComp
  elseif supportLeg==1 then
    -- Right support
    delta_legs[8]  = angleShift[4] - hipRollCompensation*phComp
    delta_legs[10] = angleShift[3]
    delta_legs[11] = angleShift[1]
    delta_legs[12] = angleShift[2]*phComp
  elseif supportLeg==2 then
    -- Double support
    delta_legs[4] = angleShift[3]
    delta_legs[5] = angleShift[1]

    delta_legs[10] = angleShift[3]
    delta_legs[11] = angleShift[1]
  else --Robotis style
    delta_legs[2] = angleShift[4]
    delta_legs[4] = angleShift[3]
    delta_legs[5] = angleShift[1]
    delta_legs[6] = angleShift[2]

    delta_legs[8]  = angleShift[4]
    delta_legs[10] = angleShift[3]
    delta_legs[11] = angleShift[1]
    delta_legs[12] = angleShift[2]
  end
--]]

local comp_factor = 1
if mcm.get_stance_singlesupport()==1 then
  comp_factor = 2
end

if supportLeg == 0 then
    -- Left support
  delta_legs[2] = angleShift[4] + hipRollCompensation*phComp*comp_factor
elseif supportLeg==1 then
    -- Right support
  delta_legs[8]  = angleShift[4] - hipRollCompensation*phComp*comp_factor
else
  delta_legs[2] = angleShift[4]
  delta_legs[8]  = angleShift[4]
end

delta_legs[4] = angleShift[3]
delta_legs[5] = angleShift[1]
delta_legs[6] = angleShift[2]

delta_legs[10] = angleShift[3]
delta_legs[11] = angleShift[1]
delta_legs[12] = angleShift[2]


--  print('Ankle shift',angleShift[1]*Body.RAD_TO_DEG )

  return delta_legs, angleShift
end



--Robotis style simple feedback
function moveleg.get_leg_compensation_simple(supportLeg, phSingle, gyro_rpy,angleShift)
  local gyro_pitch = gyro_rpy[2]
  local gyro_roll = gyro_rpy[1]

  -- Ankle feedback
  local ankleShiftX = util.procFunc(gyro_pitch*ankleImuParamX[2],ankleImuParamX[3],ankleImuParamX[4])
  local ankleShiftY = util.procFunc(gyro_roll*ankleImuParamY[2],ankleImuParamY[3],ankleImuParamY[4])

  -- Ankle shift is filtered... thus a global
  angleShift[1] = angleShift[1] + ankleImuParamX[1]*(ankleShiftX-angleShift[1])
  angleShift[2] = angleShift[2] + ankleImuParamY[1]*(ankleShiftY-angleShift[2])

  -- Knee feedback
  local kneeShiftX = util.procFunc(gyro_pitch*kneeImuParamX[2],kneeImuParamX[3],kneeImuParamX[4])
  angleShift[3] = angleShift[3] + kneeImuParamX[1]*(kneeShiftX-angleShift[3])

  -- Hip feedback
  local hipShiftY=util.procFunc(gyro_roll*hipImuParamY[2],hipImuParamY[3],hipImuParamY[4])
  angleShift[4] = angleShift[4]+hipImuParamY[1]*(hipShiftY-angleShift[4])

  local delta_legs = vector.zeros(12)
  -- Change compensation in the beginning of the phase (first 10%)
  -- Saturate compensation afterwards
  -- Change compensation at the beginning of the phase (first 10%)
  -- Same sort of trapezoid at double->single->double support shape
  local phComp = 10 * math.min( phSingle, .1, 1-phSingle )

  delta_legs[4] = angleShift[3]
  delta_legs[5] = angleShift[1]

  delta_legs[10] = angleShift[3]
  delta_legs[11] = angleShift[1]

 delta_legs[2] = angleShift[4]
 delta_legs[6] = angleShift[2]
  delta_legs[8]  = angleShift[4]
  delta_legs[12] = angleShift[2]
  --[[
  if supportLeg == 0 then -- Left support
    delta_legs[2] = angleShift[4]
    delta_legs[2] = delta_legs[2] + hipRollCompensation*phComp
    delta_legs[6] = angleShift[2]
  elseif supportLeg==1 then    -- Right support
    delta_legs[8]  = angleShift[4]
    delta_legs[8]  = delta_legs[8] - hipRollCompensation*phComp
    delta_legs[12] = angleShift[2]
  elseif supportLeg==2 then

  end
  --]]

--  print('Ankle shift',angleShift[1]*Body.RAD_TO_DEG )

  return delta_legs, angleShift
end


function moveleg.set_leg_positions(uTorso,uLeft,uRight,zLeft,zRight,delta_legs,aLeft,aRight)
  local uTorsoActual = util.pose_global(vector.new({-torsoX,0,0}),uTorso)
  local pTorso = vector.new({
        uTorsoActual[1], uTorsoActual[2], mcm.get_stance_bodyHeight(),
        0,mcm.get_stance_bodyTilt(),uTorsoActual[3]})
  local pLLeg = vector.new({uLeft[1],uLeft[2],zLeft,0,0,uLeft[3]})
  local pRLeg = vector.new({uRight[1],uRight[2],zRight,0,0,uRight[3]})
  

  if aLeft then
    pLLeg = vector.new({uLeft[1],uLeft[2],zLeft,0,aLeft,uLeft[3]})
    pRLeg = vector.new({uRight[1],uRight[2],zRight,0,aRight,uRight[3]})
  end

  local qLegs = K.inverse_legs(pLLeg, pRLeg, pTorso)
  local legBias = vector.new(mcm.get_leg_bias())

  qLegs = qLegs + delta_legs + legBias

  Body.set_lleg_command_position(vector.slice(qLegs,1,6))
  Body.set_rleg_command_position(vector.slice(qLegs,7,12))

  ------------------------------------------
  -- Update the status in shared memory
  local uFoot = util.se2_interpolate(.5, uLeft, uRight)
  mcm.set_status_odometry( uFoot )
  --util.pose_relative(uFoot, u0) for relative odometry to point u0
  local bodyOffset = util.pose_relative(uTorso, uFoot)
  mcm.set_status_bodyOffset( bodyOffset )
  ------------------------------------------

end







function moveleg.set_leg_positions_torsoflex(uTorso,uLeft,uRight,zLeft,zRight,delta_legs,virtual_torso_angle)
  
  local bodyHeight = mcm.get_stance_bodyHeight()

  local bodyHeightTilt = bodyHeight * math.cos(virtual_torso_angle[1])*math.cos(virtual_torso_angle[2])


--  local uTorsoActual = util.pose_global(vector.new({-torsoX,0,0}),uTorso)

  local uTorsoM=util.pose_global(
    vector.new({
      bodyHeight*math.sin(virtual_torso_angle[1]),
      -bodyHeight*math.sin(virtual_torso_angle[2]),
      0}),    uTorso)

  local uTorsoActual = util.pose_global(vector.new({-torsoX,0,0}), uTorsoM)

  local pTorso = vector.new({
        uTorsoActual[1], uTorsoActual[2], mcm.get_stance_bodyHeight(),
        0,mcm.get_stance_bodyTilt(),uTorsoActual[3]})

local pTorso = vector.new({
        uTorsoActual[1], uTorsoActual[2], bodyHeightTilt,
        virtual_torso_angle[2],virtual_torso_angle[1]+mcm.get_stance_bodyTilt(),uTorsoActual[3]})



  local pLLeg = vector.new({uLeft[1],uLeft[2],zLeft,0,0,uLeft[3]})
  local pRLeg = vector.new({uRight[1],uRight[2],zRight,0,0,uRight[3]})
  

  if aLeft then
    pLLeg = vector.new({uLeft[1],uLeft[2],zLeft,0,aLeft,uLeft[3]})
    pRLeg = vector.new({uRight[1],uRight[2],zRight,0,aRight,uRight[3]})
  end

  local qLegs = K.inverse_legs(pLLeg, pRLeg, pTorso)
  local legBias = vector.new(mcm.get_leg_bias())

  qLegs = qLegs + delta_legs + legBias

  Body.set_lleg_command_position(vector.slice(qLegs,1,6))
  Body.set_rleg_command_position(vector.slice(qLegs,7,12))

  ------------------------------------------
  -- Update the status in shared memory
  local uFoot = util.se2_interpolate(.5, uLeft, uRight)
  mcm.set_status_odometry( uFoot )
  --util.pose_relative(uFoot, u0) for relative odometry to point u0
  local bodyOffset = util.pose_relative(uTorso, uFoot)
  mcm.set_status_bodyOffset( bodyOffset )
  ------------------------------------------
end











function moveleg.set_leg_positions_ankletilt(uTorso,uLeft,uRight,zLeft,zRight,delta_legs)

  local uTorsoActual = util.pose_global(vector.new({-torsoX,0,0}),uTorso)
  local pTorso = vector.new({
        uTorsoActual[1], uTorsoActual[2], mcm.get_stance_bodyHeight(),
        0,mcm.get_stance_bodyTilt(),uTorsoActual[3]})
  local pLLeg = vector.new({uLeft[1],uLeft[2],zLeft,0,0,uLeft[3]})
  local pRLeg = vector.new({uRight[1],uRight[2],zRight,0,0,uRight[3]})

  --LeftHeel LeftToe RightHeel RightToe
  local footLift = K.calculate_foot_tilt(pLLeg, pRLeg, pTorso)
  local aLeg={0,0}
  local aLegOld = mcm.get_status_aLeg()

  --check which leg is forward
  local uLeftTorso = util.pose_relative(uLeft,uTorso)
  local uRightTorso = util.pose_relative(uRight,uTorso)

  if uLeft[1]>uRight[1] then --left foot forward
    aLeg = {footLift[2],footLift[3]} --Lfoot toe lift, right foot heel lift
  elseif uLeft[1]<uRight[1] then  --right foot forward
    aLeg = {footLift[1],footLift[4]} --Lfoot heel lift, right foot toe lift
  else
    aLeg = {footLift[2],footLift[4]} --Lfoot toe lift, right foot toe lift
  end

  --When foot is lifted, slowly zero ankle angle

  --TODO


  local qLegs = K.inverse_legs_foot_tilt(pLLeg, pRLeg, pTorso,aLeg)
  local legBias = vector.new(mcm.get_leg_bias())

  qLegs = qLegs + delta_legs + legBias
  Body.set_lleg_command_position(vector.slice(qLegs,1,6))
  Body.set_rleg_command_position(vector.slice(qLegs,7,12))

  ------------------------------------------
  -- Update the status in shared memory
  local uFoot = util.se2_interpolate(.5, uLeft, uRight)
  mcm.set_status_odometry( uFoot )
  --util.pose_relative(uFoot, u0) for relative odometry to point u0
  local bodyOffset = util.pose_relative(uTorso, uFoot)
  mcm.set_status_bodyOffset( bodyOffset )
  ------------------------------------------

  mcm.set_status_aLeg(aLeg)
  mcm.set_status_zLeg({zLeft,zRight})
end










function moveleg.set_leg_positions_kneel(dt)
  local uTorso = mcm.get_status_uTorso()
  local uLeft = mcm.get_status_uLeft()
  local uRight = mcm.get_status_uRight()

  local uTorsoActual = util.pose_global(vector.new({-torsoX,0,0}),uTorso)

  local bodyHeightVel = 0.01

  local bodyHeight0 = mcm.get_stance_bodyHeight()
  local bodyHeight1 = mcm.get_stance_bodyHeight() + dt*bodyHeightVel
  local bodyHeight2 = mcm.get_stance_bodyHeight() - dt*bodyHeightVel

  local pTorso0 = vector.new({
        uTorsoActual[1], uTorsoActual[2], bodyHeight0,
        0,mcm.get_stance_bodyTilt(),uTorsoActual[3]})

  local pTorso1 = vector.new({
        uTorsoActual[1], uTorsoActual[2], bodyHeight1,
        0,mcm.get_stance_bodyTilt(),uTorsoActual[3]})

  local pTorso2 = vector.new({
        uTorsoActual[1], uTorsoActual[2], bodyHeight2,
        0,mcm.get_stance_bodyTilt(),uTorsoActual[3]})


  local pLLeg = vector.new({uLeft[1],uLeft[2],0,0,0,uLeft[3]})
  local pRLeg = vector.new({uRight[1],uRight[2],0,0,0,uRight[3]})

  local qLegs0 = K.inverse_legs(pLLeg, pRLeg, pTorso0)
  local qLegs1 = K.inverse_legs(pLLeg, pRLeg, pTorso1)
  local qLegs2 = K.inverse_legs(pLLeg, pRLeg, pTorso2)

  local kneePadding = 0.08

  local kneeHeight0 = K.calculate_knee_height(vector.slice(qLegs0,1,6)) - kneePadding
  local kneeHeight1 = K.calculate_knee_height(vector.slice(qLegs1,1,6)) - kneePadding
  local kneeHeight2 = K.calculate_knee_height(vector.slice(qLegs2,1,6)) - kneePadding

  if kneeHeight0>0 then
    if kneeHeight2>0 and kneeHeight2<kneeHeight0 then
      mcm.set_stance_bodyHeight(bodyHeight2)
      Body.set_lleg_command_position(qLegs2)
    else
      Body.set_lleg_command_position(qLegs0)
    end
  else
    if kneeHeight1<0 and kneeHeight1>kneeHeight0 then
      mcm.set_stance_bodyHeight(bodyHeight1)
      Body.set_lleg_command_position(qLegs1)
    else
      Body.set_lleg_command_position(qLegs0)
    end
  end

  ------------------------------------------
  -- Update the status in shared memory
  local uFoot = util.se2_interpolate(.5, uLeft, uRight)
  mcm.set_status_odometry( uFoot )
  --util.pose_relative(uFoot, u0) for relative odometry to point u0
  local bodyOffset = util.pose_relative(uTorso, uFoot)
  mcm.set_status_bodyOffset( bodyOffset )
  ------------------------------------------
end


function moveleg.set_leg_transforms(pLLeg,pRLeg,pTorso,supportLeg,delta_legs)
  local qLegs = K.inverse_legs(pLLeg, pRLeg, pTorso, supportLeg)
  qLegs = qLegs + delta_legs
  Body.set_lleg_command_position(qLegs)
end

function moveleg.get_ph_single(ph,phase1,phase2)
  return math.min(1, math.max(0, (ph-phase1)/(phase2-phase1) ))
end


local function eval_spline(breaks,coefs,ph)
  local x_offset, xf = 0,0
  for i=1,#breaks do
    if ph<=breaks[i] then
      local x=ph - x_offset
      xf = coefs[i][1]*x^3 + coefs[i][2]*x^2 + coefs[i][3]*x + coefs[i][4]
      break;
    end
    x_offset = breaks[i]    
  end
  return xf
end




function moveleg.foot_trajectory_walkkick(phSingle,uStart,uEnd,stepHeight)



local breaksTX={0.300000,0.600000,0.700000,0.800000,0.900000,1.000000,}
local breaksTY={0.300000,0.500000,0.700000,0.800000,0.900000,1.000000,}
local coefsX={
  {-5.566198,3.898467,1.664751,0.000000,},
  {-5.566198,-1.111111,2.500958,0.700000,},
  {28.065134,-6.120690,0.331418,1.200000,},
  {-17.911877,2.298851,-0.050766,1.200000,},
  {-6.417625,-3.074713,-0.128352,1.200000,},
  {-6.417625,-5.000000,-0.935824,1.150000,},
}
local coefsY={
  {5.646481,-9.517185,5.346972,0.000000,},
  {5.646481,-4.435352,1.161211,0.900000,},
  {-8.878887,-1.047463,0.064648,1.000000,},
  {5.728314,-6.374795,-1.419804,0.900000,},
  {-1.145663,-4.656301,-2.522913,0.700000,},
  {-1.145663,-5.000000,-3.488543,0.400000,},
}


local breaksTX={0.300000,0.400000,0.600000,0.800000,0.900000,1.000000,}
local breaksTY={0.300000,0.500000,0.700000,0.800000,0.900000,1.000000,}
local coefsX={
  {8.359213,0.815217,-0.163561,0.000000,},
  {8.359213,8.338509,2.582557,0.250000,},
  {-54.257246,10.846273,4.501035,0.600000,},
  {31.573499,-21.708075,2.328675,1.500000,},
  {34.213251,-2.763975,-2.565735,1.350000,},
  {34.213251,7.500000,-2.092133,1.100000,},
}
local coefsY={
  {5.646481,-9.517185,5.346972,0.000000,},
  {5.646481,-4.435352,1.161211,0.900000,},
  {-8.878887,-1.047463,0.064648,1.000000,},
  {5.728314,-6.374795,-1.419804,0.900000,},
  {-1.145663,-4.656301,-2.522913,0.700000,},
  {-1.145663,-5.000000,-3.488543,0.400000,},
}







  local xf=eval_spline(breaksTX, coefsX,phSingle)  
  local zf=eval_spline(breaksTY, coefsY,phSingle)  
  local uFoot = util.se2_interpolate(xf, uStart,uEnd)
  local zFoot = stepHeight * zf*1.5
  return uFoot, zFoot
end

--csapi([0 0.1 0.3 0.7 0.8 0.9 1],[0 -0.2 -1 2  2 1.4 1])
function moveleg.foot_trajectory_kick(phSingle,uStart,uEnd,stepHeight)
  local breaksTX={0.100000,0.300000,0.600000,0.700000,0.800000,0.900000,1.000000,}
  local breaksTY={0.100000,0.300000,0.500000,0.700000,0.800000,0.900000,1.000000,}
  local coefsX={
    {141.283466,-63.180053,2.905171,0.000000,},
    {141.283466,-20.795013,-5.492336,-0.200000,},
    {-137.068828,63.975066,3.143675,-1.000000,},
    {241.855712,-59.386879,4.520131,2.000000,},
    {-221.540977,13.169834,-0.101574,2.100000,},
    {244.308195,-53.292459,-4.113836,2.000000,},
    {244.308195,20.000000,-7.443082,1.300000,},
  }
  local coefsY={
    {36.771326,-33.041864,9.936473,0.000000,},
    {36.771326,-22.010466,4.431240,0.700000,},
    {-1.251969,0.052330,0.039613,1.000000,},
    {-6.763448,-0.698852,-0.089692,1.000000,},
    {-34.346163,-4.756921,-1.180846,0.900000,},
    {66.869233,-15.060770,-3.162615,0.700000,},
    {66.869233,5.000000,-4.168692,0.300000,},
  }


  --More swing back

local breaksTX={0.100000,0.400000,0.670000,0.720000,0.800000,0.900000,1.000000,}
local breaksTY={0.100000,0.300000,0.500000,0.700000,0.800000,0.900000,1.000000,}
local coefsX={
  {103.699049,-45.182858,-3.518705,0.000000,},
  {103.699049,-14.073143,-9.444305,-0.700000,},
  {-229.010397,79.256001,10.110553,-2.000000,},
  {995.160754,-106.242421,2.824219,2.000000,},
  {-485.346581,43.031692,-0.336317,2.000000,},
  {311.504957,-73.451487,-2.769901,2.000000,},
  {311.504957,20.000000,-8.115050,1.300000,},
}
local coefsY={
  {-32.433041,1.306550,7.193675,0.000000,},
  {-32.433041,-8.423363,6.481994,0.700000,},
  {108.898830,-27.883187,-0.779316,1.400000,},
  {-165.662278,37.456111,1.135269,1.000000,},
  {295.588566,-61.941256,-3.761760,1.400000,},
  {-39.117713,26.735314,-7.282354,0.700000,},
  {-39.117713,15.000000,-3.108823,0.200000,},
}



  local xf=eval_spline(breaksTX, coefsX,phSingle)  
  local zf=eval_spline(breaksTY, coefsY,phSingle)  
  local uFoot = util.se2_interpolate(xf, uStart,uEnd)
  local zFoot = stepHeight * zf*2.5
  return uFoot, zFoot
end



function moveleg.foot_trajectory_base(phSingle,uStart,uEnd,stepHeight)
  local phSingleSkew = phSingle^0.8 - 0.17*phSingle*(1-phSingle)
  local xf = .5*(1-math.cos(math.pi*phSingleSkew))
  local zf = .5*(1-math.cos(2*math.pi*phSingleSkew))
  local uFoot = util.se2_interpolate(xf, uStart,uEnd)
  local zFoot = stepHeight * zf

  return uFoot, zFoot
end

function moveleg.foot_trajectory_square(phSingle,uStart,uEnd,stepHeight)
  local phase1,phase2 = 0.2, 0.7 --TODO: automatic detect
  local xf,zf = 0,0

  if phSingle<phase1 then --Lifting phase
    ph1 = phSingle / phase1
    zf = ph1;
  elseif phSingle<phase2 then
    ph1 = (phSingle-phase1) / (phase2-phase1)
    xf,zf = ph1, 1
  else
    ph1 = (phSingle-phase2) / (1-phase2)
    xf,zf = 1, 1-ph1
  end
  local uFoot = util.se2_interpolate(xf, uStart,uEnd)
  local zFoot = stepHeight * zf
  return uFoot, zFoot
end


function moveleg.foot_trajectory_square_stair(phSingle,uStart,uEnd, stepHeight, walkParam)
  local phase1,phase2 = 0.2, 0.7 --TODO: automatic detect
  local xf,zf = 0,0
  local zFoot,aFoot = 0,0
  local zHeight0, zHeight1= 0,0,0
  local special = false


  if walkParam then    
    zHeight0, zHeight1 = walkParam[1],walkParam[3]
    stepHeight = walkParam[2]
    --hack for the special step for block climbing
    if walkParam[1]==-999 then
      zHeight0 = 0
      special = true
    end


    local move1 = math.abs(zHeight0-stepHeight)
    local move2 = math.abs(zHeight1-stepHeight)
  
    if move1>move2*2.0 then --step up
      phase1,phase2 = 0.5,0.8
    elseif move1*2.0<move2 then --step down
      phase1,phase2 = 0.2,0.5
    end
  end

  if phSingle<phase1 then --Lifting phase
    ph1 = phSingle / phase1
    zf = ph1;
    zFoot = zHeight0 + (stepHeight-zHeight0) * zf
    if special then
      if ph1<0.4 then ph2=ph1/0.4
      elseif ph1<0.7 then ph2 = 1
      else ph2 = (1-ph1)/0.3 end
      aFoot = 20*math.pi/180*ph2      
    end
  elseif phSingle<phase2 then
    ph1 = (phSingle-phase1) / (phase2-phase1)
    xf,zf = ph1, 1
    zFoot = stepHeight * zf

  else
    ph1 = (phSingle-phase2) / (1-phase2)
    xf,zf = 1, 1-ph1
    zFoot = zHeight1 + (stepHeight-zHeight1) * zf
  end

  local uFoot = util.se2_interpolate(xf, uStart,uEnd)
  return uFoot, zFoot, aFoot
end

function moveleg.foot_trajectory_square_stair_touchdown(phSingle,uStart,uEnd, stepHeight, walkParam, zOld,forceZ, touched)
  local phase1,phase2 = 0.2, 0.7 --TODO: automatic detect
  local xf,zf = 0,0
  local zFoot,aFoot = 0,0
  local zHeight0, zHeight1= 0,0,0


  if walkParam then    
    zHeight0, zHeight1 = walkParam[1],walkParam[3]
    stepHeight = walkParam[2]
    local move1 = math.abs(zHeight0-stepHeight)
    local move2 = math.abs(zHeight1-stepHeight)
  
    if move1>move2*2.0 then --step up
      phase1,phase2 = 0.5,0.8
    elseif move1*2.0<move2 then --step down
      phase1,phase2 = 0.2,0.5
    end
  end

  if phSingle<phase1 then --Lifting phase
    ph1 = phSingle / phase1
    zf = ph1;
    zFoot = zHeight0 + (stepHeight-zHeight0) * zf
  elseif phSingle<phase2 then
    ph1 = (phSingle-phase1) / (phase2-phase1)
    xf,zf = ph1, 1
    zFoot = stepHeight * zf
  else
    if forceZ>50 or touched or phSingle==1 then
      print("TOUCHDOWN!",zOld,forceZ)
      xf=1
      zFoot = zOld
      local uFoot = util.se2_interpolate(xf, uStart,uEnd)
      return uFoot, zFoot, aFoot, true
    end

    ph1 = (phSingle-phase2) / (1-phase2)
    xf,zf = 1, 1-ph1
    zFoot = zHeight1 + (stepHeight-zHeight1) * zf
  end

  local uFoot = util.se2_interpolate(xf, uStart,uEnd)
  return uFoot, zFoot, aFoot, false
end



function moveleg.foot_trajectory_square_touchdown(phSingle,uStart,uEnd,stepHeight, touched)
  local phase1,phase2 = 0.2, 0.7 --TODO: automatic detect
  local xf,zf = 0,0

  if phSingle<phase1 then --Lifting phase
    ph1 = phSingle / phase1
    zf = ph1;
  elseif phSingle<phase2 then
    ph1 = (phSingle-phase1) / (phase2-phase1)
    xf,zf = ph1, 1
  else
    ph1 = (phSingle-phase2) / (1-phase2)
    xf,zf = 1, 1-ph1
  end
  local uFoot = util.se2_interpolate(xf, uStart,uEnd)
  local zFoot = stepHeight * zf
  return uFoot, zFoot
end



function moveleg.get_foot(ph,start_phase,finish_phase)
  -- Computes relative x, z motion of foot during single support phase
  -- phSingle = 0: x=0, z=0, phSingle = 1: x=1,z=0
  -- phSingle is 100% @ finish_phase, and 0% at start_phase
  -- It just ignores the double support phase so we know how long we've been in single support
  local phSingle = math.min( math.max(ph-start_phase, 0)/(finish_phase-start_phase), 1)
  local phSingleSkew = phSingle^0.8 - 0.17*phSingle*(1-phSingle)
  local xf = .5*(1-math.cos(math.pi*phSingleSkew))
  local zf = .5*(1-math.cos(2*math.pi*phSingleSkew))
  -- xf and zf and percentages, it seems
  return xf, zf, phSingle
end

function moveleg.get_foot_square(ph,start_phase,finish_phase)
  --SJ: Square wave walk pattern
  local phSingle = math.min( math.max(ph-start_phase, 0)/(finish_phase-start_phase), 1)
  local phase1 = 0.2;
  local phase2 = 0.7;

  if phSingle<phase1 then --Lifting phase
    ph1 = phSingle / phase1
    xf = 0;
    zf = ph1;
  elseif phSingle<phase2 then
    ph1 = (phSingle-phase1) / (phase2-phase1)
    xf = ph1;
    zf = 1;
  else
    ph1 = (phSingle-phase2) / (1-phase2)
    xf = 1;
    zf = (1-ph1)
  end
  return xf,zf,phSingle
end



function moveleg.get_leg_compensation_new(supportLeg, ph, gyro_rpy,angleShift,supportRatio,dt)

--New compensation code to cancelout backlash on ALL leg joints
  dt= dt or 0.010

  --Now we limit the angular velocity of compensation angles 
  local DEG_TO_RAD = math.pi/180
  local dShift = {30*DEG_TO_RAD,30*DEG_TO_RAD,30*DEG_TO_RAD,30*DEG_TO_RAD}

  local gyro_pitch = gyro_rpy[2]
  local gyro_roll = gyro_rpy[1]

  -- Ankle feedback
  local ankleShiftX = util.procFunc(gyro_pitch*ankleImuParamX[2],ankleImuParamX[3],ankleImuParamX[4])
  local ankleShiftY = util.procFunc(gyro_roll*ankleImuParamY[2],ankleImuParamY[3],ankleImuParamY[4])
  -- Knee feedback
  local kneeShiftX = util.procFunc(gyro_pitch*kneeImuParamX[2],kneeImuParamX[3],kneeImuParamX[4])
  -- Hip feedback
  local hipShiftY=util.procFunc(gyro_roll*hipImuParamY[2],hipImuParamY[3],hipImuParamY[4])

  local dShiftTarget = {}
  dShiftTarget[1]=ankleImuParamX[1]*(ankleShiftX-angleShift[1])
  dShiftTarget[2]=ankleImuParamY[1]*(ankleShiftY-angleShift[2])
  dShiftTarget[3]=kneeImuParamX[1]*(kneeShiftX-angleShift[3])
  dShiftTarget[4]=hipImuParamY[1]*(hipShiftY-angleShift[4])
  
-- Ankle shift is filtered... thus a global
  angleShift[1] = angleShift[1] + math.max(-dShift[1]*dt,math.min(dShift[1]*dt,dShiftTarget[1]))
  angleShift[2] = angleShift[2] + math.max(-dShift[2]*dt,math.min(dShift[2]*dt,dShiftTarget[2])) 
  angleShift[3] = angleShift[3] + math.max(-dShift[3]*dt,math.min(dShift[3]*dt,dShiftTarget[3])) 
  angleShift[4] = angleShift[4] + math.max(-dShift[4]*dt,math.min(dShift[4]*dt,dShiftTarget[4])) 


  local delta_legs = vector.zeros(12)

  --How much do we need to apply the compensation?
  local supportRatioRight = supportRatio;
  local supportRatioLeft = 1-supportRatio;
--  supportRatioLeft = math.max(0,supportRatioLeft*4-3);
--  supportRatioRight = math.max(0,supportRatioRight*4-3);

  supportRatioLeft = math.max(0,supportRatioLeft*2-1);
  supportRatioRight = math.max(0,supportRatioRight*2-1);



--print("SR:",supportRatio,supportRatioLeft,supportRatioRight)
  --SJ: now we apply the compensation during DS too
  local phComp1 = Config.walk.phComp[1]
  local phComp2 = Config.walk.phComp[2]
  local phCompSlope = Config.walk.phCompSlope

  local phSingleComp = math.min( math.max(ph-phComp1, 0)/(phComp2-phComp1), 1)
  local phComp = math.min( phSingleComp/phCompSlope, 1,
                          (1-phSingleComp)/phCompSlope)
  supportRatioLeft, supportRatioRight = 0,0


  if mcm.get_stance_singlesupport()==1 then
    phComp = phComp*2
  end



  if supportLeg == 0 then -- Left support
    supportRatioLeft = phComp;
  elseif supportLeg==1 then
    supportRatioRight = phComp;
  end
    delta_legs[2] = angleShift[4] + hipRollCompensation*supportRatioLeft
    delta_legs[3] = - hipPitchCompensation*supportRatioLeft
    delta_legs[4] = angleShift[3] - kneePitchCompensation*supportRatioLeft
    delta_legs[5] = angleShift[1] - anklePitchCompensation*supportRatioLeft
    delta_legs[6] = angleShift[2] + ankleRollCompensation*supportRatioLeft

    delta_legs[8]  = angleShift[4] - hipRollCompensation*supportRatioRight
    delta_legs[9] = -hipPitchCompensation*supportRatioRight
    delta_legs[10] = angleShift[3] - kneePitchCompensation*supportRatioRight
    delta_legs[11] = angleShift[1] - anklePitchCompensation*supportRatioRight
    delta_legs[12] = angleShift[2] - ankleRollCompensation

  return delta_legs, angleShift
end


function moveleg.process_ft(lf_z,rf_z,lt_x,rt_x,lt_y,rt_y,  support)






end
















return moveleg

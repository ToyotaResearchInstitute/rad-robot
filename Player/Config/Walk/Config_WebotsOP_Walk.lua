module(..., package.seeall); require('vector')
require 'unix'
-- Walk Parameters

walk = {};

----------------------------------------------
-- Stance and velocity limit values
----------------------------------------------
walk.stanceLimitX={-0.16,0.16};
walk.stanceLimitY={0.07,0.20};
walk.stanceLimitA={-10*math.pi/180,30*math.pi/180};
walk.velLimitX={-.04,.08};
walk.velLimitY={-.03,.03};
walk.velLimitA={-.3,.3};
walk.velDelta={0.02,0.02,0.15} 

walk.footSizeX = {-0.05, 0.05};
walk.stanceLimitMarginY = 0.015;

----------------------------------------------
-- Stance parameters
---------------------------------------------
walk.bodyHeight = 0.295; 
walk.bodyTilt=20*math.pi/180; 
walk.footX= -0.0; 
walk.footY = 0.0375;
walk.supportX = 0;
walk.supportY = 0.025;
walk.qLArm=math.pi/180*vector.new({90,8,-40});
walk.qRArm=math.pi/180*vector.new({90,-8,-40});
walk.qLArmKick=math.pi/180*vector.new({90,15,-40});
walk.qRArmKick=math.pi/180*vector.new({90,-15,-40});

walk.hardnessSupport = 1;
walk.hardnessSwing = 1;
walk.hardnessArm=.3;
---------------------------------------------
-- Gait parameters
---------------------------------------------
walk.tStep = 0.50;
walk.tZmp = 0.165;
walk.stepHeight = 0.025;
walk.phSingle={0.2,0.8};

--------------------------------------------
-- Compensation parameters
--------------------------------------------
walk.hipRollCompensation = 3*math.pi/180;
walk.ankleMod = vector.new({-1,0})/0.12 * 10*math.pi/180;

--------------------------------------------------------------
--Imu feedback parameters, alpha / gain / deadband / max
--------------------------------------------------------------
gyroFactor = 0.273*math.pi/180 * 300 / 1024; --dps to rad/s conversion

--Disabled for webots
--gyroFactor = 0;

walk.ankleImuParamX={1,0.75*gyroFactor, 2*math.pi/180, 10*math.pi/180};
walk.kneeImuParamX={1,1.5*gyroFactor, 2*math.pi/180, 10*math.pi/180};
walk.ankleImuParamY={1,1*gyroFactor, 2*math.pi/180, 10*math.pi/180};
walk.hipImuParamY={1,1*gyroFactor, 2*math.pi/180, 10*math.pi/180};
walk.armImuParamX={0.3,10*gyroFactor, 20*math.pi/180, 45*math.pi/180};
walk.armImuParamY={0.3,10*gyroFactor, 20*math.pi/180, 45*math.pi/180};

--------------------------------------------
-- WalkKick parameters
--------------------------------------------
walk.walkKickVel = {0.06, 0.12} --step / kick / follow 
walk.walkKickSupportMod = {{0,0},{0,0}}
walk.walkKickHeightFactor = 1.5;
	
walk.walkKickSupportMod = {{0,0},{-0.02,0}}
walk.walkKickHeightFactor = 2.5;
walk.tStepWalkKick = 0.50;

--kick to left default
walk.sideKickVel1 = {0.04,0.04,0};
walk.sideKickVel2 = {0.09,-0.05,0};
walk.sideKickVel3 = {0.09,0.02,0};
walk.sideKickSupportMod = {{0,0},{0,0}};
walk.tStepSideKick = 0.50;


--[[


--angled sidekick #1
walk.sideKickVel1 = {0.04,0.04,20*math.pi/180};
walk.sideKickVel2 = {0.05,-0.06,-40*math.pi/180};
walk.sideKickVel3 = {0.09,0.03,20*math.pi/180};

--45 deg angled sidekick #2
walk.sideKickVel1 = {0.04,0.02,20*math.pi/180};
walk.sideKickVel2 = {-0.04,-0.03,20*math.pi/180};
walk.sideKickVel3 = {0.06,0.02,20*math.pi/180};

--]]

--------------------------------------------
-- Robot - specific calibration parameters
--------------------------------------------

walk.kickXComp = 0;
walk.supportCompL = {0,0,0};
walk.supportCompR = {0,0,0};

walk.footHeight = 0.0355;
walk.legLength = 0.093+0.093;
walk.hipOffsetX = 0.008;
walk.hipOffsetY = 0.037;
walk.hipOffsetZ = 0.096;

--[[
--Faster turning test
walk.stanceLimitA={-20*math.pi/180,45*math.pi/180};
walk.velLimitA={-.6,.6};
--]]

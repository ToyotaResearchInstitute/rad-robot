module(..., package.seeall)

motion = {}

motion.keyframes =
  _HOME_..'/Data/webots_ash_lowerbody/keyframes.lua'

motion.walk = {}
motion.walk.module = 'walk_osc'
motion.walk.parameters = 
  _HOME_..'/Data/webots_ash_lowerbody/parameters_walk_osc.lua'

motion.stand = {}
motion.stand.module = 'stand_osc'
motion.stand.parameters = 
  _HOME_..'/Data/webots_ash_lowerbody/parameters_stand_osc.lua'

motion.step = {}
motion.step.module = nil
motion.step.parameters = nil

assert(Config, 'Need a pre-existing Config table!')
local vector = require'vector'
local T = require'Transform'

------------------------------------
-- For the arm FSM
-- Weights: cusage, cdiff, ctight, cshoulder, cwrist
local arm = {}

-- This comes after the walkInit
arm.ready = {}

arm.ready[1] =
	-- Pitch up
	{
	left = {
		via='jacobian_preplan',
		timeout=12,
		tr={0.19, 0.246, 0.17, 0, -65*DEG_TO_RAD,0}, --6D is accepted and converted to tr :)
		qArmGuess = vector.new{135, 0, 0, -135, 90, 45, -90}*DEG_TO_RAD,
		weights = {1,1,-1,1,2},
	},
	right = {
		via='jacobian_preplan',
		timeout=12,
		tr={0.19, -0.246, 0.08, 0, -65*DEG_TO_RAD, 0},
		qArmGuess = vector.new{135, 0, 0, -135, -90, -45, 90}*DEG_TO_RAD,
		weights = {1,1,-1,1,2},
	}
}

-- Now go to yaw
--[[
table.insert(arm.init,
	{
	left = {
		tr={0.25, 0.25, 0.18,    0, 0, -45*DEG_TO_RAD},
		qArmGuess = vector.new{70,25,-28, -150, 10,-80,-90}*DEG_TO_RAD,
		timeout=15,
		via='jacobian_preplan', weights = {0,1,0,1}
	},
	right = {
		tr={0.25, -0.25, 0.18, 0*DEG_TO_RAD, 0*DEG_TO_RAD, 45*DEG_TO_RAD},
		--qArmGuess = vector.new{70,-25,28, -150, 10,-80,-90}*DEG_TO_RAD,
		timeout=15,
		via='jacobian_preplan',
		weights = {0,1,0,1},
	}
})
--]]
arm.pushdoorup = {}
-- Weights: cusage, cdiff, ctight, cshoulder, cwrist
arm.pushdoordown = {}


arm.pushdoordown[1] = {
	left = {
		via='jacobian_preplan',
		timeout=8,
		tr={0.37, 0.31, -0.05, -90*DEG_TO_RAD, 0*DEG_TO_RAD,0},
		weights = {1,1,0}
	},
	right = false,
}


arm.down = {}
arm.down[1] = {
	left = false,
	right = {
		via='jacobian_preplan',
		timeout=5,
		tr={0.16, -0.285, -0.15, 0*DEG_TO_RAD, 0*DEG_TO_RAD,0},
		weights = {1,1,0}
	},
}

arm.plug = {}
arm.plug[1] = {
	right = {
		via='jacobian_preplan',
		timeout=8,
		tr={0.28, -0.285, 0.35, 0*DEG_TO_RAD, -90*DEG_TO_RAD,0*DEG_TO_RAD},
		weights = {1,1,0}
	},
	right = false,
}

















-- Weights: cusage, cdiff, ctight, cshoulder, cwrist
arm.valve = {}
--[[
table.insert(arm.valve, {
	right = false,
	left = {
		timeout=12,
		via='jacobian_preplan',
		tr={0.25, 0.3, 0.3, 0*DEG_TO_RAD, -80*DEG_TO_RAD, 0*DEG_TO_RAD},
		--tr={0.52, 0.43, 0.14, 0*DEG_TO_RAD, 0*DEG_TO_RAD, 0*DEG_TO_RAD},
		--qArmGuess = vector.new{-15, 60, 90, -120, -80, -70, 0}*DEG_TO_RAD,
		--weights = {0,1,0,1},
	}
})
--]]
--[[
table.insert(arm.valve, {
	right = false,
	left = {
		timeout=20,
		--via='jacobian_preplan',
		via='joint_preplan',
		--tr={0.45, 0.246, 0.14, 0*DEG_TO_RAD, 0*DEG_TO_RAD, 0*DEG_TO_RAD},
		tr={0.46, 0.27, 0.2, 0*DEG_TO_RAD, 0*DEG_TO_RAD, 0*DEG_TO_RAD}, -- seems ok for ICRA
		--qArmGuess = vector.new{-15, 60, 90, -120, -80, -70, 0}*DEG_TO_RAD,
		--weights = {1,1,1},
		weights = {1,0,-2, 0, 1}, -- Out
		--weights = {0,0,2, 0,1}, -- Tight

		-- Door
		--tr={0.8, 0.1, 0.246, 0*DEG_TO_RAD, 0*DEG_TO_RAD, 0*DEG_TO_RAD},
		--weights = {1,1,-1, 0,1},
		--qWaistGuess = {-45*DEG_TO_RAD,0},

		-- Drill

	}
})
--]]

table.insert(arm.valve, {
	left = false,
	right = {
		timeout=20,
		via='jacobian_preplan',
		--via='joint_preplan',
		tr={0.46, -0.27, 0.3, 0*DEG_TO_RAD, 0*DEG_TO_RAD, 60*DEG_TO_RAD}, -- seems ok for ICRA
		--qArmGuess = vector.new{-15, 60, 90, -120, -80, -70, 0}*DEG_TO_RAD,
		--weights = {1,1,1},
		weights = {1,0,-2, 0, 1}, -- Out
		--weights = {0,0,2, 0,1}, -- Tight

		-- Door
		--tr={0.8, 0.1, 0.246, 0*DEG_TO_RAD, 0*DEG_TO_RAD, 0*DEG_TO_RAD},
		--weights = {1,1,-1, 0,1},
		--qWaistGuess = {-45*DEG_TO_RAD,0},

		-- Drill

	}
})


--[[
table.insert(arm.valve, {
	right = false,
	left = {
		timeout=5,
		via='jacobian_preplan',
		tr={0.6, 0.3, 0.14, 0*DEG_TO_RAD, 0*DEG_TO_RAD, 0*DEG_TO_RAD},
		--qArmGuess = vector.new{-15, 60, 90, -120, -80, -70, 0}*DEG_TO_RAD,
		--weights = {0,1,0,1},
	}
})
table.insert(arm.valve, {
	right = false,
	left = {
		timeout=5,
		via='jacobian_preplan',
		tr={0.4, 0.3, 0.14, 0*DEG_TO_RAD, 0*DEG_TO_RAD, 0*DEG_TO_RAD},
		--qArmGuess = vector.new{-15, 60, 90, -120, -80, -70, 0}*DEG_TO_RAD,
		--weights = {0,1,0,1},
	}
})
--]]

-- Weights: cusage, cdiff, ctight, cshoulder, cwrist
-- Useful: t = m.rPlanner.forward(Body.get_rarm_command_position(), Body.get_safe_waist_command_position())
arm.drill = {}
-- Low Drill is on 1.1m table. 1.1-0.97 = 0.13 table top
-- High Drill is on 0.75m table. 0.75-0.97 = -0.22 table top
-- mid of drill handle is like 10cm higher... : 0.25 and -0.12
-- Left views the drill
-- Right grabs the drill

table.insert(arm.drill, {
	--left = false,
	left = {
		timeout=8,
		via='jacobian_preplan',
		tr={0.28, 0.23, 0.42, 0*DEG_TO_RAD, -80*DEG_TO_RAD, 0*DEG_TO_RAD},
		qArmGuess = vector.new{60, 10, 0, -135, 0, 30, 0}*DEG_TO_RAD,
		weights = {0,1,0,1},
	},
	right = {
		timeout=8,
		via='jacobian_preplan',
		tr={0.31, -0.23, 0.37, 0*DEG_TO_RAD, -80*DEG_TO_RAD, 0*DEG_TO_RAD},
		qArmGuess = vector.new{75, -10, -5, -135, -10, -12, 10}*DEG_TO_RAD,
		weights = {0,1,0,1},
	}
})
--[[
table.insert(arm.drill, {
	left = {
		timeout=10,
		--via='joint_preplan',
		--q = vector.new{0, 75, 90, -90, -90, -45, 0}*DEG_TO_RAD,
		via='jacobian_preplan',
--		tr={0.345, 0.03, 0.25, 0*DEG_TO_RAD, 0*DEG_TO_RAD, -75*DEG_TO_RAD},
		tr={0.31, 0.03, 0.25, 0*DEG_TO_RAD, 0*DEG_TO_RAD, -75*DEG_TO_RAD},
		qArmGuess = vector.new{0, 60, 90, -120, -90, -15, 0}*DEG_TO_RAD,
		weights = {0,1, 0, 1, 2},
	},
	right = false
})
--]]

--1.22m
table.insert(arm.drill, {
	left = false,
	right = {
		timeout=10,
		via='jacobian_preplan',
		--tr={0.28, -0.3, 0.3, 0*DEG_TO_RAD, 0*DEG_TO_RAD, 30*DEG_TO_RAD},
		tr={0.46, -0.27, 0.3, 0*DEG_TO_RAD, 0*DEG_TO_RAD, 0*DEG_TO_RAD},
		qArmGuess = vector.new{-20, -60, -90, -120, 0, -45, 0}*DEG_TO_RAD,
		weights = {1,1,-1,1, 0},
	}
})

-- Tip faces right
arm.drillright = {}
table.insert(arm.drillright, {
	left = {
		timeout=5,
		via='jacobian_preplan',
		tr={0.5, -0.25, 0.25, 0*DEG_TO_RAD, 0*DEG_TO_RAD, -80*DEG_TO_RAD},
	},
	right = {
		timeout=10,
		via='jacobian_preplan',
		--via='jacobian_waist_preplan',
		--qWaistGuess = {10*DEG_TO_RAD,0},
		tr={0.53, -0.43, 0.25, 0*DEG_TO_RAD, 0*DEG_TO_RAD, -45*DEG_TO_RAD},
	}
})

-- Tip faces right
arm.drillleft = {}
table.insert(arm.drillleft, {
	left = {
		timeout=5,
		via='jacobian_preplan',
		tr={0.5, -0.05, 0.25, 0*DEG_TO_RAD, 0*DEG_TO_RAD, -80*DEG_TO_RAD},
	},
	right = {
		timeout=5,
		via='jacobian_preplan',
		tr={0.7, -0.23, 0.25, 0*DEG_TO_RAD, 0*DEG_TO_RAD, 90*DEG_TO_RAD},
	}
})

arm.carry = {}

table.insert(arm.carry, {
	left = {
		timeout=5,
		via='jacobian_preplan',
		tr={0.5, 0.2, 0.3, 0*DEG_TO_RAD, 0*DEG_TO_RAD, -45*DEG_TO_RAD},
	},
	right = false
})
table.insert(arm.carry, {
	left = {
		via='jacobian_preplan',
		timeout=12,
		tr={0.19, 0.28, 0.17, 0, -65*DEG_TO_RAD,0},
		qArmGuess = vector.new{135, 0, 0, -135, 90, 45, -90}*DEG_TO_RAD,
		weights = {1,1,-1,1,2},
	},
	right = false
})
table.insert(arm.carry, {
	left = false,
	right = {
		timeout=5,
		via='jacobian_preplan',
		tr={0.55, -0.23, 0.4, 0*DEG_TO_RAD, 0*DEG_TO_RAD, 90*DEG_TO_RAD},
	}
})
--[[
table.insert(arm.carry, {
	left = false,
	right = {
		timeout=5,
		via='jacobian_preplan',
		tr={0.4, -0.18, 0.4, 0*DEG_TO_RAD, 0*DEG_TO_RAD, 60*DEG_TO_RAD},
	}
})
--]]
table.insert(arm.carry, {
	left = false,
	right = {
		timeout=5,
		via='jacobian_preplan',
		tr={0.15, -0.1, 0.15, 0*DEG_TO_RAD, 0*DEG_TO_RAD, 90*DEG_TO_RAD},
	}
})


arm.shower = {}
arm.shower[1] = {
	left = {
		via='jacobian_preplan',
		timeout=15,
		tr={0.28, 0.35, 0.75, 0, -90*DEG_TO_RAD,0},
		qArmGuess = vector.new{0, 0, 0, -90, 0, 0, 0}*DEG_TO_RAD,
		weights = {1,1,-1,1,2},
	},
	right = {
		via='jacobian_preplan',
		timeout=15,
		tr={0.3, -0.3, 0.7, 0, -80*DEG_TO_RAD, 0},
		qArmGuess = vector.new{0, 0, 0, -90, 0, 0, 0}*DEG_TO_RAD,
		weights = {1,1,-1,1,2},
	},
}













--Gripper end position offsets (Y is inside)
arm.handoffset = {}
--0.130 + 0.60+0.50
--arm.handoffset.gripper = {0.241,0,0} --Default gripper
arm.handoffset.gripper = {0.23,0,0} --Default gripper (VT)
--0.130+0.139+0.80-0.10
arm.handoffset.outerhook = {0.339,0,0.060} --Single hook (for door)
--0.130 + 0.140+0.80-0.10
--arm.handoffset.chopstick = {0.340,0,0} --Two rod (for valve)
--FROM EMPIRICAL DATA
arm.handoffset.chopstick = {0.440,0,0} --Two rod (for valve)
--New 3 finger gripper
arm.handoffset.gripper3 = {0.28,-0.05,0}




------------------------------------------------------------------------------
------------------------------------------------------------------------------

arm.torque={}
arm.torque.movement = 5
arm.torque.open = -10
arm.torque.grip_hose = 10
arm.torque.grip_drill = 10
arm.torque.grip_drill_trigger1 = 40
arm.torque.grip_drill_trigger2 = 40

arm.handoffset={}
local offset_ucla_hand = {0.15,0,0}
local offset_chipsticks = {0.30,0,0}
local offset_wrist = {0,0,0}
arm.handoffset.left = offset_wrist
arm.handoffset.right = offset_wrist
--arm.handoffset.right = offset_ucla_hand

--Walk arm pose and shoulder angle
arm.trLArm0 = {0.0, 0.25,-0.25,0,0,0}
arm.trRArm0 = {0.0, -0.25,-0.25,0,0,0}
arm.ShoulderYaw0=vector.new({-1,1})*DEG_TO_RAD

arm.vel_angular_limit = vector.new({20,20,20,20,30,30,30})*DEG_TO_RAD
arm.vel_angular_limit_init = vector.new({20,20,20,20,30,30,30})*DEG_TO_RAD


--faster
arm.vel_angular_limit_init = vector.new({20,20,20,20,30,60,30})*DEG_TO_RAD


arm.vel_linear_limit = vector.new({0.02,0.02,0.02, 30*DEG_TO_RAD,30*DEG_TO_RAD,30*DEG_TO_RAD})
arm.vel_waist_limit = vector.new({3,3})*DEG_TO_RAD
arm.shoulder_yaw_limit = 30*DEG_TO_RAD
arm.torso_comp_limit = vector.new({0.06,0.03})

--Old teleop init/uninit sequence
armfsm = {}
armfsm.teleopr = {}
armfsm.teleopr.arminit={
 	{'move0',nil,{0.25,-0.25,-0.15  ,0,0,0}},
  {'move0',nil,{0.30,-0.25,0.24,0,0,0}},
  {'move0',nil,{0.30,-0.10,0.24,0,0,0}},
}
armfsm.teleopr.armuninit={
  {'move0',nil,{0.30,-0.25, 0.24  ,0,0,0}},
  {'move0',nil,{0.25,-0.25,-0.15  ,0,0,0}},
  {'move0',nil,arm.trRArm0},
}
armfsm.teleopl = {}
armfsm.teleopl.arminit={
 	{'move0',{0.25,0.25,-0.15  ,0,0,0},nil},
--  {'move0',{0.30,0.25,0.24,0,0,0},nil},
--  {'move0',{0.30,0.10,0.24,0,0,0},nil},
}
armfsm.teleopl.armuninit={
--  {'move0',{0.30,0.25, 0.24  ,0,0,0},nil},
--  {'move0',{0.25,0.25,-0.15  ,0,0,0},nil},
  {'move0',arm.trLArm0,nil},
}


-- Export
Config.arm = arm
Config.armfsm  = armfsm

return Config

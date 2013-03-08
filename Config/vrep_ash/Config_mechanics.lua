module(..., package.seeall)

require('Transform')

mechanics = {}

mechanics.l_foot = {
  sole_dimensions = {0.230, 0.125},
  sole_transform = Transform.pose{0.0213876, 0, 0},
  heel_transform = Transform.pose{-0.0936124, 0, 0},
  toe_transform = Transform.pose{0.1363876, 0, 0},
  force_torque_transform = Transform.pose{0.00361603, 0.00027595, 0.01908749} -- FIXME add model transform !
}

mechanics.r_foot = {
  sole_dimensions = {0.230, 0.125},
  sole_transform = Transform.pose{0.0213876, 0, 0},
  heel_transform = Transform.pose{-0.0936124, 0, 0},
  toe_transform = Transform.pose{0.1363876, 0, 0},
  force_torque_transform = Transform.pose{0.00361603, 0.00027595, 0.01908749} -- FIXME add model transform !
}

mechanics.l_hand = {
  force_torque_transform = Transform.pose{0, 0, 0} -- FIXME
}

mechanics.r_hand = {
  force_torque_transform = Transform.pose{0, 0, 0} -- FIXME
}

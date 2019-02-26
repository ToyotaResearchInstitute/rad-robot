os.execute('clear')

if try_torch then
  T = require'libTransform'
else
  T = require'Transform'
end

q = require'quaternion'

angle0 = math.rad(45)
axis0 = {0,1,0}
angle1 = math.rad(45)
axis1 = {1,0,0}
t = 0
print(math.deg(angle0), unpack(axis0))
print(math.deg(angle1), unpack(axis1))
--
q0 = q.from_angle_axis(angle0,axis0)
q1 = q.from_angle_axis(angle1,axis1)
print('q0', unpack(q0))
print('q1', unpack(q1))
--
qSlerp = q.slerp(q0,q1,t)
angle, axis = q.angle_axis( qSlerp )
print(math.deg(angle), unpack(axis))

print('=======')
function randAA()
	local ang = 2*math.pi * math.random() - math.pi
	local axis = {
		math.random(),
		math.random(),
		math.random()
	}
	local anorm = math.sqrt(axis[1]*axis[1]+axis[2]*axis[2]+axis[3]*axis[3])
	return ang, {axis[1] / anorm, axis[2] / anorm, axis[3] / anorm}
end
local quats = {}
for i=1,100 do
	local ang, axis = randAA()
	print(ang, axis)
	quats[i] = q.from_angle_axis(ang, axis)
end

local fromQ = require'Transform'.from_quatp
local toQ = require'Transform'.to_quatp

local quatps = {}
for i,quat in ipairs(quats) do
	local quatp = {unpack(quat)}
	quatp[5] = math.random() * 10
	quatp[6] = math.random() * 10
	quatp[7] = math.random() * 10
	quatps[i] = v.new(quatp)
end

for i,quatp in ipairs(quatps) do
	print('*')
	print(quatp)
	local tr = fromQ(quatp)
	print(tr)
	local quatp1 = toQ(tr)
	--local tr1 = fromQ(quatp1)
	print("quatp", unpack(quatp))
	print("quatp1", unpack(quatp1))

	--print(quatp1-quatp)
end

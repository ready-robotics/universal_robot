




v = [0.15, -0.75, 0.4, -0.0001, -2.2358, 2.2045]

p = PyKDL.Vector(v[0],v[1],v[2])
R = PyKDL.Vector(v[3],v[4],v[5])
Rn = R.Normalize()

T = PyKDL.Frame()
T.p = PyKDL.Vector(v[0],v[1],v[2])
T.M = PyKDL.Rotation.Rot(R,Rn)

a, rot = T.M.GetRotAngle()

v_out = list(T.p)+[a*rot[0],a*rot[1],a*rot[2]]
print v
print v_out
p_out = PyKDL.Vector(T.p)







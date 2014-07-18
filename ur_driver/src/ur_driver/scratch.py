




v = [0.1926, -0.7, 0.5804, -0.1788, 2.3957, -1.885]

T = PyKDL.Frame()

T.p = PyKDL.Vector(v[0],v[1],v[2])

Rx = PyKDL.Rotation.RotX(v[3])
Ry = PyKDL.Rotation.RotY(v[4])
Rz = PyKDL.Rotation.RotZ(v[5])

# T.M = Rz*Ry*Rx
T.M = PyKDL.Rotation.EulerZYZ(v[3],v[4],v[5])

v_out = list(T.p)+list(T.M.GetEulerZYZ())
print v_out
v_out = list(T.p)+list(T.M.GetRPY())
print v_out






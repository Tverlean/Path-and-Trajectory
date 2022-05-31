import roboticstoolbox as rtb
from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH, PrismaticDH
import numpy as np

# link lengths in cm
a1 = float(input("a1 = ")) 
a2 = float(input("a2 = ")) 
a3 = float(input("a3 = ")) 

# link converted to meters
def mm_to_meter(a):
    m = 1000 # 1 meter = 1000 mm
    return a/m

a1 = mm_to_meter(a1)
a2 = mm_to_meter(a2)
a3 = mm_to_meter(a3)

# link limits converted to meters
lm2 = float(input("lm2 = ")) # 50mm
lm2 = mm_to_meter(lm2)

lm3 = float(input("lm3 = ")) # 40mm
lm3 = mm_to_meter(lm3)

# Create Links
# [robot variable]=DHRobot([RevoluteDH(d,r/a,alpha,offset)])
#PrismaticDH(theta,r,alpha,offset)
Cyl_Standard = DHRobot([
    RevoluteDH(a1,0,(0/180)*np.pi,0,qlim=[(-90/180)*np.pi,(90/180)*np.pi]),
    PrismaticDH((270/180)*np.pi,0,(270/180)*np.pi,a2,qlim=[0, lm2]),
    PrismaticDH(0,0,0,a3,qlim=[0, lm3]),
], name='Cylindrical')

print(Cyl_Standard)

# degrees to radian converter
def deg_to_rad(T):
    return (T/180.0)*np.pi


## q Paths
# for CYLINDRICAL joint variables = ([T1,d2,d3])
q_init = np.array([0,0,0]) # origin

q_pick = np.array([deg_to_rad(float(input("T1 = "))),
                mm_to_meter(float(input("d2 = "))),
                mm_to_meter(float(input("d3 = ")))]) # 1st path

q2 = np.array([deg_to_rad(float(input("T1 = "))),
                mm_to_meter(float(input("d2 = "))),
                mm_to_meter(float(input("d3 = ")))]) # 2nd path

q3 = np.array([deg_to_rad(float(input("T1 = "))),
                mm_to_meter(float(input("d2 = "))),
                mm_to_meter(float(input("d3 = ")))]) # 3rd path


# Trajectory commands
traj1 = rtb.jtraj(q_init,q_pick,50)#time vector or steps
print(traj1)
print(traj1.q)
traj2 = rtb.jtraj(q_pick,q2,50)
print(traj2)
print(traj2.q)
traj3 = rtb.jtraj(q2,q3,50)
print(traj3)

print(traj3.q)

#plot scale
x1 = -0.1
x2 = 0.1
y1 = -0.1
y2 = 0.1
z1 = -0.1
z2 = 0.1

# plot command
#for joint varaible vs Time(s) table
rtb.qplot(traj1.q)
rtb.qplot(traj2.q)
rtb.qplot(traj3.q)

# plot of trajectory
Cyl_Standard.plot(traj1.q,limits=[x1,x2,y1,y2,z1,z2],movie='Cyl_Standard_1.gif')
Cyl_Standard.plot(traj2.q,limits=[x1,x2,y1,y2,z1,z2],movie='Cyl_Standard_2.gif')
Cyl_Standard.plot(traj3.q,limits=[x1,x2,y1,y2,z1,z2],movie='Cyl_Standard_3.gif',block=True)

#Cyl_Standard.teach(jointlabels=1)

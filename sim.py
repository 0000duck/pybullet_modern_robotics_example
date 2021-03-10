import pybullet as p
import time
import pybullet_data
import numpy as np
import math
import modern_robotics as mr
def Inertia_matrix(ixx,iyy,izz,ixy,iyz,ixz):
	I = np.array([[ixx ,ixy ,ixz],[ixy ,iyy ,iyz ],[ixz ,iyz ,izz ]])
	return I

## setup
useMaximalCoordinates = False
p.connect(p.GUI)
p.setGravity(0, 0, -10)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

indy = p.loadURDF("indy7/indy7.urdf", [0, 0, -0.05],[0, 0, 0, 1])
numJoints = p.getNumJoints(indy)
p.resetBasePositionAndOrientation(indy, [0, 0, 0], [0, 0, 0, 1])
for i in range(0,numJoints):
	p.setJointMotorControl2(indy, i, p.POSITION_CONTROL, targetPosition=0, force=0)

timeStepId = p.addUserDebugParameter("timeStep", 0.001, 0.1, 0.01)
useRealTimeSim = False
p.setRealTimeSimulation(useRealTimeSim)

## MR
H1 = 0.3
H2 = 0.45
H3 = 0.350
H4 = 0.228
W1 = 0.0035
W2 = 0.183

S1 = np.array([0, 0,  1,  0, 0,  0])
S2 = np.array([0, -1,  0, H1,0 ,  0])
S3 = np.array([0, -1,  0, H1+H2, 0, 0])
S4 = np.array([0, 0,   1, -W1, 0, 0])
S5 = np.array([0, -1,  0, H1+H2+H3, 0, 0])
S6 = np.array([0, 0,  1, -W1-W2,0, 0])

M = np.array([[1 ,0 ,0 ,0],
    [0 ,1 ,0 ,-W1-W2 ],
    [0, 0 ,1 ,H1+H2+H3+H4],
    [0, 0, 0 ,1 ]]);

M01 = np.array([[1 ,0, 0, 0],
       [0 ,1 ,0 ,0],
       [0 ,0 ,1 ,H1],
       [0 ,0 ,0 ,1]])
M12 = np.array([[0 , -1  , 0  , 0],
      [0 ,  0   ,-1   ,0],
      [1  , 0  ,  0  , 0],
      [0  , 0 ,   0  , 1]])
M23 = np.array([[0 ,-1 ,0, H2],
       [1 ,0, 0 ,0],
       [0 ,0, 1, 0],
       [0 ,0, 0 ,1]])
M34 = np.array([[-1, 0 , 0, 0],
       [0 ,0 , -1 ,-H3],
       [0 ,-1, 0 ,W1],
       [0 ,0 ,0 ,1 ]])
M45 = np.array([[1 ,0, 0, 0],
      [0  ,0 ,-1, -W2 ],
      [0  ,1 ,0 ,0],
      [0  ,0 ,0 ,1 ]])
M56 = np.array([[1, 0, 0,0],
      [0 ,0 ,1 ,H4],
      [0 ,-1 ,0 ,0],
      [0 ,0 ,0 ,1 ]])

I0 = Inertia_matrix(0.00572623,0.00558989,0.00966674,0.00000251,-0.0000014,-0.00011380);
I1 = Inertia_matrix(0.15418559,0.12937017,0.05964415,-0.00000235,-0.04854267,0.00001739);
I2 = Inertia_matrix(0.2935698,0.28094142,0.03620609,-0.0000004,0.03727972,0.00001441);
I3 = Inertia_matrix(0.03424593,0.03406024,0.00450477,0.00000149,0.00186009,0.00000724);
I4 = Inertia_matrix(0.00670405,0.00279246,0.00619341,0.00000375,-0.00127967,0.0000015);
I5 = Inertia_matrix(0.00994891,0.00978189,0.00271492,0.00000014,-0.00093546,0.00000321);
I6 = Inertia_matrix(0.00043534,0.00044549,0.00059634,0.00000013,0.00000051,-0.00000002);

mass0 = 1.59306955;
mass1 = 11.8030102;
mass2 = 7.99292141;
mass3 = 2.99134127;
mass4 = 2.12317035;
mass5 = 2.28865091;
mass6 = 0.40083918;


G1 = np.eye(6)*mass1;
G1[0:3,0:3] = I1;
G2 = np.eye(6)*mass2;
G2[0:3,0:3] = I2;
G3 = np.eye(6)*mass3;
G3[0:3,0:3] = I3;
G4 = np.eye(6)*mass4;
G4[0:3,0:3] = I4;
G5 = np.eye(6)*mass5;
G5[0:3,0:3] = I5;
G6 = np.eye(6)*mass6;
G6[0:3,0:3] = I6;

print(G1)
Glist = np.array([G1, G2, G3,G4,G5,G6])
Mlist = np.array([np.eye(4),M01,M12,M23,M34,M45,M56])
Slist = np.array([S1,S2,S3,S4,S5,S6]).T

g = np.array([0, 0,-9.8])
thetalist = np.array([0,0,0,0,0,0])
dthetalist = np.array([0.1 ,0.1, 0.1, 0 ,0 ,0]);
taulist = np.array([0,0,0,0,0,0])
Ftip = np.array([0,0,0,0,0,0])

desired_thetalist = np.array([1.57,1.57,0,0,0,0])
desired_dthetalist = np.array([0.1 ,0.1, 0.1, 0.1 ,0.1 ,0.1]);
Kp = 10;


print(Mlist.shape)

while p.isConnected():
	timeStep = p.readUserDebugParameter(timeStepId)
	p.setTimeStep(timeStep)
	dt = timeStep
	jointStates = p.getJointStates(indy,[1,2,3,4,5,6])
	thetalist = np.array([jointStates[0][0],jointStates[1][0],jointStates[2][0],jointStates[3][0],jointStates[4][0],jointStates[5][0]])
	e_thetalist = desired_thetalist -thetalist ;
	dthetalist = Kp*(e_thetalist);
	ddthetalist = mr.ForwardDynamics(thetalist, dthetalist, taulist, g, Ftip, Mlist,  Glist, Slist)
	dthetalist  = dthetalist+ddthetalist*dt;                         
	thetalist = thetalist+dthetalist*dt;    
	for j in range(1,6):
		p.resetJointState(indy,j,thetalist[j-1])
	if (not useRealTimeSim):
		p.stepSimulation()
		time.sleep(timeStep)

#!/usr/bin/env python3

from __future__ import print_function
import rospy
from sensor_msgs.msg import JointState
import os
from markers import *
from meshmarkers import *
from funciones import *
from time import sleep

path = os.path.dirname(__file__)

# Create folder for files
try:
    os.mkdir(os.path.join(path, "datos_diffkine"))
except OSError as error:
    print(error)


folder = os.path.join(path, "datos_diffkine")




# Initialize the node
rospy.init_node("testKineControlPosition")
print('starting motion ... ')
# Publisher: publish to the joint_states topic
pub = rospy.Publisher('joint_states', JointState, queue_size=10)

# ------------------------ Mesh Objects ------------------------
# Conveyor belt objects
belt1 = MeshMarker("kr16_v4", "faja.dae")
belt2 = MeshMarker("kr16_v4", "faja.dae")
# Motor object
motor = MeshMarker("kr16_v4", "motor.dae")


# Belts pose (x, y, z, quaternion)
x_belt1 = np.array([-1.0, 0.5, 0.0])
x_belt2 = np.array([1.0, 1.5, 0.0])
rot_belts = -np.pi/2 # Rotation angle
belt_axis = np.array([0.0, 0.0, 1.0]) # Unit axis on z axis
belt_axis *= np.sin(rot_belts/2) #Quaternion format
belt_w = np.cos(rot_belts/2) # Quaternion format
belt_quat = np.hstack((belt_w, belt_axis))
belt_quat /= np.linalg.norm(belt_quat) # Normalized
pose_belt1 = np.hstack((x_belt1, belt_quat)) # Belt 1 pose
pose_belt2 = np.hstack((x_belt2, belt_quat)) # Belt 2 pose
belt1.pose(pose_belt1)
belt2.pose(pose_belt2)

# Motor pose (no initial rotation)
x_motor = np.array([-1.0, 0.5, 0.77])
xf_motor = np.array([1.0, 0.5, 0.77])
motor.position(x_motor)

# Initial robot position
x_r0 = np.array([0.0, 1.0, 1.0])

# Files for the logs
path = os.path.dirname(__file__)
fxcurrent = open(os.path.join(folder, "f_xcurrent.txt"), "w")                
fxdesired = open(os.path.join(folder, "f_xdesired.txt"), "w")
fq = open(os.path.join(folder, "f_q.txt"), "w")

# Markers for the current and desired positions
bmarker_current  = BallMarker(color['RED'])
bmarker_desired = BallMarker(color['GREEN'])

# Joint names
jnames = ('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6')



# Initial configuration
q0  = np.array([0.0, 0, 0, 0, 0, 0.0])
q0 = ikine_kr16(x_r0, q0)
xd = x_motor

# Resulting initial position (end effector with respect to the base link)
T = fkine_kr16(q0)
x0 = T[0:3,3]
print("Posicion inicial: ",x0)
print("Posicion deseada: ",xd)

# Red marker shows the achieved position
bmarker_current.xyz(x0)
# Green marker shows the desired position
bmarker_desired.xyz(xd)

# Instance of the JointState message
jstate = JointState()
# Values of the message
jstate.header.stamp = rospy.Time.now()
jstate.name = jnames
# Add the head joint value (with value 0) to the joints
jstate.position = q0

# Frequency (in Hz) and control period
freq = 100
dt = 1.0/freq
rate = rospy.Rate(freq)

# Initial joint configuration
q = copy(q0)

# Main loop
while not rospy.is_shutdown():
    # Current time (needed for ROS)
    jstate.header.stamp = rospy.Time.now()
    # Kinematic control law for position (complete here)
    # -----------------------------
    k = 1
    T = fkine_kr16(q)
    x = T[0:3,3]
    e = x - xd        # Pos Inicial - Pos Final
    ep = -k*e
    J = jacobian_position(q, delta=0.0001)
    qp = np.linalg.pinv(J).dot(ep)    
    q = q + dt * qp
    # -----------------------------

    # Log values                                                      
    fxcurrent.writelines(str(x[0])+' '+str(x[1]) +' '+str(x[2])+'\n')
    fxdesired.writelines(str(xd[0])+' '+str(xd[1])+' '+str(xd[2])+'\n')
    fq.writelines(str(q[0])+" "+str(q[1])+" "+str(q[2])+" "+str(q[3])+" "+
             str(q[4])+" "+str(q[5])+"\n")
    
    # Publish the message
    jstate.position = q
    pub.publish(jstate)
    belt1.publish()
    belt2.publish()
    #motor.position(x)
    motor.publish()
    bmarker_desired.xyz(xd)
    bmarker_current.xyz(x)
    
    epsilon  = 0.001
    if (np.linalg.norm(e)<epsilon):
            print("Punto 1 alcanzado")
            break

    # Wait for the next iteration
    rate.sleep()

q0 = ikine_kr16(x_motor, q0)
xd = xf_motor

sleep(1)

while not rospy.is_shutdown():
    # Current time (needed for ROS)
    jstate.header.stamp = rospy.Time.now()
    # Kinematic control law for position (complete here)
    # -----------------------------
    k = 1
    T = fkine_kr16(q)
    x = T[0:3,3]
    e = x - xd        # Pos Inicial - Pos Final
    ep = -k*e
    J = jacobian_position(q, delta=0.0001)
    qp = np.linalg.pinv(J).dot(ep)    
    q = q + dt * qp
    # -----------------------------

    # Log values                                                      
    fxcurrent.writelines(str(x[0])+' '+str(x[1]) +' '+str(x[2])+'\n')
    fxdesired.writelines(str(xd[0])+' '+str(xd[1])+' '+str(xd[2])+'\n')
    fq.writelines(str(q[0])+" "+str(q[1])+" "+str(q[2])+" "+str(q[3])+" "+
             str(q[4])+" "+str(q[5])+"\n")
    
    # Publish the message
    jstate.position = q
    pub.publish(jstate)
    belt1.publish()
    belt2.publish()
    motor.position(x)
    motor.publish()
    bmarker_desired.xyz(xd)
    bmarker_current.xyz(x)
    
    epsilon  = 0.001
    if (np.linalg.norm(e)<epsilon):
            print("Error es minimo :)")
            break

    # Wait for the next iteration
    rate.sleep()

print('ending motion ...')
fxcurrent.close()
fxdesired.close()
fq.close()


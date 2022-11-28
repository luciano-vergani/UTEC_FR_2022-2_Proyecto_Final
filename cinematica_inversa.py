#!/usr/bin/env python3
import rospy
from time import sleep
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from funciones import *
from meshmarkers import *
from markers import *



rospy.init_node("Keyboard_Ikine")
pub = rospy.Publisher('joint_states', JointState, queue_size=1000)

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
    
# Markers
bmarker = BallMarker(color['RED'])
bmarker_des = BallMarker(color['GREEN']) 

     
# Joint names
jnames = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

# Posicion inicial
q0 = np.array([0, 0, 0, 0, 0, 0])



# Posicion deseada
xd = np.array([0.91, 0.81, 0.85])

# Initial y actual configuration
qi=np.array([0.11346999,  0.18864725, -0.55529391, -0.03905625,  0.44990365,  0])

#Inverse kinematics
q= ikine_kr16(xd,qi)


if abs(q[0]) <= 3.2289 and abs(q[1]) <= 1.13446 and abs(q[2]) <= 2.26893 and abs(q[3]) <= 5.236 and abs(q[4]) <= 5.236 and abs(q[5]) <= 5.236:
        print("Limites Correctos")
        q = q
else:
        q = qi
        print("Limites Incorrectos")

print("Los limites son: ",q)

T = fkine_kr16(q)

print("La posicion deseada es: ",xd)
print("La posicion alcanzada es: ",T[0:3, 3])



# Loop rate (in Hz)
rate = rospy.Rate(100)

# Continuous execution loop
while not rospy.is_shutdown():

        # Red marker shows the achieved position
        
        
        



        # Object (message) whose type is JointState
        jstate = JointState()
        # Set values to the message
        jstate.header.stamp = rospy.Time.now()
        jstate.name = jnames
        # Add the head joint value (with value 0) to the joints
        jstate.position = q
        # Publish the message
        pub.publish(jstate)
        belt1.publish()
        belt2.publish()
        motor.position([1.0, 0.8, 0.77])
        motor.publish()


        # Wait for the next iteration
        rate.sleep()
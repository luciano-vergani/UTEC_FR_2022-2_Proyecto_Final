#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from markers import * 
from funciones import *
rospy.init_node("testForwardKinematics")
pub = rospy.Publisher('joint_states', JointState, queue_size=1000)
bmarker = BallMarker(color['GREEN'])
# Joint names
jnames = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
# Joint Configuration

q1=75*pi/180
q2=-25*pi/180
q3=40*pi/180
q4=100*pi/180
q5=0*pi/180
q6=0*pi/180

""" q1=0*pi/180
q2=0*pi/180
q3=0*pi/180
q4=0*pi/180
q5=0*pi/180
q6=0*pi/180 """

q = [q1,q2,q3,q4,q5,q6]  # Posicion Modificada

# End effector with respect to the base
T = fkine_kr16(q)
print(np.round(T, 3))
bmarker.position(T)
# Object (message) whose type is JointState
jstate = JointState()
# Set values to the message
jstate.header.stamp = rospy.Time.now()
jstate.name = jnames
# Add the head joint value (with value 0) to the joints
jstate.position = q
# Loop rate (in Hz)

rate = rospy.Rate(100)
# Continuous execution loop
while not rospy.is_shutdown():
    # Current time (needed for ROS)
    jstate.header.stamp = rospy.Time.now()
    # Publish the message
    pub.publish(jstate)
    bmarker.publish()
    # Wait for the next iteration
    rate.sleep()
#!/usr/bin/env python3
 
from __future__ import print_function
 
import rospy
from sensor_msgs.msg import JointState
from markers import *
from funciones import *
from roslib import packages
from meshmarkers import *
import rbdl
import os


def camino(pts, xsample):
    cam = []
    for i in range(len(pts)-1):
        # Get initial and final points
        p0 = pts[i]
        pf = pts[i + 1]

        # Get subintervals
        intervals = np.linspace(p0, pf, 4)
        
        #number of points per subinterval
        n = np.linalg.norm(pf-p0)/xsample
        nfast = int(n * 0.1)
        nmed = int(n * 0.2)
        nslow = int(n * 0.7)

        v = [[], [] ,[]]
        #first subinterval: medium speed
        v[0] = np.linspace(intervals[0], intervals[1], nfast)
        #second subinterval: fast
        v[1] = np.linspace(intervals[1], intervals[2], nmed)
        #third subinterval: slooooow
        v[2] = np.linspace(intervals[2], intervals[3], nslow)
        for j in v:
            for k in j:
                cam.append(k)
    return cam



path = os.path.dirname(__file__)

try:
    os.mkdir(os.path.join(path, "datos"))
except OSError as error:
    print(error)

folder = os.path.join(path, "datos")

# Archivos donde se almacenara los datos
fqact = open(os.path.join(folder, "qactual.txt"), "w")
fqdes = open(os.path.join(folder, "qdeseado.txt"), "w")
fxact = open(os.path.join(folder, "xactual.txt"), "w")
fxdes = open(os.path.join(folder, "xdeseado.txt"), "w")

rospy.init_node("control_pdg")
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
 
 

# Nombres de las articulaciones
jnames = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

# Objeto (mensaje) de tipo JointState
jstate = JointState()
# Valores del mensaje
jstate.header.stamp = rospy.Time.now()
jstate.name = jnames
 

# =============================================================
# Configuracion articular inicial (en radianes)
q0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

x_r0 = np.array([0.0, 1.0, 1.0])
q = ikine_kr16(x_r0, q0)
# Velocidad inicial
dq = np.array([0., 0., 0., 0., 0., 0.])
# =============================================================
 

# Copiar la configuracion articular en el mensaje a ser publicado
jstate.position = q
pub.publish(jstate)

# Modelo RBDL
modelo = rbdl.loadModel("../urdf/kr16_v4.urdf")
ndof   = modelo.q_size     # Grados de libertad

# Frecuencia del envio (en Hz)
freq = 1000
dt = 1.0/freq
rate = rospy.Rate(freq)

# Simulador dinamico del robot
robot = Robot(q, dq, ndof, dt)

# Se definen las ganancias del controlador
valores = 0.5*np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
Kp = np.diag(valores)
Kd = 2*np.sqrt(Kp)

# Bucle de ejecucion continua
t = 0.0


def control(qdes, i, t):
    while not rospy.is_shutdown():
        # Leer valores del simulador
        q  = robot.read_joint_positions()
        dq = robot.read_joint_velocities()

        # Posicion actual del efector final
        x = fkine_kr16(q)[0:3,3]
        # Posicion deseada del efector final
        xd = fkine_kr16(qdes)[0:3,3]
        # Tiempo actual (necesario como indicador para ROS)
        jstate.header.stamp = rospy.Time.now()


        # Almacenamiento de datos
        fxact.write(str(t)+' '+str(x[0])+' '+str(x[1])+' '+str(x[2])+'\n')
        fxdes.write(str(t)+' '+str(xd[0])+' '+str(xd[1])+' '+str(xd[2])+'\n')
        fqact.write(str(t)+' '+str(q[0])+' '+str(q[1])+' '+ str(q[2])+' '+ str(q[3])+' '+str(q[4])+' '+str(q[5])+'\n ')
        fqdes.write(str(t)+' '+str(qdes[0])+' '+str(qdes[1])+' '+ str(qdes[2])+' '+ str(qdes[3])+' '+str(qdes[4])+' '+str(qdes[5])+'\n ')
        
        # ----------------------------
        # Control dinamico (COMPLETAR)
        # ----------------------------
        # Vector de zeross
        zeros = np.zeros(ndof)
        # Vector de gravedad
        v_gravedad = np.zeros(ndof) 
        rbdl.InverseDynamics(modelo, q, zeros, zeros, v_gravedad)

        # Calculo de tau
        u = v_gravedad + Kp.dot(qdes - q) - Kd.dot(dq) 
        
        # Simulacion del robot
        robot.send_command(u)

        # Publicacion del mensaje
        jstate.position = q
        pub.publish(jstate)

        if(i>3):
            motor.position(x)

        belt1.publish()
        belt2.publish()
        motor.publish()

        t = t+dt

        if(np.linalg.norm(x-xd)< 0.05):
            break

        # Esperar hasta la siguiente  iteracion
        rate.sleep()
    


# Vector de posiciones deseadas

qvect = [x_r0, np.array([0.0, 1.0, 1.2]), np.array([-1.0, 0.5, 1.0]), x_motor, np.array([-1.0, -0.5, 1.0]),np.array([0.5, -0.5, 1.0]), np.array([1.0, 0.5, 1.0]), xf_motor]
qvect = [x_r0, x_motor, np.array([0.0, -1.5, 1.0]), xf_motor]
cm = camino(copy(qvect), 0.2)
qvect = cm
qdp = q0
print(len(qvect))

for i in range(len(qvect)):
    qd = ikine_kr16(qvect[i], qdp)
    qdp = qd
    control(qd, i, t)
    print(f"punto {i} alcanzado")

fqact.close()
fqdes.close()
fxact.close()
fxdes.close()

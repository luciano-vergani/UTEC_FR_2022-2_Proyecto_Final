from copy import copy
import numpy as np
import rbdl

cos = np.cos
sin = np.sin
pi = np.pi

class Robot(object):
    def __init__(self, q0, dq0, ndof, dt):
        self.q = q0    # numpy array (ndof x 1)
        self.dq = dq0  # numpy array (ndof x 1)
        self.M = np.zeros([ndof, ndof])
        self.b = np.zeros(ndof)
        self.dt = dt
        self.robot = rbdl.loadModel('../urdf/kr16_v4.urdf')

    def send_command(self, tau):
        rbdl.CompositeRigidBodyAlgorithm(self.robot, self.q, self.M)
        rbdl.NonlinearEffects(self.robot, self.q, self.dq, self.b)
        ddq = np.linalg.inv(self.M).dot(tau-self.b)
        self.q = self.q + self.dt*self.dq
        self.dq = self.dq + self.dt*ddq

    def read_joint_positions(self):
        return self.q

    def read_joint_velocities(self):
        return self.dq

def dh(d, theta, a, alpha):

# Escriba aqui la matriz de transformacion homogenea en funcion de los valores de d, theta, a, alpha

    T = np.array([[cos(theta), -cos(alpha)*sin(theta),  sin(alpha)*sin(theta), a*cos(theta)],
                  [sin(theta),  cos(alpha)*cos(theta), -sin(alpha)*cos(theta), a*sin(theta)],
                  [         0,             sin(alpha),             cos(alpha),            d],
                  [         0,                      0,                      0,            1]])
    return T

def fkine_kr16(q):
    """
    Calcular la cinematica directa del robot KR16 dados sus valores articulares.
    q es un vector numpy de la forma [q1, q2, q3, q4, q5, q6]
    """


    # Matrices DH (completar), emplear la funcion dh con los parametros DH para cada articulacion
    T1 = dh(0.65,   q[0]+pi/2,  00.26,       pi/2)
    T2 = dh(0.00,   q[1]+pi/2,  00.69,       00000)
    T3 = dh(0.00,   q[2] + 00,  00.00,        pi/2)
    T4 = dh(0.67,   q[3] + pi,  00.00,        pi/2)
    T5 = dh(0.00,   q[4] + pi,  00.00,        pi/2)
    T6 = dh(0.15,   q[5] + 00,  00.00,       00000)
   
    T =T1.dot(T2).dot(T3).dot(T4).dot(T5).dot(T6)
    return T

def jacobian_kr16(q, delta=0.0001):
    """
    Jacobiano analitico para la posicion. Retorna una matriz de 3x6 y toma como
    entrada el vector de configuracion articular q=[q1, q2, q3, q4, q5, q6]
    """
    # Crear una matriz 3x6
    J = np.zeros((3,6))
    # Transformacion homogenea inicial (usando q)
    Ti = fkine_kr16(q)
    
    # Iteracion para la derivada de cada columna
    for i in range(6):
        # Copiar la configuracion articular inicial
        dq = copy(q)
        # Incrementar la articulacion i-esima usando un delta
        dq[i]=dq[i]+delta
        # Transformacion homogenea luego del incremento (q+delta)
        Tf = fkine_kr16(dq)
        # Aproximacion del Jacobiano de posicion usando diferencias finitas
        J[0,i]=(Tf[0,3]-Ti[0,3])/delta
        J[1,i]=(Tf[1,3]-Ti[1,3])/delta
        J[2,i]=(Tf[2,3]-Ti[2,3])/delta
    return J

def ikine_kr16(xdes, q0):
    """
    Calcular la cinematica inversa de UR5 numericamente a partir de la configuracion articular inicial de q0.
    Emplear el metodo de newton
    """
    epsilon  = 0.001
    max_iter = 1000
    delta    = 0.00001

    q  = copy(q0)
    for i in range(max_iter):
        # Main loop
        J= jacobian_kr16(q,delta)
        T = fkine_kr16(q)
        f = T [0:3,3]
        e = xdes -f
        q = q + np.dot(np.linalg.pinv(J),e)
        if (np.linalg.norm(e)<epsilon):
            break
    return q

def jacobian_position(q, delta=0.0001):
    """
    Jacobiano analitico para la posicion. Retorna una matriz de 3x6 y toma como
    entrada el vector de configuracion articular q=[q1, q2, q3, q4, q5, q6]

    """
    # Alocacion de memoria
    J = np.zeros((3,6))
    # Transformacion homogenea inicial (usando q)
    T0 = fkine_kr16(q)
    x0 = T0[0:3, 3]
    x0 = x0.T
    # Iteracion para la derivada de cada columna
    for i in range(6):
        # Copiar la configuracion articular inicial (usar este dq para cada incremento en una articulacion)
        dq = copy(q)
        # Incrementar la articulacion i-esima usando un delta
        dq[i] = dq[i] + delta
        # Transformacion homogenea luego del incremento (q+dq)
        T = fkine_kr16(dq)
        x = T[0:3, 3]
        x = x.T
        # Aproximacion del Jacobiano de posicion usando diferencias finitas
        J[0:3,i] = (T[0:3, 3] - T0[0:3, 3])/delta

    return J


def jacobian_pose(q, delta=0.0001):
    """
    Jacobiano analitico para la posicion y orientacion (usando un
    cuaternion). Retorna una matriz de 7x6 y toma como entrada el vector de
    configuracion articular q=[q1, q2, q3, q4, q5, q6]

    """
    J = np.zeros((7,6))
    # Implementar este Jacobiano aqui
    T = fkine_kr16(q)
    for i in range(6):
        x = TF2xyzquat(T)
 
        dq = copy(q)
        dq[i] = dq[i] + delta
        Tf = fkine_kr16(dq)
        xf = TF2xyzquat(Tf)
 
        J[0,i] = (xf[0] - x[0])/delta
        J[1,i] = (xf[1] - x[1])/delta
        J[2,i] = (xf[2] - x[2])/delta
        J[3,i] = (xf[3] - x[3])/delta
        J[4,i] = (xf[4] - x[4])/delta
        J[5,i] = (xf[5] - x[5])/delta
        J[6,i] = (xf[6] - x[6])/delta
        
    return J



def rot2quat(R):
    """
    Convertir una matriz de rotacion en un cuaternion

    Entrada:
      R -- Matriz de rotacion
    Salida:
      Q -- Cuaternion [ew, ex, ey, ez]

    """
    dEpsilon = 1e-6
    quat = 4*[0.,]

    quat[0] = 0.5*np.sqrt(R[0,0]+R[1,1]+R[2,2]+1.0)
    if ( np.fabs(R[0,0]-R[1,1]-R[2,2]+1.0) < dEpsilon ):
        quat[1] = 0.0
    else:
        quat[1] = 0.5*np.sign(R[2,1]-R[1,2])*np.sqrt(R[0,0]-R[1,1]-R[2,2]+1.0)
    if ( np.fabs(R[1,1]-R[2,2]-R[0,0]+1.0) < dEpsilon ):
        quat[2] = 0.0
    else:
        quat[2] = 0.5*np.sign(R[0,2]-R[2,0])*np.sqrt(R[1,1]-R[2,2]-R[0,0]+1.0)
    if ( np.fabs(R[2,2]-R[0,0]-R[1,1]+1.0) < dEpsilon ):
        quat[3] = 0.0
    else:
        quat[3] = 0.5*np.sign(R[1,0]-R[0,1])*np.sqrt(R[2,2]-R[0,0]-R[1,1]+1.0)

    return np.array(quat)


def TF2xyzquat(T):
    """
    Convert a homogeneous transformation matrix into the a vector containing the
    pose of the robot.

    Input:
      T -- A homogeneous transformation
    Output:
      X -- A pose vector in the format [x y z ew ex ey ez], donde la first part
           is Cartesian coordinates and the last part is a quaternion
    """
    quat = rot2quat(T[0:3,0:3])
    res = [T[0,3], T[1,3], T[2,3], quat[0], quat[1], quat[2], quat[3]]
    return np.array(res)


def skew(w):
    R = np.zeros([3,3])
    R[0,1] = -w[2]; R[0,2] = w[1]
    R[1,0] = w[2];  R[1,2] = -w[0]
    R[2,0] = -w[1]; R[2,1] = w[0]
    return R
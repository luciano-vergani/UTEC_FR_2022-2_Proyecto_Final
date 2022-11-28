#!/usr/bin/env python3

import rbdl
import numpy as np


# Lectura del modelo del robot a partir de URDF (parsing)
modelo = rbdl.loadModel("../urdf/kr16_v4.urdf")

# Grados de libertad
ndof = modelo.q_size

# Configuracion articular
q = np.array([0.5, 0.2, 0.3, 0.8, 0.5, 0.6])
# Velocidad articular
dq = np.array([0.8, 0.7, 0.8, 0.6, 0.9, 1.0])
# Aceleracion articular
ddq = np.array([0.2, 0.5, 0.4, 0.3, 1.0, 0.5])

# Arrays numpy
zeros = np.zeros(ndof)          # Vector de ceros
tau   = np.zeros(ndof)          # Para torque
g     = np.zeros(ndof)          # Para la gravedad
c     = np.zeros(ndof)          # Para el vector de Coriolis+centrifuga
M     = np.zeros([ndof, ndof])  # Para la matriz de inercia
e     = np.eye(6)               # Vector identidad

# Torque dada la configuracion del robot
rbdl.InverseDynamics(modelo, q, dq, ddq, tau)

# Parte 1: Calcular vector de gravedad, vector de Coriolis/centrifuga,
# y matriz M usando solamente InverseDynamics

# Vector gravedad
rbdl.InverseDynamics(modelo, q, zeros, zeros, g)
print("Gravedad:")
print(np.round(g,2))

# Coriolis/centrifuga
rbdl.InverseDynamics(modelo, q, dq, zeros, c)
Co=c-g
print("Coriolis:")
print(np.round(Co,2))

# Matriz M usando solamente InverseDynamics
for i in range(ndof):
   mi = np.zeros(ndof)
   rbdl.InverseDynamics(modelo, q, zeros, e[i,:], mi)
   M[i,:]=mi-g
print("Matriz de inercia:")
print(np.round(M,2))

# Parte 2: Calcular M y los efectos no lineales b usando las funciones
# CompositeRigidBodyAlgorithm y NonlinearEffects. Almacenar los resultados
# en los arreglos llamados M2 y b2
b2 = np.zeros(ndof)          # Para efectos no lineales
M2 = np.zeros([ndof, ndof])  # Para matriz de inercia
 
rbdl.NonlinearEffects(modelo, q, dq, b2)
print("Efectos no lineales:")
b2=b2-g
print(np.round(b2,2))
rbdl.CompositeRigidBodyAlgorithm(modelo, q, M2)
print("Matriz usando CRBA:")
print(np.round(M2,2))

# Parte 2: Verificacion de valores

# Verificacion de la matriz
M=np.round(M,2)
M2=np.round(M2,2)
print("Matriz usando Inverse Dynamics:")
print(np.round(M,2))
print("Matriz usando CRBA:")
print(np.round(M2,2))
print("Comprobacion:")
print(M2==M)

# Parte 3: Verificacion de la expresion de la dinamica

# Verificacion del Corolosis
Co=np.round(Co,2)
b2=np.round(b2,2)
print("Valores de Corolosis Inverse Dynamics:")
print(Co)
print("Valores de Corolosis con NonlinearEffects:")
print(b2)
print("Comprobacion:")
print(b2==Co)
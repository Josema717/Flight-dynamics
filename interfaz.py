import numpy as np
from stl import mesh
import pyvista as pv
import os 
from pyvista import _vtk
import pprint
import matplotlib.pyplot as plt

from kivy.app import App
from kivy.uix.widget import Widget
from kivy.graphics import PushMatrix, PopMatrix, Rotate, Translate
from kivy.uix.image import Image
from kivy.clock import Clock
import math
#Convencion de NED: X norte, Y este, Z abajo

#Inicializar variables
alpha, beta, climb, u, v, w, p, q, r, phi, theta, psi = 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0

# Voy a hacer un menu sencillo para que el usuario pueda elegir cual de los tres escenarios de vuelo quiere ver

respuesta = input("Por favor ingrese el numero del caso de vuelo que desea visualizar: \n1. Vuelo recto y nivelado\n2. Ascenso\n3. Giro\n")
while respuesta not in ["1", "2", "3"]:
        respuesta = input("Opcion no valida, por favor ingrese 1, 2 o 3: ")
if respuesta == "1":
    phi = 0
    theta = 0
    psi = 0
    u = 100
    v = 0
    w = 0
    print("Para el vuelo recto y nivelado, vamos a trabajar con estos angulos {phi, theta, psi} = {0, 0, 0} grados y con velocidades en el cuerpo de {u, v, w} = {100, 0, 0} m/s    ")
elif respuesta == "2":
    phi = 0
    theta = 10
    psi = 0
    u = 100
    v = 0
    w = 0
    print("Para el ascenso, vamos a trabajar con estos angulos {phi, theta, psi} = {0, 10, 0} grados y con velocidades en el cuerpo de {u, v, w} = {100, 0, 0} m/s    ")
elif respuesta == "3":
    phi = 20
    theta = 5
    psi = 30
    u = 100
    v = 0
    w = 0
    print("Para el giro, vamos a trabajar con estos angulos {phi, theta, psi} = {20, 5, 30} grados y con velocidades en el cuerpo de {u, v, w} = {100, 0, 0} m/s    ")
    
v_body = np.array([u, v, w])

#Aca viene la parte logica, lo que voy a hacer es crear una funcion donde voy a simualar una IMU y meter p,q,r
# la funcion va a aplicar la matriz de transformacion para llegar a las angular rates, las cuales voy a integrar para obtener los angulos de Euler,
#  y luego voy a usar esos angulos para rotar el modelo 3D del avion, y asi mostrar la orientacion del avion en cada caso de vuelo.

def rotation_matrix(phi, theta, psi, v_body):
    phi_rad = np.radians(phi)
    theta_rad = np.radians(theta)
    psi_rad = np.radians(psi)
    R_z = np.array([[np.cos(psi_rad), -np.sin(psi_rad), 0],
                    [np.sin(psi_rad), np.cos(psi_rad), 0],
                    [0, 0, 1]])
    R_y = np.array([[np.cos(theta_rad), 0, np.sin(theta_rad)],
                    [0, 1, 0],
                    [-np.sin(theta_rad), 0, np.cos(theta_rad)]])
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(phi_rad), -np.sin(phi_rad)],
                    [0, np.sin(phi_rad), np.cos(phi_rad)]])
    R_zyx = R_z @ R_y @ R_x
    v_NED = R_zyx @ v_body
    return R_zyx, v_NED, phi, theta, psi

R_zyx, v_NED, phi, theta, psi = rotation_matrix(phi, theta, psi, v_body)


#Este coso que dio el profe, todavia no entiendo para que sirve, pero lo dejo por las dudas.
def aircraft_state(alpha, beta, climb ,u, v, w, p, q, r, phi, theta, psi, v_body, v_NED=v_NED):
    "Returns aircraft state values in a structured format"
    state_values = {
        "angles": {
            "alpha": alpha, #Angle of attack [deg]
            "beta": beta,   #Sideslip angle [deg]
            "gamma": climb, #Climb angle [deg]
        },
        "velocities_body": np.array([u, v, w]), #Velocities in body frame [m/s]
        "velocities_ned": v_NED,  #Velocities in NED frame [m/s]
        "angular_rates": np.array([p, q, r]),   #Angular rates in body frame [rad/s]
        "attitude": np.array([phi, theta, psi]), #Euler angles: roll, pitch, yaw [deg]
    }
    return state_values

estado = aircraft_state(alpha=alpha, beta=beta, climb=climb, u=u, v=v, w=w, p=p, q=q, r=r, phi=phi, theta=theta, psi=psi, v_body=v_body)
pprint.pprint(estado)


ruta = os.path.join(os.path.dirname(__file__), "kf21v3.stl")

# ------------------ CARGAR MODELO ------------------
mesh = pv.read(ruta)

# Centrar el modelo en el origen
mesh.translate(-np.array(mesh.center), inplace=True)

# ------------------ MATRIZ NED → PYVISTA ------------------
C_ned_to_pv = np.array([
    [0, 1, 0],   # X_ned → Y_pv
    [1, 0, 0],   # Y_ned → X_pv
    [0, 0, -1]   # Z_ned → -Z_pv
])

# ------------------ ROTAR AVIÓN ------------------
mesh_rotated = mesh.copy()
mesh_rotated.points = (C_ned_to_pv @ R_zyx @ mesh.points.T).T

# ------------------ EJES BODY ------------------
longitud_ejes = mesh.length * 0.5

eje_x_body = pv.Arrow(start=(0,0,0), direction=(0,-1,0), scale=longitud_ejes)
eje_y_body = pv.Arrow(start=(0,0,0), direction=(-1,0,0), scale=longitud_ejes)
eje_z_body = pv.Arrow(start=(0,0,0), direction=(0,0,-1), scale=longitud_ejes)

# Función para rotar cualquier objeto (malla o eje)
def rotar_actor(actor, R, C):
    puntos = actor.points
    puntos_rotados = (C @ R @ puntos.T).T
    actor.points = puntos_rotados

# Rotar ejes con la misma transformación del avión
rotar_actor(eje_x_body, R_zyx, C_ned_to_pv)
rotar_actor(eje_y_body, R_zyx, C_ned_to_pv)
rotar_actor(eje_z_body, R_zyx, C_ned_to_pv)

# ------------------ VISUALIZACIÓN ------------------
plotter = pv.Plotter()

# Avión
plotter.add_mesh(mesh_rotated, color="gray")

# Ejes body (ya rotados correctamente)
plotter.add_mesh(eje_x_body, color="red")
plotter.add_mesh(eje_y_body, color="white")
plotter.add_mesh(eje_z_body, color="blue")

# ------------------ SISTEMA NED VISUAL ------------------
transform = _vtk.vtkTransform()
transform.RotateX(180)
transform.RotateZ(90)

axes_NED = _vtk.vtkAxesActor()
axes_NED.SetTotalLength(1, 1, 1)
axes_NED.SetUserTransform(transform)

plotter.add_orientation_widget(axes_NED)

plotter.show_grid()
plotter.show()

# Funcion para plotear los tres planos de actitud (pitch, roll, heading)

def plot_attitude(phi, theta, psi):
    fig, axs = plt.subplots(1, 3, figsize=(15, 5))

    # 1. PITCH (plano XZ)
    ax = axs[0]
    ax.axhline(0)  # horizonte

    theta_rad = np.radians(theta)
    length = 1

    x = [-length*np.cos(theta_rad), length*np.cos(theta_rad)]
    z = [-length*np.sin(theta_rad), length*np.sin(theta_rad)]

    ax.plot(x, z)

    ax.set_title(f"Pitch (θ = {theta}°)")
    ax.set_xlim(-1.5, 1.5)
    ax.set_ylim(-1.5, 1.5)
    ax.set_aspect('equal')
    ax.grid()

    # -------------------------
    # 2. HEADING (plano XY)
    # -------------------------
    ax = axs[1]

    psi_rad = np.radians(psi)

    x = np.sin(psi_rad)
    y = np.cos(psi_rad)

    ax.arrow(0, 0, x, y, head_width=0.1)

    ax.set_title(f"Heading (ψ = {psi}°)")
    ax.set_xlim(-1.5, 1.5)
    ax.set_ylim(-1.5, 1.5)
    ax.set_aspect('equal')
    ax.grid()

    # -------------------------
    # 3. ROLL (plano YZ)
    # -------------------------
    ax = axs[2]

    ax.axhline(0)  # horizonte

    phi_rad = np.radians(phi)

    y = [-length*np.cos(phi_rad), length*np.cos(phi_rad)]
    z = [-length*np.sin(phi_rad), length*np.sin(phi_rad)]

    ax.plot(y, z)

    ax.set_title(f"Roll (φ = {phi}°)")
    ax.set_xlim(-1.5, 1.5)
    ax.set_ylim(-1.5, 1.5)
    ax.set_aspect('equal')
    ax.grid()

    plt.tight_layout()
    plt.show()

# Ejemplo
plot_attitude(phi=phi, theta=theta, psi=psi)
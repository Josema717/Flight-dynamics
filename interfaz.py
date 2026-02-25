import numpy as np
import pprint
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.transforms import Affine2D
#Convencion de NED: X norte, Y este, Z abajo

#Inicializar variables
alpha, beta, climb, u, v, w, p, q, r, phi, theta, psi = 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0

# Voy a hacer un menu sencillo para que el usuario pueda elegir cual de los tres escenarios de vuelo quiere ver o si quiere uno personalizado

respuesta = input("Por favor ingrese el numero del caso de vuelo que desea visualizar: \n1. Vuelo recto y nivelado\n2. Ascenso\n3. Giro\n4. Personalizado\n")
while respuesta not in ["1", "2", "3", "4"]:
        respuesta = input("Opcion no valida, por favor ingrese 1, 2, 3 o 4: ")
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
    w = 10
    print("Para el giro, vamos a trabajar con estos angulos {phi, theta, psi} = {20, 5, 30} grados y con velocidades en el cuerpo de {u, v, w} = {100, 0, 10} m/s    ")
elif respuesta == "4":
    phi = float(input("Ingrese el angulo de roll (phi) en grados: "))
    theta = float(input("Ingrese el angulo de pitch (theta) en grados: "))
    psi = float(input("Ingrese el angulo de yaw (psi) en grados: "))
    u = float(input("Ingrese la velocidad en el eje X del cuerpo (u) en m/s: "))
    v = float(input("Ingrese la velocidad en el eje Y del cuerpo (v) en m/s: "))
    w = float(input("Ingrese la velocidad en el eje Z del cuerpo (w) en m/s: "))
    print(f"Para el caso personalizado, vamos a trabajar con estos angulos {{phi, theta, psi}} = {{{phi}, {theta}, {psi}}} grados y con velocidades en el cuerpo de {{u, v, w}} = {{{u}, {v}, {w}}} m/s    ")


v_body = np.array([u, v, w])

# La matriz de rotacion para cambiar del body al NED

def rotation_matrix(phi, theta, psi, v_body):
    phi_rad = np.radians(phi)
    theta_rad = np.radians(theta)
    psi_rad = np.radians(psi)

    # Esta es la matriz transpuesta, ya que pensamos ir del body al NED.
    
    R_z = np.array([[np.cos(psi_rad), np.sin(psi_rad), 0],
                    [-np.sin(psi_rad), np.cos(psi_rad), 0],
                    [0, 0, 1]])
    R_y = np.array([[np.cos(theta_rad), 0, -np.sin(theta_rad)],
                    [0, 1, 0],
                    [np.sin(theta_rad), 0, np.cos(theta_rad)]])
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(phi_rad), np.sin(phi_rad)],
                    [0, -np.sin(phi_rad), np.cos(phi_rad)]])
    R_zyx = R_z @ R_y @ R_x # Del NED al body
    R_body_to_NED = R_zyx.T # Transpuesta para ir del body al NED
    v_NED = R_body_to_NED @ v_body
    return R_body_to_NED, v_NED, phi, theta, psi

R_body_to_NED, v_NED, phi, theta, psi = rotation_matrix(phi, theta, psi, v_body)

# AERODYNAMIC ANGLES

def angle_of_attack(u, w):
    """
    Alpha (α) — Angle of Attack [deg]
    Angle between the velocity vector projected on the XZ body plane and
    the body X-axis. Defined as atan2(w, u).
    """
    alpha = np.rad2deg(np.arctan2(w, u))
    return alpha

alpha = angle_of_attack(u, w)

def sideslip_angle(u, v, w):
    """
    Beta (β) — Sideslip Angle [deg]
    Angle between the total velocity vector and the body XZ plane.
    Defined as atan2(v, sqrt(u²+w²)) or equivalently asin(v/V).
    """
    V = np.sqrt(u**2 + v**2 + w**2)
    return np.rad2deg(np.arcsin(v/V))


def climb_angle(alpha_deg, pitch_deg):
    """
    Gamma (γ) — Climb Angle [deg]
    Relationship: pitch = alpha + gamma  →  gamma = pitch - alpha
    Valid when sideslip is zero (wings-level flight).
    """
    return pitch_deg - alpha_deg


#Este coso que dio el profe, todavia no entiendo para que sirve, pero lo dejo por las dudas.
def aircraft_state(alpha, beta, climb ,u, v, w, p, q, r, phi, theta, psi, v_body, v_NED=v_NED):
    "Returns aircraft state values in a structured format"
    state_values = {
        "angles": {
            "alpha": float(alpha), #Angle of attack [deg]
            "beta": float(beta),   #Sideslip angle [deg]
            "gamma": float(climb), #Climb angle [deg]
        },
        "velocities_body": np.array([u, v, w]), #Velocities in body frame [m/s]
        "velocities_ned": v_NED,  #Velocities in NED frame [m/s]
        "angular_rates": np.array([p, q, r]),   #Angular rates in body frame [rad/s]
        "attitude": np.array([phi, theta, psi]), #Euler angles: roll, pitch, yaw [deg]
    }
    return state_values

estado = aircraft_state(alpha=angle_of_attack(u,w), beta=sideslip_angle(u,v,w), climb=climb_angle(angle_of_attack(u,w),theta), u=u, v=v, w=w, p=p, q=q, r=r, phi=phi, theta=theta, psi=psi, v_body=v_body)
pprint.pprint(estado)

# Interfaz grafica 


# Funcion para plotear los tres planos de actitud (pitch, roll, heading)
phi = phi     # roll
theta = theta   # pitch
psi = psi     # yaw

# Cargar las imagenes de los aviones en las tres vistas

img_top = mpimg.imread("top.png")      # vista superior
img_side = mpimg.imread("side.png")    # vista lateral
img_front = mpimg.imread("front.png")  # vista frontal

fig, axs = plt.subplots(1, 3, figsize=(12,4))

#Ejes de referencia

# Yaw (vista superior)
ax = axs[0]

# Crear los ejes de referencia fijos
ax.arrow(0, 0, 0, 1, head_width=0.1, head_length=0.1, fc='pink', ec='pink', linewidth=2, zorder=1)
ax.arrow(0, 0, 1, 0, head_width=0.1, head_length=0.1, fc='purple', ec='purple', linewidth=2, zorder=1)
ax.text(1.1, 0, 'E', fontsize=10, color='purple', fontweight='bold')
ax.text(0, 1.1, 'N', fontsize=10, color='pink', fontweight='bold')

im = ax.imshow(img_top, extent=[-1,1,-1,1], zorder=2)

psi = -psi  # Invertir el ángulo de yaw para que la rotación sea en la dirección correcta
trans = Affine2D().rotate_deg(psi) + ax.transData
im.set_transform(trans)

ax.arrow(0, 0, 1, 0, fc='red', ec='red',
         transform=trans, zorder=3)
ax.arrow(0, 0, 0, 1, fc='green', ec='green',
         transform=trans, zorder=3)

ax.text(1.1, 0, 'y_b', color='red', transform=trans)
ax.text(0, 1.1, 'x_b', color='green', transform=trans)

ax.set_title(f"Yaw (ψ) = {psi}°")
ax.set_xlim(-2,2)
ax.set_ylim(-2,2)
ax.axis("off")


# Pitch (vista lateral)
ax = axs[1]

# Crear los ejes de referencia fijos
ax.arrow(0, 0, 1, 0, head_width=0.1, head_length=0.1, fc='purple', ec='purple', linewidth=2, zorder=1)
ax.arrow(0, 0, 0, -1, head_width=0.1, head_length=0.1, fc='black', ec='black', linewidth=2, zorder=1)
ax.text(1.1, 0, 'X', fontsize=10, color='purple', fontweight='bold')
ax.text(0, -1.1, 'Z', fontsize=10, color='black', fontweight='bold')

im = ax.imshow(img_side, extent=[-1,1,-1,1], zorder=2)

trans = Affine2D().rotate_deg(theta) + ax.transData
im.set_transform(trans)

ax.arrow(0, 0, 1, 0, fc='red', transform=trans)
ax.arrow(0, 0, 0, -1, fc='blue', transform=trans)

ax.text(1.1, 0, 'x_b', color='red', transform=trans)
ax.text(0, -1.1, 'z_b', color='blue', transform=trans)


ax.set_title(f"Pitch (θ) = {theta}°")
ax.set_xlim(-2,2)
ax.set_ylim(-2,2)
ax.axis("off")

# Roll (vista frontal)

ax = axs[2]

# Crear los ejes de referencia fijos
ax.arrow(0, 0, -1, 0, head_width=0.1, head_length=0.1, fc='pink', ec='pink', linewidth=2, zorder=1)
ax.arrow(0, 0, 0, -1, head_width=0.1, head_length=0.1, fc='black', ec='black', linewidth=2, zorder=1)
ax.text(-1.1, 0, 'Y', fontsize=10, color='pink', fontweight='bold')
ax.text(0, -1.1, 'Z', fontsize=10, color='black', fontweight='bold')

im = ax.imshow(img_front, extent=[-1,1,-1,1], zorder=2)
phi = -phi  # Invertir el ángulo de roll para que la rotación sea en la dirección correcta
trans = Affine2D().rotate_deg(phi) + ax.transData
im.set_transform(trans)

# Ejes del body, los cuales rotan con la imagen
ax.arrow(0, 0, -1, 0, fc='green', transform=trans)
ax.arrow(0, 0, 0, -1, fc='blue', transform=trans)
ax.text(-1.1, 0, 'y_b', color='green', transform=trans)
ax.text(0, -1.1, 'z_b', color='blue', transform=trans)

ax.set_title(f"Roll (φ) = {phi}°")
ax.set_xlim(-2,2)
ax.set_ylim(-2,2)
ax.axis("off")

state = {
    "u": u,
    "v": v,
    "w": w,
    "alpha": alpha,
    "beta": beta,
    "climb": climb_angle(angle_of_attack(u,w),theta),
    "phi": phi,
    "theta": theta,
    "psi": psi
}

fig, ax = plt.subplots()
ax.axis('off')

text = (
    f"Velocity:\n"
    f"u = {state['u']} ft/s\n"
    f"v = {state['v']} ft/s\n"
    f"w = {state['w']} ft/s\n\n"
    f"Angles:\n"
    f"alpha = {state['alpha']}°\n"
    f"beta  = {state['beta']}°\n\n"
    f"climb = {state['climb']}°\n\n"
    f"Euler:\n"
    f"phi = {state['phi']}°\n"
    f"theta = {state['theta']}°\n"
    f"psi = {state['psi']}°"
)

ax.text(0.05, 0.95, text, fontsize=12, va='top', family='monospace')

fig, ax = plt.subplots()
ax.axis('off')  # quitar ejes


ax.text(0.1, 0.5, text, fontsize=12)


# Eje de referencia NED en 3d
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Tama;o de los ejes del NED
axis_length = 125

# Norte (X)
ax.quiver(0, 0, 0, axis_length, 0, 0)
ax.text(axis_length, 0, 0, 'N', color='b')

# Este (Y)
ax.quiver(0, 0, 0, 0, axis_length, 0)
ax.text(0, axis_length, 0, 'E', color='b')

# Down (Z)
ax.quiver(0, 0, 0, 0, 0, axis_length)
ax.text(0, 0, axis_length, 'D', color='b')

# Vector de velocidad en el NED
ax.quiver(0, 0, 0, v_NED[0], v_NED[1], v_NED[2], color ='r')
ax.text(v_NED[0], v_NED[1], v_NED[2], 'V', color='k')

ax.set_xlim([0, axis_length])
ax.set_ylim([0, axis_length])
ax.set_zlim([0, axis_length])

# Ponerle titulo a cada eje
ax.set_xlabel('North')
ax.set_ylabel('East')
ax.set_zlabel('Down')

ax.set_title("Sistema NED con vector de velocidad")

# Mostrar la actitud de la aeronave en el NED

x_body = np.array([1, 0, 0])  # eje del avión
x_ned = R_body_to_NED @ x_body

scale = 80  # tamaño de la flecha

ax.quiver(0, 0, 0,
          x_ned[0]*scale,
          x_ned[1]*scale,
          x_ned[2]*scale,
          color='orange', linewidth=3)

ax.text(x_ned[0]*scale,
        x_ned[1]*scale,
        x_ned[2]*scale,
        'Attitude', color='orange')

plt.tight_layout()
plt.show()
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
    phi = -20
    theta = 5
    psi = 30
    u = 100
    v = 0
    w = 0
    print("Para el giro, vamos a trabajar con estos angulos {phi, theta, psi} = {-20, 5, 30} grados y con velocidades en el cuerpo de {u, v, w} = {100, 0, 0} m/s    ")
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

# =============================================================================
# AERODYNAMIC ANGLES
# =============================================================================

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
    return np.rad2deg(np.arctan2(v, np.sqrt(u**2 + w**2)))
beta = sideslip_angle(u, v, w)


def climb_angle(alpha_deg, pitch_deg):
    """
    Gamma (γ) — Climb Angle [deg]
    Relationship: pitch = alpha + gamma  →  gamma = pitch - alpha
    Valid when sideslip is zero (wings-level flight).
    """
    return pitch_deg - alpha_deg
gamma = climb_angle(alpha, theta)

climb = climb_angle(angle_of_attack(u, w), theta)

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

estado = aircraft_state(alpha=angle_of_attack(u,w), beta=sideslip_angle(u,v,w), climb=climb_angle(alpha,theta), u=u, v=v, w=w, p=p, q=q, r=r, phi=phi, theta=theta, psi=psi, v_body=v_body)
pprint.pprint(estado)

# Funcion para plotear los tres planos de actitud (pitch, roll, heading)

# ===== ANGULOS =====
phi = phi     # roll
theta = theta   # pitch
psi = psi     # yaw

# ===== CARGAR IMAGENES =====
img_top = mpimg.imread("top.png")      # vista superior
img_side = mpimg.imread("side.png")    # vista lateral
img_front = mpimg.imread("front.png")  # vista frontal

fig, axs = plt.subplots(1, 3, figsize=(12,4))

#Ejes de referencia

# =========================
# YAW (vista superior)
# =========================
ax = axs[0]

# Ejes de referencia (sin rotación - fijos)
ax.arrow(0, 0, 0, 1, head_width=0.1, head_length=0.1, fc='pink', ec='pink', linewidth=2, zorder=1)
ax.arrow(0, 0, 1, 0, head_width=0.1, head_length=0.1, fc='purple', ec='purple', linewidth=2, zorder=1)
ax.text(1.1, 0, 'E', fontsize=10, color='purple', fontweight='bold')
ax.text(0, 1.1, 'N', fontsize=10, color='pink', fontweight='bold')

im = ax.imshow(img_top, extent=[-1,1,-1,1], zorder=2)

psi = -psi  # Invertir el ángulo de yaw para que la rotación sea en la dirección correcta
trans = Affine2D().rotate_deg(psi) + ax.transData
im.set_transform(trans)

ax.set_title(f"Yaw (ψ) = {psi}°")
ax.set_xlim(-2,2)
ax.set_ylim(-2,2)
ax.axis("off")

# =========================
# PITCH (vista lateral)
# =========================
ax = axs[1]

# Ejes de referencia (sin rotación - fijos)
ax.arrow(0, 0, 1, 0, head_width=0.1, head_length=0.1, fc='purple', ec='purple', linewidth=2, zorder=1)
ax.arrow(0, 0, 0, -1, head_width=0.1, head_length=0.1, fc='black', ec='black', linewidth=2, zorder=1)
ax.text(1.1, 0, 'X', fontsize=10, color='purple', fontweight='bold')
ax.text(0, -1.1, 'Z', fontsize=10, color='black', fontweight='bold')

im = ax.imshow(img_side, extent=[-1,1,-1,1], zorder=2)

trans = Affine2D().rotate_deg(theta) + ax.transData
im.set_transform(trans)

ax.set_title(f"Pitch (θ) = {theta}°")
ax.set_xlim(-2,2)
ax.set_ylim(-2,2)
ax.axis("off")

# =========================
# ROLL (vista frontal)
# =========================
ax = axs[2]

# Ejes de referencia (sin rotación - fijos)
ax.arrow(0, 0, -1, 0, head_width=0.1, head_length=0.1, fc='pink', ec='pink', linewidth=2, zorder=1)
ax.arrow(0, 0, 0, -1, head_width=0.1, head_length=0.1, fc='black', ec='black', linewidth=2, zorder=1)
ax.text(-1.1, 0, 'Y', fontsize=10, color='pink', fontweight='bold')
ax.text(0, -1.1, 'Z', fontsize=10, color='black', fontweight='bold')

im = ax.imshow(img_front, extent=[-1,1,-1,1], zorder=2)
phi = -phi  # Invertir el ángulo de roll para que la rotación sea en la dirección correcta
trans = Affine2D().rotate_deg(phi) + ax.transData
im.set_transform(trans)

ax.set_title(f"Roll (φ) = {phi}°")
ax.set_xlim(-2,2)
ax.set_ylim(-2,2)
ax.axis("off")

plt.tight_layout()
plt.show()
import numpy as np
import pprint
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.transforms import Affine2D
import calculos as calculos

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



# Interfaz grafica 


# Funcion para plotear los tres planos de actitud (pitch, roll, heading)
phi = phi     # roll
theta = theta   # pitch
psi = psi     # yaw

#Importar el resultado de la funcion de rotacion para optener la velocidad en el NED y la matriz de rotacion del cuerpo al NED
v_body = np.array([u, v, w])
R_body_to_NED, v_NED, phi, theta, psi = calculos.rotation_matrix(phi, theta, psi, v_body)

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

# Definir el state del body con las variables de estado
state = {
    "u": u,
    "v": v,
    "w": w,
    "alpha": calculos.angle_of_attack(u,w),
    "beta": calculos.sideslip_angle(u,v,w),
    "climb": calculos.climb_angle(calculos.angle_of_attack(u,w),theta),
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
    f"beta  = {state['beta']}°\n"
    f"climb = {state['climb']}°\n\n"
    f"Euler:\n"
    f"phi = {state['phi']}°\n"
    f"theta = {state['theta']}°\n"
    f"psi = {state['psi']}°"
)

ax.text(0.05, 0.95, text, fontsize=12, va='top', family='monospace')

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
# Codigo para el stl


# ------------------ CARGAR MODELO ------------------
ruta = os.path.join(os.path.dirname(__file__), "kf21v3.stl")
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

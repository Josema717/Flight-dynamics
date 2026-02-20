import numpy as np
from stl import mesh
import pyvista as pv
import os 

ruta = os.path.join(os.path.dirname(__file__), "kf21v3.stl")
mesh = pv.read(ruta)

mesh.translate(-np.array(mesh.center), inplace=True)

longitud_ejes = mesh.length * 0.5

eje_x = pv.Arrow(start=(0,0,0), direction=(0,-1,0), scale=longitud_ejes)
eje_y = pv.Arrow(start=(0,0,0), direction=(-1,0,0), scale=longitud_ejes)
eje_z = pv.Arrow(start=(0,0,0), direction=(0,0,-1), scale=longitud_ejes)

mesh = pv.read(ruta)


plotter = pv.Plotter()

actor_mesh = plotter.add_mesh(mesh, color="gray")


actor_x = plotter.add_mesh(eje_x, color="red")
actor_y = plotter.add_mesh(eje_y, color="white")
actor_z = plotter.add_mesh(eje_z, color="blue")

plotter.add_axes()
plotter.show_grid()

plotter.show()
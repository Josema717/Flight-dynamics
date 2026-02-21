import numpy as np
from stl import mesh
import pyvista as pv
import os 
from pyvista import _vtk
import pprint
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
    

#Este coso que dio el profe, todavia no entiendo para que sirve, pero lo dejo por las dudas.
def aircraft_state(alpha, beta, climb ,u, v, w, p, q, r, phi, theta, psi, v_body):
    "Returns aircraft state values in a structured format"
    state_values = {
        "angles": {
            "alpha": alpha, #Angle of attack [deg]
            "beta": beta,   #Sideslip angle [deg]
            "gamma": climb, #Climb angle [deg]
        },
        "velocities_body": np.array([u, v, w]), #Velocities in body frame [m/s]
        "velocities_ned": np.array([u, v, w]),  #Velocities in NED frame [m/s]
        "angular_rates": np.array([p, q, r]),   #Angular rates in body frame [rad/s]
        "attitude": np.array([phi, theta, psi]), #Euler angles: roll, pitch, yaw [deg]
    }
    return state_values

v_body = np.array([u, v, w])


#Aca viene la parte logica, lo que voy a hacer es crear una funcion donde voy a simualar una IMU y meter p,q,r
# la funcion va a aplicar la matriz de transformacion para llegar a las angular rates, las cuales voy a integrar para obtener los angulos de Euler,
#  y luego voy a usar esos angulos para rotar el modelo 3D del avion, y asi mostrar la orientacion del avion en cada caso de vuelo.

def rotation_matrix(alpha, beta, climb ,u, v, w, p, q, r, phi, theta, psi, v_body):
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
    rotation_matrix = R_z @ R_y @ R_x
    v_NED = rotation_matrix @ v_body
    return rotation_matrix, v_NED, alpha, beta, climb ,u, v, w, p, q, r, phi, theta, psi, v_body


estado = aircraft_state(alpha=alpha, beta=beta, climb=climb, u=u, v=v, w=w, p=p, q=q, r=r, phi=phi, theta=theta, psi=psi, v_body=v_body)
pprint.pprint(estado)


#A;adir el path del STL
ruta = os.path.join(os.path.dirname(__file__), "kf21v3.stl")
mesh = pv.read(ruta)

mesh.translate(-np.array(mesh.center), inplace=True)

#Ejes del body
longitud_ejes = mesh.length * 0.5
eje_x_body = pv.Arrow(start=(0,0,0), direction=(0,-1,0), scale=longitud_ejes)
eje_y_body = pv.Arrow(start=(0,0,0), direction=(-1,0,0), scale=longitud_ejes)
eje_z_body = pv.Arrow(start=(0,0,0), direction=(0,0,-1), scale=longitud_ejes)

ejes = [eje_x_body, eje_y_body, eje_z_body]

mesh = pv.read(ruta)


plotter = pv.Plotter()

actor_mesh = plotter.add_mesh(mesh, color="gray")


actor_x = plotter.add_mesh(eje_x_body, color="red")
actor_y = plotter.add_mesh(eje_y_body, color="white")
actor_z = plotter.add_mesh(eje_z_body, color="blue")

#Cambiando la orientacion del NED para que siga la convencion
transform = _vtk.vtkTransform()
transform.RotateX(180)
transform.RotateZ(90)

axes_NED = _vtk.vtkAxesActor()
axes_NED.SetTotalLength(1, 1, 1)
axes_NED.origin = (0, 0, 0)
axes_NED.SetUserTransform(transform)
plotter.add_orientation_widget(axes_NED)

plotter.show_grid()
plotter.show()



class FlightInstrumentPanel:
    def __init__(self, root):
        self.root = root
        self.root.title("FLIGHT INSTRUMENT PANEL — Coordinate Frames Task 1")
        self.root.configure(bg=BG)
        self.root.geometry("1380x860")
        self.root.minsize(1100, 700)

        self.current_case_name = list(FLIGHT_CASES.keys())[0]
        self._build_ui()
        self._update_display()

    # ── UI STRUCTURE ───────────────────────────────────────────────────────

    def _build_ui(self):
        # ── HEADER ──
        hdr = tk.Frame(self.root, bg="#080c12", height=46)
        hdr.pack(fill='x', side='top')
        hdr.pack_propagate(False)

        tk.Label(hdr, text="✈  FLIGHT INSTRUMENT PANEL",
                 bg="#080c12", fg=ACCENT,
                 font=("Courier", 15, "bold")).pack(side='left', padx=18, pady=10)
        tk.Label(hdr, text="Coordinate Frames & Transformation Matrix",
                 bg="#080c12", fg=DIM,
                 font=("Courier", 9)).pack(side='left', padx=4)

        # ── CASE SELECTOR ──
        sel_frame = tk.Frame(self.root, bg=BG)
        sel_frame.pack(fill='x', padx=10, pady=(6,0))

        tk.Label(sel_frame, text="FLIGHT CASE:", bg=BG, fg=DIM,
                 font=("Courier", 9, "bold")).pack(side='left', padx=(4,8))

        self.case_var = tk.StringVar(value=self.current_case_name)
        self.case_buttons = {}
        for name, cfg in FLIGHT_CASES.items():
            btn = tk.Button(sel_frame, text=name,
                            bg=BEZEL, fg=WHITE,
                            activebackground=cfg["color"],
                            activeforeground=BG,
                            relief='flat', bd=0, padx=14, pady=4,
                            font=("Courier", 9, "bold"),
                            cursor='hand2',
                            command=lambda n=name: self._select_case(n))
            btn.pack(side='left', padx=3)
            self.case_buttons[name] = btn

        # ── MAIN CONTENT: left instruments + right terminal ──
        main = tk.Frame(self.root, bg=BG)
        main.pack(fill='both', expand=True, padx=8, pady=6)

        # LEFT: matplotlib instrument panel
        left = tk.Frame(main, bg=BG)
        left.pack(side='left', fill='both', expand=True)

        self.fig = Figure(figsize=(13.5, 8), facecolor=BG, tight_layout=True)
        self._build_figure_layout()

        self.canvas = FigureCanvasTkAgg(self.fig, master=left)
        self.canvas.get_tk_widget().pack(fill='both', expand=True)

        # RIGHT: terminal readout
        right = tk.Frame(main, bg=PANEL, width=340)
        right.pack(side='right', fill='y', padx=(6,0))
        right.pack_propagate(False)

        tk.Label(right, text="TERMINAL OUTPUT", bg=PANEL, fg=ACCENT,
                 font=("Courier", 9, "bold")).pack(pady=(8,2))

        self.terminal = tk.Text(right, bg="#050810", fg=GREEN,
                                font=("Courier", 8),
                                relief='flat', bd=0, wrap='none',
                                insertbackground=GREEN)
        self.terminal.pack(fill='both', expand=True, padx=6, pady=(2,8))

        sb = ttk.Scrollbar(right, command=self.terminal.yview)
        self.terminal['yscrollcommand'] = sb.set

    def _build_figure_layout(self):
        """Create all subplot axes using GridSpec.
        Row 0 — two round instruments: attitude indicator, compass
        Row 1 — DCM matrix panel (full width)
        """
        gs = gridspec.GridSpec(2, 2, figure=self.fig,
                               left=0.03, right=0.99,
                               top=0.97, bottom=0.06,
                               wspace=0.30, hspace=0.40)

        # Row 0: two round instruments
        self.ax_adi = self.fig.add_subplot(gs[0, 0])   # attitude indicator
        self.ax_cmp = self.fig.add_subplot(gs[0, 1])   # compass / heading

        # Row 1: DCM matrix panel (full width)
        self.ax_dcm = self.fig.add_subplot(gs[1, 0:2]) # DCM text panel

    def _select_case(self, name):
        self.current_case_name = name
        for n, btn in self.case_buttons.items():
            active = (n == name)
            col = FLIGHT_CASES[n]["color"]
            btn.configure(bg=col if active else BEZEL,
                          fg=BG if active else WHITE)
        self._update_display()
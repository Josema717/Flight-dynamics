import numpy as np
from stl import mesh
import pyvista as pv
import os 
from pyvista import _vtk
import pprint
import matplotlib.pyplot as plt
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



# ─────────────────────────────────────────────
#  COLOUR PALETTE  (dark cockpit aesthetic)
# ─────────────────────────────────────────────
BG       = "#0a0d12"
PANEL    = "#111820"
BEZEL    = "#1a2535"
ACCENT   = "#00d4ff"
GREEN    = "#39ff14"
ORANGE   = "#ff6b35"
RED      = "#ff3b3b"
YELLOW   = "#ffd700"
WHITE    = "#e8eef4"
DIM      = "#3a5060"
HORIZON_SKY   = "#1a4a7a"
HORIZON_EARTH = "#5a3a1a"

# ─────────────────────────────────────────────
#  INSTRUMENT DRAWING HELPERS
# ─────────────────────────────────────────────

def draw_bezel(ax, title="", title_color=ACCENT):
    """Draw a round instrument bezel on given axes."""
    ax.set_facecolor(PANEL)
    ax.set_xlim(-1.1, 1.1)
    ax.set_ylim(-1.1, 1.1)
    ax.set_aspect('equal')
    ax.axis('off')
    # outer ring
    outer = plt.Circle((0, 0), 1.08, color=BEZEL, zorder=0)
    inner = plt.Circle((0, 0), 1.0,  color="#050810", zorder=1)
    ax.add_patch(outer)
    ax.add_patch(inner)
    if title:
        ax.text(0, -1.18, title, ha='center', va='top',
                fontsize=7, color=title_color,
                fontfamily='monospace', fontweight='bold')

def draw_tick_ring(ax, n_major=36, n_minor=None, r=0.90, length=0.06, color=DIM):
    """Draw tick marks around a circle."""
    for i in range(n_major):
        ang = np.radians(i * 360 / n_major) - np.pi/2
        x0 = r * np.cos(ang);         y0 = r * np.sin(ang)
        x1 = (r-length)*np.cos(ang);  y1 = (r-length)*np.sin(ang)
        ax.plot([x0, x1], [y0, y1], color=color, lw=1, zorder=2)

def needle(ax, angle_deg, length=0.75, width=3, color=WHITE, zorder=5):
    """Draw instrument needle from centre."""
    ang = np.radians(angle_deg - 90)
    ax.annotate("", xy=(length*np.cos(ang), length*np.sin(ang)),
                xytext=(0, 0),
                arrowprops=dict(arrowstyle="-|>", color=color,
                                lw=width, mutation_scale=12),
                zorder=zorder)

# ─────────────────────────────────────────────
#  INDIVIDUAL INSTRUMENTS
# ─────────────────────────────────────────────

def render_adi_image(phi_deg, theta_deg, size=400):
    """
    Render the moving horizon ball as an RGBA numpy array.

    Strategy:
      1. Build a square RGBA canvas (sky above, earth below).
      2. Shift the split vertically by pitch (theta): positive pitch → horizon moves down.
      3. Rotate the entire canvas by roll (phi): positive roll → right bank.
      4. Clip to a circle mask.
      5. Draw pitch ladder lines into the pixel array.

    Returns an RGBA uint8 array of shape (size, size, 4).
    """
    # ── colours ──────────────────────────────────────────────────────────────
    SKY_TOP    = np.array([10,  60, 120, 255], dtype=np.float32)   # deep blue
    SKY_BOT    = np.array([30, 120, 200, 255], dtype=np.float32)   # lighter near horizon
    EARTH_TOP  = np.array([90,  55,  20, 255], dtype=np.float32)   # lighter near horizon
    EARTH_BOT  = np.array([50,  30,  10, 255], dtype=np.float32)   # dark brown

    S = size
    half = S // 2

    # pixel coordinate grid, normalised to [-1, 1]
    y_px, x_px = np.mgrid[0:S, 0:S]
    xn = (x_px - half) / half   # -1 … +1  (right is positive)
    yn = (half - y_px) / half   # -1 … +1  (up is positive)

    # ── pitch shift: 1 unit = 90° of pitch, clip to avoid going completely off ──
    pitch_shift = np.clip(theta_deg / 90.0, -0.92, 0.92)

    # ── roll: rotate the (xn, yn) sample coordinates by -phi so the image
    #    appears to rotate by +phi visually ──
    phi_r = np.radians(-phi_deg)
    cos_p, sin_p = np.cos(phi_r), np.sin(phi_r)
    xr =  cos_p * xn + sin_p * yn   # rotated x
    yr = -sin_p * xn + cos_p * yn   # rotated y

    # ── sky / earth split based on rotated y coordinate ──
    # A pixel belongs to SKY if yr > pitch_shift (horizon line), else EARTH
    sky_frac = np.clip((yr - pitch_shift) / 0.4, 0.0, 1.0)   # 0=earth, 1=sky

    # Gradient within each half for realism
    # Sky: blend SKY_TOP / SKY_BOT based on how far above horizon
    sky_t  = np.clip((yr - pitch_shift) / 1.2, 0.0, 1.0)[..., None]
    earth_t = np.clip((pitch_shift - yr) / 1.2, 0.0, 1.0)[..., None]

    sky_col   = SKY_TOP   * sky_t   + SKY_BOT   * (1 - sky_t)
    earth_col = EARTH_BOT * earth_t + EARTH_TOP  * (1 - earth_t)

    sky_mask = (yr >= pitch_shift)[..., None].astype(np.float32)
    img = sky_col * sky_mask + earth_col * (1 - sky_mask)

    # ── horizon line: thin white band at yr ≈ pitch_shift ──
    near_horizon = np.abs(yr - pitch_shift) < (1.5 / half)
    img[near_horizon] = [255, 255, 255, 255]

    # ── pitch ladder: horizontal lines at ±10°, ±20° ──
    for pitch_step in [-20, -10, 10, 20]:
        line_y = pitch_shift + pitch_step / 90.0
        width  = 0.25 if abs(pitch_step) == 10 else 0.35
        near   = (np.abs(yr - line_y) < (1.2 / half)) & (np.abs(xr) < width)
        img[near] = [220, 220, 220, 255]
        # small tick label would be per-pixel text which is hard; skip here

    # ── circular clip mask ──
    r2 = xn**2 + yn**2
    inside = r2 <= 1.0
    img[~inside] = [15, 20, 28, 0]   # transparent outside

    return np.clip(img, 0, 255).astype(np.uint8)


def draw_attitude_indicator(ax, phi_deg, theta_deg):
    """
    Attitude indicator rendered as a pixel image (moving horizon ball)
    with a fixed aircraft reticle and roll arc drawn on top in data coordinates.
    """
    S = 400
    img = render_adi_image(phi_deg, theta_deg, size=S)

    # ── background bezel ──
    ax.set_facecolor(PANEL)
    ax.set_xlim(-1.15, 1.15)
    ax.set_ylim(-1.15, 1.15)
    ax.set_aspect('equal')
    ax.axis('off')

    # Draw the rendered pixel image inside data coords [-1, 1]
    ax.imshow(img, extent=[-1, 1, -1, 1], origin='upper',
              zorder=2, interpolation='bilinear')

    # ── outer bezel ring (drawn on top of image) ──
    bezel_ring = plt.Circle((0, 0), 1.0, color=BEZEL,
                             fill=False, lw=8, zorder=5)
    ax.add_patch(bezel_ring)

    # ── roll arc (fixed, at r=0.90) with tick marks ──
    arc_r = 0.90
    for angle_offset in [-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60]:
        # arc tick positions are fixed; the image rotates behind them
        ang = np.radians(angle_offset - 90)   # 0° offset = top
        r0 = arc_r - (0.07 if angle_offset % 30 == 0 else 0.04)
        x0, y0 = arc_r * np.cos(ang), arc_r * np.sin(ang)
        x1, y1 = r0    * np.cos(ang), r0    * np.sin(ang)
        lw = 1.8 if angle_offset % 30 == 0 else 1.0
        ax.plot([x0, x1], [y0, y1], color=WHITE, lw=lw, zorder=6)

    # ── bank pointer triangle (moves with phi, points inward from arc) ──
    bank_ang = np.radians(-phi_deg - 90)   # top = 0°, positive roll → right
    tri_tip_r  = arc_r - 0.09
    tri_base_r = arc_r + 0.005
    tip_x  = tri_tip_r  * np.cos(bank_ang)
    tip_y  = tri_tip_r  * np.sin(bank_ang)
    left_x = tri_base_r * np.cos(bank_ang - np.radians(4))
    left_y = tri_base_r * np.sin(bank_ang - np.radians(4))
    rght_x = tri_base_r * np.cos(bank_ang + np.radians(4))
    rght_y = tri_base_r * np.sin(bank_ang + np.radians(4))
    tri = mpatches.Polygon(
        [[tip_x, tip_y], [left_x, left_y], [rght_x, rght_y]],
        closed=True, color=YELLOW, zorder=7)
    ax.add_patch(tri)

    # ── fixed aircraft reticle (always centred, never rotates) ──
    # Wings: two horizontal bars with downward stubs
    wing_inner, wing_outer, stub = 0.12, 0.42, 0.10
    for sign in [-1, 1]:
        ax.plot([sign*wing_inner, sign*wing_outer], [0, 0],
                color=ORANGE, lw=5, solid_capstyle='round', zorder=8)
        ax.plot([sign*wing_outer, sign*wing_outer], [0, -stub],
                color=ORANGE, lw=5, solid_capstyle='round', zorder=8)

    # Centre dot
    ax.plot(0, 0, 'o', color=ORANGE, ms=7, zorder=9)
    # Centre crosshair tick
    ax.plot([0, 0], [-0.04, 0.04], color=ORANGE, lw=3, zorder=9)

    # ── readout labels ──
    ax.text(-1.10, 0.92, f"φ {phi_deg:+.1f}°", fontsize=7.5,
            color=YELLOW, fontfamily='monospace', fontweight='bold', zorder=10)
    ax.text(-1.10, 0.74, f"θ {theta_deg:+.1f}°", fontsize=7.5,
            color=YELLOW, fontfamily='monospace', fontweight='bold', zorder=10)

    # ── title ──
    ax.text(0, -1.12, "ATTITUDE", ha='center', va='top',
            fontsize=7, color=ACCENT,
            fontfamily='monospace', fontweight='bold', zorder=10)


def draw_compass(ax, psi_deg, heading_color=ACCENT):
    """Heading / compass rose."""
    draw_bezel(ax, "HEADING")
    draw_tick_ring(ax, n_major=72, r=0.88, length=0.05, color=DIM)
    draw_tick_ring(ax, n_major=36, r=0.88, length=0.09, color=WHITE)

    # Cardinal labels rotating with heading
    for label, ang in [("N",0),("E",90),("S",180),("W",270)]:
        a = np.radians(ang - psi_deg) - np.pi/2
        lx = 0.72 * np.cos(a)
        ly = 0.72 * np.sin(a)
        col = RED if label == "N" else WHITE
        ax.text(lx, ly, label, ha='center', va='center',
                fontsize=9, color=col, fontweight='bold',
                fontfamily='monospace', zorder=5)

    # Fixed lubber line (top)
    ax.plot([0, 0], [0.85, 0.97], color=RED, lw=3, zorder=6)

    # Heading readout
    ax.text(0, -0.45, f"{psi_deg % 360:05.1f}°", ha='center',
            fontsize=11, color=heading_color,
            fontfamily='monospace', fontweight='bold', zorder=7)
    ax.text(0, 0.1, "HDG", ha='center', fontsize=7,
            color=DIM, fontfamily='monospace', zorder=7)

# ─────────────────────────────────────────────
#  TERMINAL / DATA READOUT
# ─────────────────────────────────────────────

def format_terminal(state, case_name, case_desc):
    phi, theta, psi = np.degrees(state["attitude"])
    vb = state["velocities_body"]
    vn = state["velocities_ned"]
    alpha = state["angles"]["alpha"]
    beta  = state["angles"]["beta"]
    gamma = state["angles"]["gamma"]
    V     = state["airspeed"]
    p, q, r = state["angular_rates"]

    lines = [
        f"{'═'*52}",
        f"  FLIGHT CASE: {case_name}",
        f"  {case_desc}",
        f"{'─'*52}",
        f"  EULER ANGLES",
        f"    φ (Roll)    = {phi:+8.3f} °",
        f"    θ (Pitch)   = {theta:+8.3f} °",
        f"    ψ (Yaw)     = {psi % 360:8.3f} °",
        f"{'─'*52}",
        f"  BODY FRAME  [u, v, w]",
        f"    u = {vb[0]:+8.3f} m/s   (forward)",
        f"    v = {vb[1]:+8.3f} m/s   (right  )",
        f"    w = {vb[2]:+8.3f} m/s   (down   )",
        f"    V = {V:8.3f} m/s   (airspeed)",
        f"{'─'*52}",
        f"  NED FRAME   [Vn, Ve, Vd]",
        f"    Vn = {vn[0]:+8.3f} m/s  (North)",
        f"    Ve = {vn[1]:+8.3f} m/s  (East )",
        f"    Vd = {vn[2]:+8.3f} m/s  (Down )",
        f"{'─'*52}",
        f"  AERODYNAMIC ANGLES",
        f"    α (AoA)     = {alpha:+8.3f} °",
        f"    β (Sideslip)= {beta:+8.3f} °",
        f"    γ (Climb)   = {gamma:+8.3f} °",
        f"{'─'*52}",
        f"  ANGULAR RATES",
        f"    p (Roll) ={p:+7.4f} rad/s",
        f"    q (Pitch)={q:+7.4f} rad/s",
        f"    r (Yaw)  ={r:+7.4f} rad/s",
        f"{'═'*52}",
    ]
    return "\n".join(lines)
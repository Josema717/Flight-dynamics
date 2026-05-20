import numpy as np
import pprint
import csv as csv
#Convencion de NED: X norte, Y este, Z abajo

#Inicializar variables
alpha, beta, climb, u, v, w, p, q, r, phi, theta, psi = 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
v_body = np.array([u, v, w]) # Velocidad en el sistema de referencia del body

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
    if abs(u) < 1e-3 and abs(w) < 1e-3:
        return 0.0
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
    if V < 1e-3:
        return 0.0
    return np.rad2deg(np.arcsin(v/V))


def climb_angle(v_NED):
    """
    Gamma (γ) — Climb Angle [deg]
    Relationship: pitch = alpha + gamma  →  gamma = pitch - alpha
    Valid when sideslip is zero (wings-level flight).
    """
    vx = v_NED[0]
    vy = v_NED[1]
    vz = -v_NED[2]

    return np.rad2deg(np.arctan2(-vz, np.sqrt(vx**2 + vy**2)))


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

estado = aircraft_state(alpha=angle_of_attack(u,w), beta=sideslip_angle(u,v,w), climb=climb_angle(v_NED), u=u, v=v, w=w, p=p, q=q, r=r, phi=phi, theta=theta, psi=psi, v_body=v_body)
# pprint.pprint(estado)  # suppressed: would spam stdout on every HUD import

def angular_rates_to_euler(p, q, r, phi, theta):
    phi_rad = np.radians(phi)
    theta_rad = np.radians(theta)

    # Matriz de transformación de angular rates a Euler rates
    H = np.array([[1, np.sin(phi_rad)*np.tan(theta_rad), np.cos(phi_rad)*np.tan(theta_rad)],
                  [0, np.cos(phi_rad), -np.sin(phi_rad)],
                  [0, np.sin(phi_rad)/np.cos(theta_rad), np.cos(phi_rad)/np.cos(theta_rad)]])
    
    angular_rates = np.array([p, q, r])
    euler_rates = H @ angular_rates
    return euler_rates


try:
    with open('tello_imu_example.csv', 'r', newline='', encoding='utf-8') as imu_raw:
        imu = list(csv.DictReader(imu_raw))
except FileNotFoundError:
    imu = []   # CSV not present — IMU integration unavailable; RCAM scenarios still work
u, v, w = 0, 0, 0
x, y, z = 0, 0, 0
phi, theta, psi = 0, 0, 0


def integrate_imu_data(imu):
    u,v,w = 0, 0, 0 # Inicializar velocidades en el body
    x,y,z = 0, 0, 0 # Inicializar posiciones en el NED
    vx,vy,vz = 0,0,0 # Inicializar velocidades en el NED
    phi, theta, psi = 0, 0, 0 # Inicializar ángulos de Euler localmente
    #listas para guardar los datos para graficar
    time_list, u_list, v_list, w_list, x_list, y_list, z_list, phi_list, theta_list, psi_list = [], [], [], [], [], [], [], [], [], []
    vNED_list = []
    P_ned_list = []
    p_list, q_list, r_list = [], [], []
    u_body_list, v_body_list, w_body_list = [], [], []
    # Body-frame velocity accumulators (gravity removed in body frame, like check.py)
    u_b, v_b, w_b = 0.0, 0.0, 0.0
    for i in range(1, len(imu)):
        row = imu[i]
        row_prev = imu[i-1]
        dt = float(row["time_s"]) - float(row_prev["time_s"]) # Paso en segundos entre dos filas consecutivas

    # Crear el vector omega de velocidad angular para el body
        p = float(row["gyro_p_rad_s"])
        q = float(row["gyro_q_rad_s"])
        r = float(row["gyro_r_rad_s"])
        omega_body = np.array([p, q, r]) # Velocidad angular en el sistema de referencia del body
        
        #Integral para los ángulos de Euler
        euler_rates = angular_rates_to_euler(p, q, r, phi, theta)
        phi += np.rad2deg(euler_rates[0] * dt)
        theta += np.rad2deg(euler_rates[1] * dt)
        psi += np.rad2deg(euler_rates[2] * dt)
        R_body_to_NED, _, _, _, _ = rotation_matrix(phi, theta, psi, v_body)

        # Crear el vector de aceleracion en el body
        u_dot = float(row["accel_x_m_s2"])
        v_dot = float(row["accel_y_m_s2"])
        w_dot = float(row["accel_z_m_s2"]) + 9.81  # Remove gravity in body frame (accel_z ≈ -9.81 level → w_dot ≈ 0)
        a_body = np.array([u_dot, v_dot, w_dot])

        # Body-frame velocity integration (clean, no gravity drift)
        u_b += u_dot * dt
        v_b += v_dot * dt
        w_b += w_dot * dt

        a_ned = R_body_to_NED @ a_body # Transformar la aceleración del body al NED
        # a_body already has gravity removed in body frame; rotating to NED gives
        # the true NED acceleration directly (no further gravity subtraction needed).
        
        # Integrar aceleraciones para obtener velocidades en NED
        vx += a_ned[0] * dt
        vy += a_ned[1] * dt
        vz += a_ned[2] * dt
        v_NED = np.array([vx, vy, vz]) # Velocidad actual en el NED
        #Integral para la posicion en el NED
        # Integrar velocidades para obtener posición (acumular)
        x += v_NED[0] * dt
        y += v_NED[1] * dt
        z += v_NED[2] * dt * -1 # El eje Z del NED apunta hacia abajo
        P_ned = np.array([x, y, z])

    
        #Guardar los datos para graficar
        time_list.append(float(imu[i]["time_s"]))
        u_list.append(vx); v_list.append(vy); w_list.append(vz)
        x_list.append(P_ned[0]); y_list.append(P_ned[1]); z_list.append(P_ned[2])
        phi_list.append(phi); theta_list.append(theta); psi_list.append(psi)
        vNED_list.append(v_NED.copy())
        P_ned_list.append(P_ned.copy())
        p_list.append(p); q_list.append(q); r_list.append(r)
        # Body-frame velocities from direct body integration (w ≈ 0 during level flight)
        u_body_list.append(u_b)
        v_body_list.append(v_b)
        w_body_list.append(w_b)

    return (time_list, vNED_list, P_ned_list,
            phi_list, theta_list, psi_list,
            p_list, q_list, r_list,
            u_body_list, v_body_list, w_body_list)

def angle_2_quaternion(R_ned_to_body):
    qs = np.sqrt(0.25 * (R_ned_to_body[0,0] + R_ned_to_body[1,1] + R_ned_to_body[2,2] + 1))
    qx = np.sqrt(0.25 * (R_ned_to_body[0,0] - R_ned_to_body[1,1] - R_ned_to_body[2,2] + 1))
    qy = np.sqrt(0.25 * (-R_ned_to_body[0,0] + R_ned_to_body[1,1] - R_ned_to_body[2,2] + 1))
    qz = np.sqrt(0.25 * (-R_ned_to_body[0,0] - R_ned_to_body[1,1] + R_ned_to_body[2,2] + 1))
    q = np.array([qs, qx, qy, qz])
    return q
    
def quaternion_2_angle(q):
    theta = 2 * np.arccos(q[0])
    e = []
    if abs(theta) < 1e-3:
        return 0.0, np.array([0, 0, 0])
    e[0] = q[1] / np.sin(theta/2)
    e[1] = q[2] / np.sin(theta/2)
    e[2] = q[3] / np.sin(theta/2)
    return np.rad2deg(theta), e


def xdot(X, U):
    # X = [u, v, w, p, q, r, phi, theta, psi]  (9 elements)
    # U = [delta_ailerons, delta_elevator, delta_rudder, delta_throttle_1, delta_throttle_2]
    # Extract state variables with clear names to avoid index confusion
    u_s, v_s, w_s   = X[0], X[1], X[2]   # linear body-frame velocities [m/s]
    p_s, q_s, r_s   = X[3], X[4], X[5]   # angular rates [rad/s]
    phi_s, theta_s, psi_s = X[6], X[7], X[8]  # Euler angles [rad]

##########################################################################

# Step 1 control limits and constants

###########################################################################
    u1, u2, u3, u4, u5 = U[0], U[1], U[2], U[3], U[4]  # control inputs
    u1 = max(np.radians(-25), min(np.radians(25), u1))  # Estableciendo los limites de los controles en radianes
    u2 = max(np.radians(-25), min(np.radians(10), u2))  # Limitar el ángulo de pitch (theta)
    u3 = max(np.radians(-30), min(np.radians(30), u3))  # Limitar el ángulo de yaw (psi)

    g = 9.81 # Gravedad en m/s²
    St = 64 # Área del estabilizador horizontal m²
    S = 260 # Área del ala m²
    c_mac = 6.6 # Cuerda media aerodinámica  m
    Ixx, Ixy, Ixz = 40.07, 0, 2.098 # Componentes de la matriz de inercia  
    Iyx, Iyy, Iyz = 0, 64, 0 # Componentes de la matriz de inercia
    Izx, Izy, Izz = 2.098, 0, 99.92 # Componentes de la matriz de inercia
    m = 120000 # Masa del avión (kg)

    X_apt_1 = 0 # Posición X del primer motor en el eje del body [m]
    Y_apt_1 = 7.94 # Posición Y del primer motor en el eje del body [m]
    Z_apt_1 = 1.9 # Posición Z del primer motor en el eje del body [m]

    X_apt_2 = 0 # Posición X del segundo motor en el eje del body [m]
    Y_apt_2 = -7.94 # Posición Y del segundo motor en el eje del body [m]
    Z_apt_2 = 1.9 # Posición Z del segundo motor en el eje del body [m]

##############################################################################

# Step 2: Intermediate variables

###############################################################################
    Va = np.sqrt(u_s**2 + v_s**2 + w_s**2)
    alpha = np.arctan2(w_s, u_s) if Va > 1e-3 else 0.0
    beta = np.arcsin(v_s / Va) if Va > 1e-3 else 0.0
    Q = 0.5 * 1.225 * Va**2 # Presión dinámica (ρ * V² / 2)
    w_be = np.array([p_s, q_s, r_s])  # Velocidad angular en el body [p, q, r]
    V_body = np.array([u_s, v_s, w_s])  # Velocidad en el sistema de referencia del body

###############################################################################

#Step 3: Nontedimensional aerodynamic coefficients in stability frame

###############################################################################

    # Calculo del Cl del ala y el cuerpo
    n = 5.5
    alpha_lift_0 = -11.5 * np.pi/180
    a0 = 15.212
    a1 = -155.2
    a2 = 609.2
    a3 = -768.5

    if alpha < 14.5*np.pi/180:
        Cl_wb = n*(alpha-alpha_lift_0)
    else:
        Cl_wb = a0 + a1*alpha + a2*alpha**2 + a3*alpha**3

    depsilon_dalpha = 0.25 # Esto salio del documento del RCAM
    
    epsilon = depsilon_dalpha * (alpha - alpha_lift_0)
    lt = 24.8
    alpha_t = alpha - epsilon + u2 + (1.3*q_s*lt/Va if Va > 1e-3 else 0.0)
    Cl_t = 3.1*St/S * alpha_t   # St es el área del estabilizador horizontal, S es el área del ala

    Cl = Cl_wb + Cl_t

    #Total drag coefficient
    # Total drag coefficient (spec eq. 2.31): CD = 0.13 + 0.07*(CLwb - 0.45)^2
    Cd = 0.13 + 0.07*(Cl_wb - 0.45)**2

    # Total side-force coefficient (spec eq. 2.32) — NOTE: -1.6, not -16
    Cy = -1.6*beta + 0.24*u3

    # ── Dimensional aerodynamic forces ────────────────────────────────────────
    D = Cd * Q * S   # drag  (positive scalar)
    Y_sf = Cy * Q * S   # side force
    L = Cl * Q * S   # lift  (positive scalar)

##################################################################################

#Step 4: Aerodynamic forces and moments in body frame

##################################################################################

    # Transform wind-axis forces to body axes (spec section 2.3.4 explicit formulas)
    #   FxA =  L sinα - D cosα cosβ - Y cosα sinβ
    #   FyA = -D sinβ  + Y cosβ
    #   FzA = -L cosα - D sinα cosβ - Y sinα sinβ
    ca, sa = np.cos(alpha), np.sin(alpha)
    cb, sb = np.cos(beta),  np.sin(beta)
    FxA =  L*sa - D*ca*cb - Y_sf*ca*sb
    FyA = -D*sb  + Y_sf*cb
    FzA = -L*ca - D*sa*cb - Y_sf*sa*sb
    Fab = np.array([FxA, FyA, FzA])   # aerodynamic force in body frame

    # ── Aerodynamic moment coefficients about CoG in body frame (spec eq. 2.33) ──
    # Cl, Cm, Cn are given directly about the CoG — no AC→CG transfer needed.
    b    = 44.8   # m — wingspan (spec Table 2.4)
    l_Va = (c_mac / Va) if Va > 1e-3 else 0.0   # generalised length / airspeed

###################################################################################

#Step 5: Aerodynamic moments coefficients about aerodynamic center in body frame

###################################################################################

    # Static (alpha/beta) terms
    n_dash = np.array([
        -1.4*beta,
        -0.59 - 3.1*(St*lt)/(S*c_mac)*(alpha - epsilon),
        (1.0 - alpha*(180.0/(15.0*np.pi)))*beta
    ])

    # Rate-damping matrix (l/V factor; spec eq. 2.33)
    dcm_dx = l_Va * np.array([
        [-11.0,  0.0,                           5.0  ],
        [  0.0, -4.03*(St*lt**2)/(S*c_mac**2),  0.0  ],
        [  1.7,  0.0,                          -11.5  ]   # -11.5, NOT -11.5*beta
    ])

    # Control-effectiveness matrix (spec eq. 2.33)
    dcm_du = np.array([
        [-0.6,  0.0,                    0.22],
        [ 0.0, -3.1*(St*lt)/(S*c_mac), 0.0 ],
        [ 0.0,  0.0,                   -0.63]
    ])

    # Full nondimensional moment-coefficient vector [Cl_total, Cm_total, Cn_total]
    Cm_b = n_dash + dcm_dx @ np.array([p_s, q_s, r_s]) + dcm_du @ np.array([u1, u2, u3])

##########################################################################

# Step 6: Dimensional forces and moments about CoG in body frame

##########################################################################

    # Dimensional moments about CoG (spec section 2.3.4)
    #   Rolling  moment ℒ = Cl * qbar * S * b       (b = wingspan)
    #   Pitching moment ℳ = Cm * qbar * S * c_mac
    #   Yawing   moment ℕ = Cn * qbar * S * b
    Ma_cg_b = Q * S * np.array([
        Cm_b[0] * b,        # ℒ — rolling  moment about CoG
        Cm_b[1] * c_mac,    # ℳ — pitching moment about CoG
        Cm_b[2] * b         # ℕ — yawing   moment about CoG
    ])


###################################################################

# Step 7: Propulsion effects

###################################################################


    # ── Engine thrust forces and moments (spec eqs. 2.34–2.37) ──────────────────
    F1 = u4 * m * g   # thrust of engine 1  [N]
    F2 = u5 * m * g   # thrust of engine 2  [N]
    F_prop_total = np.array([F1 + F2, 0.0, 0.0])   # both act along x_body

    # Moment arms are given in body axes w.r.t. CoG (spec Table 2.4 / eq. 2.37)
    r_apt_1 = np.array([X_apt_1, Y_apt_1, Z_apt_1])   # already relative to CoG
    r_apt_2 = np.array([X_apt_2, Y_apt_2, Z_apt_2])
    M_total_engine_cg_b = (np.cross(r_apt_1, np.array([F1, 0.0, 0.0]))
                         + np.cross(r_apt_2, np.array([F2, 0.0, 0.0])))

#################################################################

# Step 8: gravity effects and equations of motion

#################################################################

    # Gravity effects
    F_gravity_ned = np.array([0.0, 0.0, m*g])
    R_body_to_NED, _, _, _, _ = rotation_matrix(
        phi=np.degrees(phi_s), theta=np.degrees(theta_s),
        psi=np.degrees(psi_s), v_body=np.array([u_s, v_s, w_s]))
    F_gravity_body = R_body_to_NED.T @ F_gravity_ned

    # ── Equations of motion ─────────────────────────────────────────────────────
    F_total_body = Fab + F_prop_total + F_gravity_body
    lineal_acceleration = (1.0/m) * F_total_body - np.cross(w_be, V_body)
    u_dot, v_dot, w_dot = lineal_acceleration

    M_cg = Ma_cg_b.flatten() + M_total_engine_cg_b.flatten()   # total moment about CoG [N·m]

    # Inertia tensor (spec eq. 2.11): I = m * [[Ix, 0, -Ixz], [0, Iy, 0], [-Ixz, 0, Iz]]
    # Values in the spec are in m²; multiplying by mass m gives kg·m².
    # Sign convention: off-diagonal terms are NEGATIVE (-Ixz).
    Ib = m * np.array([
        [ Ixx,  0.0, -Ixz],
        [ 0.0,  Iyy,  0.0],
        [-Ixz,  0.0,  Izz]
    ])

    rotational_acceleration = np.linalg.inv(Ib) @ (M_cg - np.cross(w_be, Ib @ w_be))
    p_dot, q_dot, r_dot = rotational_acceleration[0], rotational_acceleration[1], rotational_acceleration[2]

    # Euler angle rates — angular_rates_to_euler(p, q, r, phi_deg, theta_deg)
    # euler_rates returns values in rad/s because p, q, r are in rad/s
    euler_rates = angular_rates_to_euler(p_s, q_s, r_s, np.degrees(phi_s), np.degrees(theta_s))
    phi_dot, theta_dot, psi_dot = euler_rates[0], euler_rates[1], euler_rates[2]

    x_dot = np.array([u_dot, v_dot, w_dot, p_dot, q_dot, r_dot, phi_dot, theta_dot, psi_dot])
    return x_dot

def pso_trim():
    print("Corriendo PSO para encontrar Trimado...")
    def cost(vars):
        alpha, d_T, d_th = vars
        u = 78.0 * np.cos(alpha)
        w = 78.0 * np.sin(alpha)
        X = np.array([u, 0.0, w, 0.0, 0.0, 0.0, 0.0, alpha, np.pi/4])
        U = np.array([0.0, d_T, 0.0, d_th, d_th])
        X_dot = xdot(X, U)
        return X_dot[0]**2 + X_dot[2]**2 + X_dot[4]**2

    n_particles = 30
    n_iters = 100
    dim = 3
    bounds = np.array([[-0.2, 0.2], [-0.5, 0.5], [0.0, 1.0]])  #Se establece el limite de angulo de ataque, delta elevar y delta throttle
    # El limite de angulo de ataque, si se pasa a grados esta entre -11.5 y 11.5 grados.
    X_pos = np.random.uniform(bounds[:,0], bounds[:,1], (n_particles, dim))
    V = np.zeros((n_particles, dim))
    
    pbest_pos = X_pos.copy()
    pbest_cost = np.array([cost(p) for p in X_pos])
    gbest_idx = np.argmin(pbest_cost)
    gbest_pos = pbest_pos[gbest_idx].copy()
    gbest_cost = pbest_cost[gbest_idx]
    
    w_weight, c1, c2 = 0.5, 1.5, 1.5
    for i in range(n_iters):
        r1, r2 = np.random.rand(n_particles, dim), np.random.rand(n_particles, dim)
        V = w_weight * V + c1 * r1 * (pbest_pos - X_pos) + c2 * r2 * (gbest_pos - X_pos)
        X_pos += V
        X_pos = np.clip(X_pos, bounds[:,0], bounds[:,1])
        costs = np.array([cost(p) for p in X_pos])
        better_idx = costs < pbest_cost
        pbest_cost[better_idx] = costs[better_idx]
        pbest_pos[better_idx] = X_pos[better_idx]
        if np.min(pbest_cost) < gbest_cost:
            gbest_cost = np.min(pbest_cost)
            gbest_pos = pbest_pos[np.argmin(pbest_cost)].copy()

    alpha, d_T, d_th = gbest_pos
    u = 78.0 * np.cos(alpha)
    w = 78.0 * np.sin(alpha)
    X_trim = np.array([u, 0.0, w, 0.0, 0.0, 0.0, 0.0, alpha, np.pi/4])
    U_trim = np.array([0.0, d_T, 0.0, d_th, d_th])
    return X_trim, U_trim


def simulate(t_end, dt, X0, U_func):
    N = int(t_end / dt) + 1
    t = np.linspace(0, t_end, N)
    X = np.zeros((N, 9))
    X[0] = X0
    for i in range(1, N):
        U = U_func(t[i-1])
        X_dot = xdot(X[i-1], U)
        X[i] = X[i-1] + X_dot * dt
    return t, X

def calc_L_D(X, U):
    """Compute dimensional Lift and Drag forces consistent with the xdot aerodynamics."""
    u_s, v_s, w_s = X[0], X[1], X[2]
    q_s = X[4]
    u2 = max(np.radians(-25), min(np.radians(10), U[1]))
    Va = np.sqrt(u_s**2 + v_s**2 + w_s**2)
    alpha = np.arctan2(w_s, u_s) if Va > 1e-3 else 0.0
    beta  = np.arcsin(np.clip(v_s / Va, -1, 1)) if Va > 1e-3 else 0.0
    Q  = 0.5 * 1.225 * Va**2
    S  = 260.0
    St = 64.0
    lt = 24.8
    c_mac = 6.6
    n = 5.5
    alpha_lift_0 = -11.5 * np.pi / 180
    # Lift coefficient (spec eq. 2.24–2.28)
    if alpha < 14.5 * np.pi / 180:
        Cl_wb = n * (alpha - alpha_lift_0)
    else:
        Cl_wb = 15.212 - 155.2*alpha + 609.2*alpha**2 - 768.5*alpha**3
    epsilon  = 0.25 * (alpha - alpha_lift_0)
    alpha_t  = alpha - epsilon + u2 + (1.3*q_s*lt/Va if Va > 1e-3 else 0.0)
    Cl       = Cl_wb + 3.1*(St/S)*alpha_t
    # Drag coefficient (spec eq. 2.31)
    Cd = 0.13 + 0.07*(Cl_wb - 0.45)**2
    # Dimensional forces
    L_force =  Cl * Q * S   # lift  (positive up in wind frame)
    D_force =  Cd * Q * S   # drag  (positive, opposing motion)
    return L_force, D_force

def run_rcam_scenario(scenario_id):
    """
    Returns the exact tuple that HUD.py expects:
    (time_l, vNED_l, Pned_l, phi_l, theta_l, psi_l, p_l, q_l, r_l, u_l, v_l, w_l)
    """
    X0_nom = np.array([85.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0])
    U_nom = np.array([0.0, -0.1, 0.0, 0.08, 0.08])
    dt = 0.05
    t_end = 180.0
    
    if scenario_id == 1: # Nominal
        def U_func(time): return U_nom
        t, X = simulate(t_end, dt, X0_nom, U_func)
    elif scenario_id == 2: # Aileron Doublet
        def U_func(time):
            U = U_nom.copy()
            if 30 <= time <= 32:
                U[0] = np.radians(5)
            return U
        t, X = simulate(t_end, dt, X0_nom, U_func)
    elif scenario_id == 3: # Engine 1 Shutdown
        def U_func(time):
            U = U_nom.copy()
            if time >= 80:
                U[3] = 0.0
            return U
        t, X = simulate(t_end, dt, X0_nom, U_func)
    elif scenario_id == 4: # PSO Trim
        X_trim, U_trim = pso_trim()
        def U_func(time): return U_trim
        t, X = simulate(60.0, dt, X_trim, U_func)
    else:
        raise ValueError("Invalid scenario")

    # Format arrays for HUD.py
    N = len(t)
    vNED_list = []
    Pned_list = []
    L_list = []
    D_list = []
    
    # Init position
    pos_ned = np.array([0.0, 0.0, 0.0])
    
    # Downsample factor to maintain performance in HUD (approx 5 frames per second)
    step = max(1, int(0.2 / dt))
    
    for i in range(N):
        u, v, w = X[i, 0:3]
        phi, theta, psi = np.degrees(X[i, 6:9])
        v_body = np.array([u, v, w])
        
        R_body_to_NED, v_ned, _, _, _ = rotation_matrix(phi, theta, psi, v_body)
        
        # Integrate position (P_NED)
        if i > 0:
            pos_ned = pos_ned + v_ned * dt
            
        if i % step == 0:
            vNED_list.append(v_ned)
            
            # NOTE: calculos.py previously returned z as negative for altitude.
            Pned_list.append(np.array([pos_ned[0], pos_ned[1], -pos_ned[2]]))
            
            # Calculate Aero Forces
            L_force, D_force = calc_L_D(X[i], U_func(t[i]))
            L_list.append(L_force)
            D_list.append(D_force)
        
    return (
        list(t[::step]),
        vNED_list,
        Pned_list,
        list(np.degrees(X[::step, 6])), # phi
        list(np.degrees(X[::step, 7])), # theta
        list(np.degrees(X[::step, 8])), # psi
        list(X[::step, 3]), # p
        list(X[::step, 4]), # q
        list(X[::step, 5]), # r
        list(X[::step, 0]), # u
        list(X[::step, 1]), # v
        list(X[::step, 2]), # w
        L_list,        # L
        D_list         # D
    )


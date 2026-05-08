import numpy as np

# ==========================================
# 1. CONSTANTS FOR RCAM
# ==========================================
m = 120000.0  # kg
c_mac = 6.6   # m
lt = 24.8     # m
S = 260.0     # m^2
St = 64.0     # m^2
g = 9.81      # m/s^2

# Center of Gravity and Aerodynamic Center
X_cg = 0.23 * c_mac
Y_cg = 0.0
Z_cg = 0.1 * c_mac
r_cg = np.array([X_cg, Y_cg, Z_cg])

X_ac = 0.12 * c_mac
Y_ac = 0.0
Z_ac = 0.0
r_ac = np.array([X_ac, Y_ac, Z_ac])

# Engine positions
X_apt_1 = 0.0
Y_apt_1 = -7.94
Z_apt_1 = 1.9
r_apt_1 = np.array([X_apt_1, Y_apt_1, Z_apt_1])

X_apt_2 = 0.0
Y_apt_2 = 7.94
Z_apt_2 = 1.9
r_apt_2 = np.array([X_apt_2, Y_apt_2, Z_apt_2])

# Inertia
Ixx = 40.07e6
Iyy = 64.0e6
Izz = 99.12e6
Ixz = 2.79e6
Ixy = 0.0
Iyz = 0.0
Ib = np.array([
    [Ixx, -Ixy, -Ixz],
    [-Ixy, Iyy, -Iyz],
    [-Ixz, -Iyz, Izz]
])
invIb = np.linalg.inv(Ib)

# ==========================================
# 2. UTILITY FUNCTIONS (COMPATIBLE WITH HUD.PY)
# ==========================================
def rotation_matrix(phi, theta, psi, v_body):
    """
    HUD.py calls: R_body_to_NED, _, _, _, _ = rotation_matrix(phi, theta, psi, v_body)
    Input phi, theta, psi are in DEGREES for compatibility with original code.
    """
    phi_rad = np.radians(phi)
    theta_rad = np.radians(theta)
    psi_rad = np.radians(psi)

    R_z = np.array([[np.cos(psi_rad), np.sin(psi_rad), 0],
                    [-np.sin(psi_rad), np.cos(psi_rad), 0],
                    [0, 0, 1]])
    R_y = np.array([[np.cos(theta_rad), 0, -np.sin(theta_rad)],
                    [0, 1, 0],
                    [np.sin(theta_rad), 0, np.cos(theta_rad)]])
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(phi_rad), np.sin(phi_rad)],
                    [0, -np.sin(phi_rad), np.cos(phi_rad)]])
    
    R_NED_to_body = R_x @ R_y @ R_z
    R_body_to_NED = R_NED_to_body.T
    v_NED = R_body_to_NED @ v_body
    return R_body_to_NED, v_NED, phi, theta, psi

def angle_of_attack(u, w):
    if abs(u) < 1e-3 and abs(w) < 1e-3:
        return 0.0
    alpha = np.rad2deg(np.arctan2(w, u))
    return alpha

def sideslip_angle(u, v, w):
    V = np.sqrt(u**2 + v**2 + w**2)
    if V < 1e-3:
        return 0.0
    return np.rad2deg(np.arcsin(v/V))

def climb_angle(v_NED):
    vx, vy, vz = v_NED[0], v_NED[1], v_NED[2]
    return np.rad2deg(np.arctan2(-vz, np.sqrt(vx**2 + vy**2)))

def angular_rates_to_euler(p, q, r, phi, theta):
    phi_rad = np.radians(phi)
    theta_rad = np.radians(theta)
    H = np.array([[1, np.sin(phi_rad)*np.tan(theta_rad), np.cos(phi_rad)*np.tan(theta_rad)],
                  [0, np.cos(phi_rad), -np.sin(phi_rad)],
                  [0, np.sin(phi_rad)/np.cos(theta_rad), np.cos(phi_rad)/np.cos(theta_rad)]])
    angular_rates = np.array([p, q, r])
    return H @ angular_rates

# ==========================================
# 3. IMU INTEGRATION (FOR CSV FILE)
# ==========================================
def integrate_imu_data(imu):
    u,v,w = 0, 0, 0
    x,y,z = 0, 0, 0
    vx,vy,vz = 0,0,0
    phi, theta, psi = 0, 0, 0
    
    time_list, u_list, v_list, w_list, x_list, y_list, z_list, phi_list, theta_list, psi_list = [], [], [], [], [], [], [], [], [], []
    vNED_list = []
    P_ned_list = []
    p_list, q_list, r_list = [], [], []
    u_body_list, v_body_list, w_body_list = [], [], []
    
    u_b, v_b, w_b = 0.0, 0.0, 0.0
    
    for i in range(1, len(imu)):
        row = imu[i]
        row_prev = imu[i-1]
        dt = float(row["time_s"]) - float(row_prev["time_s"])

        p = float(row["gyro_p_rad_s"])
        q = float(row["gyro_q_rad_s"])
        r = float(row["gyro_r_rad_s"])
        
        euler_rates = angular_rates_to_euler(p, q, r, phi, theta)
        phi += np.rad2deg(euler_rates[0] * dt)
        theta += np.rad2deg(euler_rates[1] * dt)
        psi += np.rad2deg(euler_rates[2] * dt)
        R_body_to_NED, _, _, _, _ = rotation_matrix(phi, theta, psi, np.array([0,0,0]))

        u_dot = float(row["accel_x_m_s2"])
        v_dot = float(row["accel_y_m_s2"])
        w_dot = float(row["accel_z_m_s2"]) + 9.81  # Remove gravity
        
        a_body = np.array([u_dot, v_dot, w_dot])
        u_b += u_dot * dt
        v_b += v_dot * dt
        w_b += w_dot * dt

        a_ned = R_body_to_NED @ a_body
        vx += a_ned[0] * dt
        vy += a_ned[1] * dt
        vz += a_ned[2] * dt
        v_NED = np.array([vx, vy, vz])
        
        x += v_NED[0] * dt
        y += v_NED[1] * dt
        z += v_NED[2] * dt * -1 # El eje Z del NED apunta hacia abajo
        P_ned = np.array([x, y, z])

        time_list.append(float(imu[i]["time_s"]))
        u_list.append(vx); v_list.append(vy); w_list.append(vz)
        x_list.append(P_ned[0]); y_list.append(P_ned[1]); z_list.append(P_ned[2])
        phi_list.append(phi); theta_list.append(theta); psi_list.append(psi)
        vNED_list.append(v_NED.copy())
        P_ned_list.append(P_ned.copy())
        p_list.append(p); q_list.append(q); r_list.append(r)
        
        u_body_list.append(u_b)
        v_body_list.append(v_b)
        w_body_list.append(w_b)

    return (time_list, vNED_list, P_ned_list,
            phi_list, theta_list, psi_list,
            p_list, q_list, r_list,
            u_body_list, v_body_list, w_body_list)

# ==========================================
# 4. RCAM MODEL
# ==========================================
def xdot(X, U):
    u, v, w, p, q, r, phi, theta, psi = X
    d_A, d_T, d_R, d_th1, d_th2 = U

    V_body = np.array([u, v, w])
    w_be = np.array([p, q, r])
    
    Va = np.linalg.norm(V_body)
    if Va < 1e-3: Va = 1e-3

    alpha = np.arctan2(w, u)
    beta = np.arcsin(v / Va)
    Q = 0.5 * 1.225 * Va**2

    alpha_lift_0 = -11.5 * np.pi/180
    n = 5.5
    if alpha < 14.5 * np.pi/180:
        Cl_wb = n * (alpha - alpha_lift_0)
    else:
        a0 = 15.212
        a1 = -155.2
        a2 = 609.2
        a3 = -768.5
        Cl_wb = a0 + a1*alpha + a2*alpha**2 + a3*alpha**3

    depsilon_dalpha = 0.25
    epsilon = depsilon_dalpha * (alpha - alpha_lift_0)
    alpha_t = alpha - epsilon + d_T + 1.3 * q * lt / Va
    Cl_t = 3.1 * (St/S) * alpha_t
    Cl = Cl_wb + Cl_t

    Cd = 0.13 + 0.07 * (n * alpha + 0.654)**2
    Cy = -1.6 * beta + 0.24 * d_R

    Fas = Q * S * np.array([-Cd, Cy, -Cl])
    R_s_to_body = np.array([
        [np.cos(alpha), 0, -np.sin(alpha)],
        [0, 1, 0],
        [np.sin(alpha), 0, np.cos(alpha)]
    ])
    Fab = R_s_to_body @ Fas

    eta11 = -1.4 * beta
    eta12 = -0.59 - 3.1 * (St * lt) / (S * c_mac) * (alpha - epsilon)
    eta13 = (1 - alpha * 180 / (15 * np.pi)) * beta
    eta = np.array([eta11, eta12, eta13])
    
    dcm_dx = (c_mac / Va) * np.array([
        [-11.0, 0.0, 5.0],
        [0.0, -4.03 * (St * lt**2) / (S * c_mac**2), 0.0],
        [1.7, 0.0, -11.5]
    ])
    dcm_du = np.array([
        [-0.6, 0.0, 0.22],
        [0.0, -3.1 * (St * lt) / (S * c_mac), 0.0],
        [0.0, 0.0, -0.63]
    ])
    
    Cm_ac_b = eta + dcm_dx @ w_be + dcm_du @ np.array([d_A, d_T, d_R])
    M_a_ac_b = c_mac * Q * S * Cm_ac_b
    Ma_cg_b = M_a_ac_b + np.cross(Fab, r_cg - r_ac)

    F1 = d_th1 * m * g
    F2 = d_th2 * m * g
    F_prop_1 = np.array([F1, 0.0, 0.0])
    F_prop_2 = np.array([F2, 0.0, 0.0])
    F_prop_total = F_prop_1 + F_prop_2
    
    r_eng1 = r_apt_1 - r_cg
    r_eng2 = r_apt_2 - r_cg
    M_engine_cg_1_body = np.cross(r_eng1, F_prop_1)
    M_engine_cg_2_body = np.cross(r_eng2, F_prop_2)
    M_total_engine_cg_b = M_engine_cg_1_body + M_engine_cg_2_body

    F_gravity_ned = np.array([0, 0, m * g])
    R_body_to_NED, _, _, _, _ = rotation_matrix(np.degrees(phi), np.degrees(theta), np.degrees(psi), V_body)
    F_gravity_body = R_body_to_NED.T @ F_gravity_ned

    F_total_body = Fab + F_prop_total + F_gravity_body
    lineal_acceleration = (1.0 / m) * F_total_body - np.cross(w_be, V_body)

    M_cg = Ma_cg_b + M_total_engine_cg_b
    rotational_acceleration = invIb @ (M_cg - np.cross(w_be, Ib @ w_be))

    sin_phi, cos_phi = np.sin(phi), np.cos(phi)
    tan_theta, cos_theta = np.tan(theta), np.cos(theta)
    if np.abs(cos_theta) < 1e-4: cos_theta = 1e-4 * np.sign(cos_theta)
        
    H = np.array([
        [1.0, sin_phi * tan_theta, cos_phi * tan_theta],
        [0.0, cos_phi, -sin_phi],
        [0.0, sin_phi / cos_theta, cos_phi / cos_theta]
    ])
    euler_rates = H @ w_be

    X_dot = np.zeros(9)
    X_dot[0:3] = lineal_acceleration
    X_dot[3:6] = rotational_acceleration
    X_dot[6:9] = euler_rates
    return X_dot

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
    bounds = np.array([[-0.2, 0.2], [-0.5, 0.5], [0.0, 1.0]])
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

# ==========================================
# 5. INTEGRATION ADAPTER FOR HUD.PY
# ==========================================
def run_rcam_scenario(scenario_id):
    """
    Returns the exact tuple that HUD.py expects:
    (time_l, vNED_l, Pned_l, phi_l, theta_l, psi_l, p_l, q_l, r_l, u_l, v_l, w_l)
    """
    X0_nom = np.array([85.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0])
    U_nom = np.array([0.0, -0.1, 0.0, 0.08, 0.08])
    dt = 1.0
    t_end = 180.0
    
    if scenario_id == 1: # Nominal
        t, X = simulate(t_end, dt, X0_nom, lambda t: U_nom)
    elif scenario_id == 2: # Aileron Doublet
        def U_aileron(time):
            U = U_nom.copy()
            if 30 <= time <= 32:
                U[0] = np.radians(5)
            return U
        t, X = simulate(t_end, dt, X0_nom, U_aileron)
    elif scenario_id == 3: # Engine 1 Shutdown
        def U_engine(time):
            U = U_nom.copy()
            if time >= 30:
                U[3] = 0.0
            return U
        t, X = simulate(t_end, dt, X0_nom, U_engine)
    elif scenario_id == 4: # PSO Trim
        X_trim, U_trim = pso_trim()
        t, X = simulate(60.0, dt, X_trim, lambda t: U_trim)
    else:
        raise ValueError("Invalid scenario")

    # Format arrays for HUD.py
    N = len(t)
    vNED_list = []
    Pned_list = []
    
    # Init position
    pos_ned = np.array([0.0, 0.0, 0.0])
    
    for i in range(N):
        u, v, w = X[i, 0:3]
        phi, theta, psi = np.degrees(X[i, 6:9])
        v_body = np.array([u, v, w])
        
        R_body_to_NED, v_ned, _, _, _ = rotation_matrix(phi, theta, psi, v_body)
        
        # Integrate position (P_NED)
        if i > 0:
            pos_ned = pos_ned + v_ned * dt
            
        vNED_list.append(v_ned)
        
        # NOTE: calculos.py previously returned z as negative for altitude.
        # But wait, looking at integrate_imu_data:
        # z += v_NED[2] * dt * -1 # El eje Z del NED apunta hacia abajo
        # Actually in HUD.py: 
        # altitude = self.P_ned_list[idx][2]
        # Pd = -altitude
        # Meaning HUD expects P_ned[2] to be ALTITUDE (positive up).
        # So we should store -pos_ned[2] in the 3rd component.
        Pned_list.append(np.array([pos_ned[0], pos_ned[1], -pos_ned[2]]))
        
    return (
        list(t),
        vNED_list,
        Pned_list,
        list(np.degrees(X[:, 6])), # phi
        list(np.degrees(X[:, 7])), # theta
        list(np.degrees(X[:, 8])), # psi
        list(X[:, 3]), # p
        list(X[:, 4]), # q
        list(X[:, 5]), # r
        list(X[:, 0]), # u
        list(X[:, 1]), # v
        list(X[:, 2])  # w
    )
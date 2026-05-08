import numpy as np

# ==========================================
# 1. CONSTANTS FOR RCAM
# ==========================================
m      = 120000.0   # kg  – aircraft mass
g_acc  = 9.81       # m/s^2
c_mac  = 6.6        # m   – mean aerodynamic chord
lt     = 24.8       # m   – tail moment arm
S      = 260.0      # m^2 – wing area
St     = 64.0       # m^2 – tail area
rho    = 1.225      # kg/m^3 – sea-level air density

# Centre of gravity
X_cg = 0.23 * c_mac
Y_cg = 0.0
Z_cg = 0.1  * c_mac

# Aerodynamic centre
X_ac = 0.12 * c_mac
Y_ac = 0.0
Z_ac = 0.0

# Engine attachment points (body frame)
X_apt1, Y_apt1, Z_apt1 = 0.0, -7.94,  1.9
X_apt2, Y_apt2, Z_apt2 = 0.0,  7.94,  1.9

# Inertia tensor (body frame)
Ixx = 40.07e6
Iyy = 64.00e6
Izz = 99.12e6
Ixz =  2.79e6

Ib     = np.array([[Ixx,   0.0, -Ixz],
                   [0.0,   Iyy,  0.0],
                   [-Ixz,  0.0,  Izz]])
invIb  = np.linalg.inv(Ib)


# ==========================================
# 2. UTILITY FUNCTIONS
# ==========================================

def rotation_matrix(phi, theta, psi, v_body):
    """
    Returns (R_body_to_NED, v_NED, phi, theta, psi).
    phi, theta, psi in DEGREES (for compatibility with RCAM_HUD.py).
    """
    phi_r   = np.radians(phi)
    theta_r = np.radians(theta)
    psi_r   = np.radians(psi)

    Rz = np.array([[ np.cos(psi_r),  np.sin(psi_r), 0],
                   [-np.sin(psi_r),  np.cos(psi_r), 0],
                   [0,               0,              1]])
    Ry = np.array([[ np.cos(theta_r), 0, -np.sin(theta_r)],
                   [0,                1,  0              ],
                   [ np.sin(theta_r), 0,  np.cos(theta_r)]])
    Rx = np.array([[1,  0,              0            ],
                   [0,  np.cos(phi_r),  np.sin(phi_r)],
                   [0, -np.sin(phi_r),  np.cos(phi_r)]])

    R_NED_to_body = Rx @ Ry @ Rz
    R_body_to_NED = R_NED_to_body.T
    v_NED = R_body_to_NED @ v_body
    return R_body_to_NED, v_NED, phi, theta, psi


def angle_of_attack(u, w):
    """Alpha in degrees."""
    if abs(u) < 1e-3 and abs(w) < 1e-3:
        return 0.0
    return np.rad2deg(np.arctan2(w, u))


def sideslip_angle(u, v, w):
    """Beta in degrees."""
    Va = np.sqrt(u**2 + v**2 + w**2)
    if Va < 1e-3:
        return 0.0
    return np.rad2deg(np.arcsin(np.clip(v / Va, -1.0, 1.0)))


def climb_angle(v_NED):
    """Climb angle in degrees (positive = climbing)."""
    vx, vy, vz = v_NED[0], v_NED[1], v_NED[2]
    return np.rad2deg(np.arctan2(-vz, np.sqrt(vx**2 + vy**2)))


def angular_rates_to_euler_rates(p, q, r, phi_deg, theta_deg):
    """
    Kinematic equations: maps body rates [p,q,r] -> Euler rates [phi_dot, theta_dot, psi_dot].
    phi_deg, theta_deg in DEGREES.
    Returns rates in rad/s (integrate to get radians, then convert to degrees for storage).
    """
    phi_r   = np.radians(phi_deg)
    theta_r = np.radians(theta_deg)

    cos_t = np.cos(theta_r)
    if abs(cos_t) < 1e-6:          # Avoid singularity at ±90 °
        cos_t = 1e-6

    H = np.array([[1, np.sin(phi_r)*np.tan(theta_r), np.cos(phi_r)*np.tan(theta_r)],
                  [0, np.cos(phi_r),                 -np.sin(phi_r)               ],
                  [0, np.sin(phi_r)/cos_t,            np.cos(phi_r)/cos_t         ]])
    return H @ np.array([p, q, r])   # [phi_dot, theta_dot, psi_dot] in rad/s


# ==========================================
# 3. RCAM AERODYNAMIC / DYNAMIC MODEL
# ==========================================

def xdot(X, U):
    """
    RCAM 9-DOF equations of motion (translational + rotational + kinematics).

    State  X = [u, v, w,  p, q, r,  phi, theta, psi]  (m/s, rad/s, rad)
    Input  U = [d_A, d_E, d_R, d_th1, d_th2]
               d_A  : aileron deflection  (rad)
               d_E  : elevator deflection (rad)
               d_R  : rudder deflection   (rad)
               d_th1: throttle engine 1   (0-1, mapped to fraction of m*g)
               d_th2: throttle engine 2   (0-1)

    Returns x_dot (9,) in same units as X.
    """
    u, v, w, p, q, r, phi, theta, psi = X
    d_A, d_E, d_R, d_th1, d_th2       = U

    # --- Airspeed and aero angles ---
    Va = np.sqrt(u**2 + v**2 + w**2)
    if Va < 1.0:
        Va = 1.0                         # Avoid /0 at startup

    alpha = np.arctan2(w, u)             # rad
    beta  = np.arcsin(np.clip(v/Va, -1.0, 1.0))   # rad

    # Dynamic pressure
    Q = 0.5 * rho * Va**2

    # --- Lift coefficient (wing+body) ---
    n              = 5.5
    alpha_L0       = np.radians(-11.5)   # zero-lift angle
    a0, a1, a2, a3 = 15.212, -155.2, 609.2, -768.5

    if alpha < np.radians(14.5):
        Cl_wb = n * (alpha - alpha_L0)
    else:
        Cl_wb = a0 + a1*alpha + a2*alpha**2 + a3*alpha**3

    # Tail downwash
    depsilon_dalpha = 0.25
    epsilon  = depsilon_dalpha * (alpha - alpha_L0)
    alpha_t  = alpha - epsilon + d_E + 1.3 * q * lt / Va
    Cl_t     = 3.1 * (St / S) * alpha_t

    Cl = Cl_wb + Cl_t

    # --- Drag coefficient ---
    Cd = 0.13 + 0.07 * (n * alpha + 0.654)**2

    # --- Side-force coefficient ---
    Cy = -1.6 * beta + 0.24 * d_R

    # --- Aerodynamic forces in stability frame, then body frame ---
    # Stability → body rotation by alpha about y-axis
    cos_a, sin_a = np.cos(alpha), np.sin(alpha)
    cos_b, sin_b = np.cos(beta),  np.sin(beta)

    # Forces in stability frame: [Drag (fwd), Side, Lift (up)]
    F_aero_s = Q * S * np.array([-Cd, Cy, -Cl])

    # Stability → body
    R_s2b = np.array([[ cos_a, 0, -sin_a],
                      [ 0,     1,  0    ],
                      [ sin_a, 0,  cos_a]])
    Fab = R_s2b @ F_aero_s   # aerodynamic force in body frame

    # --- Aerodynamic moment coefficients about ac (body frame) ---
    # Non-dimensional rates
    pb = p * c_mac / (2 * Va)
    qb = q * c_mac / (2 * Va)
    rb = r * c_mac / (2 * Va)   # (rb computed for completeness; not all used below)

    # Pitching moment
    Cm_ac = (-0.59 - 3.1 * (St * lt) / (S * c_mac)) * alpha_t

    # Rolling moment
    Cl_ac = -1.4 * beta + (-11 * pb + 5 * rb) * c_mac / Va + (-0.6 * d_A + 0.22 * d_R)

    # Yawing moment
    Cn_ac = ((1 - alpha * (180.0 / (15.0 * np.pi))) * beta
             + (1.7 * pb - 11.5 * rb) * c_mac / Va
             + (-0.63 * d_R))

    n_dash = np.array([Cl_ac, Cm_ac, Cn_ac])
    M_a_ac_b = Q * S * c_mac * n_dash

    # Transfer moment from ac to cg
    r_cg_vec = np.array([X_cg, Y_cg, Z_cg])
    r_ac_vec = np.array([X_ac, Y_ac, Z_ac])
    Ma_cg_b  = M_a_ac_b + np.cross(r_cg_vec - r_ac_vec, Fab)

    # --- Propulsion ---
    F1 = d_th1 * m * g_acc
    F2 = d_th2 * m * g_acc
    F_prop = np.array([F1 + F2, 0.0, 0.0])

    r_apt1 = np.array([X_apt1, Y_apt1, Z_apt1])
    r_apt2 = np.array([X_apt2, Y_apt2, Z_apt2])
    M_eng  = (np.cross(r_apt1 - r_cg_vec, np.array([F1, 0, 0])) +
              np.cross(r_apt2 - r_cg_vec, np.array([F2, 0, 0])))

    # --- Gravity in body frame ---
    R_b2n, _, _, _, _ = rotation_matrix(np.degrees(phi), np.degrees(theta), np.degrees(psi),
                                        np.array([u, v, w]))
    F_grav_ned  = np.array([0.0, 0.0, m * g_acc])
    F_grav_body = R_b2n.T @ F_grav_ned

    # --- Equations of motion ---
    omega   = np.array([p, q, r])
    V_body  = np.array([u, v, w])

    F_total = Fab + F_prop + F_grav_body
    # Newton: F = m*(V_dot + omega x V)
    V_dot   = (1.0/m) * F_total - np.cross(omega, V_body)   # [u_dot, v_dot, w_dot]

    M_total = Ma_cg_b + M_eng
    # Euler: M = Ib*omega_dot + omega x (Ib*omega)
    omega_dot = invIb @ (M_total - np.cross(omega, Ib @ omega))  # [p_dot, q_dot, r_dot]

    # Kinematics
    euler_rates = angular_rates_to_euler_rates(p, q, r,
                                               np.degrees(phi), np.degrees(theta))
    # [phi_dot, theta_dot, psi_dot] in rad/s

    return np.array([V_dot[0],    V_dot[1],    V_dot[2],
                     omega_dot[0], omega_dot[1], omega_dot[2],
                     euler_rates[0], euler_rates[1], euler_rates[2]])


# ==========================================
# 4. SCENARIO DEFINITIONS
# ==========================================

def _build_scenario(scenario_id):
    """
    Returns (X0, U, T_end, dt) for each RCAM scenario.
    X0 = [u, v, w, p, q, r, phi, theta, psi]  (SI, radians for angles)
    """
    # Trim condition: ~78 m/s level flight, 2° pitch
    u0     = 78.0
    theta0 = np.radians(2.0)
    # Rough trim throttle to balance lift
    d_th   = 0.3

    X0 = np.array([u0, 0.0, u0*np.tan(theta0), 0.0, 0.0, 0.0, 0.0, theta0, 0.0])

    if scenario_id == 1:
        # Nominal simulation
        U = np.array([0.0, np.radians(-1.0), 0.0, d_th, d_th])
        T_end = 60.0

    elif scenario_id == 2:
        # Aileron step +5 °
        U = np.array([np.radians(5.0), np.radians(-1.0), 0.0, d_th, d_th])
        T_end = 60.0

    elif scenario_id == 3:
        # Engine 1 shutdown
        U = np.array([0.0, np.radians(-1.0), 0.0, 0.0, d_th])
        T_end = 60.0

    elif scenario_id == 4:
        # PSO trim NE heading (psi0 = 45°)
        X0[8] = np.radians(45.0)
        U = np.array([0.0, np.radians(-1.0), 0.0, d_th, d_th])
        T_end = 60.0

    else:
        U = np.array([0.0, 0.0, 0.0, d_th, d_th])
        T_end = 60.0

    return X0, U, T_end


# ==========================================
# 5. NUMERICAL INTEGRATION — RK4
# ==========================================

def _rk4_step(X, U, dt):
    k1 = xdot(X,            U)
    k2 = xdot(X + dt/2*k1,  U)
    k3 = xdot(X + dt/2*k2,  U)
    k4 = xdot(X + dt*k3,    U)
    return X + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)


def run_rcam_scenario(scenario_id, dt=0.05):
    """
    Integrates the RCAM model for the given scenario.

    Returns a tuple matching RCAM_HUD.py's expected signature:
        (time_l, vNED_l, Pned_l,
         phi_l, theta_l, psi_l,
         p_l, q_l, r_l,
         u_l, v_l, w_l)

    All angle lists are in DEGREES.
    vNED_l  : list of np.array([vN, vE, vAlt])  (Alt positive up)
    Pned_l  : list of np.array([Pn, Pe, Alt])   (Alt positive up)
    """
    X0, U, T_end = _build_scenario(scenario_id)
    steps = int(T_end / dt)

    X   = X0.copy()
    pos = np.zeros(3)   # [Pn, Pe, Alt]   Alt positive-up

    time_l  = []
    vNED_l  = []
    Pned_l  = []
    phi_l   = []; theta_l = []; psi_l = []
    p_l     = []; q_l     = []; r_l   = []
    u_l     = []; v_l     = []; w_l   = []

    for i in range(steps):
        t = (i + 1) * dt

        X = _rk4_step(X, U, dt)

        u_s, v_s, w_s, p_s, q_s, r_s, phi_s, theta_s, psi_s = X

        # Velocity in NED
        R_b2n, v_NED, _, _, _ = rotation_matrix(
            np.degrees(phi_s), np.degrees(theta_s), np.degrees(psi_s),
            np.array([u_s, v_s, w_s])
        )

        # Position integration (NED, Alt positive up = -Pd)
        pos[0] += v_NED[0] * dt   # North
        pos[1] += v_NED[1] * dt   # East
        pos[2] -= v_NED[2] * dt   # Alt (up) = -vD*dt

        # vNED stored as [vN, vE, vAlt_up]
        v_ned_stored = np.array([v_NED[0], v_NED[1], -v_NED[2]])

        time_l.append(t)
        vNED_l.append(v_ned_stored.copy())
        Pned_l.append(pos.copy())

        phi_l.append(np.degrees(phi_s))
        theta_l.append(np.degrees(theta_s))
        psi_l.append(np.degrees(psi_s))

        p_l.append(p_s)
        q_l.append(q_s)
        r_l.append(r_s)

        u_l.append(u_s)
        v_l.append(v_s)
        w_l.append(w_s)

    return (time_l, vNED_l, Pned_l,
            phi_l, theta_l, psi_l,
            p_l, q_l, r_l,
            u_l, v_l, w_l)

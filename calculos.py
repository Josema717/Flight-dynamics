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
    euler_rates = H @ angular_rates
    return euler_rates


with open('tello_imu_example.csv', 'r', newline='',encoding='utf-8') as imu_raw:
    imu = list(csv.DictReader(imu_raw))
u,v,w = 0, 0, 0 # Inicializar velocidades en el body
x,y,z = 0, 0, 0 # Inicializar posiciones en el NED
phi, theta, psi = 0, 0, 0 # Inicializar ángulos de Euler


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
    # X = [x, y, z, phi, theta, psi, u, v, w, p, q, r]
    # U = [delta_ailerons, delta_elevator, delta_rudder, delta_throttle_1, delta_throttle_2]
    x_dot = np.zeros(12)
    # Aquí iría la dinámica del sistema para calcular x_dot a partir de X y U
    X = np.array([0]*12) # Iniciar el vector de estado con ceros para evitar errores de índice
    U = np.array([0]*5)  # Iniciar el vector de control con ceros para evitar errores de índice
    x1, x2, x3, x4, x5, x6, x7, x8, x9 = X[0], X[1], X[2], X[3], X[4], X[5], X[6], X[7], X[8] #u, v, w, p, q, r , phi, theta, psi
    u1,u2,u3,u4,u5 = U[0], U[1], U[2], U[3], U[4] #delta_ailerons, delta_elevator, delta_rudder, delta_throttle_1, delta_throttle_2
    
    
    Va = np.sqrt(x1**2 + x2**2 + x3**2)
    alpha = np.atan2(x3, x1) if Va > 1e-3 else 0.0
    beta = np.arcsin(x2 / Va) if Va > 1e-3 else 0.0
    Q = 0.5 * 1.225 * Va**2 # Presión dinámica (ρ * V² / 2)
    w_be = np.array([x7, x8, x9]).T # Velocidad angular en el body
    V_body = np.array([x1, x2, x3]).T # Velocidad en el sistema de referencia del body

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
    

    p = np.poly1d([1, 0, 1])  # Representa x^2 + 1
    derivada = p.deriv()      # Devuelve 2x
    print(derivada(5))        # Evalúa la derivada en 5

    epsilon = depsilon_dalpha * (alpha - alpha_lift_0)
    lt = 24.8
    alpha_t = alpha - epsilon + u2 + 1.3*x5*lt/Va
    Cl_t = 3.1*St/S * alpha_t   # St es el área del estabilizador horizontal, S es el área del ala

    Cl = Cl_wb + Cl_t

    #Total drag coefficient
    Cd = 0.13 + 0.07(n*alpha+0.654)**2

    # Total side forces coefficient
    Cy = -16*beta + 0.24*u3

    #Rotate from Fs to Fw
    C_s_to_w= np.array([[np.cos(beta), np.sin(beta), 0],
                     [-np.sin(beta), np.cos(beta), 0],
                     [0, 0, 1]])
    CF_s = np.array([Cd, Cy, Cl]).T
    CF_w = C_s_to_w @ CF_s

    # Aerodynamic forces in body frame
    Fas = Q* S * CF_s
    R_s_to_body = np.array([[np.cos(alpha), 0, -np.sin(alpha)],
                            [0, 1, 0], 
                            [np.sin(alpha), 0, np.cos(alpha)]])
    Fab = R_s_to_body @ Fas

    # Nondimensional aero moment coefficient about the center of gravity in Fb
    n_dash = np.array([-1.4*beta],
                       [-0.59-3.1*St*lt*(alpha-epsilon)/Sl], 
                       [(1-alpha*180/15*np.pi)*beta])
    c_mac = c_mac ###Cuerda media aerodinámica
    
    dcm_dx = c_mac / Va * np.array([[-11, 0, 5],
                                    [0, -4.03*(St*lt**2)/(S*c_mac**2), 0],  
                                    [1.7, 0, -11.5*beta]])
    dcm_du = np.array([[-0.6, 0, 0.22],
                    [0, -3.1*(St*lt)/(Sl*c_mac), 0],  
                    [0, 0, -0.63]])

    #The moments about the aerodynamic center in the body frame
    Cm_ac_b = n_dash + dcm_dx @ np.array([x4, x5, x6]).T + dcm_du @ np.array([u1, u2, u3]).T
    M_a_ac_b = c_mac * Q * S * Cm_ac_b
    
    r_cg = np.array([0.23*c_mac, 0, 0.1*c_mac]) # Vector desde el centro de gravedad al centro aerodinámico en el sistema de referencia del body
    r_ac = np.array([0.12*c_mac, 0, 0])

    Ma_cg_b = M_a_ac_b + np.cross(Fab, (r_cg - r_ac))
    
    #Propulsion effects
    F1 = u4*m*g
    F2 = u5*m*g   #m es la masa del avión, g es la gravedad
    F_prop_1 = np.array([F1, 0, 0]) # Asumiendo que la fuerza de propulsión actúa en el eje X del body
    F_prop_2 = np.array([F2, 0, 0]) # Asumiendo que la fuerza de propulsión actúa en el eje X del body
    F_prop_total = F_prop_1 + F_prop_2

    #Momentos que generan las fuerzas de propulsion en el centro de gravedad y en el eje de referencia del body
    u_dash_1 = np.array([X_cg - X_apt_1],     #Posicion del primer motor en el eje del body
                        [Y_apt_1 - Y_cg],
                        [Z_eg - Z_apt_1])

    u_dash_2 = np.array([X_cg - X_apt_2],     #Posicion del segundo motor en el eje del body
                        [Y_apt_2 - Y_cg],
                        [Z_eg - Z_apt_2])

    M_engine_cg_1_body = np.cross(u_dash_1, F_prop_1)   # Momento generado por la fuerza de propulsion del primer motor respecto al centro de gravedad en el sistema de referencia del body
    M_engine_cg_2_body = np.cross(u_dash_2, F_prop_2)   # Momento generado por la fuerza de propulsion del segundo motor respecto al centro de gravedad en el sistema de referencia del body

    M_engine_cg = M_engine_cg   #Placeholder, tengo que buscar este valor en el documento del RCAM
    M_total_engine_cg_b = M_engine_cg + M_engine_cg_1_body + M_engine_cg_2_body


    # Grativy effects
    F_gravity_ned = np.array([0, 0, m*g]) # Fuerza de gravedad en el sistema de referencia NED
    R_body_to_NED = R_body_to_NED, _, _, _, _, _,  =rotation_matrix(phi=x7, theta=x8, psi=x9,v_body=np.array([x1, x2, x3])) # Matriz de rotación del body al NED
    F_gravity_body = R_body_to_NED.T @ F_gravity_ned # Fuerza de gravedad en el sistema de referencia del body


    # Explicit first order form

    F_total_body = Fab + F_prop_total + F_gravity_body
    X1_dot, X2_dot, X3_dot, X4_dot, X5_dot, X6_dot, X7_dot, X8_dot, X9_dot = 0, 0, 0, 0, 0, 0, 0, 0, 0

    
    lineal_acceleration = 1/m * F_total_body - np.cross(w_be, V_body) # Aceleración en el body frame (u_dot, v_dot, w_dot)

    X1_dot, X2_dot, X3_dot = lineal_acceleration[0], lineal_acceleration[1], lineal_acceleration[2] # Aceleraciones en el body frame (u_dot, v_dot, w_dot)


    M_cg = Ma_cg_b + M_total_engine_cg_b # Momento total en el centro de gravedad en el sistema de referencia del body
    Ib = np.array([[Ixx, Ixy, Ixz],
                    [Iyx, Iyy, Iyz],
                    [Izx, Izy, Izz]]) # Matriz de inercia del avión en el sistema de referencia del body, placeholder

    
    rotational_acceleration = np.linalg.inv(Ib) @ (M_cg - np.cross(w_be, Ib @ w_be))

    X4_dot, X5_dot, X6_dot = rotational_acceleration[0], rotational_acceleration[1], rotational_acceleration[2]


    euler_rates = angular_rates_to_euler(x4, x5, x6, x8) # Euler rates (p, q, r)
    X7_dot, X8_dot, X9_dot = euler_rates[0], euler_rates[1], euler_rates[2]
    x_dot = np.array([X1_dot, X2_dot, X3_dot, X4_dot, X5_dot, X6_dot, X7_dot, X8_dot, X9_dot])

    return x_dot

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
    return np.rad2deg(np.arcsin(v/V))


def climb_angle(alpha_deg, pitch_deg):
    """
    Gamma (γ) — Climb Angle [deg]
    Relationship: pitch = alpha + gamma  →  gamma = pitch - alpha
    Valid when sideslip is zero (wings-level flight).
    """
    return pitch_deg - alpha_deg


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

estado = aircraft_state(alpha=angle_of_attack(u,w), beta=sideslip_angle(u,v,w), climb=climb_angle(angle_of_attack(u,w),theta), u=u, v=v, w=w, p=p, q=q, r=r, phi=phi, theta=theta, psi=psi, v_body=v_body)
pprint.pprint(estado)

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
with open('tello_imu_example.csv', 'r', newline='',encoding='utf-8') as imu_raw:
    imu = list(csv.DictReader(imu_raw))
u,v,w = 0, 0, 0 # Inicializar velocidades en el body
x,y,z = 0, 0, 0 # Inicializar posiciones en el NED
phi, theta, psi = 0, 0, 0 # Inicializar ángulos de Euler


def integrate_imu_data(imu):
    u,v,w = 0, 0, 0 # Inicializar velocidades en el body
    x,y,z = 0, 0, 0 # Inicializar posiciones en el NED
    #listas para guardar los datos para graficar
    time_list, u_list, v_list, w_list, x_list, y_list, z_list, phi_list, theta_list, psi_list = [], [], [], [], [], [], [], [], [], []
    for i in range(1, len(imu)):
        row = imu[i]
        row_prev = imu[i-1]
        dt = float(row["time_s"]) - float(row_prev["time_s"]) # Paso en segundos entre dos filas consecutivas

    # Crear el vector omega de velocidad angular para el body
        p = float(row["gyro_p_rad_s"])
        q = float(row["gyro_q_rad_s"])
        r = float(row["gyro_r_rad_s"])
        omega_body = np.array([p, q, r]) # Velocidad angular en el sistema de referencia del body

        # Crear el vector de velocidades en el body
        u_dot = float(row["accel_x_m_s2"]) + 0
        v_dot = float(row["accel_y_m_s2"]) + 0
        w_dot = float(row["accel_z_m_s2"]) + 9.81
                
        #Integral para las velocidades
        u += u_dot * dt
        v += v_dot * dt
        w += w_dot * dt

        v_body = np.array([u, v, w]) # Actualizar el vector de velocidades en el body
        v_NED = R_body_to_NED @ v_body # Transformar las velocidades del body al NED

        #Integral para la posicion en el NED
        P_n, P_e, P_d = 0, 0, 0 # Posición inicial en el NED
        P_ned = np.array([P_n, P_e, P_d]) # Posición actual en el NED
        P_ned[0] += v_NED[0] * dt
        P_ned[1] += v_NED[1] * dt
        P_ned[2] += v_NED[2] * dt

        #Integral para los ángulos de Euler
        euler_rates = angular_rates_to_euler(p, q, r, phi, theta)
        phi += np.rad2deg(euler_rates[0] * dt)
        theta += np.rad2deg(euler_rates[1] * dt)
        psi += np.rad2deg(euler_rates[2] * dt)

        #Guardar los datos para graficar
        time_list.append(float(imu[i]["time_s"]))
        u_list.append(u); v_list.append(v); w_list.append(w)
        x_list.append(P_ned[0]); y_list.append(P_ned[1]); z_list.append(P_ned[2])
        phi_list.append(phi); theta_list.append(theta); psi_list.append(psi)

    return time_list, v_NED, P_ned, phi, theta, psi
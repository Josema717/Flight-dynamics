import sys
import numpy as np
import csv
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.transforms import Affine2D

from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QSlider, QLabel, QSplitter, QPushButton, QTabWidget)
from PyQt6.QtCore import Qt, QTimer
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import calculos

class MplCanvas(FigureCanvas):
    def __init__(self, parent=None, width=5, height=4, dpi=100, projection=None):
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        if projection == '3d':
            self.ax = self.fig.add_subplot(111, projection='3d')
        else:
            self.ax = self.fig.add_subplot(111)
            self.ax.axis('off')
        super(MplCanvas, self).__init__(self.fig)

class ViewsCanvas(FigureCanvas):
    def __init__(self, parent=None, width=12, height=4, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        self.axs = self.fig.subplots(1, 3)
        for ax in self.axs:
            ax.axis('off')
        super(ViewsCanvas, self).__init__(self.fig)

class TriplePlotCanvas(FigureCanvas):
    def __init__(self, parent=None, width=5, height=6, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        self.axs = self.fig.subplots(3, 1, sharex=True)
        self.fig.tight_layout(pad=2.0)
        super(TriplePlotCanvas, self).__init__(self.fig)

class HUDInterface(QMainWindow):
    def __init__(self, csv_file):
        super().__init__()
        self.setWindowTitle("HUD Viewer")
        self.resize(1200, 800)

        # Load images
        self.img_top = mpimg.imread("top.png")
        self.img_side = mpimg.imread("side.png")
        self.img_front = mpimg.imread("front.png")

        # Load Data
        with open(csv_file, 'r', newline='', encoding='utf-8') as f:
            imu = list(csv.DictReader(f))

        self.time_list = [float(imu[0]["time_s"])]
        self.u_list, self.v_list, self.w_list = [0.0], [0.0], [0.0]
        self.phi_list, self.theta_list, self.psi_list = [0.0], [0.0], [0.0]
        self.v_NED_list = [np.array([0.0, 0.0, 0.0])]
        self.P_ned_list = [np.array([0.0, 0.0, 0.0])]
        self.p_list, self.q_list, self.r_list = [0.0], [0.0], [0.0]

        u, v, w = 0.0, 0.0, 0.0
        phi, theta, psi = 0.0, 0.0, 0.0
        P_ned = np.array([0.0, 0.0, 0.0])

        for i in range(1, len(imu)):
            row = imu[i]
            row_prev = imu[i-1]
            dt = float(row["time_s"]) - float(row_prev["time_s"])
            
            p = float(row["gyro_p_rad_s"])
            q = float(row["gyro_q_rad_s"])
            r = float(row["gyro_r_rad_s"])
            
            u_dot = float(row["accel_x_m_s2"])
            v_dot = float(row["accel_y_m_s2"])
            w_dot = float(row["accel_z_m_s2"]) + 9.81
            
            u += u_dot * dt
            v += v_dot * dt
            w += w_dot * dt
            
            v_body = np.array([u, v, w])
            # Getting v_NED and R_body_to_NED at current step
            R_body_to_NED, v_NED, _, _, _ = calculos.rotation_matrix(phi, theta, psi, v_body)
            P_ned = P_ned + v_NED * dt
            
            euler_rates = calculos.angular_rates_to_euler(p, q, r, phi, theta)
            phi += np.rad2deg(euler_rates[0] * dt)
            theta += np.rad2deg(euler_rates[1] * dt)
            psi += np.rad2deg(euler_rates[2] * dt)
            
            self.time_list.append(float(row["time_s"]))
            self.u_list.append(u)
            self.v_list.append(v)
            self.w_list.append(w)
            self.phi_list.append(phi)
            self.theta_list.append(theta)
            self.psi_list.append(psi)
            self.v_NED_list.append(v_NED)
            self.P_ned_list.append(P_ned)
            self.p_list.append(p)
            self.q_list.append(q)
            self.r_list.append(r)
            
        self.n_points = len(self.time_list)

        # UI Setup
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        splitter = QSplitter(Qt.Orientation.Vertical)
        main_layout.addWidget(splitter)

        top_widget = QWidget()
        top_layout = QHBoxLayout(top_widget)
        
        self.views_canvas = ViewsCanvas(self, width=9, height=3)
        top_layout.addWidget(self.views_canvas)
        
        bottom_widget = QWidget()
        bottom_layout = QHBoxLayout(bottom_widget)
        
        self.text_canvas = MplCanvas(self, width=3, height=4)
        bottom_layout.addWidget(self.text_canvas)
        
        self.plot_tabs = QTabWidget()
        bottom_layout.addWidget(self.plot_tabs)

        self.ned_canvas = MplCanvas(self, width=4, height=4, projection='3d')
        self.plot_tabs.addTab(self.ned_canvas, "NED Frame")
        
        self.traj3d_canvas = MplCanvas(self, width=4, height=4, projection='3d')
        self.plot_tabs.addTab(self.traj3d_canvas, "3D Trajectory")
        
        self.traj2d_canvas = MplCanvas(self, width=4, height=4)
        self.plot_tabs.addTab(self.traj2d_canvas, "2D Trajectory")
        
        self.euler_canvas = TriplePlotCanvas(self, width=4, height=4)
        self.plot_tabs.addTab(self.euler_canvas, "Euler Angles")
        
        self.rates_canvas = TriplePlotCanvas(self, width=4, height=4)
        self.plot_tabs.addTab(self.rates_canvas, "Angular Rates")

        splitter.addWidget(top_widget)
        splitter.addWidget(bottom_widget)

        ctrl_layout = QHBoxLayout()
        self.play_button = QPushButton("Play")
        self.play_button.clicked.connect(self.toggle_play)
        ctrl_layout.addWidget(self.play_button)

        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setMaximum(self.n_points - 1)
        self.slider.setValue(0)
        self.slider.valueChanged.connect(self.on_slider_moved)
        ctrl_layout.addWidget(self.slider)

        self.info_label = QLabel("Time: 0.00s")
        ctrl_layout.addWidget(self.info_label)
        main_layout.addLayout(ctrl_layout)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.dt_ms = 33 # roughly 30 FPS target visual update for smoother GUI event loop

        self.current_idx = 0
        self.is_playing = False

        # Init references for views
        self.vp_yaw = self.views_canvas.axs[0]
        self.vp_pitch = self.views_canvas.axs[1]
        self.vp_roll = self.views_canvas.axs[2]
        
        self.text_display = self.text_canvas.ax.text(0.05, 0.95, "", fontsize=12, va='top', family='monospace')
        
        # Precompute trajectories to save rendering time
        self.Pn = np.array([p[0] for p in self.P_ned_list])
        self.Pe = np.array([p[1] for p in self.P_ned_list])
        self.Pd = np.array([p[2] for p in self.P_ned_list])
        
        self.update_plots()

    def toggle_play(self):
        if self.is_playing:
            self.timer.stop()
            self.play_button.setText("Play")
            self.is_playing = False
        else:
            if self.current_idx >= self.n_points - 1:
                self.slider.setValue(0)
            self.timer.start(self.dt_ms)
            self.play_button.setText("Pause")
            self.is_playing = True

    def on_slider_moved(self, value):
        self.current_idx = value
        self.update_plots()

    def update_frame(self):
        if self.current_idx < self.n_points - 1:
            # Advance frames by a proportional amount based on dt_ms so it plays in real time if possible
            # tello dt is around 0.002s, so for 20ms we advance about 10 steps
            step_size = max(1, int(self.dt_ms / 1000.0 / (self.time_list[min(self.current_idx+1, self.n_points-1)] - self.time_list[self.current_idx] + 1e-6)))
            self.current_idx += step_size
            self.current_idx = min(self.current_idx, self.n_points - 1)
            self.slider.blockSignals(True)
            self.slider.setValue(self.current_idx)
            self.slider.blockSignals(False)
            self.update_plots()
        else:
            self.toggle_play()

    def update_plots(self):
        idx = self.current_idx
        
        u, v, w = self.u_list[idx], self.v_list[idx], self.w_list[idx]
        phi, theta, psi = self.phi_list[idx], self.theta_list[idx], self.psi_list[idx]
        v_body = np.array([u, v, w])
        v_NED = self.v_NED_list[idx]
        
        self.info_label.setText(f"Time: {self.time_list[idx]:.2f}s")
        
        # Calculate derived metrics using calculos
        alpha = calculos.angle_of_attack(u, w)
        beta = calculos.sideslip_angle(u, v, w)
        climb = calculos.climb_angle(alpha, theta)
        
        # Text update
        text = (
            f"Velocity on the body:\n"
            f"u = {u:.2f} m/s\n"
            f"v = {v:.2f} m/s\n"
            f"w = {w:.2f} m/s\n\n"
            f"Angles:\n"
            f"alpha = {alpha:.2f}°\n"
            f"beta  = {beta:.2f}°\n"
            f"climb = {climb:.2f}°\n\n"
            f"Euler:\n"
            f"phi   = {phi:.2f}°\n"
            f"theta = {theta:.2f}°\n"
            f"psi   = {psi:.2f}°"
        )
        self.text_display.set_text(text)
        self.text_canvas.draw_idle()

        # Update Views
        self.vp_yaw.clear()
        self.vp_pitch.clear()
        self.vp_roll.clear()
        
        self.vp_yaw.axis("off"); self.vp_yaw.set_xlim(-2,2); self.vp_yaw.set_ylim(-2,2); self.vp_yaw.set_title(f"Yaw (\u03c8) = {psi:.1f}°")
        self.vp_pitch.axis("off"); self.vp_pitch.set_xlim(-2,2); self.vp_pitch.set_ylim(-2,2); self.vp_pitch.set_title(f"Pitch (\u03b8) = {theta:.1f}°")
        self.vp_roll.axis("off"); self.vp_roll.set_xlim(-2,2); self.vp_roll.set_ylim(-2,2); self.vp_roll.set_title(f"Roll (\u03c6) = {phi:.1f}°")

        # Yaw
        self.vp_yaw.arrow(0, 0, 0, 1, head_width=0.1, head_length=0.1, fc='pink', ec='pink', linewidth=2, zorder=1)
        self.vp_yaw.arrow(0, 0, 1, 0, head_width=0.1, head_length=0.1, fc='purple', ec='purple', linewidth=2, zorder=1)
        self.vp_yaw.text(1.1, 0, 'E', fontsize=10, color='purple', fontweight='bold')
        self.vp_yaw.text(0, 1.1, 'N', fontsize=10, color='pink', fontweight='bold')
        im1 = self.vp_yaw.imshow(self.img_top, extent=[-1,1,-1,1], zorder=2)
        trans1 = Affine2D().rotate_deg(-psi) + self.vp_yaw.transData
        im1.set_transform(trans1)
        self.vp_yaw.arrow(0, 0, 1, 0, fc='red', ec='red', transform=trans1, zorder=3)
        self.vp_yaw.arrow(0, 0, 0, 1, fc='green', ec='green', transform=trans1, zorder=3)
        self.vp_yaw.text(1.1, 0, 'y_b', color='red', transform=trans1)
        self.vp_yaw.text(0, 1.1, 'x_b', color='green', transform=trans1)

        # Pitch
        self.vp_pitch.arrow(0, 0, 1, 0, head_width=0.1, head_length=0.1, fc='purple', ec='purple', linewidth=2, zorder=1)
        self.vp_pitch.arrow(0, 0, 0, -1, head_width=0.1, head_length=0.1, fc='black', ec='black', linewidth=2, zorder=1)
        self.vp_pitch.text(1.1, 0, 'X', fontsize=10, color='purple', fontweight='bold')
        self.vp_pitch.text(0, -1.1, 'Z', fontsize=10, color='black', fontweight='bold')
        im2 = self.vp_pitch.imshow(self.img_side, extent=[-1,1,-1,1], zorder=2)
        trans2 = Affine2D().rotate_deg(theta) + self.vp_pitch.transData
        im2.set_transform(trans2)
        self.vp_pitch.arrow(0, 0, 1, 0, fc='red', transform=trans2)
        self.vp_pitch.arrow(0, 0, 0, -1, fc='blue', transform=trans2)
        self.vp_pitch.text(1.1, 0, 'x_b', color='red', transform=trans2)
        self.vp_pitch.text(0, -1.1, 'z_b', color='blue', transform=trans2)

        # Roll
        # Fix Y dimension to point in opposite direction (now points right, positive to East)
        self.vp_roll.arrow(0, 0, 1, 0, head_width=0.1, head_length=0.1, fc='pink', ec='pink', linewidth=2, zorder=1)
        self.vp_roll.arrow(0, 0, 0, -1, head_width=0.1, head_length=0.1, fc='black', ec='black', linewidth=2, zorder=1)
        self.vp_roll.text(1.1, 0, 'Y', fontsize=10, color='pink', fontweight='bold')
        self.vp_roll.text(0, -1.1, 'Z', fontsize=10, color='black', fontweight='bold')
        im3 = self.vp_roll.imshow(self.img_front, extent=[-1,1,-1,1], zorder=2)
        trans3 = Affine2D().rotate_deg(-phi) + self.vp_roll.transData
        im3.set_transform(trans3)
        self.vp_roll.arrow(0, 0, -1, 0, fc='green', transform=trans3)
        self.vp_roll.arrow(0, 0, 0, -1, fc='blue', transform=trans3)
        self.vp_roll.text(-1.1, 0, 'y_b', color='green', transform=trans3)
        self.vp_roll.text(0, -1.1, 'z_b', color='blue', transform=trans3)
        
        self.views_canvas.draw_idle()

        # Update 3D NED
        ax3d = self.ned_canvas.ax
        ax3d.cla()
        axis_length = max(10, np.linalg.norm(v_NED) * 1.5)
        
        # Base axes mapping to ENU plot format to preserve standard 3D chirality: X=East, Y=North, Z=-Down(Altitude)
        ax3d.quiver(0, 0, 0, 0, axis_length, 0, color='b', arrow_length_ratio=0.1)
        ax3d.text(0, axis_length, 0, 'N', color='b')
        ax3d.quiver(0, 0, 0, axis_length, 0, 0, color='b', arrow_length_ratio=0.1)
        ax3d.text(axis_length, 0, 0, 'E', color='b')
        ax3d.quiver(0, 0, 0, 0, 0, -axis_length, color='b', arrow_length_ratio=0.1)
        ax3d.text(0, 0, -axis_length, 'D', color='b')
        
        if np.linalg.norm(v_NED) > 0.1:
            ax3d.quiver(0, 0, 0, v_NED[1], v_NED[0], -v_NED[2], color='r')
            ax3d.text(v_NED[1], v_NED[0], -v_NED[2], 'V', color='k')
            
        R_body_to_NED, _, _, _, _ = calculos.rotation_matrix(phi, theta, psi, v_body)
        
        # Aircraft wireframe
        scale = axis_length * 0.4
        pts_body = np.array([
            [ 1.0,  0.0,  0.0],  # 0 Nose
            [-1.0,  0.0,  0.0],  # 1 Tail
            [-0.2,  1.0,  0.0],  # 2 Right wing
            [-0.2, -1.0,  0.0],  # 3 Left wing
            [-1.0,  0.0, -0.4],  # 4 Tail fin tip
            [-1.0,  0.0,  0.0],  # 5 Tail base
        ]).T
        
        pts_ned = R_body_to_NED @ pts_body * scale
        
        # Plotting mapped into ENU layout
        ax3d.plot(pts_ned[1,[0,1]], pts_ned[0,[0,1]], -pts_ned[2,[0,1]], color='orange', linewidth=4)
        ax3d.plot(pts_ned[1,[2,3]], pts_ned[0,[2,3]], -pts_ned[2,[2,3]], color='orange', linewidth=4)
        ax3d.plot(pts_ned[1,[4,5]], pts_ned[0,[4,5]], -pts_ned[2,[4,5]], color='orange', linewidth=4)
        
        ax3d.set_xlim([-axis_length, axis_length])
        ax3d.set_ylim([-axis_length, axis_length])
        ax3d.set_zlim([-axis_length, axis_length])
        ax3d.set_xlabel('East')
        ax3d.set_ylabel('North')
        ax3d.set_zlabel('-Down (Altitude)')
        ax3d.set_title("NED Attitude Tracker")
        
        self.ned_canvas.draw_idle()

        # Update 3D Trajectory
        axt3 = self.traj3d_canvas.ax
        axt3.cla()
        axt3.plot(self.Pn[:idx+1], self.Pe[:idx+1], -self.Pd[:idx+1], color='blue', label='Path')
        if idx > 0:
            axt3.scatter(self.Pn[idx], self.Pe[idx], -self.Pd[idx], color='red', label='Current Pos')
        axt3.set_title("3D Trajectory")
        axt3.set_xlabel("North")
        axt3.set_ylabel("East")
        axt3.set_zlabel("Altitude (-D)")
        axt3.legend(loc='best')
        self.traj3d_canvas.draw_idle()

        # Update 2D Trajectory
        axt2 = self.traj2d_canvas.ax
        axt2.cla()
        axt2.plot(self.Pe[:idx+1], self.Pn[:idx+1], color='green', label='Path')
        if idx > 0:
            axt2.scatter(self.Pe[idx], self.Pn[idx], color='red', label='Current Pos')
        axt2.set_title("2D Trajectory (East vs North)")
        axt2.set_xlabel("East")
        axt2.set_ylabel("North")
        axt2.legend(loc='best')
        axt2.grid(True)
        self.traj2d_canvas.draw_idle()

        # Update Euler Angles
        ax_phi, ax_theta, ax_psi = self.euler_canvas.axs
        ax_phi.cla(); ax_theta.cla(); ax_psi.cla()
        t = self.time_list[:idx+1]
        ax_phi.plot(t, self.phi_list[:idx+1], 'r')
        ax_theta.plot(t, self.theta_list[:idx+1], 'g')
        ax_psi.plot(t, self.psi_list[:idx+1], 'b')
        if idx > 0:
            ax_phi.plot(t[-1], self.phi_list[idx], 'ro')
            ax_theta.plot(t[-1], self.theta_list[idx], 'go')
            ax_psi.plot(t[-1], self.psi_list[idx], 'bo')
            
        ax_phi.set_ylabel("Roll (°)")
        ax_theta.set_ylabel("Pitch (°)")
        ax_psi.set_ylabel("Yaw (°)")
        ax_psi.set_xlabel("Time (s)")
        ax_phi.grid(True); ax_theta.grid(True); ax_psi.grid(True)
        self.euler_canvas.draw_idle()

        # Update Angular Rates
        ax_p, ax_q, ax_r = self.rates_canvas.axs
        ax_p.cla(); ax_q.cla(); ax_r.cla()
        ax_p.plot(t, self.p_list[:idx+1], 'r')
        ax_q.plot(t, self.q_list[:idx+1], 'g')
        ax_r.plot(t, self.r_list[:idx+1], 'b')
        if idx > 0:
            ax_p.plot(t[-1], self.p_list[idx], 'ro')
            ax_q.plot(t[-1], self.q_list[idx], 'go')
            ax_r.plot(t[-1], self.r_list[idx], 'bo')
            
        ax_p.set_ylabel("p (rad/s)")
        ax_q.set_ylabel("q (rad/s)")
        ax_r.set_ylabel("r (rad/s)")
        ax_r.set_xlabel("Time (s)")
        ax_p.grid(True); ax_q.grid(True); ax_r.grid(True)
        self.rates_canvas.draw_idle()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    csv_file = "tello_imu_example.csv"
    window = HUDInterface(csv_file)
    window.show()
    sys.exit(app.exec())
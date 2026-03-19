import sys
import numpy as np
import csv
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QSlider, QLabel, QSplitter, QPushButton, QTabWidget)
from PyQt6.QtCore import Qt, QTimer, QPointF, QRectF
from PyQt6.QtGui import QPainter, QPen, QColor, QFont, QImage, QPolygonF
import calculos

class TextCanvas(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        self.label = QLabel("")
        self.label.setFont(QFont("Monospace", 12))
        self.label.setAlignment(Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft)
        layout.addWidget(self.label)
        
    def set_text(self, text):
        self.label.setText(text)

def draw_arrow(painter, x1, y1, x2, y2, color, text='', text_offset=(0,0)):
    painter.setPen(QPen(QColor(color), 2))
    painter.setBrush(QColor(color))
    painter.drawLine(int(x1), int(y1), int(x2), int(y2))
    
    if x1 == x2 and y1 == y2: return
    
    angle = np.arctan2(y2 - y1, x2 - x1)
    arrow_len = 10
    p1 = QPointF(x2 - arrow_len * np.cos(angle - np.pi/6),
                 y2 - arrow_len * np.sin(angle - np.pi/6))
    p2 = QPointF(x2 - arrow_len * np.cos(angle + np.pi/6),
                 y2 - arrow_len * np.sin(angle + np.pi/6))
    poly = QPolygonF([QPointF(x2, y2), p1, p2])
    painter.drawPolygon(poly)
    
    if text:
        painter.drawText(int(x2 + text_offset[0]), int(y2 + text_offset[1]), text)

class BaseView(QWidget):
    def __init__(self, title, img_path):
        super().__init__()
        self.title = title
        self.img = QImage(img_path)
        self.angle = 0.0

    def set_angle(self, angle):
        self.angle = angle
        self.update()
        
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform)
        w, h = self.width(), self.height()
        painter.fillRect(0, 0, w, h, QColor(255, 255, 255))
        painter.drawText(10, 20, f"{self.title} = {self.angle:.1f}°")
        self.draw_custom(painter, w/2, h/2, w, h)

class YawView(BaseView):
    def draw_custom(self, painter, cx, cy, w, h):
        scale = min(w, h) / 4.0
        painter.translate(cx, cy)
        draw_arrow(painter, 0, 0, 0, -scale, 'pink', 'N', (-15, -10))
        draw_arrow(painter, 0, 0, scale, 0, 'purple', 'E', (10, 5))
        painter.save()
        painter.rotate(self.angle)
        img_s = scale * 1.0
        painter.drawImage(QRectF(-img_s, -img_s, 2*img_s, 2*img_s), self.img)
        draw_arrow(painter, 0, 0, 0, -scale, 'green', 'x_b', (-15, -10))
        draw_arrow(painter, 0, 0, scale, 0, 'red', 'y_b', (10, 5))
        painter.restore()

class PitchView(BaseView):
    def draw_custom(self, painter, cx, cy, w, h):
        scale = min(w, h) / 4.0
        painter.translate(cx, cy)
        draw_arrow(painter, 0, 0, scale, 0, 'purple', 'X', (10, 5))
        draw_arrow(painter, 0, 0, 0, scale, 'black', 'Z', (-15, 20))
        painter.save()
        painter.rotate(-self.angle)
        img_s = scale * 1.0
        painter.drawImage(QRectF(-img_s, -img_s, 2*img_s, 2*img_s), self.img)
        draw_arrow(painter, 0, 0, scale, 0, 'green', 'x_b', (10, 5))
        draw_arrow(painter, 0, 0, 0, scale, 'blue', 'z_b', (-15, 20))
        painter.restore()

class RollView(BaseView):
    def draw_custom(self, painter, cx, cy, w, h):
        scale = min(w, h) / 4.0
        painter.translate(cx, cy)
        draw_arrow(painter, 0, 0, scale, 0, 'pink', 'Y', (10, 5))
        draw_arrow(painter, 0, 0, 0, scale, 'black', 'Z', (-15, 20))
        painter.save()
        painter.rotate(self.angle)
        img_s = scale * 1.0
        painter.drawImage(QRectF(-img_s, -img_s, 2*img_s, 2*img_s), self.img)
        draw_arrow(painter, 0, 0, -scale, 0, 'red', 'y_b', (-25, 5))
        draw_arrow(painter, 0, 0, 0, scale, 'blue', 'z_b', (-15, 20))
        painter.restore()

class ViewsCanvas(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QHBoxLayout(self)
        self.yaw_view = YawView("Yaw (\u03c8)", "top.png")
        self.pitch_view = PitchView("Pitch (\u03b8)", "side.png")
        self.roll_view = RollView("Roll (\u03c6)", "front.png")
        layout.addWidget(self.yaw_view)
        layout.addWidget(self.pitch_view)
        layout.addWidget(self.roll_view)

class Plot3DWidget(QWidget):
    def __init__(self, title="3D"):
        super().__init__()
        self.title = title
        self.lines = []
        self.texts = []
        self.scatter = []
        self.elev = 30
        self.azim = 45
        self.axis_length = 10

    def project(self, x, y, z, cx, cy, scale):
        az = np.deg2rad(self.azim)
        el = np.deg2rad(self.elev)
        x_rot = x * np.cos(az) - y * np.sin(az)
        y_rot = x * np.sin(az) + y * np.cos(az)
        xp = x_rot
        yp = y_rot * np.sin(el) - z * np.cos(el)
        return cx + xp * scale, cy - yp * scale

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        painter.fillRect(0, 0, w, h, QColor(255, 255, 255))
        painter.drawText(10, 20, self.title)
        
        cx, cy = w/2, h/2
        scale = min(w, h) / (2.5 * max(1, self.axis_length))
        
        for pts, color, width in self.lines:
            painter.setPen(QPen(QColor(color), width))
            poly = QPolygonF()
            for pt in pts:
                px, py = self.project(pt[0], pt[1], pt[2], cx, cy, scale)
                poly.append(QPointF(px, py))
            painter.drawPolyline(poly)
            
        for x, y, z, text, color in self.texts:
            painter.setPen(QPen(QColor(color)))
            px, py = self.project(x, y, z, cx, cy, scale)
            painter.drawText(QPointF(px, py), text)

        for x, y, z, color in self.scatter:
            painter.setPen(Qt.PenStyle.NoPen)
            painter.setBrush(QColor(color))
            px, py = self.project(x, y, z, cx, cy, scale)
            painter.drawEllipse(QPointF(px, py), 4, 4)

class Plot2DWidget(QWidget):
    def __init__(self, title="", xlabel="", ylabel=""):
        super().__init__()
        self.title = title
        self.xlabel = xlabel
        self.ylabel = ylabel
        self.lines = []
        self.scatter = []

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        painter.fillRect(0, 0, w, h, QColor(255, 255, 255))
        
        pad_l, pad_r, pad_t, pad_b = 50, 20, 30, 40
        plot_w = w - pad_l - pad_r
        plot_h = h - pad_t - pad_b
        
        if plot_w <= 0 or plot_h <= 0: return

        x_min, x_max, y_min, y_max = float('inf'), float('-inf'), float('inf'), float('-inf')
        for x_data, y_data, _ in self.lines:
            if len(x_data) > 0:
                x_min = min(x_min, np.min(x_data))
                x_max = max(x_max, np.max(x_data))
                y_min = min(y_min, np.min(y_data))
                y_max = max(y_max, np.max(y_data))
                
        for x, y, _ in self.scatter:
            x_min, x_max = min(x_min, x), max(x_max, x)
            y_min, y_max = min(y_min, y), max(y_max, y)
            
        if x_max == float('inf'):
            x_min, x_max, y_min, y_max = 0, 1, 0, 1
            
        if x_max == x_min: x_max, x_min = x_min + 1, x_min - 1
        if y_max == y_min: y_max, y_min = y_min + 1, y_min - 1
        
        dy = (y_max - y_min) * 0.1
        y_min -= dy
        y_max += dy
        
        def to_px(x, y):
            px = pad_l + (x - x_min) / (x_max - x_min) * plot_w
            py = h - pad_b - (y - y_min) / (y_max - y_min) * plot_h
            return QPointF(px, py)

        painter.setPen(QPen(QColor(200, 200, 200), 1))
        for i in range(5):
            y_val = y_min + i * (y_max - y_min) / 4
            p1 = to_px(x_min, y_val)
            p2 = to_px(x_max, y_val)
            painter.drawLine(p1, QPointF(p2.x(), p1.y()))
            painter.setPen(QPen(QColor('black')))
            painter.drawText(QRectF(0, p1.y() - 10, pad_l - 5, 20), Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter, f"{y_val:.1f}")
            painter.setPen(QPen(QColor(200, 200, 200), 1))

        for x_data, y_data, color in self.lines:
            if len(x_data) == 0: continue
            painter.setPen(QPen(QColor(color), 2))
            poly = QPolygonF()
            for x, y in zip(x_data, y_data):
                poly.append(to_px(x, y))
            painter.drawPolyline(poly)
            
        for x, y, color in self.scatter:
            painter.setPen(Qt.PenStyle.NoPen)
            painter.setBrush(QColor(color))
            pt = to_px(x, y)
            painter.drawEllipse(pt, 4, 4)
            
        painter.setPen(QPen(QColor('black')))
        painter.drawText(QRectF(pad_l, 0, plot_w, pad_t), Qt.AlignmentFlag.AlignCenter, self.title)
        
        painter.save()
        painter.translate(15, pad_t + plot_h/2)
        painter.rotate(-90)
        painter.drawText(QRectF(-plot_h/2, -15, plot_h, 30), Qt.AlignmentFlag.AlignCenter, self.ylabel)
        painter.restore()

        painter.drawText(QRectF(pad_l, h - 25, plot_w, 25), Qt.AlignmentFlag.AlignCenter, self.xlabel)

class TriplePlotCanvas(QWidget):
    def __init__(self, title1, title2, title3, xlabel, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0,0,0,0)
        self.p1 = Plot2DWidget(ylabel=title1)
        self.p2 = Plot2DWidget(ylabel=title2)
        self.p3 = Plot2DWidget(ylabel=title3, xlabel=xlabel)
        layout.addWidget(self.p1)
        layout.addWidget(self.p2)
        layout.addWidget(self.p3)

    def set_data(self, t, list1, idx, color1='red', color2='green', color3='blue'):
        self.p1.lines = [(t[:idx+1], list1[0][:idx+1], color1)]
        self.p2.lines = [(t[:idx+1], list1[1][:idx+1], color2)]
        self.p3.lines = [(t[:idx+1], list1[2][:idx+1], color3)]
        if idx > 0:
            self.p1.scatter = [(t[idx], list1[0][idx], color1)]
            self.p2.scatter = [(t[idx], list1[1][idx], color2)]
            self.p3.scatter = [(t[idx], list1[2][idx], color3)]
        self.p1.update()
        self.p2.update()
        self.p3.update()

class HUDInterface(QMainWindow):
    def __init__(self, csv_file):
        super().__init__()
        self.setWindowTitle("HUD Viewer")
        self.resize(1200, 800)

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
        
        self.views_canvas = ViewsCanvas(self)
        top_layout.addWidget(self.views_canvas)
        
        bottom_widget = QWidget()
        bottom_layout = QHBoxLayout(bottom_widget)
        
        self.text_canvas = TextCanvas(self)
        bottom_layout.addWidget(self.text_canvas, stretch=1)
        
        self.plot_tabs = QTabWidget()
        bottom_layout.addWidget(self.plot_tabs, stretch=3)

        self.ned_canvas = Plot3DWidget()
        self.plot_tabs.addTab(self.ned_canvas, "NED Frame")
        
        self.traj3d_canvas = Plot3DWidget(title="3D Trajectory")
        self.plot_tabs.addTab(self.traj3d_canvas, "3D Trajectory")
        
        self.traj2d_canvas = Plot2DWidget(title="2D Trajectory (East vs North)", xlabel="East", ylabel="North")
        self.plot_tabs.addTab(self.traj2d_canvas, "2D Trajectory")
        
        self.euler_canvas = TriplePlotCanvas("Roll (°)", "Pitch (°)", "Yaw (°)", "Time (s)", self)
        self.plot_tabs.addTab(self.euler_canvas, "Euler Angles")
        
        self.rates_canvas = TriplePlotCanvas("p (rad/s)", "q (rad/s)", "r (rad/s)", "Time (s)", self)
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
        self.dt_ms = 33

        self.current_idx = 0
        self.is_playing = False
        
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
        
        alpha = calculos.angle_of_attack(u, w)
        beta = calculos.sideslip_angle(u, v, w)
        climb = calculos.climb_angle(alpha, theta)
        
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
        self.text_canvas.set_text(text)

        self.views_canvas.yaw_view.set_angle(psi)
        self.views_canvas.pitch_view.set_angle(theta)
        self.views_canvas.roll_view.set_angle(phi)

        axis_length = max(10, np.linalg.norm(v_NED) * 1.5)
        
        lines3d = [
            ([(0,0,0), (0,axis_length,0)], 'b', 1),
            ([(0,0,0), (axis_length,0,0)], 'b', 1),
            ([(0,0,0), (0,0,-axis_length)], 'b', 1),
        ]
        texts3d = [
            (0, axis_length+1, 0, 'N', 'b'),
            (axis_length+1, 0, 0, 'E', 'b'),
            (0, 0, -axis_length-1, 'D', 'b')
        ]
        
        if np.linalg.norm(v_NED) > 0.1:
            lines3d.append(([(0,0,0), (v_NED[1], v_NED[0], -v_NED[2])], 'r', 2))
            texts3d.append((v_NED[1]+1, v_NED[0]+1, -v_NED[2]-1, 'V', 'k'))
            
        R_body_to_NED, _, _, _, _ = calculos.rotation_matrix(phi, theta, psi, v_body)
        
        scale = axis_length * 0.4
        pts_body = np.array([
            [ 1.0,  0.0,  0.0],
            [-1.0,  0.0,  0.0],
            [-0.2,  1.0,  0.0],
            [-0.2, -1.0,  0.0],
            [-1.0,  0.0, -0.4],
            [-1.0,  0.0,  0.0],
        ]).T
        
        pts_ned = R_body_to_NED @ pts_body * scale
        
        def map_ned(pts_ned, indices):
            return [(pts_ned[1,i], pts_ned[0,i], -pts_ned[2,i]) for i in indices]

        lines3d.append((map_ned(pts_ned, [0,1]), 'orange', 4))
        lines3d.append((map_ned(pts_ned, [2,3]), 'orange', 4))
        lines3d.append((map_ned(pts_ned, [4,5]), 'orange', 4))
        
        self.ned_canvas.axis_length = axis_length
        self.ned_canvas.lines = lines3d
        self.ned_canvas.texts = texts3d
        self.ned_canvas.update()

        path3d_x = self.Pe[:idx+1]
        path3d_y = self.Pn[:idx+1]
        path3d_z = -self.Pd[:idx+1]
        
        path3d_line = list(zip(path3d_x, path3d_y, path3d_z))
        
        self.traj3d_canvas.axis_length = max(10, np.max(np.abs(path3d_x)), np.max(np.abs(path3d_y)), np.max(np.abs(path3d_z)))
        self.traj3d_canvas.lines = [(path3d_line, 'blue', 2)]
        if idx > 0:
            self.traj3d_canvas.scatter = [(path3d_x[-1], path3d_y[-1], path3d_z[-1], 'red')]
        else:
            self.traj3d_canvas.scatter = []
        self.traj3d_canvas.update()

        self.traj2d_canvas.lines = [(self.Pe[:idx+1], self.Pn[:idx+1], 'green')]
        if idx > 0:
            self.traj2d_canvas.scatter = [(self.Pe[idx], self.Pn[idx], 'red')]
        else:
            self.traj2d_canvas.scatter = []
        self.traj2d_canvas.update()
# Aqui se cambian los colores de las graficas
        self.euler_canvas.set_data(self.time_list, [self.phi_list, self.theta_list, self.psi_list], idx, 'magenta', 'cyan', 'orange')
        self.rates_canvas.set_data(self.time_list, [self.p_list, self.q_list, self.r_list], idx, 'red', 'green', 'blue')

if __name__ == '__main__':
    app = QApplication(sys.argv)
    csv_file = "tello_imu_example.csv"
    window = HUDInterface(csv_file)
    window.show()
    sys.exit(app.exec())
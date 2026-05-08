import sys
import numpy as np
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QSlider, QLabel, QSplitter, QPushButton, QTabWidget, QComboBox)
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
        self.setMinimumSize(250, 180)

    def set_angle(self, angle):
        self.angle = angle
        self.update()
        
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform)
        w, h = self.width(), self.height()
        painter.fillRect(0, 0, w, h, QColor('#1e1e1e')) # Subtle gray for views
        
        painter.setPen(QPen(QColor('#c9d1d9'), 1))
        painter.setFont(QFont("Segoe UI", 10, QFont.Weight.Bold))
        painter.drawText(10, 25, f"{self.title} = {self.angle:.1f}\u00b0")
        
        self.draw_custom(painter, w/2, h/2, w, h)

class YawView(BaseView):
    def draw_custom(self, painter, cx, cy, w, h):
        NED_N_COLOR = '#58a6ff' # Blueish
        NED_E_COLOR = '#8b949e' # Gray
        BODY_X_COLOR = '#3fb950' # Green
        BODY_Y_COLOR = '#f85149' # Red
        scale = min(w, h) / 3.2
        painter.translate(cx, cy)
        draw_arrow(painter, 0, 0, 0, -scale, NED_N_COLOR, 'N', (-8, -15))
        draw_arrow(painter, 0, 0, scale, 0, NED_E_COLOR, 'E', (10, 5))
        
        painter.save()
        painter.rotate(self.angle)
        img_s = scale * 0.8
        if not self.img.isNull():
            painter.drawImage(QRectF(-img_s, -img_s, 2*img_s, 2*img_s), self.img)
        else:
            painter.drawRect(QRectF(-img_s/2, -img_s, img_s, 2*img_s)) # Placeholder
        draw_arrow(painter, 0, 0, 0, -scale, BODY_X_COLOR, 'x_b', (-10, -15))
        draw_arrow(painter, 0, 0, scale, 0, BODY_Y_COLOR, 'y_b', (10, 5))
        painter.restore()

class PitchView(BaseView):
    def draw_custom(self, painter, cx, cy, w, h):
        scale = min(w, h) / 3.2
        painter.translate(cx, cy)
        draw_arrow(painter, 0, 0, scale, 0, '#8b949e', 'X_ned', (10, 5))
        draw_arrow(painter, 0, 0, 0, scale, '#58a6ff', 'Z_ned', (-15, 20))
        
        painter.save()
        painter.rotate(-self.angle)
        img_s = scale * 0.8
        if not self.img.isNull():
            painter.drawImage(QRectF(-img_s, -img_s, 2*img_s, 2*img_s), self.img)
        draw_arrow(painter, 0, 0, scale, 0, '#3fb950', 'x_b', (10, 5))
        draw_arrow(painter, 0, 0, 0, scale, '#d29922', 'z_b', (-15, 20))
        painter.restore()

class RollView(BaseView):
    def draw_custom(self, painter, cx, cy, w, h):
        scale = min(w, h) / 3.2
        painter.translate(cx, cy)
        draw_arrow(painter, 0, 0, scale, 0, '#8b949e', 'Y_ned', (10, 5))
        draw_arrow(painter, 0, 0, 0, scale, '#58a6ff', 'Z_ned', (-15, 20))
        
        painter.save()
        painter.rotate(self.angle)
        img_s = scale * 0.8
        if not self.img.isNull():
            painter.drawImage(QRectF(-img_s, -img_s, 2*img_s, 2*img_s), self.img)
        draw_arrow(painter, 0, 0, scale, 0, '#f85149', 'y_b', (10, 5))
        draw_arrow(painter, 0, 0, 0, scale, '#d29922', 'z_b', (-15, 20))
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
        x_rot = x * np.cos(az) + y * np.sin(az)
        y_rot = -x * np.sin(az) + y * np.cos(az)
        xp = y_rot
        yp = -x_rot * np.sin(el) + z * np.cos(el)
        return cx + xp * scale, cy + yp * scale

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
        self.setMinimumHeight(180)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        
        BG_COLOR     = QColor('#0d1117')
        GRID_COLOR   = QColor('#21262d')
        AXIS_COLOR   = QColor('#8b949e')
        TEXT_COLOR   = QColor('#c9d1d9')
        
        painter.fillRect(0, 0, w, h, BG_COLOR)
        
        pad_l = max(65, int(w * 0.08))
        pad_r = max(20, int(w * 0.03))
        pad_t = max(40, int(h * 0.12))
        pad_b = max(50, int(h * 0.15))
        
        plot_w = w - pad_l - pad_r
        plot_h = h - pad_t - pad_b
        
        if plot_w <= 0 or plot_h <= 0: return

        x_min, x_max, y_min, y_max = float('inf'), float('-inf'), float('inf'), float('-inf')
        for x_data, y_data, _ in self.lines:
            if len(x_data) > 0:
                x_min = min(x_min, float(np.min(x_data)))
                x_max = max(x_max, float(np.max(x_data)))
                y_min = min(y_min, float(np.min(y_data)))
                y_max = max(y_max, float(np.max(y_data)))
                
        for x, y, _ in self.scatter:
            x_min, x_max = min(x_min, float(x)), max(x_max, float(x))
            y_min, y_max = min(y_min, float(y)), max(y_max, float(y))
            
        if x_max == float('inf'):
            x_min, x_max, y_min, y_max = 0, 1, 0, 1
            
        if x_max == x_min: x_max, x_min = x_min + 1, x_min - 1
        if y_max == y_min: y_max, y_min = y_min + 1, y_min - 1
        
        min_y_range = 10.0 if "°" in self.ylabel else 0.5
        if (y_max - y_min) < min_y_range:
            mid = (y_max + y_min) / 2
            y_max = mid + min_y_range / 2
            y_min = mid - min_y_range / 2

        dy = (y_max - y_min) * 0.30 
        y_min -= dy
        y_max += dy
        
        def to_px(x, y):
            px = pad_l + (x - x_min) / (x_max - x_min) * plot_w
            py = h - pad_b - (y - y_min) / (y_max - y_min) * plot_h
            return QPointF(px, py)

        painter.setPen(QPen(GRID_COLOR, 1, Qt.PenStyle.DashLine))
        
        base_font_size = max(7, min(10, int(h / 60)))
        font = QFont("Segoe UI", base_font_size)
        painter.setFont(font)
        
        for i in range(5):
            y_val = y_min + i * (y_max - y_min) / 4
            p1 = to_px(x_min, y_val)
            p2 = to_px(x_max, y_val)
            painter.drawLine(int(p1.x()), int(p1.y()), int(p2.x()), int(p1.y()))
            painter.setPen(QPen(AXIS_COLOR))
            painter.drawText(QRectF(0, p1.y() - 10, pad_l - 8, 20), Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter, f"{y_val:.1f}")
            painter.setPen(QPen(GRID_COLOR, 1, Qt.PenStyle.DashLine))

        painter.setPen(QPen(AXIS_COLOR, 2))
        painter.drawLine(pad_l, pad_t, pad_l, h-pad_b)
        painter.drawLine(pad_l, h-pad_b, w-pad_r, h-pad_b)

        for x_data, y_data, color in self.lines:
            if len(x_data) < 2: continue
            painter.setPen(QPen(QColor(color), 2, Qt.PenStyle.SolidLine))
            poly = QPolygonF()
            for x, y in zip(x_data, y_data):
                px_pt = to_px(x, y)
                poly.append(px_pt)
            painter.drawPolyline(poly)
            
        for x, y, color in self.scatter:
            painter.setPen(Qt.PenStyle.NoPen)
            painter.setBrush(QColor(color))
            pt = to_px(x, y)
            painter.drawEllipse(pt, 5, 5)
            
        title_font_size = max(9, min(12, int(h / 45)))
        painter.setPen(QPen(TEXT_COLOR))
        painter.setFont(QFont("Segoe UI", title_font_size, QFont.Weight.Bold))
        painter.drawText(QRectF(pad_l, 0, plot_w, pad_t), Qt.AlignmentFlag.AlignCenter, self.title)
        
        painter.setFont(QFont("Segoe UI", base_font_size))
        painter.save()
        painter.translate(15, pad_t + plot_h/2)
        painter.rotate(-90)
        painter.drawText(QRectF(-plot_h/2, -15, plot_h, 30), Qt.AlignmentFlag.AlignCenter, self.ylabel)
        painter.restore()

        painter.drawText(QRectF(pad_l, h - 30, plot_w, 25), Qt.AlignmentFlag.AlignCenter, self.xlabel)

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

class RCAM_HUD_Interface(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("RCAM HUD & Dynamic Modes Analyzer")
        self.resize(1200, 850)
        
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
        
        self.euler_canvas = TriplePlotCanvas("Roll (°)", "Pitch (°)", "Yaw (°)", "Time (s)", self)
        self.plot_tabs.addTab(self.euler_canvas, "Euler Angles")
        
        self.rates_canvas = TriplePlotCanvas("p (rad/s)", "q (rad/s)", "r (rad/s)", "Time (s)", self)
        self.plot_tabs.addTab(self.rates_canvas, "Angular Rates")
        
        # New RCAM-specific tabs
        self.vel_canvas = TriplePlotCanvas("u (m/s)", "v (m/s)", "w (m/s)", "Time (s)", self)
        self.plot_tabs.addTab(self.vel_canvas, "Body Velocities")
        
        self.aero_canvas = TriplePlotCanvas("Alpha (°)", "Beta (°)", "Va (m/s)", "Time (s)", self)
        self.plot_tabs.addTab(self.aero_canvas, "Aero Angles")

        top_widget.setMinimumHeight(280)
        splitter.addWidget(top_widget)
        splitter.addWidget(bottom_widget)
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 2)

        ctrl_layout = QHBoxLayout()
        
        self.combo_scenario = QComboBox()
        self.combo_scenario.addItems([
            "1: RCAM Nominal Simulation",
            "2: RCAM Aileron Deflection (+5°)",
            "3: RCAM Engine 1 Shutdown",
            "4: RCAM PSO Trim (78 m/s, NE)"
        ])
        self.combo_scenario.currentIndexChanged.connect(self.on_scenario_changed)
        ctrl_layout.addWidget(self.combo_scenario)

        self.play_button = QPushButton("Play")
        self.play_button.clicked.connect(self.toggle_play)
        ctrl_layout.addWidget(self.play_button)

        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setMinimum(0)
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
        
        self.setStyleSheet("""
            QMainWindow, QWidget#centralWidget {
                background-color: #0d1117;
                color: #c9d1d9;
            }
            QTabWidget::pane {
                border: 1px solid #30363d;
                background-color: #0d1117;
            }
            QTabBar::tab {
                background-color: #161b22;
                border: 1px solid #30363d;
                padding: 8px 15px;
                color: #8b949e;
                border-top-left-radius: 4px;
                border-top-right-radius: 4px;
            }
            QTabBar::tab:selected {
                background-color: #0d1117;
                color: #c9d1d9;
                border-bottom-color: #0d1117;
            }
            QPushButton {
                background-color: #21262d;
                border: 1px solid #30363d;
                border-radius: 4px;
                padding: 5px 15px;
                color: #c9d1d9;
            }
            QPushButton:hover {
                background-color: #30363d;
            }
            QLabel {
                color: #c9d1d9;
            }
            QSlider::groove:horizontal {
                border: 1px solid #30363d;
                height: 8px;
                background: #161b22;
                margin: 2px 0;
                border-radius: 4px;
            }
            QSlider::handle:horizontal {
                background: #58a6ff;
                border: 1px solid #30363d;
                width: 18px;
                margin: -5px 0;
                border-radius: 9px;
            }
        """)
        
        splitter.setSizes([400, 400])
        self.load_simulation_data(0)

    def on_scenario_changed(self, index):
        self.load_simulation_data(index)

    def load_simulation_data(self, scenario_index):
        if self.is_playing:
            self.toggle_play()

        # ComboBox items: 0->Nominal(1), 1->Aileron(2), 2->Engine(3), 3->PSO(4)
        rcam_scenario_id = scenario_index + 1
        t0 = 0.0
        (time_l, vNED_l, Pned_l, phi_l, theta_l, psi_l, p_l, q_l, r_l, u_l, v_l, w_l) = calculos.run_rcam_scenario(rcam_scenario_id)

        _z = np.array([0., 0., 0.])
        self.time_list  = [t0]   + time_l
        self.phi_list   = [0.0]  + phi_l
        self.theta_list = [0.0]  + theta_l
        self.psi_list   = [0.0]  + psi_l
        self.p_list     = [0.0]  + p_l
        self.q_list     = [0.0]  + q_l
        self.r_list     = [0.0]  + r_l
        self.v_NED_list = [_z.copy()] + vNED_l
        self.P_ned_list = [_z.copy()] + Pned_l
        
        u0, v0, w0 = (u_l[0], v_l[0], w_l[0]) if len(u_l) > 0 else (0.0, 0.0, 0.0)
        self.u_list     = [u0]  + u_l
        self.v_list     = [v0]  + v_l
        self.w_list     = [w0]  + w_l
        
        # Calculate full aero histories
        self.alpha_list = []
        self.beta_list = []
        self.Va_list = []
        
        for u, v, w in zip(self.u_list, self.v_list, self.w_list):
            self.alpha_list.append(calculos.angle_of_attack(u, w))
            self.beta_list.append(calculos.sideslip_angle(u, v, w))
            Va = np.sqrt(u**2 + v**2 + w**2)
            self.Va_list.append(Va)
            
        self.n_points = len(self.time_list)

        self.slider.blockSignals(True)
        self.slider.setMaximum(self.n_points - 1)
        self.slider.setValue(0)
        self.slider.blockSignals(False)

        self.current_idx = 0
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
            step_diff = self.time_list[min(self.current_idx+1, self.n_points-1)] - self.time_list[self.current_idx]
            if step_diff <= 0:
                step_size = 1
            else:
                step_size = max(1, int(self.dt_ms / 1000.0 / step_diff))
                step_size = min(step_size, 5)

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
        
        alpha = self.alpha_list[idx]
        beta = self.beta_list[idx]
        climb = calculos.climb_angle(v_NED=v_NED)
        
        Pn       = self.P_ned_list[idx][0]
        Pe       = self.P_ned_list[idx][1]
        altitude = self.P_ned_list[idx][2]   
        Pd       = -altitude                  

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
            ([(0,0,0), (axis_length,0,0)], 'b', 1),
            ([(0,0,0), (0,axis_length,0)], 'b', 1),
            ([(0,0,0), (0,0,axis_length)], 'b', 1),
        ]
        texts3d = [
            (axis_length+1, 0, 0, 'N', 'b'),
            (0, axis_length+1, 0, 'E', 'b'),
            (0, 0, axis_length+1, 'D', 'b')
        ]
        
        if np.linalg.norm(v_NED) > 0.1:
            lines3d.append(([(0,0,0), (v_NED[0], v_NED[1], v_NED[2])], 'r', 2))
            texts3d.append((v_NED[0]+1, v_NED[1]+1, v_NED[2]+1, 'V', 'k'))
            
        R_body_to_NED, _, _, _, _ = calculos.rotation_matrix(phi, theta, psi, v_body)
        
        body_scale = axis_length * 0.4
        pts_body = np.array([
            [ 1.0,  0.0,  0.0],
            [-1.0,  0.0,  0.0],
            [-0.2,  1.0,  0.0],
            [-0.2, -1.0,  0.0],
            [-1.0,  0.0, 0.4],
            [-1.0,  0.0,  0.0],
        ]).T
        
        pts_ned = R_body_to_NED @ pts_body * body_scale
        pts_ned[2, :] = -pts_ned[2, :]
        
        def map_ned(pts_ned, indices):
            return [(pts_ned[0,i], pts_ned[1,i], pts_ned[2,i]) for i in indices]

        lines3d.append((map_ned(pts_ned, [0,1]), 'orange', 4))
        lines3d.append((map_ned(pts_ned, [2,3]), 'orange', 4))
        lines3d.append((map_ned(pts_ned, [4,5]), 'orange', 4))
        
        self.ned_canvas.axis_length = axis_length
        self.ned_canvas.lines = lines3d
        self.ned_canvas.texts = texts3d
        self.ned_canvas.update()

        # Update Triple Plots
        t_arr = self.time_list
        
        self.euler_canvas.set_data(t_arr, [self.phi_list, self.theta_list, self.psi_list], idx, '#58a6ff', '#3fb950', '#d29922')
        self.rates_canvas.set_data(t_arr, [self.p_list, self.q_list, self.r_list], idx, '#f85149', '#a371f7', '#3fb950')
        
        # New RCAM plots
        self.vel_canvas.set_data(t_arr, [self.u_list, self.v_list, self.w_list], idx, '#3fb950', '#58a6ff', '#f85149')
        self.aero_canvas.set_data(t_arr, [self.alpha_list, self.beta_list, self.Va_list], idx, '#d29922', '#a371f7', '#58a6ff')



if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RCAM_HUD_Interface()
    window.show()
    sys.exit(app.exec())

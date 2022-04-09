import matplotlib.pyplot as plt
import numpy as np
import sys
from PyQt5.QtWidgets import QDialog, QApplication, QPushButton, QVBoxLayout
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from threading import Timer
import warnings
import matplotlib.cbook
warnings.filterwarnings("ignore",category=matplotlib.cbook.mplDeprecation)

class BikeModel():
    def __init__(self, Ts):
        
        # Bike State
        self.x_pos = 0.
        self.y_pos = 0.
        self.phi = 0.1
        self.psi = 0.
        self.d = 0.
        self.theta = 0.
        self.delta = 0. #TODO CORRECTED GEOMETRY
        self.valid = True

        # Bike State Derivatives
        self.phi_dot = 0.
        self.phi_ddot = 0.
        self.psi_dot = 0.
        self.d_dot = 0.

        # Bike Control Input
        self.delta_dot = 0
        self.v = 0.6

        # Config Params
        self.h = 0.015 # Height
        self.l_f = 0.02 # Front Axle CG
        self.l_r = 0.01 # Real Axle CG
        self.wd = 0.01 # Wheel Diameter
        self.w = self.l_f + self.l_r # Wheelbase
        self.steer_max = np.pi / 3. # Maximum Steering Angle (Symmetrical)
        self.steer_rate_max = 10.
        self.g = 9.81 # Gravity
        self.dt = Ts

        # Controller Params
        self.K1 = 80.
        self.K2 = 20.
        self.K3 = -20.
        self.K4 = 0.9
        self.K5 = 0.2
        self.K6 = 0.1
        return

    def step(self):
        self.navigation_controller()
        self.balance_controller()

        #Position Update
        #self.x_pos += self.v * np.cos(self.psi) * self.dt
        #self.y_pos += self.v * np.sin(self.psi) * self.dt

        #Heading Update
        self.psi_dot = self.v / (self.w * np.tan(self.delta) * np.cos(self.phi))
        self.psi += self.psi_dot * self.dt
        
        #Balance Update
        self.phi_ddot = (self.g / self.h) * self.phi - (self.v**2 / (self.h * self.w)) * self.delta - (self.l_r * self.v / (self.h * self.w)) * self.delta_dot
        self.phi_dot += self.phi_ddot * self.dt  
        self.phi = min(np.pi/2., max(-np.pi/2., self.phi + self.phi_dot * self.dt))
        self.delta = min(self.steer_max, max(-self.steer_max, self.delta + self.delta_dot * self.dt))

        #Path Update
        #self.theta = 
        self.d_dot = self.v * np.sin(self.theta)
        self.d += self.d_dot * self.dt 

        #print(self.phi, self.phi_dot, self.phi_ddot, self.delta, self.delta_dot)
        #print("1: ", (self.g / self.h) * self.phi, " 2: ", - (self.v**2 / (self.h * self.w)) * self.delta, " 3: ", - (self.l_r * self.v / (self.h * self.w)) * self.delta_dot)
        #Check Toppling Condition
        if abs(self.phi) > np.pi/6:
            self.valid = False
            print("Simulation Stopped, Bike Toppling")
        return

    def balance_controller(self):
        self.delta_dot = max(-self.steer_rate_max, min(self.steer_rate_max, self.K1 * self.phi + self.K2 * self.phi_dot + self.K3 * (self.delta - self.delta_target)))
        return
    
    def navigation_controller(self):
        self.delta_target = self.K4 * self.theta + self.K5 * self.d + self.K6 * self.d_dot
        return
    
    def desired_path(self):
        return

    def get_psi_des(self):
        return 0.1 # Circle

    def get_d(self):
        return

class SimulatorNode():
    def __init__(self, Ts):
        self.time_step = Ts
        self.model = BikeModel(self.time_step) 
        self.running = True
        return
    
    def step(self):
        if self.running != False:
            self.model.step()
        return

class VisualizerNode():
    def __init__(self):
        self.app = QApplication(sys.argv)
        visualizer_window = VisualizerWindow(sys.argv)
        visualizer_window.show()
        sys.exit(self.app.exec_())
        return

class VisualizerWindow(QDialog):
    def __init__(self, parent = None):
        super(VisualizerWindow, self).__init__(None)

        # START TIMER      
        self.running = False
        self.time_step = 0.01
        self.time = 0.
        self.timer = Timer(interval=self.time_step, function=self.step)
        self.timer.start()
        self.visualization = sys.argv[1] != "False"

        # BIKE MODEL
        self.bike_sim = SimulatorNode(self.time_step)

        # MPL FIGURE
        self.angle_figure = plt.figure()
        self.position_figure = plt.figure()
        self.angle_canvas = FigureCanvas(self.angle_figure)
        self.position_canvas = FigureCanvas(self.position_figure)
        self.angle_toolbar = NavigationToolbar(self.angle_canvas, self)
        self.position_toolbar = NavigationToolbar(self.position_canvas, self)

        # START BUTTON
        self.start_button = QPushButton('START/RESTART')
        self.start_button.clicked.connect(self.start_clicked)

        # STOP BUTTON
        self.stop_button = QPushButton('STOP/PAUSE')
        self.stop_button.clicked.connect(self.stop_clicked)

        # LAYOUT
        layout = QVBoxLayout()
        layout.addWidget(self.angle_toolbar)
        layout.addWidget(self.angle_canvas)
        layout.addWidget(self.position_toolbar)
        layout.addWidget(self.position_canvas)
        layout.addWidget(self.start_button)
        layout.addWidget(self.stop_button)
        self.setLayout(layout)
        return

    def start_clicked(self):
        print("Starting Simulation")
        self.running = True
        return

    def stop_clicked(self):
        print("Stopping Simulation")   
        self.running = False
        return

    def angle_plot(self):
        phi_ax = self.angle_figure.add_subplot(121)
        phi_ax.cla()
        phi_ax.plot([-3.*self.bike_sim.model.h/2., 3.*self.bike_sim.model.h/2.], [0., 0.], '--', c = 'k', alpha = 0.2)
        phi_ax.plot([0., 0.], [-self.bike_sim.model.h/2., 3.*self.bike_sim.model.h/2.], '--', c = 'k', alpha = 0.2)
        phi_ax.plot([0., -self.bike_sim.model.h * np.sin(self.bike_sim.model.phi)], 
                    [0., self.bike_sim.model.h * np.cos(self.bike_sim.model.phi)], c = 'b')
        phi_ax.set_title('Lean Angle (Front View)')
        phi_ax.set_xlim(-3.*self.bike_sim.model.h/2., 3.*self.bike_sim.model.h/2.)
        phi_ax.set_ylim(-self.bike_sim.model.h/2., 3. * self.bike_sim.model.h/2.)
        phi_ax.set_aspect('equal')

        delta_ax = self.angle_figure.add_subplot(122)
        delta_ax.cla() 
        delta_ax.plot([-self.bike_sim.model.w, self.bike_sim.model.w], [0., 0.], '--', c = 'k', alpha = 0.2)
        delta_ax.plot([0., 0.], [- self.bike_sim.model.w, self.bike_sim.model.w], '--', c = 'k', alpha = 0.2)
        delta_ax.plot([0., 0., 0.], [-(self.bike_sim.model.l_r), 0., (self.bike_sim.model.l_f - self.bike_sim.model.wd)], c = 'b')
        delta_ax.plot([0., -self.bike_sim.model.wd * np.sin(self.bike_sim.model.delta)], [
                     (self.bike_sim.model.l_f - self.bike_sim.model.wd), (self.bike_sim.model.l_f - self.bike_sim.model.wd) + self.bike_sim.model.wd * np.cos(self.bike_sim.model.delta)], c = 'r')
        delta_ax.set_title('Steer Angle (Top View)')
        delta_ax.set_xlim(-self.bike_sim.model.w, self.bike_sim.model.w)
        delta_ax.set_ylim(-self.bike_sim.model.w, self.bike_sim.model.w)
        delta_ax.set_aspect('equal')

        self.angle_figure.tight_layout()
        self.angle_canvas.draw()
        return

    def position_plot(self):
        t = np.linspace(0, 2*np.pi, 100)
        x_des = np.sin(t)
        y_des = np.sin(t) * np.cos(t)
        position_ax = self.position_figure.add_subplot()
        position_ax.cla()
        position_ax.plot(x_des, y_des, c='r')
        position_ax.scatter(self.bike_sim.model.x_pos, self.bike_sim.model.y_pos, c = 'b')
        position_ax.set_xlabel('X-Position')
        position_ax.set_ylabel('Y-Position')
        self.position_figure.tight_layout()
        self.position_canvas.draw()
        return

    def step(self):
        self.timer = Timer(interval=self.time_step, function=self.step)
        if self.running and self.bike_sim.model.valid:
            self.time += self.time_step 
            print("Simulation Time: ", self.time)
            self.bike_sim.step()
            if self.visualization:
                self.angle_plot()
                self.position_plot()
        if self.isVisible() or self.time == 0:    
            self.timer.start()
        return 

def main():
    print("Starting GUI")
    VisualizerNode()

if __name__ == '__main__':
    main()


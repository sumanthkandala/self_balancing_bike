from simulator import SimulatorNode
from simulator import Path
import matplotlib.pyplot as plt
import numpy as np
import sys
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QDialog, QApplication, QPushButton, QVBoxLayout, QHBoxLayout, QLabel
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.animation import FuncAnimation
from threading import Timer
import keyboard
import warnings
import matplotlib.cbook
import argparse
matplotlib.use('GTK3Agg') 
warnings.filterwarnings("ignore",category=matplotlib.cbook.mplDeprecation)

controller = "nav"
mode = "auto"

class PositionFigureCanvas(FigureCanvas):
    def __init__(self, fig, bike_sim):
        FigureCanvas.__init__(self, fig)
        self.fig = fig
        self.bike_sim = bike_sim
        self.animate()
        return
    
    def animate(self):
        self.anim = FuncAnimation(self.fig, self.animator, interval=0.1, repeat=False)
        self.draw()
        return

    def animator(self, i):
        x_pos = []
        y_pos = []
        with open("result.txt", "r") as result_file:
            for line in result_file:
                words = line.split()
                x_pos.append(float(words[0]))
                y_pos.append(float(words[1]))
        result_file.close()

        x_des, y_des = Path().getPath()
        position_ax = self.fig.add_subplot()
        position_ax.cla()
        if mode == "auto":
            position_ax.plot(x_des, y_des, c = 'r')
            position_ax.scatter(x_pos, y_pos, s = 0.1, c = 'g', label = 'previous path')
            position_ax.set_aspect('equal')
        elif mode == "manual":
            position_ax.scatter(x_pos[-500:], y_pos[-500:], s = 0.1, c = 'g', label = 'previous path')
            position_ax.set_aspect('equal')
        position_ax.plot([self.bike_sim.model.x_pos - self.bike_sim.model.l_r * np.cos(self.bike_sim.model.psi), 
                        self.bike_sim.model.x_pos + self.bike_sim.model.l_f * np.cos(self.bike_sim.model.psi)], 
                        [self.bike_sim.model.y_pos - self.bike_sim.model.l_r * np.sin(self.bike_sim.model.psi), 
                        self.bike_sim.model.y_pos + self.bike_sim.model.l_f * np.sin(self.bike_sim.model.psi)], c = 'b', linewidth = 3, label = 'bike')
        position_ax.set_xlabel('X-Position (m)')
        position_ax.set_ylabel('Y-Position (m)')
        position_ax.legend()
        self.fig.tight_layout()
        return

class AngleFigureCanvas(FigureCanvas):
    def __init__(self, fig, bike_sim):
        FigureCanvas.__init__(self, fig)
        self.fig = fig
        self.bike_sim = bike_sim
        self.animate()
        return
    
    def animate(self):
        self.anim = FuncAnimation(self.fig, self.animator, interval=0.1, repeat=False)
        self.draw()
        return

    def animator(self, i):
        phi_ax = self.fig.add_subplot(211)
        phi_ax.cla()
        phi_ax.plot([-3.*self.bike_sim.model.h/2., 3.*self.bike_sim.model.h/2.], [0., 0.], '--', c = 'k', alpha = 0.2)
        phi_ax.plot([0., 0.], [-self.bike_sim.model.h/2., 3.*self.bike_sim.model.h/2.], '--', c = 'k', alpha = 0.2)
        phi_ax.plot([0., -self.bike_sim.model.h * np.sin(self.bike_sim.model.phi)], 
                    [0., self.bike_sim.model.h * np.cos(self.bike_sim.model.phi)], c = 'b', label = 'chassis')
        phi_ax.set_title('Lean Angle (Front View)')
        phi_ax.set_xlim(-3.*self.bike_sim.model.h/2., 3.*self.bike_sim.model.h/2.)
        phi_ax.set_ylim(-self.bike_sim.model.h/2., 3.*self.bike_sim.model.h/2.)
        phi_ax.set_aspect('equal')
        phi_ax.legend()

        delta_ax = self.fig.add_subplot(212)
        delta_ax.cla() 
        delta_ax.plot([-self.bike_sim.model.w, self.bike_sim.model.w], [0., 0.], '--', c = 'k', alpha = 0.2)
        delta_ax.plot([0., 0.], [- self.bike_sim.model.w, self.bike_sim.model.w], '--', c = 'k', alpha = 0.2)
        delta_ax.plot([0., 0., 0.], [-(self.bike_sim.model.l_r), 0., (self.bike_sim.model.l_f - self.bike_sim.model.wd)], c = 'b', label = 'chassis')
        delta_ax.plot([0., -self.bike_sim.model.wd * np.sin(self.bike_sim.model.delta)], [
                     (self.bike_sim.model.l_f - self.bike_sim.model.wd), (self.bike_sim.model.l_f - self.bike_sim.model.wd) + self.bike_sim.model.wd * np.cos(self.bike_sim.model.delta)], c = 'r', label = 'steering wheel')
        delta_ax.set_title('Steer Angle (Top View)')
        delta_ax.set_xlim(-self.bike_sim.model.w, self.bike_sim.model.w)
        delta_ax.set_ylim(-self.bike_sim.model.w, self.bike_sim.model.w)
        delta_ax.set_aspect('equal')
        delta_ax.legend()

        self.fig.tight_layout()
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
        global controller, mode, visualizer
        self.running = False
        self.time_step = 0.01
        self.time = 0.
        self.timer = Timer(interval=self.time_step, function=self.step)
        self.timer.start()

        # BIKE MODEL & PARAMS
        self.key_input = 0.
        self.key_time = 0.
        self.command = "None"
        self.bike_sim = SimulatorNode(self.time_step, controller, mode)

        # MPL FIGURE
        self.angle_figure = plt.figure()
        self.position_figure = plt.figure()
        self.angle_canvas = AngleFigureCanvas(self.angle_figure, self.bike_sim)
        self.position_canvas = PositionFigureCanvas(self.position_figure, self.bike_sim)

        # TEXT BOX
        self.text_box = QLabel(self)
        self.text_box.setFixedHeight(50)
        self.text_box.setFont(QFont('Arial', 30))
        self.text_box.setAlignment(Qt.AlignCenter)
        self.text_box.setText("Simulation Time: 0.0" + "\t Mode: " + mode.upper() + "\t Controller: " + controller.upper() + "\t Command: NONE")

        # START BUTTON
        self.start_button = QPushButton('START/RESTART')
        self.start_button.clicked.connect(self.start_clicked)

        # STOP BUTTON
        self.stop_button = QPushButton('STOP/PAUSE')
        self.stop_button.clicked.connect(self.stop_clicked)

        # LAYOUT
        self.grid_layout = QVBoxLayout()
        self.plot_layout = QHBoxLayout()
        self.plot_layout.addWidget(self.angle_canvas)
        self.plot_layout.addWidget(self.position_canvas)
        self.grid_layout.addLayout(self.plot_layout)
        self.grid_layout.addWidget(self.text_box)
        self.grid_layout.addWidget(self.start_button)
        self.grid_layout.addWidget(self.stop_button)
        self.setLayout(self.grid_layout)
        self.showMaximized()
        open('result.txt', 'w').close()
        return

    def start_clicked(self):
        print("Starting Simulation")
        self.running = True
        return

    def stop_clicked(self):
        print("Stopping Simulation")   
        self.running = False
        return
    
    def left_key(self, event):
        self.key_input += 1
        self.key_time = self.time
        self.command = "left"
        return
    
    def right_key(self, event):
        self.key_input -= 1
        self.key_time = self.time
        self.command = "right"
        return

    def step(self):
        self.timer = Timer(interval=self.time_step, function=self.step)
        if self.running and self.bike_sim.model.valid:
            self.time += self.time_step 
            self.text_box.setText("Simulation Time: " + str(round(self.time, 2)) + "\t Mode: " + mode.upper() + "\t Controller: " + controller.upper() + "\t Command: " + self.command.upper())
            if mode == "manual":
                keyboard.on_press_key("left arrow", self.left_key)
                keyboard.on_press_key("right arrow", self.right_key)
                if self.time - self.key_time > 10 * self.time_step:
                    self.key_input = 0
                    self.command = "None"
            self.bike_sim.step(self.key_input)
            with open("result.txt", 'a') as result_file:
                result_file.write(''.join([str(self.bike_sim.model.x_pos),  " ", str(self.bike_sim.model.y_pos), " ", str(self.bike_sim.model.phi), " ", str(self.bike_sim.model.delta), "\n"]))
            result_file.close()
        if self.isVisible() or self.time == 0:    
            self.timer.start()
        return 

def main():
    parser = argparse.ArgumentParser(description = "Description for my parser")
    parser.add_argument("-c", "--controller", help = "Specify Controller (pid, nav)", required = False, default = "model")
    parser.add_argument("-m", "--mode", help = "Specify Control Mode (auto, manual)", required = False, default = "auto")
    argument = parser.parse_args()

    global controller, mode

    if argument.mode:
        mode = argument.mode
    if argument.controller:
        controller = argument.controller
        if controller == "pid" and mode == "auto":
            print("PID only supports manual mode, switching to manual mode.")
            mode = "manual"

    VisualizerNode()
    return

if __name__ == '__main__':
    main()


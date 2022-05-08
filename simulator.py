import numpy as np
import math

r = 5.

class Path():
    def __init__(self):
        t = np.linspace(0, 2*np.pi, 100)
        self.x_des = np.sin(t) * r
        self.y_des = r + np.cos(t) * r
        return 

    def getPath(self):
        return self.x_des, self.y_des

class BikeModel():
    def __init__(self, Ts, controller, mode):
        
        # Bike State
        self.x_pos = 0.
        self.y_pos = 0.
        self.phi = 0.
        self.psi = 0.0
        self.d = 0.
        self.theta = 0.
        self.psi_des = 0.
        self.delta = 0.
        self.valid = True
        self.itr = 0

        # Bike State Derivatives
        self.phi_dot = 0.
        self.phi_ddot = 0.
        self.psi_dot = 0.
        self.d_dot = 0.
        self.v_dot = 0.

        # Bike Control Input
        self.delta_dot = 0.
        self.v = 0.5
        self.key_input = 0.

        # Config Params
        self.h = 0.15 # Height
        self.l_f = 0.2 # Front Axle CG
        self.l_r = 0.1 # Real Axle CG
        self.wd = 0.1 # Wheel Diameter
        self.w = self.l_f + self.l_r # Wheelbase
        self.steer_max = np.pi / 3. # Maximum Steering Angle (Symmetrical)
        self.steer_rate_max = 100.
        self.g = 9.81 # Gravity
        self.dt = Ts

        # Model Controller Params
        self.K1 = 80.
        self.K2 = 20.
        self.K3 = -4.
        self.K4 = 0.8
        self.K5 = 0.4
        self.K6 = 0.05

        # PID Controller Params
        self.Kp = 40.
        self.Ki = 10.
        self.Kd = 10.
        self.err = 0.

        # Control Methods
        self.controller = controller
        self.mode = mode
        return

    def step(self, key_input):
        self.itr += 1

        if self.controller == "nav":
            self.navigation_controller()
            self.balance_controller()
        elif self.controller == "pid":
            self.pid_controller()
        else:
            print("Incorrect Controller: Options are 'nav' and 'pid', ")

        #Position Update
        self.x_pos += self.v * np.cos(self.psi) * self.dt
        self.y_pos += self.v * np.sin(self.psi) * self.dt

        #Heading Update
        self.psi_dot = self.v * np.tan(self.delta) / (self.w * np.cos(self.phi))
        self.psi = math.atan2(np.sin(self.psi + self.psi_dot * self.dt), np.cos(self.psi + self.psi_dot * self.dt)) 

        #Balance Update
        self.phi_ddot = (self.g / self.h) * np.sin(self.phi) - np.tan(self.delta) * self.v**2 / (self.h * self.w) - self.l_r * self.v * self.delta_dot / (self.h * self.w * np.cos(self.delta)**2) - self.l_r * self.v_dot * np.tan(self.delta) / (self.h * self.w) + self.v**2 * np.tan(self.delta)**2 * np.tan(self.theta) / (self.w**2) - self.l_r * self.phi_dot * np.tan(self.delta) * np.tan(self.phi) / (self.h * self.w)
        self.phi_dot += self.phi_ddot * self.dt #REPLACE WITH GYRO FEEDBACK
        self.phi = self.phi + self.phi_dot * self.dt
        self.delta = min(self.steer_max, max(-self.steer_max, self.delta + self.delta_dot * self.dt))

        #Path Update
        if self.mode == "auto" and self.controller == 'nav':
            self.path_updater()
        elif self.mode == "manual" or self.controller == 'pid':
            self.key_input = key_input
            self.key_updater()
        else:
            print("Incorrect Mode: Options are 'auto' or 'manual'")

        #Tracking Update
        self.theta = np.mod(self.psi - self.psi_des + np.pi, 2*np.pi) - np.pi
        self.d_dot = self.v * np.sin(self.theta)
        self.d += self.d_dot * self.dt 

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

    def pid_controller(self):
        self.err = min(np.pi/2., max(-np.pi/2., self.err + self.phi * self.dt))
        self.delta_dot = max(-self.steer_rate_max, min(self.steer_rate_max, self.Kp * self.phi + self.Ki * self.err + self.Kd * self.phi_dot))
        return

    def path_updater(self):
        x_path, y_path = Path().getPath()
        dist = (x_path - self.x_pos * np.ones(len(x_path)))**2 + (y_path - self.y_pos * np.ones(len(y_path)))**2
        x_next = x_path[(np.argmin(dist) + 1) % len(x_path)]
        y_next = y_path[(np.argmin(dist) + 1) % len(y_path)]
        x_prev = x_path[(np.argmin(dist) - 1) % len(x_path)]
        y_prev = y_path[(np.argmin(dist) - 1) % len(y_path)]
        self.psi_des = np.mod(math.atan2((y_next - y_prev), (x_next - x_prev)) + 2*np.pi, 2*np.pi) - np.pi
        return

    def key_updater(self):
        self.psi_des = self.key_input * 0.001
        return

class SimulatorNode():
    def __init__(self, Ts, controller, mode):
        self.time_step = Ts
        self.model = BikeModel(self.time_step, controller, mode) 
        self.running = True
        return
    
    def step(self, key_input):
        if self.running != False:
            self.model.step(key_input)
        return
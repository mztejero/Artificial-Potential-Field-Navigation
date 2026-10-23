import numpy as np
import matplotlib.pyplot as plt
from apf_params import Dimensions, FrameParams, InitialConditions,PIDParams, APFParams, Params

class APF:
    def __init__(self):
        self.params = Params(
            dimensions=Dimensions(),
            frame=FrameParams(),
            init=InitialConditions(),
            pid=PIDParams(),
            apf=APFParams()
        )

        self.x = self.params.init.x
        self.y = self.params.init.y
        self.yaw = self.params.init.yaw

        self.target_x = 0.0
        self.target_y = 0.0

        self.v_long = self.params.init.v_long

        self.vr = self.params.init.vr
        self.vl = self.params.init.vl

        self.sum_error_steer = 0.0 
        self.prev_error_steer = 0.0

        self.sum_error_speed = 0.0
        self.prev_error_speed = 0.0

        self.fig, self.ax = plt.subplots()
        self.ax.set_aspect('equal')
        self.ax.set_xlim(self.params.frame.xmin - 1, self.params.frame.xmax + 1)
        self.ax.set_ylim(self.params.frame.ymin - 1, self.params.frame.ymax + 1)
        self.ax.grid(True)

        self.carx, self.cary = self.create_car(self.x, self.y, self.yaw)
        self.car_line = self.draw_car(self.carx, self.cary)

        self.obstacles_x = np.array([])
        self.obstacles_y = np.array([])

        self.create_random_obstacle_points(21)
        self.plot_obstacles()

        self.create_random_target()
        self.target_line = self.draw_target()

        self.X = 0.0
        self.X = 0.0
        self.U_obstacle = 0.0
        self.V_obstacle = 0.0

        self.create_apf()
        self.apf_plot = self.draw_apf()
        self.plot_apf()

    def R_yaw(self, yaw):
        return np.array([[np.cos(yaw), -np.sin(yaw), 0],
                         [np.sin(yaw), np.cos(yaw), 0],
                         [0, 0, 1]])

    def create_car(self, x, y, yaw):

        pos = np.array([x, y, 0])

        lf = self.params.dimensions.L / 2      # distance from COM to front
        lr = self.params.dimensions.L / 2      # distance from COM to rear
        wl = self.params.dimensions.W / 2      # distance from COM to left
        wr = self.params.dimensions.W / 2      # distance from COM to right


        rot = self.R_yaw(yaw)

        fr = rot @ np.array([lf, -wr, 0]) + pos
        fl = rot @ np.array([lf, wl, 0]) + pos
        rl = rot @ np.array([-lr, wl, 0]) + pos
        rr = rot @ np.array([-lr, -wr, 0]) + pos

        points = np.array([fr, fl, rl, rr, fr])

        carx = points[:, 0]
        cary = points[:, 1]

        return carx, cary
    
    def create_random_target(self):
        xmin = self.params.frame.xmin
        xmax = self.params.frame.xmax
        ymin = self.params.frame.ymin
        ymax = self.params.frame.ymax

        x, y = np.random.uniform(xmin, xmax), np.random.uniform(ymin, ymax)
        i = 0
        while i < len(self.obstacles_x):
            if (x > self.obstacles_x[i] - self.params.apf.d and x < self.obstacles_x[i] + self.params.apf.d and y > self.obstacles_y[i] - self.params.apf.d and y < self.obstacles_y[i] + self.params.apf.d):
                x, y = np.random.uniform(xmin, xmax), np.random.uniform(ymin, ymax)
                i = 0
            else:
                i += 1

        self.target_x = x
        self.target_y = y
    
    def draw_car(self, carx, cary):
        line, = self.ax.plot(carx, cary, color='blue')
        return line
    
    def draw_target(self):
        line = self.ax.scatter(self.target_x, self.target_y, color='green', s=50)
        return line
    
    def plot_car(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw
        
        self.car_line.remove()
        self.carx, self.cary = self.create_car(self.x, self.y, self.yaw)
        self.car_line = self.draw_car(self.carx, self.cary)

    def plot_random_target(self):
        self.target_line.remove()
        self.create_random_target()
        self.target_line = self.draw_target()

    def target_found(self):
        if self.target_x > min(self.carx) and self.target_x < max(self.carx) and self.target_y > min(self.cary) and self.target_y < max(self.cary):
            return True
        else:
            return False
        
    def plot_obstacles(self):
        for i in range(len(self.obstacles_x)):
            self.ax.scatter(self.obstacles_x[i], self.obstacles_y[i], color='red', s=50)

    def create_random_obstacle_points(self, num):
        xmin = self.params.frame.xmin
        xmax = self.params.frame.xmax
        ymin = self.params.frame.ymin
        ymax = self.params.frame.ymax

        for i in range(num):
            x, y = np.random.uniform(xmin, xmax), np.random.uniform(ymin, ymax)

            self.obstacles_x = np.append(self.obstacles_x, x)
            self.obstacles_y = np.append(self.obstacles_y, y)
    
    def apf_vector(self, x, y):

        d_target_x = self.target_x - x
        d_target_y = self.target_y - y
        d_target = np.sqrt(d_target_x ** 2 + d_target_y ** 2 + 1)

        U_target = 2.5 * d_target_x / d_target
        V_target = 2.5 * d_target_y / d_target

        U_obstacle = 0.0
        V_obstacle = 0.0
        
        for i in range(len(self.obstacles_x)):
            d_obstacle_x = x - self.obstacles_x[i]
            d_obstacle_y = y - self.obstacles_y[i]
            d_obstacle = np.sqrt(d_obstacle_x ** 2 + d_obstacle_y ** 2 + 1)

            U_obstacle += 10 * np.exp(-0.4 * d_obstacle) * d_obstacle_x / d_obstacle
            V_obstacle += 10 * np.exp(-0.4 * d_obstacle) * d_obstacle_y / d_obstacle

        U_border = 3 / (x - self.params.frame.xmin) ** 2 - 1 / (x - self.params.frame.xmax) ** 2
        V_border = 3 / (y - self.params.frame.ymin) ** 2 - 1 / (y - self.params.frame.ymax) ** 2

        U = U_target + U_obstacle + U_border
        V = V_target + V_obstacle + V_border

        return U, V
    
    def create_apf(self):
        x = np.linspace(self.params.frame.xmin, self.params.frame.xmax, 50)
        y = np.linspace(self.params.frame.ymin, self.params.frame.ymax, 50)

        self.X, self.Y = np.meshgrid(x, y)

        self.U, self.V = self.apf_vector(self.X, self.Y)

    def draw_apf(self):
        self.create_apf()
        apf_plot = self.ax.quiver(self.X, self.Y, self.U, self.V, color='orange',
          angles='xy', scale_units='xy', scale=3)
        return apf_plot
    
    def plot_apf(self):
        self.apf_plot.remove()
        self.apf_plot = self.draw_apf()

    def global_kinematic_model(self, x, y, yaw, vr, vl):

        W = self.params.dimensions.W

        v_long = (vr + vl) * 0.5
        yaw_dot = (vr - vl) / W
        x_dot = v_long * np.cos(yaw)
        y_dot = v_long * np.sin(yaw)

        x += x_dot * self.params.dt
        y += y_dot * self.params.dt
        yaw += yaw_dot * self.params.dt

        return v_long, x, y, yaw
    
    def speed_pid(self, error):
        dt = self.params.dt

        Kp = self.params.pid.Kp_speed
        Ki = self.params.pid.Ki_speed
        Kd = self.params.pid.Kd_speed

        u = Kp*error + Ki*self.sum_error_speed*dt + Kd*(error - self.prev_error_speed)/dt
        self.sum_error_speed += error
        self.prev_error_speed = error
        return u

    def steer_pid(self, error):
        dt = self.params.dt

        Kp = self.params.pid.Kp_steer
        Ki = self.params.pid.Ki_steer
        Kd = self.params.pid.Kd_steer

        u = Kp*error + Ki*self.sum_error_steer*dt + Kd*(error - self.prev_error_steer)/dt
        self.sum_error_steer += error
        self.prev_error_steer = error
        return u
    
    def step_pid(self, v, x, y, yaw, v_ref):

        W = self.params.dimensions.W

        u_apf, v_apf = self.apf_vector(x, y)
        yaw_ref = np.arctan2(v_apf, u_apf)
        yaw_error = yaw_ref - yaw
        yaw_error = (yaw_error + np.pi) % (2 * np.pi) - np.pi

        K = self.params.pid.K_scalling
        scaling_factor = np.exp(-K * abs(yaw_error)) # * np.sqrt(u_apf ** 2 + v_apf ** 2)
        v_ref *= scaling_factor
        v_error = v_ref - v

        u_speed = v_ref # self.speed_pid(v_error)
        u_steer = self.steer_pid(yaw_error)

        vr = u_speed + 0.5 * W * u_steer
        vl = u_speed - 0.5 * W * u_steer

        return vr, vl
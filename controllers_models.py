import math
import numpy as np

def angle_confine(ang):
    ang %= 2*math.pi
    if ang > math.pi:
        ang -= 2*math.pi
    return ang

class Integrator:
    def __init__(self, Ts, initial_value:float=0, is_angle:bool=False):
        self.Ts = Ts
        self.integral = initial_value
        self.is_angle = is_angle

    def calc(self, val):
        self.integral += val*self.Ts
        if self.is_angle:
            self.integral = angle_confine(self.integral)
        return self.integral

    def get_curr(self):
        return self.integral
    
class PIDController:
    def __init__(self, Kp, Ki, Kd, Ts, initial_e_prev):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.Ts = Ts
        self.e_prev = initial_e_prev
        self.e_integral = Integrator(Ts)

    def __call__(self, e):
        P = self.Kp*e
        I = self.Ki*self.e_integral.calc(e)
        D = self.Kd*(e - self.e_prev)/self.Ts
        self.e_prev = e
        return P + I + D

class KanayamaController:
    def __init__(self, Kx, Ky, Kw, max_vel, min_vel, max_ang_vel, min_ang_vel):
        self.Kx = Kx
        self.Ky = Ky
        self.Kw = Kw
        self.max_vel = max_vel
        self.min_vel = min_vel
        self.max_ang_vel = max_ang_vel
        self.min_ang_vel = min_ang_vel

    def __call__(self, x_ref, y_ref, theta_ref, x, y, theta, v_ref, w_ref):
        x_sub = x_ref - x
        y_sub = y_ref - y
        x_err = math.cos(theta)*x_sub + math.sin(theta)*y_sub
        y_err = -math.sin(theta)*x_sub + math.cos(theta)*y_sub
        theta_err = angle_confine(theta_ref - theta)
        v = v_ref*math.cos(theta_err) + self.Kx*x_err
        w = w_ref + v_ref*(self.Ky*y_err + self.Kw*math.sin(theta_err))
        v = min(v, self.max_vel)
        v = max(v, self.min_vel)
        w = min(w, self.max_ang_vel)
        w = max(w, self.min_ang_vel)
        return v, w

class MassSpringDamper:
    def __init__(self, initial_pos, initial_vel, mass, spring, damp, Ts):
        self.pos = Integrator(Ts, initial_pos)
        self.vel = Integrator(Ts, initial_vel)
        self.mass = mass
        self.spring = spring
        self.damp = damp
        self.Ts = Ts
    
    def __call__(self, force):
        curr_pos = self.pos.get_curr()
        curr_vel = self.vel.get_curr()
        return self.pos.calc(curr_vel), self.vel.calc((force - self.spring*curr_pos - self.damp*curr_vel)/self.mass)

class BicycleModel:
    def __init__(self, length, initial_x, initial_y, initial_theta, Ts):
        self.length = length
        self.x = Integrator(Ts, initial_x)
        self.y = Integrator(Ts, initial_y)
        self.theta = Integrator(Ts, initial_theta, is_angle=True)
    
    def __call__(self, speed, steer_ang):
        curr_theta = self.theta.get_curr()
        return self.x.calc(speed*math.cos(curr_theta)), self.y.calc(speed*math.sin(curr_theta)), self.theta.calc(speed*math.tan(steer_ang)/self.length)

class DriveModel:
    def __init__(self, initial_x, initial_y, initial_theta, Ts):
        self.x = Integrator(Ts, initial_x)
        self.y = Integrator(Ts, initial_y)
        self.theta = Integrator(Ts, initial_theta, is_angle=True)

    def __call__(self, speed, ang_vel):
        curr_theta = self.theta.get_curr()
        return self.x.calc(speed*math.cos(curr_theta)), self.y.calc(speed*math.sin(curr_theta)), self.theta.calc(ang_vel)
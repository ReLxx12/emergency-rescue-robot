#!/home/orangepi/miniconda3/envs/rknn/bin/python
# -*- coding: utf-8 -*-


class PID_incremental:
    def __init__(self, p, i, d, i_a, o_t):
        self.update_param(p, i, d, i_a, o_t)
        self.reset()

    def update_param(self, p, i, d, i_a, o_t):
        self.kp = p
        self.ki = i
        self.kd = d
        self.i_a = i_a
        self.o_t = o_t
        self.reset()

    def reset(self):
        self.err = 0.0
        self.last_err = 0.0
        self.sigma_err = 0.0

    def update(self, target, now):
        self.err = target - now

        if abs(self.err) < self.i_a:
            self.sigma_err += self.err
        
        u_out = self.kp * self.err + self.ki * self.sigma_err + self.kd * (self.err - self.last_err)

        self.last_err = self.err
        
        if abs(u_out) > self.o_t:
            u_out_g = self.o_t if u_out >= 0 else -self.o_t
        else:
            u_out_g = u_out
            
        return u_out_g
    
# import rospy

MIN_NUM = float('-inf')
MAX_NUM = float('inf')


class PID(object):
    def __init__(self, kp, ki, kd, mn=MIN_NUM, mx=MAX_NUM):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min = mn
        self.max = mx

        self.int_val = self.last_int_val = self.last_error = 0.


    def reset(self):
        self.int_val = 0.0
        self.last_int_val = 0.0


    def step(self, error, sample_time):
        self.last_int_val = self.int_val

        integral = self.int_val + error * sample_time ;
        integral = max(MIN_NUM, min(MAX_NUM, integral)) 
        derivative = (error - self.last_error) / sample_time;

        val0 = self.kp * error + self.ki * integral + self.kd * derivative;
        val = min(val0, self.max)

        self.int_val = integral
        self.last_error = error
        
        return val

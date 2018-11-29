import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
import yaml


GAS_DENSITY = 2.858
ONE_MPH = 0.44704
MAX_SPEED = 40.0 # [km/h]


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, 
                    accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        # rospy.logwarn("{0}".format(self.config['is_site']))

        min_speed = 0.1
        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        kp = rospy.get_param('~throttle_k_p', 0.5)
        ki = rospy.get_param('~throttle_k_i', 0.0001)
        kd = rospy.get_param('~throttle_k_d', 0.0)
        mn = 0.0 # Minimum throttle value
        mx = 0.2 # Maximum throttle value
        self.throttle_controller = PID(kp, ki, kd, mx=mx)

        tau = 0.002 # 1/ (2*pi*tau) = cutoff frequency
        ts = 0.02 # Sample time
        self.vel_lpf = LowPassFilter(tau, ts)

        self.vehicle_mass = vehicle_mass + fuel_capacity * GAS_DENSITY
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = -1*brake_deadband
        self.max_brake_torque = decel_limit * self.vehicle_mass * wheel_radius # Torque [N*m]
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.last_time = rospy.get_time()


    def control(self, current_v, dbw_enabled, target_v, target_w):
        # Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        if not dbw_enabled:
            self.throttle_controller.reset()
            # self.last_time = rospy.get_time()
            return 0.0, 0.0, 0.0

        current_time = rospy.get_time()
        dt = current_time - self.last_time
        self.last_time = current_time

        error_v = target_v - current_v

        throttle = self.throttle_controller.step(error_v, dt)
        if throttle > 0.05:
            # Make sure it's always between 0.0 an 1.0
            throttle = max(0.0, min(1.0, throttle))
            brake = 0.0
        else:
            throttle = 0.0
            decel = max(error_v, self.decel_limit)
            # brake = 50 
            brake = self.max_brake_torque * min(self.brake_deadband, throttle/10.0)

            if target_v == 0.0 and current_v < 0.1:
                '''
                Note
                In the walkthrough, only 400 Nm of torque is applied to hold the vehicle stationary. 
                This turns out to be slightly less than the amount of force needed, and Carla will 
                roll forward with only 400Nm of torque. To prevent Carla from moving you should 
                apply approximately 700 Nm of torque.
                '''
                brake = 700 if self.config['is_site'] else 400 # [N*m] - to hold car in place if we are stopped at a light. Acceleration ~ 1 [m/sec^2]

        # Calculate steering
        steering = self.yaw_controller.get_steering(target_v, target_w, current_v)
        steering = self.vel_lpf.filt(steering)
        # rospy.logwarn(">> Controller - control | throttle, brake, steering: {0}, {1}, {2}".format(throttle, brake, steering))
        return throttle, brake, steering
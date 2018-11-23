from math import fabs
from time import sleep
import threading as thr
import include.Constants as const
from include.Vrep import Vrep

class Robot(thr.Thread):
    def __init__(self, handle, path, port_number, callback=None):
        thr.Thread.__init__(self)
        self.handle = handle
        self.path = path
        self.port_number = port_number
        self.callback = callback
        self.vrep_con = Vrep(self.port_number)
        if self.vrep_con.client_id == -1:
            raise Exception('Failed to connect to remote API server.')
        self.left_motor_handle = self.vrep_con.get_object_child(self.handle, 1)
        self.right_motor_handle = self.vrep_con.get_object_child(self.handle, 0)
        self.direction_point = self.vrep_con.get_object_child(self.handle, 15)

    def wheel_rotation(self, left_motor_speed, right_motor_speed):
        self.vrep_con.wheel_rotation(self, left_motor_speed, right_motor_speed)

    def stop(self):
        self.wheel_rotation(0, 0)

    def get_robot_orientation(self):
        angle = self.vrep_con.get_object_orientation(self.handle, self.direction_point)
        return angle

    def turn_to_a_point(self, point_pos):
        goal_angle = self.vrep_con.get_object_orientation(self.handle, point_pos)
        angle_difference = goal_angle - self.get_robot_orientation()
        if angle_difference > 180:
            angle_difference = -(360 - angle_difference)
        elif angle_difference < -180:
            angle_difference = 360 + angle_difference
        if angle_difference > 0:
            self.wheel_rotation(-const.ROTATION_SPEED, const.ROTATION_SPEED)
        else:
            self.wheel_rotation(const.ROTATION_SPEED, -const.ROTATION_SPEED)
        while fabs(angle_difference) > 2:
            current_angle = self.get_robot_orientation()
            angle_difference = goal_angle - current_angle
            if angle_difference > 180:
                angle_difference = -(360 - angle_difference)
            elif angle_difference < -180:
                angle_difference = 360 + angle_difference
        self.wheel_rotation(const.INIT_SPEED, const.INIT_SPEED)

    def run(self):
        """
        Robot motion function
        """
        for state in self.path.getStates():
            error_sum = 0
            old_error = 0
            goal = (state.getX(), state.getY())
            #print(goal_pos)
            if self.vrep_con.get_distance(self.handle, goal) >= 0.1:
                self.turn_to_a_point(goal)
                current_distance = self.vrep_con.get_distance(self.handle, goal)
                while current_distance > 0.1:
                    desired_dir = self.vrep_con.get_object_orientation(self.handle, goal)
                    error = desired_dir - self.get_robot_orientation()
                    error_sum = error_sum + error
                    if error_sum < const.iMin:
                        error_sum = const.iMin
                    elif error_sum > const.iMax:
                        error_sum = const.iMax
                    up = const.kp * error
                    ui = const.ki * error_sum
                    ud = const.kd * (error - old_error)
                    old_error = error
                    u = up + ui + ud
                    #print("u = {0}".format(u))
                    if u > 0:
                        leftSpeed = const.INIT_SPEED - fabs(u)
                        rightSpeed = const.INIT_SPEED
                    else:
                        leftSpeed = const.INIT_SPEED
                        rightSpeed = const.INIT_SPEED - fabs(u)
                    self.wheel_rotation(leftSpeed, rightSpeed)
                    current_distance = self.vrep_con.get_distance(self.handle, goal)
                    sleep(0.1)
        sleep(0.1)
        self.stop()
        sleep(0.1)
        self.vrep_con.finish_connection()
        self.callback()

import include.vrep as vrep
from math import sqrt, pow, acos, fabs, pi, cos, sin
from numpy import array
from matplotlib.path import Path
import include.Constants as const


class Vrep():
    def __init__(self, port_number):
        self.client_id = vrep.simxStart(const.CON_ADDRESS, port_number, False, True, const.TIMEOUT_IN_MS, \
                                   const.COMM_THREAD_CYCLE_IN_MS)

    def get_object_handle(self, obj_name):
        ret, handle = vrep.simxGetObjectHandle(self.client_id, obj_name, vrep.simx_opmode_oneshot_wait)
        return handle

    def wheel_rotation(self, robot, left_motor_speed, right_motor_speed):
        retCode = vrep.simxSetJointTargetVelocity(self.client_id, robot.left_motor_handle, \
                                                  left_motor_speed, vrep.simx_opmode_streaming)
        retCode = vrep.simxSetJointTargetVelocity(self.client_id, robot.right_motor_handle, \
                                                  right_motor_speed, vrep.simx_opmode_streaming)

    def get_object_child(self, parent_handle, index):
        ret, child_handle = vrep.simxGetObjectChild(self.client_id, \
                            parent_handle, index, vrep.simx_opmode_oneshot_wait)
        return child_handle

    def get_object_position(self, object_handle, relative_to_object_handle):
        """
        Function that returns position of object on the scene in V-REP
        """
        res, object_position = vrep.simxGetObjectPosition(self.client_id, object_handle, relative_to_object_handle, \
                                                          vrep.simx_opmode_blocking)
        if res == vrep.simx_return_ok:
            return (object_position[0], object_position[1])
        else:
            print('Remote function call failed with result {0}.'.format(res))
            return ()

    def get_object_orientation(self, object_handle, target):
        object_pos = self.get_object_position(object_handle, -1)
        if type(target) == int:
            target_pos = self.get_object_position(target, -1)
        else:
            target_pos = (target[0], target[1])
        direction_vector = (target_pos[0] - object_pos[0], target_pos[1] - object_pos[1])
        direction_vector_mod = sqrt(pow(direction_vector[0], 2) + pow(direction_vector[1], 2))
        norm_direction_vector = (direction_vector[0] / direction_vector_mod, direction_vector[1] / direction_vector_mod)
        if norm_direction_vector[1] != 0:
            angle = acos(norm_direction_vector[0]) * 180 / pi * fabs(norm_direction_vector[1]) / \
                norm_direction_vector[1]
        else:
            angle = acos(norm_direction_vector[0]) * 180 / pi
        return angle

    def get_distance(self, object_handle, target_position):
        object_position = self.get_object_position(object_handle, -1)
        try:
            vector = (target_position[0] - object_position[0], target_position[1] - object_position[1])
        except IndexError:
            vector = (0, 0)
        dist = sqrt(pow(vector[0], 2) + pow(vector[1], 2))
        return dist

    def get_object_childs(self, obj_name):
        """
        Function that return handles of object's childs from the V-REP scene.
        This function is useful when the exact number of objects is unknown
        """
        index = 0
        children_list = []
        child = 0
        parent_handle = self.get_object_handle(obj_name)
        while child != -1:
            res, child = vrep.simxGetObjectChild(self.client_id, parent_handle, index, vrep.simx_opmode_blocking)
            if res == vrep.simx_return_ok:
                children_list.append(child)
                index = index + 1
            else:
                print('Remote fucntion get_object_childs call failed.')
                return []
        del children_list[len(children_list) - 1]
        return children_list

    def get_boundary_points(self, object_handle):
        """
        Function that returns boundary points of object's (obstacle) boundary box
        """
        obstacle_position = self.get_object_position(object_handle, -1)

        ret, orient = vrep.simxGetObjectOrientation(self.client_id, object_handle, -1, \
                                                    vrep.simx_opmode_blocking)
        ret, x_1 = vrep.simxGetObjectFloatParameter(self.client_id, object_handle, 15, \
                                                    vrep.simx_opmode_blocking)
        ret, y_1 = vrep.simxGetObjectFloatParameter(self.client_id, object_handle, 16, \
                                                    vrep.simx_opmode_blocking)
        ret, x_2 = vrep.simxGetObjectFloatParameter(self.client_id, object_handle, 18, \
                                                    vrep.simx_opmode_blocking)
        ret, y_2 = vrep.simxGetObjectFloatParameter(self.client_id, object_handle, 19, \
                                                    vrep.simx_opmode_blocking)
        angle = orient[2]
        # Extension of boundaries, so that the robots moves without collisions
        x_1 = x_1 - 0.3
        x_2 = x_2 + 0.3
        y_1 = y_1 - 0.3
        y_2 = y_2 + 0.3

        p_1 = [x_1 * cos(angle) - y_1 * sin(angle) + obstacle_position[0], y_1 * \
               cos(angle) + x_1 * sin(angle) + obstacle_position[1]]
        p_2 = [x_1 * cos(angle) - y_2 * sin(angle) + obstacle_position[0], y_2 * \
               cos(angle) + x_1 * sin(angle) + obstacle_position[1]]
        p_3 = [x_2 * cos(angle) - y_2 * sin(angle) + obstacle_position[0], y_2 * \
               cos(angle) + x_2 * sin(angle) + obstacle_position[1]]
        p_4 = [x_2 * cos(angle) - y_1 * sin(angle) + obstacle_position[0], y_1 * \
               cos(angle) + x_2 * sin(angle) + obstacle_position[1]]
        return Path(array([p_1, p_2, p_3, p_4]))

    def finish_connection(self):
        vrep.simxFinish(self.client_id)

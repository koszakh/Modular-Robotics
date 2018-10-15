"""
This program is intended to plan paths for group of robots and their goal states
"""
import time
import math
import numpy as np
import threading as thr
from ompl import base as ob
from ompl import geometric as og
import matplotlib.path as mplPath
import vrep
import OptimalPlanning as op
import Constants as cons
from functools import partial

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
        self.name = self.vrep_con.get_object_name(self.handle)
        self.direction_point = self.vrep_con.get_object_child(self.handle, 15)

    def wheel_rotation(self, left_motor_speed, right_motor_speed):
        retCode = vrep.simxSetJointTargetVelocity(self.vrep_con.client_id, self.left_motor_handle, \
                                                  left_motor_speed, vrep.simx_opmode_streaming)
        retCode = vrep.simxSetJointTargetVelocity(self.vrep_con.client_id, self.right_motor_handle, \
                                                  right_motor_speed, vrep.simx_opmode_streaming)

    def stop(self):
        self.wheel_rotation(0, 0)
        print("Робот {0} закончил движение.".format(self.name))

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
            self.wheel_rotation(-cons.ROTATION_SPEED, cons.ROTATION_SPEED)
        else:
            self.wheel_rotation(cons.ROTATION_SPEED, -cons.ROTATION_SPEED)
        while math.fabs(angle_difference) > 2:
            current_angle = self.get_robot_orientation()
            angle_difference = goal_angle - current_angle
            if angle_difference > 180:
                angle_difference = -(360 - angle_difference)
            elif angle_difference < -180:
                angle_difference = 360 + angle_difference
        self.wheel_rotation(cons.INIT_SPEED, cons.INIT_SPEED)

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
                    if error_sum < cons.iMin:
                        error_sum = cons.iMin
                    elif error_sum > cons.iMax:
                        error_sum = cons.iMax
                    up = cons.kp * error
                    ui = cons.ki * error_sum
                    ud = cons.kd * (error - old_error)
                    old_error = error
                    u = up + ui + ud
                    #print("u = {0}".format(u))
                    if u > 0:
                        leftSpeed = cons.INIT_SPEED - math.fabs(u)
                        rightSpeed = cons.INIT_SPEED
                    else:
                        leftSpeed = cons.INIT_SPEED
                        rightSpeed = cons.INIT_SPEED - math.fabs(u)
                    self.wheel_rotation(leftSpeed, rightSpeed)
                    current_distance = self.vrep_con.get_distance(self.handle, goal)
                    time.sleep(0.1)
        time.sleep(0.1)
        self.stop()
        time.sleep(0.1)
        vrep.simxFinish(self.vrep_con.client_id)
        self.callback()

class Vrep():
    def __init__(self, port_number):
        self.client_id = vrep.simxStart(cons.CON_ADDRESS, port_number, False, True, cons.TIMEOUT_IN_MS, \
                                   cons.COMM_THREAD_CYCLE_IN_MS)

    def get_object_handle(self, obj_name):
        ret, handle = vrep.simxGetObjectHandle(self.client_id, obj_name, vrep.simx_opmode_oneshot_wait)
        return handle

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
        direction_vector_mod = math.sqrt(math.pow(direction_vector[0], 2) + math.pow(direction_vector[1], 2))
        norm_direction_vector = (direction_vector[0] / direction_vector_mod, direction_vector[1] / direction_vector_mod)
        if norm_direction_vector[1] != 0:
            angle = math.acos(norm_direction_vector[0]) * 180 / math.pi * math.fabs(norm_direction_vector[1]) / \
                norm_direction_vector[1]
        else:
            angle = math.acos(norm_direction_vector[0]) * 180 / math.pi
        return angle

    def get_distance(self, object_handle, target_position):
        object_position = self.get_object_position(object_handle, -1)
        try:
            vector = (target_position[0] - object_position[0], target_position[1] - object_position[1])
        except IndexError:
            vector = (0, 0)
        dist = math.sqrt(math.pow(vector[0], 2) + math.pow(vector[1], 2))
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

        p_1 = [x_1 * math.cos(angle) - y_1 * math.sin(angle) + obstacle_position[0], y_1 * \
               math.cos(angle) + x_1 * math.sin(angle) + obstacle_position[1]]
        p_2 = [x_1 * math.cos(angle) - y_2 * math.sin(angle) + obstacle_position[0], y_2 * \
               math.cos(angle) + x_1 * math.sin(angle) + obstacle_position[1]]
        p_3 = [x_2 * math.cos(angle) - y_2 * math.sin(angle) + obstacle_position[0], y_2 * \
               math.cos(angle) + x_2 * math.sin(angle) + obstacle_position[1]]
        p_4 = [x_2 * math.cos(angle) - y_1 * math.sin(angle) + obstacle_position[0], y_1 * \
               math.cos(angle) + x_2 * math.sin(angle) + obstacle_position[1]]
        return mplPath.Path(np.array([p_1, p_2, p_3, p_4]))

    def get_object_name(self, objectHandle):
        emptyBuff = bytearray()
        res, retInts, retFloats, objName, retBuffer = vrep.simxCallScriptFunction(self.client_id, 'goalConfigurations',
                                                                                     vrep.sim_scripttype_childscript,
                                                                                     'getObjectName', [objectHandle], [], [], emptyBuff,
                                                                                     vrep.simx_opmode_blocking)
        return objName[0]

class Controller():
    def __init__(self):
        vrep.simxFinish(-1)
        self.vrep_con = Vrep(cons.CON_PORT)
        print(self.vrep_con.client_id)
        self.robots = self.vrep_con.get_object_childs(cons.ROBOTS_NAMES_TREE)
        self.targets = self.vrep_con.get_object_childs(cons.TARGETS_NAMES_TREE)
        self.obstacles = list(map(lambda x: self.vrep_con.get_boundary_points(x), \
                                             self.vrep_con.get_object_childs(cons.OBSTACLES_NAMES_TREE)))
        #self.full_obstacles = self.obstacles + list(map(lambda x: self.vrep_con.get_boundary_points(x), \
        #                                    self.vrep_con.get_object_childs(cons.TARGETS_NAMES_TREE)))
        #for obstacle in self.full_obstacles:
        #    print(self.vrep_con.get_object_name(obstacle))

    def multiple_paths_planning(self):
        if self.vrep_con.client_id != -1:
            print('Program started')
            print('Connected to remote API server')
            print("Число роботов: {0}".format(len(self.robots)))
            robots_paths, estimated_time = self.target_assignment()
            for i in range(len(self.robots)):
                path = robots_paths[str(self.robots[i])]
                print("Робот №{0} имеет путь, состоящий из {1} точек, длиной {2}" \
                      .format(i + 1, path.getStateCount(), path.length()))
            sum = 0
            for robot in self.robots:
                sum = sum + robots_paths[str(robot)].length()
            print("Сумма длин всех путей: {0}\nВремя, затраченное на планирование пути: {1}" \
                  .format(sum, estimated_time))
            port_num = cons.CON_PORT + 1
            #robot1_thread = Robot(self.robots[0], robots_paths[str(self.robots[0])], \
            #                      port_num, callback=self.on_thread_finished)
            #robot1_thread.start()
            #robot2_thread = Robot(self.robots[1], robots_paths[str(self.robots[1])], \
            #                      port_num+1, callback=self.on_thread_finished)
            #robot2_thread.start()
            for robot in self.robots:
                robot_thread = Robot(robot, robots_paths[str(robot)], \
                                     port_num, callback=self.on_thread_finished)
                port_num += 1
                robot_thread.start()
        else:
            print('Failed connecting to remote API server')

    def target_assignment(self):
        """
        Target assignment and path planning function for group of robots
        """
        start_time = time.time()
        paths = {}

        for robot in self.robots:
            avoidable_targets = []
            min_path_length = float('Inf')
            for target in self.targets:
                robot_pos = self.vrep_con.get_object_position(robot, -1)
                target_pos = self.vrep_con.get_object_position(target, -1)
                path = self.plan(robot_pos, target_pos, None, self.obstacles)
                if path.length() < min_path_length:
                    chosen_target = target
                    min_path_length = path.length()
            goal_pos = self.vrep_con.get_object_position(chosen_target, -1)
            avoidable_targets = self.targets.copy()
            avoidable_targets.remove(chosen_target)
            full_obstacles = self.obstacles + list(map(lambda x: self.vrep_con.get_boundary_points(x), \
                                             avoidable_targets))
            paths[str(robot)] = self.plan(robot_pos, goal_pos, cons.PLANNER_RANGE, full_obstacles)
            self.targets.remove(chosen_target)
        final_time = time.time() - start_time
        return paths, final_time

    def isStateValid(self, obstacle_list, state):
        """
        Function that checks the validity of a state of the configuration space
        """
        return self.beyond_obstacles(state.getX(), state.getY(), obstacle_list)

    def beyond_obstacles(self, x_coord, y_coord, obstacle_list):
        """
        Function that check whether a point is inside an obstacle
        """
        for obstacle in obstacle_list:
            if obstacle.contains_point((x_coord, y_coord)):
                return False
        return True

    def space_creation(self):
        """
        Function of creating a configuration space
        """
        space = ob.SE2StateSpace()
        bounds = ob.RealVectorBounds(2)
        bounds.setLow(cons.LOW_BOUNDS)
        bounds.setHigh(cons.HIGH_BOUNDS)
        space.setBounds(bounds)
        return space

    def space_info_creation(self, space, obstacle_list):
        """
        Function of creating space information that includes valid and invalid states
        """
        space_info = ob.SpaceInformation(space)
        isValidFn = ob.StateValidityCheckerFn(partial(self.isStateValid, obstacle_list))
        space_info.setStateValidityChecker(isValidFn)
        #space_info.setStateValidityChecker(ob.StateValidityCheckerFn(self.isStateValid))
        space_info.setup()
        return space_info

    def problem_definition(self, space, space_info, start_pos, goal_pos):
        """
        Function which define path problem that includes start and goal states
        between which the path must be built
        """
        pdef = ob.ProblemDefinition(space_info)
        start = ob.State(space)
        start[0] = start_pos[0]
        start[1] = start_pos[1]
        goal = ob.State(space)
        goal[0] = goal_pos[0]
        goal[1] = goal_pos[1]
        pdef.setStartAndGoalStates(start, goal)
        return pdef

    def allocate_planner(self, space_info, pdef, planner_range):
        """
        Planner setting function that includes choice of planner type,
        determination of maximum distance between 2 points of path and setting problem definition
        """
        optimizing_planner = op.allocatePlanner(space_info, cons.PLANNER_TYPE)
        if planner_range != None:
            optimizing_planner.setRange(planner_range)
        optimizing_planner.setProblemDefinition(pdef)
        optimizing_planner.setup()
        return optimizing_planner

    def path_optimization(self, path, space_info):
        """
        Function of optimizing the path and splitting it into equal segments
        """
        ps = og.PathSimplifier(space_info)
        shorteningPath = ps.shortcutPath(path)
        reduceVertices = ps.reduceVertices(path)
        path.interpolate(int(path.length() * 10))
        return path

    def plan(self, start_pos, goal_pos, planner_range, obstacles):
        """
        Function that returns path for one robot
        """
        space = self.space_creation()
        space_info = self.space_info_creation(space, obstacles)
        pdef = self.problem_definition(space, space_info, start_pos, goal_pos)
        optimizing_planner = self.allocate_planner(space_info, pdef, planner_range)
        solved = optimizing_planner.solve(cons.RUN_TIME)
        if solved:
            path = self.path_optimization(pdef.getSolutionPath(), space_info)
            return path
        else:
            print("No solution found")
            return None

    def on_thread_finished(self):
        print("active_threads_count = {0}".format(thr.active_count()))
        if thr.active_count() == 2:
            print("The end.")
            vrep.simxStopSimulation(self.vrep_con.client_id, vrep.simx_opmode_oneshot_wait)
            vrep.simxFinish(self.vrep_con.client_id)


if __name__ == "__main__":
    solving = Controller()
    solving.multiple_paths_planning()

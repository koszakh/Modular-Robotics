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

class Robot(thr.Thread):
    def __init__(self, robot_handle, path, client_id):
        thr.Thread.__init__(self)
        self.robot_handle = robot_handle
        self.path = path
        self.client_id = client_id

    def run(self):
        """
        Robot motion function
        """
        for state in self.path.getStates():
            ret_code = vrep.simxSetObjectPosition(self.client_id, self.robot_handle, -1, \
                                                  (state.getX(), state.getY(), 0.3), vrep.simx_opmode_oneshot)
            time.sleep(0.2)
        if thr.active_count() == 2:
            vrep.simxStopSimulation(self.client_id, vrep.simx_opmode_oneshot_wait)
            vrep.simxFinish(self.client_id)

class Vrep():
    def __init__(self):
        print('Program started')
        vrep.simxFinish(-1)

        self.client_id = vrep.simxStart(cons.CON_ADDRESS, cons.CON_PORT, True, True, cons.TIMEOUT_IN_MS, \
                                   cons.COMM_THREAD_CYCLE_IN_MS)

    def get_object_position(self, object_handle):
        """
        Function that returns position of object on the scene in V-REP
        """
        res, object_position = vrep.simxGetObjectPosition(self.client_id, object_handle, -1, \
                                                          vrep.simx_opmode_blocking)
        if res == vrep.simx_return_ok:
            return (object_position[0], object_position[1])
        else:
            print('Remote function call failed.')
            return ()

    def get_object_childs(self, objname):
        """
        Function that return handles of object's childs from the V-REP scene.
        This function is useful when the exact number of objects is unknown
        """
        index = 0
        children_list = []
        child = 0
        res, parent_handle = vrep.simxGetObjectHandle(self.client_id, objname, vrep.simx_opmode_oneshot_wait)
        while child != -1:
            res, child = vrep.simxGetObjectChild(self.client_id, parent_handle, index, vrep.simx_opmode_blocking)
            if res == vrep.simx_return_ok:
                children_list.append(child)
                index = index + 1
            else:
                print('Remote fucntion call failed.')
                return []
        del children_list[len(children_list) - 1]
        return children_list

    def get_boundary_points(self, object_handle):
        """
        Function that returns boundary points of object's (obstacle) boundary box
        """
        position = self.get_object_position(object_handle)

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
        x_1 = x_1 - 0.33
        x_2 = x_2 + 0.33
        y_1 = y_1 - 0.33
        y_2 = y_2 + 0.33

        p_1 = [x_1 * math.cos(angle) - y_1 * math.sin(angle) + position[0], y_1 * \
               math.cos(angle) + x_1 * math.sin(angle) + position[1]]
        p_2 = [x_1 * math.cos(angle) - y_2 * math.sin(angle) + position[0], y_2 * \
               math.cos(angle) + x_1 * math.sin(angle) + position[1]]
        p_3 = [x_2 * math.cos(angle) - y_2 * math.sin(angle) + position[0], y_2 * \
               math.cos(angle) + x_2 * math.sin(angle) + position[1]]
        p_4 = [x_2 * math.cos(angle) - y_1 * math.sin(angle) + position[0], y_1 * \
               math.cos(angle) + x_2 * math.sin(angle) + position[1]]
        return mplPath.Path(np.array([p_1, p_2, p_3, p_4]))

class Controller():
    def __init__(self):
        self.vrep_con = Vrep()
        self.robots = self.vrep_con.get_object_childs(cons.ROBOTS_NAMES_TREE)
        self.targets = self.vrep_con.get_object_childs(cons.TARGETS_NAMES_TREE)
        self.obstacles_boundary_points = list(map(lambda x: self.vrep_con.get_boundary_points(x), \
                                             self.vrep_con.get_object_childs(cons.OBSTACLES_NAMES_TREE)))

    def multiple_paths_planning(self):
        if self.vrep_con.client_id != -1:
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
            for robot in self.robots:
                robot_thread = Robot(robot, robots_paths[str(robot)], \
                                     self.vrep_con.client_id)
                robot_thread.start()

    def target_assignment(self):
        """
        Target assignment and path planning function for group of robots
        """
        start_time = time.time()
        paths = {}
        for start_state in self.robots:
            min_path_length = float('Inf')
            for target in self.targets:
                robot_pos = self.vrep_con.get_object_position(start_state)
                target_pos = self.vrep_con.get_object_position(target)
                path = self.plan(robot_pos, target_pos, None)
                if path.length() < min_path_length:
                    chosen_target = target
                    #paths[str(start_state)] = path
                    min_path_length = path.length()
            goal_pos = self.vrep_con.get_object_position(chosen_target)
            paths[str(start_state)] = self.plan(robot_pos, goal_pos, cons.PLANNER_RANGE)
            self.targets.remove(chosen_target)
        final_time = time.time() - start_time
        return paths, final_time

    def isStateValid(self, state):
        """
        Function that checks the validity of a state of the configuration space
        """
        return self.beyond_obstacles(state.getX(), state.getY())

    def beyond_obstacles(self, x_coord, y_coord):
        """
        Function that check whether a point is inside an obstacle
        """
        for obstacle in self.obstacles_boundary_points:
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

    def space_info_creation(self, space):
        """
        Function of creating space information that includes valid and invalid states
        """
        space_info = ob.SpaceInformation(space)
        space_info.setStateValidityChecker(ob.StateValidityCheckerFn(self.isStateValid))
        space_info.setup()
        return space_info

    def problem_definition(self, space, space_info, start_pos, goal_pos):
        """
        Function of path planning problem defintition that includes start and goal states
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
        path.interpolate(int(path.length() * 10))
        return path

    def plan(self, start_pos, goal_pos, planner_range):
        """
        Function that returns path for one robot
        """
        space = self.space_creation()
        space_info = self.space_info_creation(space)
        pdef = self.problem_definition(space, space_info, start_pos, goal_pos)
        optimizing_planner = self.allocate_planner(space_info, pdef, planner_range)
        solved = optimizing_planner.solve(cons.RUN_TIME)
        if solved:
            path = self.path_optimization(pdef.getSolutionPath(), space_info)
            return path
        else:
            print("No solution found")
            return None

    #def are_targets_reached(self):
    #    if thr.active_count() == 2:
    #        vrep.simxStopSimulation(self.vrep_con.client_id, vrep.simx_opmode_oneshot_wait)
    #        vrep.simxFinish(self.vrep_con.client_id)

if __name__ == "__main__":
    solving = Controller()
    solving.multiple_paths_planning()

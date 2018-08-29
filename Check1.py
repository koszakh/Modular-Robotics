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

OBJECTIVE_TYPE = 'pathlength'
PLANNER_TYPE = 'rrtconnect'
RUN_TIME = 2

class Robot(thr.Thread):
    def __init__(self, robot_handle, path):
        thr.Thread.__init__(self)
        self.robot_handle = robot_handle
        self.path = path

    def run(self):
        for state in self.path:
            ret_code = vrep.simxSetObjectPosition(client_id, self.robot_handle, -1, \
                                (state.getX(), state.getY(), 0.3), vrep.simx_opmode_oneshot)
            time.sleep(0.2)
        if thr.active_count() == 2:
            vrep.simxStopSimulation(client_id, vrep.simx_opmode_oneshot_wait)
            vrep.simxFinish(client_id)


def planner(start_states, goal_states):
    """
    Target assignment and path planning function for group of robots
    """
    start_time = time.time()
    target_assignment = {}
    paths = {}
    for start_state in start_states:
        min_path_length = float('Inf')
        for target in goal_states:
            robot_pos = get_object_position(start_state)
            target_pos = get_object_position(target)
            path = plan(robot_pos, target_pos, PLANNER_TYPE)
            if path.length() < min_path_length:
                target_assignment[start_state] = target
                min_path_length = path.length()
        goal_pos = get_object_position(target_assignment[start_state])
        paths[str(start_state)] = plan(robot_pos, goal_pos, \
                                       PLANNER_TYPE)
        goal_states.remove(target_assignment[start_state])
    final_time = time.time() - start_time
    return paths, final_time

def get_object_position(object_handle):
    """
    Function that returns position of object on the scene in V-REP
    """
    res, object_position = vrep.simxGetObjectPosition(client_id, object_handle, -1, \
                                                     vrep.simx_opmode_blocking)
    if res == vrep.simx_return_ok:
        return (object_position[0], object_position[1])
    else:
        print('Remote function call failed.')
        return ()

def get_object_childs(objname):
    """
    Function that return handles of object's childs
    """
    index = 0
    children_list = []
    child = 0
    res, parent_handle = vrep.simxGetObjectHandle(client_id, objname, vrep.simx_opmode_oneshot_wait)
    while child != -1:
        res, child = vrep.simxGetObjectChild(client_id, parent_handle, index, vrep.simx_opmode_blocking)
        if res == vrep.simx_return_ok:
            children_list.append(child)
            index = index + 1
        else:
            print('Remote fucntion call failed.')
            return []
    del children_list[len(children_list) - 1]
    return children_list

def get_boundary_points(object_handle):
    """
    Function that returns boundary points of object's (obstacle) boundary box
    """
    position = get_object_position(object_handle)
    ret, orient = vrep.simxGetObjectOrientation(client_id, object_handle, -1, \
                                                vrep.simx_opmode_blocking)
    ret, x_1 = vrep.simxGetObjectFloatParameter(client_id, object_handle, 15, \
                                               vrep.simx_opmode_blocking)
    ret, y_1 = vrep.simxGetObjectFloatParameter(client_id, object_handle, 16, \
                                               vrep.simx_opmode_blocking)
    ret, x_2 = vrep.simxGetObjectFloatParameter(client_id, object_handle, 18, \
                                               vrep.simx_opmode_blocking)
    ret, y_2 = vrep.simxGetObjectFloatParameter(client_id, object_handle, 19, \
                                               vrep.simx_opmode_blocking)
    angle = orient[2]
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

def isStateValid(state):
    """
    Function that checks the validity of a state of the configuration space
    """
    return beyond_obstacles(state.getX(), state.getY())

def beyond_obstacles(x_coord, y_coord):
    """
    Function that check whether a point is inside an obstacle
    """
    for obstacle in obstacles_boundary_points:
        if obstacle.contains_point((x_coord, y_coord)):
            return False
    return True

def plan(start_pos, goal_pos, planner_type):
    """
    Function that returns path for one robot
    """
    space = ob.SE2StateSpace()
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(-4.75)
    bounds.setHigh(4.75)
    space.setBounds(bounds)
    space = ob.SE2StateSpace()
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(-4.75)
    bounds.setHigh(4.75)
    space.setBounds(bounds)
    space_info = ob.SpaceInformation(space)
    space_info.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))
    space_info.setup()
    start = ob.State(space)
    start[0] = start_pos[0]
    start[1] = start_pos[1]
    goal = ob.State(space)
    goal[0] = goal_pos[0]
    goal[1] = goal_pos[1]
    pdef = ob.ProblemDefinition(space_info)
    pdef.setStartAndGoalStates(start, goal)
    pdef.setOptimizationObjective(op.allocateObjective(space_info, OBJECTIVE_TYPE))
    optimizing_planner = op.allocatePlanner(space_info, planner_type)
    optimizing_planner.setProblemDefinition(pdef)
    optimizing_planner.setup()
    #print(si.settings())
    #print(pdef)
    solved = optimizing_planner.solve(RUN_TIME)
    if solved:
        # Output the length of the path found
        print('{0} found solution of path length {1:.4f}\n'.format( \
        optimizing_planner.getName(), \
        pdef.getSolutionPath().length()))
        path = pdef.getSolutionPath()
        print(path.length())
        ps = og.PathSimplifier(space_info)
        shorteningPath = ps.shortcutPath(path)
        #smoothBSplineForPath = ps.smoothBSpline(path)
        path.interpolate(int(path.length() * 10))
        print(path.length())
        return path
    else:
        print("No solution found")
        return None

if __name__ == "__main__":
    print('Program started')
    vrep.simxFinish(-1)
    client_id = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
    if client_id != -1:
        print('Connected to remote API server')
        robots = get_object_childs('startConfigurations')
        targets = get_object_childs('goalConfigurations')
        print("Число роботов: {0}".format(len(robots)))
        obstacles_boundary_points = list(map(lambda x: get_boundary_points(x),\
                                           get_object_childs("OuterWalls")))
        #start_pos = get_object_position(robots[0])
        #goal_pos = get_object_position(targets[0])
        #path1 = plan(start_pos, goal_pos, 'pathlength', "rrtconnect", 0.2, 5.0)
        robot_paths, estimated_time = planner(robots, targets)
        sum = 0
        for i in range(len(robots)):
            path = robot_paths[str(robots[i])]
            print("Робот №{0} имеет путь, состоящий из {1} точек, длиной {2}"\
                  .format(i + 1, path.getStateCount(), path.length()))
        for robot in robots:
            sum = sum + robot_paths[str(robot)].length()
        print("Сумма длин всех путей: {0}\nВремя, затраченное на планирование пути: {1}"\
              .format(sum, estimated_time))
        for robot in robots:
           my_thread = Robot(robot, robot_paths[str(robot)].getStates())
           my_thread.start()

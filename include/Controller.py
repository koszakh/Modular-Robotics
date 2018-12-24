from time import time
import threading as thr
from ompl import base as ob
from ompl import geometric as og
from ompl.util import OMPL_ERROR
from include.Robot import Robot
from include.Vrep import Vrep
import include.vrep as vrep
import include.Constants as const
from functools import partial


class Controller():
    def __init__(self):
        vrep.simxFinish(-1)
        self.vrep_con = Vrep(const.CON_PORT)
        print(self.vrep_con.client_id)
        self.robots = self.vrep_con.get_object_childs(const.ROBOTS_NAMES_TREE)
        self.targets = self.vrep_con.get_object_childs(const.TARGETS_NAMES_TREE)
        self.obstacles = list(map(lambda x: self.vrep_con.get_boundary_points(x), \
                                             self.vrep_con.get_object_childs(const.OBSTACLES_NAMES_TREE)))
        print(self.obstacles)
        #self.full_obstacles = self.obstacles + list(map(lambda x: self.vrep_con.get_boundary_points(x), \
        #                                    self.vrep_con.get_object_childs(const.TARGETS_NAMES_TREE)))
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
            port_num = const.CON_PORT + 1


            # Robots moution


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
        start_time = time()
        paths = {}

        for robot in self.robots:
            avoidable_targets = []
            min_path_length = float('Inf')
            for target in self.targets:
                robot_pos = self.vrep_con.get_object_position(robot, -1)  # Robot_position from marker
                target_pos = self.vrep_con.get_object_position(target, -1)  # Goal pos from marker
                print("test", robot_pos, target_pos, self.obstacles)

                path = self.plan(robot_pos, target_pos, None, self.obstacles)
                if path.length() < min_path_length:
                    chosen_target = target
                    min_path_length = path.length()
            goal_pos = self.vrep_con.get_object_position(chosen_target, -1)

            avoidable_targets = self.targets.copy()
            avoidable_targets.remove(chosen_target)
            full_obstacles = self.obstacles + list(map(lambda x: self.vrep_con.get_boundary_points(x), avoidable_targets))
            print(full_obstacles)

            paths[str(robot)] = self.plan(robot_pos, goal_pos, const.PLANNER_RANGE, full_obstacles)
            self.targets.remove(chosen_target)
        final_time = time() - start_time
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
        bounds.setLow(const.LOW_BOUNDS)
        bounds.setHigh(const.HIGH_BOUNDS)
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

    def choose_planner(self, si, plannerType):
        if plannerType.lower() == "bfmtstar":
            return og.BFMT(si)
        elif plannerType.lower() == "bitstar":
            return og.BITstar(si)
        elif plannerType.lower() == "fmtstar":
            return og.FMT(si)
        elif plannerType.lower() == "informedrrtstar":
            return og.InformedRRTstar(si)
        elif plannerType.lower() == "prmstar":
            return og.PRMstar(si)
        elif plannerType.lower() == "rrtconnect":
            return og.RRTConnect(si)
        elif plannerType.lower() == "rrtsharp":
            return og.RRTsharp(si)
        elif plannerType.lower() == "rrtstar":
            return og.RRTstar(si)
        elif plannerType.lower() == "sorrtstar":
            return og.SORRTstar(si)
        else:
            OMPL_ERROR("Planner-type is not implemented in allocation function.")

    def allocate_planner(self, space_info, pdef, planner_range):
        """
        Planner setting function that includes choice of planner type,
        determination of maximum distance between 2 points of path and setting problem definition
        """
        optimizing_planner = self.choose_planner(space_info, const.PLANNER_TYPE)
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
        solved = optimizing_planner.solve(const.RUN_TIME)
        if solved:
            path = self.path_optimization(pdef.getSolutionPath(), space_info)
            return path
        else:
            print("No solution found")
            return None

    def on_thread_finished(self):
        if thr.active_count() == 2:
            print("The end.")
            vrep.simxStopSimulation(self.vrep_con.client_id, vrep.simx_opmode_oneshot_wait)
            vrep.simxFinish(self.vrep_con.client_id)
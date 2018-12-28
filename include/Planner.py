from time import time
from ompl import base as ob
from ompl import geometric as og
from ompl.util import OMPL_ERROR
import include.Constants as const
from functools import partial
from include.Fileds_objects import Point
import numpy as np
from matplotlib.path import Path


class Paths_planner():
    def __init__(self):
        self.robots = {}
        self.targets = {}
        self.obstacles = {}

    def multiple_paths_planning(self):
        # print('Program started')
        # print('Connected to remote API server')
        # print("Число роботов: {0}".format(len(self.robots)))
        robots_paths, estimated_time = self.target_assignment()
        return robots_paths, estimated_time

    def target_assignment(self):
        """
        Target assignment and path planning function for group of robots
        """
        start_time = time()
        paths = {}

        if len(self.robots) == len(self.targets):

            for robot_id in self.robots.keys():
                tmp_robots = self.robots.copy()
                tmp_targets = self.targets.copy()
                robot_position = tmp_robots.pop(robot_id).get_center().remap_to_ompl_coord_system().get_xy()
                target_position = tmp_targets.pop(robot_id).get_center().remap_to_ompl_coord_system().get_xy()
                full_obstacles = []

                if len(self.obstacles):
                    for obstacle in self.obstacles:
                        full_obstacles.append(obstacle.get_ompl_path())

                for target in tmp_targets.values():
                    full_obstacles.append(target.get_ompl_path())
                for robot in tmp_robots.values():
                    full_obstacles.append(robot.get_ompl_path())
                paths[robot_id] = self.plan(robot_position, target_position, const.PLANNER_RANGE, full_obstacles)

        final_time = time() - start_time
        return paths, final_time

    def get_line_cntr(self, pt1, pt2):
        line_cntr = tuple(map(lambda x: x, ((pt1[0] + pt2[0]) / 2, (pt1[1] + pt2[1]) / 2)))
        return line_cntr

    def get_marker_cntr(self, sqr):
        front_left_corner = sqr[0]
        behind_right_corner = sqr[2]
        cntr = self.get_line_cntr(front_left_corner, behind_right_corner)
        return cntr

    def set_robots(self, robots_dict):
        self.robots = robots_dict

    def set_obstacles(self, obstacles_dict):
        self.obstacles = list(obstacles_dict.values())

    def set_targets(self, goals_dict):
        self.targets = {}
        self.targets = goals_dict

    def set_targets_corners(self, corners):
        self.targets_corners = []
        self.targets_corners = corners

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

    def path_optimization(self, path, space_info):
        """
        Function of optimizing the path and splitting it into equal segments
        """
        ps = og.PathSimplifier(space_info)
        shorteningPath = ps.shortcutPath(path)
        reduceVertices = ps.reduceVertices(path)
        path.interpolate(int(path.length() * 1.1))
        return path

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
        # space_info.setStateValidityChecker(ob.StateValidityCheckerFn(self.isStateValid))
        space_info.setup()
        return space_info

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

    def path_to_point_list(self, path):
        states = path.getStates()
        path_lst = []
        for state in states:
            x = state.getX()
            y = state.getY()
            cv_point = Point((x, y)).remap_to_img_coord_system()
            path_lst.append(cv_point)
        return path_lst
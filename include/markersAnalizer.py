from math import sqrt, degrees, acos
import cv2
from cv2 import aruco

class markersAnalizer():
    def __init__(self):
        self.__robots = {}
        self.__walls = {}
        self.__goals = {}

    def get_robots(self):
        return self.__robots

    def get_walls(self):
        return self.__walls

    def get_goals(self):
        return self.__goals

    def detect_markers(self, img, aruco_dict, parameters):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        return corners, ids

    def parse_ids(self, markers_dict):
        """ This func create dictionaries: robots, walls, goals.
            Key is ID. Value is center (for robots and goals) or corners (for walls). """
        tmp_markers_dict = {}
        for id in markers_dict.keys():
            if len(str(id)) == 1:
                robot_cntr = self.get_marker_cntr(markers_dict[id])
                robot_direction = self.get_marker_direction(markers_dict[id])
                self.__robots[id] = [robot_cntr, robot_direction]
            elif len(str(id)) == 3:
                self.__goals[id] = markers_dict[id]
            else:
                if not id in tmp_markers_dict.keys():
                    tmp_markers_dict[id] = markers_dict[id]
                else:
                    tmp_markers_dict[id] = [markers_dict[id], tmp_markers_dict[id]]
        walls_markers_dict = self.create_walls_markers_dict(tmp_markers_dict)
        self.__walls = self.get_wall_corners(walls_markers_dict)

    def create_walls_markers_dict(self, markers_dict):
        """ Every wall marked up by two markers.
            So we need to find wall's 4 corners from 8 marker's corners and put them into a walls dict.
            This func create dict with structure: {ID: [the first marker's corners, the second marker's corners]}"""
        walls = {}
        for corners_key in markers_dict:
            id = corners_key // 10
            if not id in walls.keys():
                walls[id] = markers_dict[corners_key]
            else:
                walls[id] = [walls[id], markers_dict[corners_key]]
        return walls

    def get_distance_between_pts(self, pt1, pt2):
        return sqrt((pt2[0] - pt1[0])**2 + (pt2[1] - pt1[1])**2)

    def get_wall_corners(self, walls_markers_dict):
        """ Extract walls corners from walls_markers_dict """
        walls_corners = {}
        for key in walls_markers_dict.keys():
            corners = walls_markers_dict[key]
            if len(corners) >= 2:
                sqr1 = corners[0].tolist()
                sqr2 = corners[1].tolist()

                cntr1 = self.get_marker_cntr(sqr1)
                cntr2 = self.get_marker_cntr(sqr2)

                lenghts1 = []
                for pt in sqr1:
                    lenghts1.append(self.get_distance_between_pts(pt, cntr2))

                max1 = 0
                for i in range(0, len(lenghts1)):
                    if lenghts1[i] > max1:
                        lenghts1[i] = lenghts1[i] + 0.1
                        max1 = lenghts1[i]

                lenghts1_copy = lenghts1.copy()
                lenghts1_copy.remove(max1)

                max2 = lenghts1_copy[0]
                for i in range(1, len(lenghts1_copy)):
                    if lenghts1_copy[i] > max2:
                        max2 = lenghts1_copy[i]

                lenghts2 = []
                for pt in sqr2:
                    lenghts2.append(self.get_distance_between_pts(pt, cntr1))

                max3 = 0
                for i in range(0, len(lenghts2)):
                    if lenghts2[i] > max3:
                        lenghts2[i] = lenghts2[i] + 0.1
                        max3 = lenghts2[i]

                lenghts2_copy = lenghts2.copy()
                lenghts2_copy.remove(max3)

                max4 = lenghts2_copy[0]
                for i in range(1, len(lenghts2_copy)):
                    if lenghts2_copy[i] > max4:
                        max4 = lenghts2_copy[i]

                walls_corners[key] = [sqr1[lenghts1.index(max1)], sqr1[lenghts1.index(max2)],
                                      sqr2[lenghts2.index(max3)], sqr2[lenghts2.index(max4)]]

        return walls_corners

    def get_marker_cntr(self, sqr):
        front_left_corner = sqr[0]
        behind_right_corner = sqr[2]
        cntr = self.get_line_cntr(front_left_corner, behind_right_corner)
        return cntr

    def get_line_cntr(self, pt1, pt2):
        line_cntr = tuple(map(lambda x: int(x), ((pt1[0] + pt2[0] ) / 2, (pt1[1] + pt2[1]) / 2)))
        return line_cntr

    def get_marker_direction(self, sqr):
        front_left_corner = sqr[0]
        front_right_corner = sqr[1]
        direction_point = self.get_line_cntr(front_left_corner, front_right_corner)
        return direction_point

    def get_angle_error(self, cntr, direction, destination_pt):
        dir_vec = (direction[0] - cntr[0]), (direction[1] - cntr[1])
        trajectory_vec = (destination_pt[0] - cntr[0]), (destination_pt[1] - cntr[1])
        scalar_multiply = dir_vec[0] * trajectory_vec[0] + dir_vec[1] * trajectory_vec[1]
        dir_vec_module = sqrt(dir_vec[0] ** 2 + dir_vec[1] ** 2)
        trajectory_vec_module = sqrt(trajectory_vec[0] ** 2 + trajectory_vec[1] ** 2)
        if (trajectory_vec_module * dir_vec_module) != 0:
            cos_a = scalar_multiply / (trajectory_vec_module * dir_vec_module)
            angle = round(degrees(acos(min(1, max(cos_a, -1)))))
        else:
            angle = 0

        angle = self.get_angle_sign(cntr, direction, destination_pt, angle)

        return angle

    def get_angle_sign(self, cntr, direction, dist_pt, angle):
        """ This func needed for computing angle sign
         If we need to turn our robot clockwise, it well be < + >.
         Else: < - >. """
        projection = self.get_projection_of_direction_pt_on_trajectory(direction, cntr, dist_pt)
        if cntr[0] <= dist_pt[0]:
            if direction[1] >= projection[1]:
                result_angle = -angle
            else:
                result_angle = angle
        else:
            if direction[1] >= projection[1]:
                result_angle = angle
            else:
                result_angle = -angle
        return result_angle

    def get_projection_of_direction_pt_on_trajectory(self, center, direction, dist_pt):
        """ This func helps  to find angle sign."""
        x_cnt, y_cnt = center[0], center[1]
        x_dir, y_dir = direction[0], direction[1]
        x_dist, y_dist = dist_pt[0], dist_pt[1]
        x_projection = x_dir
        if (x_dist - x_cnt) != 0:
            y_projection = (x_projection - x_cnt) * (y_dist - y_cnt) / (x_dist - x_cnt) + y_cnt
        else:
            y_projection = (x_projection - x_cnt) * (y_dist - y_cnt) / 1 + y_cnt
        return tuple(int(x_projection), int(y_projection))
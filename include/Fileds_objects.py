from numpy import array
from math import degrees, sqrt, acos
from matplotlib.path import Path
from include.Constants import IMAGE_SIZE, HIGH_BOUNDS, LOW_BOUNDS


class markerAnalizer:
    def __init__(self):
        self.__robots = {}
        self.__goals = {}
        self.__obstacles = {}

    def get_robots(self):
        return self.__robots

    def get_goals(self):
        return self.__goals

    def get_obstacles(self):
        return self.__obstacles

    def parse_fields_objects_by_id(self, objects_dict):
        tmp_obstacles_dict = {}
        tmp_goals_dict = {}
        for key in objects_dict.keys():
            if len(str(key)) == 1:
                self.__robots[key] = Robot(objects_dict[key])
            elif len(str(key)) == 3:
                tmp_goals_dict[key] = Goal(objects_dict[key])
            else:
                if key//10 not in tmp_obstacles_dict:
                    tmp_obstacles_dict[key//10] = [Marker(objects_dict[key])]
                else:
                    tmp_obstacles_dict[key // 10].append(Marker(objects_dict[key]))
        for key in tmp_obstacles_dict.keys():
            self.__obstacles[key] = Obstacle(tmp_obstacles_dict[key])

        self.set_goals_id_from_platform_id(tmp_goals_dict)

    def set_goals_id_from_platform_id(self, goals_dict):
        for (platform_id, goal_id) in list(zip(self.__robots.keys(), goals_dict.keys())):
            self.__goals[platform_id] = goals_dict[goal_id]


class Point:
    def __init__(self, xy=(None, None)):
        self.x = xy[0]
        self.y = xy[1]

    def set_x(self, x):
        self.x = x

    def set_y(self, y):
        self.y = y

    def set_xy(self, x, y):
        self.x = x
        self.y = y

    def get_x(self):
        return self.x

    def get_y(self):
        return self.y

    def get_xy(self):
        return self.x, self.y

    def remap_to_ompl_coord_system(self):
        ompl_x = self.x * (HIGH_BOUNDS - LOW_BOUNDS) / IMAGE_SIZE + LOW_BOUNDS
        ompl_y = self.y * (HIGH_BOUNDS - LOW_BOUNDS) / IMAGE_SIZE + LOW_BOUNDS
        return ompl_x, ompl_y

    def remap_to_img_coord_system(self):
        img_x = int((self.x + HIGH_BOUNDS) * IMAGE_SIZE / (HIGH_BOUNDS - LOW_BOUNDS))
        img_y = int((self.y + HIGH_BOUNDS) * IMAGE_SIZE / (HIGH_BOUNDS - LOW_BOUNDS))
        return img_x, img_y


class Marker:
    def __init__(self, corners):
        self.__corners = list(Point(xy) for xy in corners.tolist())

    def get_corners(self):
        return self.__corners

    def remap_marker_to_ompl_coord_system(self):
        ompl_corners = list(xy.remap_to_ompl_coord_system() for xy in self.__corners)
        return ompl_corners

    def get_center(self):
        front_left_corner = self.__corners[0]
        behind_right_corner = self.__corners[2]
        center = self.get_line_cntr(front_left_corner, behind_right_corner)
        return center

    def get_line_cntr(self, pt1, pt2):
        line_cntr = tuple(map(lambda x: x, ((pt1.x + pt2.x) / 2, (pt1.y + pt2.y) / 2)))
        return line_cntr

    def points_to_list(self, points_list):
        return [(pt.x, pt.y) for pt in points_list]

    def get_ompl_path(self):
        ompl_corners = self.remap_marker_to_ompl_coord_system()
        points = self.points_to_list(ompl_corners)
        return Path(array(points))


class Goal(Marker):
    pass


class Robot(Marker):
    def __init__(self, corners):
        Marker.__init__(corners)
        self.direction = self.get_direction()

    def get_direction(self):
        front_left_corner = self.__corners[0]
        front_right_corner = self.__corners[1]
        direction_point = self.get_line_cntr(front_left_corner, front_right_corner)
        return direction_point

    def get_angle_to_point(self, destination_point):
        dir_vec = Point(((self.direction.x - self.center.x), (self.direction.y - self.center.y)))
        trajectory_vec = (destination_point.x - self.center.x), (destination_point.y - self.center.y)
        scalar_multiply = dir_vec.x * trajectory_vec.x + dir_vec.y * trajectory_vec.y
        dir_vec_module = sqrt(dir_vec.x ** 2 + dir_vec.y ** 2)
        trajectory_vec_module = sqrt(trajectory_vec.x ** 2 + trajectory_vec.y ** 2)
        if (trajectory_vec_module * dir_vec_module) != 0:
            cos_a = scalar_multiply / (trajectory_vec_module * dir_vec_module)
            angle = round(degrees(acos(min(1, max(cos_a, -1)))))
        else:
            angle = 0
        angle = self.get_angle_sign(destination_point, angle)
        return angle

    def get_angle_sign(self, destination_point, angle):
        """ This func needed for computing angle sign
         If we need to turn our robot clockwise, it well be < + >.
         Else: < - >. """
        projection = self.get_projection_of_direction_pt_on_trajectory(destination_point)
        if self.center.x <= destination_point.x:
            if self.direction.y >= projection.y:
                result_angle = -angle
            else:
                result_angle = angle
        else:
            if self.direction.y >= projection.y:
                result_angle = angle
            else:
                result_angle = -angle
        return result_angle

    def get_projection_of_direction_pt_on_trajectory(self, destination_point):
        """ This func helps  to find angle sign."""
        projection = Point()
        projection.x = self.direction.x
        if (destination_point.x - self.center.x) != 0:
            projection.y = (projection.x - self.center.x) * (destination_point.y - self.center.y) / \
                           (destination_point.x - self.center.x) + self.center.y
        else:
            projection.y = (projection.x - self.center.x) * (destination_point.y - self.center.y) / 1 + self.center.y
        return projection


class Obstacle:
    def __init__(self, marker_list):
        unsorted_points = self.get_obstacles_points(marker_list)
        self.obstacle_points = self.sort_obstacles_points(unsorted_points)

    def get_obstacles_points(self, markers_list, marker_border_points_num=2):

        obstacle_border_points = []

        geometric_center = self.get_geometric_center(markers_list)

        for marker in markers_list:
            distances_to_geometric_center = {}
            for pt in marker:
                distance = self.get_distance_between_pts(geometric_center, pt)
                while distance in distances_to_geometric_center:
                    distance += 0.001
                distances_to_geometric_center[distance] = pt
            for num in range(marker_border_points_num):
                obstacle_border_points.append(distances_to_geometric_center.pop(
                                              max(list(distances_to_geometric_center.keys()))))
                obstacle_border_points.append(distances_to_geometric_center.pop(
                    max(list(distances_to_geometric_center.keys()))))

        return obstacle_border_points

    def get_distance_between_pts(self, pt1, pt2):
        return sqrt((pt2.x - pt1.x) ** 2 + (pt2.y - pt1.y) ** 2)

    def get_geometric_center(self, markers_list):
        all_points = []
        for marker in markers_list:
            all_points += marker.get_corners()

        max_x = max([pt.x for pt in all_points])
        min_x = min([pt.x for pt in all_points])
        max_y = max([pt.y for pt in all_points])
        min_y = min([pt.y for pt in all_points])

        geometric_center = Point((max_x - min_x, max_y - min_y))
        return geometric_center

    def sort_obstacles_points(self, corners):
        init_corner = corners.pop(0)
        sorted_corners = [init_corner]
        while len(corners) != 1:
            distances = []
            for corner in corners:
                distances.append(self.get_distance_between_pts(init_corner, corner))
            init_corner = corners.pop(distances.index(min(distances)))
            sorted_corners.append(init_corner)
        sorted_corners.append(corners[0])
        return sorted_corners

    def points_to_list(self, points_list):
        return [(pt.x, pt.y) for pt in points_list]

    def remap_points_to_ompl_coord_system(self):
        ompl_corners = list(xy.remap_to_ompl_coord_system() for xy in self.obstacle_points)
        return ompl_corners

    def get_ompl_path(self):
        ompl_points = self.remap_points_to_ompl_coord_system()
        points = self.points_to_list(ompl_points)
        return Path(array(points))

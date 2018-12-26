from include.Fileds_objects import Robot, Goal, Obstacle, Marker


class MarkerAnalizer:
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

        if len(self.__robots.keys()):
            self.set_goals_id_from_platform_id(tmp_goals_dict)
        else:
            self.__goals = tmp_goals_dict

    def set_goals_id_from_platform_id(self, goals_dict):
        for (platform_id, goal_id) in list(zip(self.__robots.keys(), goals_dict.keys())):
            self.__goals[platform_id] = goals_dict[goal_id]
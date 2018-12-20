import cv2
from cv2 import aruco
from include.markersAnalizer import markersAnalizer
from include.Paths_planner import Paths_planner


# Init aruco parametrs
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
parameters = aruco.DetectorParameters_create()
analizer = markersAnalizer()
planner = Paths_planner()

# Init cv window
cv2.namedWindow("Vrep thres", cv2.WINDOW_NORMAL)

# Init dictionaries
platforms_dict = {}
walls_dict = {}
goals_dict = {}

RUN = True

if __name__ == "__main__":
    while RUN:

        result_image = cv2.imread("test1.png")
        result_image = cv2.resize(result_image, (480, 480))

        gray = cv2.cvtColor(result_image, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_OTSU)

        corners, ids, _ = aruco.detectMarkers(thresh, aruco_dict, parameters=parameters)

        markers_dict = {}
        for x in range(len(ids)):
            id = ids[x][0]
            markers_dict[id] = corners[x][0]

        ompl_markers_dict = analizer.convert_to_ompl_coordinate_system(markers_dict)

        analizer.parse_ids(ompl_markers_dict)

        aruco.drawDetectedMarkers(result_image, corners)

        obstacles = analizer.get_obstacles()
        robots = analizer.get_robots()
        goals = analizer.get_goals()
        goals_bbox = analizer.get_goals_bbox()

        for id in goals_bbox.keys():
            cv2.putText(result_image, str(id), (analizer.remap_to_cv(goals_bbox[id][0][0]),
                                                analizer.remap_to_cv(goals_bbox[id][0][1])-20), 1, 1, (0, 0, 0), 2)
            for pt in goals_bbox[id]:
                cv2.circle(result_image, (analizer.remap_to_cv(pt[0]), analizer.remap_to_cv(pt[1])), 2, (150, 40, 255), 2)

        for id in robots.keys():
            cv2.putText(result_image, str(id), (analizer.remap_to_cv(robots[id][0][0]),
                                                analizer.remap_to_cv(robots[id][0][1])-40), 1, 1, (0, 0, 255), 2)

        planner.set_robots(robots)
        planner.set_obstacles(obstacles)
        planner.set_targets(goals)
        planner.set_targets_corners(goals_bbox)

        paths, final_time = planner.multiple_paths_planning()

        for path_id in paths.keys():
            states = paths[path_id].getStates()
            for state in states:
                x = state.getX()
                y = state.getY()
                cv_x = analizer.remap_to_cv(x)
                cv_y = analizer.remap_to_cv(y)
                cv2.circle(result_image, (cv_x, cv_y), 1, (255, 0, 0), 1)

        cv2.imshow("Vrep thres", cv2.resize(result_image, (960, 960)))

        if cv2.waitKey() & 0xFF == 27:
            RUN = False

    cv2.destroyAllWindows()

from include import vrep
import cv2
from cv2 import aruco
from include.markersAnalizer1 import markersAnalizer
from include.Paths_planner import Paths_planner
from include.visualization_tools import *


# Init aruco parametrs
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
parameters = aruco.DetectorParameters_create()
analizer = markersAnalizer()
planner = Paths_planner()

# Init cv window
cv2.namedWindow("Vrep thres", cv2.WINDOW_NORMAL)

# Init v-rep streams
all_vision_sectors = []
for sector in range(4):
    vision_sector = []
    sector_name = "Vision_sensor" + str(sector)
    for sub_sector in range(4):
        sub_sector_name = sector_name + str(sub_sector)
        vision_sector.append(vrep.vrep("127.0.0.1", sensor=sub_sector_name))
    all_vision_sectors.append(vision_sector)

# Init dictionaries
platforms_dict = {}
walls_dict = {}
goals_dict = {}

RUN = True

if __name__ == "__main__":
    # while RUN:
    images_in_sector = []
    for sector in all_vision_sectors:
        images_in_sub_sector = []
        for vision_sensor in sector:
            image_form_vision_sensor = vision_sensor.getVrepImage()
            images_in_sub_sector.append(image_form_vision_sensor)
        images_in_sector.append(images_concatenate_4x4(images_in_sub_sector))
    result_image = images_concatenate_4x4(images_in_sector)
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
    print("goals_bbox", goals_bbox)
    planner.set_robots(robots)
    planner.set_obstacles(obstacles)
    planner.set_targets(goals)
    planner.set_targets_corners(goals_bbox)

    paths, final_time = planner.multiple_paths_planning()
    for path_id in paths.keys():
        ret = paths[path_id][0]
        states = paths[path_id][1].getStates()
        for state in states:
            x = state.getX()
            y = state.getY()
            cv_x = analizer.remap_to_cv(x)
            cv_y = analizer.remap_to_cv(y)
            cv2.circle(result_image, (cv_x, cv_y), 5, (255, 0, 255), 2)

    for k in obstacles.keys():
        borders = []
        for c in obstacles[k]:
            cv_c = (analizer.remap_to_cv(c[0]), analizer.remap_to_cv(c[1]))
            draw_pt(result_image, cv_c)
            borders.append(cv_c)
        draw_walls(result_image, borders)

    #
    # for k in robots.keys():
    #     cntr = robots[k][0]
    #     dir = robots[k][1]
    #     draw_pt(result_image, cntr)
    #     draw_pt(result_image, dir)

    cv2.imshow("Vrep thres", result_image)

    if cv2.waitKey() & 0xFF == 27:
        RUN = False

    cv2.destroyAllWindows()

import cv2
from cv2 import aruco
from include.Constants import IMAGE_SIZE, CAMERA_INDEX
from include.Planner import Paths_planner
from include.Fileds_objects import Robot, Goal, Obstacle, Point
from include.MarkersAnalizer import MarkerAnalizer


# Init aruco parametrs
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
parameters = aruco.DetectorParameters_create()

# Init cv2 stream and window parameters
stream = cv2.VideoCapture(CAMERA_INDEX)
stream.set(cv2.CAP_PROP_FRAME_WIDTH, IMAGE_SIZE)
stream.set(cv2.CAP_PROP_FRAME_HEIGHT, IMAGE_SIZE)
stream.set(cv2.CAP_PROP_FPS, 30)
cv2.namedWindow("Capture", cv2.WINDOW_NORMAL)
cv2.moveWindow("Capture", 50, 50)
RUN_CAPTURE = True

# Init marker analizer
analizer = MarkerAnalizer()

# Init path planner
planner = Paths_planner()


def resize_image_to_square_size(image):
    w, h, _ = image.shape
    if h != w:
        h_to_cut = (h - w)//2
        sqr_image = image[0:w, h_to_cut:h - h_to_cut]
        return sqr_image
    else:
        return image


while RUN_CAPTURE:
    ret, img = stream.read()
    if ret:
        img = resize_image_to_square_size(img)
        gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners_np, ids_np, _ = aruco.detectMarkers(gray_img, aruco_dict, parameters=parameters)

        if type(ids_np) != type(None):
            ids = list(map(lambda x: x[0], ids_np))
            corners = list(map(lambda x: x[0], corners_np))
            markers_dict = dict(zip(ids, corners))

            analizer.parse_fields_objects_by_id(markers_dict)

            robots = analizer.get_robots()
            goals = analizer.get_goals()
            obstacles = analizer.get_obstacles()

            planner.set_robots(robots)
            planner.set_targets(goals)
            planner.set_obstacles(obstacles)

            paths, time = planner.multiple_paths_planning()
            if len(paths):
                for path_id in paths.keys():
                    ret = paths[path_id]
                    states = paths[path_id].getStates()
                    for state in states:
                        x = state.getX()
                        y = state.getY()
                        cv_point = Point((x, y)).remap_to_img_coord_system()
                        cv2.circle(img, (int(cv_point.x), int(cv_point.y)), 5, (255, 0, 255), 2)
                for obs in obstacles.values():
                    for pt in obs.get_obstacle_points():
                        cv2.circle(img, (int(pt.x), int(pt.y)), 3, (255, 0, 0), 5)

                    c = obs.get_geometric_center()
                    cv2.circle(img, (int(c.x), int(c.y)), 5, (255, 255, 0), 8)

        cv2.imshow("Capture", img)

        if cv2.waitKey(10) & 0xFF == 27:
            RUN_CAPTURE = not RUN_CAPTURE
    else:
        print("Can't connect to camera with index << {} >>".format(CAMERA_INDEX))
        RUN_CAPTURE = not RUN_CAPTURE

stream.release()
cv2.destroyAllWindows()
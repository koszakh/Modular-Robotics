import cv2
from cv2 import aruco
from include.Constants import IMAGE_SIZE, CAMERA_INDEX
from include.markersAnalizer import markersAnalizer
from include.Paths_planner import Paths_planner


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
analizer = markersAnalizer()

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
    img = resize_image_to_square_size(img)
    # ret = True
    # img = cv2.imread("test2.png")
    # img = cv2.resize(img, (IMAGE_SIZE, IMAGE_SIZE))
    print(img.shape)
    if ret:
        gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, threshed_img = cv2.threshold(gray_img, 100, 255, cv2.THRESH_BINARY)

        thr_color = cv2.cvtColor(threshed_img, cv2.COLOR_GRAY2BGR)

        corners_np, ids_np, _ = aruco.detectMarkers(threshed_img, aruco_dict, parameters=parameters)

        if type(ids_np) != type(None):
            ids = list(map(lambda x: x[0], ids_np))
            corners_cv = list(map(lambda x: x[0], corners_np))
            corners_ompl_np = list(map(analizer.remap_to_ompl, corners_cv))
            corners_ompl = list(map(lambda x: x.tolist(), corners_ompl_np))
            markers_dict = dict(zip(ids, corners_ompl))

            analizer.parse_ids(markers_dict)

            robots = analizer.get_robots()
            targets = analizer.get_goals()
            obstacles = analizer.get_obstacles()

            if len(robots) and len(robots) == len(targets):

                planner.set_robots(analizer.get_robots())
                planner.set_obstacles(analizer.get_obstacles())
                planner.set_targets(analizer.get_goals())

                paths, final_time = planner.multiple_paths_planning()

                for path_id in paths.keys():
                    if type(paths[path_id]) != type(None):
                        states = paths[path_id].getStates()
                        for state in states:
                            x = state.getX()
                            y = state.getY()
                            cv_x = analizer.remap_to_cv(x)
                            cv_y = analizer.remap_to_cv(y)
                            cv2.circle(thr_color, (cv_x, cv_y), 1, (255, 0, 0), 7)

            aruco.drawDetectedMarkers(thr_color, corners_np)

        cv2.imshow("Capture", thr_color)

        if cv2.waitKey(10) & 0xFF == 27:
            RUN_CAPTURE = not RUN_CAPTURE
    else:
        print("Can't connect to camera with index <{}>".format(CAMERA_INDEX))
        RUN_CAPTURE = not RUN_CAPTURE

stream.release()
cv2.destroyAllWindows()






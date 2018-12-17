from cv2 import line, circle
import numpy as np

def draw_line(img, pt1, pt2):
    x1 = int(pt1[0])
    y1 = int(pt1[1])
    x2 = int(pt2[0])
    y2 = int(pt2[1])
    line(img, (x1, y1), (x2, y2), (30, 255, 15), 1)


def draw_pt(img, pt):
    x = int(pt[0])
    y = int(pt[1])
    circle(img, (x, y), 7, (255, 0, 50), 5)


def draw_walls(img, corners):
    x_y = list((int(pt[0]), int(pt[1])) for pt in corners)
    for idx in range(len(x_y)):
        x1, y1 = x_y[idx-1][0], x_y[idx-1][1]
        x2, y2 = x_y[idx][0], x_y[idx][1]
        x3, y3 = x_y[idx-2][0], x_y[idx-2][1]
        line(img, (x1, y1), (x2, y2), (255, 50, 180), 2, lineType=8)
        line(img, (x1, y1), (x3, y3), (255, 50, 180), 2, lineType=8)


def images_concatenate_4x4(sector):
    top = np.concatenate((sector[0], sector[1]), axis=1)
    bot = np.concatenate((sector[2], sector[3]), axis=1)
    result_img = np.concatenate((top, bot), axis=0)
    return result_img



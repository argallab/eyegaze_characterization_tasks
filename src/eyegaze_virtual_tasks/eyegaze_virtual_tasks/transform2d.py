from enum import Enum
import math
from scipy.interpolate import interp1d
import numpy as np


class ScreenResolution(Enum):
    FOURK = (3840, 2160)
    QHD = (2560, 1440)
    DELLXPS13 = (1920, 1200)
    STANDARD = (1920, 1080)
    STUDY = (1600, 900)


class SceneToScreen2D():
    def __init__(self, screen_res, screen_coords, logger):
        self.monitor_x = None
        self.monitor_y = None
        self.logger = logger
        self.__screen_res = screen_res  # this should be a tuple
        coord1, coord2 = self.__pick_calibration_edge(screen_coords)
        self.se2 = self.__compute_se2_matrix(coord1, coord2)

    def __pick_calibration_edge(self, screen_coords):
        coord1 = None
        coord2 = None
        height_inscene = None
        width_inscene = None

        ## this is the bottom left corner of the screen
        self.monitor_x = screen_coords[0, 0]
        self.monitor_y = screen_coords[0, 1]

        """
        coord 3 ------------------------
        |
        |
        |
        | Screen coord definitions
        |
        |
        coord 1 ----------------- coord 2
        """
        coord1 = screen_coords[0, :]
        coord2 = screen_coords[3, :]
        coord3 = screen_coords[1, :]
        width_inscene = np.linalg.norm(coord2-coord1)
        height_inscene = np.linalg.norm(coord3-coord1)

        self.y_mapper = interp1d([0, height_inscene], [0, self.__screen_res[1]], bounds_error=False, fill_value="extrapolate")
        self.x_mapper = interp1d([0, width_inscene], [0, self.__screen_res[0]], bounds_error=False, fill_value="extrapolate")

        return coord1, coord2

    def __compute_se2_matrix(self, coord1, coord2):
        se2 = np.zeros((3, 3))
        angle = 0.0
        x1 = coord1[0]
        y1 = coord1[1]
        x2 = coord2[0]
        y2 = coord2[1]
        x = self.monitor_x
        y = self.monitor_y

        # if edges[1] or edges[2]:
        #     angle = math.atan2(abs(x2-x1), abs(y2-y1))
        # elif edges[0] or edges[3]:
        angle = math.atan2(abs(y2-y1), abs(x2-x1))
    
        cos_val = math.cos(angle)
        sin_val = math.sin(angle)

        R_T = np.array([[cos_val, sin_val], 
                        [-sin_val, cos_val]])
        p = np.array([[x], [y]])
        col = - (R_T @ p).flatten()

        se2 = np.array([[cos_val, sin_val,  col[0]], 
                        [-sin_val, cos_val, col[1]], 
                        [0.0, 0.0, 1.0]])

        return se2

    def compute(self, point, onscreen_flag):
        """
        Apply SE2 transform, and scale value
        Only run this function if the gaze is within the screen
        """
        # self.logger.info("se2 transform: ")
        # self.logger.info("{}".format(self.se2))
        # self.logger.info("point in scene: {}".format(point))
        point_ = np.array([[point[0]], [point[1]], [1.0]])
        transform_pt = (self.se2 @ point_).flatten()

        scaled_x = self.x_mapper(transform_pt[0])
        scaled_y = self.y_mapper(transform_pt[1])
        # self.logger.info("point on screen: {}, {}".format(scaled_x, scaled_y))

        # return float(scaled_x), float(scaled_y)

        if onscreen_flag:
            return float(scaled_x), float(scaled_y)
        else:
            # self.logger.info("gaze2d SCENE coords: {}".format(point))
            # self.logger.info("gaze2d screen coords: {}, {}".format(scaled_x, scaled_y))
            return self.__get_closest_screen_point(scaled_x, scaled_y)


    def __get_closest_screen_point(self, x, y):
        """
        Find the closest point on the screen. 
        Do this using a whole bunch of cases. 
        """
        if x <= 0.0:
            if y <= 0.0:
                # self.logger.info("{}, {} in REGION 1".format(x, y))
                return float(0.0), float(0.0)
            elif y < self.__screen_res[1]:
                # self.logger.info("{}, {} in REGION 2".format(x, y))
                return float(0.0), float(y)
            else: 
                # self.logger.info("{}, {} in REGION 3".format(x, y))
                return float(0.0), float(self.__screen_res[1])
        elif x < self.__screen_res[0]:
            if y <= 0.0:
                # self.logger.info("{}, {} in REGION 4".format(x, y))
                return float(x), float(0.0)
            else:
                # self.logger.info("{}, {} in REGION 5".format(x, y))
                return float(x), float(self.__screen_res[1])
        else:
            if y <= 0.0:
                # self.logger.info("{}, {} in REGION 6".format(x, y))
                return float(self.__screen_res[0]), float(0.0)
            elif y < self.__screen_res[1]:
                # self.logger.info("{}, {} in REGION 7".format(x, y))
                return float(self.__screen_res[0]), float(y)
            else: 
                # self.logger.info("{}, {} in REGION 8".format(x, y))
                return float(self.__screen_res[0]), float(self.__screen_res[1])